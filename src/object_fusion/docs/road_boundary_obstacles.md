# How the perception outputs handle obstacles near the road edge

Every change made to keep obstacles from corrupting the left/right road boundaries, in one place,
with where each one lives, its parameter and how to turn it off. The measurements behind them are
in [`road_boundary.md`](road_boundary.md) and
[`road_boundary_upstream_plan.md`](road_boundary_upstream_plan.md); open decisions are in
[`road_boundary_status.md`](road_boundary_status.md).

## The failure

Both producers published the **outermost extent of whatever they labelled road**, so an obstacle
near the edge moved the boundary:

- **SphereFormer:** a car is labelled car, and its LiDAR shadow gets no label at all. The road's
  extent stops at the car's inner side, and the boundary jumps inward.
- **SAM3 contours:** the node kept the LARGEST mask contour and split each row band at its MEAN x.
  A car on the road notches the mask, so the contour runs around the car and the car's outline
  becomes boundary. A car that cuts the near road off from the far road leaves the far road as a
  smaller contour, which was thrown away.
- **SAM3 lifting to 3D:** contour pixels were matched to LiDAR returns within a pixel radius,
  which near an edge also catches returns behind it.
- **The ego vehicle:** SphereFormer labels the roof and hood returns (0.8–0.9 m from `lidar_tc`)
  "car", putting an obstacle on top of the sensor. Before this was excluded, 42% of samples read
  as occluded.

Two layers fix this. They are independent and can be used together.

## Layer 1, downstream: `road_boundary_node`

On the vehicle branch `radar_integration_and_fusion` (commit `695a874`), **off by default**;
the tracker fix below is only on `upstream_boundary_fixes` (`caddd21`). Code:
`object_fusion/road_boundary.py` (rclpy-free, unit-tested), with `road_boundary_node.py` as the
ROS wrapper.

It re-derives both boundaries from the producers' own outputs and publishes drop-in twins:
`/road_boundary/{sphereformer,sam3}/{left,right}`, with the same type (`PointCloud2` xyz), frame
(`lidar_tc`) and stamps as the raw topics. The raw topics keep flowing. A consumer switches with a
topic remap, and dropping the remap is the rollback. Debug markers go to `/road_boundary/markers`:
grey spheres for occluded samples, orange for unsupported ones.

### Where the obstacles come from

| source | detail | parameter |
|---|---|---|
| SphereFormer sweep | points of classes 1–8 (vehicles, persons) | — |
| ego exclusion | obstacle points within 3 m of the LiDAR are the vehicle itself | `ego_radius` (3.0) |
| noise floor | a bearing is shadowed only by its 3rd-nearest obstacle return, so one misclassified point cannot shadow it | `BoundaryParams.min_obstacle_points` (3; code constant, not a ROS parameter) |
| fused objects | `/perception/objects` boxes, filled in as footprint points (`box_footprint_points`), carried to the frame's stamp with ego motion | `use_objects` (true), `objects_max_age` (0.5 s), `obstacle_max_skew` (0.15 s) |
| SAM3 twin | uses the SphereFormer sweep nearest in stamp, plus the fused boxes | — |

### The four steps (each switchable at runtime)

The `enable_*` switches and the tunables named here (except the two marked as code constants)
can be changed with `ros2 param set /road_boundary <name> <value>`.

1. **Occlusion** (`enable_occlusion`, on). `ShadowMap` holds, per 0.5° bearing, the nearest
   obstacle; anything at or beyond it could not have been seen. `bin_extremes` classifies each
   per-bin road extreme:
   - **OCCLUDED:** an obstacle, or an obstacle's shadow, within `occl_margin` (1 m) beyond it;
   - **UNSUPPORTED:** no non-road return within `edge_evidence_m` (1 m) beyond it, which is a
     bin that only caught an arc of a LiDAR ring;
   - **EDGE:** otherwise. Only EDGE samples are fitted; the others are removed, never moved.
2. **Extraction** (`enable_extraction`, **off**). `select_run` keeps, per bin, the lateral road
   run that continues the previous bin's run, with obstacles and their shadows bridging gaps, so
   a car splitting the road does not disconnect the far road. Off because on wide paved lots it
   locked onto arbitrary strips (3–6 m jumps).
3. **Fit** (`enable_fit`, on). `fit_side` fits a Huber polynomial per side through the EDGE
   samples, bridging the removed ones, and extrapolates at most `max_extrap` (10 m). A side with
   less than `min_support_frac` (30%) support is mirrored from the other side at the road width
   (`default_width` 7 m only when nothing better is known). **Seen road is a floor:** the edge is
   never placed inside an OCCLUDED or UNSUPPORTED extreme. It is pushed out to it by at most
   `max_floor_push` (1.5 m), so one stray road return cannot drag it outward.
4. **Tracking** (`enable_tracking`, on). `BoundaryTracker` keeps a scalar Kalman filter per
   station in the ego frame, carried forward by the odometry twist. A stretch hidden by an
   obstacle coasts on what was seen for up to `coast_s` (3 s), then publishes empty.
   **Fix on the branch only:** stations whose left edge lies right of the right edge are
   dropped, and a station gated 3 frames running re-initialises (`BoundaryParams.reinit_after_gated`, a
   code constant). Without
   it, a bad initialisation through an occluded stretch coasted into a ~4 m, ~1 s swing of the
   planner's edge.

## Layer 2, upstream: inside SphereFormer and SAM3

On branch `upstream_boundary_fixes` (worktree `~/Downloads/ADPS-upstream`); SAM3 changes on the
submodule branch of the same name. Each one is a parameter; the "off" value restores the old
behaviour.

### SphereFormer (`sphereformer_ros.py`, `sphereformer_boundary.py`)

| change | what it does for obstacles | parameter, off value |
|---|---|---|
| S2 ego crop (`crop_ego`) | points within 2.5 m are removed **before** inference, so the roof is never labelled car | `ego_crop_radius`, `0.0` |
| S3 fixed vehicle-frame bins with edge evidence (`fixed_bin_boundary`) | 2 m bins, 5–60 m along the vehicle axis. An extreme is an edge only with non-road, non-obstacle returns (class ≥ 10) within 1 m beyond it, so **road that ends at a car is dropped rather than published as an edge**. This also removes ring-arc ends (−0.3 m reported vs −2.9 m true at 40 m). | `boundary_mode`, `legacy`; `require_edge_evidence`, `false` |
| S1 `label` field | uint8 class per point, so consumers read obstacle classes directly instead of decoding colours | (extra field only) |
| S7 3-sweep accumulation (`SweepAccumulator`) | fills the gaps between rings with the last 3 sweeps, moved by odometry. Only road and non-road ground are accumulated, so moving traffic does not smear in. Warns after 20 sweeps without odometry. | `accumulate_sweeps`, `1` |
| S4 ego-corridor runs | the upstream twin of extraction; **off**, it misfired on wide lots | `connected_runs` |

### SAM3 segmentation node (`segmentation.py`, `road_mask_boundary.py`)

| change | what it does for obstacles | parameter, off value |
|---|---|---|
| A4 obstacle prompt | `car or truck or person` runs on the **same** image encoding as the road prompt (batched with it) and is published as a PNG mask on `/segmentation/obstacle_mask/compressed` | `obstacle_prompt`, `""` |
| A3 ego component (`ego_component`) | fills holes, so road continues under a car; keeps the component the vehicle is on. Obstacle pixels **bridge** components for the labelling only, so road beyond a car that cuts the mask stays connected; obstacle pixels are never returned as road. | `boundary_mode`, `legacy` |
| A3 row-wise extents (`rowwise_boundary`) | each row's outermost road pixels, without horizon rows or image-border pixels, and **without pixels within `obstacle_dilate_px` (6) of an obstacle** | `boundary_mode`, `legacy` |
| A1 lossless masks | road and obstacle masks are PNG, so mask ids and edges arrive exactly | (format only) |

### SAM3 mask transform (`sam3_mask_transform.py`, `lift_mask_boundary`)

| change | what it does for obstacles | parameter, off value |
|---|---|---|
| A5 whole-mask lifting | every projected LiDAR return whose pixel is road becomes a road point, instead of matching contour pixels | `lift_mode`, `contour` |
| ground-only points | only returns within 0.3 m of the road surface are lifted, so road label bleeding onto a car's lower body is not lifted | (part of A5) |
| ground-only edge evidence | an extreme needs non-road **ground** within 1 m beyond it; before, a pedestrian on the road "confirmed" an edge at their feet. Extremes at the field-of-view border are dropped. | (part of A5) |
| obstacle drop | an extreme whose pixel touches the dilated obstacle mask (paired with the road mask by capture stamp) is dropped | `obstacle_prompt` `""` disables the mask |

## What it achieved

Sep 22 stationary road bags, through the planner's unmodified `lmp_v10c`, at 20 m, in obstacle
frames (in-lane vehicles count for both sides). Script: `scripts/road_boundary_planner_ab.py`.

| setup | edge pulled in > 0.5 m / > 1 m |
|---|---|
| vehicle code today, raw | 27–31% / 13% |
| today + `road_boundary` | 0–2.4% / 0–0.2% |
| branch upstream fixes, raw | 0.4–3.9% / 0–2.5% |
| branch + `road_boundary` | 0–1.0% / 0% |
| branch + `road_boundary` + `lmp_v10c` guards | **0% / 0%** |

## Limits and open points

- **Not shown while moving.** Both road bags are stationary. On Route2 only 51–87 frames had a
  fused vehicle near an edge, too few to judge. Record a drive in traffic with
  `scripts/record_rosbag.sh boundary_eval`, run from the worktree.
- **The obstacle prompt costs SAM3 rate.** Before batching it took Route2 from 2.3 to 1.4 Hz;
  batching recovered most of it (4 terms: ~4.5 Hz under the full stack). `"car"` or `""` is
  cheaper; decision open.
- **The tracker fix is not on the vehicle branch yet.** A `road_boundary` test from the vehicle
  checkout runs without it.
- **The planner guards matter with a faster SAM3:** without `enable_coherent_source_mode` and
  `enable_pair_dropout_hysteresis`, `lmp_v10c` flickers between SAM3-fused and SphereFormer-only
  pairs.
- **Not built:** per-point edge/occluded/unseen state in the published boundary plus a typed
  message the planner can weight (S5 + X1), and retraining SphereFormer to label road under
  vehicles (S8).

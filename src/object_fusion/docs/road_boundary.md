# road_boundary: obstacle-robust road boundaries

## What it does
SphereFormer and SAM3 publish the outermost extent of whatever they labelled road. An obstacle
near the edge hides the road behind it, so that extent jumps inward. `road_boundary_node`
re-derives both boundaries from the producers' own outputs and publishes **drop-in twins**:

| raw (unchanged) | twin |
|---|---|
| `/sphereformer_{left,right}_boundary` | `/road_boundary/sphereformer/{left,right}` |
| `/sam3_{left,right}_boundary` | `/road_boundary/sam3/{left,right}` |

The twins have the same type (`PointCloud2` xyz), frame (`lidar_tc`) and stamp. There are also
debug markers on `/road_boundary/markers`: green/blue lines for the boundaries, grey spheres for
samples rejected as occluded, and orange spheres for samples rejected as unsupported.

No upstream node is edited. The SAM3 twin lifts the **whole** road mask
(`/segmentation/mask_image/compressed`) onto the ground-flagged projection, rather than lifting
contour pixels. The rule lives in `object_fusion/road_boundary.py` (rclpy-free, unit-tested);
the node is a thin wrapper around `BoundaryPipeline`.

### Steps

| step | parameter | default | what it does |
|---|---|---|---|
| occlusion | `enable_occlusion` | on | Classifies each per-bin road extreme. An obstacle (≥3 returns) or its shadow within 1 m beyond it makes it OCCLUDED. No non-road return seen within 1 m beyond it makes it UNSUPPORTED (the bin only caught a ring arc). Only EDGE samples are fitted. |
| extraction | `enable_extraction` | **off** | Per bin, keeps the lateral road run that continues the previous bin's run. Neutral on the road replays, but on wide paved lots it locked onto arbitrary runs (3–6 m jumps). Experimental. |
| fit | `enable_fit` | on | Huber polynomial per side; the degree grows with the supported span. Extrapolation is limited to 10 m. A side with <30% support is mirrored from the other at the road width. Seen road acts as a floor: the edge is never placed inside an occluded or unsupported extreme. |
| tracking | `enable_tracking` | on | Per-station scalar KF in the ego frame, carried forward by the odometry twist. Occluded stretches coast for up to 3 s and are then published empty. The KF re-initialises when most stations fail the gate. |

All of these, plus the tunables (`x_min`, `x_max`, `bin_len`, `occl_margin`, `edge_evidence_m`,
`run_gap`, `max_extrap`, `min_support_frac`, `default_width`, `max_floor_push`, `coast_s`,
`gate_m`, `output_step`, `ego_radius`), can be set at runtime with `ros2 param set
/road_boundary ...`. A source can be switched off with `enable_sphereformer` / `enable_sam3`.

## Running it and rolling it back
- Off by default. Turn it on with `scripts/run_fusion.sh --road-boundary` or
  `ENABLE_ROAD_BOUNDARY=true`; per step, use `ROAD_BOUNDARY_{OCCLUSION,EXTRACTION,FIT,TRACKING}`.
  It needs SphereFormer and SAM3 up, so start `run_radar.sh` **without** `--obstacle-only`.
- `perception-object-fusion:latest` predates this node. Rebuild it before using `--road-boundary`
  on the vehicle:
  `docker compose -f docker-compose.yml -f docker-compose.fusion.yml --profile fusion build object_fusion_node`.
  A tested image of this code exists as `perception-object-fusion:roadb`; use it with
  `OBJECT_FUSION_IMAGE_TAG=roadb`.
- Switching a consumer is a remap; no planner code changes. For example:
  `python3 lmp_v10c.py --ros-args -r /sphereformer_left_boundary:=/road_boundary/sphereformer/left -r /sphereformer_right_boundary:=/road_boundary/sphereformer/right -r /sam3_left_boundary:=/road_boundary/sam3/left -r /sam3_right_boundary:=/road_boundary/sam3/right`.
  Dropping the remap is the rollback. The raw topics keep flowing either way.
- To remove it entirely, delete `road_boundary*.py`, this doc, `scripts/road_boundary_ab.py`
  and `test/test_road_boundary.py`, and revert the `road_boundary` lines in `config/topics.yaml`,
  `setup.py`, the launch file, `run_fusion.sh`, `docker-compose.fusion.yml` and
  `Dockerfile.object_fusion`.

## Measured at the planner (2026-10-07) -- the comparison that counts
The raw-topic numbers further down overstate the gain. On unmarked roads the planner reads the
boundaries through `lmp_v10c`, which already does robust fitting, SAM3/SphereFormer fusion, width
sanity checks and a motion-compensated temporal filter. It has no notion of obstacles.

The test ran three **unmodified** `lmp_v10c` instances on the same live replay, so every frame is
paired, with outputs remapped to `/ab/<arm>/road_{left,right}`:
- `raw`: today's inputs
- `rb`: road_boundary with its default steps
- `rbo`: road_boundary with occlusion only (no fit, no tracking)

All three had odometry remapped to `/novatel/oem7/odom`. Run `scripts/road_boundary_planner_ab.py`
on the recordings `~/fusion_data/recordings/planner_ab_2026-09-22_*`; the output is in
`planner_ab_results.txt`.

Both bags are stationary, so the true edge does not move. Each arm's reference is its own median
edge on obstacle-free frames; the arms' references agree to about 0.1 m at 20–40 m. Obstacle
frames are frames with a fused object 3–45 m ahead on that side, where in-lane vehicles
(-1 < y < 1) count for both sides. An earlier version excluded in-lane vehicles and so put the
worst raw events into the reference; the time-series plot exposed it.

| station | bag | pulled in > 0.5 m / > 1 m (raw → rb) | centreline shift p95 (raw → rb) | jitter (raw → rb) |
|---|---|---|---|---|
| 20 m | 14-18-56 | 30.9% / 12.6% → 2.4% / 0.2% | 0.63 → 0.19 m | 0.099 → 0.076 m |
| 20 m | 14-21-45 | 27.5% / 13.0% → 0.0% / 0.0% | 0.63 → 0.15 m | 0.104 → 0.099 m |
| 30 m | 14-18-56 | 10.1% / 1.6% → 5.9% / 1.0% | 0.42 → 0.27 m | 0.099 → 0.079 m |
| 30 m | 14-21-45 | 20.1% / 5.1% → 0.0% / 0.0% | 0.45 → 0.25 m | 0.098 → 0.085 m |
| 40 m | 14-18-56 | 11.9% / 3.9% → 3.6% / 0.8% | 0.56 → 0.39 m | 0.088 → 0.105 m |
| 40 m | 14-21-45 | 16.9% / 6.4% → 0.7% / 0.0% | 0.57 → 0.29 m | 0.120 → 0.110 m |

Jitter is the mean frame-to-frame |Δy|, both sides, all frames. At 10 m the planner's output is
empty in 92–95% of frames on raw input, because SphereFormer is used only beyond 15 m there; on rb
input it is empty in 18%.

Plots come from `scripts/road_boundary_planner_ab_plot.py`, output in
`~/fusion_data/recordings/planner_ab_plots/`:
- time series of the edges and centreline at 20 m, with obstacle frames shaded
- summary bars per station
- top-down snapshots of the worst raw frame

What this says:
- Through `lmp_v10c` on raw input, a passing or in-lane vehicle pulls the edge in by more than
  0.5 m in 28–31% of obstacle frames at 20 m (more than 1 m in 13%). The p95 centreline shift is
  0.63 m. On rb input this drops to 0–2.4% and 0.15–0.19 m.
- **Remaining jitter is a sensor disagreement, not this node.** Each rb source is steady on its
  own (mean |Δy| 0.02–0.06 m), but at 20 m SAM3 puts the left edge about 0.75 m inside
  SphereFormer (3.1 vs 3.87 m) and the right about 0.43 m inside (-2.95 vs -3.38 m). `lmp_v10c`
  switches between the sources, so its output flickers between two levels. The raw sources
  disagree by the same amount.
- On 14-21-45, rb's edges sit 0.05–0.15 m **outward** in obstacle frames. That is the seen-road
  floor; it is small, but it is a bias.
- In the worst 14-18-56 frame, the rb planner output's right edge curls inward beyond about 35 m;
  the far range is not clean either.
- **Occlusion only (`rbo`) is worse, not better, as planner input.** It is sparse and leaves
  stations empty (100% at 10 m). `lmp_v10c` needs the dense fitted input, so keep the default steps.
- During this run SAM3 used `text_prompt="gravel road"` and `confidence_threshold=0.15`
  (set live). That applies to all arms equally, since they share the SAM3 source.
- Still stationary only. `lmp_v10c` adapts its filtering to speed, so a moving drive with traffic
  remains the deciding test.

## Measured against the raw topics (2026-10-07; overstates the gain, see above)
`scripts/road_boundary_ab.py` replays recorded inputs through the production pipeline and
compares against the raw topics recorded in the same replay. Recordings are in
`~/fusion_data/recordings/roadb_inputs_*`; the tables are in `roadb_road_bags_ab.txt` and
`roadb_selfcal_ab.txt`.

**Only the two `perception_input_2026-09-22` bags are roads.** Raw edges there are about +3.9 /
-3.5 m. In both, the vehicle is **stationary** and traffic passes it.

Default config versus raw, in frames with an obstacle near the edge:

| bag | source | jump at 10/20/40 m (raw → new) | narrowing at 20 m | empty at 20 m |
|---|---|---|---|---|
| 14-18-56 | SphereFormer | 0.19/0.26/0.34 → 0.02/0.03/0.04 m | 15.2% → 0% | 15.9% → 0% |
| 14-21-45 | SphereFormer | 0.32/0.19/0.38 → 0.04/0.03/0.03 m | 9.3% → 0% | 19.3% → 0% |
| 14-18-56 | SAM3 | 0.48/0.27/0.61 → 0.11/0.05/0.21 m | 13.6% → 0% | — |

Narrowing means the width is more than 1 m below its ±5 s median. In clear frames the default
config matches or beats raw on every metric but one: SAM3's empty share at 10 m on 14-18-56
rises from 5.4% to 6.3%. Tracking contributes most of the smoothing. Occlusion is what keeps the
width in obstacle frames: SphereFormer width deviation at 20 m is 0.21 → 0.06 m (14-18-56) and
0.17 → 0.12 m (14-21-45), occlusion off → on.

The Aug 25 bags and `selfcal_loc_2026-09-08` are **wide paved lots**, with raw "edges" 17–39 m
out. A segmentation boundary there is ill-defined for raw and new alike, so those runs say
nothing about obstacles. They are why extraction is off.

## Not yet shown
- A **moving** vehicle on a road with traffic: no such recording exists with SphereFormer, SAM3
  and odometry together. Record `/sphereformer_road_segmentation`,
  `/perception/lidar_2d_projection_ground`, `/segmentation/mask_image/compressed`,
  `/perception/objects`, the odom and the raw boundaries on the next drive (see the header of
  `scripts/road_boundary_ab.py`).
- The SAM3 twin's tracked width (4.8–5.6 m live) is below the raw SAM3 median (6.2 m); not yet
  explained.
- `default_width` (7.0 m) is invented. It is used only when a side is unsupported and neither the
  frame nor the tracker has a width.

Upstream changes that would help are in `road_boundary_upstream_plan.md`.

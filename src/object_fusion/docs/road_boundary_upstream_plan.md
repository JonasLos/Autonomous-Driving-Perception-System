# Road boundaries: upstream changes worth making

`road_boundary_node` repairs the SphereFormer and SAM3 boundaries **downstream**, without
editing either node. This is the list of changes **inside** those nodes that would help,
ordered from small to large effort.

## Status (2026-10-07): tiers 1–4 implemented on branch `upstream_boundary_fixes`

The work is in a separate `git worktree` (`~/Downloads/ADPS-upstream`), so the checkout the
vehicle runs from is untouched. SAM3 is a submodule; its changes are on branch
`upstream_boundary_fixes` of the SAM3 repo. SphereFormer is built as
`perception-sphereformer:upstream`; `:latest` is not touched. To bring the stack up from the
branch:

```
EXTRA_COMPOSE_FILES=docker-compose.upstream.yml scripts/run_radar.sh --replay BAG
```

Each change sits behind a parameter, so the previous behaviour stays one `ros2 param set` away.

| item | where | default on branch | revert with |
|---|---|---|---|
| A1 PNG mask | `segmentation.py` | on | (format only) |
| A2 per-term prompt stats | `segmentation.py` | every 50 inferences | `prompt_stats_period:=0` |
| A3 row-wise ego-component contours | `segmentation.py`, `road_mask_boundary.py` | `boundary_mode=rowwise` | `boundary_mode:=legacy` |
| A4 obstacle prompt (same encoding) | `segmentation.py` | `obstacle_prompt="car or truck or person"` | `obstacle_prompt:=""` |
| A5 whole-mask lifting, with edge evidence | `sam3_mask_transform.py`, `road_mask_boundary.py` | `lift_mode=mask` | `lift_mode:=contour` |
| S1 `label` field (uint8) | `sphereformer_ros.py` | on | (extra field only) |
| S2 ego crop before inference | `sphereformer_ros.py` | `ego_crop_radius=2.5` | `ego_crop_radius:=0.0` |
| S3 fixed vehicle-frame bins, with edge evidence | `sphereformer_boundary.py` | `boundary_mode=fixed` | `boundary_mode:=legacy` |
| S4 ego-corridor runs | `sphereformer_boundary.py` | **off** (`connected_runs`) | — |
| S6 numpy cloud packing | `sphereformer_ros.py` | on | — |
| S7 3-sweep accumulation | `sphereformer_ros.py`, `sphereformer_boundary.py` | `accumulate_sweeps=3` | `accumulate_sweeps:=1` |
| X2 `boundary_eval` recording category | `perception_common/topics.yaml` | — | — |
| X3 edge-offset diagnostic | `scripts/boundary_edge_offset.py` | — | — |

Found while doing it (each now covered by a unit test):
- **Ring arcs.** A 2 m bin at range often holds only an arc of one LiDAR ring, which ends inside
  the road. Without an edge-evidence rule, fixed bins reported a right edge at -0.3 m (true
  -2.9 m) at 40 m, and SAM3 lifting reported one at +1.4…+2.6 m. Both rules now keep an extreme
  only when non-road GROUND is seen within 1 m beyond it. Ground matters: a pedestrian on the road
  otherwise "confirmed" an edge at their feet. Extremes at the camera's field-of-view border are
  dropped too.
- **S4 misfires on wide lots.** On the Aug 25 drives, connected-run selection latched onto strips
  of an open paved lot (jumps of 5–14 m). Off by default, as in road_boundary.
- **A road_boundary tracker bug** (fixed on this branch). A sparse fit through an occluded
  stretch initialised stations with a left edge right of the right edge. Correct measurements were
  then gated as outliers, so the bad stations coasted for up to 3 s; through `lmp_v10c` that was a
  ~1 s, 4 m swing of the planner's edge. Now: crossed stations are dropped, and a station gated 3
  frames running re-initialises. **This fix should also go to `radar_integration_and_fusion`**,
  where road_boundary is committed (off by default).
- **X3 answered:** the camera and LiDAR disagree on what counts as road, not on calibration. On
  this gravel road SAM3 (prompt "gravel road") stops at the gravel, and SphereFormer stops at the
  vegetation, so it includes a ~0.5–1 m dirt shoulder on both sides. SAM3 is narrower on BOTH sides
  (left -0.4…-0.8 m, right +0.2…+0.5 m at 15–30 m); a calibration error would shift both edges the
  same way. The overlay is `~/fusion_data/recordings/planner_ab_plots/x3_overlay_*.png`.

Measured through three/four unmodified `lmp_v10c` instances on the same live replay (stationary
Sep 22 road bags, `scripts/road_boundary_planner_ab.py`). At 20 m, in obstacle frames:

| arm | pulled in > 0.5 m / > 1 m | centreline p95 | jitter | empty |
|---|---|---|---|---|
| today's vehicle code, raw | 27–31% / 13% | 0.63 m | 0.10 m | 7–8% |
| today + road_boundary | 0–2.4% / 0–0.2% | 0.15–0.19 m | 0.08–0.10 m | 0% |
| branch, raw (upstream fixes only; 3 runs) | 0.4–3.9% / 0–2.5% | 0.13–0.29 m | 0.04–0.10 m | 8–12% |
| branch + road_boundary | 0–1.0% / 0% | 0.08–0.12 m | 0.02–0.05 m | 0% |
| branch + road_boundary + `lmp_v10c` guards* | 0% / 0% | 0.06–0.10 m | 0.006–0.007 m | 0% |

\* `enable_coherent_source_mode` and `enable_pair_dropout_hysteresis`, set at runtime with no
planner edit.

Other measurements:
- **SphereFormer rate:** about 7.9 Hz, up from 6.3 Hz (S6).
- **SAM3 frame time:** about 0.70–0.74 s. The obstacle prompt costs about 160 ms of that
  (three terms at about 53 ms each); `'gravel road'` itself takes 124 ms.
- **Prompt stats** on the road bag: `'gravel road'` gives a mask in 100% of frames (mean score
  0.62), `'person'` in 30%, `'car'` in 2%, `'truck'` in 0%.

**Decision needed:** on the branch the planner's road is about 0.9 m wider: at 20 m the edges sit
at about +3.8/-3.4 m against +3.2/-3.0 m today. `lmp_v10c` now follows SphereFormer's edge,
which includes the dirt shoulder (X3). Whether the shoulder is drivable decides whether that is
right, or whether SAM3's prompt should include it.

Not done (tier 5): S8 (amodal retraining), A6 (SAM3 video mode), X1 (typed boundary message),
and S5 (per-point state field). Not yet shown: a moving drive; the A/B above is stationary only.

Each item gives what to change, why (with the evidence), and how to measure it. The measuring
tool for every item is `scripts/road_boundary_ab.py`. Its `raw` arm is whatever the upstream
nodes publish, so an upstream change shows up directly as a new raw arm on the same recording.

Measured facts used below come from the `perception_input_2026-09-22_14-21-45` replay
(2026-10-07):
- SphereFormer published 269 frames in 42.8 s, about 6.3 Hz from a 10 Hz LiDAR.
- SAM3 published 93 masks, about 2.2 Hz.
- `/sam3_*_boundary` was published only 52 times.
- SAM3 logged "No masks detected" 122 times over that replay plus the start of the selfcal one.

---

## SphereFormer (`src/sphereformer_ros/sphereformer_ros.py`)

### S1. Publish the class id as a field (small)
`/sphereformer_road_segmentation` carries only a packed RGB colour. Consumers have to invert the
SemanticKITTI palette, and `road_boundary.py` keeps a copy of it. Add a `label` field (uint8)
next to `rgb`; the colours stay for RViz.
*Measure:* none needed. This is an interface fix that removes a hidden coupling.

### S2. Crop the ego vehicle before inference (small)
Every sweep has ~250–560 points at **0.8–0.9 m** from `lidar_tc`, labelled `car`: the roof and
hood. Downstream they cast shadows over everything; before `ego_radius` existed, 42% of boundary
samples read as occluded. They may also distort neighbouring predictions.
Crop a vehicle box (or r < 2.5 m) from the input cloud in `inference_from_ros_message`.
*Measure:* the share of `car` points within 3 m should drop to ~0, with no change in road labels.

### S3. Fixed, vehicle-frame bins for the boundary (small)
The boundary uses `np.linspace(min_x, max_x, 22)` over **all** road points. On the replay that
range was -30…92 m, so the bins are about 5.8 m wide, cover the road behind the vehicle, and
change width every frame as the farthest road point moves. (They are cut along `lidar_tc` x,
yawed 5.35°, but that does not shear the picked points: on a straight edge the extreme of a
tilted strip still lies on the edge. The yaw only matters for where the stations fall.)
Use fixed bins (for example 2 m from 3 to 60 m) after rotating into the vehicle frame
(`perception_common.lane_geometry.DEFAULT_EGO_YAW_DEG`, as the lane node already does).
*Measure:* `jump` and `empty` in the raw arm, on clear frames.

### S4. Keep only road connected to the ego corridor (small–medium)
At x≈4 m the raw left edge read **11.7–22 m** on most frames: a road-labelled patch beside the
road became the boundary. The disabled `cluster_pointcloud` DBSCAN does not fix this either,
because it removes outliers but keeps every cluster.
Do **not** port `road_boundary.select_run` as it stands. It keeps, per bin, the lateral run
overlapping the previous bin's run. It fixed this patch on the road replay, but on the wide
paved lots (the Aug 25 and selfcal drives) it latched onto arbitrary runs and caused 3–6 m
jumps, which is why `enable_extraction` is off by default.
A version that works upstream needs a run reference that does not drift, for example the
lane centreline from CLRerNet, or the previous frame's published boundary carried forward with
odometry, rather than the previous bin.
*Measure:* the `narrow` and `wdev` metrics on BOTH a road bag and a wide-lot bag.

### S5. Mark, don't hide, what the bin could not see (medium)
Today an empty bin is silently skipped, and an extreme with an obstacle beyond it is published
as an edge. Publish a per-point `state` field (edge / occluded / unsupported) in the boundary
cloud, using the same rules as `road_boundary.bin_extremes`.
The planner could then weight the points, instead of the downstream node having to re-derive
them. This also needs the consumer change in X1.
*Measure:* the planner-side A/B in `~/latest_planner`, since the effect is on consumption.

### S6. Throughput: drop the per-point Python loops (small)
`create_cloud` and `create_cloud_rgb` build a Python list of tuples per point (35k points, every
frame). Use `point_cloud2.create_cloud` with a structured numpy array, or write the buffer
directly.
The node runs at about 6.3 Hz on a 10 Hz input, and more frames would feed the tracker and
shorten the time an occlusion has to be coasted.
*Measure:* the published rate (`scripts/publish_rate.py`) and per-frame time in the node log.

### S7. Multi-sweep accumulation before the boundary (medium)
Ground rings are 3–6 m apart along x beyond 20 m, so a single sweep often holds only an arc of
a ring in a bin. That is why `road_boundary` needs the edge-evidence rule.
Accumulate the last N labelled sweeps (N≈5), motion-compensated with the odometry twist
(`object_fusion.ego_motion.TwistBuffer`), before taking extremes. This densifies the rings and
partly fills shadows behind moving obstacles.
*Measure:* the `unsupported` share in the node log should fall; `jump` on clear frames.

### S8. Amodal "road under obstacles" labels (large)
Fine-tune SphereFormer so that ground points under or around vehicles are labelled road. The
labels can be generated automatically: aggregate many sweeps per drive with odometry, take the
road footprint where no obstacle was present at some time, and project it back into each sweep.
This fixes the root cause in the model, but needs a labelling pipeline, training compute and a
held-out evaluation.

---

## SAM3 (`src/SAM3_ROS_NODE/src/sam3_ros/segmentation.py`, `sam3_mask_transform.py`)

### A1. Lossless mask (small)
`mask_image/compressed` is produced by `cv2_to_compressed_imgmsg(mask_image_np)`, which encodes
JPEG. The ids are 0..N, so compression rings around the outline, and `road_boundary` has to
apply a 3×3 opening to undo it. Pass `dst_format="png"`. The mask is mostly flat, so PNG is also
small.
*Measure:* message size (should be similar or smaller), and the opening becomes a no-op.

### A2. Investigate the empty frames first (small, diagnostic)
"No masks detected" was logged 122 times over about two replays, and `/sam3_*_boundary` was
published for 52 of 93 masks. When SAM3 finds no road at all, every downstream rule has nothing
to work with, so this is the largest single loss in the camera path.
Things to check:
- whether the text prompt (`"road or gravel road or …"`, split into OR queries) fails on
  specific surfaces or lighting
- what the score threshold on each query is
- whether `input_max_size=1024` downscaling hurts at range

Log which prompt term produced each mask and its score.
*Measure:* the share of frames with at least one mask, per prompt term, on the selfcal replay.

### A3. Row-wise extents of the ego-connected component (small–medium)
`_extract_and_publish_boundaries` keeps the **largest** external contour and splits each row
band at the band's **mean x**. Two failures follow:
- A car's outline becomes part of the boundary, because a notch puts four contour points in a
  row band and the mean-x split hands two of them to each side.
- A car cutting off the far road removes that road, because the far part is a separate contour
  and only the largest survives.

Also, the contour's top edge (the horizon line of the mask) is classified into left and right
too. Instead:
1. Fill holes.
2. Keep the connected component containing the bottom-centre ego pixel.
3. For each row, take the outermost road pixels.
4. Drop the top rows of the mask, which are the horizon, not an edge.

*Measure:* the `narrow` metric on frames with an obstacle near the edge, in the SAM3 raw arm.

### A4. Obstacles as occluders, from the same pass (medium)
Run a second prompt (`"car . truck . person"`) on the same `inference_state`. `set_image` is the
expensive part and is shared, so the extra prompt is cheap.
Publish the obstacle mask, and mark a boundary pixel occluded when it touches a dilated obstacle
mask. This gives the camera path its own occlusion evidence, instead of borrowing SphereFormer's
(which `road_boundary` does today, and which is missing whenever SphereFormer is not running).
*Measure:* the SAM3 `all` arm with and without SphereFormer obstacles.

### A5. Lift the whole mask, ground-only, in `sam3_mask_transform` (medium)
The transform matches **contour pixels** to LiDAR returns within 5 px (`query_ball_point`). Near
an edge, that radius also catches non-ground returns behind it, such as vegetation, a car body,
or a distant wall.
Lifting every projected **ground** return whose pixel is road gives a 3D road set. That is what
`BoundaryPipeline.sam3_frame` does, and both paths can then share one extraction rule. The
ground flag is already available on `/perception/lidar_2d_projection_ground`.
*Already measured:* this is the A/B's `none` arm (lifting only, no other step). On the two road
replays, at 10 m:

| bag | arm | jump | empty |
|---|---|---|---|
| 14-18-56 | raw | 0.37–0.48 m | 5–16% |
| 14-18-56 | lifting only | 0.01–0.04 m | 0% |
| 14-21-45 | raw | 0.40–0.60 m | 33–46% |
| 14-21-45 | lifting only | 0.01–0.02 m | 57–65% |

On 14-21-45 lifting alone is empty more often. The cause has not been checked yet; the tracker
in the default arm fills those stations (4–19% empty). This is the
cheapest large SAM3 gain on the list.

### A6. Rate and latency (large)
Masks come at about 2.2 Hz and 0.5–1.2 s after capture. SAM3's video predictor can propagate
masks between keyframes for far less than a full `set_image` per frame. Running keyframes at
about 2 Hz and propagating in between would raise the boundary rate to the camera rate.
That is a restructuring of the node: a session per stream, keyframe policy, and drift resets.
*Measure:* rate, latency (`ros2 topic delay`), and `jump`.

---

## Cross-cutting

### X1. A typed boundary message (medium, needs a consumer change)
All four boundary topics are anonymous `PointCloud2` xyz. A message carrying, per point, a
station, a lateral offset, a state (edge / occluded / unsupported / predicted) and a sigma would
let the planner trust the boundary where it was seen and widen its margin where it was coasted.
This needs a planner change, so it should go through the `perception-fusion-integration` branch
in `~/latest_planner`.

### X2. Record what the boundary saw (small)
Add `/sphereformer_road_segmentation`, `/segmentation/mask_image/compressed` and the
ground-flagged projection to the vehicle recording profile. Every boundary question so far has
needed a fresh replay just to get them.

### X3. Make SAM3 and SphereFormer agree on where the edge is (small diagnostic, then medium)
Found in the planner-level A/B (2026-10-07; see `road_boundary.md`). On the Sep 22 road bags at
20 m, SAM3 places the left edge about **0.75 m inside** SphereFormer's (3.1 vs 3.87 m) and the
right about **0.43 m inside** (-2.95 vs -3.38 m). This holds both for the raw topics and for
road_boundary's twins. Each source is steady on its own, so `lmp_v10c` switching between them is
now the main source of flicker in the planner's edge.

Possible causes, in the order they are cheap to check:
1. **The two models define "road" differently.** SphereFormer's road class may include a gravel
   shoulder that the "gravel road" prompt excludes, or the reverse. Check by projecting both
   edges into the camera image on a few frames.
2. **Camera–LiDAR extrinsic error.** SAM3's edge is lifted through `T1`. An error there would
   show up as an offset that grows with range, rather than a constant one.
3. **Lifting bias.** Contour pixels matched within 5 px pick up inner returns (raw path); in
   road_boundary's lifting, ground-flag gaps at the edge would have the same effect.

The fix depends on the answer: re-prompting, a calibrated per-source lateral offset, or an
extrinsic correction. Until then, `lmp_v10c` should not switch sources from frame to frame.
That is a planner-side change (hysteresis on source mode, which it already has as the
`enable_pair_dropout_hysteresis` / `enable_coherent_source_mode` flags, both off in v10c).
*Measure:* SAM3-vs-SphereFormer edge offset at 10/20/40 m, and planner output jitter.

## Suggested order
1. A1, S1, S2, S6 and X2 are small, low-risk interface or throughput fixes.
2. A2 and X3 are the biggest unknowns, and each costs only logging or a projection check.
3. Then A3 and S3/S4, measured with `road_boundary_ab.py` against the downstream node. If the
   upstream raw arm catches up with `road_boundary`, the downstream node can be reduced to its
   tracking step.
4. Only after that, S7, A4 and A5.
5. S8, A6 and X1 are projects in their own right.

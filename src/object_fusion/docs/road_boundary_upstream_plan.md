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

### Public-road test: Route2 (`rosbag2_2026_05_05-14_52_50`, 12 min at ~19.5 m/s)

Each stack was run once over the full drive, with `text_prompt="paved road or gravel road"`.
The branch run also had `obstacle_prompt="car or person"`. In a first-150 s prompt survey,
`paved road` found road in 60–100% of frames, and `gravel road` took over on weathered
stretches. The camera here is `/camera_fl/image_color`, remapped on playback; radar was left out
because its tracks topic has a different type. Results files are
`~/fusion_data/recordings/route2_*_{planner,perception}.txt`.

**Perception level, clear frames (>95% of the drive), today → branch:**

| | SphereFormer 10/20/40 m | SAM3 10/20/40 m |
|---|---|---|
| frame-to-frame jump | 1.28/1.18/0.94 → 0.90/0.73/0.54 m | 0.36/0.48/0.67 → 0.30/0.49/0.55 m |
| gaps | 6.6/17.6/16.4 → 2.8/5.1/16.7% | 31.7/12.2/28.2 → **67.1/25.6/43.4%** |
| rate | 4.65 → 6.4 Hz | 2.3 → **1.4 Hz** (obstacle prompt) |

road_boundary on top brings SphereFormer's jumps to 0.30–0.37 m and its gaps to 0–0.4%, on
either version. Only 51–87 frames had a fused vehicle near an edge, too few to settle the
obstacle question on this drive.

**Planner level: `lmp_v10c` does not fit this road.** The segmented road is the whole two-way
road: the median width of rejected pairs is about 11 m, against `max_lane_width` 8.5 m. After a
run of rejections, its "insufficient overlap with the previous motion-compensated boundary"
check compares against a boundary left far behind at 20 m/s and never accepts again. It
accepted 2 pairs in 12 min on today's code, and 723–976 on the branch. These are planner
issues, not fixed here: it needs a lane, not the road, on two-way roads, and it needs to re-seed
when its reference is stale.

**Suggested SAM3 follow-ups from this drive:** drop or shorten the obstacle prompt
(`obstacle_prompt:=""` or `"car"`) to recover the rate. Also check whether the field-of-view and
edge-evidence rules are too strict where the road is wider than the camera's view at 10 m.

### SAM3 rate with several prompts (2026-10-07)

Each OR term was a full `set_text_prompt`: the text encoder, then the grounding encoder, decoder
and heads, re-run per term over the same image features. Only the image encoder (~67 ms) was
shared. The grounding kernels are small and launch-bound, so CPU load from the rest of the
stack stretches them. Measured on GPU 1 for four terms (two road, two obstacle):

| | sequential (per term) | batched (one pass, text cached) |
|---|---|---|
| model only, idle machine | 177 ms | 123 ms |
| model only, full stack running | 269 ms | 143 ms |
| the node, full stack running | 566 ms (1.7 Hz) | 270 ms (3.7 Hz) |

The batched masks are identical (IoU 1.000 per term). This batches the prompts of ONE frame,
not several frames, so it adds no latency. Sharing GPU 1 with CLRerNet costs about 7%. CPU
contention from the fusion stack and the GPU-0 nodes is what slows the per-term path (grounding
92 → 165 ms). The visualisation timer and per-frame image decoding made no measurable difference.
`batch_prompts:=false` restores the per-term path. Profiler: `scripts/sam3_prompt_profile.py`.

**Around the model (SAM3 `62bfe25`):** the node's per-stage timing (`profile_stages:=true`)
showed the row-wise boundary extraction costing 70–90 ms per frame under load. It now runs at half
resolution (21–24 ms). The raw result image is built only when subscribed (12–16 → 2–3 ms). Per
frame under the full stack: about 307 → 244 ms, about 4 Hz with four terms. What remains is the
model: image encoder about 80 ms and batched prompts about 110 ms (about 70 ms on an idle
machine). Tried and dropped: PNG compression level 1 (slower to publish), and fp16 weights (SAM3
mixes in fp32 tensors internally; autocast already runs the matrix maths in fp16).

**Effect of the faster SAM3 (reruns, 2026-10-07):**
- **Sep 22, through `lmp_v10c`:** without its source-switching guards the edge flickers more.
  With SAM3 arriving more often, it alternates between SAM3-fused and SphereFormer-only pairs,
  and those disagree by about 0.6 m (X3). Jitter for `up` went from 0.04–0.09 m to 0.12–0.16 m.
  With the guards on, `up_rb_coh` stays the best arm: jitter 0.006 m, no pulls over 1 m. So turn
  the guards on together with the faster SAM3.
- **Route2:** SAM3 rate 1.5 → 3.1 Hz. Through road_boundary the SAM3 edge's frame-to-frame
  jump falls (0.46 → 0.30 m at 20 m), partly just because frames are closer together in time.
  Raw SAM3 gaps are unchanged; that's the field-of-view and edge-evidence issue.

### SAM3 speed options 1, 2, 3 and a TensorRT scope (2026-10-07)

Measured with the instrumented node (`profile_stages:=true`) under the full replay stack on
Route2, GPU 1, four prompt terms, at least two runs each. The baseline in this session was
210–217 ms (4.4–4.5 Hz):

| option | SAM3 per frame | effect on the rest | status |
|---|---|---|---|
| 1 CPU pinning: SAM3 0-3, SphereFormer 4-11, rest 12-31 | 194–196 ms (p90 ~205), ~5.0 Hz | SphereFormer 7.4 → 8.6 Hz; YOLO, CLRerNet unchanged; fusion lateness unchanged | **opt-in** `docker-compose.cpuset.yml` / `docker-compose.fusion.cpuset.yml` |
| 2a SAM3's built-in `compile=True` | idle 123 → 117 ms | +30 s start-up | not adopted |
| 2b CUDA graphs for the batched pass (`cuda_graphs`) | 167–171 ms, ~5.6 Hz | none measured | **opt-in** parameter |
| 1 + 2b together | 159 ms (p90 168), ~6.0 Hz | as 1 | — |
| 3 CLRerNet on GPU 0 | 188 ms (vs 195 with pinning), within noise | SphereFormer slightly lower; a third model on the YOLO GPU | not adopted |

The other SAM3 models above don't move: the image encoder stays at about 79 ms, now the
largest single stage.

**CUDA graphs:** masks match eager (IoU ≥ 0.9989 per term). The first two frames compile and
record: about 166 s cold under load, about 47 s with the persistent cache
(`compile_cache_dir`, about 200 MB, inside the Hugging Face mount; about 27 s with pinning as
well). Compiler workers are capped at 4 threads.

**Pinning only restricts the perception containers.** The planner, controller and sensor drivers
run outside them, keep every core, and now share 12-31 with fusion, YOLO, transform and
CLRerNet. It was measured with perception alone; validate on the vehicle with the whole software
running (controller loop rate, driver rates) before relying on it. The layout is specific to
this i9-14900 (P-cores 0-15 as hyperthread pairs, E-cores 16-31).

#### TensorRT for the image encoder: scope, not started
- **Target:** the encoder at about 67 ms idle and about 79 ms under load, now the largest stage.
- **Export works.** `torch.export.export(..., strict=False)` captures the vision backbone at
  1008×1008 in 2 s, giving 1,763 ops. **The blocker** is the rotary position encoding,
  implemented with complex numbers: 64 `view_as_complex` and 64 `view_as_real` (`vitdet.py`,
  `torch.polar`). TensorRT has no complex type. Fix: replace the rotation with the equivalent
  real-valued cos/sin form, patched from the node rather than editing the sam3 package, and
  check that the output is unchanged.
- **Packages:**
  - The SAM3 venv has torch 2.10.0+cu128. `torch-tensorrt` 2.10.0 targets exactly that torch
    (x86_64 wheel) and requires `tensorrt` ≥ 10.14.1, < 10.15. That generation supports the
    RTX 5080 (Blackwell).
  - Both would be added to the SAM3 `pyproject.toml`, which the container installs with
    `uv sync` at every start: a download at first start and roughly 2 GB more disk.
  - `onnxscript` is not installed, so the ONNX route was not tried. The Torch-TensorRT route
    compiles the exported program directly.
- **Integration:**
  - A `tensorrt_encoder` parameter wraps `backbone.forward_image`. Its outputs (FPN features and
    position encodings, plus the SAM2 branch the processor post-processes) must match the eager
    ones in structure.
  - The engine is built once per GPU model and TensorRT version and cached on disk like the
    compile cache. Engine build is minutes.
- **Validation:**
  - per-term mask IoU against eager on the profiler frames (≥ 0.99, as for CUDA graphs)
  - encoder output difference in fp16
  - a Route2 perception-level rerun
- **Expected gain (an estimate, not measured):** fp16 TensorRT on a ViT this size is typically
  1.5–3× faster than eager with autocast, so the encoder would drop from about 79 to about
  30–50 ms, and a frame from about 160 to about 115–140 ms (about 7–8 Hz with CUDA graphs and
  pinning).
- **Effort:** about 2–4 days. In order: the rotary rewrite and an equivalence test, export plus
  Torch-TensorRT compile, node integration with the engine cache, container dependencies, and
  validation.
- **Risks:**
  - further unsupported operations after the rotary fix
  - fp16 drift in the encoder showing up as mask changes
  - TensorRT/torch version pinning tying future torch upgrades to a matching torch-tensorrt
  - longer first start while the engine builds

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

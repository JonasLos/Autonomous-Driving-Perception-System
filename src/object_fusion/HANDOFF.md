# object_fusion — handoff

State as of 2026-09-15. Everything here is NEW; no pre-existing repository file was modified
at any point. `git status` shows the same 12 pre-existing entries it did at the start.

## RESUME HERE (updated 2026-09-16)

**Start with "TO DO" at the bottom of this file** -- it is the prioritised plan, each item with
its steps and a done-when test.

**User constraints -- keep obeying them:** never modify a pre-existing repository file
(`git status --short | grep -v '^??'` must show exactly the 12 pre-existing entries); never touch
the existing pipeline in `Custom_YOLO_ROS`; all new code lives in `object_fusion` or new files.
Runtime `ros2 param set` is fine. Measure offline before changing behaviour and show the user the
numbers. sudo needs a password -- never ask for it; `systemctl --no-ask-password reboot` works,
restarting system services (e.g. anydesk) does not. Nothing is committed yet (all untracked).

**Live demo** (the user watches RViz on DISPLAY=:1). The `perception-object-fusion` image is
current as of 2026-09-15; rebuild only after code changes.

```bash
scripts/run_radar.sh  --replay ~/selfcal_loc_2026-09-08_11-47-43 --obstacle-only     # existing stack
scripts/run_fusion.sh --replay ~/selfcal_loc_2026-09-08_11-47-43 --ground --debug-clouds
DISPLAY=:1 rviz2 -d src/object_fusion/config/object_fusion.rviz --ros-args -p use_sim_time:=true &
scripts/play_rosbag.sh -l ~/selfcal_loc_2026-09-08_11-47-43      # from the START (tf_static)
```

Neither stack depends on the bag path, so switching bags = stop the player, start another
(`adps_2026-08-25_11-58-32` and `_11-50-45` in the repo root were added by the user; the truck in
11-58-32 is the class-vote case). The scratchpad (`/tmp/claude-*`) does not survive a reboot:
reinstall host Patchwork++ per "Run it" before any offline harness.

**Live by default (all measured, all with a rollback parameter):** Patchwork++ ground flags
(`--ground`), empty-box drop (`segmentation_empty_fallback:=true` reverts), depth-jump gate
(`enable_depth_gate`), class vote (`enable_class_vote`), near-field odometry levelling
(`ground_levelling`). `publish_mode` stays `passthrough`, extent estimation off (user's choice).

**This version is the one the user watched and called good** (2026-09-16, on
`adps_2026-08-25_11-58-32` in `publish_mode:=filtered`). It carries the three motion fixes and the
restricted merge rule below; the image is current. Reading the node while it runs: the 5 s status
line shows `rewinds=` and per-sensor `dt<=0 n/N med +Xms` -- if `dt<=0` is not ~0/N the filter is
not predicting, which is the failure that looked like "the boxes update slowly". In RViz, a white
arrow on a track is one second of its velocity; in `passthrough` there are none by design.

**Two things the user still wants looked at**, deliberately NOT chased yet (to-do items 11 and 12):
cones still being dropped, and the aggregator's positional accuracy against the measurements.

## What exists

```
src/object_fusion/            ROS-free modules + 6 nodes + launch + config (incl. RViz) + 149 tests
src/custom_msgs/fusion_msgs/  Detection3D(Array), FusedObject(Array)
scripts/object_ab.py          offline harness (imports production rules, never copies)
scripts/ground_ab.py          ground-removal A/B (point selection vs radar range + jitter)
scripts/neighbour_ab.py       box-jumping A/B (depth cut / neighbour objects / clustering / gate)
scripts/lean_ab.py            ground misclassification in curves (levelling variants)
scripts/run_fusion.sh         bring-up / status / shadow-report / down
scripts/fusion_isolation_check.sh + isolation_compare.py   does the new stack perturb the old one
scripts/record_baseline_replay.sh   record the existing pipeline's output for a new bag
docker/Dockerfile.object_fusion      layered on perception-transform:latest (+ pypatchworkpp 1.4.1)
docker-compose.fusion.yml + .replay.yml
```

Rollback is `rm -rf` on those paths plus `docker image rm perception-object-fusion`.

## Run it

```bash
docker compose -f docker-compose.yml -f docker-compose.fusion.yml --profile fusion build object_fusion_node
scripts/run_radar.sh  --replay ~/selfcal_loc_2026-09-08_11-47-43 --obstacle-only   # existing stack
scripts/run_fusion.sh --replay ~/selfcal_loc_2026-09-08_11-47-43 --ground --debug-clouds
rviz2 -d src/object_fusion/config/object_fusion.rviz --ros-args -p use_sim_time:=true
scripts/play_rosbag.sh -l ~/selfcal_loc_2026-09-08_11-47-43                        # FROM THE START
python3 -m pytest src/object_fusion/test -q
```

- `--ground` points the detector at `/perception/lidar_2d_projection_ground` (Patchwork++
  flags per point); without it the detector reads the existing `/lidar_2d_projection`.
- `--debug-clouds` publishes `/perception/ground_debug/{ground,nonground}` for RViz.
- **Never replay with `--start-offset`.** `/tf_static` is only at the bag start; skip it and the
  radar frame never resolves (RViz shows the radar display red, the aggregator gets nothing).
- RViz colours: grey non-ground, brown ground, green existing `/fused_bbox`, blue new
  camera-LiDAR measurement, orange radar tracks.
- Offline Patchwork++ on the host: `pip install --target <dir> pypatchworkpp==1.4.1`, then
  delete `numpy*` from `<dir>` (it pulls numpy 2.x, which breaks the system scipy/mcap), and run
  with `PYTHONPATH=<dir>`. No venv — `python3.12-venv` is not installed and sudo needs a password.

`/perception/objects` cannot be echoed from the host — `fusion_msgs` lives only inside the
image. Use `docker exec perception_object_fusion_node bash -lc '. /opt/ros/jazzy/setup.sh &&
. install/setup.sh && ros2 topic echo /perception/objects --once'`.

## Bags

| bag | what it is |
|---|---|
| `~/selfcal_loc_2026-09-08_11-47-43` | **source**, 398 s, has `/odom` AND `/odom_grid`, 3 lidars, camera, radar |
| `~/fused_replay_selfcal_2026-09-08` | baseline replay output, 242 MB, has `/lidar_2d_projection` |
| `~/fused_replay_widecrop_2026-09-08` | crop widened to 60 m lateral / 200 m range |
| `~/fused_replay_voxelon` | voxel filter 0.1 m |
| `~/fused_replay_ground{low,10,15}` | ground gate at 5 / 10 / 15 m |
| `~/selfcal_loc_2026-09-03_11-17-00` | 332 s, all topics; out-of-sample drive for levelling (only 15 curve sweeps) |
| `adps_2026-08-25_11-58-32` (repo root) | 18 s, all topics; truck passing at ~9 m (the truck->train class-vote case) |
| `adps_2026-08-25_11-50-45` (repo root) | 52 s, all topics, straight; no `/fused_bbox` replay yet |

## Measured — settled, do not re-derive

| finding | value |
|---|---|
| Vehicle axis in `lidar_tc` | **−5.35°**, three independent methods within 0.19° |
| Radar boresight vs direction of travel | +0.041 ± 0.007° — the radar frame IS the vehicle axis |
| `base_link` vs direction of travel | ~0.2° (INS reports constant +0.212° "sideslip" on straights) |
| `/odom` vs `/odom_grid` | twist **bit-identical**; only pose yaw differs, by +1.287° grid convergence |
| INS twist reference point | the **IMU**, not `base_link` (lever arm fit 3.63 m ≈ radar−IMU 3.573) |
| `pose.position` refresh | ~1/3 of message rate — **integrate the twist, never difference the pose** |
| Fused range error vs radar | −1.1 m to 80 m; **−7.6 m** at 80–100; **−32 m** at 100–175 |
| Ground gate 25→10 m | 15–25 m err sd **2.24 → 1.15**; 0–15 m >1 m jumps **25.9% → 17.4%** |
| Camera gate rejection | 16.8% → **10.6%** after the lockout escape; concentrated NEAR field |
| Innovation shape | heavy-tailed: q90–q99 run **4–12×** χ²(2) while the bulk sits *below* it |
| Radar NIS | **94.5–96.9%** under-gate in every band 0–180 m — calibrated, leave alone |
| `transform.py` image bounds | uses camera_info **2064×1544**; the principal-point fallback is 1 px short and drops the last column (`projection.CAMERA_INFO_WH`) |
| Patchwork++ cost | 4 ms/sweep offline, 7.3 ms live; `max_range` **120** (150 breaks 60–80 m: jitter p90 49 m) |
| Odom attitude | `/novatel/oem7/odom` orientation roll = INSPVA roll (corr +1.000); pitch **sign-inverted** (−1.000) |
| Patchwork++ `intensity_thr` | the Python binding leaves it **uninitialised** (read 3.9e+180) — always set it |

### Adopted ground rule (live since 2026-09-14)

Patchwork++ ground flags → drop flagged points → 10th-percentile height cut from 10 m → nearest
depth cluster. (A box left EMPTY by segmentation now publishes nothing — see "Box jumping"; the
table below was scored with the old fall-back-to-all-points rule, arm G+f vs A25, 4519 detections,
`ground_ab.py --backend patchworkpp --max-range 120`.)

| band | range err sd | jitter median / p90 |
|---|---|---|
| 15–25 m | 2.24 → **1.06** | 0.40 / 3.24 → **0.12 / 1.21** |
| 25–40 m | 1.04 → **0.69** | 0.25 / 4.33 → **0.15 / 2.43** |
| 40–60 m | 2.30 → **1.58** | 0.31 / 6.73 → **0.17 / 2.16** |
| 60–80 m | 4.31 → **2.76** | 0.30 / 2.64 → 0.30 / 2.88 |
| 80+ m | unchanged | p90 slightly worse, n=111 |

Live: segmentation used on ~98.5% of boxes, fallback ~1.5%.

### Isolation: the new stack does not perturb the existing one (2026-09-15)

`scripts/fusion_isolation_check.sh BAG OUT [DURATION]` replays the same 150 s through the existing
stack alone (A1), with object_fusion up (B, `--ground --debug-clouds`), and alone again (A2);
`scripts/isolation_compare.py OUT` scores it. Two configurations, both on selfcal 09-08:

| | frames | `/fused_bbox` and `/tracked_objects` |
|---|---|---|
| obstacle-only stack, A1 vs A2 (noise floor) | 1484 | **bit-identical**, max move 0.000 m |
| obstacle-only, A1 vs B | 1484 | **bit-identical** |
| full stack incl. SphereFormer + CLRerNet, A1 vs B (`ISO_RADAR_FLAGS=""`) | 1483 | **bit-identical** |

- The existing nodes' pairing counters do not degrade: fusion_node matched 1447 / unmatched 1 in
  all three obstacle-only runs (expired 196 / 200 / 205, B inside the A1-A2 spread); under full
  load fusion_node expired 55 (A1) vs 29 (B), clrernet 13 vs 0.
- Replay is deterministic here, so a single pair is conclusive; that is why the full-load check
  ran only A1/B.
- CPU (mean / max % of one core): object_fusion 84.5 / 114.8 obstacle-only, 112.3 / 152.7 under
  full load. YOLO ~275-310, clrernet ~172-186, sphereformer ~134, transform ~50.
- Caveat: bag replay at 1.0x on an otherwise idle machine, and `test_sam3_container` (~150% CPU)
  was running in both runs. On the vehicle the drivers add load the replay does not have.
- Harness trap, fixed and worth remembering: `ros2 bag record` IGNORES SIGINT when it is not a
  terminal's foreground job -- the recording sat 0 bytes and `wait` never returned. SIGTERM
  flushes and exits.

### Cones: percentile cut and camera fill-in (2026-09-15) — `scripts/neighbour_ab.py`

- **The 10 m percentile height cut does not hurt cones; keep it.** Without it (arm DropNF_nopct)
  cones are unchanged (spikes 1.6 -> 1.7%) but vehicles get worse (1.5 -> 2.9%) and radar spread
  worsens in every band.
- **Segmentation fixed near-cone placement.** At 10-25 m, 79 of 191 cones sit > 1.5 m farther with
  segmentation than with fusion_node's rule. Plotted (x-z of the box points): the object is a
  vertical column of non-ground returns and the segmented position is ON it; the old rule and the
  camera estimate both land on a road ring 1.5-4.5 m in front.
- **Camera fill-in for ground-only boxes: rejected.** Arm DropNF_cam spikes 1.5 -> 2.1% (cones 1.6 ->
  2.9%); only 67/169 ground-only boxes get a camera position, 30% of fill-ins > 2 m from their own
  LiDAR neighbours. The camera ground intercept reads short of radar: -3.5 m median (sd 4.4-5.2)
  inside 40 m, -8.8 m at 40-60 m -- a box-bottom or ~0.2 deg pitch calibration bias. Calibrate
  before re-trying.
- **Bug fixed (was dormant, path off by default):** `detection_geometry.camera_only_range` took image
  row cy as the horizon and ignored the extrinsic; camera_fl is pitched ~2.7 deg down and 1.2 m
  ahead of the LiDAR, so a 30 m cone came out at ~200 m. The detector's camera-only path now uses
  `projection.camera_ground_position` (bottom-centre ray through T1 vs local LiDAR ground height).

### Ground misclassification near the car (live since 2026-09-14) — `scripts/lean_ab.py`

**The earlier table here was invalid and is removed**: its reference road was the inliers of a
RANSAC plane, the same plane the "level by road plane" variant levelled by (graded against its own
answer). `lean_ab.py` now scores against an independent reference: flat, smooth, locally-lowest
0.4 m cells, restricted to a +-6 m corridor around the vehicle's path arc. Flat terrain OUTSIDE the
corridor (drop-offs, embankments -- most of the apparent error on 09-03) is a separate column.
It also scores the cost, returns 0.25-2.5 m above the road labelled ground.

Adopted: `LevelledGroundSegmenter` -- two Patchwork++ passes, one on the sweep levelled by
`slopes_from_attitude(odom roll, pitch)`, labels from it inside 25 m, unlevelled beyond.

| road called non-ground, straight / curve | production | levelled (model fitted on the OTHER drive) |
|---|---|---|
| selfcal 09-08 (232 curve sweeps) | 3.4% / 0.8% | 2.4% / 0.4% |
| selfcal 09-03 (only 15 curve sweeps) | 3.1% / 6.6% | 2.8% / 4.3% |
| adps 08-25 11-50-45 (no curves) | 0.2% | 0.1% |
| off-road terrain 09-08 | 4.2% / 6.0% | 3.2% / 3.5% |
| objects called ground, 09-03 curves | 1.2% | 1.5% (15 sweeps) |

- Attitude model (`ODOM_TILT_COEF`, pooled 2427 sweeps): residual road tilt 1.30 -> 0.51 deg; each
  drive's fit held on the other (1.41 -> 0.59, 1.19 -> 0.61). Not rigid: ~0.8x roll, ~0.5x pitch
  (banked roads tilt with the car) plus a ~0.8 deg constant mount tilt.
- **Whole-sweep levelling regressed boxes** (neighbour_ab `--level odom`: DropNF spikes 1.5% ->
  2.6%, cones 60-80 m 5.7% -> 18.9%): the mount constant shifts the far field ~1 m at 70 m. Without
  the constant (`odom-nomount`) boxes were fine but the ground gain vanished (09-03 curves 7.2%).
  The near-only hybrid (`--level odom-near`) leaves every box metric equal to unlevelled.
- **Refuted**: real intensity (only acts through RNR); RNR off (helps off-road, hurts 09-03 curves
  12.2 -> 18.9% high side); per-sweep plane levelling (best in curves, worse on 09-08 straights);
  plane/odom blend and clamp (in between). No Patchwork++ parameter fixes it.
- Live: 12 ms/sweep (was 7-8), levelled 260/260 sweeps, no-odom 0. Params `ground_levelling`,
  `levelling_near_range` (25), `max_odom_gap_s` (0.2), `odom_topic`.

## Refuted — do not repeat these experiments

- **Crop starvation.** Widening `transform.py`'s crop 3× laterally moved the far-field error
  by ~0.35 m out of 32. Point coverage was never the cause.
- **Missing far-field returns.** Median z-spread inside an 80–100 m box is 1.24 m against a
  ~1.5 m car; only 14.6% are road-only. The vehicle points are there; selection discards them.
- **Voxel filter.** Turning it on (0.1 m) changed nothing measurable; everything ≥25 m was
  bit-identical. `transform.py`'s docstring predicted it would matter. It doesn't.
- **The in-filter ego-yaw diagnostic.** A consistent frame error is unobservable from inside
  the filter — 435σ outside, indistinguishable inside. Use `object_ab.py --ego-yaw`.
- **Gating on the rectangle fit's `quality` score.** A degenerate single-face fit scores the
  MAXIMUM (20.00) because every point lies on an edge; a good noisy fit scores 12.41.

## Cross-check against `~/jeep_selfcal_loc` (2026-09-15)

The user pointed at `~/jeep_selfcal_loc`, a separate on-vehicle self-calibration project for this
Jeep. `calib/measured_calibration.yaml` is a status-tagged record (measured / confirmed /
provisional / disputed / not measured). What it says about numbers this stack uses:

| quantity | jeep_selfcal_loc | object_fusion | verdict |
|---|---|---|---|
| LiDAR height above road | **2.366 +- 0.005 m** (measured) | 2.46 | ours was WRONG; a direct median of near-field ground returns here gives 2.394. Now 2.37. **Inert**: lean_ab at 2.46 vs 2.37 is byte-identical -- Patchwork++ re-learns the offset |
| static mount tilt | roll **-0.865 deg**, pitch -0.310 (measured) | levelling constant row -0.0142, -0.0092 (= -0.813, -0.527 deg) | **independent agreement to 0.05 deg in roll**; pitch differs by 0.2 deg (road grade is in ours) |
| `camera_fl` extrinsic | **confirmed** vendor, tilt -0.065 deg | used as-is (T1) | so the +0.7 deg cone ground-intercept offset is YOLO's BOX BOTTOM, not camera pitch -- it closes item 5's open question |
| IMU -> radar lever arm | imu at (-0.658, 0.159), radar at (2.915, -0.650) -> **(3.573, -0.809)** | same surveyed pair | confirms the numbers; the lateral one is still unfitted (item 9) |
| `lidar_tc -> base_link` yaw | **-5.4078 +- 0.020 deg** (two bags, pooled) | -5.35 (lane geometry) | a FOURTH independent method. 0.06 deg apart = 0.1 m of lateral at 100 m. Not adopted: it would change the published `ego` frame, and the gain is below the other errors here. Candidate if `ego` is ever used for control |
| camera stamp | **+18 to +21 ms late** (provisional, bags disagree by 10 ms) | pairing budget 60 ms | inside the budget, but it is a systematic the pairing does not model |
| radar x | vendor 2.915 **disputed by 0.21 m** | vendor value | affects radar->lidar association at close range; unresolved there too |
| `lidar_tc` stamp offset | **disputed**, not constant across a drive | not modelled | would show up as skew-dependent pairing error |

Also useful: tyre radius 0.3782 m, radar speed scale 0.99523 (INS/radar, agrees with wheels to
five decimals), `imu_link` is the `imu` entry and its gyro axes are swapped (pitch/roll rates
cross-track at 0.879 / 0.932).

Their `base_link` fragment (`calib/base_link.yaml`) is a drop-in for the vendor extrinsics and is
explicitly NOT on the vehicle. Nothing here was copied into the vehicle stack by this work; the
only changes made are the sensor height (2.37) and the camera height derived from it (1.5275).

## The adopted rules on the user's adps drives (2026-09-15) -- to-do item 4

`scripts/record_baseline_replay.sh BAG OUT` records what the existing pipeline publishes while a
bag replays; the harnesses then take `--source BAG --replay OUT`. Both adps drives now have one
(`~/fused_replay_adps_11-58-32`, `~/fused_replay_adps_11-50-45`), and both pass the A25 self-check.

**adps_2026-08-25_11-50-45** (52 s, 1196 detections, all vehicles, no cones):

| arm | spikes > 2 m | 0-25 m err | 25-40 m | 40-60 m |
|---|---|---|---|---|
| A10 (old rule) | 1.0% | -0.30 / 0.75 | -3.16 / 4.30 | -5.95 / 6.40 |
| G+f / NF | 1.0% | -0.30 / 0.80 | -3.20 / 4.33 | -5.73 / 6.37 |
| DropNF (live) | **0.0%** | -0.30 / 0.79 | -3.27 / 4.38 | -5.73 / 6.37 |

**adps_2026-08-25_11-58-32** (18 s, 74 detections): spikes A10 10.4% -> G+f 1.5% -> DropNF 1.6%
(one event in 14 at 0-25 m; n is tiny). Range error 0-25 m: A10 -2.54 / 1.53 -> G+f -1.67 / 0.65.

- The live rules hold on both drives: segmentation improves near-field range, the depth gate
  removes the spikes, and nothing regresses. Segmentation emptied 0 and 1 boxes respectively --
  the 3.7% seen on the selfcal drive is cone-specific, as expected.
- **Worth a look later:** on 11-50-45 every arm reads -3 to -6 m with a 4-6 m spread at 25-60 m,
  against -1.0 / 0.64 on the selfcal drive. Identical across arms, so it is a property of that
  drive or of the radar matching there (that bag's radar runs at 20.9 Hz, not 30), not of the
  selection rule.

## Camera ground-intercept: calibrated, and still not good enough (2026-09-15) -- to-do item 5

The idea was to rescue the 3.7% of boxes segmentation leaves with no object return (nearly all
cones) by ranging them from where the box's bottom edge meets the road. It reads short (-3.5 m vs
radar inside 40 m), so the question was whether a calibration fixes it.

**A box-bottom pixel offset and a camera pitch error are the SAME parameter** -- an offset of dv
pixels is dv/f radians -- so one angle covers both. Measured per detection as
`theta_ray - atan(h / r_lidar)` (scratchpad `cam_pitch.py` over the neighbour_ab dump):

| band | cones: offset (sd) | vehicles: offset (sd) | 1 px of box edge |
|---|---|---|---|
| 20-30 m | +0.86 deg (0.48) | +0.27 deg (0.57) | 0.11 m |
| 30-40 m | +0.68 (0.31) | +0.39 (0.44) | 0.22 m |
| 40-55 m | +0.68 (0.24) | +0.46 (0.30) | 0.40 m |
| 55-80 m | +0.72 (0.21) | +0.91 (0.01) | 0.81 m |

- **Cones are calibratable**: a constant ~+0.7 deg (42 px) across 20-80 m.
- **Vehicles are not**: the offset drifts -0.08 -> +0.91 deg with range. The box bottom is not the
  contact patch (shadow, image-edge truncation close in, the wheels unresolved far out).
- **But the residual spread kills it anyway.** Even with the constant removed, the remaining
  angular spread converts to **3.3 m of range error at 25 m and 10.1 m at 67 m** for cones -- far
  worse than the +-1-2 m LiDAR measurement it would stand in for, which is exactly why the
  DropNF_cam arm made the spike rate worse (1.5% -> 2.1%).

**Verdict: leave `enable_camera_only_fallback` off.** Not because the geometry is wrong -- it is
now right (extrinsic-correct, local ground height) -- but because a 2D box bottom cannot range an
object on this camera to better than ~13% of range. Revisit only with a bottom-edge regressor
better than YOLO's box, or a second camera.

## Three motion bugs and the merge rule, all found from RViz (2026-09-16)

The user watched `publish_mode:=filtered` in RViz and reported the tracks sitting BEHIND their own
detections. Measured from a 75 s recording of `/perception/objects_markers` against
`/perception/measurements/camera_lidar_markers`: the track closed at -6.43 m/s while the object
approached at -3.01 m/s, sat 0.79 m nearer than its measurement between camera frames, and carried
a median 4.1 m/s of velocity on a scene of static cones. Three separate bugs came out of it --
two in production, one in the harness -- plus a merge rule that was eating objects.

**1. The live one: the prediction clock rewound.** `_apply` set `_last_t = t` even when the
measurement arrived out of capture order (`dt <= 0`). The camera path is slower than the radar's,
so releases genuinely interleave -- `late=421` in one run -- and the next measurement then
predicted across an interval already applied. Ego motion counted twice, so every static object
drifted FORWARD at ~35% of ego speed (measured: median vx +3.78 m/s) and the box trailed its
detection between camera frames. Fix: `_last_t = max(_last_t, t)`. Live after the fix: track minus
measurement between camera frames **-0.80 m -> +0.00 m**, spurious speed **4.13 -> 0.00 m/s**.

**2. The offline one: `scripts/object_ab.py` pre-loaded a whole drive into an 8 s TwistBuffer**,
which keeps only the LAST 8 s, and `TwistBuffer.at()` served the oldest retained sample for every
earlier query -- a twist from ~200 s later -- instead of refusing. Integrated ego displacement came
out at **86.3% of v*dt**, so static objects acquired ~31% of ego speed BACKWARDS. The harness now
feeds odometry incrementally (`_odom_feeder`), and `at()` refuses a stamp more than `max_hold`
before its oldest sample, symmetric with the stale-buffer case it already handled. Displacement
ratio after: **1.0000**.

**3. The live one that fix 1 created: on a LOOPING replay the clock stopped moving at all.**
The user then reported the boxes updating far more slowly than the measurements. A diagnostic added
to the node's status line settled it in one line: **74% of measurements arrived with `dt <= 0`,
median -8.3 s**. A looping bag rewinds sim time by its own length, and `max(_last_t, t)` pins the
clock at the highest stamp ever seen, so from the second loop onward NOTHING was ever predicted --
tracks moved only when an update nudged them. Fix: a backwards jump over `REWIND_S = 1.0 s` is a
rewind (drop the tracks, clear the odometry buffer, restart the clock), while ordinary out-of-order
arrivals still cannot rewind it. Measured on the truck pass of adps_2026-08-25_11-58-32:

| | before | after |
|---|---|---|
| track vs its own measurement | 6.79 m median, 16.20 p90 | **1.46 m, 9.06** |
| track closing rate (measurement: -24.99 m/s) | **+0.00 m/s** (frozen) | **-24.22 m/s** |
| best-fit time lag | +310 ms | **+70 ms** |
| live `dt <= 0` | 277/376 camera, 828/1125 radar | **0/149, 0/446** (`rewinds=3`) |

This is also why the user saw it work while stationary and fail once anything moved: with no
prediction running, a still scene looks correct and any motion drifts immediately.

**4. The merge rule was swallowing cones.** The user reported cones missing from the aggregator
output that the measurements clearly had. `should_merge`'s second clause -- generous in RANGE, same
bearing -- exists to reconcile a camera branch sitting 14 m short of its radar branch at long range.
A line of traffic cones has exactly that signature: consecutive cones within a degree of bearing,
metres apart in range. Measured as ORPHANED measurements (a published measurement with no track
within 3 m), `scripts/object_ab.py --count-objects`:

| arm | orphaned |
|---|---|
| as it was | **30.6%** (live: 29.7%, worst 0-25 m at -31%) |
| merging off entirely (the floor) | 7.8% |
| **restricted to the cross-branch split (live)** | **17.2%** |
| association 6 -> 2.5 m | 29.1% -- not the cause |
| camera gate on | 25.4% -- not the cause |

The clause now requires the two tracks to come from DIFFERENT branches (`split_branch_only`: two
camera-backed tracks are two objects the camera saw separately) and `min_range_for_gap = 60 m`
(under 80 m the camera range is good to ~1 m, so a 20 m merge there cannot be that split). The
Mahalanobis clause is untouched. **Still 17.2% against a 7.8% floor** -- the remainder is the
Mahalanobis clause on tracks whose covariance has grown, and it is to-do item 11.

All four are regression-tested (`test_ego_motion.py`, `test_measurement_queue.py`,
`test_track_store.py`). The filter numbers below were re-measured after the ego-motion fixes and
hold: raw median 1.57 m -> filtered 1.21 m, p90 14.82 -> 12.44, jumps > 2 m 4.4% -> 2.9%, lag
+0.04 m, and 60-80 m 3.17 -> 0.76 m.

**What shipped alongside, for the next person debugging this:** the node's 5 s status line now
carries `rewinds=` and, per sensor, `dt<=0 n/N med +Xms` -- the statistic that made bug 3 obvious;
`/perception/objects_markers` draws a velocity arrow (one second of travel) for any track above
1 m/s, so passthrough draws none and a wrong velocity is visible immediately; and the RViz config
ships with the "Aggregator tracks" display enabled.

**CONTAMINATED MEASUREMENTS, do not quote them:** anything recorded live under a LOOPING replay
before bug 3 was fixed was taken with a frozen filter. That includes the live 29.7% orphan rate and
the live track-speed figures in this section. The offline numbers (`--count-objects`, `--full-ab`,
`--track-speed`) are unaffected -- the harness feeds measurements in stamp order.

## Radar matching in the harnesses, fixed (2026-09-15) -- to-do item 3

Every "range error vs radar" number in these harnesses depends on deciding which radar return is
the same object. The old rule -- nearest in azimuth within +-1 deg -- is wrong twice over:

- **A close object subtends more than the gate.** A 1.8 m car at 10 m spans ~10 deg, so its bumper
  return sits nowhere near the box centre's bearing. The gate is now the object's own half-extent
  plus the ESR's 0.5 deg: 5.6 deg at 10 m, 1.15 deg at 80 m (i.e. unchanged far away).
  `ground_ab.azimuth_gate_deg` / `object_ab.match_gate_deg`.
- **Azimuth alone matches different objects.** At 5-15 m the objects on this drive sit at 15-26 deg
  azimuth -- the edge of the camera's field of view, where radar coverage is sparse -- and pairs
  like "6.7 m truck vs 69.9 m return on the same bearing" were being scored. A deliberately WIDE
  range window (3 m + 50% of range: +-8 m at 10 m, +-43 m at 80 m) excludes those without
  truncating any error under test. Note the Phase 0 trap it must not repeat: a +-3 m gate hid the
  road-adoption failure entirely.

Effect on the measured spread (arm DropNF): 65-80 m sd **2.56 -> 1.85**, 80-100 m median
-6.58 -> -5.00 (sd 9.31 -> 6.82), mid bands unchanged (25-35 m: -1.02/0.67 -> -0.99/0.64), and
matched pairs up ~20%. `SIGMA_ALONG_TABLE` was re-measured from this cleaner matching.

**The 0-15 m band is still not trustworthy** (median -3.95 m, sd 4.94, n=76, was -5.31/6.49): those
objects are at the FOV edge where the radar barely sees. Do not quote near-field range accuracy
from radar pairs; a different reference is needed there.

## The filter, re-measured on the CURRENT measurements (2026-09-15)

The old verdict here ("the filter is worse than its input", median 3.52 vs 3.05 m) was measured
two ways that no longer describe the system:

1. on the OLD `/fused_bbox` positions, before segmentation, the empty-box drop and the depth gate;
2. through `filter_ab()`, which applies CAMERA updates only -- radar is just the ruler. It scores
   the motion model alone, not the aggregator.

`scripts/object_ab.py` now takes `--measurements <neighbour_ab --dump>` (the rule the detector
runs today, behind neighbour_ab's own self-check) and `--full-ab`, which scores raw vs filtered
inside the full pipeline, with `--radar-holdout N` and a lag statistic.

**Measurement quality, same harness, same scoring** (`--filter-ab`):

| | old /fused_bbox | current rule |
|---|---|---|
| raw median range error vs radar | 3.05 m | **1.65 m** |
| filtered median | 3.52 m (worse than raw) | 1.66 m (a tie) |
| raw / filtered jumps > 2 m | 15.3% / 12.4% | 8.2% / **6.3%** |

**The filter now earns its keep, on the honest test.** `--full-ab --radar-holdout 4` holds radar
out of one camera track in four (keyed on the ByteTrack id, so the held-out set is identical
across arms) and scores only those, n=373 paired:

| Q (sigma_long/lat) | filtered median | jumps > 2 m | lag behind raw |
|---|---|---|---|
| raw (passthrough) | 1.40 m | 4.7% | - |
| 2.0 / 1.0 | 1.30 m | **1.6%** | +0.17 m |
| 4.0 / 2.0 | 1.36 m | 2.0% | +0.04 m |
| 8.0 / 4.0 | ~1.16-1.36 m | 2.6% | +0.00 m |

Beyond 80 m the filtered state is still worse than raw in some bands (60-80 m: 1.98 -> 2.2-5.9 m
depending on Q) -- that is the far-field bias being carried forward, and the reason item 6 waits
on this one.

**Circularity, stated plainly:** with radar updates ON, the filtered state is FITTED to radar
ranges and then scored against them; it "wins" by construction (median 0.25 m vs raw 1.63 m).
That number is not evidence. The holdout above is. What independently supports radar-corrected
range is Phase 0: camera range is biased (-1.1 m under 80 m, -7.6 m at 80-100) while the ESR
resolves range to ~0.1 m.

**`sigma_cross` was the real problem, and it is now bracketed by measurement.** It was invented
at 0.30 + 0.010 r (0.5-1.2 m). Two independent estimates, both from the neighbour_ab dump:

| band | lateral spread vs radar azimuth | of which radar's own 0.5 deg | track's own lateral jitter (white noise) | old model |
|---|---|---|---|---|
| 15-25 m | 0.18 m | 0.17 m | 0.045 m | 0.50 m |
| 45-55 m | 0.30 m | 0.44 m | 0.061 m | 0.80 m |
| 65-80 m | 0.57 m | 0.63 m | 0.143 m | 1.02 m |

The radar pairs bound it (its own azimuth explains the whole spread, so the object's share is ~0
and unmeasurable with this ruler); the jitter measures the white-noise part. Model set between
them: **0.10 + 0.004 r**. With it, the held-out A/B at the SHIPPED Q (2.0/1.0):

| band | raw | filtered, old sigma_cross | filtered, new |
|---|---|---|---|
| 20-40 m | 0.93 | 0.87 | 0.89 |
| 40-60 m | 1.28 | 1.30 | **1.08** |
| 60-80 m | 1.98 | 5.91 | **0.89** |
| 80-100 m | 4.38 | 7.34 | 8.48 (radar held out; RANGE_TRUST_MAX_M drops camera range here, so in production radar supplies it) |
| overall median / p90 | 1.40 / 22.45 | 1.30 / 27.19 | **1.21 / 21.12** |
| jumps > 2 m | 4.7% | 1.6% | **1.6%** |

Re-scored once more after the matching fix (item 3), which is the number to quote -- radar held
out, shipped Q 2.0/1.0, n=425 paired: raw median **1.64** mean 5.70 p90 14.71, jumps > 2 m 4.7%;
filtered median **1.22** mean 4.81 p90 12.49, jumps **1.4%**, lag +0.12 m. By band, raw -> filtered:
0-20 1.48 -> 1.37, 20-40 0.98 -> 0.89, 40-60 1.54 -> 1.13, 60-80 3.17 -> 1.22, 100-120 16.73 -> 10.23;
only 80-100 is worse (4.38 -> 8.24), and only because the holdout removes the radar that
RANGE_TRUST_MAX_M expects to supply range there.

Cost: camera gate rejection 3.1% -> ~8% overall (19.5% at 0-20 m, where the radar-matching
artifact of to-do item 3 also lives), and the near-field under-gate fraction falls to 76%. The
innovations are a heavy-tailed mixture, so no single sigma fixes both the median and the tail --
the same conclusion Phase 2 reached for the along-ray part.

**Calibration after re-measuring `SIGMA_ALONG_TABLE`** (done today; the old table was up to 1.5x
too wide at 65-80 m, and described the superseded rule):

- radar NIS unchanged and calibrated: 94.5-96.9% under gate in every band.
- camera NIS median **0.17** against a target of 2 -- the filter still under-trusts its input. The
  along-ray part is now measured; `sigma_cross` (0.5 + 0.007 r) is still INVENTED and is the
  prime suspect. Measuring it needs the matched radar AZIMUTH stored in the neighbour_ab dump
  (radar azimuth ~0.5 deg is a weak ruler: 0.35 m at 40 m, so it bounds rather than measures).
- camera gate rejection on the current measurements: **3.1%** overall (11.6% at 0-20 m, 0% past
  80 m), against 10.6% on the old stream.

`publish_mode` still ships `passthrough`. Switching it is a user decision and should follow a live
look in RViz; the user rejected filtered before, on the old measurements.

## TO DO (prioritised; updated 2026-09-15)

Done so far, for orientation: ground removal (Patchwork++), box-jump fix (empty-box drop + depth
gate), class vote, near-field levelling, cone checks. Details in the sections above.

### 1. ~~Replay isolation check~~ -- DONE 2026-09-15
Existing pipeline bit-identical with object_fusion running, obstacle-only and under full load; see
"Isolation" above. Re-run on the VEHICLE before relying on it there:
`ISO_RADAR_FLAGS="" scripts/fusion_isolation_check.sh <bag> ~/iso_vehicle 150` is replay-shaped, so
on the car compare live runs instead (record /fused_bbox with and without the fusion stack up).

### 2. Re-tune the tracking filter -- PARTLY DONE 2026-09-15 (see "The filter, re-measured")
Done: `--measurements` / `--full-ab` / `--radar-holdout` / lag in object_ab.py, SIGMA_ALONG_TABLE
re-measured, Q swept (2/1, 4/2, 8/4 -- all beat raw on jumps; 4/2 is the balanced pick, 0.04 m lag).
Also done: `sigma_cross` bracketed by measurement and set to 0.10 + 0.004 r, which fixed the
60-80 m band (filtered 5.91 -> 0.89 m vs raw 1.98) and the tail (p90 27.19 -> 21.12 vs raw 22.45).
~~(a) show the user filtered live~~ -- done 2026-09-16; it exposed the three motion bugs above, and
after fixing them the user called the result good. `publish_mode` still ships `passthrough`; the
default is theirs to change.
Remaining: (b) camera NIS median is still 0.29 vs a target of 2 and near-field gating rejects 19.5%,
both entangled with item 3; (c) 80-100 m is worse than raw when radar is absent, by design
(RANGE_TRUST_MAX_M).
OLD PLAN, kept for context:
Why: `--filter-ab` found the filter worse than its input (median 3.52 vs 3.05 m vs radar), but it was
fed the OLD `/fused_bbox` positions. The measurements are now much cleaner (spikes 8.2% -> 1.5%), and
the filter is what would give velocity, smoother boxes and radar-owned range beyond 80 m (item 6).
1. `scripts/neighbour_ab.py --dump rows.pkl` already holds the production DropNF position per
   detection (t, tracker id, class). Add `object_ab.py --measurements rows.pkl` (arm "DropNF") so the
   tracker is driven by the live rule instead of `/fused_bbox` (constant FUSED at line ~96).
2. Re-run `--filter-ab`, `--nis`, `--rejection` on it; sweep Q (`sigma_long`, `sigma_lat`) and the
   camera chi-square gate.
3. Score raw vs filtered: range error vs radar by band, spike rate, AND lag (filtered position vs
   raw on a braking/turning object) -- smoothing that lags is not a win.
Done when: filtered beats raw on median error and spikes without visible lag; then show the user
`--filtered` in RViz and let them decide (they rejected filtered + extent before).

### 3. ~~Fix the close-range (0-15 m) scoring~~ -- DONE 2026-09-15, see "Radar matching" above
The -5.9 m was mostly a matching artifact (now -3.95 m); the band remains unusable as a
radar-referenced metric because those objects sit at the edge of the camera FOV. OLD PLAN:
Why: the -5.9 m "error" there is probably the metric: radar is matched by azimuth +-1 deg of the
object centre, but a car at 10 m spans ~10 deg, so the nearest-azimuth radar return is often not
the object. Every "0-25 m" number in ground_ab/neighbour_ab inherits this.
Plan: match a radar return whose azimuth falls inside the object's angular extent (box width
through the camera model) and whose range is nearest; re-score. Done when the 0-15 m band has a
believable error distribution, or the error is shown to be real.

### 4. Generalise the harnesses -- MOSTLY DONE 2026-09-15 (see the section above)
Done: `--source` / `--replay` on both harnesses, `scripts/record_baseline_replay.sh`, baselines for
both adps drives, and both scored (rules hold). Remaining: a drive with many CURVES and cones
(levelling still rests on 232 + 15 curve sweeps), and the -3 to -6 m at 25-60 m on 11-50-45.
OLD PLAN:
Why: everything is scored on selfcal 09-08 (+ 09-03 for levelling). 09-03 has only 15 curve sweeps;
the user's adps bags are unscored.
1. `ground_ab.py` hard-codes SOURCE/REPLAY (lines 53-54); make them `--source/--replay` flags
   (neighbour_ab imports them).
2. For each new bag, create the replay: existing stack + `ros2 bag record /fused_bbox
   /delphi_esr_interface/radar/tracks` -> `~/fused_replay_<bag>` (needs the self-check to pass).
3. Re-run neighbour_ab (box metrics) and lean_ab (ground; `--fit` on one drive, `--bags` others).
4. Ask the user for a drive with many curves and cones.
Done when: the adopted rules hold (or not) on >= 2 more drives, numbers recorded here.

### 5. ~~Calibrate the camera ground-intercept~~ -- DONE 2026-09-15: REFUTED, see the section above
Cones show a constant +0.7 deg offset (fixable); vehicles drift with range (not); and the residual
spread is 3.3 m at 25 m, 10.1 m at 67 m either way, so camera-only fill-in stays off. OLD PLAN:
Why: ground-only cone boxes are dropped (3.7%). The extrinsic-correct estimator
(`projection.camera_ground_position`) still reads -3.5 m vs radar inside 40 m.
Plan: on vehicles 10-40 m with good LiDAR, regress the intercept residual on range to separate a
box-bottom pixel offset (error ~ r^2) from a pitch error (~ r^2 too, but independent of box
height) -- e.g. fit offset_px and delta_pitch jointly; apply; re-run neighbour_ab arm DropNF_cam.
Done when: DropNF_cam spike rate <= DropNF's; only then enable `enable_camera_only_fallback`.

### 6. The 80+ m band  (after 2)
12.6% spikes and ~-6.7 m range bias remain. Refuted causes are listed above (crop, missing returns,
voxel). The planned fix is radar-owned range beyond ~80 m in the filter (`RANGE_TRUST_MAX_M`),
which only acts in `publish_mode: filtered` -- so this follows item 2.

### 7. Phase 3: 360-degree LiDAR clusters  (large)
`lidar_cluster_detector_node` from the plan. Patchwork++ already segments the full sweep in
ground_projection_node; publish its non-ground cloud and cluster that (range-adaptive eps). Ship
gated off with shadow logging; clusters may SUSTAIN tracks, never birth them (clutter).

### 8. Phase 4: radar-only track birth  (large, blocked)
Blocked on a false-alarm rate: in the camera/radar overlap, count radar-only candidates that never
get a camera detection over their life. Measure, then decide `enable_radar_only_birth`.

### 9. ~~Radar lateral lever arm~~ -- DONE 2026-09-15: NOT IDENTIFIABLE, surveyed value stands
`scripts/radar_ab.py --lever-arm` now fits both components on turning data (and reads every mcap
in a bag, not just the first -- that alone took the sample from 384 to 59 484 observations).
Pooled: ly = -0.105 +/- 0.002, lx = +3.451 +/- 0.005 m. But split by turn direction ly is
**-0.248 (left) / +0.040 (right)**: a mounting offset cannot change sign with the turn, so the
estimate is absorbing sideslip at the sensor, which has the same `w * ly` signature. A boresight
term does not absorb it (fits separately at +0.692 +/- 0.018 deg). lx is stable (3.453 / 3.642)
and matches the surveyed 3.573. `lever_arm_xy` stays zero; resolving ly needs a reference that
separates sideslip from geometry (a stationary yaw test, or the INS sideslip estimate).

### 10. Constants still marked INVENTED  (ongoing)
`grep -rn INVENTED src/object_fusion/object_fusion` (17 hits): shape-fit min points, velocity-yaw
speed, extent decay/clamp, camera sigma_px, odom hold, track_store increments and coast budgets,
tracker inflation/noise. Most are tunable with NIS or shadow logs once item 2 runs.

### 11. Cones still dropped by the aggregator  (small-medium, user-reported)
17.2% of measurements still have no track within 3 m, against a 7.8% floor with merging off (see
"Three motion bugs"). The remaining share is the Mahalanobis clause in `should_merge`: an unupdated
track's covariance grows until it swallows a neighbour.
1. Re-measure LIVE first -- the 29.7% figure predates the rewind fix and is contaminated. Record
   `/perception/objects_markers` + `/perception/measurements/camera_lidar_markers` on the cone-rich
   stretch of selfcal_loc_2026-09-08 and compute the orphan rate by band.
2. Offline, sweep the Mahalanobis gate (`chi2`, default 9.21) and cap the covariance the clause may
   use, with `--count-objects`; a track that has not been updated recently should not be allowed to
   absorb a fresh one.
3. Check the camera gate's share separately (25.4% arm) -- it is not the driver but it is not zero.
Done when: the orphan rate approaches the 7.8% floor without re-inflating the duplicate-track rate
the merge exists to control.

### 12. Aggregator positional accuracy  (small, user-reported)
The user finds the published position less accurate than the measurement in RViz. Measured, the
track sits 1.46 m from its measurement (p90 9.06) on the truck pass -- and the expected honest part
of that is the radar-vs-LiDAR surface difference (~1.1 m; the radar's scattering centre is deeper
into the vehicle than the visible face).
Plan: one pass scoring track, measurement and matched radar range together by band, so the split
between "the filter moved it toward radar, correctly" and "the filter is wrong" is visible. Reuse
`object_ab.py --full-ab` plumbing and the matching rules in `ground_ab.match_radar_range`.
Done when: the residual is attributed, and either accepted as the scattering-centre offset or
fixed via the per-sensor offset model the plan calls for (`o_s(x)` in the measurement models).

### Housekeeping
- Commit the new work (all untracked) -- user's call on branch and message.
- CHANGELOG.md / README.md are pre-existing files: document the new stack there only with the
  user's OK.
- `/perception/objects` cannot be echoed from the host: add a NEW install script for fusion_msgs
  (the existing install_host_custom_msgs.sh must not change).
- Expose the new rollback parameters (`ground_levelling`, `enable_depth_gate`, `enable_class_vote`,
  `segmentation_empty_fallback`) as env vars in docker-compose.fusion.yml / Dockerfile CMD so they
  can be A/B'd without a rebuild.
- Low priority: why turning RNR off changes labels far from the returns it filters; unused imports
  cleanup; `detection_geometry.camera_only_range` is now dead (kept, marked invalid).

### Known costs of what is live (re-check on the vehicle)
- Ground projection 12-13 ms/sweep (two Patchwork++ passes).
- Depth gate: a withheld frame makes the blue box blink for one frame (~2.5% of frames).
- Class vote: a genuine reclassification takes up to ~2 s to show.
- Ground-only boxes (mostly cones missed by the LiDAR that sweep) publish nothing.
- A bag rewind (looping replay, or switching bags) DROPS every track by design and starts over;
  the node logs it as `rewinds=`. Expect `skipped_no_odom` to track `rewinds` one-for-one (measured
  36 against 37): the first measurement after a rewind finds the odometry buffer empty and is
  skipped while it refills. Any larger ratio is a real odometry problem.
- 17.2% of measurements still have no published track within 3 m (item 11).
- Levelling evidence in curves rests on 232 + 15 curve sweeps from two drives.
- The 360 deg path (item 7) would add clutter with no semantic check outside the camera FOV.

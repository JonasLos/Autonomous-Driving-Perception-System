# object_fusion — handoff

State as of 2026-09-15. Everything here is NEW; no pre-existing repository file was modified
at any point. `git status` shows the same 12 pre-existing entries it did at the start.

## RESUME HERE (updated 2026-09-16)

**Start with "TO DO" at the bottom of this file** -- it is the prioritised plan, each item with
its steps and a done-when test.

**User constraints -- keep obeying them:** never modify a pre-existing repository file; never touch
the existing pipeline in `Custom_YOLO_ROS`; all new code lives in `object_fusion` or new files.
Runtime `ros2 param set` is fine. Measure offline before changing behaviour and show the user the
numbers. sudo needs a password -- never ask for it; `systemctl --no-ask-password reboot` works,
restarting system services (e.g. anydesk) does not.

**The work is COMMITTED** on branch **`radar_integration_and_fusion`**: `e462094` (by the user,
2026-09-16) which also carried the 12 pre-existing modifications that were already in the working
tree, then `de9a28d` (the merge distance bound, items 11/12, and the replicability work). Neither
is pushed. So the old isolation check -- "`git status` shows exactly 12 entries" -- no longer
applies; the tree is clean. Check isolation instead with
`git diff --name-only e462094..HEAD` and confirm every path is new work
(`src/object_fusion/`, `src/custom_msgs/fusion_msgs/`, `scripts/*_ab.py`, `scripts/run_fusion.sh`,
`scripts/fusion_isolation_check.sh`, `scripts/isolation_compare.py`,
`scripts/record_baseline_replay.sh`, `scripts/live_orphans.py`, `scripts/radar_pull.py`,
`docker/Dockerfile.object_fusion`, `docker-compose.fusion*`). As of 2026-09-16 the tree carries
nine modified paths, every one of them first added by `e462094` itself -- check with
`git log --oneline --diff-filter=A -- <path>`.

**Live demo** (the user watches RViz on DISPLAY=:1). The `perception-object-fusion` image is
current as of 2026-09-17 (filtered default); rebuild only after code changes.

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
(`ground_levelling`), merge distance bound (`MERGE_MAX_DIST=inf` reverts), camera-referenced
radar range gate (`RADAR_CAMERA_GATE=false` reverts). OFF by default: the 360-degree cluster path
(`ENABLE_LIDAR_CLUSTERS=true` / `--clusters` turns it on).

**Dockerfile bug, fixed 2026-09-21:** two CMD lines ended with a DOUBLE backslash, so every env var
after `merge_max_dist` was silently dropped -- `RADAR_CAMERA_GATE=false` did nothing and the launch
default merely made it look right. Both rollbacks are now verified end to end by reading the
parameter back from the running node. Check that way after adding any new env var. **`publish_mode` is `filtered`** (the user's decision, 2026-09-17; `--passthrough` or
`PUBLISH_MODE=passthrough` is the rollback). Extent estimation off (user's choice).

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
scripts/live_orphans.py       LIVE audit: does every measurement have a box on it? (items 11/12)
scripts/radar_pull.py         LIVE: is the track-vs-measurement offset the radar, or error?
scripts/install_host_fusion_msgs.sh   fusion_msgs on the host (planner bridge, ros2 topic echo)
scripts/run_planner_ab_nodes.sh       legacy tracker.py + planner bridge side by side, for the A/B
scripts/planner_objects_ab.py LIVE: planner obstacle input, legacy tracker.py vs bridge
scripts/lever_arm_ab.py       lever arm vs radial bias vs ego-yaw, from static tracks (--selftest)
scripts/read_insconfig.py     the receiver's own configured lever arms, out of a bag
scripts/radar_false_alarm_ab.py  item 8: would radar-only birth publish false alarms?
scripts/lidar_cluster_ab.py   item 7: would 360-deg LiDAR clusters keep departing tracks alive?
scripts/radar_vehicle_truth.py  item 12: radar vs camera on a vehicle -- what does the LiDAR see?
scripts/filter_scale_ab.py    item 2: does scaling the camera sigma change held-out accuracy? (no)
scripts/coast_budget_ab.py    item 10: re-acquisition gaps per sensor vs the coast budgets
scripts/coast_sweep_ab.py     item 10: what a longer coast budget costs (orphans, held-out error)
scripts/cluster_sustain_ab.py item 7: cluster path on vs off -- lifetime, rear survival, ghosts
scripts/far_orphan_split.py   why a >80 m measurement has no track: none, off-bearing, or
                              on the bearing but out of range
scripts/publish_rate.py       publish cadence and tracks per published frame, per recording
scripts/velocity_truth_ab.py  false velocity: apparent speed of STATIC objects by ego yaw rate
                              --rule scores a velocity_valid rule: false arrows kept vs real
                              motion kept (a static recording and the truck bag together)
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
- `--clusters` turns on the 360-degree LiDAR cluster path (item 7; OFF by default). It adds
  `/perception/measurements/lidar_markers` -- thin cyan boxes, one per cluster -- so what the
  path sees beside and behind the car is visible next to the objects.
- **Never replay with `--start-offset`.** `/tf_static` is only at the bag start; skip it and the
  radar frame never resolves (RViz shows the radar display red, the aggregator gets nothing).
- RViz colours: grey non-ground, brown ground, green existing `/fused_bbox`, blue new
  camera-LiDAR measurement, orange radar tracks.
- Offline Patchwork++ on the host: **`scripts/install_host_patchworkpp.sh`** (idempotent; it
  installs to `~/.local/lib/patchworkpp`, deletes the numpy 2.x it drags in — that one breaks the
  system scipy and mcap — and verifies the import). Then run harnesses with
  `PYTHONPATH=$(scripts/install_host_patchworkpp.sh --path)`. No venv — `python3.12-venv` is not
  installed and sudo needs a password.

`/perception/objects` on the host needs `fusion_msgs` there: **`scripts/install_host_fusion_msgs.sh`**
merge-installs it into `~/.local/opt/adps_custom_msgs`, the overlay `~/.bashrc` already sources
(done 2026-09-17; the planner's bridge depends on it). A new terminal can then
`ros2 topic echo /perception/objects`.

## Replicating any number in this file

Everything here was produced by a script in `scripts/`, from data that lives in **`~/fusion_data/`**
(331 MB, outside the repo, with its own README naming every file and the command that regenerates
it). None of it is an input to the running system — deleting it costs replay time, not work.

    ~/fusion_data/measurements/rows_gate2.pkl   detection dump, the offline harness's input
    ~/fusion_data/recordings/loop{A,B,C}/       the item-11 live A/B, one full replay loop each

    host Patchwork++          scripts/install_host_patchworkpp.sh
    offline, filter + tracks  scripts/object_ab.py --count-objects|--full-ab|--nis|--track-speed
                                --measurements ~/fusion_data/measurements/rows_gate2.pkl
    offline, one comparison   scripts/merge_accuracy.py <pkl>        (radar held out, edit ARMS)
    offline, point selection  scripts/neighbour_ab.py, ground_ab.py, lean_ab.py, radar_ab.py
    offline, one-off fits     scripts/cross_sigma.py (sigma_cross), cam_pitch.py (item 5)
    LIVE, objects vs boxes    scripts/live_orphans.py <recording>.mcap
    LIVE, where the offset is scripts/radar_pull.py <recording>.mcap
    planner obstacle input    scripts/planner_objects_ab.py ~/fusion_data/recordings/planner_ab
    isolation                 scripts/fusion_isolation_check.sh + isolation_compare.py
    the published write-up    src/object_fusion/docs/object_fusion_internals.html

Three rules that this work got wrong at least once each, and which silently produce confident
wrong answers:

1. **Record a FULL loop** (>= 420 s; the bag is 398 s) before comparing two live arms. A 4-minute
   window covers a different stretch each time and the counts move by 2x.
2. **Rotate `lidar_tc` -> `ego` (5.35 deg)** when comparing the measurement markers to the object
   markers. Unrotated, the aggregator is charged 6.5 m of cross-offset at 70 m: 84% "orphans".
3. **Hold radar out** (`--radar-holdout 4`) when scoring the filter against radar, or the state is
   fitted to the very reference it is graded against.

`ros2 bag record` ignores SIGINT off a terminal — always `timeout -s TERM`, or it runs forever and
leaves a 0-byte mcap.

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

## The merge gate's reach, and where the offset comes from (2026-09-16) -- items 11 and 12

Both of the user's remaining concerns, measured live over FULL replay loops so the two arms see the
same drive. The bag is 398 s; a 4-minute recording covers a different stretch each time and the
counts move by a factor of two, which is not a result. Use `scripts/live_orphans.py` on a >= 420 s
recording of `/perception/objects_markers` + `/perception/measurements/camera_lidar_markers`.

**The two marker streams are in DIFFERENT FRAMES** -- measurements in `lidar_tc`, tracks in `ego`,
5.35 deg apart. Comparing them unrotated charges the aggregator 6.5 m of cross-offset at 70 m and
reads as a 98.8% orphan rate in the far bands. `live_orphans.py` rotates; anything that does not is
measuring the frame difference.

### Item 11: the Mahalanobis clause did not bound a distance

`should_merge`'s chi-square clause measures separation in units of the tracks' covariance, and a
coasting track's covariance grows without limit. At 2 m sigma the 99% two-DOF value spans 8.6 m --
past the next cone in a line. So the clause written to collapse a track onto its own duplicate was
reaching across to the neighbour instead, which is what the user saw as cones disappearing.

Fixed with a hard bound in metres, `max_merge_dist = 2.5` (parameter `merge_max_dist`, `inf`
restores the old behaviour). The two cases it has to separate are a duplicate (under 2 m apart,
usually well under) and the next cone up the line (5 m and more on this drive).

    full loop, live            orphaned   two boxes on ONE measurement
    unbounded (as shipped)       15.7%              1.3%
    bounded at 2.5 m             10.5%              3.0%     <- ADOPTED
    bounded + assoc 6 -> 4 m      9.0%              5.6%     <- measured, REJECTED

Read the third row before repeating it: narrowing `assoc_max_dist` buys 1.5 more points of objects
and pays 2.6 points of duplicate boxes, because a detection that cannot reach its coasting track
births a second one beside it. The bound buys 5.2 points for 1.7. `ASSOC_MAX_DIST=4.0` reaches the
rejected arm without a rebuild if it ever looks worth re-testing.

Also measured and rejected: **tightening the chi-square** (9.21 -> 4.0) instead of bounding the
distance. It looks good offline (orphans 16.6% -> 11.1%) but it switches merging OFF -- the node
logged `merged=0` over an entire replay -- and duplicates went to 5.6%. With the bound in place the
same node logs ~100 merges per replay loop, so the rule is alive rather than quietly disabled;
`merged=` in the 5 s status line is the one-line check for that (read it a few minutes in, it is
cumulative and a freshly restarted node legitimately shows 0).

Offline the picture agrees and is cheaper to re-run
(`scripts/object_ab.py --count-objects --measurements <dump>`): orphaned 16.6% -> 8.7%, against a
10.1% floor with merging off entirely. Note the offline duplicate column BARELY MOVES across these
arms (6.2-6.8%) where live it triples -- the offline metric counts tracks within 2 m of each other,
which on a cone drive counts two genuine cones. `live_orphans.py` reports both, and the honest one
is TRUE DUP, two tracks whose nearest measurement is the same measurement.

Held out from radar (one track in four, `scripts/object_ab.py --full-ab --radar-holdout 4`), the
bound costs nothing measurable: median range error 1.10 -> 1.07 m, p90 15.06 -> 14.03, jumps > 2 m
3.3% -> 3.2%, lag +0.13 -> +0.08 m.

### Item 12: the offset is the radar, and only for tracks the radar touched

`scripts/radar_pull.py` splits the published tracks by whether a radar return exists on the same
bearing, and asks where the track sits on the camera -> radar segment.

    tracks with NO radar return   |along-ray offset| median 0.12 m   <- the filter on its own
    tracks WITH a radar return    track sits +0.91 m behind the camera measurement
                                  the radar itself sits +0.32 m behind it
                                  51% of tracks lie BETWEEN the two sensors

So the filter is not adding error of its own: left alone it sits 12 cm from its measurement. The
offset the user sees is the radar disagreeing with the camera about where the object is, and the
filter splitting the difference -- which is its job. The tail is the real risk: the p75 of the
camera-radar gap is +6.5 m, i.e. the return on that bearing is sometimes a DIFFERENT object behind
the target, and 19% of tracks sit beyond the radar rather than between the two.

Pick the radar return by BEARING, never by nearest range: picking the return closest in range to
the camera measurement forces the measured gap to zero and answers the question with its own
assumption (it reported the radar 0.06 m from the camera; by bearing it is 0.32 m).

## Feeding the planner (2026-09-17)

The local planner (`~/planner/src/AVA_Local_Planner`, its own repo) now has launch files that take
obstacles from this stack. **The full write-up lives in the planner repo:
`~/planner/docs/object_fusion_integration.md`.** In short:

- `fusion_object_bridge.py` (planner repo) replaces `tracker.py`: `/perception/objects` +
  `/novatel/oem7/odom_grid` -> `ObjectList` on `/planner/tracked_objects`, in the format the planners
  already parse. New launch files `tracker_planner_refGenerator_lane_fusion.launch.py` and
  `gps_obj_tracker_fusion.launch.py` use it. The second enables obstacles in the GPS follower,
  whose original has them commented out; that was the user's call. The originals are the rollback,
  and every existing planner file is untouched except three lines in `CMakeLists.txt`.
- Replay A/B, one full loop, both fed from the same replay at the same time
  (`scripts/planner_objects_ab.py`):

      latency error (along-track error vs ego speed)   legacy +99 +/- 5 ms    bridge +16 +/- 22 ms
      planner's kalman_predict reads object > 3 m/s    legacy 44.9%           bridge 5.8%

  The second row is the one that matters for safety. The drive is almost all static cones and parked
  cars, and the FSM follows a "moving" object instead of avoiding it.
- **Found and fixed here on the way:** the aggregator published COASTING tracks as TENTATIVE, and
  never set `missed_updates`. An old camera-only track that misses one cycle cannot be re-promoted
  (`may_confirm` stops after 5 camera opportunities), so most long-lived cones were going out
  TENTATIVE, and a consumer that drops tentative tracks would have dropped them.
  `Track.published_status()` / `Track.missed_updates()`, with tests.
- **Open (to-do 13):** where the LiDAR sits relative to the `/odom_grid` position. Three sources
  disagree: 2.39 m forward (what `tracker.py` uses and the bridge defaults to), 0.66 m (vendor
  IMU extrinsic, if the INS reports at the IMU), and about 3.0 m (static objects seen from opposite
  headings on the replay, 9 pairs only). The legacy tracker has the same uncertainty.

**Replay trap, hit this session: exactly ONE bag player.** A player the user had started was still
running when another was launched. The two interleaved camera/LiDAR/radar from points 95 s apart
and both published `/clock`: 90% of the bridge's messages found no odometry, and a ROS-time timer
fired every 10 ms. `pgrep -fa 'bin/ros2 bag play'` before starting one, and when the bridge's
`no_pose` count climbs, suspect this first.

## False velocity arrows: the turning error (2026-09-21, IN PROGRESS)

The user saw more velocity arrows once the cluster path was on. Measured on two full-loop
recordings of the selfcal drive, where every object is static so **every arrow is false**:

    ego yaw rate      arrows (> 1 m/s)  clusters OFF / ON
    < 0.02 rad/s            6.0% / 4.0%      correlation of apparent speed with yaw rate +0.36
    0.05-0.10              33.9% / 28.3%                                with ego speed   -0.07
    > 0.10                 40.6% / 33.9%

**The clusters are not the cause** -- they lower the rate in every bin. They raise the COUNT of
arrows because tracks now survive beside and behind the car (side sector 416 -> 1427 published
objects, rear 0 -> 1460), which is exactly where turning puts a passed object. The side sector was
always the worst place for this: 47.1% arrows before clusters existed.

**Where the error is: the MEASUREMENT, not the filter.** Ego-compensate consecutive detections of
the same ByteTrack id (production `TwistBuffer.increment`) and the residual scales with yaw rate:

    median apparent speed between frames   straight 0.77 m/s   turning 2.99 m/s
    split                                  along ray 0.34 -> 1.50, across ray 0.56 -> 1.92

Across-ray dominates, which is what a BEARING error predicts: while the vehicle yaws, an object's
bearing sweeps and the set of LiDAR returns inside its 2D box changes between frames, so the
nearest-depth-cluster median moves. The filter then reads that as velocity, faithfully.

**Four candidate causes measured and REFUTED** (`scripts/velocity_truth_ab.py`, which runs the
production filter over the same measurements with one thing changed):

1. Missing IMU->LiDAR lever arm in the prediction (the aggregator passes none): turning bin
   26.8% -> 27.4%, and x2/x4 arms are worse. Not it.
2. Odometry timing offset: swept +-80 ms against the measurement jitter directly -- turning stays
   3.25-3.56 m/s with no minimum. Not it.
3. Un-deskewed LiDAR sweep. The cloud is NOT deskewed (`time` field spans -0.0995..0 s), and
   `scripts/neighbour_ab.py --deskew` undoes it properly, but the gain is small: turning jitter
   2.99 -> 2.83 m/s, though the yaw-rate correlation halves (+0.115 -> +0.048). Real but minor;
   keep the flag, do not adopt on this evidence alone.
4. Widening the cross-ray sigma with bearing rate (k * omega * r * frame period) is the WRONG
   lever and is non-monotonic: k=0.5 gives 22.5% but k=1 gives 40.3%. Wider R also widens the
   chi-square gate, so outliers that were rejected get admitted and move the state further.

**FIXED 2026-09-22, live by default.** The measurement cannot support a velocity while the bearing
sweeps, so the stack no longer claims one. Nothing in the ESTIMATOR changed -- the uncertainty the
filter cannot see is added where velocity is PUBLISHED:

- `tracker.turn_velocity_sigma(omega, range)` = `|omega| * range` (k = 1.0), the velocity error a
  sweeping bearing produces, added to the published `velocity_covariance`;
- `tracker.velocity_is_significant(v, P_vv, extra)` is a chi-square test of the velocity against
  ZERO, and `velocity_valid` now means "distinguishable from standing still" rather than "the
  filter is running and odometry exists";
- cluster updates are position-only (`kalman_update(..., position_only=True)`, which zeroes the
  velocity rows of the gain): a cluster sits at the footprint centroid while the camera measures
  the near face, so the difference between consecutive clusters is an offset, not motion.

Measured live over a full loop of the static drive, where every arrow is false:

    arrows drawn, by ego yaw rate      straight  0.02-0.05  0.05-0.10   turning    ALL
    before, clusters off                 6.0%      14.7%      33.9%      40.6%    13.1%
    before, clusters on                  4.0%       9.7%      28.3%      33.9%    10.1%
    AFTER                                0.4%       1.3%       0.0%       0.0%     0.4%

and the two things that had to survive did:

    real motion (truck bag, objects > 5 m/s, velocity_valid)   100% -> 96.9%
    cluster sustain (tracks past 90 deg, median life)          6 @ 9.75 s -> 5 @ 10.1 s

k = 1.0 was chosen by measurement, not taste: k = 2 drops real motion to 58%, k = 0.5 leaves 22.5%
of the turning arrows. The remaining 0.4% are straight-line measurement jumps, not turning.

**Nothing else moved** (checked 2026-09-22, both ways it could have):

- offline, `scripts/object_ab.py --full-ab --radar-holdout 4 --measurements rows_gate2.pkl` run on
  this commit and on its parent is BYTE-IDENTICAL. That is the expected result and worth stating
  plainly: the estimator never sees any of this. `kalman_update` still defaults to
  `position_only=False`, and the covariance inflation and the significance test both happen in
  `_publish`, after the state is final;
- live, `scripts/live_orphans.py` over full loops of the shipped build: orphaned 7.8% / 9.0%,
  TRUE DUP 2.6% / 2.5%. Those two numbers are the SAME build on two consecutive loops, so read the
  1.2-point gap as the run-to-run spread of this metric before reading anything into a comparison.

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

`publish_mode` now defaults to `filtered` -- the user switched it on 2026-09-17 after watching it
live on these measurements (they had rejected filtered once before, on the old ones).

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
after fixing them the user called the result good, and on 2026-09-17 made `filtered` the default.
~~(b) camera NIS median 0.29 vs a target of 2~~ -- EXPLAINED 2026-09-18, not a defect to fix.
Re-measured on the current stack: camera NIS median 0.18 (a consistent 2-DOF filter shows 1.39)
but mean 9.0 (target 2) -- a heavy-tailed mixture, as on record. `scripts/filter_scale_ab.py`
scaled the camera's sigma_along by 1.0/0.7/0.5/0.35 with radar held out:

    median 1.07-1.12 m, p90 14.03 m, jumps 3.2-3.3%, in EVERY arm; lag +0.08 -> +0.01 m
    camera NIS median 0.063 -> 0.064 even at x0.35

The NIS does not respond to the camera's R because the innovation covariance is dominated by the
TRACK's own P, which the deliberately generous process noise (sigma_long 2 / sigma_lat 1,
chosen for near-zero lag) keeps wide. Making NIS consistent means shrinking Q, which the Q sweep
already showed trades lag (+0.23 m at 0.5/0.25) for a small median gain. Consequence to know: the
PUBLISHED covariance overstates uncertainty in the typical case -- conservative, and nothing
downstream (the planner's ObjectList) reads it today. Tightening sigma_cross (x0.7) made jumps
worse (3.2% -> 4.6%), so leave both scales at 1.0.
(c) 80-100 m is worse than raw when radar is absent, by design (RANGE_TRUST_MAX_M) -- see item 6.
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

### 4. Generalise the harnesses -- MOSTLY DONE (2026-09-15, extended 2026-09-17)
Done: `--source` / `--replay` on both harnesses, `scripts/record_baseline_replay.sh`, baselines for
both adps drives, and both scored (rules hold).

2026-09-17, out of sample on **adps_2026-08-25_11-55-43** (150 s, never scored before; it and three
more sit in `/media/avalocal/1.0 TB Disk/perception_eval_debug/`, only two of the six were ever in
the repo root). Baseline in `~/fusion_data/replays/adps_1155`, dump in
`~/fusion_data/measurements/rows_adps1155.pkl`:

    spikes > 2 m (second difference of one id)   A25 (old fusion_node rule)  6.1%
                                                A10                         5.9%
                                                G+f (segmentation + cut)    4.8%
                                                DropNF (LIVE)               2.6%
                                                Dhold (cluster switching)   2.4%

The adopted rule holds: 2.3x fewer spikes than the rule the existing pipeline runs, on a drive it
was never tuned on. Far-field content is the same as the reference drive (6.8% past 80 m, 33
detections past 100 m), so it does not unblock item 6.

**Harness bug found and fixed by this drive.** `neighbour_ab` scored the DropNF arm with a SECOND
implementation of the depth gate and asserted the two agreed. On this bag they diverged: once a
frame is dropped the two gates' miss counters and histories are in different states and nothing
resynchronises them, so the reference is not a check, it is a race. The arm now calls the
production `DepthJumpGate` directly; `DepthGate` remains for the Dhold arms, which are a genuinely
different rule (they may switch cluster). This is why the assert never fired on the reference
drive and fired on the first new one.

**adps_2026-08-25_14-13-42 is simply EMPTY, not broken** (settled 2026-09-18): stamps line up
across the full 122 s (1218 sweeps, 1209 /fused_bbox), but only 5 /fused_bbox messages contain any
detection at all -- 4 people and a bench. Nothing to score.

The last two unscored drives, 2026-09-18 (baselines `~/fusion_data/replays/adps_{115215,120223}`):

    spikes > 2 m        A25 (old rule)   DropNF (live)
    adps 11-52-15 (53 s)     15.6%            2.8%
    adps 12-02-23 (34 s)      9.7%            7.1%     <- 2898 detections in 34 s

So the adopted rules hold on every adps drive with content (11-50-45, 11-55-43, 11-58-32, 11-52-15,
12-02-23). 12-02-23 is the exception worth a look: a dense scene (~85 detections a second) where the
depth gate recovers much less (9.7% -> 7.1% against 6.1% -> 2.6% elsewhere). Broken down
2026-09-18: it is a car park (2365 cars, 533 trucks); 100% of its remaining spikes are ALONG the ray
and they concentrate on cars at 40-60 m (11%) and 60+ m (30%) -- OCCLUSION: a nearer parked car
overlaps the target's 2D box and the nearest-depth-cluster rule ranges the occluder. The depth gate
withholds one frame, but the occlusion persists, so the second disagreement is accepted. Same
occlusion is why the radar gate costs more there (item 12). An occlusion flag from the detector
(several depth clusters in one box) would serve both.

Remaining: a drive with many CURVES and cones (levelling still rests on 232 + 15 curve sweeps),
and the -3 to -6 m at 25-60 m on 11-50-45.
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

### 6. The 80+ m band  -- BLOCKED ON DATA, not on ideas (2026-09-17)
12.6% spikes and ~-6.7 m range bias in the raw measurement. Refuted causes are above (crop, missing
returns, voxel). The planned fix, radar-owned range past `RANGE_TRUST_MAX_M`, is live now that
`filtered` is the default -- and measuring it is where this stops.

**What the reference drive can and cannot say.** Only **6.8% of its camera detections are past 80 m**
(287 of 4236; 52 past 100 m, none past 150 m). Held out from radar, that leaves 25-33 scored
samples per band out there. On that sample:

- 80-100 m, radar held out: filtered **0.10 m** median range error against radar, raw **2.97 m**.
- `range_trust` swept at 80 m (live), 120 m and "never drop" is **indistinguishable** -- identical
  to two decimals in every band. With 287 far detections in the whole drive there is nothing for
  the rule to act on, so this is not evidence that the rule works, it is evidence the drive cannot
  test it.

**What IS solid, from the live recording (`scripts/support.py`-style provenance counts over
~1.2k far observations):** past 80 m the camera range is dropped 100% of the time by design, and
**radar has never touched 41% of tracks at 80-100 m and 57% at 100-150 m**. For those, "radar owns
the range" means nothing owns it: the track keeps the range it was born with, corrected only by ego
motion, until the object comes inside 80 m. That is the mechanism to fix, and the candidates are
(a) a wide-sigma along-ray update instead of dropping it, (b) a measured range-bias correction
applied past 80 m, (c) leave it and accept that far objects are bearing-only until radar sees them.

**Also settled here:** the 30.5% orphan rate past 80 m (item 11's remainder) is mostly the metric,
not lost objects. Out there the filtered range and the camera measurement are MEANT to disagree --
raw is 2.97 m from radar where filtered is 0.10 m -- and the orphan test calls anything over 3 m a
miss. Re-read that band with a range-aware threshold before treating it as a defect.

Done when: a drive with real far-field content (a highway run, or anything with vehicles held at
80-150 m for tens of seconds) exists, and (a)/(b)/(c) are scored on it with `--radar-holdout`.

### 7. Phase 3: 360-degree LiDAR clusters  -- MEASURED 2026-09-18: small gain on current data
Plan unchanged: clusters may SUSTAIN tracks, never birth them. Stage 1 is built and tested:
`object_fusion/lidar_clusters.py` (voxelise 0.2 m, range-adaptive gap 0.45 + 0.012 r, reject walls
and flat patches; 8 tests). Stage 2 measured whether it is worth a node
(`scripts/lidar_cluster_ab.py`): for each camera-track death on selfcal_loc_2026-09-08, propagate
the object with ego motion and look for a cluster in the following sweeps, with the same test at
the MIRRORED position as a chance control.

    track deaths                      n    kept alive >= 0.5 s     chance (mirror)
    after leaving the camera view    96    26.0%                   0.0%
    dying inside the view            76     3.9%                   0.0%

    how long it keeps them: median 0.7 s, p90 1.3 s; the full 3 s window only 4%
    cost: 37.8 ms/sweep for Patchwork++ + clustering (the clustering alone is ~20-25 ms, since
          ground_projection already runs Patchwork++); 41 clusters/sweep, 39 outside the camera

Live on the selfcal replay the node confirms it: 47 clusters/sweep, 22.3 ms mean, p90 27.4 ms
against a 100 ms sweep budget. MEASURE THAT AT REST -- running one of these offline harnesses on
the same machine triples the figure (70.9 ms, p90 96.0), which reads as a node that cannot keep up.

**That first reading was PESSIMISTIC, and the reason is instructive (re-measured 2026-09-21).**
It required CONSECUTIVE sweeps from the moment of death -- and an object that leaves the camera
dies in the SIDE sector, which is the worst-covered place around the car. `--coverage` follows each
static object for 10 s past its last publication and asks where a cluster actually is:

    sector                 2.5-15 m    15-40 m    40-60 m     (mirror = chance)
    ahead  |bearing| < 30    62.7%      58.8%      52.7%       0.0 / 4.2 / 30.9%
    side   30-150            40.7%      22.0%      69.0%       0.2 / 11.0 / 0.0%
    behind > 150             52.7%      64.7%      69.5%       0.0 / 1.0 / 0.7%

The REAR sector is the best covered of all. The side sector is the roof LiDAR's blind zone, and a
passed object crosses it on its way to the rear. Tolerating that gap (`--max-miss 5`, half a
second) changes the answer:

    departing tracks held   25.0% -> 45.1%      (chance 0.0 -> 1.1%)
    for how long            0.7 s -> 4.3 s median, p90 5.4 s   (10 s window, none reach it)

**But only for STATIC objects. Moving ones sit at chance (5.1% vs 5.1%).** Read that with its
caveat: this harness PROPAGATES a dead track at constant velocity and never updates it from the
clusters, which is accurate for a static object (ego motion is known) and drifts within a second
for a moving one -- so it understates what a real implementation, which would lock onto the
cluster each sweep, can do. Only a filter-in-the-loop test settles moving objects.

**BUILT 2026-09-21**, after the user confirmed the planner needs side and rear objects for
avoidance and lane changes ("once we pass an object it will drop out of the tracker otherwise").

    /perception/nonground            ground_projection, publish_nonground:=true -- the full-sweep
                                     non-ground cloud, segmented BEFORE transform.py's crop
      -> lidar_cluster_detector_node -> /perception/measurements/lidar (Detection3DArray, SENSOR_LIDAR)
      -> object_aggregator._apply_lidar_clusters   SUSTAIN ONLY, never birth

One flag turns the whole chain on: `ENABLE_LIDAR_CLUSTERS=true` (`scripts/run_fusion.sh --clusters`),
default OFF until it has been watched on the vehicle. The association is
`lidar_clusters.associate_clusters`: gate 1.0 + 0.02 r, exclusive assignment (one cluster cannot
feed two tracks, which is how a duplicate becomes self-sustaining), and a miss is only charged where
a cluster could have been seen (2.5-60 m).

First live A/B, adps_2026-08-25_11-58-32, same bag and settings apart from the path:

    tracks 20 -> 21 (no ghost inflation)   lifetime p90 and max 4.89 s -> 7.79 s
    cost: 81 clusters/sweep, 29-31 ms (p90 45), on top of ground_projection's existing Patchwork++

The longest-lived tracks nearly doubled and the count barely moved -- but ~20 tracks is
supporting evidence, not proof.

**Full-loop A/B on selfcal, 2026-09-21** (`scripts/cluster_sustain_ab.py`, two recordings of
/perception/objects from the same drive, same build):

    clusters OFF   tracks 337 | lifetime median 0.31 s p90 4.56 | past 90 deg   0 | tail p90 0.92 s
    clusters ON    tracks 308 | lifetime median 0.31 s p90 4.96 | past 90 deg   6, median life
                                                                   9.75 s | tail p90 1.30 s, one > 10 s

Six objects the car had passed stayed tracked for ~10 s into the side and rear sectors, where
NONE survived before. Fewer tracks overall, and the ghost tail (time published after the camera
last saw the object) stays around a second.

**Two bugs found by this A/B, both worth remembering:**

1. **Charging an existence MISS for a missing cluster is wrong** and was removed. The coverage
   measurement says a cluster is found on an object that is certainly there only 40-70% of the
   time, so "no cluster" is not evidence of absence. At -0.2 log-odds per sweep it is -2.0 a
   second, which killed tracks wholesale and made the camera re-birth them.
2. **The metric nearly rejected a working feature.** `cluster_sustain_ab.py` first required >= 5
   observations per track. Sustaining a weak track PAST that threshold adds it to the sample as a
   short-lived one, so the count rose (194 -> 294) and the median lifetime fell (2.07 -> 0.31 s)
   while the truth was the opposite. Count every track, and read count and lifetime together.

**What the path costs the rest of the stack, measured 2026-09-22** (`scripts/live_orphans.py`,
three full loops, same build and same bag, only `ENABLE_LIDAR_CLUSTERS` differing):

    arm                      orphaned   TRUE DUP   published   80-200 m band
    clusters OFF               9.7%       3.0%      39.7 Hz       15.1%
    clusters ON, loop 1        7.8%       2.6%      49.6 Hz       19.9%
    clusters ON, loop 2        9.0%       2.5%      49.6 Hz       21.2%

The path pays for itself where it works -- more tracks published (mean 1.45 per frame against
1.23), fewer orphans, fewer true duplicates -- and it costs about 5 points in the 80-200 m band,
which is 10-14 measurements a loop that end up with no track on their bearing at all
(`scripts/far_orphan_split.py`: "nothing on the bearing" 22 -> 31-33 of 292).
That is NOT the velocity fix (the same loops with clusters off read 15.1%, as does loopD from
before either feature) and it is outside the 1.2-point run-to-run spread. It is unexplained:
clusters stop at 60 m, so nothing out there is cluster-updated. Next thing to check is whether a
cluster-sustained track is merging with, or stealing the association of, a far camera-only track
(`merge_max_dist` 2.5 m, `assoc_max_dist` 6.0 m) -- run the arms above with merging off.

Watch items for the vehicle: the one track with a cluster-only tail over 10 s, and a p99
per-message step of 3.3 m with clusters on (1.4 m without) -- a track occasionally jerks onto a
cluster. Moving objects are still unmeasured here (the cone drive has none).

### 8. Phase 4: radar-only track birth  -- MEASURED 2026-09-18: blocked on traffic, keep it OFF
`scripts/radar_false_alarm_ab.py` follows each ESR object by track_id (split on gaps and on range
jumps, because ids recycle), applies the production radar-only confirmation (>= 8 observations,
>= 80% moving over the ground), and asks whether the camera ever saw it where the two overlap.

    drive (minutes)          would be born, in view   never seen by camera   born outside view
    selfcal 09-08 (6.6)                 0                     -                      1
    adps 11-55-43 (2.5)                32                  62.5%                     6
    adps 11-58-32 (0.3)                 4                  25.0%                     3
    adps 11-50-45 (0.9)                 0                     -                      0
    pooled (10.3)                      36                  58%  (21)                10

- The stationarity gate is doing its job: 91.5% of STATIONARY radar objects in view are never seen
  by the camera (~700 lives a minute on selfcal) and none of them can be born.
- Among movers that would be born, the unconfirmed ones are NOT compensation leakage from turning
  (0% of them during turns; same ~3.5 m/s as confirmed ones). What differs is lifetime: 1.9 s
  against 4.6 s for the confirmed.
- 58% is an UPPER bound (YOLO's missing classes, occlusion), but n = 36 cannot set a gate, and it
  is small because these drives have almost no moving traffic -- selfcal has none at all.
Keep `enable_radar_only_birth` false. Re-run on a drive with real traffic; if the rate stays high,
the lifetime difference is the first thing to try as a gate (it costs birth latency).

### 9. ~~Radar lateral lever arm~~ -- DONE 2026-09-15: NOT IDENTIFIABLE, surveyed value stands
`scripts/radar_ab.py --lever-arm` now fits both components on turning data (and reads every mcap
in a bag, not just the first -- that alone took the sample from 384 to 59 484 observations).
Pooled: ly = -0.105 +/- 0.002, lx = +3.451 +/- 0.005 m. But split by turn direction ly is
**-0.248 (left) / +0.040 (right)**: a mounting offset cannot change sign with the turn, so the
estimate is absorbing sideslip at the sensor, which has the same `w * ly` signature. A boresight
term does not absorb it (fits separately at +0.692 +/- 0.018 deg). lx is stable (3.453 / 3.642)
and matches the surveyed 3.573. `lever_arm_xy` stays zero; resolving ly needs a reference that
separates sideslip from geometry (a stationary yaw test, or the INS sideslip estimate).

### 10. Constants still marked INVENTED  -- the LIVE ones are now measured (2026-09-18)
Measured and relabelled:
- **Coast budgets** (`MAX_COAST_S`). The re-acquisition gaps are real -- 13-28% of camera
  re-acquisitions arrive after the 0.5 s budget, p95 0.7-1.7 s across four drives
  (`scripts/coast_budget_ab.py`) -- but the comment's own method, "take the 95th percentile", is
  WRONG: `scripts/coast_sweep_ab.py` shows 1.5 s raising orphans 10.6% -> 13.3% and the held-out
  range error 1.07 -> 1.58 m, because the stale coasting track drifts off the detection it was kept
  for. 1.0 s is within noise of 0.5 s on every metric. **Kept at 0.5 s.**
- `assoc_max_dist` (6 m vs 4 m, full-loop A/B, item 11) and the process noise Q (swept, item 2).
What is still INVENTED belongs almost entirely to features that are OFF, so it is inert today:
shape-fit min points / velocity-yaw speed / extent decay (extent estimation), camera sigma_px
(camera-only fill-in), the track_store log-odds increments (tuned for radar-only birth). Left:
`MAX_POSITION_TRACE` (a divergence guard that rarely fires), `SIGMA_ALONG_FLOOR` (below the table's
20 m first row), the radar azimuth inflation (radar NIS median 0.55 against 2.37 for 3 DOF says R is
generous; not measured further), and the odometry hold.

### 11. ~~Cones still dropped by the aggregator~~ -- DONE 2026-09-16, see the section above
The Mahalanobis merge clause did not bound a distance; `merge_max_dist = 2.5` bounds it. Live over
a full replay loop, orphaned measurements 15.7% -> 10.5% for 1.7 points of duplicate boxes.
~~80-200 m reads 30.5% orphaned~~ -- a METRIC artefact, fixed 2026-09-18. Past 80 m the filter
drops the camera's range, so the track and the camera measurement are supposed to disagree along
the ray; the 3 m test counted that. `live_orphans.py` now counts a far measurement as covered when a
track sits on its bearing (within 3 m across the ray, 35% of range along it). Re-read on loopC:

    80-200 m    30.5% -> 12.7%   (in line with 7-11% in every nearer band)
    overall     10.5% ->  9.3%

Nothing left here that is not item 6's (a far-field drive).

### 12. ~~Aggregator positional accuracy~~ -- ATTRIBUTED 2026-09-16, see the section above
It is the radar, not the filter. A track with no radar return on its bearing sits 0.12 m from its
camera measurement; one the radar has touched sits 0.91 m behind it, and the radar itself is 0.32 m
behind it, with 51% of tracks between the two sensors.
WHAT IS LEFT, measured 2026-09-18 (`scripts/object_ab.py --radar-assoc`): every APPLIED radar
update, compared with the same track's camera range from the same moment.

    camera range   updates   radar BEHIND the camera by > 3 m   track left > 3 m off its camera
       0-40 m        931          0.0%                               0.5%
      40-60 m        583         14.8%                              16.0%
      60-80 m        380         17.6%                              34.2%

    40-80 m, by class:   cones 5.2% radar-behind      everything else (vehicles) 38.9%

So association is clean under 40 m, and the 40-80 m disagreement is VEHICLES, not cones -- the
obvious hypothesis (a weak-reflecting cone picking up the strong object behind it) is REFUTED.
Two things tried and dropped:
- A per-track range cap of 3 sigma_along under 80 m: 29239 -> 29227 radar updates, 60-80 m
  unchanged. The disagreement sits inside 3 sigma, and once a track has been pulled toward a
  return, the next returns agree with the TRACK. Removed from production code, not shipped.
- The class split above, which is what killed the cone idea.

**Settled the same day, and the radar was WRONG** (`scripts/radar_vehicle_truth.py`). For each of
the 119 vehicle updates at 40-80 m with radar > 3 m behind the camera, the non-ground LiDAR points
within +-1.5 deg of the radar's bearing were split into range segments:

    the vehicle at the CAMERA range, nothing at the radar range (radar ghost)   99   83.2%
    separate objects at both ranges (radar on something BEHIND the target)      15   12.6%
    nothing at the camera range (the camera was wrong, radar correcting it)      2    1.7%
    one object spanning both (radar ranging deeper into a bus)                   1    0.8%

So radar was pulling vehicle tracks 3+ m too far away, mostly on multipath ghosts. **FIXED, live
by default: `radar_camera_gate`.** A radar return is refused when it sits further from the track's
RECENT camera range (within 0.3 s) than 3 sigma_along -- only inside RANGE_TRUST_MAX_M, so past
80 m radar still owns range (`tracker.camera_radar_range_cap`, `associate_radar(camera_ref=...)`).
The earlier cap against the TRACK's predicted range did nothing because radar had already pulled
the track; referencing the camera breaks that loop.

    offline, reference drive          LIVE (no gate)     radar_camera_gate
    radar > 3 m behind, 40-60 m          14.8%               0.0%
    track pulled > 3 m off camera, 40-60 m   16.0%            0.4%
    vehicles 40-80 m pulled > 3 m        39.2%              12.7%
    LiDAR-confirmed bad updates           119                 27   (25 ghosts inside 3 sigma at 60-80 m)
    radar updates applied               29239              29056   (-0.6%, none lost under 25 m)

Cost: the 2 genuine corrections are blocked too -- 2 lost to stop ~92 wrong pulls. 3 sigma was
kept, not tuned down to the 27 leftovers. Rollback: restart with `RADAR_CAMERA_GATE=false` (it is
read at start-up, so `ros2 param set` does not change it live).

**Its cost is scene-dependent -- measured on a dense car park (adps 12-02-23, all cars), where the
nearest-depth-cluster rule often ranges an OCCLUDING car:**

    126 disagreements at 40-80 m     radar ghost 50 | radar on a car behind 38 | CAMERA WRONG 20 | neither 18
    with the gate                    3 left

So there it blocks 88 bad updates and 20 GOOD ones (the radar correcting an occluder-ranged camera):
4.4 : 1, against ~46 : 1 on the reference drive. Still net positive, so it stays on, but in crowds
it is not free. The refinement that would keep those 20: the detector knows when a box holds more
than one depth cluster (occlusion); flag such measurements and do not use them as the gate's
reference. Not built -- it touches the detector-to-aggregator message.

Live check, one full loop with the gate on (`~/fusion_data/recordings/loopD`) against loopC
(identical settings, gate off): the node runs normally and nothing got worse. The MEDIAN offset of
radar-touched tracks barely moves (+0.87 -> +0.84 m) -- expected, since the gate acts on the
40-80 m tail and the median is dominated by near objects. Orphans 9.3% -> 7.9% and true duplicates
3.0% -> 2.6% also improved, but detector run-to-run variation is of that size, so they are not
credited to the gate. The evidence for it is the offline table above, which runs the production
association code.

### 13. LiDAR position relative to the odometry output point  (user decision pending)
The planner bridge's `lidar_offset` ships at (2.393, 0.206) m -- the value `tracker.py` has always
used. **The evidence now says that is wrong by about 1.7 m, and the answer is near (0.67, -0.10) m**,
but the one thing that would measure it directly is not in any bag we have.

1. **The receiver settles the reference POINT.** `/novatel/oem7/insconfig` in
   selfcal_loc_2026-09-08 reports `number_of_translations: 0` and `number_of_rotations: 0` -- no
   lever arms configured at all, so no USER output point, so the INS position is at the IMU centre.
   (It also means the IMU->antenna arm is unset, which is the receiver's own accuracy problem, not
   ours.) Read it with `scripts/read_insconfig.py`.
2. **The vendor extrinsic then gives the arm.** `imu` sits at (-0.658, +0.159) in lidar_tc, so the
   LiDAR is 0.658 m AHEAD of the IMU; rotated into vehicle axes that is **(0.670, -0.097) m**.
   Status in `~/jeep_selfcal_loc` is "not measured" -- vendor value, never independently checked --
   and the radar chain agrees with it trivially rather than independently (IMU->radar 3.573 minus
   lidar->radar 2.915 = 0.658).
3. **The drive cannot measure it, and says so.** `scripts/lever_arm_ab.py` fits the lever arm, a
   radial range bias and a residual ego-yaw together, because on one pass all three move a static
   object the same way. Its `--selftest` recovers a known arm exactly when tracks sweep 70 deg of
   bearing. On the replay the median track sweeps 21 deg, the design condition number is 39 752,
   and bootstrapping over TRACKS gives de_x in [-4.21, -1.78] m: 2 m wide, but it EXCLUDES the
   2.393 m in use and brackets 0.67. `--profile` sweeps the offset alone and is nearly flat --
   scatter 0.744 m at -0.5 m against 0.774 m at 2.5 m, 4% over a 3 m sweep -- rising monotonically
   with larger offsets. A lean, not a measurement.

**WITHDRAWN:** the "about 3.0 m" from `planner_objects_ab.py`'s opposed-heading check. A radial
range bias flips sign with heading exactly like a lever-arm error, so that check conflates the two;
`lever_arm_ab.py` exists because of it.

What is left is the user's call, and then one drive:
- **Decide** whether to move the bridge default to (0.67, -0.10). Two independent facts point there
  (the receiver's configuration and the vehicle geometry) and the replay excludes 2.39; against
  that, the arm itself has never been measured on this vehicle, and the legacy tracker has always
  used 2.39, so adopting it changes obstacle positions by 1.7 m along the heading.
- **Measure it** with a drive the estimator can actually use: pass static objects CLOSE (a few
  metres laterally, so the bearing sweeps 60 deg or more) at moderate speed, in both directions.
  Then `scripts/lever_arm_ab.py <recording>` reports the arm with a bootstrap interval, and
  `--profile` should show a real minimum rather than a flat line.
Done when: the fit's bootstrap interval is under ~0.4 m wide and agrees with the vendor geometry,
and the bridge default is set to it.

### Housekeeping
- ~~Commit the new work~~ -- DONE 2026-09-16 by the user: branch `radar_integration_and_fusion`,
  commit `e462094`.
- CHANGELOG.md / README.md are pre-existing files: document the new stack there only with the
  user's OK.
- ~~`/perception/objects` cannot be echoed from the host~~ -- DONE 2026-09-17:
  `scripts/install_host_fusion_msgs.sh` (install_host_custom_msgs.sh untouched).
- ~~Expose the rollback parameters as env vars~~ -- DONE. `GROUND_LEVELLING`,
  `ENABLE_DEPTH_GATE`, `ENABLE_CLASS_VOTE`, `SEGMENTATION_EMPTY_FALLBACK`, and since 2026-09-16
  `ASSOC_MAX_DIST` / `MERGE_MAX_DIST`, since 2026-09-20 `RADAR_CAMERA_GATE` and since
  2026-09-21 `ENABLE_LIDAR_CLUSTERS` (also the `--clusters` flag), all A/B without a rebuild.
  Check a rollback took effect by reading the parameter back from the node, not by trusting
  the command: a stray `\\` in the Dockerfile CMD once dropped every env var after
  `merge_max_dist` silently, and the launch default hid it.
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
- 7.8-9.0% of measurements have no published track (range-aware test), and 2.5-2.6% of
  published tracks are a second box on a measurement that already has one -- re-measured
  2026-09-22 over two full loops of the shipped build. Two loops of the SAME build differ by
  1.2 points, so read anything under that as noise. (The older 10.5% / 3.0% pair was item 11's
  arm, before the radar-camera gate and the cluster path.)
- Velocity is now WITHHELD unless it is distinguishable from standing still, so a genuinely
  moving object reads static until the evidence accumulates -- on the truck bag 3.1% of
  objects above 5 m/s. The planner bridge zeroes velocity for exactly those, which is the
  intended behaviour but means the FSM sees them as static obstacles.
- Levelling evidence in curves rests on 232 + 15 curve sweeps from two drives.
- The 360 deg path (item 7) would add clutter with no semantic check outside the camera FOV.

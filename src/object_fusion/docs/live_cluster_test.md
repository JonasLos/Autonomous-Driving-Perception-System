# Test card: cluster path live

Written for Thu 2026-10-08, when the goal was to find out why tracks died at the camera FOV edge
on the vehicle. Both causes are fixed (HANDOFF item 7, "Live at a treeline"). The run plan below
is kept as the template for re-testing. The settings block that follows is CURRENT (2026-10-20).
Every command runs from `~/Downloads/Autonomous-Driving-Perception-System`.

## After a code change: rebuild and restart the fusion stack only

```
docker compose -f docker-compose.yml -f docker-compose.fusion.yml build object_fusion_node
scripts/run_fusion.sh --down && scripts/run_fusion.sh --vehicle     # zenoh and drivers untouched
scripts/run_fusion.sh --status    # cluster_method grid, cluster_max_lateral 0.0,
                                  # camera_miss_in_image_only true, measurement_lag 0.12
```

A restart resets everything set live back to these defaults.

| setting (live: `ros2 param set ...`) | default | rollback |
|---|---|---|
| `/lidar_cluster_detector cluster_method` | `grid` (since 2026-10-20) | `exact` |
| `/lidar_cluster_detector cluster_max_lateral` | `0.0` = no crop | `10.0` (needed with `exact` in trees) |
| `/object_aggregator camera_miss_in_image_only` | `true` (since 2026-10-08) | `false` |
| `/object_aggregator measurement_lag` | `0.12` | `0.75` was the 10-08 stopgap |

At launch: `scripts/run_fusion.sh --exact-clusters` (exact + 10 m) or `--crop-misses`. Type
numbers as decimals (`10.0`, not `10`); an integer is rejected. The cluster detector's status
line in `scripts/run_fusion.sh --logs` shows rate, ms per sweep, age at publish, crop and method.
Healthy at a treeline: 10 Hz, age at publish under ~90 ms (grid ran 71-82 ms on 2026-10-15).

## Fixed for the whole day

| what | setting |
|---|---|
| Perception | `scripts/run_radar.sh --vehicle` (full stack: YOLO, transform, SphereFormer, CLRerNet, SAM3, radar_node). **No env vars, no extra flags.** Step 6 alone stops the lane nodes. |
| Fusion | `scripts/run_fusion.sh --vehicle`. **No env vars, no extra flags.** Defaults: clusters ON, filtered, ground flags, measurement_lag 0.12. Only step 5 changes the lag, live. |
| Planner | **OFF**, except step 8 |
| RViz | Whatever you normally run. Keep it the same all day. |
| Don't | Run `ros2 topic hz` / `echo`, or start or stop nodes during a pass |

Terminals: **T1** bring-up, **T2** `fusion_live_capture.sh`, **T3** planner (step 8 only).
After every pass the script asks for a note. Give the target, the speed actually held, and
anything odd.

Targets on a straight road with at least 150 m of run-up:
**Spot A** person dummy, **4 m right of the driven line**.
**Spot B** parked car, about 60 m past A, also 4 m right.

Speeds: 2 m/s = 4.5 mph · 5 m/s = 11 mph · 9 m/s = 20 mph.

---

### 0. Bring-up and preflight (parked)

```
T1  scripts/run_radar.sh --vehicle
T1  scripts/run_fusion.sh --vehicle
    # wait 2 minutes
T2  scripts/fusion_live_capture.sh preflight
```
It must end in **GO**.
- **NO-GO** on `/perception/nonground` or `/perception/measurements/lidar`: the cluster chain is
  not running live. That is already the answer. Do step 3 at 5 m/s only, then stop.
- **NO-GO** on odom: fix the NovAtel driver before driving.

### 1. S0: at rest (parked in P)
Dummy 12-15 m ahead, about 2 m right, in camera view.
```
T2  scripts/fusion_live_capture.sh pass S0-rest        # wait 60 s, then Ctrl-C
```

### 2. S0b: walk out of view (parked in P)
A person stands 12 m ahead, walks right until about 8 m off the lane, stands 10 s, walks back.
```
T2  scripts/fusion_live_capture.sh pass S0b-walk-a     # Ctrl-C when back
T2  scripts/fusion_live_capture.sh pass S0b-walk-b
```

### 3. S1: speed sweep (manual, default lag)
Each pass: start ≥ 80 m before spot A, hold the speed, keep going until ≥ 30 m past spot B, then
Ctrl-C. Start the next pass on the way back to the start.
```
T2  scripts/fusion_live_capture.sh pass S1-v2-a        # 2 m/s
T2  scripts/fusion_live_capture.sh pass S1-v2-b
T2  scripts/fusion_live_capture.sh pass S1-v5-a        # 5 m/s
T2  scripts/fusion_live_capture.sh pass S1-v5-b
T2  scripts/fusion_live_capture.sh pass S1-v9-a        # 9 m/s
T2  scripts/fusion_live_capture.sh pass S1-v9-b
```

### 4. Pick the lag for step 5
```
T2  scripts/fusion_live_capture.sh summary
```
Section B of each S1 pass prints "a measurement_lag of X s would sequence 95% of clusters".
**L** = the largest X across the S1 passes, rounded up to 0.05, and **at least 0.40**.

### 5. S2: raised lag (manual only: /perception/objects runs L seconds behind)
```
T2  scripts/fusion_live_capture.sh lag L               # e.g. lag 0.45
T2  scripts/fusion_live_capture.sh pass S2-v5-a        # 5 m/s, same procedure as step 3
T2  scripts/fusion_live_capture.sh pass S2-v5-b
T2  scripts/fusion_live_capture.sh pass S2-v9-a        # 9 m/s
T2  scripts/fusion_live_capture.sh pass S2-v9-b
T2  scripts/fusion_live_capture.sh lag 0.12            # RESTORE. Must read back 0.12.
```

### 6. S3: lighter load (parked to switch; skip if steps 3 and 5 look identical)
Use `docker stop`, **not** `run_radar.sh --down`: that also kills the zenoh router and with it
odometry.
```
T1  docker stop perception_sphereformer_node perception_clrernet_node test_sam3_container
T2  scripts/fusion_live_capture.sh preflight           # must still be GO
T2  scripts/fusion_live_capture.sh pass S3-light-v5-a  # 5 m/s, procedure as step 3
T2  scripts/fusion_live_capture.sh pass S3-light-v5-b
```

### 7. Restore the full stack (parked)
```
T1  scripts/run_radar.sh --vehicle                     # brings the lane nodes back
    # wait 2 minutes
T2  scripts/fusion_live_capture.sh preflight           # must be GO; check odom ~100 Hz
```

### 8. S4: autonomous (default lag 0.12, full stack)
Dummy **in the lane** at spot A. Launch the planner first and wait for it to settle, **then**
start the recording, so the two don't join the graph at once.
```
T3  cd ~/latest_planner && source install/setup.bash
T3  ros2 launch ava_local_planner lmp_marked_obstacle_avoidance.launch.py path:=<route.csv>
T2  PLANNER=1 scripts/fusion_live_capture.sh pass S4-auto-a
T2  PLANNER=1 scripts/fusion_live_capture.sh pass S4-auto-b
```

### 9. End of day
```
T2  scripts/fusion_live_capture.sh summary
```
Bring back the whole folder `~/fusion_live/2026-10-08/`.

---

Optional, only if preflight shows the disk can take it: one extra pass at 5 m/s that also
records raw LiDAR:
`RAW=1 scripts/fusion_live_capture.sh pass S1-v5-raw`.

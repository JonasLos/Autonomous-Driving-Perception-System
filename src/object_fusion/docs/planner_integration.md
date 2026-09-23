# Planner obstacles from object_fusion

> **This is the versioned copy** (added 2026-09-23). The working copy the planner team edits lives
> at `~/planner/docs/object_fusion_integration.md`, beside their other planner docs, and is not in
> any git repository. When they diverge, the one to trust is whichever matches
> `~/planner/src/AVA_Local_Planner/scripts/fusion_object_bridge.py` -- check the bridge, not the
> prose.

How the planner gets obstacles from the new perception stack, why it is done this way, and how to
go back. The measured results are in the section at the end.

## What runs

```
/perception/objects (fusion_msgs/FusedObjectArray, frame ego, capture-stamped)
+ /novatel/oem7/odom_grid
   -> fusion_object_bridge.py
   -> /planner/tracked_objects (ava_local_planner/ObjectList, frame "world")
   -> planner (its /tracked_objects is remapped onto /planner/tracked_objects)
```

| launch file | planner | obstacles from | rollback |
|---|---|---|---|
| `tracker_planner_refGenerator_lane_fusion.launch.py` | `planner_main.py` (unchanged) | bridge | `tracker_planner_refGenerator_lane.launch.py` |
| `gps_obj_tracker_fusion.launch.py` | `planner_gps_follower_obstacles.py` | bridge | `gps_obj_tracker.launch.py` |

`gps_obj_tracker_fusion.launch.py` **changes driving behaviour**. The original GPS follower has its
`/tracked_objects` subscription commented out, so it has never reacted to an obstacle.
`planner_gps_follower_obstacles.py` is that same planner with the subscription added. It subclasses
`ROSPlanner` and runs the original `main()`, so nothing is copied and the original file is untouched.

Every original file is unchanged except `CMakeLists.txt`, which gains three lines in its
`install(PROGRAMS ...)` list.

## Before the first run

1. The perception stack must be running with object_fusion, in `publish_mode: filtered` (the default
   since 2026-09-17).
2. `fusion_msgs` must be on the host:
   `~/Downloads/Autonomous-Driving-Perception-System/scripts/install_host_fusion_msgs.sh`. It installs
   into the overlay `~/.bashrc` already sources, so open a new terminal after running it.
3. `colcon build --packages-select ava_local_planner`.

```bash
ros2 launch ava_local_planner gps_obj_tracker_fusion.launch.py path:=<route.csv>
ros2 launch ava_local_planner tracker_planner_refGenerator_lane_fusion.launch.py path:=<route.csv>
```

Launch arguments beyond the originals: `objects_topic` (default `/perception/objects`) and
`min_updates` (default 2). The bridge's other parameters (`lidar_offset_x/y`, `history_length`,
`history_period_s`, `forget_after_s`, `max_odom_gap_s`) are ROS parameters on `fusion_object_bridge`.

## Why a bridge, and not feeding tracker.py

`tracker.py` re-tracks `/fused_bbox`. Pointing it at the new detections would keep four problems
that the bridge removes:

- **Latency.** `tracker.py` places each detection with the newest odometry, not odometry at capture
  time. The bridge interpolates the pose at the capture stamp. The measured effect is in the
  results section.
- **Yaw.** `tracker.py` rotates by `yaw + lidar_gps_yaw_adjustment`. That is 0.1107 rad (6.34°) from
  `params.yaml` in the lane launch, and the 0.0709 rad code default in `gps_obj_tracker`, which passes
  no parameters. The measured rotation of `lidar_tc` is 5.35°, confirmed against radar to ±0.1°, and
  object_fusion's `ego` frame already applies it. The bridge applies no adjustment; adding one would
  count the rotation twice.
- **A second tracker.** The aggregator already associates, filters and confirms. `tracker.py` on top
  of it adds a 5-frame birth wait and a 1.5 m nearest-neighbour gate that can swap ids.
- **Unbounded history.** `tracker.py` keeps every position an object ever had, and `kalman_predict`
  re-filters all of it on each replan. The bridge keeps 3 s.

## Three things the bridge must get exactly right

All three are in `fusion_bridge_core.py` and tested in `test/test_fusion_bridge_core.py`
(`python3 -m pytest test/test_fusion_bridge_core.py`).

1. **Pose at capture time.** Odometry is interpolated at the object message's stamp. A stamp more than
   0.1 s outside the odometry buffer is refused, not extrapolated, and counted as `no_pose` in the
   node's 5 s status line.
2. **History on an exact 0.1 s grid.** `kalman_predict` hard-codes dt = 0.1 s, and the FSM treats
   anything at or under 3 m/s as a static obstacle to avoid. The aggregator publishes about 40 times
   a second; appending every message would make a car at 10 m/s read 2.5 m/s. The test runs the
   planner's own `kalman_predict` on both histories to prove it.
3. **Which tracks.** Tentative tracks are skipped. Coasting tracks are kept: most long-lived cones are
   coasting, because object_fusion stops re-promoting a camera-only track after 5 camera opportunities.

## What `velocity_valid` means now (changed 2026-09-22, upstream)

The bridge zeroes an object's velocity unless `velocity_valid` is set
(`fusion_object_bridge.py:99`), and that flag now carries real information. It used to mean only
"the aggregator is in filtered mode and odometry exists", i.e. almost always true; it now means the
velocity is distinguishable from standing still given its covariance, with the extra uncertainty a
sweeping bearing produces (`|omega| * range`) folded in. Measured upstream: on a drive where every
object is static, the share of objects carrying a velocity fell from 10.1% to 0.4%, while a drive
with two passing vehicles kept 96.9% of the motion above 5 m/s.

For the planner this means more obstacles arrive as static and fewer arrive with a fabricated
heading -- so the FSM's 3 m/s static test now fires on evidence rather than on filter noise. Nothing
in the bridge had to change. If a genuinely moving object ever reads static, the flag is the place
to look, not the bridge: `ros2 topic echo /perception/objects` shows `velocity_valid` and
`velocity_covariance` per object.

## The LiDAR offset changed on 2026-09-23

`lidar_offset` went from `(2.393, 0.206)` m -- the value `tracker.py` has always used -- to
**`(0.670, -0.097)` m**, the user's decision after the evidence in object_fusion's to-do item 13:

- `/novatel/oem7/insconfig` reports `number_of_translations: 0`, so the receiver has no USER
  output point and the INS position is the IMU centre, not the rear axle or CoG the old value
  assumed;
- the vendor extrinsic puts `imu` at (-0.658, +0.159) m in `lidar_tc`, i.e. the LiDAR 0.658 m
  ahead of the IMU, which in vehicle axes is (0.670, -0.097) m.

Every obstacle this bridge places in the world moves about 1.7 m along the vehicle axis. NEITHER
number has been measured on this vehicle -- the selfcal drive cannot measure it (condition number
39752) -- so the node now logs the value and its provenance as a WARNING on every startup, and
`lidar_offset_x` / `lidar_offset_y` restore the old value at runtime without a rebuild:

    ros2 run ava_local_planner fusion_object_bridge.py --ros-args \
        -p lidar_offset_x:=2.393 -p lidar_offset_y:=0.206

What would settle it: a drive that passes static objects within a few metres at varied headings.

## Why the topic is renamed

The perception stack's `radar_fusion_node` already publishes `perception_msgs/TrackedObjectArray` on
`/tracked_objects`, and `tracker.py` publishes a different type, `ObjectList`, on the same name. The
bridge publishes on `/planner/tracked_objects`, and the new launch files remap the planner onto it,
so the two can no longer collide.

## Known issues it does not fix (existing files are left as they are)

- `planner_main.py`'s `TrackerCallback` appends to `self.obj` forever. The GPS subclass rebuilds it
  per message; the lane launch still has the growth.
- Whenever the planner skips a replan, the FSM sees an empty velocity list and falls back to
  `vehicle_following`, then a state change forces the next replan. On the vehicle, watch
  `/current_fsm_state` for switching back and forth while an obstacle is in view.
- `history_keeper` still transforms lane boundaries with the 6.34° adjustment. The measured 5.35° is
  the value to calibrate it to (Defect 5 in `tracker_planner_refGenerator_analysis.md`).
- `tracker.py`'s stop-sign and traffic-light injection is not replicated. Both lists are empty in
  `params.yaml`.

## Results (replay, selfcal_loc_2026-09-08, 2026-09-17)

Both nodes ran on the same replay together and were recorded for one full loop: 4291 messages each,
about one object per message. Scored with `scripts/planner_objects_ab.py` in the perception repo;
the recording is `~/fusion_data/recordings/planner_ab`.

| | legacy `tracker.py` | bridge |
|---|---|---|
| **latency error** (along-track error vs ego speed) | **+99 ± 5 ms** (1.0 m at 10 m/s) | **+16 ± 22 ms** (not distinguishable from 0) |
| **kalman_predict speed, median / p90** | 2.13 / 38.2 m/s | 0.19 / 1.40 m/s |
| **obstacles read as moving, > 3 m/s** (the FSM does not avoid them) | **44.9%** | **5.8%** |
| position spread over a track's life (median p90) | 0.51 m | 0.44 m |
| median track lifetime | 2.5 s | 3.2 s |

The drive is almost entirely static cones and parked vehicles, so the "read as moving" row is the
safety number. Fed by the legacy tracker, the planner would classify nearly half of those objects as
moving, and a moving object is followed, not avoided. The likely cause is `tracker.py`'s birth
buffer, `temp_new_objects`, which is keyed by the detection's index in the current frame rather than
by identity, so a new track's history can mix positions of different objects. Verifying that would
need a check of its own; the effect above is measured either way.

### Open: where the LiDAR is relative to the odometry position

`lidar_offset` (default 2.393, 0.206 m, the value `tracker.py` uses today) places the LiDAR relative
to the `/novatel/oem7/odom_grid` position, and three sources disagree:

| source | forward offset |
|---|---|
| `tracker.py` today: `b = 1.5055` CoG shift + `lidar_gps_offset_x` | 2.39 m |
| vendor extrinsic: IMU at x = −0.658 in lidar_tc, if the INS reports at the IMU (`SETINSTRANSLATION` is commented out in the driver's init commands) | 0.66 m |
| static objects seen from opposite headings on the replay (`planner_objects_ab.py`), **9 pairs only** | about 3.0 m |

The legacy tracker carries the same uncertainty, so the bridge is no worse than today. It is worth
settling before relying on obstacle positions to within a metre. Drive an out-and-back course past a
line of cones and re-run the offset check: more opposed-heading pairs turn the third row into a
measurement. Alternatively, read the receiver's configured output point (`INSCONFIG`, as
`calibrate_novatel_vehicle_alignment.py` in `~/0702_planner` does).

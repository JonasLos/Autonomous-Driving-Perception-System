#!/usr/bin/env bash
# Legacy tracker.py and the planner's fusion_object_bridge side by side, for the replay A/B that
# scripts/planner_objects_ab.py scores.
#
#   1. perception + object_fusion up (run_radar.sh --obstacle-only, run_fusion.sh --ground)
#   2. play the bag WITH odom_grid -- play_rosbag.sh's default list does not include it:
#        scripts/play_rosbag.sh -l -t "/camera_fl/image /camera_fl/camera_info \
#          /lidar_tc/velodyne_points /tf_static /delphi_esr_interface/radar/tracks \
#          /novatel/oem7/odom /novatel/oem7/odom_grid" ~/selfcal_loc_2026-09-08_11-47-43
#      Exactly ONE player: two interleave the sensors from different points in the bag and
#      both publish /clock.
#   3. scripts/run_planner_ab_nodes.sh
#   4. timeout -s TERM 430 ros2 bag record -s mcap --use-sim-time -o <dir> \
#        /legacy/tracked_objects /planner/tracked_objects /novatel/oem7/odom_grid \
#        /perception/objects /fused_bbox
#   5. python3 scripts/planner_objects_ab.py <dir>
#
# The legacy output is remapped to /legacy/tracked_objects so neither node publishes on
# /tracked_objects, where radar_fusion_node already publishes a different type.
set -euo pipefail
LOGS="${LOGS:-$HOME/fusion_data}"
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/.local/opt/adps_custom_msgs/setup.bash" 2>/dev/null || true
source "$HOME/planner/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
SHARE="$HOME/planner/install/ava_local_planner/share/ava_local_planner"
PARAMS="$SHARE/config/params.yaml"
CSV="$HOME/planner/src/AVA_Local_Planner/scripts/planner_gps_follower_utm.csv"  # needed, unused

ros2 run ava_local_planner tracker.py --ros-args --params-file "$PARAMS" \
  -p use_sim_time:=true -p path:="$CSV" -r /tracked_objects:=/legacy/tracked_objects \
  > "$LOGS/legacy_tracker.log" 2>&1 &
ros2 run ava_local_planner fusion_object_bridge.py --ros-args --params-file "$PARAMS" \
  -p use_sim_time:=true > "$LOGS/bridge.log" 2>&1 &
echo "legacy tracker -> /legacy/tracked_objects, bridge -> /planner/tracked_objects"
echo "logs: $LOGS/legacy_tracker.log $LOGS/bridge.log   (stop: kill the two ros2 run PIDs)"
wait

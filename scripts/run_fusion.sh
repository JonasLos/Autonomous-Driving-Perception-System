#!/bin/bash
# Bring up the object-fusion stack alongside the existing perception pipeline. Start that first
# (scripts/run_radar.sh); this stack consumes its output.
#
#   scripts/run_fusion.sh --replay BAG        # bag replay (sets USE_SIM_TIME=true)
#   scripts/run_fusion.sh --vehicle           # live (the default mode)
#   scripts/run_fusion.sh --status            # read the live parameters back off the nodes
#   scripts/run_fusion.sh --shadow-report     # what the filter would have contributed
#   scripts/run_fusion.sh --logs | --down
#
# The defaults are the measured, adopted configuration -- no flags needed. Each live rule has a
# rollback flag, or set the env var of the same name; none needs a rebuild:
#   --no-ground       PROJECTION_TOPIC=/lidar_2d_projection  Patchwork++ ground flags + empty-box drop
#   --no-levelling    GROUND_LEVELLING=false        near-field odometry levelling
#   --no-depth-gate   ENABLE_DEPTH_GATE=false       one-frame depth outlier withholding
#   --no-class-vote   ENABLE_CLASS_VOTE=false       per-id majority class sizing
#   --no-clusters     ENABLE_LIDAR_CLUSTERS=false   360-degree LiDAR clusters sustaining tracks
#   --loose-velocity  TURN_VELOCITY_K=0.0           velocity_valid as before 2026-09-22
#   --passthrough     PUBLISH_MODE=passthrough      raw camera+LiDAR position, no radar/velocity
#                     RADAR_CAMERA_GATE=false       camera-referenced radar range gate
#                     MERGE_MAX_DIST=inf            merge distance bound
#                     SEGMENTATION_EMPTY_FALLBACK=true   all-ground box uses every point
# Other settings:
#   --debug-clouds    PUBLISH_DEBUG_CLOUDS=true     ground/non-ground clouds for RViz
#                     ODOM_TOPIC=/novatel/oem7/odom (twist only; /odom_grid's is bit-identical)
#                     EGO_YAW_CORRECTION_DEG=-5.35  lidar_tc -> ego yaw
# Still OFF by default: ENABLE_RADAR_ONLY_BIRTH, ENABLE_EXTENT_ESTIMATION,
# ENABLE_CAMERA_ONLY_FALLBACK.
#
# This composes an OVERLAY over docker-compose.yml; the base file is not modified, and
# `docker compose --profile runtime up` is unaffected whether or not this stack is running.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

COMPOSE=(docker compose -f docker-compose.yml -f docker-compose.fusion.yml)
MODE="vehicle"; BAG=""; ACTION="up"
export PUBLISH_MODE="${PUBLISH_MODE:-filtered}"
export ENABLE_RADAR_ONLY_BIRTH="${ENABLE_RADAR_ONLY_BIRTH:-false}"
export ENABLE_EXTENT_ESTIMATION="${ENABLE_EXTENT_ESTIMATION:-false}"
export EGO_YAW_CORRECTION_DEG="${EGO_YAW_CORRECTION_DEG:--5.35}"
# The ground-flagged projection is the adopted rule (default since 2026-10-05; until then it
# needed --ground). /lidar_2d_projection is the rollback.
export PROJECTION_TOPIC="${PROJECTION_TOPIC:-/perception/lidar_2d_projection_ground}"
export ODOM_TOPIC="${ODOM_TOPIC:-/novatel/oem7/odom}"
export PUBLISH_DEBUG_CLOUDS="${PUBLISH_DEBUG_CLOUDS:-false}"
# Everything measured-and-live, each with its rollback. Set any of these in the environment, or
# use the flags below, to A/B without rebuilding the image.
export GROUND_LEVELLING="${GROUND_LEVELLING:-true}"
export ENABLE_DEPTH_GATE="${ENABLE_DEPTH_GATE:-true}"
export ENABLE_CLASS_VOTE="${ENABLE_CLASS_VOTE:-true}"
export SEGMENTATION_EMPTY_FALLBACK="${SEGMENTATION_EMPTY_FALLBACK:-false}"
export ENABLE_CAMERA_ONLY_FALLBACK="${ENABLE_CAMERA_ONLY_FALLBACK:-false}"
export ASSOC_MAX_DIST="${ASSOC_MAX_DIST:-6.0}"
export MERGE_MAX_DIST="${MERGE_MAX_DIST:-2.5}"
export RADAR_CAMERA_GATE="${RADAR_CAMERA_GATE:-true}"
export ENABLE_LIDAR_CLUSTERS="${ENABLE_LIDAR_CLUSTERS:-true}"
# Velocity honesty. 0.0 is the rollback: no covariance inflation and no significance test,
# so velocity_valid means what it meant before 2026-09-22.
export TURN_VELOCITY_K="${TURN_VELOCITY_K:-1.0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --replay) MODE="replay"; BAG="${2:-}"; shift 2 ;;
    --vehicle) MODE="vehicle"; shift ;;
    --filtered) PUBLISH_MODE="filtered"; shift ;;       # the default; kept so old commands work
    --passthrough) PUBLISH_MODE="passthrough"; shift ;;
    --ground) PROJECTION_TOPIC="/perception/lidar_2d_projection_ground"; shift ;;  # the default
    --no-ground) PROJECTION_TOPIC="/lidar_2d_projection"; shift ;;                 # rollback
    --debug-clouds) PUBLISH_DEBUG_CLOUDS="true"; shift ;;
    --no-levelling) GROUND_LEVELLING="false"; shift ;;
    --no-depth-gate) ENABLE_DEPTH_GATE="false"; shift ;;
    --no-class-vote) ENABLE_CLASS_VOTE="false"; shift ;;
    --clusters) ENABLE_LIDAR_CLUSTERS="true"; shift ;;   # the default since 2026-09-23
    --no-clusters) ENABLE_LIDAR_CLUSTERS="false"; shift ;;  # rollback
    --loose-velocity) TURN_VELOCITY_K="0.0"; shift ;;   # rollback: publish velocity as before
    --down) ACTION="down"; shift ;;
    --logs) ACTION="logs"; shift ;;
    --status) ACTION="status"; shift ;;
    --shadow-report) ACTION="shadow"; shift ;;
    -h|--help) sed -n '2,31p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

case "$ACTION" in
  down)
    "${COMPOSE[@]}" --profile fusion down --remove-orphans
    echo "object-fusion stack down. The existing pipeline is untouched."
    exit 0 ;;
  logs)
    exec docker logs -f perception_object_fusion_node ;;
  status)
    docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception || echo "  nothing running"
    if docker ps --format '{{.Names}}' | grep -qx perception_object_fusion_node; then
      set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || true; set -u
      # Read back from the nodes, not the env: a broken Dockerfile CMD once dropped variables
      # silently while the launch defaults made everything look right.
      while read -r node params; do
        for p in $params; do
          echo -n "  $node $p: "; ros2 param get "$node" "$p" 2>/dev/null || echo "?"
        done
      done <<'NODES'
/object_aggregator publish_mode enable_radar_only_birth ego_yaw_correction_deg measurement_lag assoc_max_dist merge_max_dist radar_camera_gate enable_lidar_clusters turn_velocity_k odom_topic
/camera_lidar_detector projection_topic segmentation_empty_fallback enable_depth_gate enable_class_vote enable_extent_estimation enable_camera_only_fallback
/ground_projection ground_levelling publish_nonground odom_topic
NODES
    fi
    exit 0 ;;
  shadow)
    docker logs perception_object_fusion_node 2>&1 | grep -E "tracks=|pairing:" | tail -5
    cat <<'MSG'

  Reading it:
    cam rejected %      the along-ray outlier population. ~36% on the reference replay; that
                        is the road-adoption failure, not a tuning error. A track with a high
                        count has a position worth distrusting.
    radar_only          candidates accrue even with birth OFF -- that is the shadow evidence.
    odom_starved        odometry could not cover a step; prediction STOPPED rather than
                        extrapolating a stale twist across every track.
    late                a measurement arrived after its own release deadline. Rising means
                        measurement_lag is too short for the pipeline it is bounding.

  NOTE: do NOT try to verify the ego-yaw correction from this node's residuals. A consistent
  frame error is unobservable from inside the filter -- the state absorbs it. Use
  scripts/object_ab.py --ego-yaw, which measures it filter-free.
MSG
    exit 0 ;;
esac

if [[ "$MODE" == "replay" ]]; then
  [[ -n "$BAG" ]] || { echo "--replay needs a bag path" >&2; exit 2; }
  [[ -e "$BAG" ]] || { echo "no such bag: $BAG" >&2; exit 2; }
  export USE_SIM_TIME=true
  COMPOSE+=(-f docker-compose.fusion.replay.yml)
else
  export USE_SIM_TIME=false
fi

if ! pgrep -x rmw_zenohd >/dev/null; then
  echo "starting zenoh router..."
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/rmw_zenohd.log 2>&1 &
  sleep 4
fi

echo "mode=$MODE  USE_SIM_TIME=$USE_SIM_TIME"
echo "projection: $PROJECTION_TOPIC   odom: $ODOM_TOPIC   debug clouds: $PUBLISH_DEBUG_CLOUDS"
echo "live rules: levelling=$GROUND_LEVELLING depth_gate=$ENABLE_DEPTH_GATE "\
     "class_vote=$ENABLE_CLASS_VOTE empty_fallback=$SEGMENTATION_EMPTY_FALLBACK "\
     "clusters=$ENABLE_LIDAR_CLUSTERS radar_camera_gate=$RADAR_CAMERA_GATE merge=$MERGE_MAX_DIST"
echo "velocity: turn_velocity_k=$TURN_VELOCITY_K (0 = the pre-2026-09-22 always-valid rule)"
echo "gates: publish_mode=$PUBLISH_MODE radar_only_birth=$ENABLE_RADAR_ONLY_BIRTH "\
     "extent=$ENABLE_EXTENT_ESTIMATION ego_yaw=$EGO_YAW_CORRECTION_DEG"
echo
echo "This stack CONSUMES the existing pipeline's output. Start that first if it is not up:"
if [[ "$MODE" == "replay" ]]; then
  echo "  scripts/run_radar.sh --replay $BAG --obstacle-only"
else
  echo "  scripts/run_radar.sh --vehicle      # with lanes: the planner follows CLRerNet's"
fi
echo

"${COMPOSE[@]}" --profile fusion up -d object_fusion_node
docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception

cat <<'MSG'

Up.
  /perception/measurements/*   per-sensor Detection3DArray
  /perception/objects          FusedObjectArray, frame `ego`
  TF: lidar_tc -> ego          the vehicle-longitudinal frame (base_link is NOT this)

  output (live): ros2 param set /object_aggregator publish_mode passthrough|filtered
  stats:         scripts/run_fusion.sh --shadow-report
  stop:          scripts/run_fusion.sh --down
MSG
if [[ "$MODE" == "replay" ]]; then
  echo
  echo "Then replay the bag in another shell:"
  echo "  scripts/play_rosbag.sh $BAG"
  echo
  echo "Do NOT use --start-offset: /tf_static is published only at the very start of the bag,"
  echo "so an offset silently removes lidar_tc -> delphi_esr_radar. Radar association then fails"
  echo "and RViz cannot place anything in the radar frame."
fi

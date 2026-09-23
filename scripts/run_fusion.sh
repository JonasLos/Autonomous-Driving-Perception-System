#!/bin/bash
# Bring up the object-fusion stack alongside the existing perception pipeline.
#
#   scripts/run_fusion.sh --replay BAG        # bag replay (sets USE_SIM_TIME=true)
#   scripts/run_fusion.sh --vehicle           # live
#   scripts/run_fusion.sh --status            # read the gates off the running node
#   scripts/run_fusion.sh --shadow-report     # what the filter would have contributed
#
# Rollbacks for the measured-and-live rules (or set the env vars of the same name):
#   --no-levelling    ground_levelling:=false     (near-field odometry levelling)
#   --no-depth-gate   enable_depth_gate:=false    (one-frame depth outlier withholding)
#   --no-class-vote   enable_class_vote:=false    (per-id majority class sizing)
#   scripts/run_fusion.sh --down
#
# This composes an OVERLAY over docker-compose.yml; the base file is not modified, and
# `docker compose --profile runtime up` is unaffected whether or not this stack is running.
#
# Output mode (default filtered since 2026-09-17):
#   --passthrough     PUBLISH_MODE=passthrough  publish the raw camera+LiDAR position instead of
#                                               the filter's (rollback: no radar range, no velocity)
#
# Still gated OFF by default:
#   ENABLE_RADAR_ONLY_BIRTH=false   radar may refine, never create
#   ENABLE_EXTENT_ESTIMATION=false  class priors, as today
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

COMPOSE=(docker compose -f docker-compose.yml -f docker-compose.fusion.yml)
MODE="vehicle"; BAG=""; ACTION="up"
export PUBLISH_MODE="${PUBLISH_MODE:-filtered}"
export ENABLE_RADAR_ONLY_BIRTH="${ENABLE_RADAR_ONLY_BIRTH:-false}"
export ENABLE_EXTENT_ESTIMATION="${ENABLE_EXTENT_ESTIMATION:-false}"
export EGO_YAW_CORRECTION_DEG="${EGO_YAW_CORRECTION_DEG:--5.35}"
export PROJECTION_TOPIC="${PROJECTION_TOPIC:-/lidar_2d_projection}"
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
    --ground) PROJECTION_TOPIC="/perception/lidar_2d_projection_ground"; shift ;;
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
    -h|--help) sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
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
      for p in publish_mode enable_radar_only_birth ego_yaw_correction_deg measurement_lag; do
        echo -n "  $p: "; ros2 param get /object_aggregator "$p" 2>/dev/null || echo "?"
      done
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
echo "projection: $PROJECTION_TOPIC   debug clouds: $PUBLISH_DEBUG_CLOUDS"
echo "live rules: levelling=$GROUND_LEVELLING depth_gate=$ENABLE_DEPTH_GATE "\
     "class_vote=$ENABLE_CLASS_VOTE empty_fallback=$SEGMENTATION_EMPTY_FALLBACK"
echo "velocity: turn_velocity_k=$TURN_VELOCITY_K (0 = the pre-2026-09-22 always-valid rule)"
echo "gates: publish_mode=$PUBLISH_MODE radar_only_birth=$ENABLE_RADAR_ONLY_BIRTH "\
     "extent=$ENABLE_EXTENT_ESTIMATION ego_yaw=$EGO_YAW_CORRECTION_DEG"
echo
echo "This stack CONSUMES the existing pipeline's output. Start that first if it is not up:"
echo "  scripts/run_radar.sh ${BAG:+--replay $BAG} --obstacle-only"
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

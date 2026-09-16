#!/usr/bin/env bash
#
# Bring up the full perception stack INCLUDING radar association.
#
# Adds radar_node on top of the normal pipeline. It subscribes /fused_bbox and the Delphi
# ESR tracks and publishes /tracked_objects (perception_msgs/TrackedObjectArray) with radar
# range, radial velocity and provenance attached.
#
# /fused_bbox is NOT touched. radar_node is a subscriber to it and the topic has exactly one
# publisher (/yolo/fusion_node), so the existing obstacle path is unaffected by running this.
#
# RADAR IS OFF BY DEFAULT even with the container up. With enable_radar_fusion=false the node
# publishes /tracked_objects as a pure passthrough of /fused_bbox (source=LIDAR_ONLY, no
# velocity) while still associating radar in the background and logging what it WOULD have
# contributed -- "shadow mode". That is how you gather evidence before opening the gate.
#
# Usage:
#   scripts/run_radar.sh                          # full stack, radar gate CLOSED (shadow)
#   scripts/run_radar.sh --enable-radar           # full stack, radar gate OPEN
#   scripts/run_radar.sh --replay BAG             # bag replay (sets USE_SIM_TIME=true)
#   scripts/run_radar.sh --obstacle-only          # skip the lane/road nodes (much lighter)
#   scripts/run_radar.sh --shadow-report          # print measured association stats
#   scripts/run_radar.sh --status | --logs | --down
#
# Flip the gate live rather than restarting, so an A/B compares the same frames:
#   ros2 param set /radar_fusion_node enable_radar_fusion true
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

OBSTACLE_SERVICES=(transform_node yolo_node)
LANE_SERVICES=(sphereformer_node clrernet_node)

MODE="vehicle"; BAG=""; OBSTACLE_ONLY=0; ACTION="up"
export ENABLE_RADAR_FUSION="${ENABLE_RADAR_FUSION:-false}"
export PUBLISH_RADAR_ONLY="${PUBLISH_RADAR_ONLY:-false}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --replay) MODE="replay"; BAG="${2:-}"; shift 2 ;;
    --vehicle) MODE="vehicle"; shift ;;
    --obstacle-only) OBSTACLE_ONLY=1; shift ;;
    --enable-radar) ENABLE_RADAR_FUSION=true; shift ;;
    --down) ACTION="down"; shift ;;
    --logs) ACTION="logs"; shift ;;
    --status) ACTION="status"; shift ;;
    --shadow-report) ACTION="shadow"; shift ;;
    -h|--help) sed -n '2,30p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
  esac
done

# publish_radar_only lets radar CREATE objects rather than refine them, and the node rejects
# it unless fusion is enabled. Measured on the 2026-08-25 bag: with it on, 1202 of 1475
# objects (81.5%) were radar-originated -- ~10 unclassified returns per frame. Do not enable
# it without a false-alarm count broken down by range band.
if [[ "$PUBLISH_RADAR_ONLY" == "true" && "$ENABLE_RADAR_FUSION" != "true" ]]; then
  echo "PUBLISH_RADAR_ONLY=true requires ENABLE_RADAR_FUSION=true (the node rejects it otherwise)" >&2
  exit 2
fi

SERVICES=("${OBSTACLE_SERVICES[@]}")
[[ $OBSTACLE_ONLY -eq 0 ]] && SERVICES+=("${LANE_SERVICES[@]}")

case "$ACTION" in
  down)
    echo "stopping perception stack..."
    docker compose --profile runtime --profile radar down --remove-orphans || true
    pkill -x rmw_zenohd 2>/dev/null && echo "zenoh router stopped" || true
    echo "done."
    exit 0 ;;
  logs)
    exec docker compose --profile runtime --profile radar logs -f "${SERVICES[@]}" radar_node ;;
  status)
    docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception || echo "  (nothing running)"
    if pgrep -x rmw_zenohd >/dev/null; then echo "  zenoh router: up"; else echo "  zenoh router: DOWN"; fi
    if docker ps --format '{{.Names}}' | grep -qx perception_radar_node; then
      set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null || true; set -u
      echo -n "  enable_radar_fusion: "; ros2 param get /radar_fusion_node enable_radar_fusion 2>/dev/null || echo "?"
      echo -n "  publish_radar_only:  "; ros2 param get /radar_fusion_node publish_radar_only 2>/dev/null || echo "?"
    fi
    exit 0 ;;
  shadow)
    docker logs --tail 200 perception_radar_node 2>&1 | grep "pairing:" | tail -1 \
      | sed 's/.*pairing:/pairing:/' \
      || echo "no stats yet -- is radar_node running and is data flowing?"
    cat <<'HINT'

  assoc=N/M      how many fused objects got a radar match (measured ~25% on the 2026-08-25 bag)
  d_azimuth      should sit near 0. Near 5.4 deg means the TF is not being applied.
  d_range        fused minus radar. Measured median -1.3 m: LiDAR sees the near face of a
                 vehicle while radar returns from deeper in the body.
  max|skew|      must stay under max_pairing_skew (0.05). Measured 0.032 on that bag.
  radar_only_candidates  tracks that matched nothing. CANDIDATES, not false alarms.
HINT
    exit 0 ;;
esac

if [[ "$MODE" == "replay" ]]; then
  [[ -n "$BAG" ]] || { echo "--replay needs a bag path" >&2; exit 2; }
  [[ -e "$BAG" ]] || { echo "no such bag: $BAG" >&2; exit 2; }
  export USE_SIM_TIME=true
else
  export USE_SIM_TIME=false
fi

# transform_node and radar_node take their images independently: radar_node hardcodes
# perception-transform:latest (it must -- :pre-radar predates radar_ros and has no such
# package to launch), while transform_node reads TRANSFORM_IMAGE_TAG.
#
# This script is the CURRENT system, so transform_node runs :latest -- the committed source
# at HEAD, where commit 7703bb4 removed the voxel filter and raised the crop range to 150 m.
#
# That means transform behaviour differs between this script and run_baseline.sh, which pins
# :pre-radar (voxel filter ON at 0.1 m, crop range 100 m). The difference is real and it
# changes /fused_bbox. Do NOT attribute it to radar. For an A/B where radar is the only
# variable, run BOTH scripts with the same tag:
#     TRANSFORM_IMAGE_TAG=pre-radar scripts/run_radar.sh ...
export TRANSFORM_IMAGE_TAG="${TRANSFORM_IMAGE_TAG:-latest}"
if ! docker image inspect "perception-transform:$TRANSFORM_IMAGE_TAG" >/dev/null 2>&1; then
  echo "no such image: perception-transform:$TRANSFORM_IMAGE_TAG" >&2; exit 1
fi

if ! pgrep -x rmw_zenohd >/dev/null; then
  echo "starting zenoh router..."
  # ROS setup files reference unbound variables (AMENT_TRACE_SETUP_FILES and friends),
  # so `set -u` makes sourcing them fatal. Relax it just around the source.
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/rmw_zenohd.log 2>&1 &
  sleep 4
  pgrep -x rmw_zenohd >/dev/null || { echo "router failed to start, see /tmp/rmw_zenohd.log" >&2; exit 1; }
fi
echo "zenoh router: up"

echo "mode=$MODE  USE_SIM_TIME=$USE_SIM_TIME"
# State the voxel filter explicitly. The two images differ in it and the difference is easy
# to carry a wrong assumption about: :pre-radar calls voxel_downsample(0.1) unconditionally,
# :latest guards it behind a parameter that defaults to 0.0 (off).
case "$TRANSFORM_IMAGE_TAG" in
  pre-radar) VOXEL="ON  (0.1 m, hardcoded)  crop range 100 m" ;;
  latest)    VOXEL="OFF (voxel_size=0.0)    crop range 150 m" ;;
  *)         VOXEL="unknown for tag :$TRANSFORM_IMAGE_TAG" ;;
esac
echo "transform image: :$TRANSFORM_IMAGE_TAG   voxel filter: $VOXEL"

echo "radar gate: enable_radar_fusion=$ENABLE_RADAR_FUSION  publish_radar_only=$PUBLISH_RADAR_ONLY"
echo "starting: ${SERVICES[*]} radar_node"
docker compose --profile runtime up -d "${SERVICES[@]}"
[[ $OBSTACLE_ONLY -eq 0 ]] && docker compose up -d sam3_ros
docker compose --profile radar up -d radar_node

echo -n "waiting for yolo_node"
for _ in $(seq 1 40); do
  if docker logs perception_yolo_node 2>&1 | grep -q "\[yolo_node\] Activated"; then
    echo " -- ready"; break
  fi
  echo -n "."; sleep 5
done

echo
docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception
docker logs perception_radar_node 2>&1 | grep -m1 "Radar fusion node ready" | sed 's/^/  /' || true

cat <<'MSG'

Stack up.
  /fused_bbox        camera+LiDAR obstacles, UNCHANGED (yolo_msgs/DetectionArray)
  /tracked_objects   radar-aware objects   (perception_msgs/TrackedObjectArray)

To echo /tracked_objects you need the host overlay:
  source /opt/ros/jazzy/setup.bash
  source ~/.local/opt/adps_custom_msgs/setup.bash

  gate on (live):  ros2 param set /radar_fusion_node enable_radar_fusion true
  gate off:        ros2 param set /radar_fusion_node enable_radar_fusion false
  stats:           scripts/run_radar.sh --shadow-report
  stop:            scripts/run_radar.sh --down
MSG
if [[ "$MODE" == "replay" ]]; then
  echo
  echo "Now replay the bag in another shell:"
  echo "  source /opt/ros/jazzy/setup.bash"
  echo "  source ~/.local/opt/adps_custom_msgs/setup.bash"
  echo "  scripts/play_rosbag.sh $BAG"
fi

#!/usr/bin/env bash
#
# Bring up the perception stack AS IT WAS BEFORE THE RADAR WORK.
#
# This is the fallback path. It starts no radar node and cannot start one: radar lives in
# the "radar" Compose profile, which this script never activates.
#
# Two things make this genuinely the previous pipeline rather than merely radar-free:
#
#   1. transform_node is pinned to perception-transform:pre-radar, the last image verified
#      against a bag. Building the radar image also rebuilt :latest, which picks up the
#      2026-08-27 source changes (voxel filter off, crop range 100 -> 150) that the
#      previously deployed container predated. Pinning keeps transform_node exactly where
#      it was. Override with TRANSFORM_IMAGE_TAG=latest to take those changes.
#
#   2. fusion_node.py, tracking_node.py, yolo_node.py, yolo.launch.py, transform.py and
#      yolo_msgs are byte-for-byte unchanged by the radar work (`git diff` clean), so the
#      working tree already IS the previous fusion code. For a full pre-radar checkout
#      instead: `git checkout pre-radar-known-good`.
#
# Usage:
#   scripts/run_baseline.sh                      # vehicle mode (real sensors, no sim time)
#   scripts/run_baseline.sh --replay BAG         # bag replay (sets USE_SIM_TIME=true)
#   scripts/run_baseline.sh --obstacle-only      # skip the lane/road nodes (much lighter)
#   scripts/run_baseline.sh --status             # what is up, and at what rates
#   scripts/run_baseline.sh --down               # stop everything this script starts
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

# sphereformer/clrernet/sam3 produce lanes and road boundaries, not obstacles, and cost most
# of the GPU. --obstacle-only drops them, which is what you want for a fusion A/B.
OBSTACLE_SERVICES=(transform_node yolo_node)
LANE_SERVICES=(sphereformer_node clrernet_node)

MODE="vehicle"; BAG=""; OBSTACLE_ONLY=0; ACTION="up"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --replay) MODE="replay"; BAG="${2:-}"; shift 2 ;;
    --vehicle) MODE="vehicle"; shift ;;
    --obstacle-only) OBSTACLE_ONLY=1; shift ;;
    --down) ACTION="down"; shift ;;
    --logs) ACTION="logs"; shift ;;
    --status) ACTION="status"; shift ;;
    -h|--help) sed -n '2,31p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
  esac
done

# Pin unless the caller deliberately overrides. Fall back with a loud warning rather than
# silently running a different image than the header promises.
if [[ -z "${TRANSFORM_IMAGE_TAG:-}" ]]; then
  if docker image inspect perception-transform:pre-radar >/dev/null 2>&1; then
    export TRANSFORM_IMAGE_TAG=pre-radar
  else
    echo "WARNING: perception-transform:pre-radar not found; using :latest, which carries"
    echo "         the 2026-08-27 transform.py changes. Recreate the pin with:"
    echo "           docker tag perception-transform:latest perception-transform:pre-radar"
    export TRANSFORM_IMAGE_TAG=latest
  fi
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
    exec docker compose --profile runtime logs -f "${SERVICES[@]}" ;;
  status)
    docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception || echo "  (nothing running)"
    if pgrep -x rmw_zenohd >/dev/null; then echo "  zenoh router: up"; else echo "  zenoh router: DOWN"; fi
    echo "  radar node: $(docker ps --format '{{.Names}}' | grep -qx perception_radar_node \
        && echo 'RUNNING -- this is not the baseline stack' || echo 'not running (correct)')"
    exit 0 ;;
esac

if [[ "$MODE" == "replay" ]]; then
  [[ -n "$BAG" ]] || { echo "--replay needs a bag path" >&2; exit 2; }
  [[ -e "$BAG" ]] || { echo "no such bag: $BAG" >&2; exit 2; }
  export USE_SIM_TIME=true
else
  # Never on the vehicle: there is no /clock there, so every node's clock sits at 0,
  # watchdogs never fire and deferrals never expire.
  export USE_SIM_TIME=false
fi

# Refuse to half-start over a radar stack: the container names collide and you would end up
# with a mix of the two.
if docker ps --format '{{.Names}}' | grep -qx perception_radar_node; then
  echo "radar_node is running -- this is the BASELINE script." >&2
  echo "Stop it first:  scripts/run_radar.sh --down" >&2
  exit 1
fi

# Compose defines no router. Without one the containers and any host-side ROS process never
# discover each other, and the only symptom is that nothing happens.
# -x, not -f: `pgrep -f rmw_zenohd` also matches the shell running this script.
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

echo "starting: ${SERVICES[*]}"
docker compose --profile runtime up -d "${SERVICES[@]}"
[[ $OBSTACLE_ONLY -eq 0 ]] && docker compose up -d sam3_ros

# yolo_node colcon-builds at container start, so it is not ready the moment it is "Up".
echo -n "waiting for yolo_node"
for _ in $(seq 1 40); do
  if docker logs perception_yolo_node 2>&1 | grep -q "\[yolo_node\] Activated"; then
    echo " -- ready"; break
  fi
  echo -n "."; sleep 5
done

echo
docker ps --format '  {{.Names}}\t{{.Status}}' | grep perception
cat <<'MSG'

BASELINE stack up. Obstacle output is /fused_bbox (yolo_msgs/DetectionArray).
No radar node is running and none can be started from this script.

  logs:    scripts/run_baseline.sh --logs
  status:  scripts/run_baseline.sh --status
  stop:    scripts/run_baseline.sh --down
MSG
if [[ "$MODE" == "replay" ]]; then
  echo
  echo "Now replay the bag in another shell:"
  echo "  source /opt/ros/jazzy/setup.bash"
  echo "  source ~/.local/opt/adps_custom_msgs/setup.bash"
  echo "  scripts/play_rosbag.sh $BAG"
fi

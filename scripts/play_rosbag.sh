#!/bin/bash
# Replay a recorded bag into the running Docker perception stack.
#
# The containers subscribe to raw sensor topics exactly as they do on the vehicle, so an
# offline run is just "the sensors, from a file". Start the stack with sim time and play:
#
#   ros2 run rmw_zenoh_cpp rmw_zenohd &                      # if not already running
#   USE_SIM_TIME=true docker compose --profile runtime up -d \
#       transform_node sphereformer_node sam3_ros yolo_node clrernet_node
#   scripts/play_rosbag.sh /path/to/bag
#
# USE_SIM_TIME=true matters: this script publishes /clock, and it is what makes pausing
# (spacebar) actually freeze the nodes' watchdogs and pairing deferrals instead of letting
# them fire against wall time while the bag sits still.

set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/lib/topics.sh"

REPO_ROOT="$(repo_root)"
TOPIC_FILE="$(default_topic_file)"

RATE="1.0"
CLOCK_HZ="100"
CATEGORY="replay_input"
QOS_FILE="${REPO_ROOT}/config/rosbag_qos_overrides.yaml"
USE_QOS=1
LOOP=0
PAUSE=0
DRY_RUN=0
ALLOW_INTERNAL=0
START_OFFSET=""
EXPLICIT_TOPICS=""
BAG=""
declare -a PASSTHROUGH=()

usage() {
    sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    cat <<'EOF'

Usage: scripts/play_rosbag.sh [options] <bag_path>

  -r, --rate <x>             Playback rate (default 1.0)
  -l, --loop                 Loop playback
  -s, --start-offset <sec>   Skip the first N seconds
  -c, --clock <hz>           /clock publish rate (default 100); 0 disables --clock
  -p, --pause                Start paused
  -t, --topics "<a b ...>"   Explicit topic list, overrides --category
      --category <name>      topics.yaml category to play (default: replay_input)
      --qos-file <path>      QoS override file (default: config/rosbag_qos_overrides.yaml)
      --no-qos-override      Play with the QoS recorded in the bag
      --allow-internal       Permit replaying topics the containers also publish (unsafe)
  -n, --dry-run              Print the ros2 bag play command and exit
  -h, --help                 This message
  -- <args...>               Everything after -- is passed to `ros2 bag play` verbatim

Older bags may predate a topic rename and need remapping onto what the nodes subscribe to.
Bags recorded before 2026-08-13 carry the debayered /camera_fl/image_color rather than the
raw /camera_fl/image every node now uses:

  scripts/play_rosbag.sh -t "/camera_fl/image_color /camera_fl/camera_info \\
      /lidar_tc/velodyne_points /tf_static" --no-qos-override /path/to/bag \\
      -- --remap /camera_fl/image_color:=/camera_fl/image

Environment overrides:
  TOPIC_FILE          topics.yaml to read (default src/perception_common/topics.yaml)
  CUSTOM_MSGS_PREFIX  custom-message overlay (default ~/.local/opt/adps_custom_msgs)
  EXTRA_OVERLAYS      space-separated setup.bash paths to source before the overlay
  ROS_DISTRO          default jazzy
  ROS_DOMAIN_ID       default 0     -- must match the containers
  RMW_IMPLEMENTATION  default rmw_zenoh_cpp -- must match the containers

Keep --clock at 100 or above. Under sim time the nodes' 0.02s deferral pump fires on /clock
ticks, so a low clock rate makes deferral expiry coarser than the wait it is bounding.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--rate)          RATE="$2"; shift 2 ;;
        -l|--loop)          LOOP=1; shift ;;
        -s|--start-offset)  START_OFFSET="$2"; shift 2 ;;
        -c|--clock)         CLOCK_HZ="$2"; shift 2 ;;
        -p|--pause)         PAUSE=1; shift ;;
        -t|--topics)        EXPLICIT_TOPICS="$2"; shift 2 ;;
        --category)         CATEGORY="$2"; shift 2 ;;
        --qos-file)         QOS_FILE="$2"; shift 2 ;;
        --no-qos-override)  USE_QOS=0; shift ;;
        --allow-internal)   ALLOW_INTERNAL=1; shift ;;
        -n|--dry-run)       DRY_RUN=1; shift ;;
        -h|--help)          usage; exit 0 ;;
        --)                 shift; PASSTHROUGH=("$@"); break ;;
        -*)                 echo "[ERROR] Unknown option: $1" >&2; usage >&2; exit 1 ;;
        *)                  BAG="$1"; shift ;;
    esac
done

# === Validate the bag ===
if [[ -z "$BAG" ]]; then
    echo "[ERROR] No bag path given." >&2
    usage >&2
    exit 1
fi
if [[ ! -e "$BAG" ]]; then
    echo "[ERROR] No such bag: $BAG" >&2
    exit 1
fi
METADATA="$BAG/metadata.yaml"
if [[ ! -f "$METADATA" ]]; then
    echo "[ERROR] $BAG has no metadata.yaml -- point at the bag directory, not a .db3/.mcap file." >&2
    exit 1
fi
if [[ ! -f "$TOPIC_FILE" ]]; then
    echo "[ERROR] topics.yaml not found at $TOPIC_FILE (override with TOPIC_FILE=)." >&2
    exit 1
fi
require_yq

# === ROS environment, matched to the containers ===
# nounset trips on the setup scripts' internal unset vars, so relax it while sourcing.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
for overlay in ${EXTRA_OVERLAYS:-}; do
    if [[ -f "$overlay" ]]; then
        # shellcheck disable=SC1090
        source "$overlay"
    else
        echo "[WARN] EXTRA_OVERLAYS entry not found, skipping: $overlay" >&2
    fi
done
CUSTOM_MSGS_PREFIX="${CUSTOM_MSGS_PREFIX:-$HOME/.local/opt/adps_custom_msgs}"
if [[ -f "${CUSTOM_MSGS_PREFIX}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "${CUSTOM_MSGS_PREFIX}/setup.bash"
else
    # Not fatal: the raw sensor topics are all stock sensor_msgs, so playback works without
    # it. You want it anyway, because this shell is also where you run `ros2 topic echo
    # /fused_bbox` -- and those messages are custom.
    echo "[WARN] No custom-message overlay at ${CUSTOM_MSGS_PREFIX}." >&2
    echo "[WARN] Playback will work, but you will not be able to echo /fused_bbox," >&2
    echo "[WARN] /clrernet/* or /sam3_* from this shell. Fix with:" >&2
    echo "[WARN]   bash ${REPO_ROOT}/scripts/install_host_custom_msgs.sh" >&2
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

# The compose stack defines no Zenoh router -- it is expected on the host. Without one, the
# player and the containers simply never discover each other, and the only symptom is that
# nothing happens, which is a long debugging session for a one-line cause.
if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]] && ! pgrep -f rmw_zenohd >/dev/null 2>&1; then
    echo "[WARN] RMW is rmw_zenoh_cpp but no rmw_zenohd is running." >&2
    echo "[WARN] The containers will not see this player. Start one with:" >&2
    echo "[WARN]   ros2 run rmw_zenoh_cpp rmw_zenohd" >&2
fi

# === Decide what to play ===
declare -a PLAY_TOPICS=()
if [[ -n "$EXPLICIT_TOPICS" ]]; then
    read -r -a PLAY_TOPICS <<<"$EXPLICIT_TOPICS"
else
    # Via a temp file, not process substitution: mapfile would swallow the non-zero exit and
    # the caller would see only a bare "empty topic list" instead of which entry was bad.
    resolved_list=$(mktemp)
    trap 'rm -f "$resolved_list"' EXIT
    if ! resolve_category "$TOPIC_FILE" "$CATEGORY" >"$resolved_list"; then
        exit 1
    fi
    mapfile -t PLAY_TOPICS <"$resolved_list"
fi
if [[ ${#PLAY_TOPICS[@]} -eq 0 ]]; then
    echo "[ERROR] Resolved an empty topic list for category '${CATEGORY}'." >&2
    exit 1
fi

# === Guardrail: never republish what a container is already publishing ===
mapfile -t INTERNAL < <(internal_topics "$TOPIC_FILE")
mapfile -t IN_BAG < <(
    yq e '.rosbag2_bagfile_information.topics_with_message_count[].topic_metadata.name' \
        "$METADATA" 2>/dev/null | sort -u
)

declare -a COLLIDING=()
for topic in "${PLAY_TOPICS[@]}"; do
    for owned in "${INTERNAL[@]}"; do
        [[ "$topic" == "$owned" ]] && COLLIDING+=("$topic")
    done
done

if [[ ${#COLLIDING[@]} -gt 0 && $ALLOW_INTERNAL -eq 0 ]]; then
    echo "[ERROR] These topics are published by the perception containers themselves:" >&2
    printf '[ERROR]   %s\n' "${COLLIDING[@]}" >&2
    echo "[ERROR] Replaying them alongside a running stack means two publishers on one" >&2
    echo "[ERROR] topic, and consumers see the two interleaved with unrelated stamps." >&2
    echo "[ERROR] Play only raw sensor topics (--category replay_input), or pass" >&2
    echo "[ERROR] --allow-internal if the stack is genuinely not running." >&2
    exit 1
fi
if [[ ${#COLLIDING[@]} -gt 0 ]]; then
    echo "[WARN] --allow-internal: replaying ${#COLLIDING[@]} container-owned topic(s)." >&2
fi

# Missing topics are worth naming: a bag recorded before a topic existed will otherwise just
# leave part of the pipeline silent with no explanation.
if [[ ${#IN_BAG[@]} -gt 0 ]]; then
    declare -a MISSING=()
    for topic in "${PLAY_TOPICS[@]}"; do
        found=0
        for present in "${IN_BAG[@]}"; do
            [[ "$topic" == "$present" ]] && found=1 && break
        done
        [[ $found -eq 0 ]] && MISSING+=("$topic")
    done
    if [[ ${#MISSING[@]} -gt 0 ]]; then
        echo "[WARN] Not present in this bag: ${MISSING[*]}" >&2
        echo "[WARN] The stages that consume them will stay silent." >&2
    fi

    owned_in_bag=0
    for present in "${IN_BAG[@]}"; do
        for owned in "${INTERNAL[@]}"; do
            [[ "$present" == "$owned" ]] && owned_in_bag=$((owned_in_bag + 1))
        done
    done
    if [[ $owned_in_bag -gt 0 && $ALLOW_INTERNAL -eq 0 ]]; then
        echo "[INFO] Bag also holds ${owned_in_bag} container-owned topic(s); --topics excludes them."
    fi
fi

# === Build the command ===
declare -a CMD=(ros2 bag play "$BAG" --rate "$RATE")
[[ "$CLOCK_HZ" != "0" ]] && CMD+=(--clock "$CLOCK_HZ")
[[ $LOOP -eq 1 ]] && CMD+=(--loop)
[[ $PAUSE -eq 1 ]] && CMD+=(-p)
[[ -n "$START_OFFSET" ]] && CMD+=(--start-offset "$START_OFFSET")
if [[ $USE_QOS -eq 1 ]]; then
    if [[ -f "$QOS_FILE" ]]; then
        CMD+=(--qos-profile-overrides-path "$QOS_FILE")
    else
        echo "[WARN] QoS override file not found, playing with recorded QoS: $QOS_FILE" >&2
        echo "[WARN] If YOLO stays silent, this is the first thing to suspect." >&2
    fi
fi
[[ ${#PASSTHROUGH[@]} -gt 0 ]] && CMD+=("${PASSTHROUGH[@]}")
# --topics is variadic, so it has to be last or it swallows whatever follows.
CMD+=(--topics "${PLAY_TOPICS[@]}")

echo "[INFO] Bag:      $BAG"
echo "[INFO] Topics:   ${PLAY_TOPICS[*]}"
echo "[INFO] RMW:      $RMW_IMPLEMENTATION (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "[INFO] Command:  ${CMD[*]}"

if [[ $DRY_RUN -eq 1 ]]; then
    echo "[INFO] --dry-run, not playing."
    exit 0
fi

if [[ "$CLOCK_HZ" != "0" ]]; then
    echo "[INFO] Publishing /clock. Start the stack with USE_SIM_TIME=true or the nodes will"
    echo "[INFO] ignore it and keep running their watchdogs against wall time."
fi

exec "${CMD[@]}"

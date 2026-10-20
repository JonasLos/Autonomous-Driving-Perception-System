#!/bin/bash
# Record what is needed to tell WHY the 360-degree cluster path does not hold tracks live.
# Test plan: src/object_fusion/docs/live_cluster_test.md. Scoring: scripts/fusion_live_check.py.
#
#   scripts/fusion_live_capture.sh preflight       # parked, full stack up: GO / NO-GO
#   scripts/fusion_live_capture.sh pass LABEL      # record one pass; Ctrl-C ends it
#   scripts/fusion_live_capture.sh lag SECONDS     # set the aggregator's measurement_lag live
#   scripts/fusion_live_capture.sh summary         # one block per pass recorded so far
#
#   RAW=1      also record /lidar_tc/velodyne_points (lets ground_projection be re-run offline;
#              preflight prints what it costs in disk)
#   IMAGES=1   also record /yolov9/published_image (when YOLO last saw the target)
#   PLANNER=1  also record the planner topics (autonomous passes)
#   FUSION_LIVE_DIR=...   session root (default ~/fusion_live); one folder per day under it
#
# It deliberately starts as few ROS participants as it can: ONE recorder per pass and one
# `ros2 param get`, nothing repeated. Bursts of participants joining have stalled the NovAtel
# driver's odometry before (rmw_zenoh lost wakeup), and `ros2 topic hz` loops add load to the very
# thing being measured. Load is sampled with docker stats / nvidia-smi, which are not ROS.
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
REPO="$PWD"

set +u
source /opt/ros/jazzy/setup.bash
[[ -f ~/.local/opt/adps_custom_msgs/setup.bash ]] && source ~/.local/opt/adps_custom_msgs/setup.bash
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

SESSION="${FUSION_LIVE_DIR:-$HOME/fusion_live}/$(date +%F)"
FUSION_CONTAINER=perception_object_fusion_node
TOPICS=(
  /perception/objects
  /perception/measurements/camera_lidar
  /perception/measurements/lidar
  /perception/measurements/radar
  /perception/nonground
  /novatel/oem7/odom
  /novatel/oem7/odom_grid
  /tf /tf_static
  /camera_fl/camera_info
  /delphi_esr_interface/radar/tracks
)
[[ "${RAW:-0}" == 1 ]] && TOPICS+=(/lidar_tc/velodyne_points)
[[ "${IMAGES:-0}" == 1 ]] && TOPICS+=(/yolov9/published_image)
[[ "${PLANNER:-0}" == 1 ]] && TOPICS+=(/planner/tracked_objects /local_path /current_fsm_state
                                       /ctrl_ref_twist /target_traj)

mkdir -p "$SESSION"
log() { echo "$(date +%T) $*" | tee -a "$SESSION/session.log"; }

read_lag() {
  # One participant. Prints the value, or "unknown" if the node did not answer.
  local out
  out=$(timeout 10 ros2 param get /object_aggregator measurement_lag 2>/dev/null) || true
  [[ "$out" =~ value\ is:\ ([0-9.]+) ]] && echo "${BASH_REMATCH[1]}" || echo unknown
}

start_samplers() {  # $1 = output dir
  local d="$1"
  ( while :; do
      ts=$(date +%s.%N)
      docker stats --no-stream --format '{{.Name}},{{.CPUPerc}},{{.MemUsage}}' 2>/dev/null \
        | sed "s/^/$ts,/"
      sleep 1
    done > "$d/docker_stats.csv" ) &
  SAMPLERS=($!)
  if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=timestamp,index,utilization.gpu,memory.used \
      --format=csv,noheader,nounits -l 1 > "$d/gpu.csv" 2>/dev/null &
    SAMPLERS+=($!)
  fi
  ( while :; do echo "$(date +%s.%N) $(cat /proc/loadavg)"; sleep 1; done > "$d/loadavg.txt" ) &
  SAMPLERS+=($!)
}

stop_samplers() {
  for p in "${SAMPLERS[@]}"; do kill "$p" 2>/dev/null; done
  wait "${SAMPLERS[@]}" 2>/dev/null
}

record() {  # $1 = bag dir; runs in the background, sets REC
  ros2 bag record -s mcap -o "$1" "${TOPICS[@]}" > "$1.rec.log" 2>&1 &
  REC=$!
}

stop_record() {
  # `ros2 bag record` ignores SIGINT in a background job; SIGTERM flushes the mcap and exits.
  kill -TERM "$REC" 2>/dev/null
  for _ in $(seq 1 30); do kill -0 "$REC" 2>/dev/null || break; sleep 0.5; done
  kill -0 "$REC" 2>/dev/null && { echo "recorder did not exit, killing"; kill -9 "$REC"; }
}

case "${1:-}" in
# --------------------------------------------------------------------------------------------
preflight)
  N=$(find "$SESSION" -maxdepth 1 -mindepth 1 -type d | wc -l)
  P="$SESSION/$(printf '%02d' "$N")_preflight"; mkdir -p "$P"
  FAIL=()
  log "preflight -> $P"
  echo "-- containers"
  docker ps --format '   {{.Names}}\t{{.Status}}' | grep perception || true
  docker ps --format '{{.Names}}' | grep -qx "$FUSION_CONTAINER" \
    || FAIL+=("$FUSION_CONTAINER not running (scripts/run_fusion.sh --vehicle)")
  pgrep -x rmw_zenohd >/dev/null || FAIL+=("zenoh router not running")
  echo "   fusion image built: $(docker image inspect perception-object-fusion:latest \
        --format '{{.Created}}' 2>/dev/null || echo '?')   repo HEAD: $(git rev-parse --short HEAD)$(
        git diff --quiet || echo ' (+uncommitted)')"
  echo "-- host can decode fusion_msgs and score"
  ros2 interface show fusion_msgs/msg/FusedObjectArray >/dev/null 2>&1 \
    || FAIL+=("host has no fusion_msgs: scripts/install_host_fusion_msgs.sh")
  python3 -c "import mcap, mcap_ros2, scipy" 2>/dev/null \
    || FAIL+=("python: pip install --user mcap mcap-ros2-support scipy")
  echo "-- parameters, read back off the nodes"
  for n in object_aggregator ground_projection lidar_cluster_detector; do
    timeout 15 ros2 param dump "/$n" > "$P/params_$n.yaml" 2>/dev/null \
      || FAIL+=("/$n did not answer a param dump")
  done
  grep -q "enable_lidar_clusters: true" "$P/params_object_aggregator.yaml" 2>/dev/null \
    || FAIL+=("enable_lidar_clusters is not true on /object_aggregator")
  grep -q "publish_nonground: true" "$P/params_ground_projection.yaml" 2>/dev/null \
    || FAIL+=("publish_nonground is not true on /ground_projection")
  grep -E "measurement_lag|enable_lidar_clusters|publish_mode|odom_topic" \
    "$P/params_object_aggregator.yaml" 2>/dev/null | sed 's/^ */   /'
  grep -E "publish_nonground|input_topic" "$P/params_ground_projection.yaml" 2>/dev/null \
    | sed 's/^ */   /'
  echo "-- 15 s sample of every recorded topic"
  record "$P/bag"; start_samplers "$P"
  sleep 15
  stop_record; stop_samplers
  docker logs -t --since 2m "$FUSION_CONTAINER" > "$P/fusion_node.log" 2>&1
  echo "MEASUREMENT_LAG=$(grep -oE 'measurement_lag: [0-9.]+' "$P/params_object_aggregator.yaml" \
        2>/dev/null | awk '{print $2}')" > "$P/meta.env"
  echo "LABEL=preflight" >> "$P/meta.env"
  python3 scripts/fusion_live_check.py "$P" --brief | tee "$P/check.txt"
  grep -q "PREFLIGHT: GO" "$P/check.txt" || FAIL+=("streams unhealthy, see section A")
  bytes=$(du -sb "$P/bag" | cut -f1)
  free=$(df -B1 --output=avail "$SESSION" | tail -1)
  echo "-- disk: $(( bytes / 15 / 1000000 )) MB/s recorded -> $(( bytes * 40 / 1000000000 )) GB per"\
       "10 min; $(( free / 1000000000 )) GB free"
  echo
  if (( ${#FAIL[@]} )); then
    printf 'NO-GO:\n'; printf '   %s\n' "${FAIL[@]}"
    log "preflight NO-GO: ${FAIL[*]}"; exit 1
  fi
  echo "GO. Check that odom still reads ~100 Hz above: the param dumps joined the graph."
  log "preflight GO"
  ;;
# --------------------------------------------------------------------------------------------
pass)
  LABEL="${2:?usage: $0 pass LABEL   e.g. S1-v5-a}"
  LABEL="${LABEL//[^A-Za-z0-9_.-]/_}"
  N=$(find "$SESSION" -maxdepth 1 -mindepth 1 -type d | wc -l)
  P="$SESSION/$(printf '%02d' "$N")_$LABEL"
  mkdir -p "$P"
  LAG=$(read_lag)
  START_ISO=$(date --iso-8601=seconds); START=$(date +%s)
  {
    echo "LABEL=$LABEL"; echo "MEASUREMENT_LAG=$LAG"; echo "START=$START_ISO"
    echo "GIT=$(git rev-parse --short HEAD)"; echo "TOPICS=\"${TOPICS[*]}\""
  } > "$P/meta.env"
  record "$P/bag"; start_samplers "$P"
  log "pass $LABEL started (measurement_lag=$LAG) -> $P"
  echo "   Recording. Drive the pass, then Ctrl-C."
  STOP=0; trap 'STOP=1' INT
  while (( STOP == 0 )); do
    sleep 1
    kill -0 "$REC" 2>/dev/null || { echo "!! recorder exited, see $P/bag.rec.log"; break; }
    (( ( $(date +%s) - START ) % 15 == 0 )) && echo "   $(( $(date +%s) - START )) s"
  done
  trap - INT
  stop_record; stop_samplers
  echo "END=$(date --iso-8601=seconds)" >> "$P/meta.env"
  docker logs -t --since "$START_ISO" "$FUSION_CONTAINER" > "$P/fusion_node.log" 2>&1
  docker ps --format '{{.Names}}\t{{.Image}}\t{{.Status}}' > "$P/containers.txt"
  read -r -p "note (target, lateral offset, speed held, anything odd): " NOTE
  echo "NOTE=\"${NOTE//\"/\'}\"" >> "$P/meta.env"
  log "pass $LABEL stopped after $(( $(date +%s) - START )) s: $NOTE"
  python3 scripts/fusion_live_check.py "$P" | tee "$P/check.txt"
  ;;
# --------------------------------------------------------------------------------------------
lag)
  V=$(printf '%.3f' "${2:?usage: $0 lag SECONDS   (default 0.12)}")
  timeout 15 ros2 param set /object_aggregator measurement_lag "$V"
  log "measurement_lag set to $V, reads back $(read_lag)"
  ;;
# --------------------------------------------------------------------------------------------
summary)
  for c in "$SESSION"/*/check.txt; do
    [[ -e "$c" ]] || { echo "no passes in $SESSION"; exit 0; }
    echo "## $(basename "$(dirname "$c")")"
    grep -E "^== |note:|frames with a cluster update|/perception/measurements/lidar .*%|PREFLIGHT" "$c"
    awk '/Deaths ahead/{on=1;next} on && NF' "$c" | grep -v "verdict$" | head -12
    echo
  done
  ;;
*)
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
  exit 2 ;;
esac

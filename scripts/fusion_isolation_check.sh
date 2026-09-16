#!/usr/bin/env bash
# Does the object_fusion stack perturb the EXISTING pipeline?
#
# Replays the same stretch of a bag through the existing stack three times and records its outputs:
#   A1  existing stack alone
#   B   existing stack + object_fusion (scripts/run_fusion.sh --ground --debug-clouds: the heaviest
#       live configuration -- two Patchwork++ passes and two debug clouds per sweep)
#   A2  existing stack alone again
# A1 vs A2 is the run-to-run noise floor (GPU inference need not be bit-identical); B is judged
# against it. Each run saves /fused_bbox and /tracked_objects, every container's log (the
# `pairing:` counters) and `docker stats` samples. Compare with scripts/isolation_compare.py.
#
#   scripts/fusion_isolation_check.sh BAG OUT_DIR [DURATION_S=150]
#
# ISO_RADAR_FLAGS (default --obstacle-only) is passed to run_radar.sh: set it empty to bring up the
# lane nodes too (SphereFormer + CLRerNet), i.e. the vehicle's full load on the same GPU.
# ISO_RUNS (default "A1:0 B:1 A2:0") is the run list as name:with_fusion.
#
# Tears the stacks down between runs and at the end. Needs an idle machine: it stops whatever
# perception containers are running.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

BAG=${1:?usage: $0 BAG OUT_DIR [DURATION_S]}
OUT=${2:?usage: $0 BAG OUT_DIR [DURATION_S]}
DURATION=${3:-150}
RADAR_FLAGS=${ISO_RADAR_FLAGS---obstacle-only}
RUNS=${ISO_RUNS:-"A1:0 B:1 A2:0"}
mkdir -p "$OUT"

set +u
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/adps_custom_msgs/setup.bash
set -u

stop_recorder() {  # pid -- `ros2 bag record` IGNORES SIGINT when it is not a terminal's
  # foreground job (a 0-byte mcap and a `wait` that never returns); SIGTERM flushes and exits.
  kill -TERM "$1" 2> /dev/null || return 0
  local end=$((SECONDS + 30))
  while kill -0 "$1" 2> /dev/null; do
    (( SECONDS < end )) || { echo "recorder $1 ignored SIGTERM; killing (bag may be truncated)" >&2
                             kill -KILL "$1" 2> /dev/null; break; }
    sleep 1
  done
  wait "$1" 2> /dev/null || true
}

wait_for_log() {   # container, pattern, timeout_s
  local end=$((SECONDS + $3))
  until docker logs "$1" 2>&1 | grep -q "$2"; do
    (( SECONDS < end )) || { echo "timed out waiting for '$2' in $1" >&2; return 1; }
    sleep 2
  done
}

run() {
  local name=$1 with_fusion=$2
  echo "=== $(date +%T) run $name (object_fusion: $with_fusion)"
  scripts/run_radar.sh --down > /dev/null 2>&1 || true
  # shellcheck disable=SC2086
  scripts/run_radar.sh --replay "$BAG" $RADAR_FLAGS > "$OUT/$name.up.log" 2>&1
  if [[ $with_fusion == 1 ]]; then
    scripts/run_fusion.sh --replay "$BAG" --ground --debug-clouds >> "$OUT/$name.up.log" 2>&1
  fi
  wait_for_log perception_yolo_node "\] Activated" 400
  [[ $with_fusion == 1 ]] && wait_for_log perception_object_fusion_node "ground_projection: /" 120
  sleep 10

  ros2 bag record -s mcap -o "$OUT/$name" /fused_bbox /tracked_objects > "$OUT/$name.rec.log" 2>&1 &
  local rec=$!
  ( for _ in $(seq 1 $((DURATION / 5 + 12))); do
      docker stats --no-stream --format '{{.Name}},{{.CPUPerc}},{{.MemUsage}}'
      sleep 3
    done ) > "$OUT/$name.cpu.csv" 2> /dev/null &
  local stats=$!
  sleep 3

  timeout $((DURATION + 300)) scripts/play_rosbag.sh "$BAG" \
      -- --playback-duration "$DURATION" > "$OUT/$name.play.log" 2>&1 || true
  sleep 6

  stop_recorder "$rec"
  kill "$stats" 2> /dev/null || true
  local mcap
  mcap=$(find "$OUT/$name" -name '*.mcap' -size +1k | head -1)
  [[ -n "$mcap" ]] || echo "WARNING: $name recorded nothing -- check $OUT/$name.rec.log" >&2
  for c in $(docker ps --format '{{.Names}}' | grep '^perception_'); do
    docker logs "$c" > "$OUT/$name.$c.log" 2>&1 || true
  done
  scripts/run_radar.sh --down > /dev/null 2>&1 || true
  echo "=== $(date +%T) run $name done"
}

for spec in $RUNS; do
  run "${spec%%:*}" "${spec##*:}"
done
echo "compare: python3 scripts/isolation_compare.py $OUT"

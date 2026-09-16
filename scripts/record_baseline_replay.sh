#!/usr/bin/env bash
# Record what the EXISTING pipeline publishes while a bag replays -- the "replay" half that every
# offline harness needs (`ground_ab.py --replay`, `neighbour_ab.py --replay`): /fused_bbox is the
# rule being compared against, and the radar tracks are the ruler.
#
#   scripts/record_baseline_replay.sh BAG OUT_DIR
#
# The harnesses then take `--source BAG --replay OUT_DIR`. The self-check inside them (arm A25
# must reproduce the recorded /fused_bbox) is what proves the pair belongs together.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

BAG=${1:?usage: $0 BAG OUT_DIR}
OUT=${2:?usage: $0 BAG OUT_DIR}
[[ -e "$BAG" ]] || { echo "no such bag: $BAG" >&2; exit 2; }

set +u
source /opt/ros/jazzy/setup.bash
source ~/.local/opt/adps_custom_msgs/setup.bash
set -u

echo "=== $(date +%T) bringing up the existing stack for $BAG"
scripts/run_radar.sh --down > /dev/null 2>&1 || true
scripts/run_radar.sh --replay "$BAG" --obstacle-only > "$OUT.up.log" 2>&1

end=$((SECONDS + 400))
until docker logs perception_yolo_node 2>&1 | grep -q "\] Activated"; do
  (( SECONDS < end )) || { echo "YOLO never activated" >&2; exit 1; }
  sleep 2
done
sleep 10

ros2 bag record -s mcap -o "$OUT" \
    /fused_bbox /delphi_esr_interface/radar/tracks > "$OUT.rec.log" 2>&1 &
rec=$!
sleep 3
scripts/play_rosbag.sh "$BAG" > "$OUT.play.log" 2>&1 || true
sleep 6

# `ros2 bag record` ignores SIGINT off a terminal; SIGTERM flushes the mcap and exits.
kill -TERM "$rec" 2> /dev/null || true
for _ in $(seq 1 30); do kill -0 "$rec" 2> /dev/null || break; sleep 1; done
kill -KILL "$rec" 2> /dev/null || true
wait "$rec" 2> /dev/null || true

scripts/run_radar.sh --down > /dev/null 2>&1 || true
find "$OUT" -name '*.mcap' -size +1k | head -1 | grep -q . \
  || { echo "recorded nothing: see $OUT.rec.log" >&2; exit 1; }
echo "=== $(date +%T) recorded $(du -sh "$OUT" | cut -f1) to $OUT"

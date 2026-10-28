#!/bin/bash
# Shared bring-up for the four perception combinations. Source it, do not run it:
#
#   scripts/run_unmarked_v1.sh   vehicle SphereFormer + SAM3                       (unmarked roads)
#   scripts/run_unmarked_v2.sh   branch SphereFormer + SAM3 (upstream_boundary_fixes)
#   scripts/run_unmarked_v3.sh   V2 + road_boundary (planner: v10c guards on /road_boundary/*)
#   scripts/run_marked.sh        CLRerNet, no SphereFormer or SAM3                 (marked roads)
#
# All four also start transform_node, YOLO, radar and object fusion (the vehicle versions), via
# the existing run_radar.sh / run_fusion.sh, so every env knob those take still works
# (SAM3_CUDA_GRAPHS, *_GPU, CLUSTER_MAX_RANGE, ...). Each entry script sets PS_VERSION and calls
# perception_main "$@".
#
# The branch versions come from the upstream_boundary_fixes worktree (ADPS_UPSTREAM, default
# ~/Downloads/ADPS-upstream): SAM3 colcon-builds whichever checkout it is started from, and
# SphereFormer runs the perception-sphereformer:upstream image. Container names are fixed, so a
# bring-up always removes the previous combination first, from both checkouts.
#
# The zenoh router is never stopped here: run_radar.sh --down kills it and with it the drivers'
# ROS graph, so these scripts stop containers with docker compose directly.

PS_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PS_WORKTREE="${ADPS_UPSTREAM:-$HOME/Downloads/ADPS-upstream}"
PS_FUSION_IMAGE="perception-object-fusion:${OBJECT_FUSION_IMAGE_TAG:-latest}"
PS_SAM3_TIMEOUT_S="${PS_SAM3_TIMEOUT_S:-600}"   # colcon build + model load (+ cuda_graphs warm-up)

ps_compose_vehicle() { (cd "$PS_REPO" && docker compose -f docker-compose.yml -f docker-compose.gpu.yml "$@"); }
ps_compose_branch() {
  (cd "$PS_WORKTREE" && docker compose -f docker-compose.yml -f docker-compose.gpu.yml \
     -f docker-compose.upstream.yml "$@")
}
ps_ros() { set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/.local/opt/adps_custom_msgs/setup.bash" 2>/dev/null; true; }
ps_say() { echo "[$(date +%T)] $*"; }
ps_die() { echo "ERROR: $*" >&2; exit 1; }

ps_usage() {
  sed -n '2,/^[^#]/p' "$0" | sed '/^[^#]/d; s/^# \{0,1\}//'
  cat <<EOF
Options:
  --road gravel|paved   (unmarked only) SAM3 road prompt: gravel -> "gravel road" at confidence
                        0.15, paved -> "paved road". Default: SAM3's own 5-term prompt.
  --replay BAG          sim time, for testing off the vehicle; then play the bag with
                        scripts/play_rosbag.sh BAG
  --status              what is running, from where, SAM3 prompts and topic rates
  --down                stop all perception containers (keeps the zenoh router up)
EOF
}

ps_is_unmarked() { [[ "$PS_VERSION" == v1 || "$PS_VERSION" == v2 || "$PS_VERSION" == v3 ]]; }

# Everything perception, from both checkouts. Order matters: fusion first (its compose `down`
# with --profile fusion would otherwise race the others), the worktree's two branch containers,
# then the vehicle stack's runtime + radar services.
ps_stack_down() {
  ps_say "stopping perception (zenoh router stays up)"
  (cd "$PS_REPO" && scripts/run_fusion.sh --down >/dev/null 2>&1)
  [[ -d "$PS_WORKTREE" ]] && ps_compose_branch --profile runtime rm -sf sphereformer_node sam3_ros >/dev/null 2>&1
  ps_compose_vehicle --profile runtime --profile radar down >/dev/null 2>&1
  local left; left=$(docker ps --format '{{.Names}}' | grep -E '^(perception_|test_sam3_container$)' || true)
  [[ -z "$left" ]] || ps_die "still running after stop: $left"
}

ps_check_branch() {
  [[ -d "$PS_WORKTREE/.git" || -f "$PS_WORKTREE/.git" ]] \
    || ps_die "no upstream_boundary_fixes worktree at $PS_WORKTREE (set ADPS_UPSTREAM)"
  local b; b=$(git -C "$PS_WORKTREE/src/SAM3_ROS_NODE" branch --show-current 2>/dev/null)
  [[ "$b" == upstream_boundary_fixes ]] \
    || ps_die "the worktree's SAM3 submodule is on '$b', not upstream_boundary_fixes"
  docker image inspect perception-sphereformer:upstream >/dev/null 2>&1 \
    || ps_die "no perception-sphereformer:upstream image (build it from the worktree)"
}

ps_check_fusion_rb() {
  docker run --rm --entrypoint grep "$PS_FUSION_IMAGE" -q reinit_after_gated \
    /workspace/Autonomous-Driving-Perception-System/install/lib/python3.12/site-packages/object_fusion/road_boundary.py \
    || ps_die "$PS_FUSION_IMAGE predates the road_boundary tracker fix; rebuild it:
  docker compose -f docker-compose.yml -f docker-compose.fusion.yml --profile fusion build object_fusion_node"
}

ps_wait_sam3() {  # $1 = UTC start time of the bring-up
  echo -n "waiting for SAM3 (colcon build + model load)"
  local t0=$SECONDS
  while (( SECONDS - t0 < PS_SAM3_TIMEOUT_S )); do
    if docker logs --since "$1" test_sam3_container 2>&1 | grep -q "Waiting for images"; then
      echo " -- ready after $(( SECONDS - t0 )) s"; return 0
    fi
    docker logs --since "$1" test_sam3_container 2>&1 | grep -q "Traceback" \
      && { echo; ps_die "SAM3 failed to start: docker logs test_sam3_container"; }
    echo -n "."; sleep 5
  done
  echo; ps_die "SAM3 not ready after ${PS_SAM3_TIMEOUT_S} s: docker logs test_sam3_container"
}

# SphereFormer creates its point-cloud subscription only after the model is loaded and on the
# GPU, so that subscription is its ready signal (it logs no line of its own). Its input is
# /lidar_tc/velodyne_points on this stack (ring filter off) or /filtered_points with it on.
ps_wait_sphereformer() {
  ps_ros
  echo -n "waiting for SphereFormer (model load)"
  local t0=$SECONDS
  while (( SECONDS - t0 < 240 )); do
    if timeout 5 ros2 node info /sphereformer_lidar_segmentation 2>/dev/null \
         | sed -n '/Subscribers:/,/Publishers:/p' \
         | grep -qE "(velodyne_points|filtered_points): sensor_msgs/msg/PointCloud2"; then
      echo " -- ready after $(( SECONDS - t0 )) s"; return 0
    fi
    echo -n "."; sleep 3
  done
  echo; ps_die "SphereFormer has no point-cloud subscription after 240 s: docker logs perception_sphereformer_node"
}

# Live only: SAM3's first inference is slow even after "Waiting for images", so wait for its
# first boundary before measuring rates. In replay nothing flows until the bag plays.
ps_wait_first_boundary() {
  [[ "${USE_SIM_TIME:-false}" == true ]] && return 0
  ps_ros
  local t0=$SECONDS
  if timeout 90 ros2 topic echo --once /sam3_left_boundary --field header.stamp >/dev/null 2>&1; then
    ps_say "first SAM3 boundary after $(( SECONDS - t0 )) s"
  else
    ps_say "WARNING: no SAM3 boundary within 90 s (is the camera publishing?)"
  fi
}

ps_apply_road() {
  [[ -z "$PS_ROAD" ]] && return 0
  ps_ros
  case "$PS_ROAD" in
    gravel) ros2 param set /segmentation text_prompt "gravel road" >/dev/null
            ros2 param set /segmentation confidence_threshold 0.15 >/dev/null ;;
    paved)  ros2 param set /segmentation text_prompt "paved road" >/dev/null ;;
  esac
  ps_say "SAM3 road prompt set for $PS_ROAD roads"
}

ps_up() {
  local since; since=$(date -u +%FT%TZ)
  local replay=(); [[ -n "$PS_BAG" ]] && replay=(--replay "$PS_BAG")
  export USE_SIM_TIME=$([[ -n "$PS_BAG" ]] && echo true || echo false)
  case "$PS_VERSION" in
    v2|v3) ps_check_branch ;;&
    v3)    ps_check_fusion_rb ;;
  esac
  ps_stack_down
  ps_say "transform, YOLO, radar"
  (cd "$PS_REPO" && scripts/run_radar.sh "${replay[@]}" --obstacle-only >/dev/null) \
    || ps_die "run_radar.sh failed"
  case "$PS_VERSION" in
    v1)     ps_say "SphereFormer + SAM3: VEHICLE versions"
            ps_compose_vehicle --profile runtime up -d --no-deps sphereformer_node >/dev/null 2>&1 \
              && ps_compose_vehicle up -d --no-deps sam3_ros >/dev/null 2>&1 \
              || ps_die "could not start the vehicle SphereFormer/SAM3" ;;
    v2|v3)  ps_say "SphereFormer + SAM3: BRANCH versions ($PS_WORKTREE)"
            ps_compose_branch --profile runtime up -d --no-deps sphereformer_node >/dev/null 2>&1 \
              && ps_compose_branch up -d --no-deps sam3_ros >/dev/null 2>&1 \
              || ps_die "could not start the branch SphereFormer/SAM3" ;;
    marked) ps_say "CLRerNet"
            ps_compose_vehicle --profile runtime up -d --no-deps clrernet_node >/dev/null 2>&1 \
              || ps_die "could not start CLRerNet" ;;
  esac
  local rb=() label="object fusion"
  [[ "$PS_VERSION" == v3 ]] && { rb=(--road-boundary); label="object fusion + road_boundary"; }
  ps_say "$label"
  (cd "$PS_REPO" && scripts/run_fusion.sh "${replay[@]}" "${rb[@]}" >/dev/null) \
    || ps_die "run_fusion.sh failed"
  if ps_is_unmarked; then
    ps_wait_sam3 "$since"; ps_wait_sphereformer; ps_apply_road; ps_wait_first_boundary
  fi
  ps_status
}

ps_rate() {  # topic -> "<rate> Hz" or "-"
  local r; r=$(timeout 12 ros2 topic hz "$1" 2>/dev/null | grep 'average rate' | tail -1 | awk '{print $3}')
  echo "${r:--}"
}

ps_status() {
  ps_ros
  echo
  echo "== containers (image, started from)"
  docker ps --format '{{.Names}}\t{{.Image}}\t{{.Label "com.docker.compose.project.working_dir"}}' \
    | grep -E '^(perception_|test_sam3_container)' | sort | sed 's/^/  /' || echo "  nothing running"
  pgrep -x rmw_zenohd >/dev/null && echo "  zenoh router: up" || echo "  zenoh router: DOWN"
  if docker ps --format '{{.Names}}' | grep -qx test_sam3_container; then
    echo "== SAM3"
    for p in text_prompt obstacle_prompt confidence_threshold; do
      echo "  $p: $(timeout 8 ros2 param get /segmentation $p 2>/dev/null | sed 's/^[^:]*: //')"
    done
  fi
  local topics=(/fused_bbox /perception/objects)
  case "$PS_VERSION" in
    v1|v2)  topics+=(/sphereformer_left_boundary /sam3_left_boundary) ;;
    v3)     topics+=(/sphereformer_left_boundary /sam3_left_boundary
                     /road_boundary/sphereformer/left /road_boundary/sam3/left) ;;
    marked) topics+=(/clrernet/all_lanes) ;;
  esac
  if [[ "${USE_SIM_TIME:-false}" == true ]]; then
    echo "== replay mode: play the bag, then check rates with $0 --status"
    echo "   scripts/play_rosbag.sh $PS_BAG"
  else
    echo "== rates (10 s, all topics at once)"
    local tmp; tmp=$(mktemp -d)
    local i=0
    for t in "${topics[@]}"; do ps_rate "$t" > "$tmp/$i" & i=$((i + 1)); done
    wait
    i=0
    for t in "${topics[@]}"; do printf '  %-36s %s\n' "$t" "$(cat "$tmp/$i")"; i=$((i + 1)); done
    rm -rf "$tmp"
  fi
  echo "== planner"
  case "$PS_VERSION" in
    v1|v2) echo "  ros2 launch ava_local_planner lmp_unmarked_v10c.launch.py" ;;
    v3)    echo "  ros2 launch ava_local_planner lmp_unmarked_v10c_guards.launch.py boundary_source:=road_boundary" ;;
    marked) echo "  your usual lmp_marked_*.launch.py" ;;
  esac
  echo "== stop:  $0 --down"
}

perception_main() {
  PS_ROAD=""; PS_BAG=""; local action=up
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --road)   PS_ROAD="${2:-}"; shift 2
                ps_is_unmarked || ps_die "--road is for the unmarked versions"
                [[ "$PS_ROAD" == gravel || "$PS_ROAD" == paved ]] || ps_die "--road gravel|paved" ;;
      --replay) PS_BAG="${2:-}"; shift 2; [[ -e "$PS_BAG" ]] || ps_die "no such bag: $PS_BAG" ;;
      --status) action=status; shift ;;
      --down)   action=down; shift ;;
      -h|--help) ps_usage; exit 0 ;;
      *) ps_die "unknown argument: $1 (try --help)" ;;
    esac
  done
  case "$action" in
    up)     ps_up ;;
    status) ps_status ;;
    down)   ps_stack_down; ps_say "perception stopped" ;;
  esac
}

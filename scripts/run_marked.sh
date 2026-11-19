#!/usr/bin/env bash
# Marked roads: transform, YOLO, radar, CLRerNet lanes and object fusion. No SphereFormer, no SAM3.
# Planner: your usual lmp_marked_*.launch.py.
#
#   scripts/run_marked.sh [--replay BAG] | --status | --down
PS_VERSION=marked
# False-velocity fix ON for marked roads (2026-11-13, the user's call): the ESR lag model with its
# ego-acceleration term at the measured 0.15 s, and the people-only LiDAR-cluster velocity fit.
# Anything set in the environment wins, e.g. RADAR_RATE_LAG_S=0 CLUSTER_VELOCITY_ENABLE=false to roll back.
export RADAR_RATE_LAG_S="${RADAR_RATE_LAG_S:-0.15}"
export RADAR_RATE_LAG_ACCEL="${RADAR_RATE_LAG_ACCEL:-true}"
export CLUSTER_VELOCITY_ENABLE="${CLUSTER_VELOCITY_ENABLE:-true}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/perception_stack.sh"
perception_main "$@"

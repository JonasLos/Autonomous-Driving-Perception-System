#!/usr/bin/env bash
# Marked roads: transform, YOLO, radar, CLRerNet lanes and object fusion. No SphereFormer, no SAM3.
# Planner: your usual lmp_marked_*.launch.py.
#
#   scripts/run_marked.sh [--replay BAG] | --status | --down
PS_VERSION=marked
source "$(dirname "${BASH_SOURCE[0]}")/lib/perception_stack.sh"
perception_main "$@"

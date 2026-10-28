#!/usr/bin/env bash
# Unmarked roads, VERSION 2 -- the branch fixes: SphereFormer and SAM3 from upstream_boundary_fixes
# (perception-sphereformer:upstream, the ~/Downloads/ADPS-upstream worktree's SAM3), plus transform,
# YOLO, radar and object fusion. No CLRerNet, no road_boundary. Planner: lmp_unmarked_v10c.launch.py.
#
#   scripts/run_unmarked_v2.sh [--road gravel|paved] [--replay BAG] | --status | --down
PS_VERSION=v2
source "$(dirname "${BASH_SOURCE[0]}")/lib/perception_stack.sh"
perception_main "$@"

#!/usr/bin/env bash
# Unmarked roads, VERSION 3 -- V2 plus road_boundary (obstacle-robust boundary twins on
# /road_boundary/{sphereformer,sam3}/{left,right}, with the tracker fix). No CLRerNet.
# Planner: lmp_unmarked_v10c_guards.launch.py boundary_source:=road_boundary (v10c with its two
# source-switching guards, reading the road_boundary topics).
#
#   scripts/run_unmarked_v3.sh [--road gravel|paved] [--replay BAG] | --status | --down
PS_VERSION=v3
source "$(dirname "${BASH_SOURCE[0]}")/lib/perception_stack.sh"
perception_main "$@"

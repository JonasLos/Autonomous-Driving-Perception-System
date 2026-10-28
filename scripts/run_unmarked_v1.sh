#!/usr/bin/env bash
# Unmarked roads, VERSION 1 -- the vehicle code: vehicle SphereFormer (perception-sphereformer:latest)
# and vehicle SAM3 (submodule vehicle_deploy), plus transform, YOLO, radar and object fusion.
# No CLRerNet. Planner: lmp_unmarked_v10c.launch.py.
#
#   scripts/run_unmarked_v1.sh [--road gravel|paved] [--replay BAG] | --status | --down
PS_VERSION=v1
source "$(dirname "${BASH_SOURCE[0]}")/lib/perception_stack.sh"
perception_main "$@"

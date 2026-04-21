#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/jazzy/setup.bash"
# activate python virtual environment
source "/opt/venv/bin/activate"
export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:${PYTHONPATH}"

exec "$@"

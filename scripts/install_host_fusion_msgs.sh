#!/usr/bin/env bash
# Install fusion_msgs on the HOST, so host-side nodes can use /perception/objects.
#
#   scripts/install_host_fusion_msgs.sh [install_base]    # default ~/.local/opt/adps_custom_msgs
#
# fusion_msgs otherwise exists only inside the perception-object-fusion image, so neither
# `ros2 topic echo /perception/objects` nor the planner's fusion_object_bridge could read it.
#
# It merge-installs into the SAME overlay install_host_custom_msgs.sh uses, which ~/.bashrc
# already sources, so nothing else has to change for a new terminal to see the type. That script
# is deliberately left untouched: it is a pre-existing file. Re-running either script keeps the
# other's packages (merge-install only adds).
#
# fusion_msgs depends on std_msgs and geometry_msgs only.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
INSTALL_BASE="${1:-${HOME}/.local/opt/adps_custom_msgs}"
BUILD_BASE="${BUILD_BASE:-${REPO_ROOT}/build/host_fusion_msgs}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -d "${REPO_ROOT}/src/custom_msgs/fusion_msgs" ]]; then
  echo "Missing package directory: ${REPO_ROOT}/src/custom_msgs/fusion_msgs" >&2
  exit 1
fi

mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}"

set +u
source "${ROS_SETUP}"
set -u

cd "${REPO_ROOT}"
colcon build \
  --merge-install \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --base-paths src/custom_msgs/fusion_msgs \
  --packages-select fusion_msgs

echo
echo "verifying..."
set +u
source "${INSTALL_BASE}/setup.bash"
set -u
python3 -c "from fusion_msgs.msg import FusedObjectArray, FusedObject; \
print('  fusion_msgs ok: FusedObject.STATUS_COASTING =', FusedObject.STATUS_COASTING)"
echo
echo "fusion_msgs installed to: ${INSTALL_BASE}"
echo "New terminals that source it (~/.bashrc already does for the default base) can now run:"
echo "  ros2 topic echo /perception/objects"

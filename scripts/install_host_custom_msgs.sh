#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
INSTALL_BASE="${1:-${HOME}/.local/opt/adps_custom_msgs}"
BUILD_BASE="${BUILD_BASE:-${REPO_ROOT}/build/host_custom_msgs}"
PACKAGES=(clrernet_msgs sam2_msgs yolov9_msgs yolo_msgs)

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Install ROS 2 ${ROS_DISTRO} or export ROS_DISTRO to a valid distribution first." >&2
  exit 1
fi

for package in "${PACKAGES[@]}"; do
  if [[ ! -d "${REPO_ROOT}/src/custom_msgs/${package}" ]]; then
    echo "Missing package directory: ${REPO_ROOT}/src/custom_msgs/${package}" >&2
    exit 1
  fi
done

mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}"

set +u
source "${ROS_SETUP}"
set -u

cd "${REPO_ROOT}"
colcon build \
  --merge-install \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --packages-select "${PACKAGES[@]}"

echo
echo "Custom message packages installed to: ${INSTALL_BASE}"
echo "Source them with: source ${INSTALL_BASE}/setup.bash"

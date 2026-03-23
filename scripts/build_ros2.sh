#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_DIR="${ORB_SLAM3_ROS2_DIR:-${SCRIPT_DIR}/../ORB_SLAM3_ROS2}"
ROS2_DIR="$(cd "${ROS2_DIR}" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found. Set ROS_DISTRO or install ROS 2." >&2
  exit 1
fi
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
echo "Building orbslam3 in: ${ROS2_DIR} (ROS_DISTRO=${ROS_DISTRO})"
cd "${ROS2_DIR}"
colcon build --symlink-install --packages-select orbslam3 "$@"

#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ROS 2 Jazzy binary packages require Ubuntu 24.04 (Noble)." >&2
  echo "Detected: ${PRETTY_NAME:-unknown Linux distribution}" >&2
  echo "Create an Ubuntu-24.04 WSL distro or use native Ubuntu 24.04." >&2
  exit 1
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  echo "Install ROS 2 Jazzy Desktop on Ubuntu 24.04, then rerun this script." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash

if [[ "${EUID}" -eq 0 ]]; then
  sudo_cmd=()
else
  sudo_cmd=(sudo)
fi

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-librealsense2

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${sudo_cmd[@]}" rosdep init
fi
rosdep update

mkdir -p "${workspace_dir}/src"
vcs import "${workspace_dir}" < "${workspace_dir}/d435i_jazzy.repos"

rosdep install \
  --from-paths \
    "${workspace_dir}/src/realsense-ros" \
    "${workspace_dir}/src/rgbd_camera_bringup" \
  --ignore-src \
  --rosdistro jazzy \
  --skip-keys librealsense2 \
  --filter-for-installers apt \
  -r -y

cd "${workspace_dir}"
colcon build --symlink-install --packages-up-to rgbd_camera_bringup

echo
echo "Build complete. In each new terminal run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ${workspace_dir}/install/setup.bash"
echo "  ros2 launch rgbd_camera_bringup d435i.launch.py"


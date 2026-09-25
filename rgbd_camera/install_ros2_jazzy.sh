#!/usr/bin/env bash
set -euo pipefail

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "ROS 2 Jazzy binary packages require Ubuntu 24.04 (Noble)." >&2
  echo "Detected: ${PRETTY_NAME:-unknown Linux distribution}" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y

ros_apt_source_version="$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F 'tag_name' \
    | awk -F'"' '{print $4}'
)"

curl -fL \
  -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools

echo "ROS 2 Jazzy installed. Run: source /opt/ros/jazzy/setup.bash"


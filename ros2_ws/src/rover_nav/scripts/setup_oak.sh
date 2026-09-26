#!/usr/bin/env bash
# Once per laptop, for the OAK-D (nav_camera:=oak): let normal users open Luxonis devices
# (USB vendor 03e7; the ROS package doesn't install this rule). Then replug the camera.
#   sudo ros2_ws/src/rover_nav/scripts/setup_oak.sh
set -euo pipefail
rule=/etc/udev/rules.d/80-movidius.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' > "$rule"
udevadm control --reload-rules
udevadm trigger
echo "installed $rule: replug the OAK-D (use a USB 3 port; lsusb shows 03e7:2485 or 03e7:f63b)"

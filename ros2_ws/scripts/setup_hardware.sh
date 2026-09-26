#!/usr/bin/env bash
# Needs sudo. Once ever for the packages, again after every reboot / replug for CAN:
#   sudo ros2_ws/scripts/setup_hardware.sh
set -uo pipefail

# --- packages: RealSense ROS driver, can-utils (candump) ---
missing=()
[[ -d /opt/ros/jazzy/share/realsense2_camera ]] || missing+=(ros-jazzy-realsense2-camera)
command -v candump >/dev/null || missing+=(can-utils)
if ((${#missing[@]})); then
  apt-get update && apt-get install -y "${missing[@]}" || { echo "apt install failed" >&2; exit 1; }
fi

# --- StarArm102 leader ("Spark"): CH340 USB-serial, readable by the plugdev group, /dev/rebot_leader ---
rule=/etc/udev/rules.d/99-rebot-leader.rules
if [[ ! -f $rule ]]; then
  echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", GROUP="plugdev", MODE="0660", SYMLINK+="rebot_leader"' > "$rule"
  udevadm control --reload-rules
  udevadm trigger --subsystem-match=tty
  echo "leader udev rule installed: /dev/rebot_leader"
fi

# --- PEAK PCAN-USB → SocketCAN can0 at 1 Mbit/s for the RobStride motors ---
modprobe peak_usb
if ! ip link show can0 >/dev/null 2>&1; then
  echo "can0 does not exist: is the PCAN-USB adapter plugged in? (lsusb | grep -i peak)" >&2
  exit 1
fi
ip link set can0 down 2>/dev/null
if ! ip link set can0 type can bitrate 1000000 restart-ms 100; then
  cat >&2 <<'EOF'

The PCAN-USB adapter refused the bitrate (kernel: "couldn't set bitrate (err -32)", a USB stall).
Unplug the PCAN-USB, plug it straight into the laptop (not the USB hub), wait 3 s, run this again.
EOF
  exit 1
fi
if ! ip link set can0 up; then
  echo "could not bring can0 up; replug the adapter and run this again" >&2
  exit 1
fi
ip -details link show can0 | sed -n '1p;3p'
state=$(ip -details link show can0 | grep -o 'can state [A-Z-]*' | awk '{print $3}')
if [[ "$state" != "ERROR-ACTIVE" ]]; then
  echo "can0 is up but in state $state: check the motor power and the CAN cable / termination." >&2
  exit 1
fi
echo "can0 is UP (ERROR-ACTIVE). Stop motorbridge-gateway / Motorbridge Studio if they run: they hold the bus."

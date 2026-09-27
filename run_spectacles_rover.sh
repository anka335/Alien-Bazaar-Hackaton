#!/usr/bin/env bash
# Morning start of Spectacles teleop on the mobile manipulator (#45, #56). Run it in the foreground,
# inside the `ros` tmux session; Ctrl+C stops the stack (the bridge sends the rover one zero Twist).
#
#   ./run_spectacles_rover.sh stage-a     # real rover, simulated arm (driver_sim:=true rover:=true)
#   ./run_spectacles_rover.sh stage-b1    # real arm, no rover       (driver_sim:=false rover:=false)
#   ./run_spectacles_rover.sh stage-b2    # real arm and real rover  (driver_sim:=false rover:=true)
#   ./run_spectacles_rover.sh --print <stage>   # print the command and exit, launch nothing
#
# Every stage: hardware:=real enable_motors:=true run_task:=false spectacles:=true use_rviz:=true,
# lens on 127.0.0.1:9100. The script sets no ROS_DOMAIN_ID and no discovery range, so the rover's
# defaults apply. It sources this checkout's own ros2_ws/install. The B stages refuse to start
# unless can0 is UP.
set -eo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install="$repo/ros2_ws/install"
port=9100

usage() {
  echo "usage: $0 [--print] stage-a|stage-b1|stage-b2" >&2
  exit 2
}

print_only=false
if [[ "${1:-}" == "--print" ]]; then
  print_only=true
  shift
fi
[[ $# -eq 1 ]] || usage

case "$1" in
  stage-a) sim=true rover=true ;;
  stage-b1) sim=false rover=false ;;
  stage-b2) sim=false rover=true ;;
  *) usage ;;
esac
stage="$1"

cmd=(ros2 launch cloth_task task.launch.py hardware:=real "driver_sim:=$sim" enable_motors:=true
  run_task:=false spectacles:=true "rover:=$rover" "spectacles_port:=$port" use_rviz:=true)

# The real arm talks over CAN: refuse before sourcing or launching anything.
if ! $print_only && [[ $sim == false ]]; then
  can0_state="$(ip -br link show can0 2>/dev/null | awk '{print $2}')" || true
  if [[ "$can0_state" != "UP" ]]; then
    cat >&2 <<EOF
refusing $stage: can0 is ${can0_state:-absent}, it must be UP (ip -br link show can0).
Stop motorbridge-gateway and Motorbridge Studio if they run (they hold can0), then type these
yourself (rebot_b601/README.md) and run this again:
  sudo modprobe peak_usb
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 1000000 restart-ms 100
  sudo ip link set can0 up
EOF
    exit 1
  fi
fi

echo "stage: $stage (arm: $([[ $sim == true ]] && echo simulated || echo REAL), rover: $rover)"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset} (unset means 0)"
echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset} (unset means SUBNET)"
echo "lens port: 127.0.0.1:$port"
echo "build: $install"
echo "${cmd[*]}"

if $print_only; then
  exit 0
fi

if [[ ! -f "$install/setup.bash" ]]; then
  echo "no build at $install: run colcon build in $repo/ros2_ws first (ros2_ws/install missing)" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$install/setup.bash"
exec "${cmd[@]}"

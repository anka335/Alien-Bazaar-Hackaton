#!/usr/bin/env bash
# One command, real rig: arm + camera + the whole task
#   (unfold from home → box_view → SAM3 detect → approach → grasp → lift/check/retry → place →
#    next cloth … → ready, done). Ctrl+C parks the arm and switches the torque off.
# Defaults: place_target:=1, cycles:=0 (until no cloth is seen), execution_speed:=0.2.
# Extra launch arguments pass through and win, e.g.:
#   ros2_ws/scripts/run_real.sh place_target:=color cycles:=1 grasp:=false
set -eo pipefail
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! ip -details link show can0 2>/dev/null | grep -q 'ERROR-ACTIVE'; then
  echo "can0 is not up: run  sudo $ws/scripts/setup_hardware.sh  (replug the PCAN-USB if it fails)" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$ws/install/setup.bash"
exec ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true \
  place_target:=1 cycles:=0 execution_speed:=0.9 use_rviz:=true "$@"

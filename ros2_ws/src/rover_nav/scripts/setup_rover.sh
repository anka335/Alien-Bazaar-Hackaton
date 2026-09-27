#!/usr/bin/env bash
# Once per rover (again after reflashing LeoOS). Run on the laptop, connected to the rover's Wi-Fi,
# with SSH key access to the rover (ssh-copy-id pi@10.0.0.1). No sudo needed on either side.
#   ros2_ws/src/rover_nav/scripts/setup_rover.sh            # apply
#   ros2_ws/src/rover_nav/scripts/setup_rover.sh --restore  # put the original LeoOS files back
#
# What it does on the rover:
#   - ROBOT_NAMESPACE=leo in /etc/ros/setup.bash: every rover frame gets the `leo/` prefix
#     (leo/base_footprint, leo/base_link, ...) and every rover topic the /leo namespace
#     (/leo/cmd_vel, /leo/merged_odom, /leo/joint_states, ...). The reBot arm keeps `base_link`,
#     `/joint_states` and `/robot_description` (D-037).
#   - Deploys ../rover/robot.urdf.xacro: the stock model without the Panthera arm.
#   - Restarts the rover's ROS services and waits for /leo/merged_odom.
# The originals are saved once in ~/rover_nav_backup on the rover.
set -euo pipefail

rover="${ROVER:-pi@10.0.0.1}"
ns="${ROVER_NAMESPACE:-leo}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
urdf="$here/../rover/robot.urdf.xacro"
ssh_opts=(-o BatchMode=yes -o ConnectTimeout=5)

if ! ssh "${ssh_opts[@]}" "$rover" true 2>/dev/null; then
  echo "cannot SSH to $rover without a password: join the rover's Wi-Fi, then ssh-copy-id $rover" >&2
  exit 1
fi

if [[ "${1:-}" == "--restore" ]]; then
  ssh "${ssh_opts[@]}" "$rover" '
    set -e
    test -f ~/rover_nav_backup/setup.bash || { echo "no backup in ~/rover_nav_backup" >&2; exit 1; }
    cp ~/rover_nav_backup/setup.bash /etc/ros/setup.bash
    cp ~/rover_nav_backup/robot.urdf.xacro /etc/ros/urdf/robot.urdf.xacro
    systemctl --user restart ros.target'
  echo "original LeoOS config restored on $rover"
  exit 0
fi

echo "backing up the original config (once) and applying namespace '$ns' on $rover"
ssh "${ssh_opts[@]}" "$rover" '
  set -e
  mkdir -p ~/rover_nav_backup
  test -f ~/rover_nav_backup/setup.bash || cp /etc/ros/setup.bash ~/rover_nav_backup/
  test -f ~/rover_nav_backup/robot.urdf.xacro || cp /etc/ros/urdf/robot.urdf.xacro ~/rover_nav_backup/'
scp -q "${ssh_opts[@]}" "$urdf" "$rover:/etc/ros/urdf/robot.urdf.xacro"
ssh "${ssh_opts[@]}" "$rover" "
  set -e
  sed -i 's|^export ROBOT_NAMESPACE=.*|export ROBOT_NAMESPACE=\"$ns\"|' /etc/ros/setup.bash
  grep -q '^export ROBOT_NAMESPACE=\"$ns\"' /etc/ros/setup.bash
  systemctl --user restart ros.target"

echo "waiting for /$ns/merged_odom (up to 60 s)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
for _ in $(seq 12); do
  sleep 5
  if timeout 10 ros2 topic list 2>/dev/null | grep -qx "/$ns/merged_odom"; then
    echo "rover is up under /$ns"
    exit 0
  fi
done
echo "/$ns/merged_odom did not appear: check 'ssh $rover' then 'ros-logs' (source /etc/ros/setup.bash first)" >&2
exit 1

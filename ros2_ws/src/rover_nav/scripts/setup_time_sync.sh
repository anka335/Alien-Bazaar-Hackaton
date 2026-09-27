#!/usr/bin/env bash
# Once per laptop + rover pair. The rover has no internet, so its clock drifts (days, seen on our
# rover); TF and sensor messages from the two machines then don't line up. This makes the laptop
# an NTP server on the rover's network and points the rover's systemd-timesyncd at it.
# Run on the laptop, connected to the rover's Wi-Fi, as your user (asks for the sudo password of
# the laptop, then of the rover):
#   ros2_ws/src/rover_nav/scripts/setup_time_sync.sh
set -euo pipefail

rover="${ROVER:-pi@10.0.0.1}"
subnet="${ROVER_SUBNET:-10.0.0.0/24}"

laptop_ip="$(ip -4 -o addr show to "$subnet" | awk '{print $4}' | cut -d/ -f1 | head -1)"
if [[ -z "$laptop_ip" ]]; then
  echo "no laptop address in $subnet: join the rover's Wi-Fi first" >&2
  exit 1
fi
echo "laptop is $laptop_ip on the rover's network"

# --- laptop: chrony serves time to the rover's subnet ---
# `local stratum 10`: keep serving when the laptop itself is offline, so the two clocks still agree.
if ! command -v chronyd >/dev/null; then
  sudo apt-get update
  sudo apt-get install -y chrony   # replaces systemd-timesyncd on the laptop
fi
printf 'allow %s\nlocal stratum 10\n' "$subnet" | sudo tee /etc/chrony/conf.d/rover.conf >/dev/null
sudo systemctl restart chrony
if sudo ufw status 2>/dev/null | grep -q '^Status: active'; then
  sudo ufw allow from "$subnet" to any port 123 proto udp comment 'NTP for the rover'
fi

# --- rover: systemd-timesyncd uses the laptop ---
# The rover's DHCP server (systemd-networkd) derives the address from the client's MAC, so the
# laptop keeps the same address. If it ever changes, run this script again.
ssh -t "$rover" "
  set -e
  sudo mkdir -p /etc/systemd/timesyncd.conf.d
  printf '[Time]\nNTP=$laptop_ip\n' | sudo tee /etc/systemd/timesyncd.conf.d/laptop.conf >/dev/null
  sudo systemctl restart systemd-timesyncd
"

echo "waiting for the rover to sync (up to 60 s)"
for _ in $(seq 12); do
  sleep 5
  if ssh -o BatchMode=yes "$rover" timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes; then
    offset=$(( $(ssh -o BatchMode=yes "$rover" date +%s) - $(date +%s) ))
    echo "rover synchronized to the laptop (offset ${offset} s)"
    exit 0
  fi
done
echo "rover not synchronized yet: on the rover, check 'timedatectl timesync-status'" >&2
exit 1

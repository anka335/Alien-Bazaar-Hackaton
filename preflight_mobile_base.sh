#!/usr/bin/env bash
# Pre-flight of Spectacles teleop on the mobile base (#45, #58), simulated arm and rover stand-in
# only: lens stand-in -> robot bridge -> Twist on /leo/cmd_vel -> zero on release.
#
#   ./preflight_mobile_base.sh
#
# Runs from whichever checkout it is in: it sources that checkout's ros2_ws/install (colcon build
# --symlink-install first). Everything runs on the private domain ROS_DOMAIN_ID=77 with
# localhost-only discovery, the lens socket on 127.0.0.1:9110, never the rover's domain, never
# driver_sim:=false, never ngrok or the `ros` tmux session.
#
#   1. all robot pytest;
#   2. python3 -m cloth_task.preflight_mobile_base: a refused launch (base_max_vx:=0.5), then the
#      stack (hardware:=real driver_sim:=true enable_motors:=true run_task:=false
#      spectacles:=true rover:=true spectacles_port:=9110 use_rviz:=false), rover_standin and
#      the /joint_states gap monitor, the lens stand-in scenario, and the nine checks of #45;
#   3. writes preflight.log here: the commit, the date, one line per check, the pytest summary.
#
# Exits non-zero on any failure. Every process it starts has its own process group, listed in
# preflight_run/pgids; the trap stops those groups (SIGINT, then SIGKILL) on any exit. Logs of
# the run are in preflight_run/.
set -o pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
install="$repo/ros2_ws/install"
run_dir="$repo/preflight_run"
log="$repo/preflight.log"
pgids="$run_dir/pgids"
port=9110

teardown() {
  [[ -s "$pgids" ]] || return 0
  local pg alive
  while read -r pg; do kill -INT -- "-$pg" 2>/dev/null; done <"$pgids"
  for _ in $(seq 1 600); do # the simulated arm parks on SIGINT: up to 60 s
    alive=false
    while read -r pg; do kill -0 -- "-$pg" 2>/dev/null && alive=true; done <"$pgids"
    $alive || break
    sleep 0.1
  done
  while read -r pg; do kill -KILL -- "-$pg" 2>/dev/null; done <"$pgids"
  : >"$pgids"
}
trap teardown EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! -f "$install/setup.bash" ]]; then
  echo "no build at $install: run colcon build --symlink-install in $repo/ros2_ws first" >&2
  exit 1
fi
if ss -Hltn "sport = :$port" 2>/dev/null | grep -q .; then
  echo "port $port is in use: another pre-flight is running?" >&2
  exit 1
fi

rm -rf -- "$run_dir"
mkdir -p "$run_dir"
: >"$pgids"

export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS
source /opt/ros/jazzy/setup.bash
source "$install/setup.bash"
# this checkout's own cloth_task first, also for modules newer than the build
export PYTHONPATH="$repo/ros2_ws/src/cloth_task${PYTHONPATH:+:$PYTHONPATH}"

commit="$(git -C "$repo" rev-parse HEAD)"
branch="$(git -C "$repo" rev-parse --abbrev-ref HEAD)"
started="$(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "pre-flight of $commit ($branch) in $repo"

echo "1. robot pytest"
(cd "$repo" && python3 -m pytest ros2_ws/src/cloth_task/test -p no:cacheprovider) \
  >"$run_dir/pytest.log" 2>&1
pytest_rc=$?
pytest_summary="$(grep -E '(passed|failed|error)' "$run_dir/pytest.log" | tail -n 1)"
echo "   ${pytest_summary:-no summary} (exit $pytest_rc)"

echo "2. stack, stand-ins and checks on ROS_DOMAIN_ID=$ROS_DOMAIN_ID, port $port"
(cd "$repo" && python3 -m cloth_task.preflight_mobile_base --repo "$repo" --run-dir "$run_dir" \
  --pgid-file "$pgids" --results "$run_dir/results.txt")
checks_rc=$?

if [[ $pytest_rc -eq 0 && $checks_rc -eq 0 ]]; then verdict=PASS; else verdict=FAIL; fi
{
  echo "commit $commit ($branch)"
  echo "date $started"
  if [[ -s "$run_dir/results.txt" ]]; then
    cat "$run_dir/results.txt"
  else
    echo "check run: FAIL (no check results, exit $checks_rc)"
  fi
  echo "pytest: ${pytest_summary:-no summary} (exit $pytest_rc)"
  echo "pre-flight: $verdict"
} >"$log"
echo "3. $log: pre-flight $verdict"
if [[ $verdict == PASS ]]; then exit 0; fi
exit 1

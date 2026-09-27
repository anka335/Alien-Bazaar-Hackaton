#!/usr/bin/env bash
# Teleoperate the Leo Rover's base from the Spectacles lens: the bridge on :9100 and the ngrok
# tunnel the lens connects to. Run from the repo root on the rover (or a laptop on its Wi-Fi).
#
#   scripts/spectacles_base.sh                  # bridge + ngrok on the lens's domain
#   NGROK=0 scripts/spectacles_base.sh          # bridge only (ngrok runs elsewhere)
#   ROSBRIDGE=ws://127.0.0.1:9090 scripts/spectacles_base.sh
#
# Extra arguments go to the bridge (e.g. --max-vx 0.3). Ctrl+C stops both; the rover stops.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${PORT:-9100}"
ROSBRIDGE="${ROSBRIDGE:-ws://10.0.0.1:9090}"
NGROK_DOMAIN="${NGROK_DOMAIN:-blabber-routing-crop.ngrok-free.dev}"  # baked into the lens scene

if command -v uv >/dev/null; then
  run=(uv run python)
else
  run=(python3)
  export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
fi

ngrok_pid=""
cleanup() { [ -n "$ngrok_pid" ] && kill "$ngrok_pid" 2>/dev/null || true; }
trap cleanup EXIT

if [ "${NGROK:-1}" = 1 ]; then
  if ! command -v ngrok >/dev/null; then
    echo "ngrok not found: install it and run 'ngrok config add-authtoken <token>' (the lens's account)," >&2
    echo "or start with NGROK=0 if the tunnel runs elsewhere." >&2
    exit 1
  fi
  # --url is ngrok >= 3.16; older agents take --domain
  if ngrok http --help 2>&1 | grep -q -- '--url'; then flag=--url; else flag=--domain; fi
  ngrok http "$flag=$NGROK_DOMAIN" "$PORT" --log=stdout --log-level=warn > /tmp/spectacles-ngrok.log 2>&1 &
  ngrok_pid=$!
  sleep 3
  if ! kill -0 "$ngrok_pid" 2>/dev/null; then
    echo "ngrok exited:" >&2
    cat /tmp/spectacles-ngrok.log >&2
    echo "(ERR_NGROK_334 = the domain is online elsewhere: stop the other ngrok first)" >&2
    exit 1
  fi
  echo "ngrok: wss://$NGROK_DOMAIN -> 127.0.0.1:$PORT (log /tmp/spectacles-ngrok.log)"
fi

"${run[@]}" -m sorter.spectacles --rosbridge "$ROSBRIDGE" --port "$PORT" "$@"

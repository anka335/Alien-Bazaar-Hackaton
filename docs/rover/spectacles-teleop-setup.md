# Setup for an agent: Spectacles teleop of the Leo base on this machine

This is for an AI agent, or a person, setting up the rover's computer so the Snap Spectacles lens can drive the Leo Rover's base. [spectacles-teleop.md](spectacles-teleop.md) says what the bridge does. When you finish, the bridge and its ngrok tunnel run in tmux session `spectacles`, and the human only opens the lens on the glasses.

Work through the steps in order. Each step has a check; don't move on until it passes. **Ask the human** where a step says so; never guess those answers.

## Rules

- **Wheels:** the rover must not move unless the human has said the wheels are clear. Only step 6 moves it.
- **ngrok token:** never commit it, write it in a file in the repo, or print it in a log. It goes only into ngrok's own config (`ngrok config add-authtoken`).
- **Nav stack:** don't run the bridge while the nav stack (`sorter.nav ... --real`) drives the rover. Ask the human to stop it.
- **Limits:** don't change the speed limits unless the human asks. The defaults are +0.20 / −0.10 m/s and 0.6 rad/s.
- **Scope:** don't install system packages except ngrok and Python's venv support, and don't change network settings.

## 1. Where am I

```bash
hostname; uname -m; ip -br a; python3 --version
```

- **This machine is the rover** if an interface has `10.0.0.1` (LeoOS on a Raspberry Pi, usually `aarch64`). Use `ROSBRIDGE=ws://127.0.0.1:9090`.
- **This is a laptop on the rover's Wi-Fi** if its address is `10.0.0.x` with x not 1. Use `ROSBRIDGE=ws://10.0.0.1:9090` (the script's default).
- **Anything else:** stop and ask the human to join this machine to the rover's Wi-Fi, `LeoRover-…`.

Python must be 3.9 or newer.

The machine also needs internet access (for `git`, `pip` and ngrok). Check with `curl -sI https://github.com | head -1`. If there is none, ask the human. On the rover, internet usually comes from a second link, such as Ethernet or phone tethering.

## 2. The code

In the repo (`git clone https://github.com/anka335/Alien-Bazaar-Hackaton.git` if it isn't here):

```bash
git fetch origin
git status --short          # if there are local changes, ask the human before switching branch
git checkout feat/spectacles-leo-base
git pull
```

Check: `ls src/sorter/spectacles scripts/spectacles_base.sh` lists them.

## 3. Python

**With uv** (`command -v uv`), `uv sync` is enough. The script then runs through `uv run`.

**Without uv**, the bridge needs only `websockets` 12 or newer:

```bash
python3 -m venv ~/.venvs/spectacles && ~/.venvs/spectacles/bin/pip install 'websockets>=12' pytest
```

If `venv` is missing, ask the human for `sudo apt install python3-venv`. Then put `~/.venvs/spectacles/bin` first on `PATH` in the shell and the tmux session that run the bridge. The script uses `python3` when uv is absent.

Check (with uv, prefix `uv run`):

```bash
PYTHONPATH=src python3 -m pytest -q tests/spectacles
```

This must print 14 passed. The tests use fakes, so nothing moves.

## 4. The rover's rosbridge

```bash
PYTHONPATH=src python3 - <<'EOF'
import json, os, time
from websockets.sync.client import connect
url = os.environ.get("ROSBRIDGE", "ws://10.0.0.1:9090")
with connect(url, open_timeout=5) as ws:
    ws.send(json.dumps({"op": "subscribe", "topic": "merged_odom", "type": "nav_msgs/msg/Odometry"}))
    ws.recv(timeout=3); print("odometry OK from", url)
EOF
```

Run it with `ROSBRIDGE=...` from step 1.

- **Connection refused or timeout:** the rover is off, or its ROS isn't up yet. Ask the human to check the rover is on, and retry after about 1 minute.
- **`TimeoutError` on `recv`:** rosbridge is up but `merged_odom` is silent. Try `firmware/wheel_states` with type `leo_msgs/msg/WheelStates`. If that arrives, the bridge still works, because it counts either topic. If neither arrives, the rover's namespace may differ: ask the human, or list the topics with `{"op": "call_service", "service": "/rosapi/topics"}`.

## 5. ngrok

The lens connects to `wss://blabber-routing-crop.ngrok-free.dev`, which is baked into the lens. That domain belongs to one ngrok account and can be online in only one place at a time.

1. **Install it** if `command -v ngrok` finds nothing: `sudo snap install ngrok` where snap exists, or the Linux build for `uname -m` from https://ngrok.com/download (`arm64` on a Pi). Ask the human before using `sudo`.
2. **Log in:** `ngrok config check` must pass with a token. If it doesn't, **ask the human for the authtoken** of the account that owns the domain, then run `ngrok config add-authtoken <token>`.
3. **One place only:** make sure no other machine serves the domain. `ERR_NGROK_334` in step 7 means it's online elsewhere; ask the human to stop that ngrok.

Check: `ngrok version` works.

## 6. Dry run on the rover

Start the bridge without the tunnel, in the background:

```bash
NGROK=0 ROSBRIDGE=<from step 1> scripts/spectacles_base.sh > /tmp/spectacles-bridge.log 2>&1 &
sleep 4; cat /tmp/spectacles-bridge.log
```

The log must show `rosbridge ... connected` and `rover present (odometry)`.

**Ask the human whether the wheels are clear.** Only after a yes, drive forward at 0.1 m/s for 1 s:

```bash
PYTHONPATH=src python3 -m sorter.spectacles probe --vx 0.1 --seconds 1
```

- The probe prints `base: 'driving'`, then `'idle'`.
- The bridge log shows `base driving` and then `base idle`.
- The human confirms the rover moved about 10 cm forward.

Then stop the bridge: `kill %1` (or `pkill -f "m sorter.spectacles"`). Its log ends with `server closed`.

## 7. Run it for the glasses

```bash
tmux new-session -d -s spectacles "cd $PWD && ROSBRIDGE=<from step 1> scripts/spectacles_base.sh; bash"
sleep 6; tmux capture-pane -p -t spectacles | tail -8
```

Check that these lines appear:

- `ngrok: wss://blabber-routing-crop.ngrok-free.dev -> 127.0.0.1:9100`
- `lens WebSocket on ws://0.0.0.0:9100`
- `rover present (odometry)`

Check the tunnel from outside:

```bash
PYTHONPATH=src python3 - <<'EOF'
import json
from websockets.sync.client import connect
with connect("wss://blabber-routing-crop.ngrok-free.dev", open_timeout=10) as ws:
    print(json.loads(ws.recv(timeout=3)))
EOF
```

It prints a `status` with `base: 'idle'`. Opening this test socket counts as a lens connection, so it's normal for the log to show `lens connected` and then `lens disconnected`.

Then tell the human: **"Ready: open the lens on the glasses. The left hand pinches to drive; open it to stop."** When the lens connects, the tmux log shows `lens connected from ...`.

## Stop

Run `tmux send-keys -t spectacles C-c`. The bridge sends zero velocity and closes ngrok. The firmware also stops the wheels 0.5 s after the last command.

## Troubleshooting

| Symptom | Cause, and what to do |
| --- | --- |
| `ngrok exited ... ERR_NGROK_334` | The domain is online elsewhere. Ask the human to stop the other ngrok. |
| `ERR_NGROK_4018` or an auth error | No token or the wrong token. Ask the human for it (step 5). |
| `rover ABSENT: no odometry` | rosbridge is up but odometry is silent or delayed over 0.5 s. Repeat step 4. On a laptop, weak Wi-Fi to the rover also causes this; move closer. |
| Lens HUD says bridge silent, and the log shows no `lens connected` | The tunnel isn't up, or points elsewhere. Run the outside check in step 7. |
| `lens connected`, but the base stays `idle` while pinching | The pinch was held across a stop, a reconnect or the rover coming back. Open the hand once, then pinch again. |
| The rover moves in the wrong direction | Stop at once (`C-c`) and tell the human. Don't change signs in code. |

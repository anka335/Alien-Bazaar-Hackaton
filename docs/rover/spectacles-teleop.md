# Spectacles teleop of the Leo Rover's base

The Snap Spectacles lens drives the real Leo Rover. The left hand is the joystick: pinch thumb and index, then move the hand. Forward and back set the speed, and left and right set the turn. Opening the hand stops the rover. The wire is protocol v1 (`protocol.md` in the lens repo). This machine has no arm, so the right hand does nothing and the lens's HUD shows the arm as `holding`.

```
lens --wss--> ngrok --> ws://0.0.0.0:9100  python -m sorter.spectacles --rosbridge--> cmd_vel (rover)
                                                                         <---------- merged_odom
```

No ROS install is needed. It uses LeoOS's rosbridge on port 9090, like `sorter.nav.real_leo`.

## Run

An agent setting up a new machine follows [spectacles-teleop-setup.md](spectacles-teleop-setup.md), which checks every step.

Once per machine:

1. `uv sync`, as for the rest of the repo. Without uv, `python3` with `websockets` also works.
2. Install ngrok, then run `ngrok config add-authtoken <token>` with the account that owns `blabber-routing-crop.ngrok-free.dev`. That is the domain baked into the lens.

Each time:

```bash
git pull
scripts/spectacles_base.sh      # ngrok + the bridge; on the rover add ROSBRIDGE=ws://127.0.0.1:9090
```

Wait for `rover present (odometry)` in the log, then start the lens on the glasses.

- The domain can be online in only one place at a time. Stop any other ngrok on that domain first (for example on the arm laptop). Otherwise the script exits with `ERR_NGROK_334`.
- If the tunnel runs elsewhere, use `NGROK=0 scripts/spectacles_base.sh` and point that tunnel at this machine's port 9100.

To test without the glasses, in a second terminal (this drives the rover for 2 s at 0.1 m/s):

```bash
uv run python -m sorter.spectacles probe --vx 0.1 --seconds 2
```

## Limits and stops

The defaults are +0.20 m/s forward, 0.10 m/s back and 0.6 rad/s. Change them with `--max-vx`, `--max-reverse` and `--max-wz`. They are refused above the protocol's 0.35, 0.15 and 0.8.

The rover stops when:

- the pinch opens;
- no frame arrives for 0.3 s;
- the lens disconnects;
- no frame arrives for 2 s: this is a fault, and the pinch must be released first;
- no rover odometry arrives for 0.5 s;
- the process dies: the firmware stops after 0.5 s.

While nobody pinches, the bridge sends nothing, so the nav stack can still drive the rover. Don't run both at the same time.

# Manual arm tests

These programs exercise the reBot B601-RS manually. They are not part of the
automated pytest suite and may move real hardware.

Start in simulation:

```bash
cd rebot_b601
PYTHONNOUSERSITE=1 .venv/bin/python manual_tests/slow_left_right.py
```

The left/right test moves the TCP 20 mm to each side of the home position at a
5% speed scale. In the arm base frame, `+Y` is left and `-Y` is right. It uses
straight Cartesian paths and preserves the gripper's current approach axis.

To display the same simulated `Arm` instance in the virtual clone, add
`--viewer`. Open the printed URL before pressing Enter:

```bash
PYTHONNOUSERSITE=1 .venv/bin/python manual_tests/slow_left_right.py --viewer
```

If port 8765 is already occupied, choose another one, for example
`--viewer 8766`.

Only after checking the simulation and clearing the physical workspace, run:

```bash
PYTHONNOUSERSITE=1 .venv/bin/python manual_tests/slow_left_right.py --hardware
```

Real-hardware mode requires typing `MOVE` before torque is enabled. Keep a hand
at the physical power switch. `Ctrl+C` stops and holds the arm; normal completion
returns it home before disabling torque.

## RGB-D clothing-follow test

`clothing_follow.py` is a guarded ROS 2 integration test. Hardware mode first
performs the same small right/left sweep, then uses the D435i color image,
aligned depth, and the remote SAM3 `clothing` mask to center the garment
horizontally with 3 mm base-Y steps. Depth is used only as a clearance guard;
the arm never moves toward the garment. Run it inside the ROS 2 Jazzy camera
container with this repository mounted and the RealSense launch already
publishing its documented topics.

Set the SAM3 key and run preview mode first. Preview validates one synchronized
RGB-D observation and clothing mask; it does not create an arm object or open
CAN:

```bash
export SAM3_API_KEY='<key>'
PYTHONPATH=/repo/rebot_b601 \
  python3 /repo/rebot_b601/manual_tests/clothing_follow.py
```

The physical mode is deliberately not a hand-follow test. Put the garment on a
stand; no person may be inside the arm workspace. The initial sweep measures the
mask's image displacement, verifies that the wrist camera actually moved, and
infers which base-Y direction follows an image-right target. An optional
`--y-sign 1` or `--y-sign -1` asserts the expected result. Run:

```bash
PYTHONPATH=/repo/rebot_b601 \
  python3 /repo/rebot_b601/manual_tests/clothing_follow.py \
  --hardware --sweep-mm 5 --step-mm 3 --speed 0.02
```

Hardware mode requires typing `FOLLOW_CLOTHING`. It refuses a missing `can0`, a
CAN transmit queue below 100, missing RGB-D, color/depth desynchronization,
invalid depth, lost segmentation, an unverified sweep, an object closer than
350 mm, a step above 5 mm, or a total lateral offset above the configured limit (30 mm by default). A failure
stops the current move; ordinary vision failures return home before disabling.
An arm fault enters a recovery prompt that offers torque disable after the
operator confirms physical support. This path needs review against the
repository rule to disable only at the rest pose. Physical following has not
been verified.

Known limitation: the source checks color/depth timestamp skew but does not
reject an old cached pair when the camera stops publishing. Do not treat the
current hardware test as validated for loss of camera streaming.

"""Pose teaching: move the SO-101 by hand, save named joint poses to `config/rig.yaml`.

    uv run python -m sorter.arm.teach

`free` turns the motors off: HOLD THE ARM first, it falls. `lock` holds it where it is.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from sorter.arm import kinematics as kin
from sorter.arm.backend import create
from sorter.arm.controller import POSE_NAMES, So101Arm
from sorter.core.config import DEFAULT_CONFIG_DIR, load_config, update_yaml

HELP = f"""commands:
  show            joints (deg), fingertip xyz (mm), tilt from straight down, gripper
  free            motors off, to move the arm by hand. HOLD THE ARM FIRST.
  lock            motors on, holding the current position
  save <pose>     save the current joints as <pose>: {", ".join(POSE_NAMES)}
  go <pose>       move to a saved pose (motors on, half speed)
  grip <0..1>     gripper opening (0 closed, 1 open)
  poses           list saved poses
  quit            exit (asks whether to go to rest and turn the motors off)
Suggested order: home, look_bg, look_box, place_bg, bin_light, bin_dark, bin_colored, rest.
Look poses: camera straight down over the zone, the whole zone in view (check with `snap`
in `python -m sorter.calibration.setup`), not lower than needed."""


def describe(arm: So101Arm) -> str:
    q = arm.read_q()
    T = kin.fk(q)
    x, y, z = arm.tcp(q)
    joints = " ".join(
        f"{n}={math.degrees(v):6.1f}" for n, v in zip(kin.JOINT_NAMES, q, strict=True)
    )
    return (
        f"{joints}\n  tip ({x:.0f}, {y:.0f}, {z:.0f}) mm, tilt {kin.tilt_deg(T):.0f}°, "
        f"gripper {arm.read_gripper():.2f}"
    )


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.arm.teach")
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    args = p.parse_args(argv)
    cfg = load_config(args.config_dir)
    rig = Path(args.config_dir) / "rig.yaml"
    arm = create(cfg)
    arm.connect()
    torque = False
    print(HELP)
    print(describe(arm))
    while True:
        try:
            line = input("teach> ").strip()
        except (EOFError, KeyboardInterrupt):
            line = "quit"
        cmd, *rest = line.split() or [""]
        try:
            if cmd == "show":
                print(describe(arm))
            elif cmd == "free":
                if input("Holding the arm? [y/N] ").strip().lower() == "y":
                    arm.disable()
                    torque = False
                    print("motors off")
            elif cmd == "lock":
                arm.start()
                torque = True
                print("holding")
            elif cmd == "save" and len(rest) == 1:
                name = rest[0]
                if name not in POSE_NAMES:
                    print(f"unknown pose; one of {POSE_NAMES}")
                    continue
                q = [round(float(v), 4) for v in arm.read_q()]
                arm.poses[name] = q
                update_yaml(rig, {"poses": {name: q}})
                print(f"saved {name} to {rig}")
            elif cmd == "go" and len(rest) == 1:
                if not torque:
                    arm.start()
                    torque = True
                arm.goto(rest[0], speed_scale=0.5)
                print(describe(arm))
            elif cmd == "grip" and len(rest) == 1:
                if not torque:
                    arm.start()
                    torque = True
                print(f"gripper {arm.set_gripper(float(rest[0])):.2f}")
            elif cmd == "poses":
                for name in POSE_NAMES:
                    q = arm.poses.get(name)
                    print(f"  {name:12s}", "—" if q is None else np.round(np.degrees(q), 1))
            elif cmd == "quit":
                if torque and input("Go to rest and turn the motors off? [y/N] ").lower() == "y":
                    arm.shutdown()
                elif arm.bus is not None:
                    arm.bus.close()
                return
            elif cmd:
                print(HELP)
        except Exception as e:
            print(f"error: {e}")


if __name__ == "__main__":
    main()

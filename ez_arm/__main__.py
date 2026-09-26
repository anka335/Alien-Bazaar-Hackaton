"""ez_arm CLI: steer the arm in 3D, look through the camera, run the T-shirt pick-and-place.

  python -m ez_arm [--sim] state | home | grip 0..1 | pose NAME | record NAME
  python -m ez_arm [--sim] go X Y Z [down|forward|free] | rel DX DY DZ | joints A1..A6
  python -m ez_arm snap                 # camera frame at the current pose → ez_arm/runs/
  python -m ez_arm detect [--j1 DEG]    # look pose + SAM3: where is the shirt (moves to `look`)
  python -m ez_arm pick [--place X Y Z] # the whole task
  python -m ez_arm repl                 # interactive: same commands, arm stays powered

Ctrl+C stops the motion and holds; on exit the arm goes home and the torque is released.
"""

from __future__ import annotations

import argparse
import logging
import os
import shlex
import sys

os.environ.setdefault("PYTHONNOUSERSITE", "1")
os.environ.setdefault("REBOT_Z_MIN", "0.005")  # grasping cloth: fingertips go to ~1 cm above the table

import cv2  # noqa: E402

from ez_arm.robot import ArmError, Robot  # noqa: E402

log = logging.getLogger("ez_arm")


def _f(xs):
    return [float(x) for x in xs]


def run_cmd(r: Robot, argv: list[str], args) -> None:
    cmd, rest = argv[0], argv[1:]
    if cmd == "state":
        for k, v in r.state().items():
            print(f"{k:18s} {v}")
    elif cmd == "go":
        x, y, z = _f(rest[:3])
        print(r.go(x, y, z, rest[3] if len(rest) > 3 else "down", linear="linear" in rest))
    elif cmd == "rel":
        print(r.rel(*_f(rest[:3])))
    elif cmd == "joints":
        print(r.joints(_f(rest[:6])))
    elif cmd == "pose":
        print(r.pose(rest[0]))
    elif cmd == "record":
        print(rest[0], r.record(rest[0]))
    elif cmd == "grip":
        print(r.grip(float(rest[0])))
    elif cmd == "roll":
        print(r.wrist_roll(float(rest[0])))
    elif cmd == "home":
        print(r.home())
    elif cmd == "plan":
        print(r.plan(*_f(rest[:3]), rest[3] if len(rest) > 3 else "down"))
    elif cmd == "snap":
        from ez_arm.pick_place import OUT_DIR

        rgbd, _ = r.snap()
        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, "snap.jpg")
        cv2.imwrite(path, rgbd.color)
        print(path)
    elif cmd in ("detect", "pick"):
        from ez_arm.pick_place import PickConfig, PickPlace

        det = None
        if args.sim_target is None:
            from ez_arm.detect import ShirtDetector

            det = ShirtDetector(prompts=args.prompts)
        cfg = PickConfig(speed=args.speed)
        if args.place:
            cfg.place_xyz = tuple(args.place)
        pp = PickPlace(r, det, cfg, sim_target=args.sim_target)
        if cmd == "detect":
            r.pose("ready")
            t, _ = pp.look_and_find([args.j1] if args.j1 is not None else None, tag="detect")
            print("no shirt" if t is None else f"shirt at {t.xyz.round(3).tolist()} yaw {t.yaw:.2f} rad")
        else:
            ok = pp.run()
            print("SUCCESS" if ok else "FAILED")
    else:
        raise ArmError(f"unknown command {cmd!r}")


def repl(r: Robot, args) -> None:
    print("commands: state go rel joints pose record grip roll home plan snap detect pick quit")
    while True:
        try:
            line = input("ez_arm> ").strip()
        except EOFError:
            break
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        try:
            run_cmd(r, shlex.split(line), args)
        except KeyboardInterrupt:
            r.stop()
            print("stopped, holding")
        except (ArmError, ValueError, IndexError) as e:
            print("refused:", e)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulated arm, no CAN")
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--read-only", action="store_true", help="motors stay off; only state/snap")
    ap.add_argument("--place", type=float, nargs=3, metavar=("X", "Y", "Z"))
    ap.add_argument("--j1", type=float, default=None, help="detect: only this joint1 angle")
    ap.add_argument("--prompts", nargs="+", default=["t-shirt", "clothing"])
    ap.add_argument("--sim-target", type=float, nargs=4, metavar=("X", "Y", "Z", "YAW"), help="skip the camera")
    ap.add_argument("--viewer", type=int, default=0, help="3D viewer port (e.g. 8765)")
    ap.add_argument("cmd", nargs=argparse.REMAINDER)
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    if not args.cmd:
        ap.print_help()
        return
    camera = not args.no_camera and args.sim_target is None and not (args.sim and args.cmd[0] not in ("snap", "detect", "pick"))
    r = Robot(simulate=args.sim, enable=not args.read_only, camera=camera, speed=args.speed)
    if args.viewer:
        from rebot_b601.viewer import start_viewer

        start_viewer(r.arm, port=args.viewer)
    try:
        if args.cmd[0] == "repl":
            repl(r, args)
        else:
            run_cmd(r, args.cmd, args)
    except KeyboardInterrupt:
        r.stop()
        print("interrupted: holding, then going home")
    except ArmError as e:
        print("refused:", e, file=sys.stderr)
    finally:
        r.close(go_home=not args.read_only and r.arm.status().get("fault") is None)


if __name__ == "__main__":
    main()

"""Command line tools:  python -m rebot_b601 <command> ...

  ik      x y z        offline inverse kinematics (no hardware, always safe)
  state                read joint angles / TCP / temperatures (motors stay limp)
  xyz     x y z        move the TCP to a position (asks for confirmation)
  joints  a1 .. a6     move joints [deg]
  home                 go to the zero pose and switch torque off
  grip    0..1         set the gripper
  fetch-assets         download the CAD meshes + three.js for the 3D viewer (~37 MB)
  repl                 interactive session (keeps the arm energised between moves)

Add --simulate (or REBOT_DRY_RUN=1) to run against a fake arm.
"""

from __future__ import annotations

import argparse
import shlex
import sys
import time

import numpy as np

from . import config as C
from . import kinematics as K
from .arm import Arm, ArmError, path_duration
from .viewer import start_viewer


def _fmt(a, n=1) -> str:
    return "[" + ", ".join(f"{x:.{n}f}" for x in a) + "]"


def _approach_from(args):
    if getattr(args, "vector", None):
        return list(args.vector)
    return args.approach


def _add_motion_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--simulate", action="store_true", help="use a fake arm, no CAN access")
    p.add_argument("--speed", type=float, default=C.DEFAULT_SPEED_SCALE, help=f"speed scale (default {C.DEFAULT_SPEED_SCALE}, capped at {C.MAX_SPEED_SCALE})")
    p.add_argument("--yes", "-y", action="store_true", help="do not ask for confirmation")
    p.add_argument("--viewer", type=int, nargs="?", const=8765, default=0, metavar="PORT", help="serve the 3D twin on this port (default 8765)")


def _confirm(args, text: str) -> bool:
    print(text)
    if args.yes:
        return True
    return input("Proceed? [y/N] ").strip().lower() in ("y", "yes")


def cmd_ik(args) -> int:
    seed = np.radians(args.seed) if args.seed else np.array([0, 60, 90, 0, 0, 0]) * np.pi / 180
    app = _approach_from(args)
    r = K.solve_ik([args.x, args.y, args.z], app, seed, C.JOINT_LIMITS_RAD, z_min=C.Z_MIN)
    p, R = K.fk(r.q)
    print(f"target xyz      : ({args.x:.3f}, {args.y:.3f}, {args.z:.3f}) m   approach: {app or 'free'}")
    print(f"solved          : {r.success}  ({r.message})")
    print(f"joints [deg]    : {_fmt(np.degrees(r.q))}")
    print(f"FK check xyz    : ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) m   error {r.pos_error * 1000:.2f} mm")
    print(f"approach axis   : {_fmt(R[:, 0], 3)}" + ("" if r.approach_error_deg is None else f"   error {r.approach_error_deg:.2f} deg"))
    print(f"from seed [deg] : {_fmt(np.degrees(seed))}")
    return 0 if r.success else 1


def cmd_state(args) -> int:
    arm = Arm()
    try:
        s = arm.connect(enable=False, simulate=args.simulate)
    except ArmError as e:
        print(f"error: {e}")
        return 1
    try:
        print(f"joints [deg] : {_fmt(s['joints_deg'], 2)}")
        print(f"TCP xyz [m]  : {_fmt(s['tcp_xyz_m'], 4)}")
        print(f"approach     : {_fmt(s['approach_axis'], 3)}")
        print(f"gripper      : opening {s['gripper_opening']}  (motor {s['gripper_motor_deg']} deg)")
        print(f"max temp [C] : {s['max_motor_temp_c']}")
    finally:
        arm.disconnect(go_home=False)
    return 0


def _finish(arm: Arm, args) -> None:
    after = getattr(args, "after", "home")
    if after == "leave":
        print("leaving the motors energised (holding is not guaranteed once this process exits!)")
        arm._stop_thread.set()
        return
    if after == "home":
        print("returning to the home pose and switching torque off ...")
        arm.disconnect(go_home=True, speed_scale=args.speed)
    else:
        arm.disconnect(go_home=False)


def _open_viewer(arm: Arm, args) -> None:
    if getattr(args, "viewer", 0):
        _, url = start_viewer(arm, args.viewer)
        print(f"3D twin: {url}")


def _run_motion(args, action) -> int:
    arm = Arm()
    _open_viewer(arm, args)
    try:
        arm.connect(enable=True, simulate=args.simulate)
    except ArmError as e:
        print(f"error: {e}")
        return 1
    rc = 0
    try:
        rc = action(arm)
    except ArmError as e:
        print(f"error: {e}")
        rc = 1
    except KeyboardInterrupt:
        arm.stop()
        print("\ninterrupted: holding position")
        rc = 130
    finally:
        try:
            _finish(arm, args)
        except (ArmError, KeyboardInterrupt) as e:
            print(f"warning while shutting down: {e}. The torque may still be ON; check the arm.")
    return rc


def cmd_xyz(args) -> int:
    app = _approach_from(args)

    def action(arm: Arm) -> int:
        plan = arm.plan_xyz(args.x, args.y, args.z, app, args.linear)
        cur = arm.status()
        text = (
            f"current  xyz {_fmt(cur['tcp_xyz_m'], 3)} m, joints {_fmt(cur['joints_deg'])} deg\n"
            f"target   xyz ({args.x:.3f}, {args.y:.3f}, {args.z:.3f}) m, approach {app or 'free'}, "
            f"{'linear' if args.linear else 'joint-space'} path\n"
            f"solution joints {_fmt(plan['joints_deg'])} deg (IK error {plan['error_mm']} mm)"
        )
        if not _confirm(args, text):
            print("cancelled")
            return 1
        r = arm.move_to_xyz(args.x, args.y, args.z, app, args.linear, args.speed)
        print(f"done in {r['duration_s']} s: reached {_fmt(r['reached_xyz_m'], 4)} m (error {r['error_mm']} mm)")
        if args.hold > 0:
            print(f"holding for {args.hold:g} s ...")
            time.sleep(args.hold)
        return 0

    return _run_motion(args, action)


def cmd_joints(args) -> int:
    def action(arm: Arm) -> int:
        if not _confirm(args, f"move joints to {_fmt(args.angles)} deg"):
            return 1
        r = arm.move_joints(args.angles, args.speed)
        print(f"done: joints {_fmt(r['joints_deg'], 2)} deg, TCP {_fmt(r['tcp_xyz_m'], 4)} m")
        if args.hold > 0:
            time.sleep(args.hold)
        return 0

    return _run_motion(args, action)


def cmd_home(args) -> int:
    args.after = "off"

    def action(arm: Arm) -> int:
        if not _confirm(args, "go to the zero pose, then switch the torque off"):
            return 1
        arm.home(args.speed)
        return 0

    return _run_motion(args, action)


def cmd_grip(args) -> int:
    def action(arm: Arm) -> int:
        print(arm.set_gripper(args.opening))
        time.sleep(args.hold)
        return 0

    args.after = "leave"   # do not move the arm just because the gripper was used
    return _run_motion(args, action)


def cmd_repl(args) -> int:
    arm = Arm()
    _open_viewer(arm, args)
    try:
        arm.connect(enable=True, simulate=args.simulate)
    except ArmError as e:
        print(f"error: {e}")
        return 1
    print(__doc__.split("Add --simulate")[0])
    print("repl commands: xyz X Y Z [down|forward|up|free] [linear] | rel DX DY DZ | plan X Y Z [approach] | joints a1..a6")
    print("               home | grip 0..1 | state | stop | off (EMERGENCY torque off) | quit (home + torque off)")
    print("Ctrl+C during a move stops it and holds the pose.\n")
    try:
        while True:
            try:
                line = input("rebot> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = shlex.split(line)
            cmd, rest = parts[0].lower(), parts[1:]
            try:
                if cmd in ("quit", "exit", "q"):
                    break
                elif cmd == "state":
                    print(arm.status())
                elif cmd == "stop":
                    print(arm.stop())
                elif cmd == "off":
                    print(arm.emergency_disable())
                    return 0
                elif cmd == "home":
                    print(arm.home(args.speed))
                elif cmd == "grip":
                    print(arm.set_gripper(float(rest[0])))
                elif cmd == "joints":
                    print(arm.move_joints([float(v) for v in rest], args.speed))
                elif cmd in ("xyz", "plan"):
                    x, y, z = (float(v) for v in rest[:3])
                    flags = [w.lower() for w in rest[3:]]
                    linear = "linear" in flags
                    app = next((w for w in flags if w in ("down", "up", "forward", "free")), "free")
                    if cmd == "plan":
                        print(arm.plan_xyz(x, y, z, app, linear))
                    else:
                        print(arm.move_to_xyz(x, y, z, app, linear, args.speed))
                elif cmd == "rel":
                    dx, dy, dz = (float(v) for v in rest[:3])
                    print(arm.move_relative(dx, dy, dz, speed_scale=args.speed))
                else:
                    print("unknown command")
            except (ValueError, IndexError):
                print("bad arguments")
            except ArmError as e:
                print(f"refused/aborted: {e}")
            except KeyboardInterrupt:
                arm.stop()
                print("\nstopped, holding position")
    finally:
        if arm.connected and arm.status().get("torque_enabled"):
            try:
                print("returning home and switching torque off ...")
                arm.disconnect(go_home=True, speed_scale=args.speed)
            except (ArmError, KeyboardInterrupt) as e:
                print(f"warning: {e}. Torque may still be ON.")
        elif arm.connected:
            arm.disconnect(go_home=False)
    return 0


def cmd_fetch_assets(args) -> int:
    from .assets import fetch_assets

    try:
        fetch_assets(force=args.force)
    except Exception as e:
        print(f"error: {e}")
        return 1
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="rebot_b601", description="reBot B601-RS tools")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def approach_args(p):
        p.add_argument("--approach", default="free", choices=["free", "down", "up", "forward"], help="direction of the gripper axis")
        p.add_argument("--vector", type=float, nargs=3, metavar=("AX", "AY", "AZ"), help="custom approach direction (overrides --approach)")

    p = sub.add_parser("ik", help="offline IK")
    p.add_argument("x", type=float); p.add_argument("y", type=float); p.add_argument("z", type=float)
    approach_args(p)
    p.add_argument("--seed", type=float, nargs=6, metavar="DEG", help="starting joints [deg]")
    p.set_defaults(fn=cmd_ik)

    p = sub.add_parser("state", help="read the arm state")
    p.add_argument("--simulate", action="store_true")
    p.set_defaults(fn=cmd_state)

    p = sub.add_parser("xyz", help="move the TCP to x y z [m]")
    p.add_argument("x", type=float); p.add_argument("y", type=float); p.add_argument("z", type=float)
    approach_args(p)
    p.add_argument("--linear", action="store_true", help="straight-line path")
    p.add_argument("--hold", type=float, default=3.0, help="seconds to stay at the target before finishing")
    p.add_argument("--after", choices=["home", "off", "leave"], default="home", help="what to do when done (default: go home, torque off)")
    _add_motion_args(p)
    p.set_defaults(fn=cmd_xyz)

    p = sub.add_parser("joints", help="move joints [deg]")
    p.add_argument("angles", type=float, nargs=6)
    p.add_argument("--hold", type=float, default=3.0)
    p.add_argument("--after", choices=["home", "off", "leave"], default="home")
    _add_motion_args(p)
    p.set_defaults(fn=cmd_joints)

    p = sub.add_parser("home", help="go to the zero pose, torque off")
    _add_motion_args(p)
    p.set_defaults(fn=cmd_home)

    p = sub.add_parser("grip", help="gripper opening 0..1")
    p.add_argument("opening", type=float)
    p.add_argument("--hold", type=float, default=1.0)
    _add_motion_args(p)
    p.set_defaults(fn=cmd_grip)

    p = sub.add_parser("fetch-assets", help="download CAD meshes + three.js for the 3D viewer")
    p.add_argument("--force", action="store_true", help="download again even if present")
    p.set_defaults(fn=cmd_fetch_assets)

    p = sub.add_parser("repl", help="interactive session")
    _add_motion_args(p)
    p.set_defaults(fn=cmd_repl)
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.fn(args) or 0)


if __name__ == "__main__":
    sys.exit(main())

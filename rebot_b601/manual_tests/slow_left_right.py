#!/usr/bin/env python3
"""Slowly move the reBot TCP right and left along the base-frame Y axis.

Simulation is the default. Pass --hardware and confirm explicitly to use CAN.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running this file directly without installing the local package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rebot_b601.arm import Arm, ArmError  # noqa: E402
from rebot_b601.viewer import start_viewer  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="use the real arm on can0 (default: simulator)",
    )
    parser.add_argument(
        "--distance-mm",
        type=float,
        default=20.0,
        help="distance from center to each side (default: 20 mm, maximum: 50 mm)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.05,
        help="joint-speed scale (default: 0.05, maximum for this test: 0.10)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="number of right-left cycles (default: 1, maximum: 5)",
    )
    parser.add_argument(
        "--viewer",
        type=int,
        nargs="?",
        const=8765,
        default=0,
        metavar="PORT",
        help="show this test in the 3D viewer (default port: 8765)",
    )
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 1.0 <= args.distance_mm <= 50.0:
        raise ValueError("--distance-mm must be between 1 and 50")
    if not 0.01 <= args.speed <= 0.10:
        raise ValueError("--speed must be between 0.01 and 0.10")
    if not 1 <= args.cycles <= 5:
        raise ValueError("--cycles must be between 1 and 5")


def confirm_hardware(args: argparse.Namespace) -> bool:
    if not args.hardware:
        return True
    print("WARNING: this will enable and move the physical reBot B601-RS.")
    print("Clear the workspace and keep a hand at the physical power switch.")
    print(
        f"Motion: home, then {args.distance_mm:g} mm right/left at "
        f"speed scale {args.speed:g}."
    )
    return input("Type MOVE to continue: ").strip() == "MOVE"


def main() -> int:
    args = parse_args()
    try:
        validate(args)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if not confirm_hardware(args):
        print("Cancelled; torque was not enabled.")
        return 1

    arm = Arm(max_speed_scale=0.10)
    completed = False
    try:
        if args.viewer:
            _, viewer_url = start_viewer(arm, args.viewer)
            print(f"3D twin: {viewer_url}")
            input("Open the URL, then press Enter to begin the test: ")

        print("Connecting to", "hardware ..." if args.hardware else "simulator ...")
        arm.connect(enable=True, simulate=not args.hardware)

        print("Moving slowly to home ...")
        arm.home(speed_scale=args.speed)
        state = arm.status()
        center = state["tcp_xyz_m"]
        approach = state["approach_axis"]
        offset = args.distance_mm / 1000.0

        right = [center[0], center[1] - offset, center[2]]
        left = [center[0], center[1] + offset, center[2]]

        # Verify the endpoints without moving before beginning the sequence.
        arm.plan_xyz(*right, approach=approach, linear=True)
        arm.plan_xyz(*left, approach=approach, linear=True)

        for cycle in range(1, args.cycles + 1):
            print(f"Cycle {cycle}/{args.cycles}: right")
            arm.move_to_xyz(*right, approach=approach, linear=True, speed_scale=args.speed)
            print(f"Cycle {cycle}/{args.cycles}: left")
            arm.move_to_xyz(*left, approach=approach, linear=True, speed_scale=args.speed)

        print("Returning to center ...")
        arm.move_to_xyz(*center, approach=approach, linear=True, speed_scale=args.speed)
        completed = True
        return 0
    except KeyboardInterrupt:
        if arm.connected:
            arm.stop()
        print("\nInterrupted: motion stopped and position held.", file=sys.stderr)
        return 130
    except ArmError as error:
        if arm.connected:
            arm.stop()
        print(f"Arm refused or aborted the test: {error}", file=sys.stderr)
        return 1
    finally:
        if arm.connected:
            try:
                if completed:
                    print("Returning home and disabling torque ...")
                    arm.disconnect(go_home=True, speed_scale=args.speed)
                else:
                    print(
                        "Test did not complete; returning home before disabling torque ...",
                        file=sys.stderr,
                    )
                    arm.disconnect(go_home=True, speed_scale=args.speed)
            except (ArmError, KeyboardInterrupt) as error:
                print(
                    f"Shutdown failed: {error}. Torque may still be ON; check the arm.",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    raise SystemExit(main())

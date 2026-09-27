"""CLI: `python -m sorter run [--sim] [--mode load|unload|manual|calibrate] [--record]`;
`python -m sorter manual` is `run --mode manual`. The mode switches on the dashboard at any
time."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.core.types import OperatorMode


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="sorter")
    sub = parser.add_subparsers(dest="command", required=True)
    run_p = sub.add_parser("run", help="run the sorter")
    run_p.add_argument("--sim", action="store_true", help="simulate the camera and the arm")
    run_p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR, help="directory with *.yaml")
    run_p.add_argument("--no-dashboard", action="store_true", help="don't start the web server")
    run_p.add_argument("--autostart", action="store_true", help="send START right away")
    run_p.add_argument(
        "--mode",
        choices=[m.value for m in OperatorMode],
        default=OperatorMode.LOAD.value,
        help="the dashboard's operator mode to start in",
    )
    run_p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    run_p.add_argument(
        "--record",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="record the session to data/sessions/ (default: on without --sim)",
    )
    man_p = sub.add_parser("manual", help="run, starting in the manual mode (set up the rig)")
    man_p.add_argument("--sim", action="store_true", help="simulate the camera and the arm")
    man_p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR, help="directory with *.yaml")
    man_p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    man_p.add_argument(
        "--record",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="record the session to data/sessions/ (default: on without --sim)",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    from sorter.app import run

    manual = args.command == "manual"
    run(
        load_config(args.config_dir),
        sim=args.sim,
        dashboard=manual or not args.no_dashboard,
        autostart=not manual and args.autostart,
        mode=OperatorMode.MANUAL if manual else OperatorMode(args.mode),
        rig_file=Path(args.config_dir) / "rig.yaml",
        record=not args.sim if args.record is None else args.record,
    )


if __name__ == "__main__":
    main()

"""CLI: `python -m sorter run [--sim]`."""

from __future__ import annotations

import argparse
import logging

from sorter.core.config import DEFAULT_CONFIG_DIR, load_config


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="sorter")
    sub = parser.add_subparsers(dest="command", required=True)
    run_p = sub.add_parser("run", help="run the sorter")
    run_p.add_argument("--sim", action="store_true", help="simulate every component")
    run_p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR, help="directory with *.yaml")
    run_p.add_argument("--no-dashboard", action="store_true", help="don't start the web server")
    run_p.add_argument("--autostart", action="store_true", help="send START right away")
    run_p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    if args.command == "run":
        from sorter.app import run

        run(
            load_config(args.config_dir),
            sim=args.sim,
            dashboard=not args.no_dashboard,
            autostart=args.autostart,
        )


if __name__ == "__main__":
    main()

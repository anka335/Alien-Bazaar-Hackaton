"""The full mission on the simulator: `python -m sorter.mission [--seed 0] [--out DIR]`.

    uv run python -m sorter.mission --seed 0 --out data/mission/s0 --video
    uv run python -m sorter.mission --no-arm      # the driving only: fast

Prints every step as a JSON line, then the report.
"""

from __future__ import annotations

import argparse
import json
import logging

from sorter.core.config import DEFAULT_CONFIG_DIR
from sorter.mission.mission import run_mission


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.mission")
    p.add_argument("--scenario", default="mission")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--capacity", type=int, default=6, help="socks before the station")
    p.add_argument("--max-stops", type=int, default=8)
    p.add_argument("--detector", default="classic", help="classic | sam3 | seg")
    p.add_argument("--out", help="a directory: nav frames, mission.json, stop videos")
    p.add_argument("--video", action="store_true", help="an MP4 per stop (needs --out)")
    p.add_argument("--no-arm", action="store_true", help="no arm world: socks in reach count")
    p.add_argument("--config-dir", default=str(DEFAULT_CONFIG_DIR))
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(message)s")
    logging.getLogger("sorter.mission").setLevel(logging.INFO)
    report = run_mission(
        scenario=args.scenario,
        seed=args.seed,
        config_dir=args.config_dir,
        capacity=args.capacity,
        max_stops=args.max_stops,
        detector=args.detector,
        out=args.out,
        video=args.video,
        arm=not args.no_arm,
    )
    print(json.dumps(report.summary(), indent=1))


if __name__ == "__main__":
    main()

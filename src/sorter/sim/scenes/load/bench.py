"""The load benchmark (stage A, A7): N seeded scenes, the real load loop, headless and as fast as
the CPU allows; where every sock ends up.

    uv run python -m sorter.sim.scenes.load.bench [-n 50] [--socks 1-4] [--seed0 0] [--workers 4]

Each scene: `--socks` socks (a random count in the range) of random colors, scattered by the load
scene over the floor the arm reaches. The run goes until DONE, ERROR or `--time-limit` simulated
seconds. The report goes to data/bench/<run_id>/: `scenes.jsonl` (one line per scene) and
`summary.json`, and the summary is printed.
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

from sorter.arm.controller import in_polygon
from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.core.types import ColorClass, Command, OperatorMode, Phase, Zone

BENCH_DIR = Path("data/bench")


def run_scene(seed: int, n_socks: int, time_limit_s: float, config_dir: str) -> dict[str, Any]:
    """One seeded scene through the load loop; where each sock ended up."""
    from sorter.app import build_system
    from sorter.orchestrator.state_machine import StateMachine

    rng = np.random.default_rng(seed)
    colors = [ColorClass(c) for c in rng.choice([c.value for c in ColorClass], n_socks)]
    cfg = load_config(
        config_dir,
        overrides={
            "sim": {
                "realtime": 0,
                "seed": seed,
                "scenes": ["load"],
                "load": {"socks": [c.value for c in colors]},
            },
            "backends": dict.fromkeys(("camera", "arm"), "sim"),
            "arm": {"speed_scale": 1.4},
            "state_machine": {"save_runs": False},
        },
    )
    t0 = time.monotonic()
    # the socks where the rig says the arm picks from the floor
    cfg.sim.load.area, cfg.sim.load.zone_mm = "zone", list(cfg.zones[Zone.FLOOR].workspace_mm)
    system = build_system(cfg, sim=True)
    world = system.world
    assert world is not None
    zone = cfg.zones[Zone.FLOOR].workspace_mm
    reachable = []
    for it in world.items:
        c = world.vertices(it.id).mean(axis=0)
        reachable.append(in_polygon(float(c[0]), float(c[1]), zone))
    system.camera.start()
    sm = StateMachine(system)
    system.hub.set_mode(OperatorMode.LOAD)
    system.hub.send(Command.START)
    sim0 = world.time()
    phases: Counter[str] = Counter()
    end = "time limit"
    try:
        while world.time() - sim0 < time_limit_s:
            sm.poll()
            phases[sm.phase.value] += 1
            if sm.mode == "idle" and sm.phase is Phase.DONE:
                end = "done"
                break
            if sm.phase is Phase.ERROR:
                end = f"error: {sm.error}"
                break
        socks = []
        for it, reach in zip(world.items, reachable, strict=True):
            where, _ = world.location(it.id)
            sock = {"color": it.color.value, "reachable": reach, "end": where}
            if where != "cargo":  # where it is, to see why
                sock["at_mm"] = [round(float(v)) for v in world.vertices(it.id).mean(axis=0)]
            socks.append(sock)
        return {
            "seed": seed,
            "socks": socks,
            "counted": {c.value: n for c, n in sm.counters.items()},
            "cycles": sm.cycle,
            "end": end,
            "sim_s": round(world.time() - sim0, 1),
            "wall_s": round(time.monotonic() - t0, 1),
            "phases": dict(phases),
        }
    finally:
        system.camera.close()
        world.stop()


def summarize(scenes: list[dict[str, Any]]) -> dict[str, Any]:
    socks = [s for sc in scenes for s in sc["socks"]]
    reach = [s for s in socks if s["reachable"]]
    ends = Counter(s["end"] for s in reach)
    in_box = ends.get("cargo", 0)
    counted = sum(sum(sc["counted"].values()) for sc in scenes)
    boxed = sum(1 for s in socks if s["end"] == "cargo")
    # colors: per scene, the counted socks of each color against those in the box (a sock
    # counted under the wrong color shows up twice: one too many here, one too few there)
    wrong = 0
    for sc in scenes:
        truth = Counter(s["color"] for s in sc["socks"] if s["end"] == "cargo")
        wrong += sum(max(n - truth.get(c, 0), 0) for c, n in sc["counted"].items())
    return {
        "scenes": len(scenes),
        "socks": len(socks),
        "reachable": len(reach),
        "in_box_pct": round(100 * in_box / max(len(reach), 1), 1),
        "reachable_end": dict(ends),
        "unreachable_end": dict(Counter(s["end"] for s in socks if not s["reachable"])),
        "counted": counted,
        "counted_minus_in_box": counted - boxed,  # > 0: counted what didn't land; < 0: missed
        "counted_wrong_color_or_extra": wrong,
        "color_ok_pct": round(100 * (1 - wrong / max(counted, 1)), 1),
        "ends": dict(Counter(sc["end"].split(":")[0] for sc in scenes)),
        "sim_s_per_sock": round(sum(sc["sim_s"] for sc in scenes) / max(boxed, 1), 1),
        "wall_s": round(sum(sc["wall_s"] for sc in scenes), 1),
    }


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.sim.scenes.load.bench")
    p.add_argument("-n", type=int, default=50, help="scenes")
    p.add_argument("--socks", default="1-4", help="socks per scene: N or MIN-MAX")
    p.add_argument("--seed0", type=int, default=0, help="the first scene's seed")
    p.add_argument("--time-limit", type=float, default=600.0, help="simulated s per scene")
    p.add_argument("--workers", type=int, default=4, help="scenes run in parallel")
    p.add_argument("--config-dir", default=str(DEFAULT_CONFIG_DIR))
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    lo, _, hi = args.socks.partition("-")
    lo, hi = int(lo), int(hi or lo)
    rng = np.random.default_rng(args.seed0)
    jobs = [
        (args.seed0 + k, int(rng.integers(lo, hi + 1)), args.time_limit, args.config_dir)
        for k in range(args.n)
    ]
    run_id = time.strftime("%Y%m%d-%H%M%S")
    out = BENCH_DIR / run_id
    out.mkdir(parents=True, exist_ok=True)
    scenes = []
    with ProcessPoolExecutor(args.workers) as pool, (out / "scenes.jsonl").open("w") as f:
        for sc in pool.map(run_scene, *zip(*jobs, strict=True)):
            scenes.append(sc)
            f.write(json.dumps(sc) + "\n")
            f.flush()
            boxed = sum(s["end"] == "cargo" for s in sc["socks"])
            print(
                f"seed {sc['seed']}: {boxed}/{len(sc['socks'])} in the box, counted "
                f"{sum(sc['counted'].values())}, {sc['end']}, {sc['sim_s']} s sim, "
                f"{sc['wall_s']} s wall",
                flush=True,
            )
    summary = summarize(scenes)
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    print(f"report: {out}")


if __name__ == "__main__":
    main()

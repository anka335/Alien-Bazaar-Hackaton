"""Unload benchmark: N seeded scenarios on the real rover's geometry, headless, judged by what
the simulator knows (where every sock and bin really is), not by what the loop believes.

    uv run python -m sorter.sim.scenes.unload.bench [-n 20] [--socks 2,2,2] [--jobs 4]

Each scenario: `--socks` socks per color (light, dark, colored) piled in the cargo box, the
station shifted and turned by the seed (`--station-mm`, `--station-deg`, `--bin-mm`,
`--bin-deg`), the unload loop run from START to DONE / ERROR / the time limit. Report in
`data/bench/<run_id>/`: `results.json` (every scenario), `summary.json`, printed.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import time
import traceback
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from sorter.core.types import ColorClass

COLORS = list(ColorClass)


@dataclass
class Scenario:
    seed: int
    socks: tuple[int, int, int] = (2, 2, 2)
    station_mm: float = 30.0
    station_deg: float = 5.0
    bin_mm: float = 15.0
    bin_deg: float = 8.0
    max_sim_s: float = 0.0  # 0: 90 s per sock + 60


@dataclass
class Result:
    seed: int
    socks: int = 0
    in_right_bin: int = 0
    in_wrong_bin: int = 0
    left_in_cargo: int = 0
    elsewhere: int = 0  # the floor, the rover, a wall, the gripper
    counted: int = 0  # what the loop counted (seen drops)
    end: str = ""  # done / error: ... / timeout / crash: ...
    sim_s: float = 0.0
    wall_s: float = 0.0
    cycles: int = 0
    picks: list[dict] = field(default_factory=list)  # every pick: what was seen / taken
    bins: dict[str, dict] = field(default_factory=dict)  # found vs real
    where: list[dict] = field(default_factory=list)  # every sock at the end


def _truth_bins(world) -> dict[ColorClass, tuple[float, float, float]]:
    m = world.model
    out = {}
    for c in COLORS:
        g = m.geom(f"laundry_{c.value}_floor")
        w, _, _, z = g.quat
        out[c] = (float(g.pos[0] * 1000), float(g.pos[1] * 1000), 2 * math.atan2(z, w))
    return out


def _where(world, item: int, bins) -> tuple[str, str | None]:
    lay = world.layout
    with world.lock:
        if item in world._vert_item[world._grip_idx]:
            return "gripper", None
    v = world.vertices(item)
    x, y, z = v.mean(axis=0)
    half = lay.laundry.size_mm / 2
    for c, (bx, by, yaw) in bins.items():
        cs, sn = math.cos(-yaw), math.sin(-yaw)
        u, w = cs * (x - bx) - sn * (y - by), sn * (x - bx) + cs * (y - by)
        if abs(u) < half and abs(w) < half and z < lay.floor_z_mm + lay.laundry.height_mm:
            return "laundry", c.value
    cargo = lay.cargo
    if cargo.contains(x, y) and z < cargo.rim_z_mm + 20:
        return "cargo", None
    if z < lay.floor_z_mm + 40:
        return "floor", None
    return "other", None


def run(sc: Scenario, runs_dir: Path | None = None) -> Result:
    from sorter.app import build_system
    from sorter.core.types import Command, OperatorMode, Phase
    from sorter.orchestrator.state_machine import StateMachine
    from sorter.sim.scenes.unload.rover import rover_config

    res = Result(sc.seed)
    t_wall = time.monotonic()
    cfg = rover_config(
        {
            "sim": {
                "seed": sc.seed,
                "realtime": 0,
                "unload": {
                    "cargo": dict(zip([c.value for c in COLORS], sc.socks, strict=True)),
                    "sock_mm": [200, 90],
                    "station_mm": sc.station_mm,
                    "station_deg": sc.station_deg,
                    "bin_mm": sc.bin_mm,
                    "bin_deg": sc.bin_deg,
                },
            },
            "arm": {"speed_scale": 1.4},
            "state_machine": {
                "save_runs": runs_dir is not None,
                "runs_dir": str(runs_dir or "data/runs"),
            },
        }
    )
    system = build_system(cfg, sim=True)
    world = system.world
    try:
        system.camera.start()
        sm = StateMachine(system)
        loop = sm.loops[OperatorMode.UNLOAD]
        truth = _truth_bins(world)
        _spy_picks(loop, world, res)
        system.hub.set_mode(OperatorMode.UNLOAD)
        system.hub.send(Command.START)
        n = sum(sc.socks)
        limit = sc.max_sim_s or 90.0 * n + 60.0
        t0 = world.time()
        sm.poll()
        while True:
            if sm.phase is Phase.ERROR:
                res.end = f"error: {sm.error}"
                break
            if sm.mode == "idle":
                res.end = "done"
                break
            if world.time() - t0 > limit:
                res.end = "timeout"
                break
            sm.poll()
        res.sim_s = world.time() - t0
        res.cycles = sm.cycle
        res.counted = sum(sm.counters.values())
        if loop.bins:
            for c, b in loop.bins.items():
                tx, ty, tyaw = truth[c]
                res.bins[c.value] = {
                    "found": b.found,
                    "error_mm": round(math.dist(b.center, (tx, ty)), 1),
                    "yaw_error_deg": round(
                        math.degrees(math.remainder(b.yaw - tyaw, math.pi / 2)), 1
                    ),
                }
        world.stop()  # the socks stay where they are now
        world.step(round(1.0 / world.model.opt.timestep))  # let a falling one land
        res.socks = len(world.items)
        for it in world.items:
            loc, where = _where(world, it.id, truth)
            xyz = [round(float(v)) for v in world.vertices(it.id).mean(axis=0)]
            res.where.append(
                {"item": it.id, "color": it.color.value, "at": loc, "in": where, "xyz": xyz}
            )
            if loc == "laundry":
                if where == it.color.value:
                    res.in_right_bin += 1
                else:
                    res.in_wrong_bin += 1
            elif loc == "cargo":
                res.left_in_cargo += 1
            else:
                res.elsewhere += 1
    except Exception as e:  # a crash is a result too
        res.end = f"crash: {e!r}"
        logging.getLogger(__name__).error(traceback.format_exc())
    finally:
        system.camera.close()
        world.stop()
    res.wall_s = time.monotonic() - t_wall
    return res


def _spy_picks(loop, world, res: Result) -> None:
    """Record what each pick saw and what it really took (the simulator's grip)."""
    pick = loop._pick_from_cargo

    def spied():
        t = loop.target
        loop.color, loop.held, loop.gone = None, None, []
        nxt = pick()
        held = [it for it in world.items if world.location(it.id)[0] == "gripper"]
        res.picks.append(
            {
                "target": t.color.value if t else None,
                "decided": loop.color.value if loop.color else None,
                "took": [it.color.value for it in held],
                "gone": [g.color.value for g in loop.gone],
                "seen_held": loop.held.count if loop.held else None,
                "next": nxt.value,
            }
        )
        return nxt

    loop._pick_from_cargo = spied


def summarize(results: list[Result]) -> dict[str, Any]:
    socks = sum(r.socks for r in results)
    picks = [p for r in results for p in r.picks]
    took = [p for p in picks if p["took"]]
    errs = [b["error_mm"] for r in results for b in r.bins.values()]
    ends: dict[str, int] = {}
    for r in results:
        key = r.end.split(":")[0]
        ends[key] = ends.get(key, 0) + 1
    return {
        "scenarios": len(results),
        "socks": socks,
        "unloaded": round(sum(r.in_right_bin for r in results) / max(socks, 1), 3),
        "in_wrong_bin": sum(r.in_wrong_bin for r in results),
        "left_in_cargo": sum(r.left_in_cargo for r in results),
        "elsewhere": sum(r.elsewhere for r in results),
        "counted_vs_right": [sum(r.counted for r in results), sum(r.in_right_bin for r in results)],
        "ends": ends,
        "picks": len(picks),
        "empty_picks": len(picks) - len(took),
        "target_not_taken": sum(1 for p in took if p["target"] not in p["took"]),
        "color_wrong": sum(1 for p in took if p["decided"] and p["decided"] not in p["took"]),
        "held_missed": sum(1 for p in took if not p["decided"]),
        "held_but_empty": sum(1 for p in picks if p["decided"] and not p["took"]),
        "put_back": sum(1 for p in picks if len(p["gone"]) > 1 or (p["seen_held"] or 0) > 1),
        "double_picks": sum(1 for p in took if len(p["took"]) > 1),
        "bin_error_mm": {
            "mean": round(float(np.mean(errs)), 1) if errs else None,
            "max": round(float(np.max(errs)), 1) if errs else None,
        },
        "sim_s_per_sock": round(sum(r.sim_s for r in results) / max(socks, 1), 1),
        "wall_s": round(sum(r.wall_s for r in results), 1),
    }


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("-n", type=int, default=20, help="scenarios (seeds 0..n-1 + --seed0)")
    p.add_argument("--seed0", type=int, default=0)
    p.add_argument("--socks", default="2,2,2", help="per color: light,dark,colored")
    p.add_argument("--station-mm", type=float, default=30.0)
    p.add_argument("--station-deg", type=float, default=5.0)
    p.add_argument("--bin-mm", type=float, default=15.0)
    p.add_argument("--bin-deg", type=float, default=8.0)
    p.add_argument("--jobs", type=int, default=4)
    p.add_argument("--runs", action="store_true", help="also write each scenario's run log")
    a = p.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    socks = tuple(int(v) for v in a.socks.split(","))
    run_id = time.strftime("%Y%m%d-%H%M%S")
    out = Path("data/bench") / f"unload-{run_id}"
    out.mkdir(parents=True, exist_ok=True)
    scenarios = [
        Scenario(a.seed0 + i, socks, a.station_mm, a.station_deg, a.bin_mm, a.bin_deg)  # type: ignore[arg-type]
        for i in range(a.n)
    ]
    from sorter.sim.scenes.unload.rover import rover_config

    rover_config()  # the rig into the cache before the workers start
    runs = out / "runs" if a.runs else None
    results: list[Result] = []
    with ProcessPoolExecutor(a.jobs) as pool:
        for r in pool.map(run, scenarios, [runs] * len(scenarios)):
            results.append(r)
            (out / "results.json").write_text(json.dumps([asdict(x) for x in results], indent=1))
            print(
                f"seed {r.seed}: {r.in_right_bin}/{r.socks} right, {r.in_wrong_bin} wrong, "
                f"{r.left_in_cargo} in cargo, {r.elsewhere} elsewhere; {r.end}; "
                f"{r.sim_s:.0f} s sim, {r.wall_s:.0f} s wall",
                flush=True,
            )
            for p in r.picks:
                print(f"    pick {p}", flush=True)
    (out / "results.json").write_text(json.dumps([asdict(r) for r in results], indent=1))
    summary = summarize(results)
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))
    print(f"# {out}")


if __name__ == "__main__":
    main()

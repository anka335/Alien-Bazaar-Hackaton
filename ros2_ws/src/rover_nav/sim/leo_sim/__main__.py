"""Leo Rover in MuJoCo.

python -m leo_sim serve                    # web UI on http://127.0.0.1:8095
python -m leo_sim drive --target red_ball  # jevomir drives headless, log in runs/
python -m leo_sim view                     # MuJoCo viewer, arrow keys drive
python -m leo_sim snapshot                 # camera and map images to snapshots/
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]


def _seed(value: str) -> int | None:
    return None if value in ("", "fixed", "none") else int(value)


def cmd_serve(args) -> None:
    from .server import serve

    serve(args.host, args.port, args.seed, args.api_url, args.key_file, args.max_per_minute)


BENCH_TARGETS = ["red_ball", "blue_box", "green_bin", "yellow_crate", "laundry_basket", "table"]


def _client(args):
    from .jevomir import JevomirClient, ScorerError

    if args.oracle:
        return None
    try:
        return JevomirClient(args.api_url, args.key_file, max_per_minute=args.max_per_minute)
    except ScorerError as error:
        sys.exit(str(error))


def _run(args, client, seed, target: str, out: Path, verbose: bool) -> dict:
    from .agent import Agent
    from .jevomir import OracleScorer
    from .runner import SimRunner

    def show(step: dict) -> None:
        answers = ", ".join(f"{q['kind']}={q['answer']}" for q in step["questions"])
        outcome = (step.get("result") or {}).get("outcome", "")
        print(f"#{step['step']:>3} {answers:<60} -> {step['action']} {outcome}", flush=True)

    runner = SimRunner(seed=seed, realtime=0)
    try:
        scorer = client or OracleScorer(runner, target)
        agent = Agent(
            runner,
            scorer,
            target,
            policy=args.policy,
            camera=args.camera,
            max_steps=args.steps,
            both_orders=not args.one_order,
            memory=args.memory,
            log_dir=out,
            on_step=show if verbose else None,
        )
        return agent.run()
    finally:
        runner.close()


def cmd_drive(args) -> None:
    out = args.out or HERE / "runs" / time.strftime("%Y%m%d-%H%M%S")
    summary = _run(args, _client(args), args.seed, args.target, out, verbose=True)
    print(json.dumps(summary, indent=1))
    print(f"log, images and map in {out}")


def cmd_bench(args) -> None:
    """Rooms 0..N-1 (seeds), target i cycles through BENCH_TARGETS, like the oracle numbers."""
    client = _client(args)
    who = "oracle" if args.oracle else "jevomir"
    mem = "" if args.memory == "off" else f"-memory-{args.memory}"
    out = (
        args.out
        or HERE / "runs" / f"bench-{who}-{args.policy}{mem}-{time.strftime('%Y%m%d-%H%M%S')}"
    )
    rows = []
    for seed in range(args.rooms):
        target = BENCH_TARGETS[seed % len(BENCH_TARGETS)]
        s = _run(args, client, seed, target, out / f"room{seed:02d}-{target}", verbose=False)
        rows.append({"room": seed, **s})
        print(
            f"room {seed:>2} {target:<15} {s['status']:<8} steps {s['steps']:>3}  "
            f"{s['distance_m']:>5.2f} m  bumps {s['bumps']}  calls {s['questions']:>3}  "
            f"{s['wall_s']:>6.1f} s",
            flush=True,
        )
        (out / "results.json").write_text(json.dumps(rows, indent=1) + "\n")
    wins = sum(r["status"] == "success" for r in rows)
    name = f"{who} {args.policy}{'' if args.memory == 'off' else ' memory=' + args.memory}"
    print(f"{name}: {wins}/{len(rows)} success; results in {out}/results.json")


def cmd_view(args) -> None:
    import mujoco.viewer

    from .rover import LeoSim

    sim = LeoSim(seed=args.seed)
    keys = {265: (0.25, 0.0), 264: (-0.25, 0.0), 263: (0.0, 0.8), 262: (0.0, -0.8)}  # GLFW arrows
    held = {"cmd": (0.0, 0.0), "until": 0.0}

    def on_key(key: int) -> None:
        if key in keys:
            held["cmd"], held["until"] = keys[key], sim.time + 0.4
        elif key == 32:  # space
            held["until"] = 0.0

    print("arrow keys drive (each press ~0.4 s), space stops; close the window to quit")
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            start = time.monotonic()
            sim.set_cmd_vel(*(held["cmd"] if sim.time < held["until"] else (0.0, 0.0)))
            with viewer.lock():
                sim.step(0.02)
            viewer.sync()
            time.sleep(max(0.0, 0.02 - (time.monotonic() - start)))


def cmd_snapshot(args) -> None:
    from PIL import Image

    from .rover import LeoSim

    sim = LeoSim(seed=args.seed)
    args.out.mkdir(parents=True, exist_ok=True)
    for camera in ("leo", "oak", "chase", "map"):
        Image.fromarray(sim.render(camera)).save(args.out / f"{camera}.png")
    sim.close()
    print(f"wrote {args.out}/{{leo,oak,chase,map}}.png")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="leo_sim", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--seed", type=_seed, default=None, help="room layout seed (default: fixed layout)"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def run_args(p):
        p.add_argument("--policy", default="guided", choices=["guided", "direct"])
        p.add_argument("--camera", default="leo", choices=["leo", "oak"])
        p.add_argument(
            "--memory",
            default="off",
            choices=["off", "control", "prompt"],
            help="use the path from odometry: control = the agent acts on it, prompt = told "
            "to the model (makes it worse); leo_sim/memory.py",
        )
        p.add_argument("--steps", type=int, default=40)
        p.add_argument(
            "--one-order", action="store_true", help="ask each question once (half the API calls)"
        )
        p.add_argument(
            "--oracle", action="store_true", help="answer from the sim's ground truth, no API"
        )
        p.add_argument("--out", type=Path, default=None, help="log dir (default runs/...)")

    def api_args(p):
        p.add_argument(
            "--api-url", default="", help="jevomir API (env JEVOMIR_API_URL, default :8100)"
        )
        p.add_argument("--key-file", type=Path, default=None, help="default: sim/.api-key")
        p.add_argument(
            "--max-per-minute", type=int, default=90, help="free ngrok drops ~100/min; 0 = no limit"
        )

    p = sub.add_parser("serve", help="web UI: watch, drive by hand, let jevomir drive")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8095)
    api_args(p)
    p.set_defaults(func=cmd_serve)

    p = sub.add_parser("drive", help="jevomir drives to a target, headless")
    p.add_argument(
        "--target", default="red_ball", help="object name, e.g. red_ball, laundry_basket"
    )
    run_args(p)
    api_args(p)
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("bench", help="seeded rooms in a row, one target each")
    p.add_argument("--rooms", type=int, default=12)
    run_args(p)
    api_args(p)
    p.set_defaults(func=cmd_bench)

    p = sub.add_parser("view", help="MuJoCo viewer, drive with the arrow keys")
    p.set_defaults(func=cmd_view)

    p = sub.add_parser("snapshot", help="render every camera to PNG")
    p.add_argument("--out", type=Path, default=HERE / "snapshots")
    p.set_defaults(func=cmd_snapshot)

    args = parser.parse_args()
    if args.command != "view":  # offscreen rendering without a window; the viewer needs GLFW
        os.environ.setdefault("MUJOCO_GL", "egl")
    args.func(args)


if __name__ == "__main__":
    main()

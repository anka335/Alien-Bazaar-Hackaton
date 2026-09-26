"""Accuracy test of the SO-101: how far the joints and the fingertips end up from their goals.

    uv run python -m sorter.arm.accuracy [--sim] [--speed 0.3] [--set arm.p_gain=24 ...]

The arm goes to `home`, then for each test point: a joint move above it (tool down), a straight
line 20 mm down, and back up. After each move it records the goal and the measured joints twice:
when the controller says the arm has settled, and `--late-s` later (a difference means it settled
too early). Tip errors are FK(measured) − FK(goal), so they are control errors (sag, overshoot),
not model errors. Ends at `rest` with the motors off. Results go to `data/accuracy/`.
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from sorter.arm import kinematics as kin
from sorter.arm.backend import create, create_mock
from sorter.arm.controller import So101Arm
from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.core.errors import ArmError

POINTS_MM = [(200, 0, 60), (240, -40, 50), (170, 50, 40), (220, 60, 70), (180, -60, 40)]
LINE_MM = 20.0
OUT_DIR = Path("data/accuracy")


def parse_set(items: list[str]) -> dict:
    """["arm.p_gain=24", ...] → nested overrides dict (values parsed as YAML)."""
    out: dict = {}
    for item in items:
        key, _, value = item.partition("=")
        node = out
        *parents, leaf = key.split(".")
        for k in parents:
            node = node.setdefault(k, {})
        node[leaf] = yaml.safe_load(value)
    return out


def measure(arm: So101Arm, name: str, q_goal: np.ndarray, q_settled: np.ndarray, late_s: float):
    samples = []  # jitter: peak-to-peak of the joints while holding (a too high P oscillates)
    t_end = time.monotonic() + late_s
    while True:
        samples.append(arm.read_q())
        if time.monotonic() >= t_end:
            break
        time.sleep(0.02)
    q_late = samples[-1]
    jitter = np.degrees(np.ptp(samples, axis=0)).max()
    goal_tip = arm.tcp(q_goal)
    rec = {"move": name, "goal_tip_mm": goal_tip.round(1).tolist(), "jitter_deg": round(jitter, 2)}
    for when, q in (("settled", q_settled), ("late", q_late)):
        d = arm.tcp(q) - goal_tip
        rec[when] = {
            "joint_err_deg": np.degrees(q - q_goal).round(2).tolist(),
            "tip_err_mm": d.round(1).tolist(),
            "tip_err_norm_mm": round(float(np.linalg.norm(d)), 1),
        }
    s, late = rec["settled"], rec["late"]
    print(
        f"{name:28s} tip err {s['tip_err_norm_mm']:5.1f} mm {s['tip_err_mm']}"
        f"  → {late['tip_err_norm_mm']:5.1f} mm later, jitter {jitter:.2f}°;"
        f"  joints° {s['joint_err_deg']}"
    )
    return rec


def run(arm: So101Arm, speed: float, late_s: float) -> list[dict]:
    tilt = arm.cfg.grasp_max_tilt_deg
    records = []
    arm.goto("home", speed)
    for x, y, z in POINTS_MM:
        p = np.array([x, y, z], dtype=float)
        r = arm.solve_down(p, arm.read_q(), tilt)
        if r is None:
            print(f"({x}, {y}, {z}) unreachable, skipped")
            continue
        arm.move_joints(r.q, speed)
        records.append(measure(arm, f"joint → ({x},{y},{z})", r.q, arm.read_q(), late_s))
        low = p - [0, 0, LINE_MM]
        seed = r.q
        for name, a, b in (("line ↓", p, low), ("line ↑", low, p)):
            qs = arm.plan_line(a, b, seed, tilt)
            q = arm.follow(qs, LINE_MM / speed)
            records.append(measure(arm, f"{name} {LINE_MM:.0f} mm", qs[-1], q, late_s))
            seed = qs[-1]
    arm.goto("home", speed)
    return records


def summary(records: list[dict]) -> dict:
    out = {}
    for when in ("settled", "late"):
        errs = np.array([r[when]["tip_err_norm_mm"] for r in records])
        dz = np.array([r[when]["tip_err_mm"][2] for r in records])
        joints = np.abs([r[when]["joint_err_deg"] for r in records])
        out[when] = {
            "tip_mean_mm": round(float(errs.mean()), 1),
            "jitter_max_deg": max(r["jitter_deg"] for r in records),
            "tip_max_mm": round(float(errs.max()), 1),
            "dz_mean_mm": round(float(dz.mean()), 1),
            "joint_mean_abs_deg": dict(
                zip(kin.JOINT_NAMES, joints.mean(axis=0).round(2).tolist(), strict=True)
            ),
        }
    return out


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="sorter.arm.accuracy")
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    p.add_argument("--sim", action="store_true", help="mock bus with sag, no hardware")
    p.add_argument("--speed", type=float, default=0.3)
    p.add_argument("--late-s", type=float, default=1.0)
    p.add_argument("--set", action="append", default=[], help="config override, e.g. arm.p_gain=24")
    p.add_argument("--label", default="")
    p.add_argument("-v", "--verbose", action="store_true", help="log every settle pass")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(name)s %(message)s")
    if args.verbose:
        logging.getLogger("sorter.arm.controller").setLevel(logging.DEBUG)

    overrides = parse_set(args.set)
    cfg = load_config(args.config_dir, overrides)
    if args.sim:  # servos that lag and sag (ticks) like loaded P-controlled ones
        arm = create_mock(cfg, sleep=lambda s: None, lag=0.3, sag={2: -60, 3: 40, 4: -20})
    else:
        arm = create(cfg)
    arm.start()
    try:
        records = run(arm, args.speed, 0.0 if args.sim else args.late_s)
    except (ArmError, KeyboardInterrupt) as e:
        print(f"stopped: {e!r}")
        arm.hold()
        records = []
    finally:
        arm.shutdown()
    if not records:
        return
    result = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "label": args.label,
        "sim": args.sim,
        "speed": args.speed,
        "overrides": overrides,
        "summary": summary(records),
        "moves": records,
    }
    print(json.dumps(result["summary"], indent=2, ensure_ascii=False))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("%Y%m%d-%H%M%S") + (f"-{args.label}" if args.label else "")
    path = OUT_DIR / f"{name}.json"
    path.write_text(json.dumps(result, indent=1, ensure_ascii=False))
    print(f"saved {path}")


if __name__ == "__main__":
    main()

"""Score the direct prompts (agent.DIRECT_PROMPTS) on labelled camera frames.

Frames: the rover is placed in seeded rooms so the right move is known (oracle_action):
the target straight ahead and close (stop), straight ahead and far (forward), 20-45 deg to
the left or right, or out of view (search: turning either way counts as right). Each prompt
answers every frame with both option orders averaged, as in a real run.

    uv run python tools/eval_direct.py --key-file ~/dev/jevomir/web/.api-key
    uv run python tools/eval_direct.py --oracle        # checks the tool itself, no API
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from collections import Counter
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
from leo_sim.agent import DIRECT_PROMPTS, IMAGE_SIZE, SEARCH  # noqa: E402
from leo_sim.jevomir import JevomirClient, oracle_action, pick_action, target_view  # noqa: E402
from leo_sim.rover import LeoSim  # noqa: E402
from PIL import Image  # noqa: E402

TARGETS = ["red_ball", "blue_box", "green_bin", "yellow_crate", "laundry_basket", "table"]
CLASSES = {  # label: (surface distance range m, |bearing| range deg, bearing sign choices)
    "done": ((0.15, 0.4), (0, 8), (1, -1)),
    "forward": ((0.9, 3.0), (0, 8), (1, -1)),
    "left": ((0.6, 2.5), (22, 45), (1,)),
    "right": ((0.6, 2.5), (22, 45), (-1,)),
    "search": ((0.5, 2.5), (90, 180), (1, -1)),
}
OUT = Path(__file__).resolve().parents[1] / "runs" / "eval-direct"


def make_frames(per_class: int, seed: int) -> list[dict]:
    rng = random.Random(seed)
    frames: list[dict] = []
    counts: Counter = Counter()
    room = 0
    while min(counts[c] for c in CLASSES) < per_class and room < 200:
        sim = LeoSim(seed=room)
        for _ in range(60):
            label = min(CLASSES, key=lambda c: counts[c])
            if counts[label] >= per_class:
                break
            (d0, d1), (b0, b1), signs = CLASSES[label]
            target = rng.choice(TARGETS)
            obj = sim.world.obj(target)
            centre = rng.uniform(d0, d1) + obj.radius + 0.25
            towards = rng.uniform(-math.pi, math.pi)  # direction rover -> target
            x, y = obj.x - centre * math.cos(towards), obj.y - centre * math.sin(towards)
            if abs(x) > sim.world.width / 2 - 0.35 or abs(y) > sim.world.depth / 2 - 0.35:
                continue
            yaw = towards - math.radians(rng.uniform(b0, b1) * rng.choice(signs))
            if not sim.place(x, y, yaw):
                continue
            if oracle_action(target_view(sim, target)) != label:  # occluded, clipped...
                continue
            image = sim.render("leo", *IMAGE_SIZE)
            name = f"{len(frames):03d}-{label}-{target}.jpg"
            Image.fromarray(image).save(OUT / "frames" / name, quality=90)
            frames.append(
                {
                    "file": name,
                    "label": label,
                    "target": target,
                    "label_text": obj.label,
                    "room": room,
                }
            )
            counts[label] += 1
        sim.close()
        room += 1
    return frames


def correct(label: str, action: str) -> bool:
    if label == "search":
        return action in (SEARCH, "left", "right")
    return action == label


MOTION = (
    "Robot memory from its wheel odometry (step 5): Last moves, oldest first: turned 20° left; "
    "drove 0.40 m forward; turned 20° right. Position: x 0.80 m, y 0.10 m, heading 10° from the "
    "start; 1.2 m driven in total."
)
HINT = {  # a true "last seen" sentence per label
    "done": "in the center of the photo and very close; that is roughly straight ahead",
    "forward": "in the center of the photo; that is roughly straight ahead",
    "left": "in the left half of the photo; that is about 30° to the left",
    "right": "in the right half of the photo; that is about 30° to the right",
    "search": "in the left half of the photo; that is about 130° to the left",
}


def context(mode: str, frame: dict) -> str:
    """Memory text put before the question, as leo_sim.memory writes it (see --contexts)."""
    t = frame["label_text"]
    if mode == "none":
        return ""
    if mode == "motion":
        return MOTION
    if mode == "unseen":
        return MOTION.replace(" Position:", f" It has not seen {t} yet. Position:")
    if mode == "hint":
        hint = HINT[frame["label"]]
        seen = f" It last saw {t} in the previous photo, {hint} of where the robot now faces."
        return MOTION.replace(" Position:", seen + " Position:")
    raise ValueError(mode)


def score(client, frames: list[dict], prompts: list[str], contexts: list[str]) -> dict:
    results = {}
    for name, mode in [(p, c) for p in prompts for c in contexts]:
        question, options = DIRECT_PROMPTS[name]
        actions = [a for a, _ in options.values()]
        rows, confusion = [], Counter()
        for f in frames:
            image = np.asarray(Image.open(OUT / "frames" / f["file"]))
            texts = [o.format(t=f["label_text"]) for o in options]
            q = question.format(t=f["label_text"])
            if context(mode, f):
                q = f"{context(mode, f)}\n{q}"
            if client is None:  # oracle: the labelled answer
                truth = f["label"] if f["label"] != "search" else "search"
                probs = [0.0] * len(texts)
                probs[pick_action(truth, texts)] = 1.0
            else:
                p1 = client.score(q, texts, [image], "action")
                p2 = client.score(q, texts[::-1], [image], "action")[::-1]
                probs = [(a + b) / 2 for a, b in zip(p1, p2, strict=True)]
            action = actions[int(np.argmax(probs))]
            ok = correct(f["label"], action)
            confusion[(f["label"], action)] += 1
            rows.append(
                {
                    "file": f["file"],
                    "label": f["label"],
                    "action": action,
                    "ok": ok,
                    "probabilities": [round(p, 3) for p in probs],
                }
            )
        per_class = {
            c: round(np.mean([r["ok"] for r in rows if r["label"] == c]), 2) for c in CLASSES
        }
        accuracy = round(float(np.mean([r["ok"] for r in rows])), 3)
        key = name if mode == "none" else f"{name}+{mode}"
        results[key] = {
            "accuracy": accuracy,
            "per_class": per_class,
            "rows": rows,
            "confusion": {f"{a}->{b}": n for (a, b), n in sorted(confusion.items())},
        }
        print(
            f"{key:<22} {accuracy:.0%}  " + "  ".join(f"{c} {v:.0%}" for c, v in per_class.items()),
            flush=True,
        )
    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--per-class", type=int, default=10)
    parser.add_argument("--prompts", nargs="*", default=list(DIRECT_PROMPTS))
    parser.add_argument(
        "--contexts",
        nargs="*",
        default=["none"],
        choices=["none", "motion", "unseen", "hint"],
        help="memory text before the question: none, moves+pose, + 'not seen yet', + a true hint",
    )
    parser.add_argument("--oracle", action="store_true")
    parser.add_argument("--new-frames", action="store_true", help="re-render the frame set")
    parser.add_argument("--api-url", default="")
    parser.add_argument("--key-file", type=Path, default=None)
    parser.add_argument("--max-per-minute", type=int, default=90)
    args = parser.parse_args()
    (OUT / "frames").mkdir(parents=True, exist_ok=True)
    index = OUT / "frames.json"
    if args.new_frames or not index.is_file():
        frames = make_frames(args.per_class, seed=0)
        index.write_text(json.dumps(frames, indent=1) + "\n")
    frames = json.loads(index.read_text())
    print(f"{len(frames)} frames: {dict(Counter(f['label'] for f in frames))}")
    client = (
        None
        if args.oracle
        else JevomirClient(args.api_url, args.key_file, max_per_minute=args.max_per_minute)
    )
    results = score(client, frames, args.prompts, args.contexts)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = OUT / f"results-{'oracle' if args.oracle else 'jevomir'}-{stamp}.json"
    path.write_text(json.dumps(results, indent=1) + "\n")
    print(f"details in {path}")


if __name__ == "__main__":
    main()

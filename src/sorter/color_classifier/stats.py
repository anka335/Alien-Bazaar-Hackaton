"""Tuning tool: classify recorded observations and print the color stats of every item.

    uv run python -m sorter.color_classifier.stats data/runs/<run_id>/0001_sense_floor.npz [...]
    uv run python -m sorter.color_classifier.stats --sim   # a rendered sim frame of the floor

`--save DIR` writes each frame with the overlay drawn on it.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.core.config import load_config
from sorter.core.io import load_observation
from sorter.core.types import BackgroundResult, ColorClass, Zone


def _sim_frame(cfg, n_items: int):
    """A rendered frame of `n_items` socks on the floor, and the sim's segmentation."""
    from sorter.app import build_system

    colors = [list(ColorClass)[i % len(ColorClass)] for i in range(n_items)]
    load = cfg.sim.load.model_copy(update={"socks": colors})
    sim = cfg.sim.model_copy(update={"realtime": 0, "scenes": ["load"], "load": load})
    system = build_system(cfg.model_copy(update={"sim": sim}), sim=True)
    system.arm.start()
    system.arm.look(Zone.FLOOR)
    return system.camera.fresh(), system.camera.segment


def report(name: str, bg: BackgroundResult) -> None:
    print(f"{name}: {len(bg.items)} item(s)")
    for i, it in enumerate(bg.items):
        s = it.stats
        print(
            f"  #{i} {it.color:<8} conf {it.confidence:.2f}  area {it.area_px:6d} px"
            f"  L {s['L']:5.1f}  a {s['a']:6.1f}  b {s['b']:6.1f}  chroma {s['chroma']:5.1f}"
            f"  score {s['score']:.2f}  grasp ({it.grasp.px.u}, {it.grasp.px.v})"
            f" {it.grasp.depth_mm:.0f} mm{'  EDGE' if it.touches_roi_edge else ''}"
        )


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("paths", nargs="*", type=Path, help="observation .npz files")
    p.add_argument("--sim", type=int, nargs="?", const=3, metavar="N", help="N sim items")
    p.add_argument("--prompts", help="override color_classifier.sam.prompts, comma-separated")
    p.add_argument("--threshold", type=float, help="override color_classifier.sam.threshold")
    p.add_argument("--save", type=Path, help="write frames with the overlay here")
    args = p.parse_args(argv)

    over: dict = {}
    prompts = [s.strip() for s in args.prompts.split(",")] if args.prompts else None
    sam = {"prompts": prompts, "threshold": args.threshold}
    over["color_classifier"] = {"sam": {k: v for k, v in sam.items() if v is not None}}
    cfg = load_config(overrides=over)
    view = cfg.views.get(Zone.FLOOR)
    roi = view.roi if view else ()
    segment = None
    frames = [(str(path), load_observation(path).frame) for path in args.paths]
    if args.sim:
        frame, segment = _sim_frame(cfg, args.sim)
        frames.append(("sim", frame))
    if not frames:
        p.error("give observation files or --sim")
    if segment is None or cfg.sim.use_sam3:
        segment = SamSegmenter(cfg.color_classifier.sam).segment
    classifier = Sam3ColorClassifier(cfg.color_classifier, segment, roi)

    for name, frame in frames:
        bg = classifier.classify(frame)
        report(name, bg)
        if args.save:
            from sorter.dashboard.render import draw_overlay

            args.save.mkdir(parents=True, exist_ok=True)
            out = args.save / (Path(name).stem + "_overlay.png")
            cv2.imwrite(str(out), draw_overlay(frame.color, bg.overlay))
            print(f"  → {out}")


if __name__ == "__main__":
    main()

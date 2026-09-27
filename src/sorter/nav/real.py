"""The navigation commands on the real Leo Rover and OAK-D.

`RealSession(cfg)` connects both (`real_leo.LeoBase` over rosbridge, `real_oakd.RealOakD`) and
gives the same `Rover` (commands) as the simulator. Nothing about ground truth exists here:
`score()` is None.

Deployment (see README → Rover navigation on the real Leo): the OAK-D is plugged into the rover's
Raspberry Pi; run `python -m sorter.nav serve --real` there and open the Rover tab from a laptop,
or run single commands with `python -m sorter.nav real do DIR forward 0.3`.
"""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from sorter.nav.camera import Frame, colorize_depth
from sorter.nav.commands import Result, Rover
from sorter.nav.config import NavConfig
from sorter.nav.episode import goal_pixels, grid


class RealSession:
    """The real rover and camera behind the command set. Close it: it stops the rover."""

    def __init__(self, cfg: NavConfig, base=None, camera=None):
        self.cfg = cfg
        if camera is None:
            from sorter.nav.real_oakd import RealOakD

            camera = RealOakD(cfg.camera, cfg.real)
        try:
            if base is None:
                from sorter.nav.real_leo import LeoBase

                base = LeoBase(cfg)
        except Exception:
            camera.close()
            raise
        self.sim = base  # the name the commands and the live panel use for "the rover"
        self.camera = camera
        self.rover = Rover(base, camera)
        self.commands = 0
        self.spec = None

    def score(self):
        return None

    def close(self) -> None:
        try:
            self.sim.close()
        finally:
            self.camera.close()


def save_frame(root: Path, f: Frame, goal, heading: float | None = None) -> str:
    """The operator's files for a frame, as the sim's episodes write them."""
    d = root / "frames"
    d.mkdir(parents=True, exist_ok=True)
    stem = f"{f.index:03d}"
    cv2.imwrite(str(d / f"{stem}_rgb.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
    g = grid(f.rgb, heading, goal_pixels(f, goal))
    cv2.imwrite(str(d / f"{stem}_grid.png"), cv2.cvtColor(g, cv2.COLOR_RGB2BGR))
    cv2.imwrite(
        str(d / f"{stem}_depth.png"), cv2.cvtColor(colorize_depth(f.depth_mm), cv2.COLOR_RGB2BGR)
    )
    np.save(d / f"{stem}_depth.npy", f.depth_mm)
    return stem


def record(root: Path, res: Result, goal) -> dict:
    """Write a result's frames and append it to `log.jsonl`; the summary with the frame paths."""
    root.mkdir(parents=True, exist_ok=True)
    out = res.summary()
    if res.frame is not None:
        stem = save_frame(root, res.frame, goal)
        out["frame"] = str(root / "frames" / f"{stem}_grid.png")
        out["depth"] = str(root / "frames" / f"{stem}_depth.png")
        (root / "last.json").write_text(json.dumps({"stem": stem}))
    if res.frames:
        out["scan"] = [
            {
                "heading_deg": h,
                "frame": str(root / "frames" / f"{save_frame(root, f, goal, h)}_grid.png"),
            }
            for h, f in res.frames
        ]
    with (root / "log.jsonl").open("a") as fh:
        fh.write(json.dumps(out) + "\n")
    return out


def last_frame(root: Path, camera) -> Frame | None:
    """The frame the operator last saw in `root` (for pixel commands in a new process)."""
    p = root / "last.json"
    if not p.is_file():
        return None
    stem = json.loads(p.read_text())["stem"]
    d = root / "frames"
    rgb = cv2.cvtColor(cv2.imread(str(d / f"{stem}_rgb.png")), cv2.COLOR_BGR2RGB)
    depth = np.load(d / f"{stem}_depth.npy")
    return Frame(int(stem), 0.0, rgb, depth, camera.K, camera.T_rover_cam)

"""An episode: one scenario, the rover, its camera and commands, and the score at the end.

`Episode.new(dir, scenario, seed, overrides)` starts one in a directory; `Episode.load(dir)` picks
it up in another process (the CLI runs one command per process). The directory holds
`episode.json` (what it is), `state.json` (the physics and odometry after the last command),
`log.jsonl` (every command and its result) and the frames an operator sees:
`frames/NNN_rgb.png`, `NNN_grid.png` (with a pixel grid), `NNN_depth.png` (colored) and the last
depth as `last_depth.npy`. Ground truth goes only into `score()`.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from sorter.nav import scenario as scenarios
from sorter.nav.camera import Frame, OakD, colorize_depth
from sorter.nav.commands import Result, Rover
from sorter.nav.config import NavConfig
from sorter.nav.sim import RoverSim


def apply_overrides(cfg: NavConfig, overrides: dict[str, str]) -> NavConfig:
    """`{"camera.pitch_deg": "30"}` → a copy of cfg with those values (validated)."""
    data = cfg.model_dump()
    for key, raw in overrides.items():
        node = data
        parts = key.split(".")
        for p in parts[:-1]:
            if p not in node or not isinstance(node[p], dict):
                raise ValueError(f"no config key nav.{key}")
            node = node[p]
        if parts[-1] not in node:
            raise ValueError(f"no config key nav.{key}")
        try:
            val = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            val = raw
        node[parts[-1]] = val
    return NavConfig.model_validate(data)


@dataclass
class Score:
    success: bool
    sock_x_m: float  # the sock's center in the rover frame at the end (truth)
    sock_y_m: float
    in_zone: bool
    sock_pushed_m: float
    collisions: int  # control ticks with the rover touching an obstacle or wall
    commands: int
    sim_time_s: float
    distance_m: float
    sock: int = 0  # which sock was scored (0 = the scenario's goal sock)

    def summary(self) -> dict:
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in vars(self).items()}


class Episode:
    def __init__(self, spec: scenarios.WorldSpec, cfg: NavConfig, root: Path | None = None):
        self.spec, self.cfg, self.root = spec, cfg, root
        self.sim = RoverSim(spec, cfg)
        self.camera = OakD(self.sim, cfg.camera)
        self.rover = Rover(self.sim, self.camera)
        self.commands = 0

    # --- persistence ---

    @classmethod
    def new(
        cls,
        root: Path,
        scenario: str,
        seed: int,
        overrides: dict[str, str] | None = None,
        base: NavConfig | None = None,
    ) -> tuple[Episode, Result]:
        root.mkdir(parents=True, exist_ok=True)
        (root / "frames").mkdir(exist_ok=True)
        overrides = overrides or {}
        cfg = apply_overrides(base or NavConfig(), overrides)
        ep = cls(scenarios.make(scenario, seed), cfg, root)
        (root / "episode.json").write_text(
            json.dumps(
                {
                    "scenario": scenario,
                    "seed": seed,
                    "overrides": overrides,
                    "config": cfg.model_dump(),
                },
                indent=1,
            )
        )
        (root / "log.jsonl").write_text("")
        res = ep.rover.look()
        ep.record(res, count=False)
        return ep, res

    @classmethod
    def load(cls, root: Path) -> Episode:
        meta = json.loads((root / "episode.json").read_text())
        cfg = NavConfig.model_validate(meta["config"])
        ep = cls(scenarios.make(meta["scenario"], meta["seed"]), cfg, root)
        state = json.loads((root / "state.json").read_text())
        ep.sim.set_state(state["sim"])
        ep.commands = state["commands"]
        ep.camera._index = state["frame_index"]
        ep.camera._rng.bit_generator.state = state["camera_rng"]
        ep.rover.frame = ep._last_frame(state)
        return ep

    def record(self, res: Result, count: bool = True) -> None:
        """Save the state, append the log, write the frames of this result."""
        self.commands += count
        if self.root is None:
            return
        frames = [(None, res.frame)] + [(h, f) for h, f in res.frames]
        names = []
        for heading, f in frames:
            if f is None:
                continue
            names.append(self.write_frame(f, heading))
        entry = res.summary() | {"frames": names, "commands": self.commands}
        with (self.root / "log.jsonl").open("a") as fh:
            fh.write(json.dumps(entry) + "\n")
        f = res.frame
        np.save(self.root / "last_depth.npy", f.depth_mm)
        state = {
            "sim": self.sim.get_state(),
            "commands": self.commands,
            "frame_index": self.camera._index,
            "camera_rng": json.loads(json.dumps(self.camera._rng.bit_generator.state)),
            "last_frame": {"index": f.index, "t": f.t},
        }
        (self.root / "state.json").write_text(json.dumps(state))

    def write_frame(self, f: Frame, heading: float | None = None) -> str:
        stem = f"{f.index:03d}"
        d = self.root / "frames"
        cv2.imwrite(str(d / f"{stem}_rgb.png"), cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR))
        g = grid(f.rgb, heading, goal_pixels(f, self.cfg.goal))
        cv2.imwrite(str(d / f"{stem}_grid.png"), cv2.cvtColor(g, cv2.COLOR_RGB2BGR))
        cv2.imwrite(
            str(d / f"{stem}_depth.png"),
            cv2.cvtColor(colorize_depth(f.depth_mm), cv2.COLOR_RGB2BGR),
        )
        return stem

    def _last_frame(self, state: dict) -> Frame:
        lf = state["last_frame"]
        rgb = cv2.cvtColor(
            cv2.imread(str(self.root / "frames" / f"{lf['index']:03d}_rgb.png")), cv2.COLOR_BGR2RGB
        )
        depth = np.load(self.root / "last_depth.npy")
        return Frame(lf["index"], lf["t"], rgb, depth, self.camera.K, self.camera.T_rover_cam)

    # --- scoring (ground truth) ---

    def score(self) -> Score:
        """Success: a sock (any of them: the task is to approach *a* sock) in the goal zone, that
        sock pushed at most `max_push_m`, no collision. The sock scored is the one nearest the
        zone's center."""
        g = self.cfg.goal
        n = len(self.spec.socks)
        pos = [self.sim.sock_in_rover(i) for i in range(n)]
        i = min(
            range(n), key=lambda k: math.hypot(pos[k][0] - g.center_m[0], pos[k][1] - g.center_m[1])
        )
        sx, sy = pos[i]
        in_zone = (
            abs(sx - g.center_m[0]) <= g.half_size_m[0]
            and abs(sy - g.center_m[1]) <= g.half_size_m[1]
        )
        pushed = float(np.linalg.norm(self.sim.sock_pose(i)[:2] - self.sim.sock_start[i]))
        return Score(
            success=bool(in_zone and pushed <= g.max_push_m and self.sim.collisions == 0),
            sock_x_m=sx,
            sock_y_m=sy,
            in_zone=bool(in_zone),
            sock_pushed_m=pushed,
            collisions=self.sim.collisions,
            commands=self.commands,
            sim_time_s=self.sim.t,
            distance_m=self.sim.odom.distance,
            sock=i,
        )

    def close(self) -> None:
        self.camera.close()


def grid(rgb: np.ndarray, heading: float | None = None, goal=None, step: int = 80) -> np.ndarray:
    """The frame with a labeled pixel grid, so an operator can read off (u, v), and the goal
    zone (where the sock must end up) drawn on the floor where it is visible."""
    img = rgb.copy()
    h, w = img.shape[:2]
    if goal is not None:
        cv2.polylines(img, [np.array(goal, np.int32)], True, (0, 255, 120), 1, cv2.LINE_AA)
    for u in range(step, w, step):
        img[:, u] = (img[:, u].astype(np.int32) * 0.4 + 255 * 0.6).astype(np.uint8)
        cv2.putText(
            img, str(u), (u + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA
        )
    for v in range(step, h, step):
        img[v, :] = (img[v, :].astype(np.int32) * 0.4 + 255 * 0.6).astype(np.uint8)
        cv2.putText(
            img, str(v), (2, v - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA
        )
    if heading is not None:
        cv2.putText(
            img,
            f"scan heading {heading:+.0f} deg",
            (w - 190, h - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return img


def goal_pixels(frame: Frame, goal) -> list[tuple[int, int]] | None:
    """The goal zone's corners on the floor, projected into the frame (for overlays)."""
    (cx, cy), (hx, hy) = goal.center_m, goal.half_size_m
    pts = [
        frame.project((cx + a * hx, cy + b * hy, 0.0))
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))
    ]
    if any(p is None for p in pts):
        return None
    return [(int(round(u)), int(round(v))) for u, v in pts]


def angle_deg(a: float) -> float:
    return math.degrees((a + math.pi) % (2 * math.pi) - math.pi)

"""Sock hunt: one call that finds socks with the OAK-D and brings the rover up to them, on the
simulator or the real Leo Rover.

    from sorter.nav.hunt import hunt_socks
    hunt_socks(real=True, max_socks=3)          # the real Leo over rosbridge
    hunt_socks(scenario="multi", seed=0)        # the nav sim

Per sock it runs the approach algorithm (`controller.Approach`: look, scan a full circle,
turn to the nearest sock, drive in legs, stop with the sock in the pick zone `nav.goal`), then
calls `on_reached` (the arm's pick goes there) and looks for the next one. A sock it already
reached is remembered by its odometry position and ignored afterwards, so a sock left on the
floor (no arm yet) is not approached again. Ctrl+C stops the rover.

`gap_m=0.05` switches to the fast approach (`approach_sock`): find the sock, one fast leg if it
is far, then one move that stops the front bumper `gap_m` short of the sock's near edge. The
last ~15 cm are under the camera's view: that bit is driven on the odometry. The Rover tab's
RUN ROBOT button runs it.

CLI: `python -m sorter.nav hunt [--real] [--socks N] [--gap 0.30] [--detector classic|sam3]
[--out DIR]`.
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from sorter.nav.camera import Frame
from sorter.nav.commands import FRONT_M, SOCK_MAX_H, Rover
from sorter.nav.controller import CONFIDENT, MAX_STEPS, Approach
from sorter.nav.detect import Detection

VISITED_M = 0.3  # a detection this close to a reached sock (odometry frame) is that sock
GAP_M = 0.30  # the fast approach: front bumper to the sock's near edge at the stop
FAST_LEG_M = 1.0  # farther than this: one fast leg first, then measure again from closer
FAST_LEG_STOP_M = 0.7  # where that leg stops (sock center from the rover's center)
FAST_MAX_MOVES = 20
SEEK_RANGE_M = 6.0
FAST_EXPLORE = 3  # drives toward open floor when no sock is seen all round
HALF_SOCK_M = 0.12  # the near edge is at most this much nearer than the center


@dataclass
class Reached:
    """A sock brought into the pick zone: where it is in the rover frame (camera estimate) and
    in the odometry frame of the hunt's start."""

    index: int
    x_m: float
    y_m: float
    odom_x_m: float
    odom_y_m: float
    commands: int

    def summary(self) -> dict:
        return {k: (round(v, 3) if isinstance(v, float) else v) for k, v in vars(self).items()}


@dataclass
class HuntReport:
    reached: list[Reached] = field(default_factory=list)
    commands: int = 0
    wall_s: float = 0.0
    stopped: str = ""  # why it ended: "done", "no sock found", "stopped by the operator"
    score: dict | None = None  # sim only: the ground truth at the end

    def summary(self) -> dict:
        return {
            "reached": [r.summary() for r in self.reached],
            "socks": len(self.reached),
            "commands": self.commands,
            "wall_s": round(self.wall_s, 1),
            "stopped": self.stopped,
            "score": self.score,
        }


class CachedDetector:
    """A detector that remembers its last frames' detections: the live panel's overlay and the
    algorithm ask about the same frame, and a SAM3 call takes ~1 s."""

    KEEP = 16

    def __init__(self, detector):
        self.detector = detector
        self.name = getattr(detector, "name", "detector")
        self._cache: dict[tuple, list[Detection]] = {}

    def detect(self, frame: Frame) -> list[Detection]:
        key = (frame.index, frame.t)
        if key not in self._cache:
            self._cache[key] = self.detector.detect(frame)
            while len(self._cache) > self.KEEP:
                del self._cache[next(iter(self._cache))]
        return self._cache[key]


class _PoseTagger:
    """Tags every captured frame with the odometry pose it was taken at (`frame.odom_pose`):
    a `scan`'s frames are detected on after the rover has turned on."""

    def __init__(self, camera, odom_of: Callable[[], object]):
        capture = camera.capture

        def tagged() -> Frame:
            f = capture()
            o = odom_of()
            f.odom_pose = (o.x, o.y, o.yaw)
            return f

        camera.capture = tagged


class _SkipVisited:
    """A detector that drops the socks already reached (by odometry position)."""

    def __init__(self, detector, odom_of: Callable[[], object], radius_m: float = VISITED_M):
        self.detector, self.odom_of, self.radius_m = detector, odom_of, radius_m
        self.name = getattr(detector, "name", "detector")
        self.visited: list[tuple[float, float]] = []

    def to_odom(self, frame: Frame, x: float, y: float) -> tuple[float, float]:
        pose = getattr(frame, "odom_pose", None)
        if pose is None:  # a frame reloaded from disk: taken where the rover stands now
            o = self.odom_of()
            pose = (o.x, o.y, o.yaw)
        ox, oy, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * x - s * y, oy + s * x + c * y

    def detect(self, frame: Frame) -> list[Detection]:
        dets = self.detector.detect(frame)
        if not self.visited:
            return dets
        out = []
        for d in dets:
            if d.x is not None:
                wx, wy = self.to_odom(frame, d.x, d.y)
                if any(math.hypot(wx - vx, wy - vy) < self.radius_m for vx, vy in self.visited):
                    continue
            out.append(d)
        return out


@dataclass
class FastResult:
    ok: bool
    sock: Detection | None  # the sock as last seen, before the final move
    frame: Frame | None  # the frame it was seen in
    commands: int
    note: str


def nearest_sock(detector, frame: Frame) -> Detection | None:
    """The nearest sock among the confident ones (score within `CONFIDENT` of the best)."""
    dets = [d for d in detector.detect(frame) if d.x is not None]
    if not dets:
        return None
    best = max(d.score for d in dets)
    return min((d for d in dets if d.score >= CONFIDENT * best), key=lambda d: d.distance)


def near_edge_m(frame: Frame, det: Detection) -> float:
    """Distance (m, from the rover's center) to the sock's near edge, the nearer of: the
    nearest mask point by depth, and the floor ray through the mask's lowest row (where a flat
    sock meets the floor; a little too far for a thick sock, so the depth wins there)."""
    lo = det.distance - HALF_SOCK_M  # no mask: a long sock pointing at the rover
    if det.mask is None or not det.mask.any():
        return lo
    near = det.distance
    p = frame.points()[det.mask]
    p = p[~np.isnan(p[:, 0]) & (p[:, 2] < SOCK_MAX_H)]
    if len(p) >= 20:
        near = min(near, float(np.percentile(np.hypot(p[:, 0], p[:, 1]), 3)))
    ys, xs = np.nonzero(det.mask)
    v = int(ys.max())
    q = frame.floor_point(float(np.median(xs[ys >= v - 2])), float(v))
    if q is not None:
        near = min(near, float(np.hypot(q[0], q[1])))
    if v >= frame.K.height - 3:  # cut off by the image's bottom: the edge is nearer still
        near = min(near, lo)
    return max(near, FRONT_M)


def approach_sock(
    rover: Rover,
    detector,
    gap_m: float = GAP_M,
    speed: float = 0.4,
    max_moves: int = FAST_MAX_MOVES,
) -> FastResult:
    """The fast approach: stop with the front bumper `gap_m` short of the nearest sock.

    Look; nothing in sight: `seek` (turn until the detector sees one, face it); nothing all
    round: drive toward the most open floor and seek again. A sock cut off
    by the image's side: turn toward it. Farther than `FAST_LEG_M`: one leg at `speed` to
    ~`FAST_LEG_STOP_M`, look again. Then turn to the sock and drive straight until the bumper
    is `gap_m` before its near edge (`near_edge_m`; the last bit is out of the camera's view:
    on the odometry). Speeds are capped by the rover (`nav.real.max_linear_mps` on the real one).
    """
    rover.detector = detector
    n = 0

    def do(name, *a):
        nonlocal n
        n += 1
        return rover.run(name, *a)

    res = do("look")
    centered = explored = 0
    views: list = []  # the last seek's views, while they are unchecked
    last_views: list = []
    while n < max_moves:
        frame = res.frame
        det = nearest_sock(detector, frame)
        if det is None:
            if views:  # seek wasn't sure: the nearest sock in any view it took on the way
                seen = [(h, nearest_sock(detector, f)) for h, f in views]
                seen = [(h, d) for h, d in seen if d is not None]
                views = []
                if seen:
                    h, d = min(seen, key=lambda hd: hd[1].distance)
                    res = do("turn", (h + d.bearing_deg + 180) % 360 - 180, 0.8)
                    continue
                if explored >= FAST_EXPLORE:
                    return FastResult(False, None, frame, n, "no sock in sight all round")
                # nothing all round (too far to see): drive toward the most open floor
                explored += 1
                h, free = max(
                    ((h, Approach._free_ahead(f)) for h, f in last_views),
                    key=lambda hf: min(hf[1], 2.5) - 0.4 * abs(hf[0]) / 180,
                )
                if abs(h) > 1:
                    do("turn", h, 0.8)
                res = do("forward", float(np.clip(free - 0.5, 0.3, 1.5)), speed)
                continue
            res = do("seek", 45.0, 360.0, 0.7, SEEK_RANGE_M)
            views = last_views = res.frames
            continue
        if Approach._cut_off(det, frame) and centered < 2 and abs(det.bearing_deg) > 3:
            centered += 1
            res = do("turn", det.bearing_deg + math.copysign(8.0, det.bearing_deg), 0.8)
            continue
        uv = frame.project((det.x, det.y, 0.0))
        u, v = uv if uv is not None else (det.u, det.v)
        if det.distance > FAST_LEG_M:
            stop = max(FAST_LEG_STOP_M, det.distance - 2.0)
            res = do("go_to_pixel", float(u), float(v), stop, speed, 0.0, math.inf, False)
            continue
        # the final move: face the sock, drive until the bumper is `gap_m` before its near
        # edge (measured now, from this frame: the last bit is out of the camera's view)
        drive = near_edge_m(frame, det) - FRONT_M - gap_m
        if drive <= 0.01:
            return FastResult(True, det, frame, n, f"already {gap_m + drive:.2f} m from it")
        if abs(det.bearing_deg) > 1.0:
            do("turn", det.bearing_deg, 0.8)
        res = do("forward", drive, min(speed, 0.4))
        if res.blocked is None:
            return FastResult(True, det, frame, n, f"stopped ~{gap_m:.2f} m before the sock")
        if not res.blocked.startswith("obstacle"):
            return FastResult(False, det, frame, n, res.blocked)
    return FastResult(False, None, res.frame, n, f"no stop in {max_moves} moves")


def _in_rover(o, wx: float, wy: float) -> tuple[float, float]:
    """An odometry-frame point in the rover's frame now."""
    dx, dy = wx - o.x, wy - o.y
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def _config(config_dir: str | Path, rosbridge: str | None):
    """(nav config, SAM config or None) from the repo config, or the defaults without it."""
    from sorter.nav.config import NavConfig

    full = None
    try:
        from sorter.core.config import load_config

        full = load_config(Path(config_dir))
    except Exception:  # noqa: BLE001 - the nav runs with its defaults without the repo config
        pass
    cfg = (full.nav if full is not None else NavConfig()).model_copy(deep=True)
    if rosbridge:
        cfg.real.rosbridge_url = rosbridge
    sam = full.color_classifier.sam if full is not None else None
    return cfg, sam


def hunt_socks(
    *,
    real: bool = False,
    max_socks: int = 1,
    detector: str | None = None,
    scenario: str = "multi",
    seed: int = 0,
    config_dir: str | Path = "config",
    rosbridge: str | None = None,
    out: str | Path | None = None,
    on_reached: Callable[[Reached], None] | None = None,
    max_steps: int = MAX_STEPS,
    gap_m: float | None = None,
    verbose: bool = True,
) -> HuntReport:
    """Find socks with the camera and drive up to them, one after another, until `max_socks`
    are reached or none is left in sight (the algorithm gives up after `max_steps` commands
    per sock).

    `real`: the real Leo and OAK-D (`nav.real`, `rosbridge` overrides its URL); else the nav
    sim with `scenario` and `seed`. `detector`: `classic` (no model) or `sam3` (the remote
    service, falls back to classic when it fails; the default on the real rover when its key
    is configured); `seg` is the sim's oracle. `out`: a directory for every command's frames
    and the log. `on_reached(reached)` runs with the rover stopped and the sock in the pick
    zone, before the next search (hand off to the arm here). Every command's result is printed
    as a JSON line if `verbose`. `gap_m`: the fast approach instead, stopping the bumper that
    far before each sock (`approach_sock`).
    """
    from sorter.nav import detect

    t0 = time.time()
    cfg, sam = _config(config_dir, rosbridge)
    if detector is None:
        detector = "sam3" if real and sam is not None else "classic"
    root = Path(out) if out else None
    ep = session = None
    if real:
        from sorter.nav.real import RealSession, record

        session = RealSession(cfg)
        rover, camera = session.rover, None
        save = (lambda res: record(root, res, cfg.goal)) if root else None
    else:
        from sorter.nav.episode import Episode
        from sorter.nav.scenario import make

        ep = Episode(make(scenario, seed), cfg, root)
        if root is not None:
            (root / "frames").mkdir(parents=True, exist_ok=True)
        rover, camera = ep.rover, ep.camera
        save = ep.record if root else None

    def odom():
        return rover.sim.odom

    report = HuntReport()
    run = rover.run
    try:
        det = _SkipVisited(detect.make(detector, camera, sam, cfg.sam_prompt), odom)
        _PoseTagger(rover.camera, odom)  # after the detector: `seg` wraps capture too

        def logged(name, *a, **k):
            res = run(name, *a, **k)
            if save is not None:
                save(res)
            if verbose:
                print(json.dumps({"sock": len(report.reached)} | res.summary()), flush=True)
            return res

        rover.run = logged
        report.stopped = "done"
        while len(report.reached) < max_socks:
            if gap_m is not None:
                fr = approach_sock(rover, det, gap_m)
                ok, s, seen, used = fr.ok, fr.sock, fr.frame, fr.commands
            else:
                ap = Approach(rover, det, cfg.goal, max_steps)
                ok = ap.run()
                s, seen, used = ap.last, rover.frame, ap.commands
                if root is not None:
                    k = len(report.reached)
                    log = json.dumps(ap.log.steps, indent=1)
                    (root / f"controller_log_{k}.json").write_text(log)
            report.commands += used
            if not ok or s is None:
                report.stopped = "no sock found"
                break
            # the sock the algorithm stopped at, in the frame it was last seen in
            wx, wy = det.to_odom(seen, s.x, s.y)
            det.visited.append((wx, wy))
            x, y = _in_rover(odom(), wx, wy)
            r = Reached(len(report.reached), x, y, wx, wy, used)
            report.reached.append(r)
            if verbose:
                print(json.dumps({"reached": r.summary()}), flush=True)
            if on_reached is not None:
                on_reached(r)
    except KeyboardInterrupt:
        report.stopped = "stopped by the operator"
    finally:
        rover.run = run
        if ep is not None:
            report.score = ep.score().summary()
            ep.close()
        if session is not None:
            session.close()  # stops the rover
    report.wall_s = time.time() - t0
    if root is not None:
        (root / "hunt.json").write_text(json.dumps(report.summary(), indent=1))
    return report

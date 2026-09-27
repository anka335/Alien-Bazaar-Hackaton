"""The full mission on the simulator (stage C): drive, load at every stop, drive to the station.

    from sorter.mission.mission import run_mission
    report = run_mission(scenario="mission", seed=0)

The rover and the arm live in two MuJoCo worlds (D-046): the Leo Rover drives in the nav world
(`sorter.nav`, rigid socks, fast), the arm picks in the arm world (`sorter.sim.physics`, cloth).
The mission hands over between them at every stop:

1. **Drive** (nav world): the approach algorithm (`nav.controller.Approach`) finds the nearest
   sock with the OAK-D and stops with it in the pick zone (`nav.goal`).
2. **Load** (arm world): every sock whose center lies in the arm's floor zone (the rig's
   `zones.floor.workspace_mm`, where it really lies: ground truth of the nav world) is placed at
   the same spot around a fresh arm world, and the real load loop (`orchestrator.load`, the
   same as `run --sim --mode load`) runs until it is done. The arm only moves while the rover
   stands: the two never run at the same time.
3. The socks the arm put in the cargo box vanish from the nav world; the ones it didn't stay
   where they were and are not approached again (`hunt._SkipVisited`).
4. Until the cargo box holds `capacity` socks or no sock is left in sight; then **to the
   station**: the station's AprilTags (`nav.boxes.approach_box`, tag `nav.boxes.target_id`),
   stopping `nav.boxes.stop_m` before it. Unloading there is stage B (not run here).

Each arm world starts empty: the cargo box's contents are counted by the mission, not carried
from stop to stop.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.types import ColorClass, Zone
from sorter.sim.scenes.load.config import PlacedSock

log = logging.getLogger(__name__)

HIDDEN_XY = (30.0, 30.0)  # where a loaded sock goes in the nav world: far outside the room
LOAD_TIME_LIMIT_S = 900.0  # simulated s of one stop's load run


@dataclass
class Stop:
    index: int
    commands: int  # nav commands to get there
    handed: list[int]  # nav sock indices placed around the arm
    loaded: list[int]  # of those, in the cargo box at the end
    end: str  # the load run's end: done / error: ... / time limit
    sim_s: float  # the load run's simulated time
    wall_s: float

    def summary(self) -> dict:
        return {k: (round(v, 1) if isinstance(v, float) else v) for k, v in vars(self).items()}


@dataclass
class MissionReport:
    socks: int = 0  # in the room at the start
    stops: list[Stop] = field(default_factory=list)
    cargo: dict[str, int] = field(default_factory=dict)  # color class → socks in the box
    station: dict | None = None  # the drive to the station
    unload: dict | None = None  # the unload run at the station
    stopped: str = ""  # why the collecting ended
    wall_s: float = 0.0

    @property
    def loaded(self) -> int:
        return sum(self.cargo.values())

    def summary(self) -> dict:
        return {
            "socks": self.socks,
            "loaded": self.loaded,
            "left": self.socks - self.loaded,
            "cargo": self.cargo,
            "stops": [s.summary() for s in self.stops],
            "collecting_ended": self.stopped,
            "station": self.station,
            "unload": self.unload,
            "wall_s": round(self.wall_s, 1),
        }


def color_class(rgb: tuple[float, float, float]) -> ColorClass:
    """The laundry class of a nav sock's color: dark, colored (saturated) or light."""
    hi, lo = max(rgb), min(rgb)
    lum = sum(rgb) / 3
    if lum < 0.3:
        return ColorClass.DARK
    if hi > 0 and (hi - lo) / hi > 0.35:
        return ColorClass.COLORED
    return ColorClass.LIGHT if lum > 0.6 else ColorClass.DARK


def rover_to_arm_mm(cfg: Config, x_m: float, y_m: float) -> tuple[float, float]:
    """A floor point in the Leo's frame (m, from its center) in the arm base frame (mm): the
    rover's center is `sim.layout.body.center_mm` from the arm's base."""
    cx, cy = cfg.sim.layout.body.center_mm
    return x_m * 1000 + cx, y_m * 1000 + cy


def socks_in_reach(cfg: Config, sim, on_floor: set[int]) -> list[tuple[int, PlacedSock]]:
    """The nav world's socks (ground truth) whose center is in the arm's floor zone now."""
    from sorter.arm.controller import in_polygon

    zone = [tuple(p) for p in cfg.zones[Zone.FLOOR].workspace_mm]
    _, _, rover_yaw = sim.true_pose()
    out = []
    for i in sorted(on_floor):
        x, y = rover_to_arm_mm(cfg, *sim.sock_in_rover(i))
        if not in_polygon(x, y, zone):
            continue
        it = sim.spec.socks[i]
        R = sim.data.xmat[sim._sock_bodies[i]].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0]) - rover_yaw
        placed = PlacedSock(
            color=color_class(it.color),
            x_mm=round(x, 1),
            y_mm=round(y, 1),
            yaw_rad=round(yaw, 3),
            rgb=tuple(it.color),
            bunched=it.kind == "sock_bunched",
        )
        out.append((i, placed))
    return out


def hide_sock(sim, i: int) -> None:
    """Take nav sock `i` out of the room (the arm put it in the cargo box)."""
    m, d = sim.model, sim.data
    j = m.body_jntadr[sim._sock_bodies[i]]
    q, v = m.jnt_qposadr[j], m.jnt_dofadr[j]
    d.qpos[q : q + 3] = (HIDDEN_XY[0] + i, HIDDEN_XY[1], 0.02)
    d.qvel[v : v + 6] = 0
    mujoco.mj_forward(m, d)


def load_stop(
    config_dir: str | Path,
    placed: list[PlacedSock],
    seed: int,
    video: Path | None = None,
    time_limit_s: float = LOAD_TIME_LIMIT_S,
    arm_speed: float = 1.4,
    cancelled: Callable[[], bool] | None = None,
) -> tuple[list[bool], str, float]:
    """One stop's load run in a fresh arm world with `placed` around the rover: (in the cargo
    box, per sock; how it ended; simulated s)."""
    from sorter.app import build_system
    from sorter.core.types import Command, OperatorMode, Phase
    from sorter.orchestrator.state_machine import StateMachine

    cfg = load_config(
        config_dir,
        overrides={
            "sim": {
                "realtime": 0,
                "seed": seed,
                "scenes": ["load"],
                "load": {"placed": [p.model_dump(mode="json") for p in placed]},
            },
            "backends": dict.fromkeys(("camera", "arm"), "sim"),
            "arm": {"speed_scale": arm_speed},
            "state_machine": {"save_runs": False},
        },
    )
    system = build_system(cfg, sim=True)
    world = system.world
    assert world is not None
    system.camera.start()
    sm = StateMachine(system)
    system.hub.set_mode(OperatorMode.LOAD)
    stop = threading.Event()
    thread = threading.Thread(target=sm.run, args=(stop,), name="state-machine", daemon=True)
    thread.start()
    system.hub.send(Command.START)
    rec = _ArmVideo(world, system.camera, video, cfg) if video is not None else None
    t0, end, last = world.time(), "time limit", None
    try:
        while world.time() - t0 < time_limit_s:
            if cancelled is not None and cancelled():
                from sorter.nav.commands import Cancelled

                raise Cancelled
            if rec is not None:
                rec.frame(f"{world.time() - t0:5.1f} s  {sm.phase.value}")
            d = system.hub.decision()
            if d is not None and d is not last:
                last = d
                log.info("  arm %6.1f s  %-12s %s", world.time() - t0, d.phase.value, d.summary)
            if sm.phase in (Phase.DONE, Phase.ERROR) and sm.mode != "running":
                end = "done" if sm.phase is Phase.DONE else f"error: {sm.error}"
                break
            time.sleep(0.01 if rec is None else 0.001)
        loaded = [world.location(it.id)[0] == "cargo" for it in world.items]
        return loaded, end, world.time() - t0
    finally:
        stop.set()
        thread.join(timeout=5)
        if rec is not None:
            rec.close()
        system.camera.close()
        world.stop()


def unload_stop(
    cargo: dict[str, int],
    seed: int,
    video: Path | None = None,
    cancelled: Callable[[], bool] | None = None,
    arm_speed: float = 1.4,
) -> dict:
    """The unload run at the station (stage B's loop) in a fresh arm world with `cargo` (socks
    per color class) in the cargo box; the station parks within B's tolerance (±30 mm, ±5°).
    Returns where every sock ended (the sim's truth) and the loop's own count."""
    from sorter.app import build_system
    from sorter.core.types import Command, OperatorMode, Phase
    from sorter.orchestrator.state_machine import StateMachine
    from sorter.sim.scenes.unload.bench import _truth_bins, _where, unload_config

    cfg = unload_config(
        {
            "sim": {
                "seed": seed,
                "realtime": 0,
                "unload": {
                    "cargo": {c.value: cargo.get(c.value, 0) for c in ColorClass},
                    "sock_mm": [200, 90],
                    "station_mm": 30,
                    "station_deg": 5,
                    "bin_mm": 15,
                    "bin_deg": 8,
                },
            },
            "backends": dict.fromkeys(("camera", "arm"), "sim"),
            "arm": {"speed_scale": arm_speed},
            "state_machine": {"save_runs": False},
        }
    )
    system = build_system(cfg, sim=True)
    world = system.world
    assert world is not None
    system.camera.start()
    sm = StateMachine(system)
    stop = threading.Event()
    thread = threading.Thread(target=sm.run, args=(stop,), name="state-machine", daemon=True)
    thread.start()
    system.hub.set_mode(OperatorMode.UNLOAD)
    system.hub.send(Command.START)
    rec = _ArmVideo(world, system.camera, video, cfg) if video is not None else None
    limit = 90.0 * sum(cargo.values()) + 60.0
    t0, end, last = world.time(), "time limit", None
    try:
        while world.time() - t0 < limit:
            if cancelled is not None and cancelled():
                from sorter.nav.commands import Cancelled

                raise Cancelled
            if rec is not None:
                rec.frame(f"{world.time() - t0:5.1f} s  unload  {sm.phase.value}")
            d = system.hub.decision()
            if d is not None and d is not last:
                last = d
                log.info("  unload %6.1f s  %-14s %s", world.time() - t0, d.phase.value, d.summary)
            if sm.run_id is not None and (
                sm.phase is Phase.ERROR or (sm.mode == "idle" and sm.phase is Phase.DONE)
            ):
                end = "done" if sm.phase is Phase.DONE else f"error: {sm.error}"
                break
            time.sleep(0.01 if rec is None else 0.001)
        bins = _truth_bins(world)
        right: Counter[str] = Counter()
        wrong = left = 0
        for it in world.items:
            at, where = _where(world, it.id, bins)
            if at == "laundry" and where == it.color.value:
                right[it.color.value] += 1
            elif at == "laundry":
                wrong += 1
            else:
                left += 1
        return {
            "end": end,
            "sim_s": round(world.time() - t0, 1),
            "sorted": dict(right),
            "right": sum(right.values()),
            "wrong_bin": wrong,
            "not_in_a_bin": left,
            "counted": {c.value: n for c, n in sm.counters.items() if n},
        }
    finally:
        stop.set()
        thread.join(timeout=5)
        if rec is not None:
            rec.close()
        system.camera.close()
        world.stop()


class _ArmVideo:
    """An MP4 of a stop: an overview of the rover and the wrist camera, one frame per 0.1 s of
    simulated time (like `sim.scenes.load.watch --record`)."""

    W, H = 640, 480

    def __init__(self, world, camera, path: Path, cfg: Config):
        import cv2

        self.cv2, self.world, self.camera, self.path = cv2, world, camera, path
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.out = cv2.VideoWriter(str(path), fourcc, 20, (2 * self.W, self.H))
        self.renderer = mujoco.Renderer(world.model, self.H, self.W)
        self.data = mujoco.MjData(world.model)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = (0.05, 0.02, cfg.sim.layout.floor_z_mm / 1000 + 0.12)
        self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.35, -140, -38
        self.t0, self.next = world.time(), 0.0

    def frame(self, text: str) -> None:
        t = self.world.time() - self.t0
        if t < self.next:
            return
        self.next = max(self.next + 0.1, t)
        with self.world.lock:
            mujoco.mj_copyData(self.data, self.world.model, self.world.data)
        self.renderer.update_scene(self.data, camera=self.cam)
        left = self.renderer.render()[..., ::-1].copy()
        live = self.camera.latest()
        right = (
            self.cv2.resize(live.color, (self.W, self.H))
            if live is not None
            else np.zeros((self.H, self.W, 3), np.uint8)
        )
        self.cv2.putText(left, text, (10, 28), 0, 0.7, (255, 255, 255), 2, self.cv2.LINE_AA)
        self.out.write(np.hstack([left, right]))

    def close(self) -> None:
        self.out.release()
        self.renderer.close()


def run_mission(
    *,
    scenario: str = "mission",
    seed: int = 0,
    config_dir: str | Path = DEFAULT_CONFIG_DIR,
    capacity: int = 6,
    max_stops: int = 8,
    detector: str = "classic",
    out: str | Path | None = None,
    video: bool = False,
    arm: bool = True,
    on_event: Callable[[dict], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
    on_frame: Callable[[object], None] | None = None,
    unload: bool = True,
) -> MissionReport:
    """The whole mission on the sim (see the module doc). `capacity`: socks the cargo box takes
    before the rover drives to the station. `out`: the nav frames and log, `mission.json` and,
    with `video`, an MP4 per stop. `arm=False`: no arm world, every sock in the zone counts as
    loaded (to check the driving fast). `on_event(dict)`: every step, as a JSON-able dict.
    `cancelled()` True stops the mission (the rover's command raises `Cancelled`); `on_frame`
    gets every OAK-D frame a command ends with."""
    from sorter.nav import detect
    from sorter.nav.boxes import approach_box
    from sorter.nav.controller import MAX_STEPS, Approach
    from sorter.nav.episode import Episode
    from sorter.nav.hunt import CachedDetector, _PoseTagger, _SkipVisited
    from sorter.nav.scenario import make

    t0 = time.time()
    cfg = load_config(config_dir)
    spec = make(scenario, seed)
    root = Path(out) if out else None
    if root is not None:
        (root / "frames").mkdir(parents=True, exist_ok=True)
    ep = Episode(spec, cfg.nav, root)
    rover, sim = ep.rover, ep.sim
    report = MissionReport(socks=len(spec.socks))
    cargo: Counter[str] = Counter()

    def event(kind: str, **data) -> None:
        e = {"event": kind, "t_s": round(time.time() - t0, 1)} | data
        log.info("%s", json.dumps(e))
        if on_event is not None:
            on_event(e)

    def odom():
        return sim.odom

    run = rover.run
    if cancelled is not None:
        rover.cancelled = cancelled

    def recorded(name, *a, **k):
        res = run(name, *a, **k)
        if root is not None:
            ep.record(res)
        if on_frame is not None and res.frame is not None:
            on_frame(res.frame)
        return res

    rover.run = recorded
    try:
        sam = cfg.color_classifier.sam
        found = CachedDetector(detect.make(detector, ep.camera, sam, cfg.nav.sam_prompt))
        det = _SkipVisited(found, odom)
        _PoseTagger(rover.camera, odom)
        on_floor = set(range(len(spec.socks)))
        event("start", scenario=scenario, seed=seed, socks=len(spec.socks), capacity=capacity)
        report.stopped = "capacity"
        while report.loaded < capacity:
            if not on_floor:
                report.stopped = "no sock left"
                break
            if len(report.stops) >= max_stops:
                report.stopped = f"{max_stops} stops"
                break
            ap = Approach(rover, det, cfg.nav.goal, MAX_STEPS)
            ok = ap.run()
            if not ok or ap.last is None:
                report.stopped = "no sock found"
                break
            s = ap.last
            det.visited.append(det.to_odom(rover.frame, s.x, s.y))
            handed = socks_in_reach(cfg, sim, on_floor)
            k = len(report.stops)
            event("stopped", stop=k, commands=ap.commands, socks=[i for i, _ in handed])
            t1 = time.time()
            if not handed:
                report.stops.append(Stop(k, ap.commands, [], [], "no sock in the arm's zone", 0, 0))
                continue
            placed = [p for _, p in handed]
            if arm:
                vid = root / f"stop_{k}.mp4" if (video and root is not None) else None
                ok_each, end, sim_s = load_stop(
                    config_dir, placed, seed * 100 + k, vid, cancelled=cancelled
                )
            else:
                ok_each, end, sim_s = [True] * len(placed), "done (no arm)", 0.0
            loaded = []
            for (i, p), in_box in zip(handed, ok_each, strict=True):
                if in_box and report.loaded < capacity:
                    hide_sock(sim, i)
                    on_floor.discard(i)
                    loaded.append(i)
                    cargo[p.color.value] += 1
                    report.cargo = dict(cargo)
            ids = [i for i, _ in handed]
            stop = Stop(k, ap.commands, ids, loaded, end, sim_s, time.time() - t1)
            report.stops.append(stop)
            event("loaded", **stop.summary(), cargo=dict(cargo))
        event("to_station", collecting_ended=report.stopped, cargo=dict(cargo))
        b = cfg.nav.boxes
        home = None
        if spec.station is not None:  # the station is where the rover started from: go home
            home = drive_home(rover, sim, _in_start_frame(spec, spec.station))
            event("home", **home)
        res = approach_box(rover, b.target_id, b.stop_m, b.station_tag_m)
        report.station = {
            "home": home,
            "ok": res.ok,
            "gap_m": None if res.gap_m is None else round(res.gap_m, 3),
            "commands": res.commands,
            "note": res.note,
        }
        if spec.station is not None:
            report.station["true_gap_m"] = round(_station_gap(sim, spec.station), 3)
        event("at_station", **report.station)
        if unload and arm and res.ok and report.loaded:
            event("unloading", cargo=dict(cargo))
            vid = root / "unload.mp4" if (video and root is not None) else None
            report.unload = unload_stop(dict(cargo), seed, vid, cancelled=cancelled)
            event("unloaded", **report.unload)
    finally:
        rover.run = run
        ep.close()
    report.wall_s = time.time() - t0
    if root is not None:
        (root / "mission.json").write_text(json.dumps(report.summary(), indent=1))
    return report


HOME_M = 1.2  # the point to go home to: this far in front of the station's face (its center)
HOME_TOL_M = 0.25
HOME_LEG_M = 1.5  # drive at most this far before re-planning from the odometry
HOME_MAX_MOVES = 12


def _in_start_frame(spec, pose) -> tuple[float, float, float]:
    """A world pose (x, y, yaw) in the odometry frame: the rover's start pose is its origin."""
    rx, ry, ryaw = spec.rover
    dx, dy = pose[0] - rx, pose[1] - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return c * dx + s * dy, -s * dx + c * dy, pose[2] - ryaw


def drive_home(rover, sim, station_odom: tuple[float, float, float]) -> dict:
    """Drive on the odometry to `HOME_M` in front of the station (known in the odometry frame:
    where the rover started) and face it, so its tags are in view for the final approach.
    Legs of at most `HOME_LEG_M`, re-planned from the odometry; an obstacle: turn aside, go
    round."""
    from sorter.nav.model import STATION_HALF_M

    sx, sy, syaw = station_odom
    d = STATION_HALF_M[0] + HOME_M
    gx, gy = sx + d * math.cos(syaw), sy + d * math.sin(syaw)
    out = drive_to(rover, sim, gx, gy)
    o = sim.odom
    face = _wrap_deg(math.degrees(math.atan2(sy - o.y, sx - o.x) - o.yaw))
    rover.run("turn", face, 1.0)
    return out


def drive_to(rover, base, gx: float, gy: float, heading: float | None = None) -> dict:
    """Drive on the odometry to (gx, gy) (odometry frame, m), then turn to `heading` (rad) if
    given. Legs of at most `HOME_LEG_M`, re-planned from the odometry; an obstacle: turn
    aside, go round."""
    sim = base
    moves, aside = 0, 1
    while moves < HOME_MAX_MOVES:
        o = sim.odom
        dx, dy = gx - o.x, gy - o.y
        dist = math.hypot(dx, dy)
        if dist <= HOME_TOL_M:
            break
        turn = _wrap_deg(math.degrees(math.atan2(dy, dx) - o.yaw))
        if abs(turn) > 3:
            rover.run("turn", turn, 1.0)
            moves += 1
        res = rover.run("forward", min(dist, HOME_LEG_M), 0.4)
        moves += 1
        if res.blocked and res.blocked.startswith("obstacle"):  # go round: 50° aside, 0.6 m
            rover.run("turn", 50.0 * aside, 1.0)
            rover.run("forward", 0.6, 0.3)
            moves += 2
            aside = -aside
    o = sim.odom
    if heading is not None:
        rover.run("turn", _wrap_deg(math.degrees(heading - o.yaw)), 1.0)
    return {"moves": moves, "left_m": round(math.hypot(gx - o.x, gy - o.y), 2)}


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _station_gap(sim, station) -> float:
    """Ground truth: the rover's front bumper to the station's face, along the face normal."""
    from sorter.nav.commands import FRONT_M
    from sorter.nav.model import STATION_HALF_M

    x, y, yaw = sim.true_pose()
    sx, sy, syaw = station
    nx, ny = math.cos(syaw), math.sin(syaw)
    face = (x - sx) * nx + (y - sy) * ny - STATION_HALF_M[0]  # rover center to the face
    return face - FRONT_M

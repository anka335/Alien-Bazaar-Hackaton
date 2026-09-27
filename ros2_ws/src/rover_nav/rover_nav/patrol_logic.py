"""A hard-coded patrol (D-046), without ROS: odometry in, velocity command out.

The rover drives a square or a circle of `size_m` (square: side; circle: diameter) from where
it stands, counter-clockwise (turning left), and at every stop turns once all the way around in
`scan_step_deg` steps, pausing `dwell_s` at each, so a camera can look in every direction. Back
at the start it faces the way it started. No map: the rover's odometry (wheels + IMU).

The route is a list of simple steps, planned once from the start pose:
    Turn(yaw)           turn in place to an absolute heading (odom frame)
    Drive(start, end)   straight from start to end, holding the line
    Dwell(seconds)      stand still (a look during a scan)
Square: the stops are the corners. Circle: `circle_stops` points on the circle, driven as
straight pieces between them (precise on odometry; a true arc isn't needed to look around).

`step(t)` is called at a fixed rate and returns (linear m/s, angular rad/s); stale odometry
(> `odom_timeout_s`) → stand still until it's back.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PatrolParams:
    shape: str = "square"  # square | circle
    size_m: float = 1.0  # square: side; circle: diameter
    circle_stops: int = 8  # circle: stops (= straight pieces) around it
    laps: int = 1  # 0 = until stopped
    scan_at_start: bool = True  # look around before driving off
    scan_step_deg: float = 90.0  # a scan turns 360° in steps of this
    dwell_s: float = 2.0  # pause at each step of a scan
    speed: float = 0.15  # m/s on the straight pieces
    min_speed: float = 0.04
    slow_down_m: float = 0.25
    turn_speed: float = 0.5  # rad/s max
    min_turn_speed: float = 0.15  # rad/s; slower and the skid-steer rover doesn't turn
    turn_tolerance_deg: float = 2.0
    distance_tolerance_m: float = 0.01
    heading_kp: float = 1.5
    cross_track_kp: float = 2.0
    max_heading_correction: float = 0.3
    odom_timeout_s: float = 0.5


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Turn:
    yaw: float
    label: str = ""


@dataclass(frozen=True)
class Drive:
    start: tuple[float, float]
    end: tuple[float, float]
    label: str = ""


@dataclass(frozen=True)
class Dwell:
    seconds: float
    label: str = ""


Step = Turn | Drive | Dwell


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def route_points(p: PatrolParams, start: Pose2D) -> list[tuple[float, float]]:
    """The stops after the start, in the odom frame; the last one is the start again."""
    if p.shape == "square":
        local = [(p.size_m, 0.0), (p.size_m, p.size_m), (0.0, p.size_m), (0.0, 0.0)]
    elif p.shape == "circle":
        if p.circle_stops < 3:
            raise ValueError("circle_stops must be at least 3")
        r = p.size_m / 2
        # centre to the rover's left; the start is the circle's point right below the centre
        local = []
        for k in range(1, p.circle_stops + 1):
            a = -math.pi / 2 + 2 * math.pi * k / p.circle_stops
            local.append((r * math.cos(a), r + r * math.sin(a)))
        local[-1] = (0.0, 0.0)
    else:
        raise ValueError("shape must be square or circle")
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return [(start.x + c * x - s * y, start.y + s * x + c * y) for x, y in local]


def _scan(p: PatrolParams, heading: float, where: str) -> list[Step]:
    """Once around from `heading`: to +step, dwell, … back to `heading` (dwell there too)."""
    n = max(1, round(360.0 / p.scan_step_deg))
    steps: list[Step] = [Dwell(p.dwell_s, f"scan {where}: look 1/{n}")]
    for k in range(1, n):
        steps.append(Turn(wrap(heading + math.radians(p.scan_step_deg) * k), f"scan {where}"))
        steps.append(Dwell(p.dwell_s, f"scan {where}: look {k + 1}/{n}"))
    steps.append(Turn(heading, f"scan {where}: back"))
    return steps


def plan_lap(p: PatrolParams, start: Pose2D, lap: int) -> list[Step]:
    """One lap from `start`: [scan], then per stop: turn towards it, drive, scan; finally face
    the start heading again."""
    steps: list[Step] = []
    if p.scan_at_start and lap == 1:
        steps += _scan(p, start.yaw, "at the start")
    here, heading = (start.x, start.y), start.yaw
    points = route_points(p, start)
    for i, pt in enumerate(points, 1):
        heading = math.atan2(pt[1] - here[1], pt[0] - here[0])
        name = "the start" if i == len(points) else f"stop {i}"
        steps.append(Turn(heading, f"lap {lap}: towards {name}"))
        steps.append(Drive(here, pt, f"lap {lap}: to {name}"))
        steps += _scan(p, heading, f"lap {lap}, {name}")
        here = pt
    steps.append(Turn(start.yaw, f"lap {lap}: facing the start heading"))
    return steps


class PatrolLogic:
    def __init__(self, params: PatrolParams | None = None):
        self.p = params or PatrolParams()
        self.steps: list[Step] = []
        self.index = 0
        self.lap = 0
        self.done = False
        self.message = "waiting for odometry"
        self._start: Pose2D | None = None
        self._pose: Pose2D | None = None
        self._odom_t: float | None = None
        self._dwell_until: float | None = None

    @property
    def current(self) -> Step | None:
        return self.steps[self.index] if self.index < len(self.steps) else None

    @property
    def scanning(self) -> bool:
        """True while pausing during a scan: the moment for a camera to look."""
        step = self.current
        return isinstance(step, Dwell) and self._dwell_until is not None

    def odom(self, pose: Pose2D, t: float) -> None:
        self._pose, self._odom_t = pose, t

    def step(self, t: float) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        if self._pose is None or t - self._odom_t > self.p.odom_timeout_s:
            self.message = "waiting for odometry"
            return 0.0, 0.0
        if self._start is None:
            self._start = self._pose
            self._next_lap()
        for _ in range(len(self.steps) + 1):  # finish zero-time steps in the same tick
            step = self.current
            if step is None:
                if self.p.laps and self.lap >= self.p.laps:
                    self.done, self.message = True, "done"
                    return 0.0, 0.0
                self._next_lap()
                continue
            self.message = step.label
            cmd = self._run(step, t)
            if cmd is not None:
                return cmd
            self.index += 1
            self._dwell_until = None
        return 0.0, 0.0

    def _next_lap(self) -> None:
        self.lap += 1
        self.steps = plan_lap(self.p, self._start, self.lap)
        self.index = 0

    def _run(self, step: Step, t: float) -> tuple[float, float] | None:
        """The command for `step`, or None when it's finished."""
        if isinstance(step, Dwell):
            if self._dwell_until is None:
                self._dwell_until = t + step.seconds
            return (0.0, 0.0) if t < self._dwell_until else None
        if isinstance(step, Turn):
            error = wrap(step.yaw - self._pose.yaw)
            if abs(error) <= math.radians(self.p.turn_tolerance_deg):
                return None
            speed = min(self.p.turn_speed, max(self.p.min_turn_speed, 1.5 * abs(error)))
            return 0.0, math.copysign(speed, error)
        return self._drive(step)

    def _drive(self, step: Drive) -> tuple[float, float] | None:
        (sx, sy), (ex, ey) = step.start, step.end
        length = math.hypot(ex - sx, ey - sy)
        heading = math.atan2(ey - sy, ex - sx)
        dx, dy = self._pose.x - sx, self._pose.y - sy
        along = dx * math.cos(heading) + dy * math.sin(heading)
        cross = -dx * math.sin(heading) + dy * math.cos(heading)  # left positive
        remaining = length - along
        if remaining <= self.p.distance_tolerance_m:
            return None
        v = self.p.speed
        if remaining < self.p.slow_down_m:
            v = max(self.p.min_speed, self.p.speed * remaining / self.p.slow_down_m)
        steer = self.p.heading_kp * wrap(heading - self._pose.yaw) - self.p.cross_track_kp * cross
        return v, _clip(steer, self.p.max_heading_correction)

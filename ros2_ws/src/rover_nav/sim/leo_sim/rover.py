"""The simulated Leo Rover: cmd_vel in, wheel odometry, cameras and closed-loop motions out.

It mimics what matters of the real rover (LeoOS firmware + odom_filter):
- `set_cmd_vel(v, w)` like /cmd_vel; the wheels get the skid-steer speeds and stop 0.5 s after
  the last command (the firmware's `input_timeout`);
- `odom()` integrates the wheel speeds for distance and the gyro for heading, like
  /merged_odom, so it drifts when the wheels slip; `pose()` is the ground truth;
- `move()` / `turn()` are closed loop on that odometry, with the outcomes of leo-rover-mcp
  (done, stalled, stopped, timeout; tilted above 30 deg, where leo-rover-mcp refuses motion).

Rendering needs an OpenGL context: headless, set MUJOCO_GL=egl (or osmesa). All calls must come
from the thread that created the sim (the GL context is bound to it); see runner.py.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import mujoco
import numpy as np

from .model import (
    BASE_HEIGHT,
    CAMERAS,
    ROVER_VISUAL_GROUP,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
    WHEELS,
    World,
    build_mjcf,
    default_world,
)

CONTROL_DT = 0.02  # firmware-like control tick, s
CMD_TIMEOUT = 0.5  # firmware input_timeout, s
MAX_WHEEL_SPEED = 6.4  # rad/s, ~0.4 m/s at the rim
MAX_LINEAR = 0.4  # m/s, leo-rover-mcp limit
MAX_ANGULAR = math.radians(57)  # rad/s, leo-rover-mcp limit
LINEAR_ACCEL = 1.0  # m/s^2
ANGULAR_ACCEL = 3.0  # rad/s^2
# Skid-steer wheels slip sideways while turning, so the wheel speed difference must exceed
# w * separation. The firmware scales w by this before splitting it into wheel speeds
# (angular_velocity_multiplier); the value is measured on this sim (tests/test_rover.py).
ANGULAR_MULTIPLIER = 2.2
STALL_WINDOW = 1.5  # s without odometry progress while commanded -> stalled
MAX_TILT = 30.0  # deg: motions end `tilted` above this (leo-rover-mcp refuses to move)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Motion:
    kind: str  # move | turn
    target: float  # metres or radians (signed)
    speed: float  # m/s or rad/s (positive)
    start: tuple[float, float, float]
    timeout: float
    elapsed: float = 0.0
    outcome: str = "running"  # running | done | stalled | stopped | timeout | tilted
    turned: float = 0.0  # radians turned so far (turn motions)
    bumped: set[str] = field(default_factory=set)  # world bodies touched on the way

    def progress(self, odom: tuple[float, float, float]) -> float:
        if self.kind == "move":
            x0, y0, h0 = self.start
            return (odom[0] - x0) * math.cos(h0) + (odom[1] - y0) * math.sin(h0)
        return self.turned


class LeoSim:
    def __init__(self, world: World | None = None, *, seed: int | None = None):
        self.world = world or default_world(seed)
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(self.world))
        self.data = mujoco.MjData(self.model)
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        m = self.model
        self._root_q = m.joint("root").qposadr[0]
        self._root_v = m.joint("root").dofadr[0]
        self._wheel_v = {n: m.joint(f"wheel_{n}").dofadr[0] for n in WHEELS}
        self._act = {n: m.actuator(f"wheel_{n}").id for n in WHEELS}
        self._rover_bodies = self._subtree(m.body("base_link").id)
        self._obstacle_geoms = {
            g
            for g in range(m.ngeom)
            if m.geom_bodyid[g] not in self._rover_bodies
            and m.geom(g).name != "floor"
            and (m.geom_contype[g] or m.geom_conaffinity[g])
        }
        self.reset()

    # ---- state -------------------------------------------------------------------------

    def _subtree(self, root: int) -> set[int]:
        bodies = {root}
        for b in range(self.model.nbody):
            parent = self.model.body_parentid[b]
            while parent not in bodies and parent != 0:
                parent = self.model.body_parentid[parent]
            if parent in bodies:
                bodies.add(b)
        return bodies

    @property
    def time(self) -> float:
        return float(self.data.time)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.cmd = (0.0, 0.0)  # commanded (v, w)
        self._cmd_time = -1e9
        self._ramped = (0.0, 0.0)  # (v, w) after acceleration limits
        self._odom = [0.0, 0.0, 0.0]
        self._odom_offset = self._true_pose()
        self.motion: Motion | None = None
        self._progress = deque()  # (time, progress) for stall detection
        self.trail: list[tuple[float, float]] = [self.pose()[:2]]
        self._next_control = 0.0
        self._settle(0.3)
        self.reset_odometry()

    def place(self, x: float, y: float, yaw: float) -> bool:
        """Teleport the parked rover to (x, y, yaw) and let it settle. False if it touches
        anything there (it is then left in place anyway)."""
        q, v = self._root_q, self._root_v
        self.data.qpos[q : q + 3] = (x, y, BASE_HEIGHT + 0.005)
        self.data.qpos[q + 3 : q + 7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        self.data.qvel[v : v + 6] = 0.0
        self.motion = None
        self.set_cmd_vel(0.0, 0.0)
        self._ramped = (0.0, 0.0)
        mujoco.mj_forward(self.model, self.data)
        self._settle(0.3)
        self.trail = [self.pose()[:2]]
        self.reset_odometry()
        return not self.touching()

    def _settle(self, seconds: float) -> None:
        for _ in range(int(seconds / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)

    def _true_pose(self) -> tuple[float, float, float]:
        q = self.data.qpos[self._root_q : self._root_q + 7]
        w, x, y, z = q[3:7]
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return float(q[0]), float(q[1]), yaw

    def pose(self) -> tuple[float, float, float]:
        """Ground truth: base_footprint x, y (m) and yaw (rad) in the world."""
        return self._true_pose()

    def odom(self) -> tuple[float, float, float]:
        """Odometry since the last reset: x forward / y left at reset, heading CCW (rad)."""
        return tuple(self._odom)

    def at_rest(self) -> bool:
        """No motion running and no command for over a second: the rover is parked."""
        running = self.motion is not None and self.motion.outcome == "running"
        return not running and self.time - self._cmd_time > CMD_TIMEOUT + 1.0

    def reset_odometry(self) -> None:
        self._odom = [0.0, 0.0, 0.0]

    def velocity(self) -> tuple[float, float]:
        """Measured (v, w) in the rover frame: the wheel mean and the gyro."""
        wheels = [self.data.qvel[self._wheel_v[n]] for n in WHEELS]
        v = WHEEL_RADIUS * float(np.mean(wheels))
        gyro = float(self.data.qvel[self._root_v + 5])  # world z rate of the free joint
        return v, gyro

    def tilt(self) -> float:
        """Angle of the rover's up axis from vertical, degrees."""
        zz = self.data.body("base_link").xmat[8]
        return math.degrees(math.acos(max(-1.0, min(1.0, zz))))

    def touching(self) -> list[str]:
        """Names of world bodies the rover is in contact with (the floor excluded)."""
        names = set()
        for c in self.data.contact[: self.data.ncon]:
            g1, g2 = c.geom1, c.geom2
            b1, b2 = self.model.geom_bodyid[g1], self.model.geom_bodyid[g2]
            if b1 in self._rover_bodies and g2 in self._obstacle_geoms:
                names.add(self._body_name(g2))
            elif b2 in self._rover_bodies and g1 in self._obstacle_geoms:
                names.add(self._body_name(g1))
        return sorted(names)

    def _body_name(self, geom: int) -> str:
        geom_name = self.model.geom(geom).name
        if geom_name.startswith("wall_"):
            return "wall"
        return self.model.body(self.model.geom_bodyid[geom]).name or "world"

    def object_position(self, name: str) -> tuple[float, float]:
        p = self.data.body(name).xpos
        return float(p[0]), float(p[1])

    # ---- commands ----------------------------------------------------------------------

    def set_cmd_vel(self, linear: float, angular: float) -> None:
        """Like publishing /cmd_vel: m/s and rad/s, clipped to the limits. Expires after 0.5 s."""
        if not (math.isfinite(linear) and math.isfinite(angular)):
            raise ValueError("cmd_vel must be finite")
        self.cmd = (
            float(np.clip(linear, -MAX_LINEAR, MAX_LINEAR)),
            float(np.clip(angular, -MAX_ANGULAR, MAX_ANGULAR)),
        )
        self._cmd_time = self.time

    def stop(self) -> None:
        if self.motion and self.motion.outcome == "running":
            self.motion.outcome = "stopped"
        self.set_cmd_vel(0.0, 0.0)

    def start_move(self, distance: float, speed: float = 0.2) -> Motion:
        speed = min(abs(speed), MAX_LINEAR)
        return self._start("move", distance, speed, abs(distance) / max(speed, 0.05) * 2 + 3)

    def start_turn(self, angle: float, speed: float = math.radians(45)) -> Motion:
        """angle in radians, positive = left (CCW)."""
        speed = min(abs(speed), MAX_ANGULAR)
        return self._start("turn", angle, speed, abs(angle) / max(speed, 0.1) * 2 + 3)

    def _start(self, kind: str, target: float, speed: float, timeout: float) -> Motion:
        if not math.isfinite(target):
            raise ValueError("target must be finite")
        if self.motion and self.motion.outcome == "running":
            self.motion.outcome = "stopped"
        self.motion = Motion(kind, target, speed, self.odom(), timeout)
        self._last_heading = self._odom[2]
        self._progress.clear()
        return self.motion

    def move(self, distance: float, speed: float = 0.2) -> str:
        """Blocking (in sim time): drive straight by `distance` m, return the outcome."""
        return self._run(self.start_move(distance, speed))

    def turn(self, angle: float, speed: float = math.radians(45)) -> str:
        """Blocking (in sim time): turn in place by `angle` rad (positive = left)."""
        return self._run(self.start_turn(angle, speed))

    def _run(self, motion: Motion) -> str:
        while motion.outcome == "running":
            self.step(CONTROL_DT)
        self.step(0.3)  # come to rest
        return motion.outcome

    # ---- simulation --------------------------------------------------------------------

    def step(self, seconds: float) -> None:
        end = self.time + seconds
        while self.time < end - 1e-9:
            if self.time >= self._next_control - 1e-9:
                self._control()
                self._next_control = self.time + CONTROL_DT
            mujoco.mj_step(self.model, self.data)
            self._integrate_odom(self.model.opt.timestep)

    def _integrate_odom(self, dt: float) -> None:
        v, w = self.velocity()
        h = self._odom[2] + w * dt / 2
        self._odom[0] += v * math.cos(h) * dt
        self._odom[1] += v * math.sin(h) * dt
        self._odom[2] = wrap(self._odom[2] + w * dt)

    def _control(self) -> None:
        if self.motion and self.motion.outcome == "running":
            self._motion_control(self.motion)
        v, w = self.cmd if self.time - self._cmd_time <= CMD_TIMEOUT else (0.0, 0.0)
        rv, rw = self._ramped
        rv += float(np.clip(v - rv, -LINEAR_ACCEL * CONTROL_DT, LINEAR_ACCEL * CONTROL_DT))
        rw += float(np.clip(w - rw, -ANGULAR_ACCEL * CONTROL_DT, ANGULAR_ACCEL * CONTROL_DT))
        self._ramped = (rv, rw)
        half = rw * ANGULAR_MULTIPLIER * WHEEL_SEPARATION / 2
        left = np.clip((rv - half) / WHEEL_RADIUS, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        right = np.clip((rv + half) / WHEEL_RADIUS, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        for name in WHEELS:
            self.data.ctrl[self._act[name]] = left if name.endswith("L") else right
        pose = self.pose()
        if math.dist(pose[:2], self.trail[-1]) > 0.03:
            self.trail.append(pose[:2])

    def _motion_control(self, m: Motion) -> None:
        m.elapsed += CONTROL_DT
        m.bumped.update(self.touching())
        odom = self.odom()
        m.turned += wrap(odom[2] - self._last_heading)
        self._last_heading = odom[2]
        done = m.progress(odom)
        remaining = m.target - done
        tolerance = 0.01 if m.kind == "move" else math.radians(1.0)
        if abs(remaining) < tolerance:
            return self._finish(m, "done")
        if self.tilt() > MAX_TILT:
            return self._finish(m, "tilted")
        if m.elapsed > m.timeout:
            return self._finish(m, "timeout")
        self._progress.append((self.time, done))
        while self._progress and self.time - self._progress[0][0] > STALL_WINDOW:
            first = self._progress.popleft()
            moved = abs(done - first[1])
            if moved < (0.01 if m.kind == "move" else math.radians(2)):
                return self._finish(m, "stalled")
        gain, floor = (1.5, 0.04) if m.kind == "move" else (2.5, math.radians(8))
        speed = math.copysign(min(m.speed, gain * abs(remaining) + floor), remaining)
        if m.kind == "move":
            heading_error = wrap(m.start[2] - odom[2])
            self.set_cmd_vel(speed, 2.0 * heading_error)
        else:
            self.set_cmd_vel(0.0, speed)

    def _finish(self, m: Motion, outcome: str) -> None:
        m.outcome = outcome
        self.set_cmd_vel(0.0, 0.0)

    # ---- rendering ---------------------------------------------------------------------

    def _renderer(self, width: int, height: int) -> mujoco.Renderer:
        key = (width, height)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, height, width)
        return self._renderers[key]

    def render(
        self, camera: str = "leo", width: int | None = None, height: int | None = None
    ) -> np.ndarray:
        """RGB uint8 image from `leo` (the rover's camera), `oak`, `chase` or `map`."""
        if camera == "map":
            return self.render_map(width or 640)
        native = CAMERAS.get(camera, {"size": (640, 480)})["size"]
        width, height = width or native[0], height or native[1]
        self._set_fovy(camera, width, height)
        options = mujoco.MjvOption()
        if camera in CAMERAS:  # the lenses sit inside the shell mesh: hide the rover's visuals
            options.geomgroup[ROVER_VISUAL_GROUP] = 0
        renderer = self._renderer(width, height)
        renderer.update_scene(self.data, camera=camera, scene_option=options)
        return renderer.render().copy()

    def _set_fovy(self, camera: str, width: int, height: int) -> None:
        if camera in CAMERAS:  # keep the real horizontal FOV at any aspect ratio
            hfov = CAMERAS[camera]["hfov"]
            fovy = 2 * math.atan(math.tan(hfov / 2) * height / width)
            self.model.cam_fovy[self.model.camera(camera).id] = math.degrees(fovy)

    def map_scale(self, width: int) -> tuple[int, float]:
        """Map image height and pixels per metre for a map `width` px wide."""
        span_x, span_y = self.world.width + 0.4, self.world.depth + 0.4
        height = int(round(width * span_y / span_x))
        return height, width / span_x

    def render_map(self, width: int = 640, goal: tuple[float, float] | None = None) -> np.ndarray:
        """Top-down orthographic view of the room with the rover's trail and heading."""
        height, ppm = self.map_scale(width)
        cam = self.model.camera("map").id
        self.model.cam_fovy[cam] = height / ppm
        renderer = self._renderer(width, height)
        renderer.update_scene(self.data, camera="map")
        image = renderer.render().copy()

        def px(x: float, y: float) -> tuple[int, int]:
            return int(width / 2 + x * ppm), int(height / 2 - y * ppm)

        for a, b in zip(self.trail, self.trail[1:], strict=False):
            _line(image, px(*a), px(*b), (255, 140, 0), 2)
        x, y, yaw = self.pose()
        _line(
            image,
            px(x, y),
            px(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)),
            (230, 0, 120),
            3,
        )
        if goal is not None:
            gx, gy = px(*goal)
            _line(image, (gx - 8, gy - 8), (gx + 8, gy + 8), (0, 160, 255), 3)
            _line(image, (gx - 8, gy + 8), (gx + 8, gy - 8), (0, 160, 255), 3)
        return image

    def close(self) -> None:
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()


def _line(
    image: np.ndarray,
    a: tuple[int, int],
    b: tuple[int, int],
    rgb: tuple[int, int, int],
    width: int = 1,
) -> None:
    """Draw a thick line into an HxWx3 image, clipped to its borders."""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1))
    xs = np.linspace(a[0], b[0], n + 1).round().astype(int)
    ys = np.linspace(a[1], b[1], n + 1).round().astype(int)
    r = width // 2
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            x, y = xs + dx, ys + dy
            ok = (x >= 0) & (x < image.shape[1]) & (y >= 0) & (y < image.shape[0])
            image[y[ok], x[ok]] = rgb

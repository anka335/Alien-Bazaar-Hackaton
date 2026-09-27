"""RoverSim: the Leo Rover in MuJoCo, driven like the real one through `cmd_vel`.

The firmware part is emulated in Python at `nav.control_hz`: a twist (v, w) comes in, the turn rate
is scaled by the angular velocity multiplier (skid-steer slip), the result is ramped, split
into left / right wheel speeds and sent to the wheels' velocity servos. No twist for
`cmd_timeout_s` → the wheels stop. Odometry is the rover's `merged_odom`: speed from the wheel
encoders, yaw rate from the IMU's gyro (with noise), integrated. There is no bump sensor: a
rover pushing a wall spins its wheels, and the odometry keeps counting.

Ground truth (`true_pose`, `sock_pose`, `contacts`) is for scoring and the debug views only; the
navigation code sees the camera and the odometry.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import mujoco
import numpy as np

from sorter.nav.config import NavConfig
from sorter.nav.model import BASE_Z, build
from sorter.nav.scenario import WorldSpec

WHEELS = ("FL", "RL", "FR", "RR")


@dataclass
class Odometry:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0  # m/s
    w: float = 0.0  # rad/s
    distance: float = 0.0  # |path| travelled, m


class RoverSim:
    def __init__(self, spec: WorldSpec, cfg: NavConfig):
        self.spec, self.cfg = spec, cfg
        xml, assets = build(spec, cfg)
        self.model = mujoco.MjModel.from_xml_string(xml, assets)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self._wheel_qvel = [m.jnt_dofadr[m.joint(f"wheel_{w}").id] for w in WHEELS]
        self._act = [m.actuator(w).id for w in WHEELS]
        self._base = m.body("base_link").id
        self._gyro = m.sensor_adr[m.sensor("gyro").id]
        self._rover_geoms = {g for g in range(m.ngeom) if _root(m, m.geom_bodyid[g]) == self._base}
        self._floor = m.geom("floor").id
        self._sock_bodies = [m.body(f"sock{i}").id for i in range(len(spec.socks))]
        self._sock_geoms = {m.geom(f"sock{i}").id for i in range(len(spec.socks))}
        # flat cloth on the floor (a shirt, a towel, ~1 cm): the rover may drive over it
        self._cloth_geoms = {
            m.geom(f"clutter{i}").id
            for i, it in enumerate(spec.distractors)
            if it.kind in ("shirt", "towel")
        }
        self.steps_per_tick = max(1, round(1 / (cfg.control_hz * cfg.timestep_s)))
        self.dt = self.steps_per_tick * cfg.timestep_s
        self.rng = np.random.default_rng([spec.seed, 3])
        self.odom = Odometry()
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9  # sim time of the last twist
        self.ref = [0.0, 0.0]  # the ramped twist the wheels follow
        # ticks with the rover touching a wall, furniture or a rigid thing (ball, shoe, toy) (truth)
        self.collisions = 0
        self.sock_touches = 0
        self.cloth_touches = 0  # driving over a flat shirt or towel: allowed, only counted
        self.settle(1.0)  # loose things come to rest before anything is measured
        self.sock_start = np.array([self.sock_pose(i)[:2] for i in range(len(spec.socks))])

    # --- driving ---

    @property
    def t(self) -> float:
        return float(self.data.time)

    def set_cmd(self, v: float, w: float) -> None:
        """A `cmd_vel` twist: forward m/s, turn rad/s (left positive). Clamped to the rover's
        limits; it holds for `cmd_timeout_s` unless repeated."""
        leo = self.cfg.leo
        self.cmd = (
            float(np.clip(v, -leo.max_linear_mps, leo.max_linear_mps)),
            float(np.clip(w, -leo.max_angular_rps, leo.max_angular_rps)),
        )
        self.cmd_t = self.t

    def tick(self) -> None:
        """One control period: the firmware loop, then the physics."""
        leo = self.cfg.leo
        v_cmd, w_cmd = self.cmd if self.t - self.cmd_t <= leo.cmd_timeout_s else (0.0, 0.0)
        dv, dw = leo.accel_mps2 * self.dt, leo.angular_accel_rps2 * self.dt
        self.ref[0] += float(np.clip(v_cmd - self.ref[0], -dv, dv))
        self.ref[1] += float(np.clip(w_cmd - self.ref[1], -dw, dw))
        v, w = self.ref
        half = leo.wheel_separation_m / 2 * leo.angular_velocity_multiplier
        left, right = (v - w * half) / leo.wheel_radius_m, (v + w * half) / leo.wheel_radius_m
        peak = max(abs(left), abs(right), 1e-9)
        if peak > leo.max_wheel_rps:  # keep the ratio, i.e. the turn radius
            left, right = left * leo.max_wheel_rps / peak, right * leo.max_wheel_rps / peak
        for a, s in zip(self._act, (left, left, right, right), strict=True):
            self.data.ctrl[a] = s
        for _ in range(self.steps_per_tick):
            mujoco.mj_step(self.model, self.data)
        self._odometry()
        self._contacts()

    def settle(self, seconds: float) -> None:
        for _ in range(int(seconds / self.dt)):
            self.tick()

    def _odometry(self) -> None:
        leo = self.cfg.leo
        q = self.data.qvel[self._wheel_qvel]
        v = leo.wheel_radius_m * float(q.mean())
        w = float(self.data.sensordata[self._gyro + 2]) + self.rng.normal(0, leo.gyro_noise_rps)
        o = self.odom
        o.yaw += w * self.dt
        o.x += v * math.cos(o.yaw) * self.dt
        o.y += v * math.sin(o.yaw) * self.dt
        o.v, o.w = v, w
        o.distance += abs(v) * self.dt

    def _contacts(self) -> None:
        bump = sock = cloth = False
        for c in self.data.contact[: self.data.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self._rover_geoms or g2 in self._rover_geoms:
                other = g2 if g1 in self._rover_geoms else g1
                if other in self._sock_geoms:
                    sock = True
                elif other in self._cloth_geoms:
                    cloth = True
                elif other != self._floor and other not in self._rover_geoms:
                    bump = True
        self.collisions += bump
        self.sock_touches += sock
        self.cloth_touches += cloth

    # --- ground truth (scoring, debug views) ---

    def true_pose(self) -> tuple[float, float, float]:
        p = self.data.xpos[self._base]
        R = self.data.xmat[self._base].reshape(3, 3)
        return float(p[0]), float(p[1]), math.atan2(R[1, 0], R[0, 0])

    def sock_pose(self, i: int = 0) -> np.ndarray:
        return self.data.xpos[self._sock_bodies[i]].copy()

    def sock_in_rover(self, i: int = 0) -> tuple[float, float]:
        """The sock's center in the rover's frame (x forward from the rover's center)."""
        x, y, yaw = self.true_pose()
        s = self.sock_pose(i)
        dx, dy = s[0] - x, s[1] - y
        c, sn = math.cos(yaw), math.sin(yaw)
        return c * dx + sn * dy, -sn * dx + c * dy

    def moving(self) -> bool:
        return abs(self.odom.v) > 0.005 or abs(self.odom.w) > 0.01

    # --- save / restore (the CLI keeps an episode across processes) ---

    def get_state(self) -> dict:
        m, d = self.model, self.data
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        st = np.empty(mujoco.mj_stateSize(m, spec))
        mujoco.mj_getState(m, d, st, spec)
        return {
            "physics": st.tolist(),
            "odom": vars(self.odom).copy(),
            "cmd": list(self.cmd),
            "cmd_t": self.cmd_t,
            "ref": list(self.ref),
            "collisions": self.collisions,
            "cloth_touches": self.cloth_touches,
            "sock_touches": self.sock_touches,
            "sock_start": self.sock_start.tolist(),
            "rng": json.loads(json.dumps(self.rng.bit_generator.state)),
        }

    def set_state(self, s: dict) -> None:
        m, d = self.model, self.data
        mujoco.mj_setState(m, d, np.array(s["physics"]), mujoco.mjtState.mjSTATE_INTEGRATION)
        mujoco.mj_forward(m, d)
        self.odom = Odometry(**s["odom"])
        self.cmd, self.cmd_t, self.ref = tuple(s["cmd"]), s["cmd_t"], list(s["ref"])
        self.collisions, self.sock_touches = s["collisions"], s["sock_touches"]
        self.cloth_touches = s.get("cloth_touches", 0)
        self.sock_start = np.array(s["sock_start"])
        self.rng.bit_generator.state = s["rng"]


def _root(m: mujoco.MjModel, body: int) -> int:
    """The body right under the world that `body` hangs from."""
    while m.body_parentid[body] != 0:
        body = m.body_parentid[body]
    return body


def floor_height() -> float:
    """base_link above the floor at rest (the rover frame's origin is on the floor below it)."""
    return BASE_Z

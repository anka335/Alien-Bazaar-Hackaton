"""High level driver for the reBot Arm B601-RS.

* :class:`Arm` runs a 50 Hz control thread that streams the current setpoint to
  the motors, executes smooth joint trajectories (constant speed between raised-cosine ramps), drives the
  gripper and enforces safety checks (soft limits, tracking error, temperature,
  lost feedback).
* :class:`HardwareBackend` talks to the motors through the ``motorbridge``
  Python SDK over SocketCAN.  :class:`SimBackend` is a fake arm used for dry
  runs and tests (``REBOT_DRY_RUN=1`` or ``Arm().connect(simulate=True)``).

All public numbers are in metres (Cartesian, base frame) and degrees (joints).

.. warning::
   The hardware backend follows Seeed's reference code but has **not** been
   run on a real arm by the author.  Start with ``simulate=True``, then
   ``connect(enable=False)`` (read only), and keep the power switch in reach.
"""

from __future__ import annotations

import contextlib
import logging
import math
import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import config as C
from . import kinematics as K

log = logging.getLogger("rebot_b601")


class ArmError(RuntimeError):
    """Raised for refused or aborted motions and hardware problems."""


# --------------------------------------------------------------------------
# Trajectories
# --------------------------------------------------------------------------


def _path_length(waypoints: np.ndarray) -> float:
    """Seconds the path takes with every segment at the peak speed of its slowest joint at
    speed_scale 1 (the path parameter of :class:`Trajectory`)."""
    seg = np.abs(np.diff(waypoints, axis=0)) / np.radians(C.JOINT_SPEED_DPS)
    return float(np.sum(np.max(seg, axis=1))) if len(seg) else 0.0


def path_timing(waypoints: np.ndarray, speed_scale: float) -> tuple[float, float]:
    """``(duration, ramp)`` [s] of ``waypoints`` (M,6 rad) at ``speed_scale``.

    The path runs at the limiting joint's peak speed (``JOINT_SPEED_DPS`` x scale), reached and
    left in a raised-cosine ramp (smooth acceleration, peak ``JOINT_ACCEL`` x ``JOINT_SPEED_DPS``
    per second). A path too short for full speed is two ramps with a lower peak.
    """
    length, a = _path_length(waypoints), C.JOINT_ACCEL
    ramp = math.pi * speed_scale / (2 * a)
    if length >= speed_scale * ramp:
        duration = length / speed_scale + ramp
    else:  # peak rate p: two ramps of pi p / 2a each cover p x ramp = length
        peak = math.sqrt(2 * a * length / math.pi)
        ramp = math.pi * peak / (2 * a)
        duration = 2 * ramp
    if duration < C.MIN_MOVE_TIME_S:  # stretch the whole profile
        ramp *= C.MIN_MOVE_TIME_S / max(duration, 1e-9)
        duration = C.MIN_MOVE_TIME_S
    return duration, ramp


def path_duration(waypoints: np.ndarray, speed_scale: float) -> float:
    """Time needed to run ``waypoints`` (M,6 rad) at ``speed_scale`` (see :func:`path_timing`)."""
    return path_timing(waypoints, speed_scale)[0]


def _progress(t: float, duration: float, ramp: float) -> float:
    """0..1 along the path at ``t``: raised-cosine ramp up, constant speed, ramp down."""
    t = min(max(t, 0.0), duration)
    ramp = min(ramp, duration / 2)
    peak = 1.0 / (duration - ramp)  # two ramps at half the peak rate + the cruise = 1

    def up(x: float) -> float:
        return peak / 2 * (x - ramp / math.pi * math.sin(math.pi * x / ramp)) if ramp > 0 else 0.0

    if t < ramp:
        return up(t)
    if t <= duration - ramp:
        return peak * ramp / 2 + peak * (t - ramp)
    return 1.0 - up(duration - t)


@dataclass
class Trajectory:
    waypoints: np.ndarray            # (M,6) rad
    duration: float
    t0: float = 0.0
    ramp: float | None = None        # s; None: all ramp (no constant-speed part)
    _u: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        vmax = np.radians(C.JOINT_SPEED_DPS)
        d = np.max(np.abs(np.diff(self.waypoints, axis=0)) / vmax, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(d)])
        self._u = cum / cum[-1] if cum[-1] > 0 else np.linspace(0.0, 1.0, len(cum))
        if self.ramp is None:
            self.ramp = self.duration / 2

    def sample(self, t: float) -> np.ndarray:
        s = _progress(t, self.duration, self.ramp)
        return np.array([np.interp(s, self._u, self.waypoints[:, j]) for j in range(6)])


# --------------------------------------------------------------------------
# Backends
# --------------------------------------------------------------------------


@dataclass
class Measurement:
    q: np.ndarray            # (6,) rad
    dq: np.ndarray           # (6,) rad/s
    tau: np.ndarray          # (6,) Nm
    temps: np.ndarray        # (7,) deg C (MOSFET), nan if unknown
    grip_pos: float          # rad (motor angle)
    grip_vel: float          # rad/s


class SimBackend:
    """Fake arm: joints follow the setpoint with a speed limit and small lag."""

    simulated = True

    def __init__(self, q0: np.ndarray | None = None):
        self.q = np.zeros(6) if q0 is None else np.array(q0, dtype=float)
        self.dq = np.zeros(6)
        self.grip = 0.0
        self.grip_vel = 0.0
        self.enabled = False
        self._q_cmd = self.q.copy()
        self._grip_tau = 0.0
        self.blocked: set[int] = set()       # joints that refuse to move (test hook)
        self.temp = 35.0
        self._t = time.monotonic()

    def connect(self, enable: bool) -> Measurement:
        self._q_cmd = self.q.copy()
        return self.read()

    def enable(self, q_hold: np.ndarray) -> None:
        self._q_cmd = np.array(q_hold, dtype=float)
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False

    def close(self) -> None:
        self.enabled = False

    def send_arm(self, q_cmd: np.ndarray) -> None:
        self._q_cmd = np.array(q_cmd, dtype=float)

    def send_gripper_torque(self, tau: float) -> None:
        self._grip_tau = float(tau)

    def read(self) -> Measurement:
        now = time.monotonic()
        dt = min(now - self._t, 0.1)
        self._t = now
        if self.enabled:
            vmax = np.radians(C.JOINT_SPEED_DPS) * 1.2
            step = np.clip(self._q_cmd - self.q, -vmax * dt, vmax * dt)
            for j in self.blocked:
                step[j] = 0.0
            self.dq = step / dt if dt > 0 else np.zeros(6)
            self.q = self.q + step
            # crude gripper: torque drives it, clipped to its travel
            new = float(np.clip(self.grip + self._grip_tau * 4.0 * dt, 0.0, math.radians(C.GRIPPER_OPEN_DEG)))
            self.grip_vel = (new - self.grip) / dt if dt > 0 else 0.0
            self.grip = new
        else:
            self.dq = np.zeros(6)
        return Measurement(self.q.copy(), self.dq.copy(), np.zeros(6), np.full(7, self.temp), self.grip, self.grip_vel)


class _Timing:
    """Time spent per part of a control tick since the last report: calls, total, worst."""

    def __init__(self):
        self._t: dict[str, list[float]] = {}

    @contextlib.contextmanager
    def __call__(self, part: str):
        t0 = time.perf_counter()
        try:
            yield
        finally:
            dt = time.perf_counter() - t0
            n_total_worst = self._t.setdefault(part, [0, 0.0, 0.0])
            n_total_worst[0] += 1
            n_total_worst[1] += dt
            n_total_worst[2] = max(n_total_worst[2], dt)

    def report(self) -> str:
        """The parts, slowest first, and a reset."""
        parts = sorted(self._t.items(), key=lambda kv: -kv[1][1])
        self._t = {}
        return ", ".join(
            f"{p} {n}x avg {tot / n * 1000:.1f} ms max {worst * 1000:.0f} ms" for p, (n, tot, worst) in parts
        )


class HardwareBackend:
    """RobStride motors 1-7 over SocketCAN through the ``motorbridge`` SDK."""

    simulated = False
    _LOC_REF = 0x7016      # position reference
    _LIMIT_SPD = 0x7017
    _LOC_KP = 0x701E
    _SPD_KP = 0x701F
    _SPD_KI = 0x7020
    _MECH_POS = 0x7019
    _VEL_MAX = 0x7024      # the Position mode's speed limit (motorbridge's POS_VEL writes it)
    # A USB-CAN adapter on macOS takes ~10-25 ms per frame, and motorbridge's send_pos_vel is
    # 2-3 parameter writes per motor: a 50 Hz tick then takes ~200 ms and the arm jerks. So
    # the mode and speed limit are set once at enable, and a tick writes only the position
    # reference of the motors whose setpoint changed, plus one motor in turn (its reply keeps
    # its feedback fresh). REBOT_SEND=pos_vel restores send_pos_vel for every motor, every tick.
    _SEND_ALL = os.environ.get("REBOT_SEND", "") == "pos_vel"
    _RESEND_S = 0.2        # the gripper torque is resent at least this often

    def __init__(self, channel: str = C.CAN_CHANNEL):
        self.channel = channel
        self.ctrl = None
        self.arm_motors: list = []
        self.gripper = None
        self._enabled = False
        self.timing = _Timing()  # where the control loop's time goes (see Arm._loop)
        self._sent_q: np.ndarray | None = None  # the position references last written
        self._turn = 0  # the motor whose reference is written this tick regardless
        self._sent_tau: tuple[float, float] | None = None  # gripper torque, when

    # -- helpers ---------------------------------------------------------
    def _all(self):
        return self.arm_motors + ([self.gripper] if self.gripper is not None else [])

    def _poll(self) -> None:
        with self.timing("request"):
            for m in self._all():
                try:
                    m.request_feedback()
                except Exception:
                    pass
        with self.timing("poll"):
            try:
                self.ctrl.poll_feedback_once()
            except Exception:
                pass

    def connect(self, enable: bool) -> Measurement:
        try:
            from motorbridge import Controller
        except ImportError as e:  # pragma: no cover - depends on the environment
            raise ArmError("the 'motorbridge' package is not installed in this Python environment") from e
        try:
            self.ctrl = Controller(self.channel)
        except Exception as e:
            raise ArmError(
                f"cannot open CAN channel {self.channel}: {e}. Is can0 UP (bitrate 1000000)? "
                "Is motorbridge-gateway still running and holding the bus?"
            ) from e
        for mid, model in zip(C.ARM_MOTOR_IDS, C.ARM_MOTOR_MODELS):
            self.arm_motors.append(self.ctrl.add_robstride_motor(mid, C.FEEDBACK_ID, model))
        if C.GRIPPER_ENABLED:
            self.gripper = self.ctrl.add_robstride_motor(C.GRIPPER_MOTOR_ID, C.FEEDBACK_ID, C.GRIPPER_MOTOR_MODEL)
        meas = None
        for _ in range(20):
            self._poll()
            meas = self._try_read()
            if meas is not None:
                break
            time.sleep(0.05)
        if meas is None:
            self.close()
            raise ArmError("no feedback from the motors (are they powered and IDs 1-7 present? try the scan)")
        return meas

    def _state_of(self, motor):
        st = motor.get_state()
        if st is not None:
            return st.pos, st.vel, st.torq, st.t_mos
        # no feedback frame from this motor: a blocking parameter read (up to its timeout)
        i = self._all().index(motor) + 1
        with self.timing(f"param read motor {i}"):
            try:
                return float(motor.robstride_get_param_f32(self._MECH_POS)), 0.0, 0.0, float("nan")
            except Exception:
                return None

    def _try_read(self) -> Measurement | None:
        q, dq, tau, temps = [], [], [], []
        for m in self.arm_motors:
            s = self._state_of(m)
            if s is None:
                return None
            q.append(s[0]); dq.append(s[1]); tau.append(s[2]); temps.append(s[3])
        gp = gv = 0.0
        gt = float("nan")
        if self.gripper is not None:
            s = self._state_of(self.gripper)
            if s is not None:
                gp, gv, _, gt = s
        return Measurement(np.array(q), np.array(dq), np.array(tau), np.array(temps + [gt]), gp, gv)

    def read(self) -> Measurement:
        self._poll()
        meas = self._try_read()
        if meas is None:
            raise ArmError("motor feedback missing")
        return meas

    def enable(self, q_hold: np.ndarray) -> None:
        """Switch to POS_VEL, pre-load the position reference with the CURRENT pose, then enable."""
        from motorbridge import Mode

        for m in self.arm_motors:
            m.disable()   # make sure torque is off while parameters/modes change
        time.sleep(0.05)
        for i, m in enumerate(self.arm_motors):
            p = C.PV_PARAMS[i]
            try:
                m.robstride_write_param_f32(self._LIMIT_SPD, C.MOTOR_VLIM_RAD_S); time.sleep(0.01)
                m.robstride_write_param_f32(self._SPD_KP, p["vel_kp"]); time.sleep(0.01)
                m.robstride_write_param_f32(self._SPD_KI, p["vel_ki"]); time.sleep(0.01)
                m.robstride_write_param_f32(self._LOC_KP, p["pos_kp"]); time.sleep(0.01)
                m.ensure_mode(Mode.POS_VEL, 1000)
                m.robstride_write_param_f32(self._VEL_MAX, C.MOTOR_VLIM_RAD_S); time.sleep(0.01)
                # without this the motor could slew towards a stale (or zero) reference when enabled
                m.robstride_write_param_f32(self._LOC_REF, float(q_hold[i]))
            except Exception as e:
                raise ArmError(f"could not configure joint{i + 1} (motor {C.ARM_MOTOR_IDS[i]}): {e}") from e
            time.sleep(0.02)
        if self.gripper is not None:
            try:
                self.gripper.ensure_mode(Mode.MIT, 1000)
            except Exception as e:
                raise ArmError(f"could not configure the gripper motor: {e}") from e
        for m in self._all():
            m.enable()
            time.sleep(0.02)
        self._enabled = True
        self._sent_q, self._sent_tau = None, None
        self.send_arm(q_hold)                 # immediately hold the current pose
        self.send_gripper_torque(0.0)

    def disable(self) -> None:
        for m in self._all():
            try:
                m.disable()
            except Exception:
                pass
        self._enabled = False

    def send_arm(self, q_cmd: np.ndarray) -> None:
        q_cmd = np.asarray(q_cmd, dtype=float)
        with self.timing("send arm"):
            if self._SEND_ALL:
                for i, m in enumerate(self.arm_motors):
                    m.send_pos_vel(float(q_cmd[i]), float(C.MOTOR_VLIM_RAD_S))
                return
            n = len(self.arm_motors)
            if self._sent_q is None:
                self._sent_q = np.full(n, np.nan)
            changed = ~(np.abs(q_cmd - self._sent_q) < 1e-5)  # NaN (never sent) counts as changed
            changed[self._turn % n] = True
            self._turn += 1
            for i in np.flatnonzero(changed):
                self.arm_motors[i].robstride_write_param_f32(self._LOC_REF, float(q_cmd[i]))
                self._sent_q[i] = q_cmd[i]

    def send_gripper_torque(self, tau: float) -> None:
        if self.gripper is not None:
            # like Seeed's follower: no position term, light damping, feed-forward torque
            now = time.monotonic()
            last = self._sent_tau
            if not self._SEND_ALL and last is not None:
                if abs(tau - last[0]) < 1e-3 and now - last[1] < self._RESEND_S:
                    return
            with self.timing("send gripper"):
                self.gripper.send_mit(0.0, 0.0, 0.0, 1.5, float(tau))
            self._sent_tau = (float(tau), now)

    def close(self) -> None:
        if self.ctrl is None:
            return
        try:
            if self._enabled:
                self.disable()
            for m in self._all():
                try:
                    m.close()
                except Exception:
                    pass
            self.ctrl.shutdown()
            time.sleep(0.05)
            self.ctrl.close()
        except Exception:
            pass
        finally:
            self.ctrl = None
            self.arm_motors = []
            self.gripper = None


# --------------------------------------------------------------------------
# Planning (shared by Arm and by simulators that execute paths their own way)
# --------------------------------------------------------------------------


def check_xyz(xyz) -> np.ndarray:
    p = np.asarray(xyz, dtype=float).reshape(3)
    if not np.all(np.isfinite(p)):
        raise ArmError("target contains NaN/inf")
    for name, v, (lo, hi) in zip("xyz", p, (C.WORKSPACE_X, C.WORKSPACE_Y, C.WORKSPACE_Z)):
        if not lo <= v <= hi:
            raise ArmError(f"{name}={v:.3f} m is outside the allowed workspace box [{lo:.2f}, {hi:.2f}] m")
    return p


def check_limits(q: np.ndarray, what: str = "target") -> None:
    tol = math.radians(0.5)
    lo, hi = C.JOINT_LIMITS_RAD[:, 0], C.JOINT_LIMITS_RAD[:, 1]
    for j in range(6):
        if q[j] < lo[j] - tol or q[j] > hi[j] + tol:
            raise ArmError(
                f"{what}: joint{j + 1} = {math.degrees(q[j]):.1f} deg is outside its limit "
                f"[{C.JOINT_LIMITS_DEG[j, 0]:.0f}, {C.JOINT_LIMITS_DEG[j, 1]:.0f}] deg"
            )


def check_path(wps: np.ndarray, n_samples: int = 60, z_min: float | None = None) -> None:
    """Sample the planned path and reject it if it dips below the table / hits the base."""
    z_min = C.Z_MIN if z_min is None else z_min
    traj = Trajectory(wps, 1.0)
    for s in np.linspace(0.0, 1.0, n_samples):
        q = np.array([np.interp(s, traj._u, wps[:, j]) for j in range(6)])
        ok, why = K.pose_is_safe(q, z_min=z_min)
        if not ok:
            # the start pose may legitimately be low (e.g. resting); only reject if it gets worse
            if s == 0.0:
                continue
            raise ArmError(f"path rejected: {why}")


def plan_path(q0, target, approach=None, linear: bool = False, z_min: float | None = None):
    """IK + checks for a move of the TCP from joints ``q0`` to ``target`` [m].

    Returns ``(q_goal, waypoints (M,6) rad, IKResult)``; raises :class:`ArmError` if the
    target or the path is not feasible.  ``linear`` = straight line in Cartesian space.
    ``z_min`` overrides the table clearance (``REBOT_Z_MIN``).
    """
    z_min = C.Z_MIN if z_min is None else z_min
    q0 = np.asarray(q0, dtype=float)
    target = np.asarray(target, dtype=float).reshape(3)
    lim = C.JOINT_LIMITS_RAD
    res = K.solve_ik(target, approach, q0, lim, z_min=z_min)
    if not res.success:
        raise ArmError(f"target {np.round(target, 3).tolist()} is not reachable: {res.message}")
    q_goal = res.q
    check_limits(q_goal)
    if not linear:
        wps = np.vstack([q0, q_goal])
        check_path(wps, z_min=z_min)
        return q_goal, wps, res
    # straight line in Cartesian space, IK continued waypoint by waypoint
    p0, _ = K.fk(q0)
    dist = float(np.linalg.norm(target - p0))
    n = max(2, int(math.ceil(dist / 0.01)) + 1)
    wps = [q0]
    q = q0
    # blend the approach direction from the current one to the requested one along the path,
    # otherwise the very first waypoint would demand an instant re-orientation of the wrist
    a_goal = K.parse_approach(approach)
    a_start = K.approach_vector(q0)
    for k in range(1, n):
        s = k / (n - 1)
        pk = p0 + (target - p0) * s
        if a_goal is None:
            ak = None
        else:
            ak = (1.0 - s) * a_start + s * a_goal
            ak = a_goal if np.linalg.norm(ak) < 1e-6 else ak / np.linalg.norm(ak)
        r = K.solve_ik(pk, ak, q, lim, n_random_starts=0, z_min=z_min)
        if not r.success:
            raise ArmError(f"straight-line path fails at {np.round(pk, 3).tolist()}: {r.message}")
        if np.max(np.abs(r.q - q)) > math.radians(12.0):
            raise ArmError("straight-line path needs a large joint jump (singularity or configuration change); use linear=False")
        check_limits(r.q, "path")
        wps.append(r.q)
        q = r.q
    wps = np.array(wps)
    check_path(wps, n_samples=max(60, n), z_min=z_min)
    return q_goal, wps, res


# --------------------------------------------------------------------------
# Arm
# --------------------------------------------------------------------------


def _round(a, n=3):
    return [round(float(x), n) for x in a]


class Arm:
    """``clock``: the time source of the control loop, trajectories and timeouts (a physics
    simulator passes its simulated time and calls :meth:`tick` itself, see :meth:`connect`)."""

    def __init__(self, hz: float = C.CONTROL_HZ, max_speed_scale: float = C.MAX_SPEED_SCALE, clock=time.monotonic):
        self.hz = hz
        self.max_speed_scale = max_speed_scale
        self._clock = clock
        self.backend = None
        self._lock = threading.RLock()
        self._move_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_thread = threading.Event()
        self._done = threading.Event()
        self._traj: Trajectory | None = None
        self._hold = np.zeros(6)
        self._q_cmd = np.zeros(6)
        self._meas: Measurement | None = None
        self._enabled = False
        self._fault: str | None = None
        self._abort: str | None = None
        self._comm_fail = 0
        self._err_since = np.full(6, np.nan)
        self._prev_cmd: tuple[float, np.ndarray] | None = None  # (time, q_cmd) of the last check
        self._lag_speed = np.zeros(6)   # recent commanded speed [deg/s], decays after a stop
        self._fb_key: np.ndarray | None = None   # the last feedback, bit for bit
        self._fb_since = 0.0                     # when it last changed
        self._stale = False                      # feedback frozen: no motion until reconnect
        self._grip_target = 0.0          # rad
        self._grip_cmd_time = 0.0
        self._last_target: np.ndarray | None = None   # last requested xyz, for the viewer

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self.backend is not None

    @property
    def simulated(self) -> bool:
        return bool(self.backend and self.backend.simulated)

    def connect(self, enable: bool = True, simulate: bool | None = None, sim_start_deg=None, backend=None, own_loop: bool = True) -> dict:
        """Connect to the arm.  ``enable=False`` only reads feedback (motors stay limp).

        ``backend``: use this backend (e.g. a physics simulator) instead of the bus or
        :class:`SimBackend`.  ``own_loop=False``: no control thread; the caller runs
        :meth:`tick` at ``hz`` (in the time of ``clock``).
        """
        if self.connected:
            raise ArmError("already connected")
        simulate = C.DRY_RUN if simulate is None else simulate
        if backend is None:
            backend = SimBackend(None if sim_start_deg is None else np.radians(sim_start_deg)) if simulate else HardwareBackend()
        meas = backend.connect(enable)
        lo, hi = C.JOINT_LIMITS_RAD[:, 0], C.JOINT_LIMITS_RAD[:, 1]
        m = math.radians(C.POSE_SANITY_MARGIN_DEG)
        bad = [i + 1 for i in range(6) if meas.q[i] < lo[i] - m or meas.q[i] > hi[i] + m]
        if bad and enable:
            backend.close()
            raise ArmError(
                f"joint(s) {bad} read {np.degrees(meas.q).round(1).tolist()} deg, outside the allowed range; "
                "the zero calibration is probably missing or wrong. Refusing to enable the motors."
            )
        self.backend = backend
        self._meas = meas
        self._fault = self._abort = None
        self._comm_fail = 0
        self._err_since[:] = np.nan
        self._stale, self._fb_key = False, None
        self._traj = None
        self._hold = np.clip(meas.q, lo, hi)
        self._q_cmd = self._hold.copy()
        self._grip_target = float(np.clip(meas.grip_pos, 0.0, math.radians(C.GRIPPER_OPEN_DEG)))
        self._grip_cmd_time = 0.0
        self._enabled = False
        if enable:
            try:
                backend.enable(self._hold)
            except Exception:
                backend.close()
                self.backend = None
                raise
            self._enabled = True
        self._stop_thread.clear()
        if own_loop:
            self._thread = threading.Thread(target=self._loop, name="rebot-control", daemon=True)
            self._thread.start()
        return self.status()

    def disconnect(self, go_home: bool = True, speed_scale: float | None = None) -> dict:
        """Optionally return home, then disable the motors and close the bus.

        With ``go_home=False`` the motors are disabled where they are: the arm
        will sag / drop under gravity.
        """
        if not self.connected:
            return {"ok": True, "message": "not connected"}
        note = ""
        if self._enabled:
            if go_home and not self._fault:
                try:
                    self.home(speed_scale)
                except ArmError as e:
                    raise ArmError(f"could not return home ({e}); torque left ON. Fix the cause or call disconnect(go_home=False)") from e
            elif go_home:
                note = f"fault active ({self._fault}): homing skipped, torque disabled where the arm is; it may have dropped"
            else:
                note = "torque disabled without homing: the arm may have dropped"
        self._stop_thread.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.backend.close()
        self.backend = None
        self._enabled = False
        return {"ok": True, "message": "disconnected" + (f" ({note})" if note else "")}

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------
    def tick(self) -> None:
        """One control cycle, for callers that drive the loop themselves (``own_loop=False``)."""
        try:
            self._tick(self._clock())
        except Exception as e:      # never let the loop die silently
            log.exception("control loop error")
            self._raise_fault(f"control loop error: {e!r}")

    def _loop(self) -> None:
        period = 1.0 / self.hz
        nxt = time.monotonic()
        last = nxt
        late, worst, reported = 0, 0.0, nxt  # ticks over 3 periods apart since the last report
        while not self._stop_thread.is_set():
            now = time.monotonic()
            gap = now - last
            last = now
            if gap > 3 * period:  # the setpoint jumps ahead by the gap: the arm jerks
                late, worst = late + 1, max(worst, gap)
            if late and now - reported > 2.0:
                timing = getattr(self.backend, "timing", None)
                log.warning(
                    "control loop late %d times in %.1f s, worst gap %.0f ms (expected %.0f): "
                    "jerky motion, tracking-error faults. Tick parts: %s",
                    late, now - reported, worst * 1000, period * 1000,
                    timing.report() if timing is not None else "n/a",
                )
                late, worst, reported = 0, 0.0, now
            self.tick()
            nxt += period
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()

    def _tick(self, now: float) -> None:
        backend = self.backend
        if backend is None:
            return
        try:
            meas = backend.read()
            self._comm_fail = 0
        except ArmError:
            self._comm_fail += 1
            if self._comm_fail == int(self.hz * 0.3):
                self._raise_fault("lost motor feedback (CAN problem or motor unpowered)")
            meas = self._meas
            if meas is None:
                return
        self._check_stale(meas, now)
        with self._lock:
            self._meas = meas
            if self._traj is not None:
                s = now - self._traj.t0
                q_cmd = self._traj.sample(s)
                if s >= self._traj.duration:
                    self._hold = q_cmd.copy()
                    self._traj = None
                    self._done.set()
            else:
                q_cmd = self._hold
            self._q_cmd = q_cmd
            enabled = self._enabled
            tau_g = self._gripper_torque(meas, now)
        if enabled:
            self._safety_checks(meas, q_cmd, now)
            if self._enabled:
                backend.send_arm(q_cmd)
                backend.send_gripper_torque(tau_g)

    def _check_stale(self, meas: Measurement, now: float) -> bool:
        """True (and a fault) once the hardware's feedback has been frozen too long."""
        if self.backend is None or self.backend.simulated or not self._enabled:
            self._fb_key = None
            return False
        key = np.concatenate([meas.q, meas.dq, meas.tau])
        if self._fb_key is None or not np.array_equal(key, self._fb_key):
            self._fb_key, self._fb_since = key, now
            return False
        limit = C.STALE_FEEDBACK_MOVING_S if self._traj is not None else C.STALE_FEEDBACK_IDLE_S
        if now - self._fb_since <= limit:
            return False
        if not self._stale:
            self._stale = True
            self._raise_fault(
                f"lost motor feedback: nothing new from the motors for {now - self._fb_since:.1f} s "
                "(the CAN adapter stopped receiving); holding the last setpoint. Quit, replug the "
                "USB-CAN adapter and start again"
            )
        return True

    def _gripper_torque(self, meas: Measurement, now: float) -> float:
        if not C.GRIPPER_ENABLED:
            return 0.0
        err = self._grip_target - meas.grip_pos
        tau = C.GRIPPER_KP * err + C.GRIPPER_KD * (0.0 - meas.grip_vel)
        # full torque only for a short time after a new command; afterwards just hold
        lim = C.GRIPPER_TORQUE_LIMIT if (now - self._grip_cmd_time) < 1.5 else C.GRIPPER_HOLD_TORQUE
        return float(np.clip(tau, -lim, lim))

    def _safety_checks(self, meas: Measurement, q_cmd: np.ndarray, now: float) -> None:
        # tracking error, allowing a lag of TRACKING_LAG_S at the recent commanded speed (it
        # decays over TRACKING_LAG_S after the setpoint stops: the joint still catches up)
        err = np.degrees(np.abs(meas.q - q_cmd))
        if self._prev_cmd is not None and (dt := now - self._prev_cmd[0]) > 1e-3:
            speed = np.degrees(np.abs(q_cmd - self._prev_cmd[1])) / dt
            decay = math.exp(-dt / C.TRACKING_LAG_S) if C.TRACKING_LAG_S > 0 else 0.0
            self._lag_speed = np.maximum(speed, self._lag_speed * decay)
        self._prev_cmd = (now, q_cmd.copy())
        allowed = C.TRACKING_ERR_DEG + C.TRACKING_LAG_S * self._lag_speed
        for j in range(6):
            if err[j] > allowed[j]:
                if np.isnan(self._err_since[j]):
                    self._err_since[j] = now
                elif now - self._err_since[j] > C.TRACKING_ERR_TIME_S:
                    self._raise_fault(
                        f"joint{j + 1} is {err[j]:.1f} deg away from its commanded position "
                        f"(commanded {math.degrees(q_cmd[j]):.1f}, measured {math.degrees(meas.q[j]):.1f} deg; "
                        "blocked, collision or motor fault); motion aborted and pose held"
                    )
                    return
            else:
                self._err_since[j] = np.nan
        # temperature
        temps = meas.temps[np.isfinite(meas.temps)]
        if temps.size:
            t = float(temps.max())
            if t >= C.TEMP_DISABLE_C:
                self._raise_fault(f"motor temperature {t:.0f} C: torque disabled")
                self.emergency_disable()
            elif t >= C.TEMP_STOP_C:
                self._raise_fault(f"motor temperature {t:.0f} C: motion aborted, let the arm cool down")

    def _raise_fault(self, msg: str) -> None:
        with self._lock:
            if self._fault is None:
                self._fault = msg
                log.error("FAULT: %s", msg)
            self._abort = msg
            if self._stale:  # the measured pose is old: hold where it was told to be, no jump
                self._hold = self._q_cmd.copy()
            elif self._meas is not None:
                self._hold = np.clip(self._meas.q, C.JOINT_LIMITS_RAD[:, 0], C.JOINT_LIMITS_RAD[:, 1])
            self._traj = None
            self._done.set()

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------
    def _require(self, motion: bool = True) -> None:
        if not self.connected:
            raise ArmError("arm is not connected; call connect first")
        if motion:
            if not self._enabled:
                raise ArmError("motors are not enabled (connected read-only, or torque was disabled); reconnect with enable=True")
            if self._fault:
                raise ArmError(f"arm is in a fault state: {self._fault}. Inspect the arm, then clear_fault (or disconnect and connect again)")

    def _q_meas(self) -> np.ndarray:
        with self._lock:
            return self._meas.q.copy()

    def joints(self) -> np.ndarray:
        """Measured joint angles [rad] (6,)."""
        self._require(motion=False)
        return self._q_meas()

    def status(self) -> dict:
        if not self.connected:
            return {"connected": False}
        with self._lock:
            meas = self._meas
            q = meas.q.copy()
            hold = self._hold.copy()
            moving = self._traj is not None
        p, R = K.fk(q)
        grip = float(np.clip(meas.grip_pos / math.radians(C.GRIPPER_OPEN_DEG), 0.0, 1.0)) if C.GRIPPER_ENABLED else None
        temps = meas.temps[np.isfinite(meas.temps)]
        return {
            "connected": True,
            "simulated": self.simulated,
            "torque_enabled": self._enabled,
            "moving": moving,
            "fault": self._fault,
            "joints_deg": _round(np.degrees(q), 2),
            "tcp_xyz_m": _round(p, 4),
            "approach_axis": _round(R[:, 0], 3),
            "gripper_opening": None if grip is None else round(grip, 3),
            "gripper_motor_deg": round(math.degrees(meas.grip_pos), 1),
            "max_motor_temp_c": None if not temps.size else round(float(temps.max()), 1),
            "joint_limits_deg": C.JOINT_LIMITS_DEG.tolist(),
        }

    def snapshot(self) -> dict:
        """Everything a 3D viewer needs (works when disconnected: shows the home pose)."""
        with self._lock:
            meas = self._meas if self.connected else None
            q = np.zeros(6) if meas is None else meas.q.copy()
            q_cmd = q if meas is None else self._q_cmd.copy()
            moving = self._traj is not None
            target = None if self._last_target is None else self._last_target.tolist()
        frames = K.joint_frames(q)
        pts = [[0.0, 0.0, 0.0]] + [f[:3, 3].tolist() for f in frames[:-1]] + [frames[-1][:3, 3].tolist()]
        Tt = frames[-1]
        grip = 0.0
        finger_m = 0.0
        if meas is not None and C.GRIPPER_ENABLED:
            grip = float(np.clip(meas.grip_pos / math.radians(C.GRIPPER_OPEN_DEG), 0.0, 1.0))
            # Seeed/Studio map the motor range 0..270 deg linearly to 0..50 mm per finger
            finger_m = float(np.clip(math.degrees(meas.grip_pos) / 270.0, 0.0, 1.0)) * K.FINGER_TRAVEL_M
        links = {name: T.flatten().round(5).tolist() for name, T in K.link_frames(q, finger_m).items()}
        return {
            "connected": self.connected,
            "simulated": self.simulated,
            "torque_enabled": self._enabled,
            "moving": moving,
            "fault": self._fault,
            "joints_deg": np.degrees(q).round(2).tolist(),
            "commanded_deg": np.degrees(q_cmd).round(2).tolist(),
            "points": pts,                                   # base, joint1..joint6 origins, TCP
            "links": links,                                  # link name -> row-major 4x4 in the base frame
            "finger_m": round(finger_m, 5),
            "tcp": Tt[:3, 3].round(4).tolist(),
            "tcp_axes": Tt[:3, :3].T.round(4).tolist(),      # rows: approach(x), y, z axes in base frame
            "gripper_opening": round(grip, 3),
            "target": target,
            "limits_deg": C.JOINT_LIMITS_DEG.tolist(),
            "workspace": {"x": C.WORKSPACE_X, "y": C.WORKSPACE_Y, "z": C.WORKSPACE_Z},
        }

    # ------------------------------------------------------------------
    # planning
    # ------------------------------------------------------------------
    def _check_xyz(self, xyz) -> np.ndarray:
        return check_xyz(xyz)

    def _check_limits(self, q: np.ndarray, what: str = "target") -> None:
        check_limits(q, what)

    def _check_path(self, wps: np.ndarray, n_samples: int = 60) -> None:
        check_path(wps, n_samples)

    def _speed(self, scale: float | None) -> float:
        s = C.DEFAULT_SPEED_SCALE if scale is None else float(scale)
        if not math.isfinite(s) or s <= 0:
            raise ArmError("speed_scale must be > 0")
        return min(s, self.max_speed_scale)

    def plan_xyz(self, x: float, y: float, z: float, approach=None, linear: bool = False) -> dict:
        """IK + checks only, no motion.  Returns the joint solution."""
        self._require(motion=False)
        target = self._check_xyz([x, y, z])
        q0 = self._q_meas()
        q_goal, waypoints, res = self._plan(q0, target, approach, linear)
        p, R = K.fk(q_goal)
        return {
            "ok": True,
            "reachable": True,
            "joints_deg": _round(np.degrees(q_goal), 2),
            "achieved_xyz_m": _round(p, 4),
            "error_mm": round(res.pos_error * 1000, 3),
            "approach_error_deg": None if res.approach_error_deg is None else round(res.approach_error_deg, 2),
            "waypoints": len(waypoints),
            "duration_s_at_default_speed": round(path_duration(waypoints, self._speed(None)), 2),
        }

    def _plan(self, q0, target, approach, linear):
        return plan_path(q0, target, approach, linear)

    # ------------------------------------------------------------------
    # motion
    # ------------------------------------------------------------------
    def _execute(self, wps: np.ndarray, speed_scale: float | None, wait: bool = True) -> dict:
        scale = self._speed(speed_scale)
        if not self._move_lock.acquire(blocking=False):
            raise ArmError("the arm is busy with another move; call stop first")
        try:
            with self._lock:
                start = self._q_cmd.copy()
                wps = np.array(wps, dtype=float)
                wps[0] = start                       # begin exactly where the setpoint is now
                duration, ramp = path_timing(wps, scale)
                self._abort = None
                self._done.clear()
                self._traj = Trajectory(wps, duration, t0=self._clock(), ramp=ramp)
            if not wait:
                return {"duration_s": round(duration, 2), "speed_scale": scale}
            deadline = self._clock() + duration + 5.0
            try:
                while not (finished := self._done.wait(timeout=0.05)) and self._clock() < deadline:
                    pass
            except BaseException:            # Ctrl+C etc.: never leave a trajectory running unattended
                self.stop()
                raise
            if not finished:
                self.stop()
                raise ArmError("move timed out")
            if self._abort:
                raise ArmError(f"move aborted: {self._abort}")
            return {"duration_s": round(duration, 2), "speed_scale": scale}
        finally:
            self._move_lock.release()

    def execute_path(self, waypoints, speed_scale: float | None = None) -> dict:
        """Run a joint path (M,6 rad, e.g. from :func:`plan_path`) that starts at the current pose.

        The path is checked against the joint limits only; plan it with :func:`plan_path`.
        """
        self._require()
        wps = np.asarray(waypoints, dtype=float)
        if wps.ndim != 2 or wps.shape[1] != 6 or len(wps) < 2 or not np.all(np.isfinite(wps)):
            raise ArmError("waypoints must be an (M>=2, 6) array of finite joint angles [rad]")
        for q in wps[1:]:
            check_limits(q, "path")
        return self._execute(wps, speed_scale)

    def move_to_xyz(self, x, y, z, approach=None, linear: bool = False, speed_scale: float | None = None) -> dict:
        """Move the TCP to (x, y, z) [m] in the base frame.

        approach: None/'free', 'down', 'up', 'forward' or [ax, ay, az]: direction of the gripper axis.
        linear:   True = straight line in Cartesian space (slower to plan, safer path).
        """
        self._require()
        target = self._check_xyz([x, y, z])
        q_goal, wps, res = self._plan(self._q_meas(), target, approach, linear)
        self._last_target = target
        info = self._execute(wps, speed_scale)
        return self._report(target, info, res)

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, approach=None, linear: bool = True, speed_scale: float | None = None) -> dict:
        self._require()
        p, _ = K.fk(self._q_meas())
        return self.move_to_xyz(p[0] + dx, p[1] + dy, p[2] + dz, approach=approach, linear=linear, speed_scale=speed_scale)

    def _report(self, target, info, res) -> dict:
        p, R = K.fk(self._q_meas())
        out = {
            "ok": True,
            "target_xyz_m": _round(target, 4),
            "reached_xyz_m": _round(p, 4),
            "error_mm": round(float(np.linalg.norm(p - target)) * 1000, 2),
            "joints_deg": _round(np.degrees(self._q_meas()), 2),
            "approach_axis": _round(R[:, 0], 3),
        }
        out.update(info)
        return out

    def move_joints(self, angles_deg, speed_scale: float | None = None) -> dict:
        self._require()
        a = np.asarray(angles_deg, dtype=float).reshape(-1)
        if a.shape != (6,) or not np.all(np.isfinite(a)):
            raise ArmError("angles_deg must be 6 finite numbers (joint1..joint6)")
        q = np.radians(a)
        self._check_limits(q)
        wps = np.vstack([self._q_meas(), q])
        self._check_path(wps)
        info = self._execute(wps, speed_scale)
        p, _ = K.fk(self._q_meas())
        return {"ok": True, "joints_deg": _round(np.degrees(self._q_meas()), 2), "tcp_xyz_m": _round(p, 4), **info}

    def home(self, speed_scale: float | None = None) -> dict:
        """Go to the calibrated zero pose (all joints 0)."""
        return self.move_joints(C.HOME_DEG, speed_scale)

    def set_gripper(self, opening: float, wait: bool = True) -> dict:
        """opening 0 (closed) .. 1 (open)."""
        self._require()
        if not C.GRIPPER_ENABLED:
            raise ArmError("gripper is disabled (REBOT_DISABLE_GRIPPER)")
        o = float(opening)
        if not math.isfinite(o) or not 0.0 <= o <= 1.0:
            raise ArmError("opening must be between 0 (closed) and 1 (open)")
        with self._lock:
            self._grip_target = o * math.radians(C.GRIPPER_OPEN_DEG)
            self._grip_cmd_time = self._clock()
        if wait:
            deadline = self._clock() + 2.0
            while self._clock() < deadline:
                time.sleep(0.01)
                if self._fault:
                    raise ArmError(f"fault while moving the gripper: {self._fault}")
                if abs(self._grip_target - self._meas.grip_pos) < math.radians(3.0):
                    break
        return {"ok": True, "gripper_opening": self.status()["gripper_opening"]}

    def stop(self) -> dict:
        """Cancel the running move and hold the current setpoint (torque stays on)."""
        with self._lock:
            if self._traj is not None:
                self._abort = "stopped by request"
            self._hold = self._q_cmd.copy()
            self._traj = None
            self._done.set()
        return {"ok": True, "message": "motion stopped, holding position"}

    def clear_fault(self) -> dict:
        """Accept a fault without cutting torque: hold where the arm is now and allow motion again.

        Only while the motors are still enabled (a tracking, feedback or temperature fault);
        after the torque went off, disconnect and connect.  A cause that persists faults again.
        """
        self._require(motion=False)
        if not self._enabled:
            raise ArmError("torque is off; disconnect and connect again")
        if self._stale:
            raise ArmError("no motor feedback (the CAN adapter stopped receiving): quit, replug it and start again")
        with self._lock:
            fault = self._fault
            self._hold = np.clip(self._meas.q, C.JOINT_LIMITS_RAD[:, 0], C.JOINT_LIMITS_RAD[:, 1])
            self._q_cmd = self._hold.copy()
            self._traj = None
            self._err_since[:] = np.nan
            self._fault = self._abort = None
        if fault:
            log.warning("fault cleared: %s", fault)
        return {"ok": True, "message": "fault cleared, holding position" if fault else "no fault"}

    def emergency_disable(self) -> dict:
        """Cut motor torque immediately.  The arm will fall / sag under gravity!"""
        self.stop()
        if self.backend is not None:
            self.backend.disable()
        self._enabled = False
        with self._lock:
            if self._fault is None:
                self._fault = "torque disabled by emergency_disable"
        return {"ok": True, "message": "torque disabled; support the arm"}

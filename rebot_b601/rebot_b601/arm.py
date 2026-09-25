"""High level driver for the reBot Arm B601-RS.

* :class:`Arm` runs a 50 Hz control thread that streams the current setpoint to
  the motors, executes smooth (minimum-jerk) joint trajectories, drives the
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

import logging
import math
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


def _min_jerk(tau: float) -> float:
    tau = min(max(tau, 0.0), 1.0)
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5


def path_duration(waypoints: np.ndarray, speed_scale: float) -> float:
    """Time needed to run ``waypoints`` (M,6 rad) with a min-jerk profile at ``speed_scale``."""
    vmax = np.radians(C.JOINT_SPEED_DPS) * speed_scale
    seg = np.abs(np.diff(waypoints, axis=0)) / vmax
    total = float(np.sum(np.max(seg, axis=1))) if len(seg) else 0.0
    return max(C.MIN_MOVE_TIME_S, 1.875 * total)     # min-jerk peak speed = 1.875 x mean


@dataclass
class Trajectory:
    waypoints: np.ndarray            # (M,6) rad
    duration: float
    t0: float = 0.0
    _u: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        vmax = np.radians(C.JOINT_SPEED_DPS)
        d = np.max(np.abs(np.diff(self.waypoints, axis=0)) / vmax, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(d)])
        self._u = cum / cum[-1] if cum[-1] > 0 else np.linspace(0.0, 1.0, len(cum))

    def sample(self, t: float) -> np.ndarray:
        s = _min_jerk(t / self.duration)
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


class HardwareBackend:
    """RobStride motors 1-7 over SocketCAN through the ``motorbridge`` SDK."""

    simulated = False
    _LOC_REF = 0x7016      # position reference
    _LIMIT_SPD = 0x7017
    _LOC_KP = 0x701E
    _SPD_KP = 0x701F
    _SPD_KI = 0x7020
    _MECH_POS = 0x7019

    def __init__(self, channel: str = C.CAN_CHANNEL):
        self.channel = channel
        self.ctrl = None
        self.arm_motors: list = []
        self.gripper = None
        self._enabled = False

    # -- helpers ---------------------------------------------------------
    def _all(self):
        return self.arm_motors + ([self.gripper] if self.gripper is not None else [])

    def _poll(self) -> None:
        for m in self._all():
            try:
                m.request_feedback()
            except Exception:
                pass
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
        for i, m in enumerate(self.arm_motors):
            m.send_pos_vel(float(q_cmd[i]), float(C.MOTOR_VLIM_RAD_S))

    def send_gripper_torque(self, tau: float) -> None:
        if self.gripper is not None:
            # like Seeed's follower: no position term, light damping, feed-forward torque
            self.gripper.send_mit(0.0, 0.0, 0.0, 1.5, float(tau))

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
# Arm
# --------------------------------------------------------------------------


def _round(a, n=3):
    return [round(float(x), n) for x in a]


class Arm:
    def __init__(self, hz: float = C.CONTROL_HZ, max_speed_scale: float = C.MAX_SPEED_SCALE):
        self.hz = hz
        self.max_speed_scale = max_speed_scale
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

    def connect(self, enable: bool = True, simulate: bool | None = None, sim_start_deg=None) -> dict:
        """Connect to the arm.  ``enable=False`` only reads feedback (motors stay limp)."""
        if self.connected:
            raise ArmError("already connected")
        simulate = C.DRY_RUN if simulate is None else simulate
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
        self.backend.close()
        self.backend = None
        self._enabled = False
        return {"ok": True, "message": "disconnected" + (f" ({note})" if note else "")}

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------
    def _loop(self) -> None:
        period = 1.0 / self.hz
        nxt = time.monotonic()
        while not self._stop_thread.is_set():
            try:
                self._tick(time.monotonic())
            except Exception as e:      # never let the loop die silently
                log.exception("control loop error")
                self._raise_fault(f"control loop error: {e!r}")
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

    def _gripper_torque(self, meas: Measurement, now: float) -> float:
        if not C.GRIPPER_ENABLED:
            return 0.0
        err = self._grip_target - meas.grip_pos
        tau = C.GRIPPER_KP * err + C.GRIPPER_KD * (0.0 - meas.grip_vel)
        # full torque only for a short time after a new command; afterwards just hold
        lim = C.GRIPPER_TORQUE_LIMIT if (now - self._grip_cmd_time) < 1.5 else C.GRIPPER_HOLD_TORQUE
        return float(np.clip(tau, -lim, lim))

    def _safety_checks(self, meas: Measurement, q_cmd: np.ndarray, now: float) -> None:
        # tracking error
        err = np.degrees(np.abs(meas.q - q_cmd))
        for j in range(6):
            if err[j] > C.TRACKING_ERR_DEG:
                if np.isnan(self._err_since[j]):
                    self._err_since[j] = now
                elif now - self._err_since[j] > C.TRACKING_ERR_TIME_S:
                    self._raise_fault(
                        f"joint{j + 1} is {err[j]:.1f} deg away from its commanded position "
                        "(blocked, collision or motor fault); motion aborted and pose held"
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
            if self._meas is not None:
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
                raise ArmError(f"arm is in a fault state: {self._fault}. Inspect the arm, then disconnect and connect again")

    def _q_meas(self) -> np.ndarray:
        with self._lock:
            return self._meas.q.copy()

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
        p = np.asarray(xyz, dtype=float).reshape(3)
        if not np.all(np.isfinite(p)):
            raise ArmError("target contains NaN/inf")
        for name, v, (lo, hi) in zip("xyz", p, (C.WORKSPACE_X, C.WORKSPACE_Y, C.WORKSPACE_Z)):
            if not lo <= v <= hi:
                raise ArmError(f"{name}={v:.3f} m is outside the allowed workspace box [{lo:.2f}, {hi:.2f}] m")
        return p

    def _check_limits(self, q: np.ndarray, what: str = "target") -> None:
        tol = math.radians(0.5)
        lo, hi = C.JOINT_LIMITS_RAD[:, 0], C.JOINT_LIMITS_RAD[:, 1]
        for j in range(6):
            if q[j] < lo[j] - tol or q[j] > hi[j] + tol:
                raise ArmError(
                    f"{what}: joint{j + 1} = {math.degrees(q[j]):.1f} deg is outside its limit "
                    f"[{C.JOINT_LIMITS_DEG[j, 0]:.0f}, {C.JOINT_LIMITS_DEG[j, 1]:.0f}] deg"
                )

    def _check_path(self, wps: np.ndarray, n_samples: int = 60) -> None:
        """Sample the planned path and reject it if it dips below the table / hits the base."""
        traj = Trajectory(wps, 1.0)
        for s in np.linspace(0.0, 1.0, n_samples):
            q = np.array([np.interp(s, traj._u, wps[:, j]) for j in range(6)])
            ok, why = K.pose_is_safe(q, z_min=C.Z_MIN)
            if not ok:
                # the start pose may legitimately be low (e.g. resting); only reject if it gets worse
                if s == 0.0:
                    continue
                raise ArmError(f"path rejected: {why}")

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
        lim = C.JOINT_LIMITS_RAD
        res = K.solve_ik(target, approach, q0, lim, z_min=C.Z_MIN)
        if not res.success:
            raise ArmError(f"target {np.round(target, 3).tolist()} is not reachable: {res.message}")
        q_goal = res.q
        self._check_limits(q_goal)
        if not linear:
            wps = np.vstack([q0, q_goal])
            self._check_path(wps)
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
            r = K.solve_ik(pk, ak, q, lim, n_random_starts=0, z_min=C.Z_MIN)
            if not r.success:
                raise ArmError(f"straight-line path fails at {np.round(pk, 3).tolist()}: {r.message}")
            if np.max(np.abs(r.q - q)) > math.radians(12.0):
                raise ArmError("straight-line path needs a large joint jump (singularity or configuration change); use linear=False")
            self._check_limits(r.q, "path")
            wps.append(r.q)
            q = r.q
        wps = np.array(wps)
        self._check_path(wps, n_samples=max(60, n))
        return q_goal, wps, res

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
                duration = path_duration(wps, scale)
                self._abort = None
                self._done.clear()
                self._traj = Trajectory(wps, duration, t0=time.monotonic())
            if not wait:
                return {"duration_s": round(duration, 2), "speed_scale": scale}
            try:
                finished = self._done.wait(timeout=duration + 5.0)
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
            self._grip_cmd_time = time.monotonic()
        if wait:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                time.sleep(0.05)
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

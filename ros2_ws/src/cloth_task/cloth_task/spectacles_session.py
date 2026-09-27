"""ROS-free Spectacles teleop session: the robot-side peer of the lens (teleop v1).

The robot bridge node feeds it teleop messages and joint measurements; it answers with joint
targets and gripper commands for `arm_bridge`'s teleop command, and with status messages.

Right clutch: the rising edge latches the measured end-effector pose (the fingertip centre,
rebot_b601's TCP) in the arm base frame. Each later engaged sample asks for
`p = p_ee + position`, `R = R(orientation) · R_ee`, solved for the full pose from the measured
joints (damped least squares, no restarts). A solve that misses 1 mm / 3° publishes nothing and
the arm report stays tracking. Releasing the clutch stops publishing: `arm_bridge` holds the
setpoint it already committed.

Left clutch (mobile base): accepted only while the rover is enabled and present (odometry within
ROVER_ABSENT_S, ADR 0014), the socket is open, the hands are not blocked (below), and the left
hand has been seen open since the rover was last absent while connected. The base is driving
while the left clutch is accepted and the latest valid teleop, under BASE_STALE_S old, has it
engaged (ADR 0013): each `tick()` then sends that teleop's `vx`, `wz`, clamped to the limits.
The call on which driving ends sends exactly one zero; nothing is sent otherwise. Base stale
needs no release: the next engaged teleop drives again. The arm ignores all of this.

Link: once a socket is accepted, TIMEOUT_S without a valid teleop on the bridge's receive clock
(not the lens timestamp) stops publishing and reports both arm and base as fault, fault timeout.
The next valid teleop clears it. A replacement socket stops the arm and base and starts with
fault null. After a timeout or a replacement, each hand must be seen open (a clutch held while
blocked consumes that hand's release) before either can command again. The first socket of a
session accepts the first right clutch straight away. A closed socket stops the base at once;
the arm and the later link timeout are unaffected by the close.

Driver: a driver-fault flag, or no joint measurement for MEAS_TIMEOUT_S, stops publishing and
reports the arm holding with fault null (the lens sees the disagreement). Tracking resumes only
on a right clutch pressed after the arm is healthy again. A socket is accepted only once teleop
mode is on and a joint measurement has arrived.

Kept free of rclpy so it can be unit-tested with plain pytest.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

PROTOCOL_VERSION = 1
TOL_POS_M = 1e-3
TOL_ANG_DEG = 3.0
MIN_QUAT_NORM = 1e-6
# Through ngrok, teleop round trips stall 300 ms to 1.2 s; the lens itself allows 3 s of silence
TIMEOUT_S = 2.0
# Generous: the driver itself faults after 0.3 s of failed motor reads
MEAS_TIMEOUT_S = 0.5
# ADR 0013: at 0.35 m/s the 2 s link timeout is ~70 cm of run-on, so the base stops sooner
BASE_STALE_S = 0.3
# ADR 0014: /leo/merged_odom arrives at 100 Hz
ROVER_ABSENT_S = 0.5
HANDS = frozenset({"arm", "base"})


@dataclass(frozen=True)
class Teleop:
    seq: int
    arm_engaged: bool
    position: np.ndarray  # (3,) m, arm base: +X forward, +Y left, +Z up
    orientation: np.ndarray  # (4,) x, y, z, w, unit
    gripper: float  # 0 closed .. 1 open, clamped
    base_engaged: bool
    vx: float  # m/s, + forward, unclamped
    wz: float  # rad/s, + left (counter-clockwise), unclamped


def _number(v: Any) -> bool:
    return isinstance(v, int | float) and not isinstance(v, bool) and math.isfinite(v)


def parse_teleop(msg: Any) -> Teleop | None:
    """A teleop v1 message (decoded JSON) → Teleop, or None if malformed. Unknown fields are
    ignored; a quaternion with a finite norm of at least MIN_QUAT_NORM is renormalized."""
    if not isinstance(msg, Mapping):
        return None
    if msg.get("v") != PROTOCOL_VERSION or msg.get("type") != "teleop":
        return None
    seq, base, arm = msg.get("seq"), msg.get("base"), msg.get("arm")
    if not isinstance(seq, int) or isinstance(seq, bool) or not _number(msg.get("timestamp")):
        return None
    if not isinstance(base, Mapping) or not isinstance(arm, Mapping):
        return None
    if not isinstance(base.get("engaged"), bool) or not all(
        _number(base.get(k)) for k in ("vx", "wz")
    ):
        return None
    pos, quat, grip = arm.get("position"), arm.get("orientation"), arm.get("gripper")
    if not isinstance(arm.get("engaged"), bool) or not _number(grip):
        return None
    if not isinstance(pos, list) or len(pos) != 3 or not all(_number(v) for v in pos):
        return None
    if not isinstance(quat, list) or len(quat) != 4 or not all(_number(v) for v in quat):
        return None
    q = np.array(quat, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < MIN_QUAT_NORM:
        return None
    return Teleop(
        seq=seq,
        arm_engaged=arm["engaged"],
        position=np.array(pos, dtype=float),
        orientation=q / norm,
        gripper=min(max(float(grip), 0.0), 1.0),
        base_engaged=base["engaged"],
        vx=float(base["vx"]),
        wz=float(base["wz"]),
    )


def quat_to_R(q: np.ndarray) -> np.ndarray:
    """Unit quaternion (x, y, z, w) → rotation matrix. R(a) · R(b) = R(a ⊗ b) (Hamilton)."""
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotvec(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → axis · angle (rad)."""
    ang = math.acos(min(max((float(np.trace(R)) - 1.0) / 2.0, -1.0), 1.0))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = float(np.linalg.norm(w))  # 2 sin(ang)
    if s > 1e-9:
        return w / s * ang
    if ang < 1e-6:
        return np.zeros(3)
    B = R + np.eye(3)  # ang ≈ π: any non-zero column of R + I is the axis
    axis = B[:, int(np.argmax(np.linalg.norm(B, axis=0)))]
    return axis / np.linalg.norm(axis) * ang


@dataclass(frozen=True)
class Solve:
    q: np.ndarray
    converged: bool
    pos_error_m: float
    ang_error_deg: float


def solve_pose(
    K: Any,
    q_seed: np.ndarray,
    p_target: np.ndarray,
    R_target: np.ndarray,
    limits: np.ndarray,
    *,
    # 30 converges on the same frames as 100; a miss then costs ~15 ms instead of ~50 ms,
    # under the lock that the status replies wait on
    max_iter: int = 30,
    w_ang: float = 0.5,
) -> Solve:
    """Full-pose (position and orientation) damped least squares from q_seed only, within
    limits (N×2 rad). Converged = within TOL_POS_M and TOL_ANG_DEG."""
    lo, hi = limits[:, 0], limits[:, 1]
    q = np.clip(np.asarray(q_seed, dtype=float), lo, hi)
    tight_pos, tight_ang = TOL_POS_M * 0.3, math.radians(TOL_ANG_DEG) * 0.3
    for _ in range(max_iter):
        p, R = K.fk(q)
        e_pos = p_target - p
        e_rot = rotvec(R_target @ R.T)
        if np.linalg.norm(e_pos) < tight_pos and np.linalg.norm(e_rot) < tight_ang:
            break
        J = K.jacobian(q)
        J[3:] *= w_ang
        err = np.concatenate([e_pos, w_ang * e_rot])
        lam2 = 0.5 * float(err @ err) + 1e-7
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), err)
        step = float(np.max(np.abs(dq)))
        if step > 0.4:
            dq *= 0.4 / step
        q = np.clip(q + dq, lo, hi)
    p, R = K.fk(q)
    pos_err = float(np.linalg.norm(p_target - p))
    ang_err = math.degrees(float(np.linalg.norm(rotvec(R_target @ R.T))))
    return Solve(q, pos_err <= TOL_POS_M and ang_err <= TOL_ANG_DEG, pos_err, ang_err)


class Session:
    """One lens session. `send_joints(q)` gets joint1..6 targets (rad), `send_gripper(opening)`
    the gripper opening 0..1; both are only called while the right clutch is accepted.
    `send_base(vx, wz)` gets the base command (m/s, rad/s) on each `tick()` while driving and
    one zero when driving ends; with `rover` false it only gets the zero from `on_shutdown()`.
    `max_vx`, `max_reverse` (positive) and `max_wz` clamp the base command.
    `clock()` is the bridge's receive clock, s. Report the result of enabling teleop mode with
    `on_teleop_mode()`, the driver-fault flag with `on_driver_fault()` and each rover odometry
    message with `on_odometry()`. Call `on_connect()` for each socket and close it if refused,
    `on_disconnect()` when the accepted socket closes, `on_shutdown()` last, and `tick()`
    periodically (well under BASE_STALE_S) so a silent link, a stale measurement or an absent
    rover is noticed."""

    def __init__(
        self,
        kinematics: Any,
        limits: np.ndarray,
        send_joints: Callable[[np.ndarray], None],
        send_gripper: Callable[[float], None],
        clock: Callable[[], float] = time.monotonic,
        *,
        send_base: Callable[[float, float], None] = lambda vx, wz: None,
        rover: bool = False,
        max_vx: float = 0.20,
        max_reverse: float = 0.10,
        max_wz: float = 0.6,
    ):
        self._K = kinematics
        self._limits = np.asarray(limits, dtype=float)
        self._send_joints = send_joints
        self._send_gripper = send_gripper
        self._clock = clock
        self._send_base = send_base
        self._rover = bool(rover)
        self._max_vx, self._max_reverse, self._max_wz = max_vx, max_reverse, max_wz
        self._t_odom: float | None = None
        self._socket_open = False
        self._shut_down = False
        self._base_engaged = False  # in the latest valid teleop of this socket
        self._base_cmd = (0.0, 0.0)  # its clamped (vx, wz)
        self._base_need_release = False
        self._driving = False
        self._q_meas: np.ndarray | None = None
        self._latch: tuple[np.ndarray, np.ndarray] | None = None  # (p_ee, R_ee)
        self._echo_seq: int | None = None
        self._connected = False
        self._last_valid = 0.0
        self._fault: str | None = None
        self._blocked = False
        self._released: set[str] = set()
        self._t_meas = 0.0
        self._teleop_mode = False
        self._driver_fault = False
        self._need_release = False

    def on_joint_state(self, q: Any) -> None:
        """A joint1..6 measurement, rad."""
        self._q_meas = np.asarray(q, dtype=float).copy()
        self._t_meas = self._clock()

    def on_teleop_mode(self, enabled: bool) -> None:
        """Whether enabling `arm_bridge`'s teleop mode succeeded (False if it was refused)."""
        self._teleop_mode = bool(enabled)

    def on_driver_fault(self, faulted: bool) -> None:
        """The driver-fault flag; it stays true until the arm is connected again."""
        self._driver_fault = bool(faulted)
        self._update()

    def on_odometry(self) -> None:
        """A rover odometry message arrived; its content is not used."""
        self._t_odom = self._clock()
        self._update()

    def on_connect(self) -> bool:
        """A lens socket arrived. Accepted only once teleop mode is on and a joint measurement
        has arrived; any accepted socket after the first replaces the previous one."""
        if not self._teleop_mode or self._q_meas is None:
            return False
        if self._connected:
            self._block()
        self._connected = True
        self._socket_open = True
        self._fault = None
        self._echo_seq = None
        self._last_valid = self._clock()
        self._base_engaged = False
        self._update()
        return True

    def on_disconnect(self) -> None:
        """The accepted socket closed: the base stops at once. The arm, the fault and the link
        timeout carry on as if the socket had gone silent."""
        self._socket_open = False
        self._update()

    def on_shutdown(self) -> None:
        """The bridge is stopping: sends one zero base command, whatever the state, and never
        drives again."""
        self._shut_down = True
        self._driving = False
        self._send_base(0.0, 0.0)

    def _arm_ok(self) -> bool:
        return (
            not self._driver_fault
            and self._q_meas is not None
            and self._clock() - self._t_meas < MEAS_TIMEOUT_S
        )

    def _rover_present(self) -> bool:
        return self._t_odom is not None and self._clock() - self._t_odom < ROVER_ABSENT_S

    def tick(self) -> None:
        """Stops the arm on a driver fault or a stale measurement, times the link out after
        TIMEOUT_S without a valid teleop, stops the base on base stale or an absent rover, and
        sends the base command while driving."""
        self._update()
        if self._driving:
            self._send_base(*self._base_cmd)

    def _update(self) -> None:
        if self._connected:
            if not self._arm_ok():
                self._latch = None
                self._need_release = True
            if self._fault is None and self._clock() - self._last_valid >= TIMEOUT_S:
                self._fault = "timeout"
                self._block()
        self._update_base()

    def _update_base(self) -> None:
        """Re-evaluates driving after any change; sends the one zero when it ends."""
        present = self._rover_present()
        if self._socket_open and not present:
            self._base_need_release = True
        driving = (
            self._rover
            and present
            and self._socket_open
            and not self._shut_down
            and not self._blocked
            and not self._base_need_release
            and self._base_engaged
            and self._clock() - self._last_valid < BASE_STALE_S
        )
        if self._driving and not driving:
            self._send_base(0.0, 0.0)
        self._driving = driving

    def _block(self) -> None:
        self._latch = None
        self._blocked = True
        self._released = set()

    def on_teleop(self, msg: Any) -> None:
        """A decoded teleop v1 message from the accepted socket. Malformed ones change nothing
        and do not refresh the link timer."""
        self._update()
        t = parse_teleop(msg)
        if t is None or not self._connected:
            return
        self._last_valid = self._clock()
        self._echo_seq = t.seq
        self._fault = None
        if self._blocked:
            for hand, engaged in (("arm", t.arm_engaged), ("base", t.base_engaged)):
                if engaged:
                    self._released.discard(hand)
                else:
                    self._released.add(hand)
            self._blocked = self._released != HANDS
        if self._base_need_release and self._rover_present() and not t.base_engaged:
            self._base_need_release = False
        self._base_engaged = t.base_engaged
        self._base_cmd = (
            min(max(t.vx, -self._max_reverse), self._max_vx),
            min(max(t.wz, -self._max_wz), self._max_wz),
        )
        self._update_base()
        if self._need_release and self._arm_ok() and not t.arm_engaged:
            self._need_release = False
        if self._blocked or self._need_release or not t.arm_engaged:
            self._latch = None
            return
        if self._latch is None:
            self._latch = self._K.fk(self._q_meas)
        p_ee, R_ee = self._latch
        sol = solve_pose(
            self._K,
            self._q_meas,
            p_ee + t.position,
            quat_to_R(t.orientation) @ R_ee,
            self._limits,
        )
        if not sol.converged:
            return
        self._send_joints(sol.q)
        self._send_gripper(t.gripper)

    def status(self) -> dict[str, Any]:
        """The status v1 message to send now."""
        self._update()
        if self._fault is not None:
            base, arm = "fault", "fault"
        else:
            base = "driving" if self._driving else "idle"
            arm = "tracking" if self._latch is not None else "holding"
        return {
            "v": PROTOCOL_VERSION,
            "type": "status",
            "echoSeq": self._echo_seq,
            "base": base,
            "arm": arm,
            "fault": self._fault,
        }

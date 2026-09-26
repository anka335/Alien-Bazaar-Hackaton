"""ROS-free Spectacles teleop session: the robot-side peer of the lens (teleop v1).

The robot bridge node feeds it teleop messages and joint measurements; it answers with joint
targets and gripper commands for `arm_bridge`'s teleop command, and with status messages.

Right clutch: the rising edge latches the measured end-effector pose (the fingertip centre,
rebot_b601's TCP) in the arm base frame. Each later engaged sample asks for
`p = p_ee + position`, `R = R(orientation) · R_ee`, solved for the full pose from the measured
joints (damped least squares, no restarts). A solve that misses 1 mm / 3° publishes nothing and
the arm report stays tracking. Releasing the clutch stops publishing: `arm_bridge` holds the
setpoint it already committed. The left clutch is never accepted: there is no mobile base.

Kept free of rclpy so it can be unit-tested with plain pytest.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

PROTOCOL_VERSION = 1
TOL_POS_M = 1e-3
TOL_ANG_DEG = 3.0
MIN_QUAT_NORM = 1e-6


@dataclass(frozen=True)
class Teleop:
    seq: int
    arm_engaged: bool
    position: np.ndarray  # (3,) m, arm base: +X forward, +Y left, +Z up
    orientation: np.ndarray  # (4,) x, y, z, w, unit
    gripper: float  # 0 closed .. 1 open, clamped
    base_engaged: bool


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
    max_iter: int = 100,
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
    the gripper opening 0..1; both are only called while the right clutch is accepted."""

    def __init__(
        self,
        kinematics: Any,
        limits: np.ndarray,
        send_joints: Callable[[np.ndarray], None],
        send_gripper: Callable[[float], None],
    ):
        self._K = kinematics
        self._limits = np.asarray(limits, dtype=float)
        self._send_joints = send_joints
        self._send_gripper = send_gripper
        self._q_meas: np.ndarray | None = None
        self._latch: tuple[np.ndarray, np.ndarray] | None = None  # (p_ee, R_ee)
        self._echo_seq: int | None = None

    def on_joint_state(self, q: Any) -> None:
        """A joint1..6 measurement, rad."""
        self._q_meas = np.asarray(q, dtype=float).copy()

    def on_teleop(self, msg: Any) -> None:
        """A decoded teleop v1 message. Malformed ones change nothing."""
        t = parse_teleop(msg)
        if t is None:
            return
        self._echo_seq = t.seq
        if not t.arm_engaged or self._q_meas is None:
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
        return {
            "v": PROTOCOL_VERSION,
            "type": "status",
            "echoSeq": self._echo_seq,
            "base": "idle",
            "arm": "tracking" if self._latch is not None else "holding",
            "fault": None,
        }

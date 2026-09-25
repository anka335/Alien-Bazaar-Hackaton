"""Forward / inverse kinematics of the SO-101 arm (5 revolute joints + gripper), in mm and rad.

Geometry: `so101_new_calib.urdf` (TheRobotStudio/SO-ARM100, Apache-2.0). Joint zero is the middle
of each joint's calibrated range, as in LeRobot's degree mode. The base frame is the URDF
`base_link`: +Z up, origin on the base plate. The tool frame (flange, what `fk` returns) is
`gripper_frame_link`, between the fingertips; its +Z axis is the approach direction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
N_JOINTS = len(JOINT_NAMES)

# (xyz [m], rpy [rad]) of each revolute joint, parent → child; every axis is local +Z
_JOINTS = (
    ((0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159)),
    ((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0)),
    ((-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708)),
    ((-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708)),
    ((5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159)),
)
_TOOL = ((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))  # gripper_link → gripper_frame

LIMITS = np.array(  # URDF limits, rad
    [
        (-1.91986, 1.91986),
        (-1.74533, 1.74533),
        (-1.69, 1.69),
        (-1.65806, 1.65806),
        (-2.74385, 2.84121),
    ]
)


def _rpy(rpy: tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array(  # URDF: Rz(y) @ Ry(p) @ Rx(r)
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy(rpy)
    T[:3, 3] = np.asarray(xyz) * 1000.0  # mm
    return T


_ORIGINS = [_origin(*j) for j in _JOINTS]
_TOOL_T = _origin(*_TOOL)


def _rz(q: float) -> np.ndarray:
    c, s = np.cos(q), np.sin(q)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def fk(q) -> np.ndarray:
    """4×4 base → tool pose, mm."""
    T = np.eye(4)
    for origin, qi in zip(_ORIGINS, q, strict=True):
        T = T @ origin @ _rz(qi)
    return T @ _TOOL_T


def _fk_all(q) -> tuple[np.ndarray, list[np.ndarray]]:
    """Tool pose and the pose of every joint frame (after its origin, before its rotation)."""
    T = np.eye(4)
    frames = []
    for origin, qi in zip(_ORIGINS, q, strict=True):
        T = T @ origin
        frames.append(T.copy())
        T = T @ _rz(qi)
    return T @ _TOOL_T, frames


def jacobian(q) -> tuple[np.ndarray, np.ndarray]:
    """(tool pose, 6×5 geometric jacobian: rows 0–2 linear mm/rad, rows 3–5 angular)."""
    T, frames = _fk_all(q)
    p = T[:3, 3]
    J = np.zeros((6, N_JOINTS))
    for i, F in enumerate(frames):
        z = F[:3, 2]
        J[:3, i] = np.cross(z, p - F[:3, 3])
        J[3:, i] = z
    return T, J


DOWN = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class IkResult:
    q: np.ndarray
    pos_err_mm: float
    tilt_deg: float  # angle between the approach axis and straight down


def tilt_deg(T: np.ndarray, approach: np.ndarray = DOWN) -> float:
    """Angle between the tool's approach axis and `approach` (straight down by default)."""
    return float(np.degrees(np.arccos(np.clip(T[:3, 2] @ approach, -1.0, 1.0))))


def _dls(J: np.ndarray, lam: float) -> np.ndarray:
    """Damped pseudo-inverse."""
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(J.shape[0]))


def _step(J: np.ndarray, e_pos: np.ndarray, e_rot: np.ndarray, free: np.ndarray) -> np.ndarray:
    """One step over the free joints (roll excluded): position first, "down" in its null space."""
    Jp, Jo = J[:3, :4] * free, J[3:, :4] * free
    Jp_pinv = _dls(Jp, 1.0)
    dq = Jp_pinv @ np.clip(e_pos, -20, 20)
    N = np.eye(4) - Jp_pinv @ Jp
    dq += N @ _dls(Jo @ N, 0.05) @ (0.5 * e_rot - Jo @ dq)
    return dq * free


def ik_down(
    target_mm,
    seed,
    *,
    roll: float | None = None,
    limits: np.ndarray = LIMITS,
    approach=DOWN,
    iters: int = 200,
) -> IkResult:
    """TCP at `target_mm` with the approach axis as close to straight down as reachable.

    Position is the primary task; "down" is secondary, in the null space of the position task, so
    near the edge of the workspace the tool tilts instead of missing the point. The wrist roll
    doesn't change position or approach, so it is kept at `roll` (the seed's if None).
    `approach` (unit vector) replaces "down", e.g. for oblique survey views.
    The caller checks `pos_err_mm` and `tilt_deg` (the angle from `approach`).
    """
    approach = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    target = np.asarray(target_mm, dtype=float)
    q = np.clip(np.asarray(seed, dtype=float).copy(), limits[:, 0], limits[:, 1])
    if roll is not None:
        q[4] = roll
    for _ in range(iters):
        T, J = jacobian(q)
        e_pos = target - T[:3, 3]
        e_rot = np.cross(T[:3, 2], approach)  # rotation that brings the tool axis onto it
        if np.linalg.norm(e_pos) < 0.05 and np.linalg.norm(e_rot) < 1e-4:
            break
        free = np.ones(4, dtype=bool)
        for _ in range(4):  # joints pushed into a limit drop out of the step (active set)
            dq = _step(J, e_pos, e_rot, free)
            nxt = q[:4] + dq
            stuck = free & ((nxt < limits[:4, 0]) | (nxt > limits[:4, 1]))
            if not stuck.any():
                break
            free &= ~stuck
        q[:4] = np.clip(q[:4] + dq, limits[:4, 0], limits[:4, 1])
    T = fk(q)
    return IkResult(q, float(np.linalg.norm(target - T[:3, 3])), tilt_deg(T, approach))

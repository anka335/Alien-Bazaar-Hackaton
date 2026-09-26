"""Arm kinematics and path planning in sorter units (mm, rad), on top of `rebot_b601` (D-019).

The flange is the URDF `link6` frame, the TCP is `gripper_end`, the end of the gripper. Its +x
axis is the approach direction (wrist → fingertips). The wrist camera is fixed to `link5`: joint 6
(wrist roll) turns the gripper, not the camera (D-027). `ee_pose()` is T_base_link5.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from rebot_b601 import arm as rb
from rebot_b601 import config as rc
from rebot_b601 import kinematics as rk

from sorter.core.errors import ArmError, TargetRejected
from sorter.core.types import Pose

N_JOINTS = rk.N_JOINTS
JOINT_LIMITS = rc.JOINT_LIMITS_RAD  # (6, 2), rad
Approach = str | Sequence[float] | None  # "down" / "forward" / "free" / a 3-vector / None = free


def _mm(T: np.ndarray) -> Pose:
    T = np.array(T, dtype=np.float64)
    T[:3, 3] *= 1000.0
    return T


def fk_flange(q: Sequence[float]) -> Pose:
    """T_base_link5, mm."""
    return _mm(rk.joint_frames(q)[N_JOINTS - 1])


def fk_link5(q: Sequence[float]) -> Pose:
    """T_base_link5, mm: the link the wrist camera is fixed to."""
    return _mm(rk.joint_frames(q)[N_JOINTS - 2])


def fk_tcp(q: Sequence[float]) -> Pose:
    """T_base_tcp, mm."""
    return _mm(rk.joint_frames(q)[N_JOINTS])


_F0 = rk.joint_frames(np.zeros(N_JOINTS))
T_FLANGE_TCP: Pose = _mm(np.linalg.inv(_F0[N_JOINTS - 1]) @ _F0[N_JOINTS])
T_LINK5_TCP0: Pose = _mm(np.linalg.inv(_F0[N_JOINTS - 2]) @ _F0[N_JOINTS])  # joint 6 at 0


def plan_to(
    q0: Sequence[float],
    xyz_mm: Sequence[float],
    approach: Approach = "down",
    *,
    linear: bool = False,
    z_min_mm: float,
) -> np.ndarray:
    """Joint waypoints (M, 6) that move the TCP from `q0` to `xyz_mm`.

    `linear`: a straight line in Cartesian space. Raises TargetRejected if IK or the path check
    fails; nothing moves.
    """
    try:
        _, wps, _ = rb.plan_path(
            q0, np.asarray(xyz_mm, dtype=float) / 1000.0, approach, linear, z_min=z_min_mm / 1000.0
        )
    except rb.ArmError as e:
        raise TargetRejected(str(e)) from None
    return wps


def plan_joints(q0: Sequence[float], q1: Sequence[float], *, z_min_mm: float) -> np.ndarray:
    """Joint-space waypoints from `q0` to `q1`. ArmError if `q1` is outside the joint limits or
    the path goes below the table or into the base."""
    wps = np.array([q0, q1], dtype=float)
    try:
        rb.check_limits(wps[1])
        rb.check_path(wps, z_min=z_min_mm / 1000.0)
    except rb.ArmError as e:
        raise ArmError(str(e)) from None
    return wps


def path_duration(wps: np.ndarray, speed_scale: float) -> float:
    """Seconds the real arm takes for `wps` at `speed_scale` (min-jerk profile)."""
    return rb.path_duration(np.asarray(wps, dtype=float), speed_scale)


def solve(
    xyz_mm: Sequence[float], approach: Approach, seed: Sequence[float] | None, z_min_mm: float
):
    """IK only: joints that put the TCP at `xyz_mm`, or None."""
    r = rk.solve_ik(
        np.asarray(xyz_mm, dtype=float) / 1000.0,
        approach,
        seed,
        JOINT_LIMITS,
        z_min=z_min_mm / 1000,
    )
    return r.q if r.success else None


def link_frames(q: Sequence[float], gripper_opening: float) -> dict[str, np.ndarray]:
    """Base-frame pose (4x4, **metres**) of every URDF link, for the 3D view."""
    return rk.link_frames(q, float(np.clip(gripper_opening, 0, 1)) * rk.FINGER_TRAVEL_M)

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
Box = Sequence[float]  # x0, x1, y0, y1, z0, z1, mm
_STEP_MM = 20.0  # spacing of the checked points along the links


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


def arm_points(q: Sequence[float]) -> np.ndarray:
    """Points (N, 3) on the arm, mm: along the chain of joint origins from the shoulder to the
    TCP every ~20 mm, plus rebot_b601's gripper points. The base and link1 are left out: they
    stand on the deck."""
    frames = rk.joint_frames(q)
    chain = np.array([f[:3, 3] for f in frames[1:]]) * 1000.0
    pts = [chain[:1]]
    for a, b in zip(chain, chain[1:], strict=False):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / _STEP_MM)))
        pts.append(a + (b - a) * (np.arange(1, n + 1)[:, None] / n))
    pts.append(rk.link_points(q)[-6:] * 1000.0)
    return np.vstack(pts)


def keep_out_hit(q: Sequence[float], boxes: Sequence[Box], margin_mm: float) -> str | None:
    """Which keep-out box (as text) a point of the arm at `q` is in, grown by `margin_mm`."""
    if not boxes:
        return None
    p = arm_points(q)
    for b in boxes:
        x0, x1, y0, y1, z0, z1 = b
        m = margin_mm
        inside = (
            (p[:, 0] > x0 - m)
            & (p[:, 0] < x1 + m)
            & (p[:, 1] > y0 - m)
            & (p[:, 1] < y1 + m)
            & (p[:, 2] > z0 - m)
            & (p[:, 2] < z1 + m)
        )
        if inside.any():
            x, y, z = p[np.argmax(inside)]
            return f"({x:.0f}, {y:.0f}, {z:.0f}) in keep-out {[round(v) for v in b]}"
    return None


def check_keep_out(wps: np.ndarray, boxes: Sequence[Box], margin_mm: float) -> None:
    """ArmError if any point of the arm enters a keep-out box along the joint path `wps`
    (straight joint-space segments, sampled every ~2°). A start pose already inside is let be,
    so the arm can always move out."""
    if not boxes:
        return
    wps = np.asarray(wps, dtype=float)
    started_inside = keep_out_hit(wps[0], boxes, margin_mm) is not None
    for a, b in zip(wps, wps[1:], strict=False):
        n = max(2, int(np.ceil(np.abs(b - a).max() / np.radians(2.0))) + 1)
        for s in np.linspace(0.0, 1.0, n)[1:]:
            hit = keep_out_hit(a + (b - a) * s, boxes, margin_mm)
            if hit is None:
                started_inside = False
            elif not started_inside:
                raise ArmError(f"path rejected: {hit}")


def tool_yaw(q: Sequence[float]) -> float:
    """The direction the fingers open along (the TCP's y axis) projected on the floor, as an
    angle from +x, rad."""
    y = fk_tcp(q)[:3, 1]
    return float(np.arctan2(y[1], y[0]))


def with_yaw(q: Sequence[float], yaw_rad: float) -> np.ndarray | None:
    """`q` with joint 6 (wrist roll) turned so the fingers open along `yaw_rad` (either way: the
    gripper is symmetric). For a gripper pointing down, joint 6 turns it about the vertical.
    None if joint 6's limits don't allow it."""
    q = np.asarray(q, dtype=float)

    def err(qq: np.ndarray) -> float:
        return (tool_yaw(qq) - yaw_rad + np.pi / 2) % np.pi - np.pi / 2

    d = err(q)
    lo, hi = JOINT_LIMITS[5]
    best = None
    for sign in (-1.0, 1.0):
        for k in (0.0, np.pi, -np.pi):
            cand = q.copy()
            cand[5] = q[5] + sign * d + k
            ok = lo <= cand[5] <= hi and abs(err(cand)) < 1e-3
            if ok and (best is None or abs(cand[5] - q[5]) < abs(best[5] - q[5])):
                best = cand
    return best


def plan_to(
    q0: Sequence[float],
    xyz_mm: Sequence[float],
    approach: Approach = "down",
    *,
    linear: bool = False,
    z_min_mm: float,
    keep_out: Sequence[Box] = (),
    keep_out_margin_mm: float = 0.0,
) -> np.ndarray:
    """Joint waypoints (M, 6) that move the TCP from `q0` to `xyz_mm`.

    `linear`: a straight line in Cartesian space. Raises TargetRejected if IK or the path check
    (floor at `z_min_mm`, the `keep_out` boxes) fails; nothing moves.
    """
    try:
        _, wps, _ = rb.plan_path(
            q0, np.asarray(xyz_mm, dtype=float) / 1000.0, approach, linear, z_min=z_min_mm / 1000.0
        )
        check_keep_out(wps, keep_out, keep_out_margin_mm)
    except (rb.ArmError, ArmError) as e:
        raise TargetRejected(str(e)) from None
    return wps


def plan_joints(
    q0: Sequence[float],
    q1: Sequence[float],
    *,
    z_min_mm: float,
    keep_out: Sequence[Box] = (),
    keep_out_margin_mm: float = 0.0,
) -> np.ndarray:
    """Joint-space waypoints from `q0` to `q1`. ArmError if `q1` is outside the joint limits or
    the path goes below the floor, into the base or into a `keep_out` box."""
    wps = np.array([q0, q1], dtype=float)
    try:
        rb.check_limits(wps[1])
        rb.check_path(wps, z_min=z_min_mm / 1000.0)
    except rb.ArmError as e:
        raise ArmError(str(e)) from None
    check_keep_out(wps, keep_out, keep_out_margin_mm)
    return wps


def path_duration(wps: np.ndarray, speed_scale: float) -> float:
    """Seconds the real arm takes for `wps` at `speed_scale`."""
    return rb.path_duration(np.asarray(wps, dtype=float), speed_scale)


def path_timing(wps: np.ndarray, speed_scale: float) -> tuple[float, float]:
    """(duration, ramp) in seconds of the real arm's profile for `wps` at `speed_scale`."""
    return rb.path_timing(np.asarray(wps, dtype=float), speed_scale)


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

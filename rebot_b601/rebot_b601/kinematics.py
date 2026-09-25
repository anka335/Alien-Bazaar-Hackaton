"""Forward / inverse kinematics of the reBot Arm B601-RS (6 revolute joints).

Geometry comes from the official Seeed URDF ``00-arm-rs_asm-v3.urdf``
(Seeed-Projects/lerobot-robot-seeed-b601, MIT licence; identical arm chain to
the URDF in Seeed-Projects/reBotArm_control_py).  On the real arm the motor
angle equals the URDF joint angle (the Seeed SDK sends URDF joint angles
straight to the motors), so ``q`` below is directly the motor angle in radians
after the zero calibration.

Frames
------
* Base frame: URDF ``base_link``.  +Z up, origin on the base plate; joint 1
  sits 0.075 m above it.  Positions are in metres.
* TCP frame: URDF ``gripper_end``.  Its +X axis is the approach direction of
  the gripper (it points from the wrist towards the fingertips).

IK is a damped-least-squares solver with joint limits, multi-start and null
space regularisation towards the seed.  It has no knowledge of the
environment; see :func:`link_points` / :func:`pose_is_safe` for the (very
rough) table / base checks.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------
# Geometry (from the URDF; xyz [m], rpy [rad], axis sign about local Z)
# --------------------------------------------------------------------------

# (xyz, rpy, axis_sign)  -- one entry per revolute joint, base -> tip
_JOINTS = (
    ((-0.00034283, -0.00098683, 0.075), (0.0, 0.0, 0.0), -1.0),   # joint1
    ((0.020343, 0.027237, 0.07), (-1.5708, 0.0, 0.0), +1.0),      # joint2
    ((-0.236, 0.0, 0.0), (0.0, 0.0, 0.0), -1.0),                  # joint3
    ((0.228, -0.072746, 0.0045), (0.0, 0.0, 0.0), -1.0),          # joint4
    ((0.087, -0.048, -0.03075), (-1.5708, 0.0, 0.0), -1.0),       # joint5
    ((0.0365, 0.0, 0.048), (0.0, 1.5708, 0.0), -1.0),             # joint6
)
# fixed transform link6 -> gripper_end (TCP)
_TCP = ((0.0, 0.0, 0.16621), (3.1416, -1.5708, 0.0))

N_JOINTS = 6

# URDF hard limits [rad]; the driver applies tighter soft limits (config.py)
URDF_LIMITS = np.array(
    [(-2.8, 2.8), (0.0, 3.14), (0.0, 3.14), (-1.57, 1.57), (-1.57, 1.57), (-3.14, 3.14)]
)


def _rpy_to_R(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    # URDF convention: R = Rz(y) @ Ry(p) @ Rx(r)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _T(xyz, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


_ORIGINS = [_T(xyz, _rpy_to_R(rpy)) for xyz, rpy, _ in _JOINTS]
_SIGNS = [s for _, _, s in _JOINTS]
_T_TCP = _T(_TCP[0], _rpy_to_R(_TCP[1]))


# --------------------------------------------------------------------------
# Forward kinematics
# --------------------------------------------------------------------------


def joint_frames(q) -> list[np.ndarray]:
    """4x4 base-frame transforms of joint frames 1..6 (after rotation) and the TCP.

    Returns a list of 7 matrices: joint1..joint6 output frames, then TCP.
    """
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    frames = []
    for i in range(N_JOINTS):
        T = T @ _ORIGINS[i]
        T = T @ _T((0, 0, 0), _rot_z(_SIGNS[i] * q[i]))
        frames.append(T)
    frames.append(T @ _T_TCP)
    return frames


# gripper fingers (URDF joint_left / joint_right): prismatic along local +Z, which maps to -Y / +Y of the TCP frame
_FINGER_L = _T((-0.041939, -7.3385e-05, 0.0), _rpy_to_R((1.5708, -1.5708, 0.0)))
_FINGER_R = _T((-0.041939, 7.3385e-05, 0.0), _rpy_to_R((-1.5708, -1.5708, 0.0)))
FINGER_TRAVEL_M = 0.05      # per-finger travel at full opening (URDF limit of joint_left)

LINK_NAMES = ("base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_end", "gripper_left", "gripper_right")


def link_frames(q, finger_m: float = 0.0) -> dict[str, np.ndarray]:
    """Base-frame 4x4 pose of every URDF link (for rendering the CAD meshes).

    ``finger_m`` is the opening of each finger in metres (0 = closed .. 0.05 = open).
    """
    fr = joint_frames(q)
    out = {"base_link": np.eye(4)}
    for i in range(N_JOINTS):
        out[f"link{i + 1}"] = fr[i]
    out["gripper_end"] = fr[N_JOINTS]
    slide = _T((0.0, 0.0, finger_m), np.eye(3))
    out["gripper_left"] = fr[N_JOINTS] @ _FINGER_L @ slide
    out["gripper_right"] = fr[N_JOINTS] @ _FINGER_R @ slide
    return out


def fk(q) -> tuple[np.ndarray, np.ndarray]:
    """TCP position (3,) [m] and rotation matrix (3,3) in the base frame."""
    T = joint_frames(q)[-1]
    return T[:3, 3].copy(), T[:3, :3].copy()


def approach_vector(q) -> np.ndarray:
    """Unit vector along the gripper approach direction (TCP +X) in the base frame."""
    return fk(q)[1][:, 0]


def jacobian(q) -> np.ndarray:
    """6x6 geometric Jacobian of the TCP: rows 0-2 linear, rows 3-5 angular (base frame)."""
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    axes, origins = [], []
    for i in range(N_JOINTS):
        T = T @ _ORIGINS[i]
        axes.append(T[:3, 2] * _SIGNS[i])
        origins.append(T[:3, 3].copy())
        T = T @ _T((0, 0, 0), _rot_z(_SIGNS[i] * q[i]))
    p_tcp = (T @ _T_TCP)[:3, 3]
    J = np.zeros((6, N_JOINTS))
    for i in range(N_JOINTS):
        J[:3, i] = np.cross(axes[i], p_tcp - origins[i])
        J[3:, i] = axes[i]
    return J


def _fk_jac(q: np.ndarray):
    """One pass FK + Jacobian: returns (tcp position, tcp rotation, 6x6 Jacobian)."""
    T = np.eye(4)
    axes = np.empty((N_JOINTS, 3))
    origins = np.empty((N_JOINTS, 3))
    for i in range(N_JOINTS):
        T = T @ _ORIGINS[i]
        axes[i] = T[:3, 2] * _SIGNS[i]
        origins[i] = T[:3, 3]
        ang = _SIGNS[i] * q[i]
        c, s = np.cos(ang), np.sin(ang)
        Rz = np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        T = T @ Rz
    T = T @ _T_TCP
    p = T[:3, 3]
    J = np.empty((6, N_JOINTS))
    for i in range(N_JOINTS):
        J[:3, i] = np.cross(axes[i], p - origins[i])
        J[3:, i] = axes[i]
    return p.copy(), T[:3, :3].copy(), J


# --------------------------------------------------------------------------
# Rough geometric sanity checks (no environment model!)
# --------------------------------------------------------------------------

# Extra points on the tool, in the TCP frame (gripper body / fingers), used
# for the table-clearance check.  The gripper body extends from the wrist to
# the TCP along -X of the TCP frame; fingers are ~4 cm behind the TCP.
_TOOL_POINTS_TCP = np.array(
    [
        [0.0, 0.0, 0.0],
        [-0.04, 0.0, 0.0],
        [-0.08, 0.0, 0.0],
        [-0.12, 0.0, 0.0],
        [-0.04, 0.03, 0.0],
        [-0.04, -0.03, 0.0],
    ]
)


def link_points(q) -> np.ndarray:
    """Points (N,3) on the arm in the base frame: joint origins, mid-links and tool."""
    frames = joint_frames(q)
    pts = [f[:3, 3] for f in frames[:-1]]
    # mid points of the two long links (upper arm, forearm)
    pts.append(0.5 * (frames[1][:3, 3] + frames[2][:3, 3]))
    pts.append(0.5 * (frames[2][:3, 3] + frames[3][:3, 3]))
    T_tcp = frames[-1]
    tool = (T_tcp[:3, :3] @ _TOOL_POINTS_TCP.T).T + T_tcp[:3, 3]
    return np.vstack([np.array(pts), tool])


def pose_is_safe(q, z_min: float = 0.02, base_radius: float = 0.10, base_height: float = 0.22) -> tuple[bool, str]:
    """Heuristic check: nothing below ``z_min`` (table) and the tool not inside the base column.

    Only a rough guard; it does not know about objects on the table.
    """
    pts = link_points(q)
    # the first joint origins sit on the base itself, exclude the first three points
    check = pts[3:]
    lowest = float(check[:, 2].min())
    if lowest < z_min:
        return False, f"part of the arm would go below z={z_min:.3f} m (lowest point z={lowest:.3f} m)"
    tool = pts[-6:]
    for p in tool:
        if np.hypot(p[0], p[1]) < base_radius and p[2] < base_height:
            return False, "tool would hit the base column"
    return True, "ok"


# --------------------------------------------------------------------------
# Inverse kinematics
# --------------------------------------------------------------------------

APPROACH_PRESETS = {
    "down": (0.0, 0.0, -1.0),
    "up": (0.0, 0.0, 1.0),
    "forward": (1.0, 0.0, 0.0),
}


def parse_approach(approach) -> np.ndarray | None:
    """Turn ``None`` / 'free' / preset name / 3-vector into a unit vector or ``None`` (free)."""
    if approach is None:
        return None
    if isinstance(approach, str):
        key = approach.strip().lower()
        if key in ("free", "none", ""):
            return None
        if key not in APPROACH_PRESETS:
            raise ValueError(f"unknown approach '{approach}', use one of free/{'/'.join(APPROACH_PRESETS)} or [x,y,z]")
        v = np.array(APPROACH_PRESETS[key], dtype=float)
    else:
        v = np.asarray(approach, dtype=float).reshape(-1)
        if v.shape != (3,):
            raise ValueError("approach vector must have 3 components")
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("approach vector must be non-zero")
    return v / n


@dataclass
class IKResult:
    success: bool
    q: np.ndarray                      # best joint solution [rad] (valid even if not success)
    pos_error: float                   # [m]
    approach_error_deg: float | None   # None when the approach is free
    iterations: int = 0
    message: str = ""
    info: dict = field(default_factory=dict)


def _solve_from(q0, target, approach, lo, hi, q_ref, max_iter, tol_pos, tol_ang, w_ang, null_gain):
    """Levenberg-Marquardt (Sugihara) iterations from ``q0``; returns (q, pos_err, approach_err_deg, iters)."""
    q = np.clip(q0, lo, hi).astype(float)
    it = 0
    for it in range(1, max_iter + 1):
        p, R, J = _fk_jac(q)
        e_pos = target - p
        if approach is None:
            e_dir, ang = None, 0.0
        else:
            a = R[:, 0]
            e_dir = np.cross(a, approach)
            ang = float(np.arccos(np.clip(a @ approach, -1.0, 1.0)))
            if a @ approach < 0 and np.linalg.norm(e_dir) < 1e-3:
                # exactly opposite: pick any perpendicular rotation axis to get out
                e_dir = np.cross(a, np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0]))
            else:
                # e_dir has length sin(angle); rescale to the angle for large errors
                n = np.linalg.norm(e_dir)
                if n > 1e-9:
                    e_dir = e_dir / n * ang
        if np.linalg.norm(e_pos) < tol_pos and ang < tol_ang:
            break
        if e_dir is None:
            Jt, err = J[:3], e_pos
        else:
            Jt = np.vstack([J[:3], w_ang * J[3:]])
            err = np.concatenate([e_pos, w_ang * e_dir])
        lam2 = 0.5 * float(err @ err) + 1e-7
        Jp = Jt.T @ np.linalg.inv(Jt @ Jt.T + lam2 * np.eye(Jt.shape[0]))
        dq = Jp @ err
        dq += (np.eye(N_JOINTS) - Jp @ Jt) @ (null_gain * (q_ref - q))
        step = np.max(np.abs(dq))
        if step > 0.4:
            dq *= 0.4 / step
        q = np.clip(q + dq, lo, hi)
    p, R, _ = _fk_jac(q)
    ang_deg = None if approach is None else float(np.degrees(np.arccos(np.clip(R[:, 0] @ approach, -1.0, 1.0))))
    return q, float(np.linalg.norm(target - p)), ang_deg, it


def solve_ik(
    target_xyz,
    approach=None,
    q_seed=None,
    limits: np.ndarray | None = None,
    *,
    tol_pos: float = 1e-3,
    tol_approach_deg: float = 3.0,
    max_iter: int = 150,
    n_random_starts: int = 10,
    z_min: float | None = 0.02,
    rng_seed: int = 0,
) -> IKResult:
    """Solve for joint angles that put the TCP at ``target_xyz`` [m] (base frame).

    ``approach``: None/'free' (position only), 'down'/'up'/'forward' or a 3-vector
    giving the desired direction of the gripper approach axis; rotation about that
    axis is left free.  The solution closest to ``q_seed`` (joint space) among the
    successful ones is returned.  ``z_min`` enables the rough table-clearance check
    (set ``None`` to disable).
    """
    target = np.asarray(target_xyz, dtype=float).reshape(3)
    app = parse_approach(approach)
    lim = URDF_LIMITS if limits is None else np.asarray(limits, dtype=float)
    lo, hi = lim[:, 0], lim[:, 1]
    seed = np.zeros(N_JOINTS) if q_seed is None else np.clip(np.asarray(q_seed, dtype=float), lo, hi)

    rng = np.random.default_rng(rng_seed)
    starts = [seed, np.clip(np.array([0.0, 1.2, 1.5, 0.0, 0.0, 0.0]), lo, hi)]
    starts += [lo + (hi - lo) * rng.random(N_JOINTS) for _ in range(n_random_starts)]

    tol_ang = np.radians(tol_approach_deg) * 0.5   # solve tighter than we accept
    best_ok: tuple[float, np.ndarray, float, float | None, int] | None = None
    best_any: tuple[float, np.ndarray, float, float | None, int] | None = None
    rejected_unsafe = 0
    for idx, q0 in enumerate(starts):
        q, ep, ea, it = _solve_from(
            q0, target, app, lo, hi, seed, max_iter, tol_pos * 0.3, tol_ang, w_ang=0.5, null_gain=0.05
        )
        ok = ep <= tol_pos and (ea is None or ea <= tol_approach_deg)
        cost = float(np.linalg.norm((q - seed)))
        cand = (cost, q, ep, ea, it)
        if best_any is None or ep + (0 if ea is None else np.radians(ea) * 0.1) < best_any[2] + (0 if best_any[3] is None else np.radians(best_any[3]) * 0.1):
            best_any = cand
        if ok:
            if z_min is not None and not pose_is_safe(q, z_min=z_min)[0]:
                rejected_unsafe += 1
                continue
            if best_ok is None or cost < best_ok[0]:
                best_ok = cand
        # the seed start usually gives the closest solution; stop early when it succeeded
        if idx == 0 and ok and best_ok is not None:
            break

    if best_ok is not None:
        _, q, ep, ea, it = best_ok
        return IKResult(True, q, ep, ea, it, "ok")

    _, q, ep, ea, it = best_any  # type: ignore[misc]
    if rejected_unsafe:
        msg = "reachable only through poses that go below the table clearance / into the base"
    else:
        msg = f"no solution within limits (best position error {ep * 1000:.1f} mm" + (
            f", approach error {ea:.1f} deg)" if ea is not None else ")"
        )
    return IKResult(False, q, ep, ea, it, msg, {"rejected_unsafe": rejected_unsafe})

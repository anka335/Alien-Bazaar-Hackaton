"""Hand-eye calibration from tape marks on the mat, no printed board (the /calibrate page, D-021).

The arm points its tip at a spot on the mat and a tape mark goes under it: the mark's arm-frame
position is known from FK. From any pose, the mark clicked in the image plus its depth gives its
camera-frame position. Each click i gives p_base_i = F_i · X · p_cam_i (F_i = T_base_flange at
that pose, X = T_flange_cam), so F_i⁻¹ · p_base_i = X · p_cam_i: X is the rigid fit (Kabsch) of
the camera points onto the marks in the flange frame, over clicks from any poses.
"""

from __future__ import annotations

import dataclasses
import datetime as dt
import itertools
from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np

from sorter.calibration.calibration import _undistort
from sorter.core.types import Frame, Intrinsics, Pose

MIN_MARKS = 3
DEPTH_WINDOW_PX = 4  # the median depth of (2w+1)² pixels around the click
SNAP_RADIUS_PX = 35  # a click snaps to the center of a tape square this close
TAPE_MM = 10.0  # the side of a tape mark
# around the mat center, arm frame (+x away from the arm, +y left); not symmetric, so a view
# of a few of them is never ambiguous
MARK_OFFSETS_MM = {
    "M1": (0.0, 0.0),
    "M2": (70.0, 50.0),
    "M3": (70.0, -50.0),
    "M4": (-70.0, -50.0),
    "M5": (-70.0, 50.0),
    "M6": (35.0, 0.0),
}


# The pattern is symmetric about the line M1-M6: a view clicked with these swapped fits as well,
# with the camera flipped. `fit_mount` with a `prior` tries both per view.
MIRROR = {"M2": "M3", "M3": "M2", "M4": "M5", "M5": "M4"}
PLAUSIBLE_DEG = 45.0  # a fit whose optical axis is this far from the nominal one is wrong
PLAUSIBLE_MM = 200.0  # or its camera this far from the nominal position


@dataclass(frozen=True)
class Mark:
    name: str
    xyz: tuple[float, float, float]  # mm, arm frame, on the mat


def marks(center_xy: Sequence[float], z_mm: float) -> list[Mark]:
    cx, cy = center_xy
    return [Mark(n, (cx + dx, cy + dy, z_mm)) for n, (dx, dy) in MARK_OFFSETS_MM.items()]


@dataclass(frozen=True)
class Click:
    """A mark seen in the image: `T_base_flange` when it was clicked, the camera-frame point."""

    mark: str
    p_base: tuple[float, float, float]  # mm, arm frame
    T_base_flange: Pose
    p_cam: tuple[float, float, float]  # mm, camera optical frame
    px: tuple[float, float]


@dataclass(frozen=True)
class MountFit:
    T_flange_cam: Pose
    rmse_mm: float
    residuals_mm: list[float]  # per click: |fit - mark|


def deproject(frame: Frame, u: float, v: float) -> tuple[float, float, float] | None:
    """The camera-frame point (mm) at pixel (u, v), or None without depth around it."""
    w = DEPTH_WINDOW_PX
    iu, iv = int(round(u)), int(round(v))
    patch = frame.depth_mm[max(iv - w, 0) : iv + w + 1, max(iu - w, 0) : iu + w + 1]
    z = patch[patch > 0]
    if z.size < patch.size // 3:
        return None
    Z = float(np.median(z))
    x, y = _undistort(frame.intrinsics, u, v)
    return (x * Z, y * Z, Z)


def snap(color: np.ndarray, u: float, v: float, radius: int = SNAP_RADIUS_PX):
    """The center of the dark tape square at or near pixel (u, v), or None if there is no
    compact dark blob within `radius`."""
    h, w = color.shape[:2]
    x0, y0 = max(int(u) - radius, 0), max(int(v) - radius, 0)
    x1, y1 = min(int(u) + radius + 1, w), min(int(v) + radius + 1, h)
    gray = cv2.cvtColor(np.ascontiguousarray(color[y0:y1, x0:x1]), cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    lo, hi = np.percentile(gray, 5), np.percentile(gray, 60)
    if hi - lo < 12:  # no contrast: no mark here
        return None
    n, labels, stats, cents = cv2.connectedComponentsWithStats(
        (gray < (lo + hi) / 2).astype(np.uint8)
    )
    best = None
    for i in range(1, n):
        bx, by, bw, bh, area = stats[i]
        touches = bx == 0 or by == 0 or bx + bw == x1 - x0 or by + bh == y1 - y0
        if area < 15 or touches or area < 0.4 * bw * bh or max(bw, bh) > 2.5 * min(bw, bh):
            continue  # a speck, cut by the window, or not square-ish
        cx, cy = cents[i][0] + x0, cents[i][1] + y0
        d = np.hypot(cx - u, cy - v)
        if best is None or d < best[0]:
            best = (d, float(cx), float(cy))
    return None if best is None else (best[1], best[2])


def find_squares(frame: Frame, side_mm: float = TAPE_MM) -> list[tuple[float, float]]:
    """Centers (px) of dark, square-ish blobs about `side_mm` wide (at their depth) with depth:
    candidate tape marks. Shadows, the box edge and other dark things may be among them."""
    gray = cv2.cvtColor(np.ascontiguousarray(frame.color), cv2.COLOR_BGR2GRAY).astype(np.float32)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    background = cv2.blur(gray, (61, 61))
    dark = ((gray < background * 0.65) & (frame.depth_mm > 0)).astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, stats, cents = cv2.connectedComponentsWithStats(dark)
    k = frame.intrinsics
    out = []
    for i in range(1, n):
        bx, by, bw, bh, area = stats[i]
        u, v = cents[i]
        patch = frame.depth_mm[by : by + bh, bx : bx + bw]
        if not (patch > 0).any():
            continue
        side = k.fx * side_mm / float(np.median(patch[patch > 0]))  # the expected side in px
        if not (0.25 * side**2 <= area <= 4.0 * side**2):
            continue
        if area < 0.45 * bw * bh or max(bw, bh) > 2.2 * min(bw, bh):
            continue
        out.append((float(u), float(v)))
    return out


def identify(
    cam: np.ndarray, positions: dict[str, tuple[float, float, float]], tol_mm: float = 8.0
) -> tuple[dict[int, str], Pose] | None:
    """Which mark each camera-frame point (N x 3, mm) is, from the distances between them alone
    (no mount needed): {point index: mark name} and T_base_cam, or None if 3 don't match.

    The marks lie on a plane, so a mirrored match fits as well with the camera flipped to
    below the table; only the one with the camera above it, looking down, is kept."""
    names = list(positions)
    P = np.array([positions[n] for n in names], dtype=float)
    cam = np.asarray(cam, dtype=float).reshape(-1, 3)
    if len(cam) < 3:
        return None
    Dc = np.linalg.norm(cam[:, None] - cam[None], axis=2)
    Dm = np.linalg.norm(P[:, None] - P[None], axis=2)
    tc = np.array([t for t in itertools.permutations(range(len(cam)), 3)])
    tm = np.array([t for t in itertools.permutations(range(len(P)), 3)])
    pairs = ((0, 1), (1, 2), (0, 2))
    err = sum(
        np.abs(Dc[tc[:, a], tc[:, b]][:, None] - Dm[tm[:, a], tm[:, b]][None, :]) for a, b in pairs
    )
    best = None
    for ic, im in zip(*np.nonzero(err < tol_mm), strict=True):
        T = _kabsch(cam[tc[ic]], P[tm[im]])  # camera → base
        if T[2, 3] < 50.0 or T[2, 2] > -0.3:  # the camera below the table, or not looking down
            continue
        mapped = cam @ T[:3, :3].T + T[:3, 3]
        d = np.linalg.norm(mapped[:, None] - P[None], axis=2)
        match: dict[int, int] = {}
        for i in np.argsort(d.min(axis=1)):
            j = int(np.argmin(d[i]))
            if d[i, j] < tol_mm and j not in match.values():
                match[int(i)] = j
        if len(match) < 3:
            continue
        idx = list(match)
        T = _kabsch(cam[idx], P[[match[i] for i in idx]])
        res = np.linalg.norm(cam[idx] @ T[:3, :3].T + T[:3, 3] - P[[match[i] for i in idx]], axis=1)
        key = (len(match), -float(np.sqrt(np.mean(res**2))))
        if best is None or key > best[0]:
            best = (key, {i: names[j] for i, j in match.items()}, T)
    return None if best is None else (best[1], best[2])


def _kabsch(src: np.ndarray, dst: np.ndarray) -> Pose:
    """The rigid T (4x4) minimizing Σ |T·src_i − dst_i|²."""
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, md - R @ ms
    return T


def _fit(clicks: Sequence[Click]) -> MountFit | None:
    if len({c.mark for c in clicks}) < MIN_MARKS:
        return None
    cam = np.array([c.p_cam for c in clicks], dtype=float)
    flange = np.array(
        [(np.linalg.inv(c.T_base_flange) @ np.array([*c.p_base, 1.0]))[:3] for c in clicks]
    )
    s = np.linalg.svd(cam - cam.mean(axis=0), compute_uv=False)
    if s[1] < 10.0:  # mm: the points are (nearly) on a line, the turn about it is unknown
        return None
    X = _kabsch(cam, flange)
    res = np.linalg.norm((cam @ X[:3, :3].T + X[:3, 3]) - flange, axis=1)
    return MountFit(X, float(np.sqrt(np.mean(res**2))), [float(r) for r in res])


def view_rmse(clicks: Sequence[Click]) -> float | None:
    """How well the clicks of one view agree with the marks' layout, with no arm FK (mm): the
    rigid fit of their camera points onto the marks. Large: a wrong label, bad depth or a tape
    off its spot; small in every view while the mount fit is poor: the arm's pose (FK) differs
    between the views. None with fewer than 3 marks or all in a line."""
    if len({c.mark for c in clicks}) < MIN_MARKS:
        return None
    cam = np.array([c.p_cam for c in clicks], dtype=float)
    base = np.array([c.p_base for c in clicks], dtype=float)
    if np.linalg.svd(cam - cam.mean(axis=0), compute_uv=False)[1] < 10.0:
        return None
    T = _kabsch(cam, base)
    res = np.linalg.norm(cam @ T[:3, :3].T + T[:3, 3] - base, axis=1)
    return float(np.sqrt(np.mean(res**2)))


def project(
    T_base_cam: Pose, k: Intrinsics, points: Sequence[Sequence[float]]
) -> list[tuple[float, float] | None]:
    """Pixels of arm-frame points (mm); None behind the camera. Not clipped to the image."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    c = (np.linalg.inv(T_base_cam) @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
    K = np.array([[k.fx, 0, k.cx], [0, k.fy, k.cy], [0, 0, 1]])
    uv, _ = cv2.projectPoints(c, np.zeros(3), np.zeros(3), K, np.array(k.coeffs or [0.0] * 5))
    return [
        (float(u), float(v)) if z > 1.0 else None
        for (u, v), z in zip(uv.reshape(-1, 2), c[:, 2], strict=True)
    ]


def mount_change(A: Pose, B: Pose) -> tuple[float, float]:
    """How far mount B is from mount A: (mm, degrees)."""
    dR = np.asarray(A)[:3, :3].T @ np.asarray(B)[:3, :3]
    angle = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)))
    return float(np.linalg.norm(np.asarray(A)[:3, 3] - np.asarray(B)[:3, 3])), float(angle)


def hand_eye_result(fit: MountFit, n_marks: int, serial: str = "") -> dict:
    """`calibration.hand_eye` for config/hand_eye.yaml (see `HandEyeResult`)."""
    return {
        "T_flange_cam": [[round(float(v), 6) for v in row] for row in fit.T_flange_cam],
        "rmse_mm": round(fit.rmse_mm, 2),
        "method": f"marks ({n_marks} marks, {len(fit.residuals_mm)} clicks)",
        "camera_serial": serial,
        "created": dt.datetime.now().isoformat(timespec="seconds"),
    }


def mirrored(c: Click, positions: dict[str, tuple[float, float, float]]) -> Click:
    """The click relabeled as its mirror twin (M2↔M3, M4↔M5)."""
    name = MIRROR.get(c.mark, c.mark)
    return dataclasses.replace(c, mark=name, p_base=positions[name])


def _looks_down(fit: MountFit, clicks: Sequence[Click]) -> bool:
    """From every click's pose the fitted camera is above the marks and looks down at them: a
    view clicked mirrored puts it under the table (the pattern is flat)."""
    z_marks = max(c.p_base[2] for c in clicks)
    for c in clicks:
        T = np.asarray(c.T_base_flange) @ fit.T_flange_cam
        if T[2, 3] < z_marks + 50.0 or T[2, 2] > -0.3:
            return False
    return True


def fit_mount(
    clicks: Sequence[Click],
    views: Sequence[int] | None = None,
    positions: dict[str, tuple[float, float, float]] | None = None,
) -> tuple[MountFit | None, list[int]]:
    """T_flange_cam from the clicks, or None with fewer than 3 marks or all in a line; and the
    views whose marks were mirrored (M2↔M3, M4↔M5) and are fixed in the fit.

    With `views` (the view of each click) and `positions` (every mark's position), each view is
    tried as clicked and mirrored: a mirrored view puts the camera under the table, so the fit
    with the camera above it in every view wins, then the lowest RMSE. No prior mount is
    needed, so a camera turned any way about its axis is found."""
    if views is None or positions is None:
        return _fit(clicks), []
    ids = sorted(set(views))
    best = None
    for flips in itertools.product((False, True), repeat=len(ids)):
        flipped = {v for v, f in zip(ids, flips, strict=True) if f}
        cand = [
            mirrored(c, positions) if v in flipped else c
            for c, v in zip(clicks, views, strict=True)
        ]
        fit = _fit(cand)
        if fit is None:
            continue
        key = (not _looks_down(fit, cand), fit.rmse_mm, len(flipped))
        if best is None or key < best[0]:
            best = (key, fit, sorted(flipped))
    return (None, []) if best is None else (best[1], best[2])


def plausible(nominal: Pose, T: Pose) -> bool:
    """T_flange_cam near the nominal mount in position and viewing direction; the turn about
    the optical axis is free (the camera may be mounted any way round)."""
    nominal, T = np.asarray(nominal), np.asarray(T)
    axis = np.degrees(np.arccos(np.clip(nominal[:3, 2] @ T[:3, 2], -1.0, 1.0)))
    return bool(np.linalg.norm(nominal[:3, 3] - T[:3, 3]) <= PLAUSIBLE_MM and axis <= PLAUSIBLE_DEG)

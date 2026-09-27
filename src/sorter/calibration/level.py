"""Measure how the arm's base leans from the floor its camera saw.

`python -m sorter.calibration.level data/runs/<run_id> [...]` fits a plane to the floor in the
scan views of the runs (`*sense_floor*.npz`; socks, the rover and depth noise are dropped as
outliers) and prints the `arm.base_tilt_deg` and `sim.layout.floor_z_mm` that make it level.
The runs must come from one setup (the rover standing the same way); each run's own
`arm.base_yaw_deg` / `base_tilt_deg` (its `run.json`) is taken into account, so running it again
on runs made with the new values prints about the same values.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from rebot_b601 import kinematics as rk

from sorter.core.io import load_observation


def _base(yaw_deg: float, tilt_deg) -> np.ndarray:
    rk.set_base(math.radians(yaw_deg), *(math.radians(a) for a in tilt_deg))
    return rk.base_rotation()


def floor_points(run: Path, step: int = 15) -> tuple[np.ndarray, float, np.ndarray]:
    """The run's floor-view depth points (N, 3) in the frame the run used, its base yaw, and the
    base rotation it used."""
    arm = json.loads((run / "run.json").read_text()).get("config", {}).get("arm", {})
    yaw = float(arm.get("base_yaw_deg", 0.0))
    R = _base(yaw, arm.get("base_tilt_deg", (0.0, 0.0)))
    pts = []
    for f in sorted(run.glob("*sense_floor*.npz")):
        o = load_observation(f.with_suffix(""))
        if o.T_base_cam is None:
            continue
        d = o.frame.depth_mm.astype(float)
        k = o.frame.intrinsics
        v, u = np.nonzero(d > 0)
        v, u = v[::step], u[::step]
        z = d[v, u]
        cam = np.stack([(u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z, np.ones_like(z)])
        pts.append((o.T_base_cam @ cam)[:3].T)
    return (np.vstack(pts) if pts else np.zeros((0, 3))), yaw, R


def fit_plane(P: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    """z = a·x + b·y + c by least squares, dropping outliers in rounds: ((a, b), c, inliers)."""
    keep = P[:, 2] < -100  # the floor, not the rover or the socks' tops high up
    for tol in (30, 20, 20, 10, 10, 10):
        A = np.c_[P[keep, :2], np.ones(keep.sum())]
        (a, b, c), *_ = np.linalg.lstsq(A, P[keep, 2], rcond=None)
        keep = np.abs(P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)) < tol
    return np.array([a, b]), float(c), keep


def level(runs: list[Path]) -> dict:
    """The tilt (about x, then y, deg) and floor height (mm) that make the runs' floor level."""
    yaws, Rs, P = [], [], []
    for run in runs:
        pts, yaw, R_used = floor_points(run)
        # the points in the base_link frame, then in the frame of the first run's base
        yaws.append(yaw)
        Rs.append(R_used)
        P.append(pts @ R_used)  # R_used.T @ p for each row: base_link coordinates
    if len(set(yaws)) > 1:
        raise SystemExit(f"the runs used different base yaws: {sorted(set(yaws))}")
    R0 = Rs[0]
    P = np.vstack(P) @ R0.T  # all in the first run's frame
    (a, b), c, keep = fit_plane(P)
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    # the smallest rotation taking the floor's normal to +z
    axis = np.cross(n, [0.0, 0.0, 1.0])
    s, cth = np.linalg.norm(axis), float(n[2])
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    K = K / s if s > 1e-9 else K
    R_corr = np.eye(3) + s * K + (1 - cth) * (K @ K)
    M = R_corr @ R0 @ rk._rot_z(math.radians(yaws[0])).T  # = Ry(pitch) · Rx(roll)
    roll = math.degrees(math.atan2(-M[1, 2], M[1, 1]))
    pitch = math.degrees(math.atan2(-M[2, 0], M[0, 0]))
    floor = P[keep] @ R_corr.T
    resid = P[keep, 2] - (a * P[keep, 0] + b * P[keep, 1] + c)
    return {
        "base_tilt_deg": [round(roll, 2), round(pitch, 2)],
        "floor_z_mm": round(float(np.median(floor[:, 2])), 1),
        "points": int(keep.sum()),
        "rms_mm": round(float(np.sqrt(np.mean(resid**2))), 1),
        "seen_slope_deg": [round(math.degrees(math.atan(v)), 2) for v in (a, b)],
    }


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(prog="python -m sorter.calibration.level", description=__doc__)
    p.add_argument("runs", nargs="+", type=Path, help="data/runs/<run_id> directories")
    r = level(p.parse_args(argv).runs)
    print(
        f"# floor fit: {r['points']} points, rms {r['rms_mm']} mm; the floor seen rose "
        f"{r['seen_slope_deg'][0]}° along x, {r['seen_slope_deg'][1]}° along y"
    )
    print(f"arm:\n  base_tilt_deg: {r['base_tilt_deg']}")
    print(f"sim:\n  layout:\n    floor_z_mm: {r['floor_z_mm']}")


if __name__ == "__main__":
    main()

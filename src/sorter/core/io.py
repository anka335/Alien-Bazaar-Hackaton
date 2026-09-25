"""Recording format: one .npz per observation, plus a .png of the color image for browsing."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from sorter.core.types import Frame, Intrinsics, Observation, Zone


def save_observation(path: str | Path, obs: Observation) -> Path:
    """Write `<path>.npz` and `<path>.png`. Returns the .npz path."""
    path = Path(path).with_suffix(".npz")
    path.parent.mkdir(parents=True, exist_ok=True)
    f, k = obs.frame, obs.frame.intrinsics
    np.savez_compressed(
        path,
        color=f.color,
        depth_mm=f.depth_mm,
        intrinsics=np.array([k.fx, k.fy, k.cx, k.cy, k.width, k.height], dtype=np.float64),
        coeffs=np.array(k.coeffs, dtype=np.float64),
        timestamp=np.float64(f.timestamp),
        seq=np.int64(f.seq),
        zone=np.str_(obs.zone.value),
        T_base_cam=np.empty(0) if obs.T_base_cam is None else obs.T_base_cam,
        joints=np.empty(0) if obs.joints is None else np.array(obs.joints, dtype=np.float64),
    )
    cv2.imwrite(str(path.with_suffix(".png")), f.color)
    return path


def load_observation(path: str | Path) -> Observation:
    with np.load(Path(path).with_suffix(".npz"), allow_pickle=False) as d:
        fx, fy, cx, cy, width, height = d["intrinsics"].tolist()
        intrinsics = Intrinsics(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=int(width),
            height=int(height),
            coeffs=tuple(d["coeffs"].tolist()),
        )
        frame = Frame(
            color=d["color"],
            depth_mm=d["depth_mm"],
            intrinsics=intrinsics,
            timestamp=float(d["timestamp"]),
            seq=int(d["seq"]),
        )
        T = d["T_base_cam"]
        joints = d["joints"]
        return Observation(
            frame=frame,
            zone=Zone(str(d["zone"])),
            T_base_cam=T if T.size else None,
            joints=tuple(joints.tolist()) if joints.size else None,
        )

"""What the gripper holds, seen by the wrist camera.

From the `show_held` pose (`show_pose`) a sock hanging from the fingers is in the image, closer
than anything behind it: the floor in front of the rover is far away. So the held
sock is the segmented instance whose pixels are mostly closer than `NEAR_MM` or have no depth at
all (the D435i sees nothing closer than ~175 mm). Its color is block 4's statistics over it.
Nothing held there: the pick caught nothing.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from sorter.box_detector.cargo import Segment
from sorter.box_detector.geometry import points
from sorter.color_classifier.classifier import color_stats, decide
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.core.types import ColorClass, Observation, Overlay

SHOW_POSE = "show_held"  # the named pose (rig) where the camera sees what the gripper holds
HANG_MM = (
    60.0,
    80.0,
    100.0,
)  # below the fingers: the rig camera sees no closer (it is off to the side)
NEAR_MM = 330.0  # held cloth is closer than this (or too close for depth)
MIN_HELD_PX = 1200
HELD_ABOVE_FLOOR_MM = 90.0
# a sock hanging from the gripper is seen from the side, in its own shade: the box's thresholds
# don't hold. What does, on the sim (to check on the rig): colored socks keep their chroma
# (31–39, the others ≤ 11); light ones stay at L* 34–45, dark ones at 5–8
SIDE_CHROMA_COLORED = 20.0
SIDE_L_LIGHT = 28.0
SIDE_L_DARK = 18.0  # cloth higher than this over the floor hangs from the gripper


@dataclass
class HeldView:
    color: ColorClass | None  # None: nothing held
    confidence: float
    area_px: int
    overlay: Overlay
    count: int = 0  # socks seen hanging: more than one, a neighbor came along
    stats: dict[str, float] | None = None  # color statistics of the held sock (side-lit)
    classes: list[ColorClass | None] | None = None  # each hanging sock's side-lit class


def find_held(
    obs: Observation, segment: Segment, classifier: ColorClassifierConfig, floor_z_mm: float
) -> HeldView:
    """The sock hanging from the gripper in `obs` (from `show_held`): a segmented instance
    mostly too close for depth or high above the floor. Socks lying in a bin in view are not."""
    frame = obs.frame
    p = points(obs)
    with np.errstate(invalid="ignore"):
        up = np.isnan(p[..., 2]) | (p[..., 2] > floor_z_mm + HELD_ABOVE_FLOOR_MM)
    near = (frame.depth_mm == 0) | (frame.depth_mm < NEAR_MM)
    held = near & up
    best = None
    count = 0
    classes: list[ColorClass | None] = []
    for inst in segment(frame.color):
        m = inst.mask
        area = int(m.sum())
        if area < MIN_HELD_PX or held[m].mean() < 0.6:
            continue
        count += 1
        classes.append(side_class(color_stats(frame.color, m, classifier.erode_px)))
        if best is None or area > best[1]:
            best = (m, area)
    if best is None:
        return HeldView(None, 0.0, 0, Overlay(text=["nothing in the gripper"]))
    m, area = best
    stats = color_stats(frame.color, m, classifier.erode_px)
    color, conf = decide(stats, classifier)
    text = f"holding a {color} sock ({conf:.2f}), {area} px, {count} in view"
    return HeldView(color, conf, area, Overlay(mask=m, text=[text]), count, stats, classes)


def side_class(stats: dict[str, float] | None) -> ColorClass | None:
    """The class of a sock seen hanging (side-lit), if clear; None in between."""
    if not stats:
        return None
    if stats["chroma"] >= SIDE_CHROMA_COLORED:
        return ColorClass.COLORED
    if stats["L"] >= SIDE_L_LIGHT:
        return ColorClass.LIGHT
    if stats["L"] <= SIDE_L_DARK:
        return ColorClass.DARK
    return None


def show_pose(
    home: np.ndarray,
    T_link5_cam: np.ndarray,
    keep_out: list,
    keep_out_margin_mm: float,
    z_min_mm: float,
    front_x_mm: float,
    floor_z_mm: float,
    focal_px: float,
    size: tuple[int, int],
    samples: int = 60000,
    seed: int = 0,
    link5_points: list | tuple = (),
    collides: Callable[[np.ndarray], bool] | None = None,
) -> np.ndarray | None:
    """Joints that show the camera what hangs from the gripper: `HANG_MM` below the fingers in
    the image, the floor behind it far away, the gripper in front of the rover (`front_x_mm`),
    nothing in the keep-out (the camera's body, `link5_points`, included). The camera is off to
    the side of the fingers (the rig's mount doesn't see the fingertips at all), so this is found
    by a search around `home`, the closest to it that shows the sock near the image middle.
    `collides(q)`: a full collision check (the arm with itself and the scene), of the pose and
    the straight way to it from `home`."""
    from sorter.arm import kinematics as kin

    w, h = size
    lim = kin.JOINT_LIMITS
    rng = np.random.default_rng(seed)
    found: list[tuple[float, np.ndarray]] = []
    for _ in range(samples):
        q = home.copy()
        q[0] = home[0] + rng.uniform(-0.6, 0.6)
        q[1:5] = home[1:5] + rng.uniform(-2.2, 2.2, 4)
        q = np.clip(q, lim[:, 0] + 0.03, lim[:, 1] - 0.03)
        tcp = kin.fk_tcp(q)[:3, 3]
        if not (40 <= tcp[2] <= 320) or tcp[0] < front_x_mm + 40:
            continue
        cam = kin.fk_link5(q) @ T_link5_cam
        axis, origin = cam[:3, 2], cam[:3, 3]
        if axis[0] < 0.3:  # looking out, away from the rover
            continue
        far = 3000.0 if axis[2] > -0.05 else (floor_z_mm - origin[2]) / axis[2]
        if far < 450:
            continue
        inv = np.linalg.inv(cam)
        off = 0.0
        for d in HANG_MM:
            c = inv @ np.array([tcp[0], tcp[1], tcp[2] - d, 1.0])
            if c[2] < 60:
                off = np.inf
                break
            u, v = focal_px * c[0] / c[2] + w / 2, focal_px * c[1] / c[2] + h / 2
            if min(u, w - u, v, h - v) < 15:
                off = np.inf
                break
            off = max(off, abs(u - w / 2) / w + abs(v - h / 2) / h)
        if not np.isfinite(off):
            continue
        score = off + 0.15 * float(np.abs(q - home)[:5].sum())
        if kin.keep_out_hit(q, keep_out, keep_out_margin_mm, link5_points) is not None:
            continue
        if kin.arm_points(q)[:, 2].min() < z_min_mm + 20:
            continue
        found.append((score, q))
    for _, q in sorted(found, key=lambda f: f[0]):
        if collides is None or not any(
            collides(home + t * (q - home)) for t in np.linspace(0, 1, 9)
        ):
            return q
    return None

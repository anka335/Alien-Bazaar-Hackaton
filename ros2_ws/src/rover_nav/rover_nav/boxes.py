"""Laundry boxes from their ArUco markers (D-047), without ROS: image in, labelled boxes out.

Three boxes, always in this order from the left: dark (black clothes), colored, light (white).
Each carries an ArUco marker. A box is known by its marker id (`ids` in config/boxes.yaml); until
the ids are filled in, markers are labelled by their left-to-right order, which needs all three
in view. Markers of any common dictionary are found (`dictionaries`), so the ids can be read off
the detector's log first.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

CLASSES = ("dark", "colored", "light")  # left → right
DEFAULT_DICTIONARIES = ("DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250", "DICT_ARUCO_ORIGINAL")


def marker_corners(size_m: float) -> np.ndarray:
    """The marker's corners in its own frame (x right, y up, z out of the marker), in OpenCV's
    detection order: top-left, top-right, bottom-right, bottom-left."""
    h = size_m / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def from_xyz_rpy(xyz, rpy) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    T[:3, 3] = xyz
    return T


def to_xyz_rpy(T: np.ndarray) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    R = T[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return tuple(float(v) for v in T[:3, 3]), (
        math.atan2(R[2, 1], R[2, 2]),
        pitch,
        math.atan2(R[1, 0], R[0, 0]),
    )


def from_quaternion(xyz, q) -> np.ndarray:
    """4x4 from a translation and a quaternion (x, y, z, w), e.g. a TF transform."""
    x, y, z, w = q
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = xyz
    return T


@dataclass(frozen=True)
class Marker:
    dictionary: str
    marker_id: int
    T_cam_marker: np.ndarray  # 4x4, optical frame (x right, y down, z forward), m
    size_px: float  # mean edge length in the image
    corners_px: np.ndarray  # 4x2, detection order
    from_depth: bool = False  # distance from the depth image (see refine_with_depth)

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.T_cam_marker[:3, 3]))


@dataclass(frozen=True)
class Box:
    label: str  # dark / colored / light
    marker: Marker
    by_id: bool  # True: known id; False: labelled by left-to-right order


@dataclass
class BoxConfig:
    marker_size_m: float = 0.04
    dictionaries: tuple[str, ...] = DEFAULT_DICTIONARIES
    ids: dict[str, list[int]] = field(default_factory=dict)  # label → marker ids

    def label_of(self, marker_id: int) -> str | None:
        for label, ids in self.ids.items():
            if marker_id in (ids or []):
                return label
        return None


def detect_markers(
    image, K, dist, size_m: float, dictionaries=DEFAULT_DICTIONARIES
) -> list[Marker]:
    """Every ArUco marker of the given dictionaries, with its pose. Iterative PnP: in OpenCV 4.6
    the IPPE solvers returned poses ~40 px off for such corners. A marker found in several
    dictionaries is kept once (first); a pose showing the marker from behind is dropped."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = None if dist is None else np.asarray(dist, dtype=np.float64).ravel()
    found: list[Marker] = []
    centres: list[np.ndarray] = []
    for name in dictionaries:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, d, parameters=cv2.aruco.DetectorParameters_create()
        )
        if ids is None:
            continue
        for c, i in zip(corners, ids.ravel(), strict=True):
            pts = c.reshape(4, 2).astype(np.float64)
            centre = pts.mean(axis=0)
            if any(np.linalg.norm(centre - o) < 5 for o in centres):
                continue  # the same square decoded by another dictionary
            ok, rvec, tvec = cv2.solvePnP(
                marker_corners(size_m), pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not ok:
                continue
            T = np.eye(4)
            T[:3, :3] = cv2.Rodrigues(rvec)[0]
            T[:3, 3] = tvec.ravel()
            if float(T[:3, 2] @ T[:3, 3]) >= 0:
                continue  # seen from behind: a mirrored fit
            edge = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
            found.append(Marker(name, int(i), T, edge, pts))
            centres.append(centre)
    return found


def refine_with_depth(marker: Marker, depth_m: np.ndarray, min_valid: int = 5) -> Marker:
    """Distance from the depth image (aligned to the same color camera) instead of the marker's
    apparent size. A 4 cm marker is only ~20 px wide at 0.8 m, and the size-based distance comes
    out 4-10 % too long there (the detected corners sit ~1 px inside); depth inside the marker's
    outline is accurate to ~1-2 %. Keeps the direction from the image, scales the position so its
    z (optical axis) equals the median depth. Returns the marker unchanged without enough depth."""
    h, w = depth_m.shape[:2]
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, np.round(marker.corners_px).astype(np.int32), 1)
    values = depth_m[(mask > 0) & np.isfinite(depth_m) & (depth_m > 0)]
    if values.size < min_valid:
        return marker
    z = float(np.median(values))
    T = marker.T_cam_marker.copy()
    T[:3, 3] *= z / T[2, 3]
    return Marker(marker.dictionary, marker.marker_id, T, marker.size_px, marker.corners_px, True)


def label_boxes(markers: list[Marker], config: BoxConfig) -> list[Box]:
    """Known ids first; the rest by left-to-right order (camera x), only if exactly the labels
    not yet taken are left, i.e. all three boxes in view."""
    boxes = []
    taken = set()
    unknown = []
    for m in markers:
        label = config.label_of(m.marker_id)
        if label is not None and label not in taken:
            boxes.append(Box(label, m, by_id=True))
            taken.add(label)
        elif label is None:
            unknown.append(m)
    free = [c for c in CLASSES if c not in taken]
    if unknown and len(unknown) == len(free):
        for label, m in zip(free, sorted(unknown, key=lambda m: m.T_cam_marker[0, 3]), strict=True):
            boxes.append(Box(label, m, by_id=False))
    return sorted(boxes, key=lambda b: CLASSES.index(b.label))


def ahead_and_left(T_base_marker: np.ndarray, front_m: float) -> tuple[float, float]:
    """(distance from the rover's front to the marker along x, sideways offset left) in the
    rover's base frame (x forward, y left), m."""
    x, y = float(T_base_marker[0, 3]), float(T_base_marker[1, 3])
    return x - front_m, y


def bearing_deg(T_base_marker: np.ndarray) -> float:
    return math.degrees(math.atan2(T_base_marker[1, 3], T_base_marker[0, 3]))

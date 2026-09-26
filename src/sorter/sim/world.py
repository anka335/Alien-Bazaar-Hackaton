"""SimWorld: the table, the items on it, and the arm's joints over time. Shared by all sim parts.

The arm is simulated at the joint level (`SimDriver` plays planned paths back on `motion`), so
everything else follows from its kinematics: the wrist camera sees the table from where FK puts
it, a closing gripper grabs the cloth under the fingertips, and an opening one drops what it
holds onto whatever is below (the mat, a bin, the box, or the bare table).
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Literal

import numpy as np
from rebot_b601.arm import Trajectory

from sorter.arm import kinematics as kin
from sorter.core.types import ColorClass, Intrinsics, Pose, Zone
from sorter.sim.config import RectConfig, SimConfig

Location = Literal["box", "background", "gripper", "bin", "table"]

_PALETTE: dict[ColorClass, list[tuple[int, int, int]]] = {  # BGR
    # white, cream, light gray, pale blue
    ColorClass.LIGHT: [(238, 238, 236), (208, 228, 240), (206, 206, 204), (236, 222, 206)],
    # black, navy, charcoal, dark brown
    ColorClass.DARK: [(32, 30, 30), (72, 38, 20), (48, 46, 46), (30, 40, 62)],
    # red, blue, green, yellow, orange, purple, pink
    ColorClass.COLORED: [
        (48, 44, 206),
        (196, 104, 34),
        (70, 160, 48),
        (40, 196, 236),
        (32, 124, 238),
        (150, 62, 118),
        (170, 120, 236),
    ],
}

BG_ITEM_HEIGHT_MM = 25.0  # an item lying on the background
TABLE_ITEM_HEIGHT_MM = 20.0
BIN_ITEM_MM = 18.0  # each item in a bin raises the pile by this
GRASP_TOL_MM = 12.0  # the fingertips must get this close to the cloth top to catch it
GRIPPER_S = 0.5  # the real gripper takes about this long to open or close
HELD_OPENING = 0.3  # a gripper closed on cloth stops here
LOOK_POSES = {Zone.BOX: "look_box", Zone.BACKGROUND: "look_bg"}


@dataclass(frozen=True)
class CamPose:
    """Where the wrist camera is. The sim renders straight down from there, whatever the real
    tilt (look poses point it straight down). `ref_z_mm` is the plane the image scale follows."""

    x: float
    y: float
    z: float
    ref_z_mm: float = 0.0


def rect_bounds(r: RectConfig, grow: float = 0.0) -> tuple[float, float, float, float]:
    (cx, cy), (w, h) = r.center_mm, r.size_mm
    return cx - w / 2 - grow, cx + w / 2 + grow, cy - h / 2 - grow, cy + h / 2 + grow


def in_rect(x: float, y: float, r: RectConfig, margin: float = 0.0) -> bool:
    x0, x1, y0, y1 = rect_bounds(r, -margin)
    return x0 <= x <= x1 and y0 <= y <= y1


def camera_mount(cfg: SimConfig) -> Pose:
    """T_link5_cam: the camera is fixed to link5, so joint 6 doesn't turn it (D-027). Where it
    is with joint 6 at 0: `camera_mount_mm` off the TCP, optical axis along the approach."""
    return kin.T_LINK5_TCP0 @ _camera_on_tcp(cfg)


def _camera_on_tcp(cfg: SimConfig) -> Pose:
    """T_tcp_cam with joint 6 at 0."""
    T = np.eye(4)
    T[:3, :3] = [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]  # columns: x_cam = y_tcp, y_cam = z_tcp, z_cam = x_tcp: the image's long side across the
    # arm, as on the rig (~110° from radial there); the box's long side runs along y to match
    T[:3, 3] = cfg.camera_mount_mm
    return T


def camera_pose(cfg: SimConfig, q: Sequence[float]) -> Pose:
    """T_base_cam for joints `q`."""
    return kin.fk_link5(q) @ camera_mount(cfg)


class JointMotion:
    """The arm's joints over time: planned paths played back with the real arm's profile."""

    def __init__(self, q: Sequence[float]):
        self._q = np.asarray(q, dtype=float)
        self._traj: Trajectory | None = None

    def joints(self, now: float | None = None) -> np.ndarray:
        if self._traj is None:
            return self._q.copy()
        now = time.monotonic() if now is None else now
        t = now - self._traj.t0
        if t >= self._traj.duration:
            self._q, self._traj = self._traj.waypoints[-1].copy(), None
            return self._q.copy()
        return self._traj.sample(max(t, 0.0))

    def moving(self, now: float | None = None) -> bool:
        self.joints(now)
        return self._traj is not None

    def start(self, waypoints: np.ndarray, duration_s: float, ramp_s: float | None = None) -> None:
        wps = np.array(waypoints, dtype=float)
        wps[0] = self.joints()
        if duration_s <= 0 or len(wps) < 2:
            self._q, self._traj = wps[-1], None
        else:
            self._traj = Trajectory(wps, duration_s, t0=time.monotonic(), ramp=ramp_s)

    def freeze(self) -> None:
        self._q, self._traj = self.joints(), None


@dataclass(frozen=True)
class ZoneView:
    """Linear mapping between the image of a zone (from its look pose) and the arm frame."""

    zone: Zone
    center_mm: tuple[float, float]
    surface_z_mm: float
    mm_per_px: float
    width: int
    height: int
    cam_z_mm: float

    @classmethod
    def at(cls, zone: Zone, cam: CamPose, surface_z_mm: float, cfg: SimConfig) -> ZoneView:
        return cls(
            zone=zone,
            center_mm=(cam.x, cam.y),
            surface_z_mm=surface_z_mm,
            mm_per_px=(cam.z - cam.ref_z_mm) / cfg.focal_px,
            width=cfg.width,
            height=cfg.height,
            cam_z_mm=cam.z,
        )

    def to_px(self, x: float, y: float) -> tuple[float, float]:
        return (
            (x - self.center_mm[0]) / self.mm_per_px + self.width / 2,
            (y - self.center_mm[1]) / self.mm_per_px + self.height / 2,
        )

    def to_xy(self, u: float, v: float) -> tuple[float, float]:
        return (
            (u - self.width / 2) * self.mm_per_px + self.center_mm[0],
            (v - self.height / 2) * self.mm_per_px + self.center_mm[1],
        )

    def contains(self, x: float, y: float, margin_mm: float = 0.0) -> bool:
        u, v = self.to_px(x, y)
        m = margin_mm / self.mm_per_px
        return m <= u <= self.width - m and m <= v <= self.height - m

    def intrinsics(self) -> Intrinsics:
        f = self.cam_z_mm / self.mm_per_px
        return Intrinsics(
            fx=f, fy=f, cx=self.width / 2, cy=self.height / 2, width=self.width, height=self.height
        )


@dataclass
class SimItem:
    id: int
    color: ColorClass
    bgr: tuple[int, int, int]
    x: float  # mm, arm frame
    y: float
    height_mm: float  # top surface above the surface it lies on
    location: Location
    bin: ColorClass | None = None


class SimWorld:
    def __init__(self, cfg: SimConfig, poses: dict[str, list[float]]):
        self.cfg = cfg
        self.layout = cfg.layout
        self.rng = random.Random(cfg.seed)
        self.lock = threading.RLock()
        missing = [p for p in ("rest", *LOOK_POSES.values()) if p not in poses]
        if missing:
            raise ValueError(
                f"the simulator needs poses {missing} (rig.yaml → poses); "
                "compute them with: python -m sorter.sim.layout"
            )
        self.poses = {k: np.asarray(v, dtype=float) for k, v in poses.items()}
        self.motion = JointMotion(self.poses["rest"])
        self.T_link5_cam = camera_mount(cfg)
        box = self.layout.box
        surfaces = {Zone.BOX: box.floor_z_mm, Zone.BACKGROUND: 0.0}
        self.views = {
            z: ZoneView.at(z, self._cam(self.poses[name]), surfaces[z], cfg)
            for z, name in LOOK_POSES.items()
        }
        self.bin_xy: dict[ColorClass, tuple[float, float]] = dict(self.layout.bins.centers_mm)
        self.items = [
            SimItem(i, color, self.rng.choice(_PALETTE[color]), 0.0, 0.0, 0.0, "box")
            for i, color in enumerate(cfg.items)
        ]
        self.fill_box()

    # --- the arm ---

    def _cam(self, q: Sequence[float]) -> CamPose:
        p = camera_pose(self.cfg, q)[:3, 3]
        return CamPose(float(p[0]), float(p[1]), float(p[2]))

    def joints(self, now: float | None = None) -> np.ndarray:
        with self.lock:
            return self.motion.joints(now)

    def camera_pose(self, now: float | None = None) -> CamPose:
        return self._cam(self.joints(now))

    def tcp(self, now: float | None = None) -> np.ndarray:
        return kin.fk_tcp(self.joints(now))[:3, 3]

    @property
    def looking_at(self) -> Zone | None:
        """The zone whose look pose the arm is still at, if any."""
        with self.lock:
            if self.motion.moving():
                return None
            q = self.motion.joints()
        for zone, name in LOOK_POSES.items():
            if np.allclose(q, self.poses[name], atol=1e-4):
                return zone
        return None

    # --- the table ---

    def surface_at(self, x: float, y: float) -> tuple[Location, float, ColorClass | None]:
        """What an item dropped at (x, y) lands in: location, surface z, bin color."""
        lay = self.layout
        if in_rect(x, y, lay.box):
            return "box", lay.box.floor_z_mm, None
        if in_rect(x, y, lay.background):
            return "background", 0.0, None
        half = lay.bins.size_mm / 2
        for color, (bx, by) in self.bin_xy.items():
            if abs(x - bx) <= half and abs(y - by) <= half:
                return "bin", lay.bins.floor_z_mm, color
        return "table", 0.0, None

    def extent(self) -> tuple[float, float, float, float]:
        """x0, x1, y0, y1 of everything on the table, mm."""
        lay = self.layout
        rects = [rect_bounds(lay.box, 25), rect_bounds(lay.background, 25)]
        half = lay.bins.size_mm / 2
        rects += [(x - half, x + half, y - half, y + half) for x, y in self.bin_xy.values()]
        rects.append((-120, 120, -120, 120))  # the arm base
        return (
            min(r[0] for r in rects),
            max(r[1] for r in rects),
            min(r[2] for r in rects),
            max(r[3] for r in rects),
        )

    # --- items ---

    def at(self, location: Location) -> list[SimItem]:
        with self.lock:
            return [it for it in self.items if it.location == location]

    def top_z(self, it: SimItem) -> float:
        """Height of the item's top surface above the table, mm."""
        if it.location == "box":
            return self.layout.box.floor_z_mm + it.height_mm
        if it.location == "bin":
            return self.layout.bins.floor_z_mm + it.height_mm
        if it.location == "gripper":
            return float(self.tcp()[2])
        return it.height_mm

    def random_point(self, rect: RectConfig, margin_mm: float) -> tuple[float, float]:
        x0, x1, y0, y1 = rect_bounds(rect, -margin_mm)
        return self.rng.uniform(x0, x1), self.rng.uniform(y0, y1)

    def fill_box(self) -> None:
        """Put every item in the box, at random places, piled in id order."""
        with self.lock:
            for i, it in enumerate(self.items):
                it.x, it.y = self.random_point(self.layout.box, self.cfg.item_radius_mm + 5)
                it.height_mm = min(15.0 + 7.0 * i, 55.0)
                it.location, it.bin = "box", None

    def reset_if_sorted(self) -> None:
        """All items out of the box and off the mat: tip them back into the box for a new run."""
        with self.lock:
            if self.items and all(it.location in ("bin", "table") for it in self.items):
                self.fill_box()

    def free_point_on_background(self) -> tuple[float, float]:
        """A point on the mat as far as possible from the items already there."""
        r = self.cfg.item_radius_mm
        others = [(o.x, o.y) for o in self.at("background")]
        best, best_d = None, -1.0
        for _ in range(200):
            x, y = self.random_point(self.layout.background, margin_mm=r + 5)
            d = min((math.hypot(x - ox, y - oy) for ox, oy in others), default=math.inf)
            if d > best_d:
                best, best_d = (x, y), d
            if d > 3.5 * r:
                break
        return best

    def grasp(self, tcp: Sequence[float]) -> list[SimItem]:
        """The gripper closes at `tcp` (mm): it catches the highest cloth under the fingertips,
        unless it slips (`miss_prob`); from the box it can drag a second item (`double_prob`)."""
        x, y, z = tcp
        with self.lock:
            near = [
                it
                for it in self.items
                if it.location in ("box", "background")
                and math.hypot(it.x - x, it.y - y) <= 1.2 * self.cfg.item_radius_mm
                and z <= self.top_z(it) + GRASP_TOL_MM
            ]
            if not near or self.rng.random() < self.cfg.miss_prob:
                return []
            grabbed = [max(near, key=self.top_z)]
            if grabbed[0].location == "box" and self.rng.random() < self.cfg.double_prob:
                rest = [it for it in self.at("box") if it is not grabbed[0]]
                if rest:
                    grabbed.append(min(rest, key=lambda it: math.hypot(it.x - x, it.y - y)))
            for it in grabbed:
                it.location = "gripper"
            return grabbed

    def release(self, tcp: Sequence[float]) -> list[SimItem]:
        """The gripper opens at `tcp` (mm): what it held drops onto whatever is below."""
        with self.lock:
            dropped = self.at("gripper")
            for i, it in enumerate(dropped):
                a = self.rng.uniform(0, 2 * math.pi)
                it.x, it.y = tcp[0] + 10 * math.cos(a), tcp[1] + 10 * math.sin(a)
                loc, _, color = self.surface_at(it.x, it.y)
                if loc == "background" and i > 0:  # a second item falls off to the side
                    it.x, it.y = self.free_point_on_background()
                it.location, it.bin = loc, color
                if loc == "background":
                    it.height_mm = BG_ITEM_HEIGHT_MM
                elif loc == "bin":
                    half = self.layout.bins.size_mm / 2 - 45
                    bx, by = self.bin_xy[color]
                    it.x = min(max(it.x, bx - half), bx + half)
                    it.y = min(max(it.y, by - half), by + half)
                    it.height_mm = BIN_ITEM_MM * sum(o.bin is color for o in self.at("bin"))
                elif loc == "box":
                    it.height_mm = max((o.height_mm for o in self.at("box")), default=0) + 10
                else:
                    it.height_mm = TABLE_ITEM_HEIGHT_MM
            return dropped

    def bins(self) -> dict[ColorClass, list[SimItem]]:
        with self.lock:
            return {c: [it for it in self.items if it.bin is c] for c in ColorClass}

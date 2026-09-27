"""OakD: the Luxonis OAK-D on the rover, rendered by MuJoCo. RGB + stereo depth aligned to it.

Depth follows how the OAK-D makes it, not the render's perfect z-buffer:
- the true depth becomes a disparity d = f·B / Z at the stereo resolution (400P with extended
  disparity, 800P normal), gets noise and is quantized to 1/2^subpixel_bits px;
- no depth where d exceeds the disparity search range (closer than MinZ: ~0.17 m extended,
  ~0.7 m normal), beyond `max_range_m`, outside the stereo pair's vertical FOV (49° vs the RGB's
  55°: bands at the top and bottom), on plain surfaces (no texture to match: the OAK-D has no IR
  projector) and on random speckles.
Depth is uint16 millimetres like DepthAI's, 0 = no data.

Frames: the rover frame has its origin on the floor under the rover's center, x forward, y left,
z up. `Frame.T_rover_cam` maps camera (OpenCV: x right, y down, z forward) to rover coordinates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import mujoco
import numpy as np

from sorter.nav.config import OakDConfig
from sorter.nav.model import BASE_Z

MAX_DISPARITY = {"extended": 190.0, "normal": 95.0}  # px at the stereo resolution
STEREO_WIDTH = {"extended": 640, "normal": 1280}
STEREO_HFOV_DEG = 72.0


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class Frame:
    index: int
    t: float
    rgb: np.ndarray  # HxWx3 uint8 RGB
    depth_mm: np.ndarray  # HxW uint16, 0 = none
    K: Intrinsics
    T_rover_cam: np.ndarray  # 4x4

    def depth_at(self, u: float, v: float, r: int = 3) -> float | None:
        """Median depth (m) of the valid pixels in a (2r+1)² window, None if there are none."""
        h, w = self.depth_mm.shape
        u0, v0 = int(round(u)), int(round(v))
        win = self.depth_mm[
            max(0, v0 - r) : min(h, v0 + r + 1), max(0, u0 - r) : min(w, u0 + r + 1)
        ]
        good = win[win > 0]
        return float(np.median(good)) / 1000 if good.size else None

    def ray(self, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
        """(origin, unit direction) of the pixel's viewing ray in the rover frame."""
        d = np.array([(u - self.K.cx) / self.K.fx, (v - self.K.cy) / self.K.fy, 1.0])
        R, o = self.T_rover_cam[:3, :3], self.T_rover_cam[:3, 3]
        d = R @ d
        return o, d / np.linalg.norm(d)

    def point(self, u: float, v: float, depth_m: float | None = None) -> np.ndarray | None:
        """The pixel as a rover-frame point: from its depth (the median around it), or, with no
        depth, where its ray meets the floor (z = 0). None if neither exists."""
        z = depth_m if depth_m is not None else self.depth_at(u, v)
        if z is not None:
            pc = np.array([(u - self.K.cx) / self.K.fx * z, (v - self.K.cy) / self.K.fy * z, z])
            return self.T_rover_cam[:3, :3] @ pc + self.T_rover_cam[:3, 3]
        return self.floor_point(u, v)

    def floor_point(self, u: float, v: float) -> np.ndarray | None:
        o, d = self.ray(u, v)
        if d[2] >= -1e-6:
            return None  # at or above the horizon
        return o + d * (-o[2] / d[2])

    def points(self) -> np.ndarray:
        """Every pixel as a rover-frame point (HxWx3, NaN without depth)."""
        z = self.depth_mm.astype(np.float32) / 1000
        h, w = z.shape
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        pc = np.stack([(uu - self.K.cx) / self.K.fx * z, (vv - self.K.cy) / self.K.fy * z, z], -1)
        p = pc @ self.T_rover_cam[:3, :3].T + self.T_rover_cam[:3, 3]
        p[z == 0] = np.nan
        return p

    def project(self, p_rover) -> tuple[float, float] | None:
        """A rover-frame point → pixel (u, v), None if behind the camera."""
        T = np.linalg.inv(self.T_rover_cam)
        pc = T[:3, :3] @ np.asarray(p_rover, float) + T[:3, 3]
        if pc[2] <= 1e-6:
            return None
        return (self.K.fx * pc[0] / pc[2] + self.K.cx, self.K.fy * pc[1] / pc[2] + self.K.cy)


class OakD:
    """Renders frames of `sim` (a RoverSim). Create and use it in one thread (OpenGL)."""

    def __init__(self, sim, cfg: OakDConfig):
        self.sim, self.cfg = sim, cfg
        self.renderer = mujoco.Renderer(sim.model, cfg.height, cfg.width)
        self.cam_id = sim.model.camera("oakd_rgb").id
        fovy = float(sim.model.cam_fovy[self.cam_id])
        f = cfg.height / 2 / math.tan(math.radians(fovy) / 2)
        self.K = Intrinsics(f, f, cfg.width / 2, cfg.height / 2, cfg.width, cfg.height)
        self.T_rover_cam = _mount(cfg)
        self._index = 0
        self._rng = np.random.default_rng([sim.spec.seed, 11])
        self._seg_ids: dict[int, int] = {}  # geom id → sock index (for the oracle detector)

    def close(self) -> None:
        self.renderer.close()

    def capture(self) -> Frame:
        r, d = self.renderer, self.sim.data
        r.update_scene(d, self.cam_id)
        rgb = r.render().copy()
        r.enable_depth_rendering()
        r.update_scene(d, self.cam_id)
        z = r.render().copy()
        r.disable_depth_rendering()
        depth = self._stereo(z, rgb)
        rgb = np.clip(
            rgb.astype(np.float32) + self._rng.normal(0, self.cfg.rgb_noise, rgb.shape), 0, 255
        ).astype(np.uint8)
        self._index += 1
        return Frame(self._index, self.sim.t, rgb, depth, self.K, self.T_rover_cam)

    def segmentation(self) -> np.ndarray:
        """Geom id per pixel (-1 = none): MuJoCo's segmentation, the oracle `seg` detector."""
        r = self.renderer
        r.enable_segmentation_rendering()
        r.update_scene(self.sim.data, self.cam_id)
        seg = r.render()[..., 0].copy()
        r.disable_segmentation_rendering()
        return seg

    def render_view(self, camera: str, width: int = 640, height: int = 480) -> np.ndarray:
        """A debug view (chase, overview) through the same renderer size."""
        r = self.renderer
        r.update_scene(self.sim.data, camera)
        return r.render().copy()

    def _stereo(self, z: np.ndarray, rgb: np.ndarray) -> np.ndarray:
        c = self.cfg
        mode = c.depth_mode
        f_st = STEREO_WIDTH[mode] / 2 / math.tan(math.radians(STEREO_HFOV_DEG) / 2)
        fb = f_st * c.baseline_m
        with np.errstate(divide="ignore", invalid="ignore"):
            disp = fb / z
        disp += self._rng.normal(0, c.disparity_noise_px, z.shape)
        step = 1.0 / (1 << c.subpixel_bits)
        disp = np.round(disp / step) * step
        valid = (disp >= 1.0) & (disp <= MAX_DISPARITY[mode]) & (z > 0)
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = np.where(valid, fb / disp, 0.0)
        valid &= depth <= c.max_range_m
        # rows outside the stereo pair's vertical FOV
        half = math.tan(math.radians(c.stereo_vfov_deg) / 2) * self.K.fy
        rows = np.abs(np.arange(c.height) - self.K.cy) > half
        valid[rows] = False
        # plain surfaces: nothing for the stereo matcher to lock on to
        if c.texture_min > 0:
            g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)
            m = cv2.blur(g, (7, 7))
            sd = np.sqrt(np.maximum(cv2.blur(g * g, (7, 7)) - m * m, 0))
            valid &= sd >= c.texture_min
        valid &= self._rng.random(z.shape) >= c.dropout
        return np.where(valid, np.round(depth * 1000), 0).astype(np.uint16)


def _mount(cfg: OakDConfig) -> np.ndarray:
    """Camera (OpenCV axes) → rover frame (origin on the floor under the rover's center)."""
    p = math.radians(cfg.pitch_deg)
    fwd = np.array([math.cos(p), 0.0, -math.sin(p)])
    down = -np.array([math.sin(p), 0.0, math.cos(p)])
    right = np.cross(down, fwd)  # x = y × z for a right-handed camera frame
    T = np.eye(4)
    T[:3, :3] = np.column_stack([right, down, fwd])
    mx, my, mz = cfg.mount_xyz_m
    T[:3, 3] = (mx + 0.002, my, BASE_Z + mz)
    return T


def colorize_depth(depth_mm: np.ndarray, max_m: float = 5.0) -> np.ndarray:
    """Depth → RGB for viewing: near = warm, far = cool, no data = black."""
    z = depth_mm.astype(np.float32) / 1000
    x = np.clip(z / max_m, 0, 1)
    img = cv2.applyColorMap((255 * (1 - x)).astype(np.uint8), cv2.COLORMAP_TURBO)[..., ::-1]
    img = img.copy()
    img[depth_mm == 0] = 0
    return img


def fit_floor(frames: list[Frame], max_m: float = 3.0) -> tuple[float, float, float, int] | None:
    """The floor plane in the depth of `frames` (lower image half, RANSAC then least squares):
    (camera height above it m, pitch down deg, roll deg, inlier count), None if too few points.
    The camera's mount can be read off the real floor this way (`hw-check`)."""
    pts = []
    for f in frames:
        z = f.depth_mm.astype(np.float32) / 1000
        h, w = z.shape
        vv, uu = np.mgrid[0:h, 0:w]
        m = (z > 0.2) & (z < max_m) & (vv > h * 0.55)
        pts.append(
            np.stack([(uu[m] - f.K.cx) / f.K.fx * z[m], (vv[m] - f.K.cy) / f.K.fy * z[m], z[m]], 1)
        )
    P = np.concatenate(pts) if pts else np.zeros((0, 3))
    if len(P) < 500:
        return None
    rng = np.random.default_rng(0)
    best = None
    for _ in range(300):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = int((np.abs((P - a) @ n) < 0.02).sum())
        if best is None or cnt > best[0]:
            best = (cnt, n, a)
    cnt, n, a = best
    Q = P[np.abs((P - a) @ n) < 0.02]
    c = Q.mean(0)
    n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
    if n[1] > 0:  # the floor's normal points up, i.e. toward the camera's -y
        n = -n
    height = abs(float(n @ c))
    pitch = math.degrees(math.asin(float(-n[2])))
    roll = math.degrees(math.atan2(float(n[0]), float(-n[1])))
    return height, pitch, roll, len(Q)

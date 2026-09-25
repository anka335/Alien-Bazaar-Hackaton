"""Sim scene: a table seen by the moving wrist camera, with crumpled clothes on it.

The table (wood, a cardboard box, a gray mat, three bins) is rendered once into a top-down map,
`K` px per mm. A frame is a crop of that map with the items pasted in, warped to the camera
image. At a look pose the warp is exactly `ZoneView.to_px`, so sim vision and sim calibration
(which use `ZoneView`) agree with the picture.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from sorter.core.types import ColorClass, Zone
from sorter.sim.world import BIN_FLOOR_Z_MM, BIN_SIZE_MM, CamPose, SimItem, SimWorld

K = 2.0  # map px per mm

_FLOOR_BGR = (46, 44, 42)
_FLOOR_Z_MM = -750.0
_WOOD_BGR = (150, 183, 208)
_MAT_BGR = (122, 124, 126)
_CARDBOARD_BGR = (92, 140, 184)
_BOX_RIM_BGR = (126, 172, 210)
_BOX_WALL_MM = 14.0
_BOX_WALL_H_MM = 130.0
_BIN_RIM_MM = 12.0
_BIN_WALL_H_MM = 220.0
_BIN_RIM_BGR = {
    ColorClass.LIGHT: (234, 236, 236),
    ColorClass.DARK: (60, 58, 56),
    ColorClass.COLORED: (170, 140, 36),
}
_FINGER_BGR = (74, 76, 82)

# The static table depends only on the layout; tests build many worlds with the same one.
_TABLES: dict[tuple, tuple[np.ndarray, np.ndarray]] = {}


@dataclass(frozen=True)
class _ItemView:
    id: int
    color: ColorClass
    bgr: tuple[int, int, int]
    x: float
    y: float
    top_z: float


def _smooth_noise(rng: np.random.Generator, shape: tuple[int, int], cells: tuple[int, int]):
    n = rng.normal(size=(max(cells[0], 2), max(cells[1], 2))).astype(np.float32)
    return cv2.resize(n, (shape[1], shape[0]), interpolation=cv2.INTER_CUBIC)


def _rounded_rect_mask(shape: tuple[int, int], x0, y0, x1, y1, r) -> np.ndarray:
    """uint8 mask of a rounded rectangle, coordinates in px."""
    m = np.zeros(shape, np.uint8)
    x0, y0, x1, y1, r = (round(v) for v in (x0, y0, x1, y1, r))
    cv2.rectangle(m, (x0 + r, y0), (x1 - r, y1), 255, -1)
    cv2.rectangle(m, (x0, y0 + r), (x1, y1 - r), 255, -1)
    for cx, cy in ((x0 + r, y0 + r), (x1 - r, y0 + r), (x0 + r, y1 - r), (x1 - r, y1 - r)):
        cv2.circle(m, (cx, cy), r, 255, -1, cv2.LINE_AA)
    return m


def _cast_shadow(img: np.ndarray, mask: np.ndarray, dx: float, dy: float, blur: float, k: float):
    """Darken `img` (float32) under `mask` shifted by (dx, dy) px and blurred."""
    m = mask.astype(np.float32) / 255
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    m = cv2.warpAffine(m, M, (m.shape[1], m.shape[0]))
    m = cv2.GaussianBlur(m, (0, 0), blur)
    img *= (1 - k * m)[..., None]


def _inner_shade(mask: np.ndarray, depth_px: float, low: float) -> np.ndarray:
    """1 inside, falling to `low` towards the mask edge: ambient occlusion by the walls."""
    d = cv2.distanceTransform((mask > 0).astype(np.uint8), cv2.DIST_L2, 5)
    return (low + (1 - low) * np.clip(d / depth_px, 0, 1)).astype(np.float32)


def cloth_sprite(
    rng: np.random.Generator, bgr: tuple[int, int, int], color: ColorClass, radius_px: float
) -> tuple[np.ndarray, np.ndarray]:
    """A crumpled piece of cloth seen from above: (BGR float32, alpha float32), centered."""
    R = radius_px * rng.uniform(1.05, 1.25)
    size = int(R * 3.2) | 1
    c = size / 2
    # outline: a stretched, rotated circle with random lobes and a ragged edge
    th = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    r = np.ones_like(th)
    for n in range(2, 14):
        amp = rng.uniform(0.03, 0.2) / n**0.9
        r += amp * np.cos(n * th + rng.uniform(0, 2 * np.pi))
    r = np.clip(r, 0.72, None) * R
    stretch, rot = rng.uniform(1.0, 1.35), rng.uniform(0, np.pi)
    px, py = r * np.cos(th) * stretch, r * np.sin(th) / stretch
    pts = np.stack(
        [c + px * math.cos(rot) - py * math.sin(rot), c + px * math.sin(rot) + py * math.cos(rot)],
        1,
    )
    mask = np.zeros((size, size), np.uint8)
    cv2.fillPoly(mask, [np.round(pts * 16).astype(np.int32)], 255, cv2.LINE_AA, shift=4)
    alpha = cv2.GaussianBlur(mask.astype(np.float32) / 255, (0, 0), 0.8)

    # folds: a smooth height field plus a few soft ridges; shading comes from its slope
    field = cv2.GaussianBlur(rng.normal(size=(size, size)).astype(np.float32), (0, 0), R * 0.2)
    field /= field.std() + 1e-6
    ridges = np.zeros((size, size), np.float32)
    for _ in range(rng.integers(2, 5)):
        a = rng.uniform(0, np.pi)
        p0 = np.array([c, c]) + rng.normal(0, R * 0.25, 2)
        d = np.array([math.cos(a), math.sin(a)]) * R * rng.uniform(0.5, 1.0)
        bend = np.array([-d[1], d[0]]) * rng.uniform(-0.5, 0.5)
        ts = np.linspace(0, 1, 32)[:, None]
        line = (1 - ts) ** 2 * (p0 - d) + 2 * (1 - ts) * ts * (p0 + bend) + ts**2 * (p0 + d)
        width = max(2, round(R * rng.uniform(0.05, 0.1)))
        cv2.polylines(ridges, [np.round(line).astype(np.int32)], False, 1.0, width, cv2.LINE_AA)
    field += 1.6 * cv2.GaussianBlur(ridges, (0, 0), R * 0.07)
    gx = cv2.Sobel(field, cv2.CV_32F, 1, 0, ksize=5)
    gy = cv2.Sobel(field, cv2.CV_32F, 0, 1, ksize=5)
    g = np.sqrt(gx**2 + gy**2).std() + 1e-6
    shade = 1 + 0.2 * (-0.6 * gx - 0.8 * gy) / g  # light from the top left
    shade *= 0.7 + 0.3 * _inner_shade(mask, R * 0.3, 0.0)  # thicker, darker at the rim

    base = np.array(bgr, np.float32)
    col = np.broadcast_to(base, (size, size, 3)).copy()
    yy, xx = np.mgrid[0:size, 0:size].astype(np.float32)
    kind = rng.random()
    if color is ColorClass.COLORED and kind < 0.45:  # stripes
        other = np.array(rng.choice([(235, 235, 235), (45, 40, 38), (70, 200, 245)]), np.float32)
        a = rng.uniform(0, np.pi)
        period = rng.uniform(9, 16) * radius_px / 70
        phase = (xx * math.cos(a) + yy * math.sin(a)) / period + 0.15 * field
        col[np.sin(phase * 2 * np.pi) > 0.35] = other
    elif color is ColorClass.DARK and kind < 0.5:  # denim twill
        twill = np.sin((xx + yy) * 1.3) * 0.5 + 0.5
        col *= (0.9 + 0.12 * twill)[..., None]
    col *= np.clip(shade, 0.45, 1.25)[..., None]
    col += rng.normal(0, 3, col.shape).astype(np.float32)  # weave
    return np.clip(col, 0, 255), alpha


def _clip(dst_shape, a_shape, x0: int, y0: int):
    """Slices of `dst` and of a source placed at (x0, y0), or None if they don't overlap."""
    h, w = a_shape
    dx0, dy0 = max(x0, 0), max(y0, 0)
    dx1, dy1 = min(x0 + w, dst_shape[1]), min(y0 + h, dst_shape[0])
    if dx1 <= dx0 or dy1 <= dy0:
        return None
    return (slice(dy0, dy1), slice(dx0, dx1)), (
        slice(dy0 - y0, dy1 - y0),
        slice(dx0 - x0, dx1 - x0),
    )


def _paste(dst: np.ndarray, src: np.ndarray, a: np.ndarray, x0: int, y0: int) -> np.ndarray | None:
    """Alpha-blend `src` into `dst` at (x0, y0), clipped. Returns the alpha as placed, or None."""
    c = _clip(dst.shape, a.shape, x0, y0)
    if c is None:
        return None
    d, s = c
    sa = a[s][..., None]
    dst[d] = dst[d] * (1 - sa) + src[s] * sa
    out = np.zeros(dst.shape[:2], np.float32)
    out[d] = a[s]
    return out


def _darken(dst: np.ndarray, a: np.ndarray, x0: int, y0: int, k: float) -> None:
    c = _clip(dst.shape, a.shape, x0, y0)
    if c is not None:
        d, s = c
        dst[d] *= (1 - k * a[s])[..., None]


class Scene:
    def __init__(self, world: SimWorld):
        self.world = world
        cfg = world.cfg
        self.W, self.H = cfg.width, cfg.height
        view = next(iter(world.views.values()))
        self.f = view.cam_height_mm / view.mm_per_px  # focal length, px
        x0, x1, y0, y1 = world.zones_extent()
        half = BIN_SIZE_MM / 2
        bx = [x for x, _ in world.bin_xy.values()]
        by = [y for _, y in world.bin_xy.values()]
        m = 90.0
        self.X0, self.Y0 = min(x0, min(bx) - half) - m, min(y0, min(by) - half) - m
        X1, Y1 = max(x1, max(bx) + half) + m, max(y1, max(by) + half) + m
        self.shape = (round((Y1 - self.Y0) * K), round((X1 - self.X0) * K))
        self.rng = np.random.default_rng(cfg.seed)
        key = (cfg.seed, self.shape, repr(world.views), repr(world.bin_xy))
        if key not in _TABLES:
            _TABLES[key] = self._build_table()
        self.color, self.height = _TABLES[key]  # read-only: frames copy what they draw on
        self._sprites: dict[int, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        yy, xx = np.mgrid[0 : self.H, 0 : self.W].astype(np.float32)
        rr = ((xx - self.W / 2) / self.W) ** 2 + ((yy - self.H / 2) / self.H) ** 2
        self._vignette = (1 - 0.55 * rr)[..., None].astype(np.float32)
        self._grain = [
            self.rng.normal(0, 2.2, (self.H, self.W, 1)).astype(np.float32) for _ in range(4)
        ]
        self._n = 0

    # --- map coordinates ---

    def _px(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.X0) * K, (y - self.Y0) * K

    def _rect_px(self, x0, y0, x1, y1) -> tuple[float, float, float, float]:
        return (*self._px(x0, y0), *self._px(x1, y1))

    # --- the static table ---

    def _build_table(self) -> tuple[np.ndarray, np.ndarray]:
        h, w = self.shape
        rng = self.rng
        grain = 0.6 * _smooth_noise(rng, (h, w), (h // 5, w // 140)) + 0.4 * _smooth_noise(
            rng, (h, w), (h // 26, w // 320)
        )
        img = np.empty((h, w, 3), np.float32)
        img[:] = _WOOD_BGR
        img *= (1 + 0.07 * grain)[..., None]
        for y in np.arange(0, h, 150 * K):  # plank seams
            img[int(y) : int(y) + 2] *= 0.82
        img += rng.normal(0, 2, (h, w, 1)).astype(np.float32)
        height = np.zeros((h, w), np.float32)

        # the gray mat under the background view
        bg = self.world.views[Zone.BACKGROUND]
        x0, y0 = bg.to_xy(0, 0)
        x1, y1 = bg.to_xy(bg.width, bg.height)
        mat = _rounded_rect_mask((h, w), *self._rect_px(x0 - 25, y0 - 25, x1 + 25, y1 + 25), 10 * K)
        _cast_shadow(img, mat, 2 * K, 3 * K, 3 * K, 0.35)
        felt = cv2.GaussianBlur(rng.normal(0, 6, (h, w)).astype(np.float32), (0, 0), 0.9)
        on = mat > 0
        img[on] = np.array(_MAT_BGR, np.float32) + felt[on, None]
        height[on] = bg.surface_z_mm

        # the cardboard box around the box view
        box = self.world.views[Zone.BOX]
        x0, y0 = box.to_xy(0, 0)
        x1, y1 = box.to_xy(box.width, box.height)
        inner = _rounded_rect_mask((h, w), *self._rect_px(x0 - 12, y0 - 12, x1 + 12, y1 + 12), 4)
        t = 12 + _BOX_WALL_MM
        outer = _rounded_rect_mask((h, w), *self._rect_px(x0 - t, y0 - t, x1 + t, y1 + t), 6)
        _cast_shadow(img, outer, 10 * K, 14 * K, 12 * K, 0.5)
        walls = (outer > 0) & (inner == 0)
        img[walls] = _BOX_RIM_BGR
        img[walls] += rng.normal(0, 3, (int(walls.sum()), 1)).astype(np.float32)
        on = inner > 0
        floor = np.array(_CARDBOARD_BGR, np.float32) + rng.normal(0, 3, (h, w, 1)).astype(
            np.float32
        )
        img[on] = (floor * _inner_shade(inner, 45 * K, 0.55)[..., None])[on]
        height[walls] = box.surface_z_mm + _BOX_WALL_H_MM
        height[on] = box.surface_z_mm

        # the bins
        half = BIN_SIZE_MM / 2
        for color, (bx, by) in self.world.bin_xy.items():
            outer = _rounded_rect_mask(
                (h, w), *self._rect_px(bx - half, by - half, bx + half, by + half), 18 * K
            )
            r = half - _BIN_RIM_MM
            inner = _rounded_rect_mask(
                (h, w), *self._rect_px(bx - r, by - r, bx + r, by + r), 12 * K
            )
            _cast_shadow(img, outer, 12 * K, 16 * K, 14 * K, 0.5)
            rim = np.array(_BIN_RIM_BGR[color], np.float32)
            walls = (outer > 0) & (inner == 0)
            img[walls] = rim
            on = inner > 0
            img[on] = (rim * 0.5 * _inner_shade(inner, 60 * K, 0.35)[..., None])[on]
            height[walls] = BIN_FLOOR_Z_MM + _BIN_WALL_H_MM
            height[on] = BIN_FLOOR_Z_MM
        return np.clip(img, 0, 255), height

    # --- items ---

    def _sprite(self, it: _ItemView) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        s = self._sprites.get(it.id)
        if s is None:
            rng = np.random.default_rng((self.world.cfg.seed, it.id))
            col, a = cloth_sprite(rng, it.bgr, it.color, self.world.cfg.item_radius_mm * K)
            shadow = cv2.GaussianBlur(
                cv2.warpAffine(a, np.float32([[1, 0, 4 * K], [0, 1, 6 * K]]), a.shape[::-1]),
                (0, 0),
                4 * K,
            )
            s = self._sprites[it.id] = (col, a, shadow)
        return s

    def _items(self) -> tuple[list[_ItemView], list[_ItemView]]:
        """Items on the table (lowest first) and in the gripper, read under the world lock."""
        out, held = [], []
        in_bin: dict[ColorClass, int] = {}
        surface = {z: v.surface_z_mm for z, v in self.world.views.items()}
        for it in self.world.items:
            v = self._item_view(it, surface, in_bin)
            (held if it.location == "gripper" else out).append(v)
        out.sort(key=lambda v: v.top_z)
        return out, held

    def _item_view(self, it: SimItem, surface: dict[Zone, float], in_bin: dict) -> _ItemView:
        x, y, top = it.x, it.y, 0.0
        if it.location in ("box", "background"):
            top = surface[Zone(it.location)] + it.height_mm
        elif it.location == "bin" and it.bin is not None:
            n = in_bin[it.bin] = in_bin.get(it.bin, 0) + 1
            rng = np.random.default_rng((it.id, 7))
            bx, by = self.world.bin_xy[it.bin]
            spread = BIN_SIZE_MM / 2 - 50
            x, y = bx + rng.uniform(-spread, spread), by + rng.uniform(-spread, spread)
            top = BIN_FLOOR_Z_MM + 18 * n
        return _ItemView(it.id, it.color, it.bgr, x, y, top)

    # --- a frame ---

    def render(self, now: float | None = None) -> tuple[np.ndarray, np.ndarray]:
        """(color BGR uint8, depth uint16 mm) as the wrist camera sees it now."""
        now = time.monotonic() if now is None else now
        with self.world.lock:
            pose = self.world.camera.pose(now)
            before = self.world.camera.pose(now - 0.06)
            items, held = self._items()
        mpp = max(pose.z - pose.ref_z_mm, 20.0) / self.f
        color, height = self._view(pose, mpp, items)
        color = self._motion_blur(color, pose, before, mpp)
        no_depth = self._gripper(color, held)
        depth = np.clip(np.rint(pose.z - height), 0, 65535).astype(np.uint16)
        depth[no_depth] = 0  # the fingers are closer than the depth camera's minimum range
        color *= self._vignette
        color += self._grain[self._n % len(self._grain)]
        self._n += 1
        return np.clip(color, 0, 255).astype(np.uint8), depth

    def _view(self, pose: CamPose, mpp: float, items: list[_ItemView]):
        W, H = self.W, self.H
        hw, hh = W / 2 * mpp, H / 2 * mpp
        mh, mw = self.shape
        i0 = int(np.clip(math.floor((pose.x - hw - self.X0) * K) - 2, 0, mw))
        i1 = int(np.clip(math.ceil((pose.x + hw - self.X0) * K) + 2, 0, mw))
        j0 = int(np.clip(math.floor((pose.y - hh - self.Y0) * K) - 2, 0, mh))
        j1 = int(np.clip(math.ceil((pose.y + hh - self.Y0) * K) + 2, 0, mh))
        if i1 - i0 < 2 or j1 - j0 < 2:
            color = np.empty((H, W, 3), np.float32)
            color[:] = _FLOOR_BGR
            return color, np.full((H, W), _FLOOR_Z_MM, np.float32)
        crop = self.color[j0:j1, i0:i1].copy()
        crop_h = self.height[j0:j1, i0:i1].copy()
        for it in items:
            col, a, shadow = self._sprite(it)
            cx, cy = self._px(it.x, it.y)
            x0 = round(cx - i0 - a.shape[1] / 2)
            y0 = round(cy - j0 - a.shape[0] / 2)
            _darken(crop, shadow, x0, y0, 0.45)
            placed = _paste(crop, col, a, x0, y0)
            if placed is not None:
                crop_h[placed > 0.5] = it.top_z
        # map px (i, j) -> image px (u, v); pixel centers at +0.5
        s = 1 / (K * mpp)
        bu = ((i0 + 0.5) / K + self.X0 - pose.x) / mpp + W / 2 - 0.5
        bv = ((j0 + 0.5) / K + self.Y0 - pose.y) / mpp + H / 2 - 0.5
        M = np.float32([[s, 0, bu], [0, s, bv]])
        color = cv2.warpAffine(
            crop, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=_FLOOR_BGR
        ).astype(np.float32)
        height = cv2.warpAffine(crop_h, M, (W, H), flags=cv2.INTER_NEAREST, borderValue=_FLOOR_Z_MM)
        return color, height

    def _motion_blur(self, color, pose: CamPose, before: CamPose, mpp: float) -> np.ndarray:
        dx, dy = (pose.x - before.x) / mpp, (pose.y - before.y) / mpp
        n = min(int(math.hypot(dx, dy) * 0.5), 31)
        if n < 3:
            return color
        k = np.zeros((n | 1, n | 1), np.float32)
        c, a = (n | 1) // 2, math.atan2(dy, dx)
        p = (round(c + c * math.cos(a)), round(c + c * math.sin(a)))
        q = (round(c - c * math.cos(a)), round(c - c * math.sin(a)))
        cv2.line(k, p, q, 1.0, 1)
        return cv2.filter2D(color, -1, k / k.sum())

    def _gripper(self, color: np.ndarray, held: list[_ItemView]) -> np.ndarray:
        """Draw what hangs in the gripper and the fingers; return the fingers' mask."""
        W, H = self.W, self.H
        if held:
            col, a, _ = self._sprite(held[0])
            scale = 1.05 * W / a.shape[1]
            col = cv2.GaussianBlur(cv2.resize(col, None, fx=scale, fy=scale), (0, 0), 5)
            a = cv2.GaussianBlur(cv2.resize(a, None, fx=scale, fy=scale), (0, 0), 5)
            _paste(color, col * 0.85, a, round(W / 2 - a.shape[1] / 2), round(H * 0.52))
        spread = 0.075 if held else 0.17
        layer = np.zeros((H, W), np.float32)
        across = np.zeros((H, W), np.float32)  # -1..1 across each finger, for the metal shading
        xs = np.arange(W, dtype=np.float32)[None, :]
        for side in (-1, 1):
            cx = W / 2 + side * spread * W
            pts = np.array(
                [
                    (cx - 0.052 * W, H + 2),
                    (cx - 0.03 * W, H * 0.79),
                    (cx + 0.03 * W, H * 0.79),
                    (cx + 0.052 * W, H + 2),
                ]
            )
            m = np.zeros((H, W), np.float32)
            cv2.fillPoly(m, [np.round(pts * 16).astype(np.int32)], 1.0, cv2.LINE_AA, shift=4)
            layer = np.maximum(layer, m)
            across += m * np.clip((xs - cx) / (0.05 * W), -1, 1)
        layer = cv2.GaussianBlur(layer, (0, 0), 2.0)  # out of focus, right in front of the lens
        metal = 0.75 + 0.55 * np.exp(-(((across + 0.35) / 0.35) ** 2))  # a highlight off-center
        metal *= np.linspace(0.7, 1.1, H, dtype=np.float32)[:, None]
        tip = np.zeros((H, W), np.float32)
        tip[round(H * 0.79) : round(H * 0.83)] = 1  # the rubber pad at the fingertip
        finger = np.array(_FINGER_BGR, np.float32) * (metal * (1 - 0.55 * tip))[..., None]
        color[:] = color * (1 - layer[..., None]) + finger * layer[..., None]
        return layer > 0.5

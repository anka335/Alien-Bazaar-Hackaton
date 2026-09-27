"""Sock detectors on an OAK-D frame: where in the image, and where on the floor (rover frame).

- `classic`: no model. Pixels whose color stands out from the floor around them (a wide median
  of the image), that lie on the floor by depth or by the floor-plane ray, grouped into blobs
  and kept if their size on the floor is a sock's.
- `sam3`: the remote SAM3 service (D-013), prompt `a sock lying on the floor`, masks → blobs
  that look like a sock on the floor; the classic detector when the service fails.
- `seg`: MuJoCo's segmentation of the sock geoms, an oracle for testing the steering alone.
  Only the simulator has it.

Every detector returns `Detection`s sorted best first.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np

from sorter.nav.camera import Frame

MIN_AREA_M2 = 0.002  # a sock seen from above covers ~0.005-0.02 m²; at a grazing angle its
# height smears it over more floor, so the upper bound is loose
MAX_AREA_M2 = 0.15
MAX_EXTENT_M = 0.40
MIN_PIXELS = 25


@dataclass
class Detection:
    u: float  # blob center, px
    v: float
    bbox: tuple[int, int, int, int]  # x, y, w, h px
    score: float
    x: float | None  # the sock's center on the floor, rover frame (m); None if unknown
    y: float | None
    area_m2: float | None
    source: str
    mask: np.ndarray | None = None  # HxW bool

    @property
    def distance(self) -> float | None:
        return None if self.x is None else float(np.hypot(self.x, self.y))

    @property
    def bearing_deg(self) -> float | None:
        return None if self.x is None else float(np.degrees(np.arctan2(self.y, self.x)))

    def summary(self) -> dict:
        r = lambda v, n=3: None if v is None else round(float(v), n)  # noqa: E731
        return {
            "u": round(self.u),
            "v": round(self.v),
            "bbox": list(self.bbox),
            "score": r(self.score, 2),
            "x_m": r(self.x),
            "y_m": r(self.y),
            "distance_m": r(self.distance),
            "bearing_deg": r(self.bearing_deg, 1),
            "area_m2": r(self.area_m2, 4),
            "source": self.source,
        }


def from_mask(frame: Frame, mask: np.ndarray, score: float, source: str) -> Detection | None:
    """A blob → its floor position: the center of the floor cells it covers (depth), else the
    floor-plane ray through its center."""
    ys, xs = np.nonzero(mask)
    if xs.size < MIN_PIXELS:
        return None
    u, v = float(xs.mean()), float(ys.mean())
    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()
    pts = _mask_points(frame, mask)
    area = None
    if pts is not None:
        cx, cy = _floor_centroid(frame, mask, pts)
        area = _floor_area(frame, mask, pts)
    else:
        p = frame.floor_point(u, v)
        cx, cy = (float(p[0]), float(p[1])) if p is not None else (None, None)
        if p is not None:
            area = _floor_area(frame, mask, None)
    return Detection(
        u,
        v,
        (int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)),
        score,
        cx,
        cy,
        area,
        source,
        mask,
    )


CELL_M = 0.01


def _floor_centroid(frame: Frame, mask: np.ndarray, pts: np.ndarray) -> tuple[float, float]:
    """The blob's center on the floor: the mean of the 1 cm floor cells it covers.

    A plain mean or median over pixels leans toward the camera (near parts cover more
    pixels): on a 30 cm sock seen at 0.5 m that is ~10 cm. Mask pixels without depth (the
    bands outside the stereo FOV, plain patches) count by where their ray meets the floor.
    """
    xy = pts[:, :2]
    rest = mask & (frame.depth_mm == 0)
    if np.any(rest):
        ys, xs = np.nonzero(rest)
        K, T = frame.K, frame.T_rover_cam
        d = np.stack([(xs - K.cx) / K.fx, (ys - K.cy) / K.fy, np.ones(xs.size)], 1) @ T[:3, :3].T
        o = T[:3, 3]
        down = d[:, 2] < -1e-6
        t = -o[2] / d[down, 2]
        fl = o[:2] + d[down, :2] * t[:, None]
        xy = np.concatenate([xy, fl[np.hypot(fl[:, 0], fl[:, 1]) < 6.0]])
    cells = np.unique(np.floor(xy / CELL_M).astype(np.int64), axis=0)
    c = (cells.mean(axis=0) + 0.5) * CELL_M
    return float(c[0]), float(c[1])


def _mask_points(frame: Frame, mask: np.ndarray) -> np.ndarray | None:
    m = mask & (frame.depth_mm > 0)
    if np.count_nonzero(m) < MIN_PIXELS:
        return None
    ys, xs = np.nonzero(m)
    z = frame.depth_mm[ys, xs].astype(np.float32) / 1000
    K = frame.K
    pc = np.stack([(xs - K.cx) / K.fx * z, (ys - K.cy) / K.fy * z, z], 1)
    return pc @ frame.T_rover_cam[:3, :3].T + frame.T_rover_cam[:3, 3]


def _floor_area(frame: Frame, mask: np.ndarray, pts) -> float:
    """The blob's area on the floor: pixel count × the floor area one pixel covers there."""
    ys, xs = np.nonzero(mask)
    u, v = float(xs.mean()), float(ys.mean())
    p = frame.floor_point(u, v)
    if p is None:
        return float("inf")
    o, d = frame.ray(u, v)
    dist = float(np.linalg.norm(p - o))
    # a pixel at range r covers (r/f)² perpendicular to the ray, 1/cos(incidence) on the floor
    cos_inc = max(abs(d[2]), 0.05)
    return xs.size * (dist / frame.K.fx) ** 2 / cos_inc


# --- classic ---

# a sock on the floor, from the models and the labeled frames (runs/tools in the nav agents' copy)
SOCK_MAX_LEN_M = 0.42  # flat: leg + foot bent 55-80 deg, up to ~0.39 m on the floor
SOCK_MIN_LEN_M = 0.05
SOCK_MAX_WIDTH_M = 0.20
SOCK_MAX_H_M = 0.06  # bunched ~0.05 m; balls, shoes, toys and furniture stand taller
RAISED_MAX_H_M = 0.075
TALL_H_M = 0.09  # a neighbour this high belongs to something that is not a sock
DARK_RATIO = 0.55  # lightness / the floor's below this: no shadow is that dark
DARK_FLOOR_L = 75.0  # the dark cue only on floors darker than this (Lab L, 0-255): dim rooms
RAISED_RANGE_M = 2.0  # nearer than this a sock stands out of the floor in the depth


def _raised_thr(r: np.ndarray | float) -> np.ndarray | float:
    """Height (m) above which a depth point is off the floor at horizontal range r: the
    stereo's depth error grows with r², the floor itself scatters ~2 mm near by."""
    return 0.004 + 0.0012 * np.square(r)


@dataclass
class _Geo:
    """Per-frame geometry the blob tests share."""

    P: np.ndarray  # HxWx3 rover-frame points, NaN without depth
    valid: np.ndarray  # HxW has depth
    XY: np.ndarray  # HxWx2 footprint: the 3D point, else where the ray meets the floor
    R: np.ndarray  # HxW horizontal range of XY (inf above the horizon)


def _geometry(frame: Frame) -> _Geo:
    P = frame.points()
    valid = ~np.isnan(P[..., 2])
    h, w = valid.shape
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    d = np.stack(
        [(uu - frame.K.cx) / frame.K.fx, (vv - frame.K.cy) / frame.K.fy, np.ones_like(uu)], -1
    )
    d = d @ frame.T_rover_cam[:3, :3].T
    o = frame.T_rover_cam[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[..., 2] < -1e-6, -o[2] / d[..., 2], np.nan)
    fxy = o[:2] + d[..., :2] * t[..., None]
    XY = np.where(valid[..., None], P[..., :2], fxy)
    R = np.hypot(XY[..., 0], XY[..., 1])
    R = np.where(np.isnan(R), np.inf, R)
    return _Geo(P, valid, XY, R)


class ClassicDetector:
    """No model: things that stand a little out of the floor (depth) or differ in color from the
    floor around them, kept if their footprint on the floor, their height and what surrounds
    them fit a sock.

    Two cues, one blob test:
    - raised: depth points 0.5-7.5 cm above the floor (flat socks are ~1 cm thick). This finds a
      sock whatever its color, and it ignores patterns painted on the floor (rugs, tile joints).
    - color: Lab contrast against a wide median of the image, for socks too far for the depth.
    A blob passes if its footprint is sock-sized (length ≤ 0.42 m, width ≤ 0.20 m), nothing in
    it stands taller than a sock, no tall thing (wall, furniture, ball, shoe) touches it, and,
    where the depth could see a sock's thickness, it is raised.
    """

    name = "classic"

    def __init__(self, contrast: float = 38.0, blur_px: int = 201):
        self.contrast = contrast
        self.blur_px = blur_px | 1

    def detect(self, frame: Frame) -> list[Detection]:
        geo = _geometry(frame)
        dist, raised, color, dark = self.cues(frame, geo)
        out = []
        # dark blobs apart from the others: a black sock on a rug must not merge with the rug's
        # pattern into one blob too big for a sock
        dark_zone = cv2.dilate(dark, np.ones((5, 5), np.uint8)).astype(bool)
        other = ((raised | color).astype(bool) & ~dark_zone).astype(np.uint8)
        for cand in (dark, other):
            n, labels, stats, _ = cv2.connectedComponentsWithStats(cand, 8)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS:
                    continue
                mask = labels == i
                det = self._blob(frame, geo, mask, dist, raised)
                if det is not None:
                    out.append(det)
                    continue
                # too big or failed as a whole (a sock on a rug pattern, touching a tile joint):
                # its raised parts on their own
                sub = (raised.astype(bool) & mask).astype(np.uint8)
                m2, lab2, st2, _ = cv2.connectedComponentsWithStats(sub, 8)
                for j in range(1, m2):
                    if st2[j, cv2.CC_STAT_AREA] >= MIN_PIXELS:
                        det = self._blob(frame, geo, lab2 == j, dist, raised)
                        if det is not None:
                            out.append(det)
        return sorted(out, key=lambda d: -d.score)

    def cues(
        self, frame: Frame, geo: _Geo
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """(color contrast per pixel, raised mask, color mask, dark mask)."""
        z = geo.P[..., 2]
        with np.errstate(invalid="ignore"):
            raised = geo.valid & (z > _raised_thr(geo.R)) & (z < RAISED_MAX_H_M)
            floorish = np.where(geo.valid, z < 0.06, np.isfinite(geo.R) & (geo.R < 6.0))
        dist, dark = self._contrast(frame, floorish)
        k3 = np.ones((3, 3), np.uint8)
        raised = cv2.morphologyEx(raised.astype(np.uint8), cv2.MORPH_OPEN, k3)
        raised = cv2.morphologyEx(raised, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        color = (dist > self.contrast) & floorish
        color = cv2.morphologyEx(color.astype(np.uint8), cv2.MORPH_OPEN, k3)
        color = cv2.morphologyEx(color, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        # thin dark lines (tile joints, plank seams) are not socks: open wider than for color
        dark = cv2.morphologyEx(
            (dark & floorish).astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
        )
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        return dist, raised, color, dark

    def _contrast(self, frame: Frame, floorish: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(Lab contrast against the floor around each pixel, dark mask). A pixel is dark when
        it is far darker than the floor, around it or as a whole (a big dark sock close up
        darkens the local median itself): a black sock on a dark carpet in dim light, which
        has no depth to be raised in and too little color to stand out otherwise."""
        lab = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        small = cv2.resize(lab, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA)
        k = max(3, (self.blur_px // 4) | 1)
        bg = np.stack(
            [
                cv2.medianBlur(np.ascontiguousarray(small[..., i]).astype(np.uint8), k)
                for i in range(3)
            ],
            -1,
        ).astype(np.float32)
        bg = cv2.resize(bg, (lab.shape[1], lab.shape[0]), interpolation=cv2.INTER_LINEAR)
        L = lab[..., 0]
        floor_L = float(np.median(L[floorish])) if np.any(floorish) else float(np.median(L))
        ref = np.maximum(bg[..., 0], floor_L)
        dark = ((L + 5) / (ref + 5) < DARK_RATIO) & (ref - L > 12)
        if floor_L >= DARK_FLOOR_L:  # a bright floor: color finds dark socks, the cue only
            dark[:] = False  # adds shadows and seams there
        diff = lab - bg
        diff[..., 0] *= np.where(dark, 1.0, 0.6)  # lightness counts less than color (shadows)
        return np.linalg.norm(diff, axis=-1), dark

    def _blob(self, frame, geo: _Geo, mask, dist, raised) -> Detection | None:
        f = blob_features(geo, mask, raised)
        if f is None or not self._is_sock(f):
            return None
        det = from_mask(frame, mask, 0.0, self.name)
        if det is None or det.x is None or (det.area_m2 or 0.0) < MIN_AREA_M2:
            return None
        color = float(np.clip(dist[mask].mean() / (2 * self.contrast), 0, 1))
        det.score = float(np.clip(0.5 * color + 0.5 * f["raised_frac"] / 0.5, 0, 1))
        if f["border"]:
            det.score *= 0.7  # partly out of view: its size is a guess
        return det

    @staticmethod
    def _is_sock(f: dict) -> bool:
        if not (SOCK_MIN_LEN_M <= f["length"] <= SOCK_MAX_LEN_M):
            return False
        if f["width"] > SOCK_MAX_WIDTH_M or f["h95"] > SOCK_MAX_H_M or f["tall_near"] > 0.1:
            return False
        if f["n_depth"] >= 25 and f["range"] < RAISED_RANGE_M:
            return f["raised_frac"] >= 0.2  # the depth can see a sock here: it must stand out
        return True

    @staticmethod
    def _on_floor(frame: Frame) -> np.ndarray:
        g = _geometry(frame)
        return np.where(g.valid, g.P[..., 2] < 0.06, np.isfinite(g.R) & (g.R < 6.0))


def blob_features(geo: _Geo, mask: np.ndarray, raised: np.ndarray) -> dict | None:
    """A blob's footprint on the floor (PCA length and width, m), height (m) and surroundings."""
    xy = geo.XY[mask]
    ok = ~np.isnan(xy[:, 0])
    if ok.sum() < MIN_PIXELS:
        return None
    xy = xy[ok]
    c = np.median(xy, 0)
    rng = float(np.hypot(*c))
    ev, evec = np.linalg.eigh(np.cov((xy - c).T) + 1e-9 * np.eye(2))
    proj = (xy - c) @ evec
    lo, hi = np.percentile(proj, [3, 97], axis=0)
    width, length = (hi - lo).tolist()
    zv = geo.P[..., 2][mask & geo.valid]
    h95 = float(np.percentile(zv, 95)) if zv.size >= 10 else 0.0
    # a tall thing right next to it (wall, furniture, a ball's top): the ring around the blob,
    # reaching further up the image, where a wall or a ball rises above its foot
    kern = np.zeros((41, 11), np.uint8)
    kern[15:] = 1  # rows 15-40 of the kernel, anchor 20: the ring reaches 20 px up, 5 down
    ring = cv2.dilate(mask.astype(np.uint8), kern).astype(bool) & ~mask
    rp = geo.P[ring & geo.valid]
    tall_near = 0.0
    if len(rp):
        near = np.hypot(rp[:, 0] - c[0], rp[:, 1] - c[1]) < max(0.25, length)
        tall_near = float(np.mean((rp[:, 2] > TALL_H_M) & near))
    ys, xs = np.nonzero(mask)
    h, w = mask.shape
    border = bool(xs.min() <= 1 or xs.max() >= w - 2 or ys.max() >= h - 2)
    n_depth = int(zv.size)
    return {
        "length": length,
        "width": width,
        "range": rng,
        "h95": h95,
        "n_depth": n_depth,
        "raised_frac": float(np.count_nonzero(raised.astype(bool) & mask) / max(n_depth, 1)),
        "tall_near": tall_near,
        "border": border,
    }


# --- SAM3 ---

# SAM3 scores the rendered socks low for the bare noun (`sock`: 0.06-0.56, mostly < 0.5) and
# much higher for a phrase (`a sock lying on the floor`: 0.57-0.96); flat towels score high
# too (0.3-0.85), but their floor area is 3-10x a sock's, so the area test removes them.
PROMPT_TEMPLATE = "a {} lying on the floor"
SAM_CONFIRM = 0.35  # a new target; true socks far/mid range were >= 0.57, junk <= 0.25
SAM_THRESHOLD = 0.15  # returned at all: a tracked sock close up can drop to ~0.25 (bunched)
SAM_MAX_AREA_M2 = 0.15  # socks 0.03-0.08 m² (two touching 0.13); towels 0.26-0.6
SAM_MAX_HEIGHT_M = 0.10  # median height of the mask's 3D points: a box, a ball are taller
EDGE_PX = 3  # a mask touching the left/right border is clipped: its size can't be judged
SAM_TIMEOUT_S = 6.0
SAM_RETRIES = 1


class Sam3Detector:
    """The remote SAM3 service (config: `color_classifier.sam`, key in `config/local.yaml`).

    `prompt` is the object noun; a bare noun is wrapped in `PROMPT_TEMPLATE`. Masks are
    de-duplicated, then kept if they look like a sock on the floor (score, floor area, height,
    not clipped by the image border). Detections down to `threshold` are returned; `confirm` is
    the score a new target needs (the controller follows a tracked one below it). If the
    service fails (after `SAM_RETRIES` retries), the frame goes to `fallback` (the classic
    detector) and `failures` counts it. `calls` and `latency_s` time every request.
    """

    name = "sam3"

    def __init__(
        self,
        sam_cfg,
        prompt: str = "sock",
        threshold: float = SAM_THRESHOLD,
        confirm: float = SAM_CONFIRM,
        fallback=None,
    ):
        from sorter.color_classifier.segmenter import SamSegmenter

        if sam_cfg is None:
            raise ValueError("sam3 needs the color_classifier.sam config")
        self.prompt = PROMPT_TEMPLATE.format(prompt) if " " not in prompt.strip() else prompt
        self.threshold, self.confirm = threshold, confirm
        cfg = sam_cfg.model_copy(
            update={
                "prompts": [self.prompt],
                "threshold": threshold,
                "timeout_s": min(sam_cfg.timeout_s, SAM_TIMEOUT_S),
            }
        )
        self.segmenter = SamSegmenter(cfg)
        self.fallback = fallback if fallback is not None else ClassicDetector()
        self.calls = 0
        self.failures = 0
        self.latency_s: list[float] = []
        self.last_error: str | None = None

    def detect(self, frame: Frame) -> list[Detection]:
        insts = self._segment(frame)
        if insts is None:  # the service is down: this frame goes to the classic detector
            dets = self.fallback.detect(frame)
            for d in dets:
                d.source = f"{self.fallback.name} (sam3 failed)"
            return dets
        out: list[Detection] = []
        for inst in sorted(insts, key=lambda i: -i.score):
            if inst.score < self.threshold:
                continue
            if any(_overlap(inst.mask, d.mask) > 0.5 for d in out):
                continue  # a duplicate or a part of a better mask
            det = from_mask(frame, inst.mask, inst.score, self.name)
            if det is None or not self._sock_like(frame, det):
                continue
            out.append(det)
        return out

    def _segment(self, frame: Frame):
        from sorter.color_classifier.segmenter import SegmentationError

        bgr = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
        for _ in range(1 + SAM_RETRIES):
            t = time.monotonic()
            self.calls += 1
            try:
                insts = self.segmenter.segment(bgr)
            except SegmentationError as e:
                self.last_error = str(e)
                continue
            finally:
                self.latency_s.append(time.monotonic() - t)
            return insts
        self.failures += 1
        return None

    @staticmethod
    def _sock_like(frame: Frame, det: Detection) -> bool:
        x, y, w, h = det.bbox
        if x <= EDGE_PX or x + w >= frame.rgb.shape[1] - EDGE_PX:
            return False
        if det.area_m2 is None or not (MIN_AREA_M2 <= det.area_m2 <= SAM_MAX_AREA_M2):
            return False
        pts = _mask_points(frame, det.mask)
        return pts is None or float(np.median(pts[:, 2])) <= SAM_MAX_HEIGHT_M

    def stats(self) -> dict:
        lat = np.array(self.latency_s) if self.latency_s else np.zeros(1)
        return {
            "calls": self.calls,
            "failures": self.failures,
            "latency_median_s": round(float(np.median(lat)), 2),
            "latency_max_s": round(float(lat.max()), 2),
        }


def _overlap(a: np.ndarray, b: np.ndarray) -> float:
    """Intersection over the smaller mask."""
    inter = np.count_nonzero(a & b)
    return inter / max(1, min(np.count_nonzero(a), np.count_nonzero(b)))


# --- oracle ---


class SegDetector:
    """MuJoCo's segmentation of the socks (sim only)."""

    name = "seg"

    KEEP = 16  # segmentations of the last frames (a `scan` returns 8)

    def __init__(self, camera):
        self.camera = camera
        m = camera.sim.model
        self.sock_geoms = [m.geom(f"sock{i}").id for i in range(len(camera.sim.spec.socks))]
        # segment every frame when it is captured, so frames of a `scan` (taken at other
        # headings) get their own segmentation, not the one of the pose the rover ends in
        self._seg: dict[int, np.ndarray] = {}
        capture = camera.capture

        def capture_with_seg() -> Frame:
            f = capture()
            self._seg[f.index] = camera.segmentation()
            for k in [k for k in self._seg if k <= f.index - self.KEEP]:
                del self._seg[k]
            return f

        camera.capture = capture_with_seg

    def detect(self, frame: Frame) -> list[Detection]:
        seg = self._seg.get(frame.index)
        if seg is None:
            seg = self.camera.segmentation()
        out = []
        for g in self.sock_geoms:
            det = from_mask(frame, seg == g, 1.0, self.name)
            if det is not None:
                out.append(det)
        return sorted(out, key=lambda d: -(d.mask.sum()))


def make(name: str, camera=None, sam_cfg=None, prompt: str = "sock"):
    if name == "classic":
        return ClassicDetector()
    if name == "sam3":
        return Sam3Detector(sam_cfg, prompt)
    if name == "seg":
        return SegDetector(camera)
    raise ValueError(f"unknown detector {name!r}: classic | sam3 | seg")


def overlay(frame: Frame, dets: list[Detection]) -> np.ndarray:
    img = frame.rgb.copy()
    for i, d in enumerate(dets):
        color = (0, 255, 0) if i == 0 else (255, 200, 0)
        if d.mask is not None:
            cnts, _ = cv2.findContours(
                d.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(img, cnts, -1, color, 2)
        cv2.drawMarker(img, (int(d.u), int(d.v)), color, cv2.MARKER_CROSS, 14, 2)
        label = f"#{i} {d.score:.2f}"
        if d.distance is not None:
            label += f" {d.distance:.2f}m {d.bearing_deg:+.0f}deg"
        cv2.putText(
            img,
            label,
            (d.bbox[0], max(12, d.bbox[1] - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
    return img

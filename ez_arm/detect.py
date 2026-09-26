"""T-shirt detection: SAM3 masks (the team's service, block 4 client) + depth → base-frame target.

Every mask is lifted to 3D with the aligned depth and the camera pose from FK, then kept only if
it lies on the table inside the reachable box: this drops people, chairs and laptops that SAM3
also calls "clothing". The target is the largest surviving blob; the grasp point is the highest
cloth point in its middle (like block 4's regrasp point), and the grasp yaw is across the blob's
short side so the fingers pinch the fabric, not slide along it.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import cv2
import numpy as np
from sorter.color_classifier.segmenter import SamSegmenter
from sorter.core.config import load_config

from ez_arm.frames import REPO, CameraFrame, deproject

# Only cloth in this box (base frame, m) is a target.
TABLE_BOX = {"x": (0.10, 0.50), "y": (-0.40, 0.40), "z": (-0.03, 0.15)}


@dataclass
class Target:
    xyz: np.ndarray  # grasp point on the cloth top, base frame, m
    yaw: float  # rad about base z: direction of the blob's long axis
    area_m2: float
    score: float
    px: tuple[int, int]
    mask: np.ndarray = field(repr=False)
    prompt: str = ""


class ShirtDetector:
    def __init__(self, prompts=("t-shirt", "clothing"), config_dir: str | None = None):
        cfg = load_config(config_dir or os.path.join(REPO, "config"))
        sam = cfg.color_classifier.sam.model_copy(update={"prompts": list(prompts)})
        self.prompts = list(prompts)
        self.seg = SamSegmenter(sam)
        self.cam = CameraFrame()

    def detect(self, rgbd, q, box=TABLE_BOX, min_area_m2=0.003) -> tuple[list[Target], np.ndarray]:
        """→ (targets largest first, debug image)."""
        insts = []
        for p in self.prompts:  # one request per prompt; keep which prompt found it
            self.seg.cfg = self.seg.cfg.model_copy(update={"prompts": [p]})
            insts += [(i, p) for i in self.seg.segment(rgbd.color)]
        depth = rgbd.depth_m
        vv, uu = np.nonzero(depth > 0)
        pts = self.cam.to_base(q, deproject(uu, vv, depth[vv, uu], rgbd.K))
        P = np.full(depth.shape + (3,), np.nan, np.float32)
        P[vv, uu] = pts
        inbox = np.ones(depth.shape, bool)
        for k, a in enumerate("xyz"):
            lo, hi = box[a]
            with np.errstate(invalid="ignore"):
                inbox &= (P[..., k] >= lo) & (P[..., k] <= hi)
        targets: list[Target] = []
        for inst, prompt in sorted(insts, key=lambda ip: -ip[0].score):
            m = inst.mask & inbox
            if any((m & t.mask).sum() > 0.5 * m.sum() for t in targets if m.sum()):
                continue  # duplicate of a better one (other prompt)
            n = int(m.sum())
            if n < 300 or n < 0.6 * inst.mask.sum():  # mostly off the table = not our shirt
                continue
            xyz = P[m]
            # area on the table: pixel footprint ≈ (z/f)² per pixel
            area = float(np.sum((depth[m] / rgbd.K[0]) * (depth[m] / rgbd.K[1])))
            if area < min_area_m2:
                continue
            gp, px = self._grasp_point(m, P)
            c = xyz[:, :2] - xyz[:, :2].mean(0)
            w, vecs = np.linalg.eigh(c.T @ c)
            long_axis = vecs[:, 1]
            targets.append(
                Target(gp, float(np.arctan2(long_axis[1], long_axis[0])), area, inst.score, px, m, prompt)
            )
        targets.sort(key=lambda t: -t.area_m2)
        return targets, self.debug_image(rgbd.color, targets)

    @staticmethod
    def _grasp_point(m: np.ndarray, P: np.ndarray):
        """Highest point (within 1.5 cm) that is deep inside the blob."""
        dist = cv2.distanceTransform(m.astype(np.uint8), cv2.DIST_L2, 5)
        z = np.where(m, P[..., 2], -np.inf)
        inner = dist >= 0.4 * dist.max()
        ztop = np.nanmax(np.where(inner, z, -np.inf))
        cand = inner & (z >= ztop - 0.015)
        v, u = np.unravel_index(int(np.argmax(np.where(cand, dist, -1))), m.shape)
        win = P[max(v - 4, 0) : v + 5, max(u - 4, 0) : u + 5].reshape(-1, 3)
        win = win[np.isfinite(win).all(1)]
        return np.median(win, axis=0), (int(u), int(v))

    @staticmethod
    def debug_image(color, targets):
        img = color.copy()
        for i, t in enumerate(targets):
            c = (0, 255, 0) if i == 0 else (0, 160, 255)
            tint = img.copy()
            tint[t.mask] = c
            img = cv2.addWeighted(tint, 0.35, img, 0.65, 0)
            cv2.drawMarker(img, t.px, (0, 0, 255), cv2.MARKER_CROSS, 28, 2)
            x, y, z = t.xyz
            cv2.putText(img, f"{t.prompt} ({x:.2f},{y:.2f},{z:.2f})", (t.px[0] + 8, t.px[1] - 8), 0, 0.5, (0, 0, 255), 2)
        return img

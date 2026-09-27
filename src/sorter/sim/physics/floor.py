"""The floor's texture: herringbone oak parquet like the room the rover runs in.

A seamless square tile in the planks' own axes (the floor geom is turned 45° so the pattern runs
diagonally, as on the photos). Planks `PLANK_MM` = W × nW; the herringbone repeats every 2nW
along both axes. Each plank gets its own shade and grain; the joints are dark lines.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

CACHE = Path(__file__).resolve().parents[4] / "data" / "cache"
PLANK_MM = (70, 5)  # width, length in widths
PX_PER_MM = 1.0
PERIODS = 2  # herringbone periods per tile side: fewer repeats in view
# oak shades, BGR: from the photos of the floor (warm orange-brown, some planks paler)
OAK_BGR = ((62, 118, 186), (78, 138, 200), (52, 102, 168), (70, 128, 190), (90, 150, 206))


def tile_mm() -> float:
    w, n = PLANK_MM
    return 2 * n * w * PERIODS


def parquet_texture(seed: int = 0) -> Path:
    """A PNG of the parquet tile (cached)."""
    path = CACHE / f"parquet_{PLANK_MM[0]}x{PLANK_MM[1]}_{PERIODS}_{PX_PER_MM:g}_{seed}.png"
    if not path.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), parquet_image(seed))
    return path


def parquet_image(seed: int = 0) -> np.ndarray:
    w, n = PLANK_MM
    period = 2 * n * PERIODS  # the tile side in plank widths
    size = int(round(tile_mm() * PX_PER_MM))
    mm = (np.arange(size) + 0.5) / PX_PER_MM
    px, py = np.meshgrid(mm, mm)  # x right, y down in the image
    cx, cy = np.floor(px / w).astype(int), np.floor(py / w).astype(int)
    d = (cx - cy) % (2 * n)
    horiz = d < n
    # horizontal plank: starts at cell (sx, cy), runs n cells along x
    sx = cx - d
    # vertical plank: at column cx, starts at row k + 1, runs n cells along y
    k = cy - (2 * n - d)
    along = np.where(horiz, px - sx * w, py - (k + 1) * w)  # mm from the plank's start
    across = np.where(horiz, py - cy * w, px - cx * w)  # mm from the plank's side
    # one key per plank in the tile, the same for the plank's copies across the tile's edges
    copy_h = ((sx - cy) // (2 * n)) % PERIODS
    copy_v = ((cx - k) // (2 * n)) % PERIODS
    nh = period * PERIODS
    key = np.where(horiz, (cy % period) * PERIODS + copy_h, nh + (cx % period) * PERIODS + copy_v)

    rng = np.random.default_rng(seed)
    nk = 2 * nh
    base = np.array(OAK_BGR, float)[rng.integers(len(OAK_BGR), size=nk)]
    base *= rng.uniform(0.9, 1.08, size=(nk, 1))
    freq = rng.uniform(0.25, 0.6, nk)  # grain lines per mm, across the plank
    phase = rng.uniform(0, 2 * np.pi, nk)
    wob = rng.uniform(0.004, 0.012, nk)  # how fast the grain wanders along the plank
    L = n * w
    # the grain: fine lines along the plank that wander a little, and a slow shade change
    t = across + 3.0 * np.sin(along * wob[key] + phase[key])
    grain = 0.5 + 0.5 * np.sin(2 * np.pi * freq[key] * t + phase[key])
    fine = rng.normal(0, 1, (size, size))
    fine = cv2.GaussianBlur(fine, (0, 0), 1.2) * 0.04
    shade = 1 + 0.06 * np.sin(2 * np.pi * along / L * rng.uniform(0.5, 1.5) + phase[key])
    img = base[key] * (shade * (0.9 + 0.12 * grain) + fine)[..., None]
    # the joints: dark lines at every plank edge
    edge = np.minimum(np.minimum(along, L - along), np.minimum(across, w - across))
    joint = np.clip(edge / 1.2, 0, 1) * 0.55 + 0.45
    img *= joint[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)

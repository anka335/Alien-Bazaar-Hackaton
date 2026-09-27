"""The load scene (stage A): socks lying on the floor around the rover.

`add` is called by `sorter.sim.physics.model.build` after the base (floor, rover, cargo box, arm)
is in place. It may add geoms and assets, and returns the cloth items to add.

A sock is the cloth grid in a sock's outline (`sock_shape`): the leg, a bend at the heel and
the foot with a rounded toe, ~200 × 90 mm lying flat, with shallow creases; sometimes bunched up.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from sorter.sim.config import SimConfig
from sorter.sim.physics.model import CLOTH_N, ItemSpec, palette_rgb

MIN_GAP_MM = 150.0  # socks don't start on top of each other (cloth passes through cloth)
ROVER_GAP_MM = 70.0  # a sock's center stays this far from the rover and the cargo box


def _spot(cfg: SimConfig, rng: np.random.Generator) -> tuple[float, float]:
    sc, lay = cfg.load, cfg.layout
    if sc.area == "view":
        x0, x1, y0, y1 = lay.floor_view.bounds(-sc.margin_mm)
        return rng.uniform(x0, x1), rng.uniform(y0, y1)
    r0, r1 = sc.reach_mm
    a = math.radians(sc.reach_deg)
    while True:
        r, t = math.sqrt(rng.uniform(r0**2, r1**2)), rng.uniform(-a, a)
        x, y = r * math.cos(t), r * math.sin(t)
        if not lay.body.contains(x, y, -ROVER_GAP_MM) and not lay.cargo.contains(
            x, y, -ROVER_GAP_MM
        ):
            return x, y


def sock_shape(rng: np.random.Generator, size_mm: tuple[float, float], bunched: bool) -> np.ndarray:
    """Rest shape of a sock (CLOTH_N² points, m, see `ItemSpec.rest_m`), centered on (0, 0).

    Rows of the grid run along the sock from the cuff to the toe. The leg (~55 %) is straight,
    the foot is turned by the heel's bend, the toe end is rounded. Creases: a few random waves.
    """
    n = CLOTH_N
    length, width = size_mm[0] / 1000, size_mm[1] / 1000
    bend = math.radians(rng.uniform(30, 65)) * rng.choice([-1, 1])
    s = np.linspace(0, length, n)  # along the centre line
    heel = 0.55 * length
    # the heel's bend happens over this length of the centre line: a radius of ~the sock's
    # width, so the grid's cells on the inside of the bend don't collapse
    arc = width * abs(bend)
    # heading of the centre line: 0 along the leg, turning smoothly to `bend` at the heel
    k = np.clip((s - heel + arc / 2) / arc, 0, 1)
    heading = bend * (3 * k**2 - 2 * k**3)
    ds = np.diff(s, prepend=0.0)
    cx = np.cumsum(ds * np.cos(heading))
    cy = np.cumsum(ds * np.sin(heading))
    # width: the leg and foot as wide as the sock, the toe rounded
    toe = np.clip((s - (length - width / 2)) / (width / 2), 0, 1)
    half = width / 2 * np.sqrt(1 - 0.8 * toe**2)
    v = np.linspace(-1, 1, n)
    x = cx[None, :] - np.sin(heading)[None, :] * v[:, None] * half[None, :]
    y = cy[None, :] + np.cos(heading)[None, :] * v[:, None] * half[None, :]
    if bunched:  # pushed together into a lump with deep folds
        x, y = x * 0.6, y * 0.6
        fold = 0.03
    else:
        fold = 0.012
    z = np.zeros_like(x)
    side = max(length, width)
    for _ in range(5):
        f = rng.uniform(2, 6) * np.pi / side
        a, ph = rng.uniform(0, np.pi), rng.uniform(0, 2 * np.pi)
        z += np.sin(f * (x * np.cos(a) + y * np.sin(a)) + ph)
    z = fold * (z - z.min()) / np.ptp(z)
    x, y = x - x.mean(), y - y.mean()
    return np.c_[x.ravel(), y.ravel(), z.ravel()]


def add(world: ET.Element, asset: ET.Element, cfg: SimConfig, rng: np.random.Generator):
    sc, lay = cfg.load, cfg.layout
    placed: list[tuple[float, float]] = []
    items = []
    for color in sc.socks:
        # the first spot at least MIN_GAP_MM from the others, else the farthest one tried
        best, gap = (0.0, 0.0), -1.0
        for _ in range(100):
            x, y = _spot(cfg, rng)
            d = min((math.hypot(x - a, y - b) for a, b in placed), default=math.inf)
            if d > gap:
                best, gap = (x, y), d
            if d >= MIN_GAP_MM:
                break
        x, y = best
        placed.append((x, y))
        rest = sock_shape(rng, sc.sock_mm, rng.random() < sc.bunched_prob)
        z = (lay.floor_z_mm + 8) / 1000
        items.append(
            ItemSpec(
                color,
                palette_rgb(color, rng),
                (x / 1000, y / 1000, z),
                rng.uniform(0, 2 * np.pi),
                rest_m=tuple(rest.ravel()),
            )
        )
    return items

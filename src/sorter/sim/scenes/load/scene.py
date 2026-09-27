"""The load scene (stage A): socks lying on the floor in front of the rover.

`add` is called by `sorter.sim.physics.model.build` after the base (floor, rover, cargo box, arm)
is in place. It may add geoms and assets, and returns the cloth items to add.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from sorter.sim.config import SimConfig
from sorter.sim.physics.model import ItemSpec, palette_rgb

MIN_GAP_MM = 90.0  # socks don't start on top of each other (cloth passes through cloth)


def add(world: ET.Element, asset: ET.Element, cfg: SimConfig, rng: np.random.Generator):
    sc, lay = cfg.load, cfg.layout
    x0, x1, y0, y1 = lay.floor_view.bounds(-sc.margin_mm)
    placed: list[tuple[float, float]] = []
    items = []
    for color in sc.socks:
        # the first spot at least MIN_GAP_MM from the others, else the farthest one tried
        best, gap = (0.0, 0.0), -1.0
        for _ in range(100):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            d = min((math.hypot(x - a, y - b) for a, b in placed), default=math.inf)
            if d > gap:
                best, gap = (x, y), d
            if d >= MIN_GAP_MM:
                break
        x, y = best
        placed.append((x, y))
        z = (lay.floor_z_mm + 8) / 1000
        items.append(
            ItemSpec(
                color, palette_rgb(color, rng), (x / 1000, y / 1000, z), rng.uniform(0, 2 * np.pi)
            )
        )
    return items

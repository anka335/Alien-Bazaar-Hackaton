"""The unload scene (stage B): the laundry bins of the station beside the rover, and socks in
the cargo box.

`add` is called by `sorter.sim.physics.model.build` after the base (floor, rover, cargo box, arm)
is in place. It may add geoms and assets, and returns the cloth items to add.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from sorter.core.types import ColorClass
from sorter.sim.config import SimConfig
from sorter.sim.physics.model import ItemSpec, palette_rgb, tray

BIN_RGBA = {
    ColorClass.LIGHT: "0.92 0.93 0.93 1",
    ColorClass.DARK: "0.22 0.23 0.24 1",
    ColorClass.COLORED: "0.14 0.55 0.67 1",
}
PILE_STEP_MM = 35.0  # socks in one box start this much above each other


def add(world: ET.Element, asset: ET.Element, cfg: SimConfig, rng: np.random.Generator):
    lay = cfg.layout
    fz = lay.floor_z_mm / 1000
    bins = lay.laundry
    t = bins.wall_t_mm / 1000
    half = bins.size_mm / 2000 - t
    for color, (x, y) in bins.centers_mm.items():
        x, y = x / 1000, y / 1000
        tray(
            world,
            f"laundry_{color.value}",
            (x - half, x + half, y - half, y + half),
            fz,
            fz + 0.005,
            fz + bins.height_mm / 1000,
            t,
            BIN_RGBA[color],
        )
    items = []
    cargo = lay.cargo
    sheet = tuple(v / 1000 for v in cfg.unload.sock_mm)
    in_rect: dict[str, int] = {}  # socks so far per compartment (or the whole box)
    for color, n in cfg.unload.cargo.items():
        own = color in cargo.compartments
        rect = cargo.compartment(color) if own else cargo
        key = color.value if own else "box"
        (cx, cy), (w, h) = rect.center_mm, rect.size_mm
        for _ in range(n):
            k = in_rect[key] = in_rect.get(key, 0) + 1
            z = cargo.floor_z_mm + 8 + PILE_STEP_MM * (k - 1)
            if own:  # a narrow compartment: in the middle, square to the walls
                dx, dy = rng.uniform(-5, 5), rng.uniform(-20, 20)
                yaw = rng.choice([0.0, np.pi / 2]) + rng.uniform(-0.1, 0.1)
            else:  # near the middle: the gathered sheet still has to fit between the walls
                dx, dy = rng.uniform(-0.15, 0.15) * w, rng.uniform(-0.15, 0.15) * h
                yaw = rng.uniform(0, np.pi)
            pos = ((cx + dx) / 1000, (cy + dy) / 1000, z / 1000)
            items.append(ItemSpec(color, palette_rgb(color, rng), pos, float(yaw), sheet))
    return items

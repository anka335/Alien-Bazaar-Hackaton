"""The unload scene (stage B): the laundry bins of the station beside the rover, and socks in
the cargo compartments.

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
SOCK_SHEET_M = (0.12, 0.12)  # smaller than the default cloth: it fits a compartment


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
    for color, n in cfg.unload.cargo.items():
        (cx, cy) = cargo.compartment(color).center_mm
        for i in range(n):
            z = (cargo.floor_z_mm + 8) / 1000 + 0.035 * i
            pos = (cx / 1000 + rng.uniform(-0.005, 0.005), cy / 1000 + rng.uniform(-0.02, 0.02), z)
            yaw = rng.choice([0.0, np.pi / 2]) + rng.uniform(-0.1, 0.1)  # square to the walls
            items.append(ItemSpec(color, palette_rgb(color, rng), pos, float(yaw), SOCK_SHEET_M))
    return items

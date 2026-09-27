"""The unload scene (stage B): the laundry bins of the station beside the rover, and socks in
the cargo box.

`add` is called by `sorter.sim.physics.model.build` after the base (floor, rover, cargo box, arm)
is in place. It may add geoms and assets, and returns the cloth items to add.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np

from sorter.core.types import ColorClass
from sorter.sim.config import SimConfig
from sorter.sim.physics.model import ItemSpec, box, palette_rgb, tray

BIN_RGBA = {
    ColorClass.LIGHT: "0.92 0.93 0.93 1",
    ColorClass.DARK: "0.22 0.23 0.24 1",
    ColorClass.COLORED: "0.14 0.55 0.67 1",
}
# the real rover's parts (`sim.unload.rover_parts`): [x0, x1, y0, y1, z0, z1] mm, arm frame
ELECTRONICS_MM = (-280.0, -90.0, -125.0, 20.0, 0.0, 70.0)  # power strip and adapters
BRACKET_MM = (-210.0, -50.0, 120.0, 280.0, -24.0, -20.0)  # holds the cargo box beside the deck
CARDBOARD_RGBA = "0.66 0.5 0.34 1"
Place = tuple[float, float, float]  # x, y (mm), yaw (rad)
PILE_STEP_MM = 35.0  # socks in one box start this much above each other


def station(cfg: SimConfig, rng: np.random.Generator) -> dict[ColorClass, Place]:
    """Where each laundry bin really stands: (x, y) of its center in mm and its yaw in rad, the
    layout's place moved by the parking and placement noise of `sim.unload`."""
    sc = cfg.unload
    centers = cfg.layout.laundry.centers_mm
    mid = np.mean(list(centers.values()), axis=0)
    shift = rng.uniform(-sc.station_mm, sc.station_mm, 2)
    turn = math.radians(rng.uniform(-sc.station_deg, sc.station_deg))
    c, s = math.cos(turn), math.sin(turn)
    out = {}
    for color, p in centers.items():
        d = np.asarray(p) - mid
        x, y = mid + shift + (c * d[0] - s * d[1], s * d[0] + c * d[1])
        x, y = (x, y) + rng.uniform(-sc.bin_mm, sc.bin_mm, 2)
        yaw = turn + math.radians(rng.uniform(-sc.bin_deg, sc.bin_deg))
        out[color] = (float(x), float(y), yaw)
    return out


def _turned(parent: ET.Element, part: ET.Element, x: float, y: float, yaw: float) -> None:
    """Add `part`'s geoms (built around the origin) to `parent`, turned by `yaw` about z and
    moved to (x, y) (m). They stay world-body geoms: the dashboard's 3D view shows those."""
    c, s = math.cos(yaw), math.sin(yaw)
    q = f"{math.cos(yaw / 2):.6g} 0 0 {math.sin(yaw / 2):.6g}"
    for g in part.findall("geom"):
        px, py, pz = (float(v) for v in g.get("pos").split())
        g.set("pos", f"{x + c * px - s * py:.6g} {y + s * px + c * py:.6g} {pz:.6g}")
        g.set("quat", q)
        parent.append(g)


def _rover_parts(world: ET.Element) -> None:
    for name, (x0, x1, y0, y1, z0, z1) in (
        ("electronics", ELECTRONICS_MM),
        ("cargo_bracket", BRACKET_MM),
    ):
        lo, hi = np.array([x0, y0, z0]) / 1000, np.array([x1, y1, z1]) / 1000
        box(world, name, lo, hi, "0.13 0.13 0.14 1" if name == "electronics" else "0.55 0.57 0.6 1")
    for g in world.iter("geom"):  # the base draws the cargo box grey; the real one is cardboard
        if (g.get("name") or "").startswith("cargo_") and g.get("name") != "cargo_bracket":
            g.set("rgba", CARDBOARD_RGBA)


def add(world: ET.Element, asset: ET.Element, cfg: SimConfig, rng: np.random.Generator):
    lay = cfg.layout
    if cfg.unload.rover_parts:
        _rover_parts(world)
    fz = lay.floor_z_mm / 1000
    bins = lay.laundry
    t = bins.wall_t_mm / 1000
    half = bins.size_mm / 2000 - t
    for color, (x, y, yaw) in station(cfg, rng).items():
        part = ET.Element("part")
        tray(
            part,
            f"laundry_{color.value}",
            (-half, half, -half, half),
            fz,
            fz + 0.005,
            fz + bins.height_mm / 1000,
            t,
            BIN_RGBA[color],
        )
        _turned(world, part, x / 1000, y / 1000, yaw)
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

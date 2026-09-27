"""The unload scene (stage B): the laundry bins of the station in front of the rover, and socks
in the cargo box.

`add` is called by `sorter.sim.physics.model.build` after the base (floor, rover, cargo box, arm)
is in place. It may add geoms and assets, and returns the cloth items to add.
"""

from __future__ import annotations

import math
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
Place = tuple[float, float, float]  # x, y (mm), yaw (rad)
# socks in one box start this much above each other. Without `sim.cloth_collisions` cloth
# passes through cloth, so a sock dropped from higher up lands on nothing and stands up against
# a wall; low, they settle overlapping on the bottom like a loose pile. With it they must start
# apart (folds CLOTH_FOLD_M + twice the cloth's radius), or they start tangled and fly out
PILE_STEP_MM = 6.0
PILE_STEP_COLLIDING_MM = 45.0


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


def add(world: ET.Element, asset: ET.Element, cfg: SimConfig, rng: np.random.Generator):
    lay = cfg.layout
    fz = lay.floor_z_mm / 1000
    bins = lay.laundry
    t = bins.wall_t_mm / 1000
    for color, (x, y, yaw) in station(cfg, rng).items():
        half = bins.size(color) / 2000 - t
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
            step = PILE_STEP_COLLIDING_MM if cfg.cloth_collisions else PILE_STEP_MM
            z = cargo.floor_z_mm + 8 + step * (k - 1)
            if own:  # a narrow compartment: in the middle, square to the walls
                dx, dy = rng.uniform(-5, 5), rng.uniform(-20, 20)
                yaw = rng.choice([0.0, np.pi / 2]) + rng.uniform(-0.1, 0.1)
            else:  # near the middle: the gathered sheet still has to fit between the walls
                dx, dy = rng.uniform(-0.2, 0.2) * w, rng.uniform(-0.2, 0.2) * h
                yaw = rng.uniform(0, np.pi)
            pos = ((cx + dx) / 1000, (cy + dy) / 1000, z / 1000)
            items.append(
                ItemSpec(
                    color,
                    palette_rgb(color, rng),
                    pos,
                    float(yaw),
                    sheet,
                    young=cfg.unload.sock_young,
                    thickness_m=cfg.unload.sock_thickness_mm / 1000,
                )
            )
    return items

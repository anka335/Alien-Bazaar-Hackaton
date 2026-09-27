"""Seeded scenarios: where the rover starts, where the sock lies, what else is in the room.

`make(name, seed)` gives a `WorldSpec`; the same name and seed always give the same world.
Presets vary one difficulty at a time (bearing, distance, obstacles, clutter, light, floor)
and `random` mixes them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

FLOORS = ("wood", "carpet", "tile", "concrete", "plain", "rug")
LIGHTS = ("bright", "dim", "side")
# sock colors, RGB 0-1: the three laundry classes and a few typical patterns' base colors
SOCK_COLORS = {
    "white": (0.93, 0.93, 0.91),
    "black": (0.08, 0.08, 0.09),
    "gray": (0.55, 0.56, 0.58),
    "navy": (0.12, 0.16, 0.35),
    "red": (0.78, 0.12, 0.13),
    "green": (0.18, 0.55, 0.25),
    "yellow": (0.95, 0.80, 0.20),
    "pink": (0.95, 0.55, 0.70),
}


@dataclass(frozen=True)
class Item:
    """A loose thing on the floor. `kind`: sock_flat | sock_bunched | shirt | towel | ball | shoe |
    toy. `pos` world xy (m), `yaw` rad, `color` RGB 0-1, `seed` for its shape and pattern."""

    kind: str
    pos: tuple[float, float]
    yaw: float
    color: tuple[float, float, float]
    seed: int
    scale: float = 1.0


@dataclass(frozen=True)
class Obstacle:
    """Furniture: `kind` box | cylinder; `size` half extents (box) or (r, r, half height)."""

    kind: str
    pos: tuple[float, float]
    yaw: float
    size: tuple[float, float, float]
    color: tuple[float, float, float]


@dataclass(frozen=True)
class WorldSpec:
    name: str
    seed: int
    arena_half_m: float
    floor: str
    light: str
    rover: tuple[float, float, float]  # x, y, yaw
    socks: tuple[Item, ...]  # socks[0] is the goal
    distractors: tuple[Item, ...] = ()
    obstacles: tuple[Obstacle, ...] = ()
    notes: dict = field(default_factory=dict)
    # the unload station against a wall, AprilTags on its face (stage C): x, y, and the yaw of
    # the face's outward normal (into the room); None: no station
    station: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class Preset:
    distance_m: tuple[float, float] = (1.0, 2.0)
    bearing_deg: tuple[float, float] = (0.0, 20.0)  # |bearing| range, either side
    arena_half_m: float = 3.0
    floors: tuple[str, ...] = ("wood",)
    lights: tuple[str, ...] = ("bright",)
    sock_kinds: tuple[str, ...] = ("sock_flat",)
    sock_colors: tuple[str, ...] = tuple(SOCK_COLORS)
    obstacles: int = 0
    blocking: bool = False  # one obstacle right on the line between rover and sock
    distractors: int = 0
    extra_socks: int = 0
    near_wall: bool = False
    station: bool = False  # the unload station against the -x wall (the mission)


STATION_CLEAR_M = 0.9  # nothing starts closer than this to the station's center
PRESETS: dict[str, Preset] = {
    "easy": Preset(),
    "side": Preset(distance_m=(1.0, 2.5), bearing_deg=(40.0, 100.0)),
    "behind": Preset(distance_m=(1.0, 2.5), bearing_deg=(120.0, 180.0)),
    "far": Preset(distance_m=(3.0, 4.5), bearing_deg=(0.0, 30.0), arena_half_m=5.0),
    "near": Preset(distance_m=(0.55, 0.9), bearing_deg=(0.0, 45.0)),
    "obstacle": Preset(distance_m=(2.0, 3.2), bearing_deg=(0.0, 20.0), obstacles=2, blocking=True),
    "clutter": Preset(
        distance_m=(1.5, 3.0),
        bearing_deg=(0.0, 60.0),
        obstacles=3,
        distractors=5,
        floors=("wood", "carpet", "tile"),
    ),
    "wall": Preset(distance_m=(1.2, 2.5), bearing_deg=(0.0, 60.0), near_wall=True),
    "dim": Preset(
        distance_m=(1.0, 2.5),
        bearing_deg=(0.0, 40.0),
        lights=("dim",),
        floors=("carpet", "rug"),
        sock_colors=("black", "navy", "gray"),
    ),
    "plain": Preset(
        distance_m=(1.0, 2.5),
        bearing_deg=(0.0, 40.0),
        floors=("plain",),
        sock_colors=("white", "gray"),
    ),
    "bunched": Preset(
        distance_m=(1.0, 2.5),
        bearing_deg=(0.0, 60.0),
        sock_kinds=("sock_bunched",),
        floors=FLOORS,
    ),
    "multi": Preset(
        distance_m=(1.2, 2.8),
        bearing_deg=(0.0, 70.0),
        extra_socks=2,
        distractors=2,
        floors=("wood", "tile"),
    ),
    "mission": Preset(  # the full mission (stage C): several socks, then the station
        distance_m=(1.0, 2.0),
        bearing_deg=(0.0, 60.0),
        extra_socks=3,
        distractors=1,
        obstacles=1,
        floors=("wood", "tile"),
        station=True,
    ),
    "random": Preset(
        distance_m=(0.8, 3.5),
        bearing_deg=(0.0, 180.0),
        obstacles=3,
        distractors=3,
        floors=FLOORS,
        lights=LIGHTS,
        sock_kinds=("sock_flat", "sock_bunched"),
    ),
}

ROVER_RADIUS_M = 0.33  # a circle around the rover's center that clears the wheels
DISTRACTOR_KINDS = ("shirt", "towel", "ball", "shoe", "toy")


def make(name: str, seed: int) -> WorldSpec:
    if name not in PRESETS:
        raise ValueError(f"unknown scenario {name!r}; one of {', '.join(PRESETS)}")
    p = PRESETS[name]
    rng = np.random.default_rng([seed, sum(map(ord, name))])
    half = p.arena_half_m
    floor = str(rng.choice(p.floors))
    light = str(rng.choice(p.lights))
    for _ in range(200):
        world = _try(name, seed, p, rng, half, floor, light)
        if world is not None:
            return world
    raise RuntimeError(f"could not place scenario {name} seed {seed}")


def _try(name, seed, p: Preset, rng, half, floor, light) -> WorldSpec | None:
    yaw0 = float(rng.uniform(-math.pi, math.pi))
    d = float(rng.uniform(*p.distance_m))
    b = math.radians(float(rng.uniform(*p.bearing_deg))) * (1 if rng.random() < 0.5 else -1)
    if p.near_wall:
        # sock 0.12-0.30 m from a wall; the rover somewhere it can see it from
        wall = int(rng.integers(4))
        along = float(rng.uniform(-half + 0.8, half - 0.8))
        # the sock's center 0.22-0.34 m off the wall: a sock is up to ~0.4 m long, any closer
        # and it spawns inside the wall and is shot out
        off = half - float(rng.uniform(0.22, 0.34))
        sx, sy = [(off, along), (-off, along), (along, off), (along, -off)][wall]
        ang = math.atan2(sy, sx) + math.pi + b  # from the sock back into the room
        rx, ry = sx + d * math.cos(ang), sy + d * math.sin(ang)
        yaw0 = math.atan2(sy - ry, sx - rx) - b
    else:
        rx, ry = (float(v) for v in rng.uniform(-half * 0.4, half * 0.4, 2))
        sx, sy = rx + d * math.cos(yaw0 + b), ry + d * math.sin(yaw0 + b)
    lim = half - 0.35
    if not (abs(rx) < lim and abs(ry) < lim and abs(sx) < half - 0.1 and abs(sy) < half - 0.1):
        return None
    taken: list[tuple[float, float, float]] = [(rx, ry, ROVER_RADIUS_M + 0.1), (sx, sy, 0.2)]

    def sock(kind_pool, pos) -> Item:
        color = str(rng.choice(p.sock_colors))
        return Item(
            str(rng.choice(kind_pool)),
            pos,
            float(rng.uniform(-math.pi, math.pi)),
            SOCK_COLORS[color],
            int(rng.integers(1 << 30)),
            float(rng.uniform(0.9, 1.1)),
        )

    socks = [sock(p.sock_kinds, (sx, sy))]
    obstacles: list[Obstacle] = []
    if p.blocking:
        t = float(rng.uniform(0.45, 0.6))
        cx, cy = rx + t * (sx - rx), ry + t * (sy - ry)
        w = float(rng.uniform(0.25, 0.4))
        obstacles.append(
            Obstacle(
                "box",
                (cx, cy),
                math.atan2(sy - ry, sx - rx) + math.pi / 2,
                (w, float(rng.uniform(0.15, 0.25)), float(rng.uniform(0.2, 0.4))),
                _furniture_color(rng),
            )
        )
        taken.append((cx, cy, w + 0.1))
    for _ in range(p.obstacles - len(obstacles)):
        pos = _free(rng, half, taken, 0.45)
        if pos is None:
            return None
        if rng.random() < 0.65:
            size = (
                float(rng.uniform(0.15, 0.45)),
                float(rng.uniform(0.15, 0.35)),
                float(rng.uniform(0.15, 0.45)),
            )
            obstacles.append(
                Obstacle("box", pos, float(rng.uniform(0, math.pi)), size, _furniture_color(rng))
            )
        else:
            r = float(rng.uniform(0.08, 0.22))
            obstacles.append(
                Obstacle(
                    "cylinder",
                    pos,
                    0.0,
                    (r, r, float(rng.uniform(0.2, 0.45))),
                    _furniture_color(rng),
                )
            )
        taken.append((*pos, 0.45))
    for _ in range(p.extra_socks):
        pos = _free(rng, half, taken, 0.3)
        if pos is None:
            return None
        socks.append(sock(("sock_flat", "sock_bunched"), pos))
        taken.append((*pos, 0.3))
    distractors = []
    for _ in range(p.distractors):
        pos = _free(rng, half, taken, 0.3)
        if pos is None:
            return None
        kind = str(rng.choice(DISTRACTOR_KINDS))
        color = tuple(float(c) for c in rng.uniform(0.1, 0.95, 3))
        distractors.append(
            Item(
                kind, pos, float(rng.uniform(-math.pi, math.pi)), color, int(rng.integers(1 << 30))
            )
        )
        taken.append((*pos, 0.3))
    station = None
    if p.station:
        station = (-half + 0.16, 0.0, 0.0)
        things = [(rx, ry)] + [s.pos for s in socks] + [o.pos for o in obstacles]
        things += [d.pos for d in distractors]
        if any(math.hypot(x - station[0], y - station[1]) < STATION_CLEAR_M for x, y in things):
            return None
    return WorldSpec(
        name,
        seed,
        half,
        floor,
        light,
        (rx, ry, yaw0),
        tuple(socks),
        tuple(distractors),
        tuple(obstacles),
        {"distance_m": round(d, 2), "bearing_deg": round(math.degrees(b), 1)},
        station,
    )


def _free(rng, half, taken, r) -> tuple[float, float] | None:
    for _ in range(100):
        x, y = (float(v) for v in rng.uniform(-half + r + 0.1, half - r - 0.1, 2))
        if all(math.hypot(x - tx, y - ty) > r + tr for tx, ty, tr in taken):
            return x, y
    return None


def _furniture_color(rng) -> tuple[float, float, float]:
    palettes = [
        (0.45, 0.30, 0.18),
        (0.75, 0.72, 0.66),
        (0.25, 0.27, 0.30),
        (0.55, 0.20, 0.18),
        (0.30, 0.40, 0.55),
        (0.85, 0.85, 0.82),
    ]
    base = np.array(palettes[int(rng.integers(len(palettes)))])
    return tuple(float(c) for c in np.clip(base + rng.normal(0, 0.04, 3), 0, 1))

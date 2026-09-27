"""MJCF of the Leo Rover in a room.

The rover is the official leo_description 3.2.0 (Fictionlab, MIT): link frames, masses,
inertias, joint origins and axes are copied from its urdf/macros.xacro; the visual meshes are
converted by tools/convert_meshes.py. Rockers are free (the sim model, `leo_sim.urdf.xacro`)
and coupled like the real differential bar: the rocker joints are equal in their own frames,
and the left frame is turned by pi, so the two rockers pitch opposite ways.
"""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field
from pathlib import Path

ASSETS = Path(__file__).resolve().parent / "assets"

# leo_description, urdf/macros.xacro (rockers relative to base_link, wheels to their rocker)
BASE_HEIGHT = 0.19783  # base_footprint -> base_link
ROCKER_POS = {"L": (0.00263, 0.14167, -0.04731), "R": (0.00263, -0.14167, -0.04731)}
WHEELS = {  # name: (rocker, origin in the rocker frame, axis, visual model)
    "FL": ("L", (-0.15256, -0.08214, -0.08802), (0, -1, 0), "WheelA"),
    "RL": ("L", (0.15256, -0.08214, -0.08802), (0, -1, 0), "WheelA"),
    "FR": ("R", (0.15256, -0.08214, -0.08802), (0, 1, 0), "WheelB"),
    "RR": ("R", (-0.15256, -0.08214, -0.08802), (0, 1, 0), "WheelB"),
}
WHEEL_RADIUS = 0.0625  # tires collide as ellipsoids: a cylinder's rim contacts chatter on the
# floor while skid-steering (MuJoCo's car example uses ellipsoid wheels for the same reason)
WHEEL_SEPARATION = 0.358  # gz DiffDrive in leo_gazebo; the firmware uses the same
ROCKER_LIMIT = 0.24

# Cameras in base_link: position, downward pitch (rad), horizontal FOV (rad), native size.
CAMERAS = {
    # the rover's own camera (leo_description camera_joint + leo_gazebo sensor)
    "leo": {"pos": (0.0971, 0.0, -0.0427), "pitch": 0.2094, "hfov": 1.9, "size": (640, 480)},
    # the OAK-D of rover_nav (config/mounts.yaml oak_mount; ~69 deg color FOV at 16:9)
    "oak": {
        "pos": (0.13, 0.0, -0.035),
        "pitch": 0.175,
        "hfov": math.radians(69),
        "size": (640, 360),
    },
}

# Friction: MuJoCo uses the larger coefficient of the two geoms. Everything is 0.4 but the
# floor (1.0), so tires grip the floor but slide on walls and furniture instead of climbing them.
# Collision bits: the rover never collides with itself; static world only with movers.
ROVER = 'contype="2" conaffinity="1"'
STATIC = 'contype="1" conaffinity="2"'
MOVER = 'contype="3" conaffinity="3"'
ROVER_VISUAL_GROUP = 2  # hidden in the rover's own camera renders (the lenses sit in the shell)


@dataclass
class Obj:
    name: str
    shape: str  # box | sphere | cylinder | table | basket
    size: tuple[float, ...]  # MuJoCo half sizes (box: 3, sphere: 1, cylinder: r, half h)
    rgba: tuple[float, float, float, float]
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    movable: bool = False
    solid: bool = True  # False: flat things the rover drives over (socks)
    label: str = ""  # how a person would call it, for goals ("the red ball")

    @property
    def radius(self) -> float:
        """Footprint radius, for spacing objects apart."""
        if self.shape in ("sphere", "cylinder"):
            return self.size[0]
        return math.hypot(self.size[0], self.size[1])


@dataclass
class World:
    width: float = 6.0  # x extent of the room, metres (origin in the middle)
    depth: float = 5.0  # y extent
    objects: list[Obj] = field(default_factory=list)
    start: tuple[float, float, float] = (-2.0, -0.9, 0.0)  # rover x, y, yaw (rad)
    seed: int | None = None

    def obj(self, name: str) -> Obj:
        return next(o for o in self.objects if o.name == name)


def _catalogue() -> list[Obj]:
    return [
        Obj("red_ball", "sphere", (0.12,), (0.85, 0.1, 0.1, 1), movable=True, label="the red ball"),
        Obj(
            "blue_box",
            "box",
            (0.15, 0.15, 0.15),
            (0.15, 0.3, 0.85, 1),
            movable=True,
            label="the blue box",
        ),
        Obj("green_bin", "cylinder", (0.18, 0.25), (0.1, 0.6, 0.2, 1), label="the green bin"),
        Obj(
            "yellow_crate", "box", (0.2, 0.15, 0.13), (0.95, 0.8, 0.1, 1), label="the yellow crate"
        ),
        Obj(
            "laundry_basket",
            "basket",
            (0.25, 0.18, 0.15),
            (0.92, 0.92, 0.95, 1),
            label="the white laundry basket",
        ),
        Obj("table", "table", (0.5, 0.3, 0.36), (0.55, 0.38, 0.22, 1), label="the wooden table"),
        Obj("couch", "box", (0.9, 0.4, 0.22), (0.3, 0.3, 0.35, 1), label="the gray couch"),
        Obj(
            "sock_red",
            "box",
            (0.1, 0.04, 0.006),
            (0.8, 0.1, 0.15, 1),
            solid=False,
            label="the red sock",
        ),
        Obj(
            "sock_black",
            "box",
            (0.1, 0.04, 0.006),
            (0.08, 0.08, 0.08, 1),
            solid=False,
            label="the black sock",
        ),
        Obj(
            "sock_white",
            "box",
            (0.1, 0.04, 0.006),
            (0.95, 0.95, 0.95, 1),
            solid=False,
            label="the white sock",
        ),
    ]


def default_world(seed: int | None = 0, width: float = 6.0, depth: float = 5.0) -> World:
    """A room with the catalogue objects scattered by `seed` (None: fixed hand layout)."""
    objects = _catalogue()
    world = World(width, depth, objects, seed=seed)
    if seed is None:
        fixed = {
            "red_ball": (1.8, 1.2),
            "blue_box": (0.6, -1.2),
            "green_bin": (2.4, -1.8),
            "yellow_crate": (-0.4, 1.6),
            "laundry_basket": (2.4, 0.2),
            "table": (-1.6, 1.5),
            "couch": (0.0, -2.0),
            "sock_red": (0.2, 0.3),
            "sock_black": (-0.8, -0.6),
            "sock_white": (1.2, -0.3),
        }
        for o in objects:
            o.x, o.y = fixed[o.name]
        return world
    rng = random.Random(seed)
    margin = 0.35
    placed: list[tuple[float, float, float]] = []
    start_r = 0.45
    for o in sorted(objects, key=lambda o: (not o.solid, -o.radius)):  # big furniture first
        clearance = 0.15 if not o.solid else 0.55  # leave a rover-wide gap between solids
        for attempt in range(2000):
            gap = clearance * (1 - attempt / 2000)  # a crowded room gets narrower gaps
            x = rng.uniform(-width / 2 + margin + o.radius, width / 2 - margin - o.radius)
            y = rng.uniform(-depth / 2 + margin + o.radius, depth / 2 - margin - o.radius)
            if all(math.hypot(x - px, y - py) > o.radius + pr + gap for px, py, pr in placed):
                break
        else:
            raise ValueError(f"no room for {o.name} in a {width} x {depth} m room")
        o.x, o.y = x, y
        o.yaw = rng.choice([0.0, math.pi / 2]) if o.shape in ("table", "basket", "box") else 0.0
        if o.solid:
            placed.append((x, y, o.radius))
    for attempt in range(2000):
        sx = rng.uniform(-width / 2 + 0.6, width / 2 - 0.6)
        sy = rng.uniform(-depth / 2 + 0.6, depth / 2 - 0.6)
        gap = 0.3 * (1 - attempt / 2000)
        if all(math.hypot(sx - px, sy - py) > pr + start_r + gap for px, py, pr in placed):
            break
    else:
        raise ValueError(f"no free start pose in a {width} x {depth} m room")
    world.start = (sx, sy, rng.uniform(-math.pi, math.pi))
    return world


def _f(*values: float) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _camera_xml(name: str, spec: dict, width: int, height: int) -> str:
    p = spec["pitch"]
    fovy = math.degrees(2 * math.atan(math.tan(spec["hfov"] / 2) * height / width))
    # MuJoCo cameras look along -z with +y up: image right = -y of the frame, up = its z
    return (
        f'<camera name="{name}" pos="{_f(*spec["pos"])}" '
        f'xyaxes="0 -1 0 {_f(math.sin(p), 0, math.cos(p))}" fovy="{fovy:.4g}"/>'
    )


def _rover_xml(meshes: dict, x: float, y: float, yaw: float) -> str:
    def visuals(part: str) -> str:
        return "".join(
            f'<geom type="mesh" mesh="{m["file"][:-4]}" rgba="{_f(*m["rgba"])}" '
            f'contype="0" conaffinity="0" group="{ROVER_VISUAL_GROUP}" mass="0"/>'
            for m in meshes[part]
        )

    wheels = {"L": [], "R": []}
    for name, (rocker, pos, axis, model) in WHEELS.items():
        wheels[rocker].append(f"""
          <body name="wheel_{name}" pos="{_f(*pos)}">
            <inertial mass="0.283642" pos="0 0.030026 0"
              fullinertia="0.000391 0.0004716 0.000391 1.23962e-6 5.52582e-7 -2.082042e-6"/>
            <joint name="wheel_{name}" type="hinge" axis="{_f(*axis)}" damping="0.01"
              armature="0.002"/>
            {visuals(model)}
            <geom name="tire_{name}" type="ellipsoid" size="{_f(WHEEL_RADIUS, WHEEL_RADIUS, 0.035)}"
              pos="0 0.04485 0" euler="90 0 0" {ROVER} margin="0.001" group="3"/>
          </body>""")
    rockers = ""
    for side, pos in ROCKER_POS.items():
        rockers += f"""
        <body name="rocker_{side}" pos="{_f(*pos)}" euler="0 0 {180 if side == "L" else 0}">
          <inertial mass="1.387336" pos="0 0.01346 -0.06506"
            fullinertia="0.002956 0.02924 0.02832 -1.489324e-6 -8.103407e-6 7.112e-5"/>
          <joint name="rocker_{side}" type="hinge" axis="0 1 0" damping="0.5" armature="0.01"
            range="{-ROCKER_LIMIT} {ROCKER_LIMIT}"/>
          {visuals("Rocker")}
          <geom type="mesh" mesh="Rocker_outline" {ROVER} group="3" mass="0"/>
          {"".join(wheels[side])}
        </body>"""
    cameras = "".join(_camera_xml(n, s, *s["size"]) for n, s in CAMERAS.items())
    pose = f'pos="{_f(x, y, BASE_HEIGHT + 0.005)}" euler="0 0 {_f(math.degrees(yaw))}"'
    return f"""
    <body name="base_link" {pose}>
      <freejoint name="root"/>
      <inertial mass="1.584994" pos="-0.019662 0.011643 -0.031802"
        fullinertia="0.01042 0.01045 0.01817 0.001177 -0.0008871 0.0002226"/>
      {visuals("Chassis")}
      <geom type="mesh" mesh="Chassis_outline" {ROVER} group="3" mass="0"/>
      <body name="antenna" pos="-0.0052 0.056 -0.0065">{visuals("Antenna")}</body>
      <site name="base_footprint" pos="0 0 {-BASE_HEIGHT}" size="0.01"/>
      <site name="imu" pos="0.0628 -0.0314 -0.0393" size="0.01"/>
      {cameras}
      <camera name="chase" pos="-1.1 0 0.75" xyaxes="0 -1 0 0.5 0 0.866" fovy="60"/>
      {rockers}
    </body>"""


def _object_xml(o: Obj) -> str:
    rgba = _f(*o.rgba)
    head = f'<body name="{o.name}" pos="{_f(o.x, o.y, 0)}" euler="0 0 {_f(math.degrees(o.yaw))}">'
    if o.shape == "table":
        sx, sy, h = o.size
        parts = [
            (0.025, 0.025, h, dx * (sx - 0.04), dy * (sy - 0.04), h)
            for dx in (-1, 1)
            for dy in (-1, 1)
        ]  # legs
        parts.append((sx, sy, 0.02, 0, 0, 2 * h + 0.02))  # top
        geoms = "".join(
            f'<geom type="box" size="{_f(a, b, c)}" pos="{_f(x, y, z)}" rgba="{rgba}" {STATIC}/>'
            for a, b, c, x, y, z in parts
        )
        return f"{head}{geoms}</body>"
    if o.shape == "basket":
        sx, sy, h = o.size
        t = 0.012
        walls = [
            (0, 0, t / 2, sx, sy, t / 2),
            (sx - t, 0, h, t, sy, h),
            (-sx + t, 0, h, t, sy, h),
            (0, sy - t, h, sx, t, h),
            (0, -sy + t, h, sx, t, h),
        ]
        geoms = "".join(
            f'<geom type="box" size="{_f(a, b, c)}" pos="{_f(x, y, z)}" rgba="{rgba}" {STATIC}/>'
            for x, y, z, a, b, c in walls
        )
        return f"{head}{geoms}</body>"
    size = _f(*o.size)
    z = o.size[-1] if o.shape != "sphere" else o.size[0]
    if o.movable:
        return (
            f'<body name="{o.name}" pos="{_f(o.x, o.y, z + 0.002)}" '
            f'euler="0 0 {_f(math.degrees(o.yaw))}"><freejoint/>'
            f'<geom type="{o.shape}" size="{size}" rgba="{rgba}" {MOVER} '
            f'density="{150 if o.shape == "sphere" else 300}"/></body>'
        )
    bits = STATIC if o.solid else 'contype="0" conaffinity="0"'
    geom = f'<geom type="{o.shape}" size="{size}" pos="0 0 {_f(z)}" rgba="{rgba}" {bits}/>'
    return f"{head}{geom}</body>"


def build_mjcf(world: World) -> str:
    meshes = json.loads((ASSETS / "meshes.json").read_text())
    mesh_assets = "".join(
        f'<mesh name="{m["file"][:-4]}" file="{m["file"]}"/>'
        for part in meshes.values()
        for m in part
    )
    mesh_assets += '<mesh name="Chassis_outline" file="Chassis_outline.stl"/>'
    mesh_assets += '<mesh name="Rocker_outline" file="Rocker_outline.stl"/>'
    w, d, wall_h, t = world.width / 2, world.depth / 2, 0.6, 0.05
    walls = "".join(
        f'<geom name="wall_{n}" type="box" size="{_f(a, b, wall_h / 2)}" '
        f'pos="{_f(x, y, wall_h / 2)}" material="wall" {STATIC}/>'
        for n, x, y, a, b in [
            ("n", 0, d + t, w + 2 * t, t),
            ("s", 0, -d - t, w + 2 * t, t),
            ("e", w + t, 0, t, d),
            ("w", -w - t, 0, t, d),
        ]
    )
    x, y, yaw = world.start
    extent = max(world.width, world.depth) + 0.4
    motors = "".join(  # the firmware's wheel speed loop; 2 N m is the URDF's effort limit
        f'<velocity name="wheel_{n}" joint="wheel_{n}" kv="10" forcerange="-2 2"/>' for n in WHEELS
    )
    return f"""<mujoco model="leo_rover_room">
  <compiler angle="degree" meshdir="{ASSETS}" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="2048"/>
    <map znear="0.005"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5" specular="0.05 0.05 0.05"/>
  </visual>
  <asset>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.72 0.62 0.48" rgb2="0.66 0.56 0.43"
      width="512" height="512"/>
    <material name="floor" texture="floor" texrepeat="12 12" reflectance="0.05"/>
    <material name="wall" rgba="0.88 0.87 0.84 1"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.9 0.9 0.92" rgb2="0.75 0.75 0.8"
      width="64" height="64"/>
    {mesh_assets}
  </asset>
  <default>
    <geom solref="0.02 1" friction="0.4 0.02 0.001"/>
  </default>
  <worldbody>
    <light name="sun" pos="0 0 4" dir="0.1 0.15 -1" directional="true" diffuse="0.6 0.6 0.6"
      castshadow="true"/>
    <geom name="floor" type="plane" size="{_f(w + 1, d + 1, 0.1)}" material="floor" contype="1"
      conaffinity="3" friction="1.0 0.02 0.001"/>
    {walls}
    <camera name="map" pos="0 0 6" xyaxes="1 0 0 0 1 0" projection="orthographic"
      fovy="{extent:.4g}"/>
    {"".join(_object_xml(o) for o in world.objects)}
    {_rover_xml(meshes, x, y, yaw)}
  </worldbody>
  <equality>
    <joint joint1="rocker_R" joint2="rocker_L" polycoef="0 1 0 0 0"/>
  </equality>
  <actuator>{motors}</actuator>
</mujoco>"""

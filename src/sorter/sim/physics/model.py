"""The physics scene as MJCF: the base (floor, rover, cargo box, the arm from its URDF) from
`sim.layout`, plus what each scene in `sim.scenes` adds (`sorter.sim.scenes.<name>.scene`).

Everything physical lives here. The arm's joints carry position servos that stand in for the
RobStride position loop (with gravity compensation for its integral action); the gripper is a
force motor driven by the torque `rebot_b601` sends to motor 7. Cloth is a 2D flex with a
crumpled rest shape, so it keeps folds the fingers can pinch.
"""

from __future__ import annotations

import importlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace

import numpy as np
from rebot_b601.assets import ASSETS_DIR
from rebot_b601.kinematics import _FINGER_L, _FINGER_R, FINGER_TRAVEL_M

from sorter.core.types import ColorClass
from sorter.sim.config import RAIL_INSET_MM, SimConfig
from sorter.sim.physics.floor import parquet_texture
from sorter.sim.physics.floor import tile_mm as floor_tile_mm
from sorter.sim.rig import PALETTE, camera_mount

URDF = ASSETS_DIR / "reBot_Lite_RS_with_gripper.urdf"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
TIMESTEP = 0.004
GRIPPER_OPEN_MOTOR_DEG = 270.0  # rebot_b601 maps 0..270° of motor 7 to 0..FINGER_TRAVEL_M a finger
# motor 7 torque → force of each finger (N per N·m). A lossless transmission with the mapping
# above would give ~94; the real linkage is unknown, and 94 x 3 N·m slams cloth out of the
# fingers. 15 gives 45 N while closing and 15 N holding (rebot_b601's torque limits): measure it.
GRIPPER_N_PER_NM = 15.0
GRIPPER_MOTOR_KD = 1.5  # N·m·s/rad: the damping rebot_b601 sets on motor 7 (MIT mode)
# that damping as felt at a finger: it limits the fingers to ~20 mm/s at 3 N·m
_FINGER_DAMPING = (
    GRIPPER_N_PER_NM * GRIPPER_MOTOR_KD * math.radians(GRIPPER_OPEN_MOTOR_DEG) / FINGER_TRAVEL_M
)
# servo stiffness (N·m/rad), damping (N·m·s/rad), rotor inertia (kg·m²) per joint
_KP = (300, 300, 300, 60, 60, 40)
_DAMPING = (8, 8, 6, 1.5, 1.2, 0.8)
_ARMATURE = (0.05, 0.05, 0.05, 0.02, 0.02, 0.01)
_BIN_RGB = {
    ColorClass.LIGHT: "0.92 0.93 0.93",
    ColorClass.DARK: "0.22 0.23 0.24",
    ColorClass.COLORED: "0.14 0.55 0.67",
}

CARDBOARD = "0.5 0.01 0.001"  # friction of the cargo box and the bins
CARDBOARD_RGBA = "0.66 0.5 0.34 1"
# tray floors reach this far into what they stand on: a cloth vertex pressed through a thin
# slab would sit between its underside and the support, pushed both ways, and stay pinned there
# (the cloth then stretches from the gripper to the support and snaps back)
_SINK = 0.02
MARK_MM = 12  # a tape mark's side
CLOTH_N = 8  # vertices per side, every item
CLOTH_SHEET_M = (0.14, 0.14)  # the flat sheet, default size
CLOTH_GATHER = 0.6  # the rest shape is the sheet gathered to this fraction, with folds
CLOTH_FOLD_M = 0.03  # height of the folds


@dataclass(frozen=True)
class ItemSpec:
    """A cloth item a scene adds. `id` is set by `build`: the index in the scene's item list."""

    color: ColorClass
    rgb: tuple[float, float, float]
    pos: tuple[float, float, float]  # m, where the crumpled rest shape starts (drops from)
    yaw: float
    sheet_m: tuple[float, float] = CLOTH_SHEET_M  # the flat sheet, before it is gathered
    gather: float = CLOTH_GATHER
    fold_m: float = CLOTH_FOLD_M  # height of the folds
    # the rest shape instead of a gathered sheet: CLOTH_N² points (x, y, z in m, flattened, in
    # the order of `crumpled_sheet`: row by row, a row along x), its lowest point at z = 0
    rest_m: tuple[float, ...] | None = None
    id: int = -1


def _f(*v: float) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def _quat(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """URDF rpy (R = Rz·Ry·Rx) → MuJoCo quaternion (w, x, y, z)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r / 2),
        math.sin(r / 2),
        math.cos(p / 2),
        math.sin(p / 2),
        math.cos(y / 2),
        math.sin(y / 2),
    )
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rpy_mat(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _mat_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    import mujoco

    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).flatten())
    return tuple(q)


def _vec(s: str | None, n: int = 3) -> tuple[float, ...]:
    return tuple(float(v) for v in s.split()) if s else (0.0,) * n


def crumpled_sheet(
    rng: np.random.Generator,
    sheet_m: tuple[float, float] = CLOTH_SHEET_M,
    gather: float = CLOTH_GATHER,
    fold_m: float = CLOTH_FOLD_M,
) -> tuple[np.ndarray, list[int]]:
    """Points (N², 3) of a gathered, folded sheet (its lowest point at z = 0) and triangles."""
    n = CLOTH_N
    U, V = np.meshgrid(
        np.linspace(-sheet_m[0] / 2, sheet_m[0] / 2, n),
        np.linspace(-sheet_m[1] / 2, sheet_m[1] / 2, n),
    )
    Z = np.zeros_like(U)
    side = max(sheet_m)
    for _ in range(6):
        k = rng.uniform(2, 6) * np.pi / side
        a, ph = rng.uniform(0, np.pi), rng.uniform(0, 2 * np.pi)
        Z += np.sin(k * (U * np.cos(a) + V * np.sin(a)) + ph)
    Z = fold_m * (Z - Z.min()) / np.ptp(Z)
    pts = np.c_[(U * gather).ravel(), (V * gather).ravel(), Z.ravel()]
    return pts, grid_triangles()


def grid_triangles() -> list[int]:
    """The triangles of the CLOTH_N × CLOTH_N grid of a cloth item, vertex indices."""
    n = CLOTH_N
    tris = []
    for i in range(n - 1):
        for j in range(n - 1):
            a, b, c, e = i * n + j, i * n + j + 1, (i + 1) * n + j, (i + 1) * n + j + 1
            tris += [a, b, e, a, e, c]
    return tris


def palette_rgb(color: ColorClass, rng: np.random.Generator) -> tuple[float, float, float]:
    """A random cloth color of class `color`, RGB 0..1."""
    b, g, r = PALETTE[color][int(rng.integers(len(PALETTE[color])))]
    return (r / 255, g / 255, b / 255)


def _arm(parent: ET.Element, cfg: SimConfig) -> None:
    """The arm under `parent`: bodies, joints, inertia and visual meshes from the URDF, simple
    collision boxes for the gripper, and the wrist camera."""
    urdf = ET.parse(URDF).getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    joints = {j.find("child").get("link"): j for j in urdf.findall("joint")}
    manifest = json.loads((ASSETS_DIR / "manifest.json").read_text())["links"]

    def inertial(body: ET.Element, name: str) -> None:
        i = links[name].find("inertial")
        o = i.find("origin")
        t = {k: float(i.find("inertia").get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        inertia = np.array(
            [
                [t["ixx"], t["ixy"], t["ixz"]],
                [t["ixy"], t["iyy"], t["iyz"]],
                [t["ixz"], t["iyz"], t["izz"]],
            ]
        )
        R = _rpy_mat(_vec(o.get("rpy")))
        inertia = R @ inertia @ R.T  # into the body frame
        ET.SubElement(
            body,
            "inertial",
            pos=_f(*_vec(o.get("xyz"))),
            mass=i.find("mass").get("value"),
            fullinertia=_f(*np.diag(inertia), inertia[0, 1], inertia[0, 2], inertia[1, 2]),
        )

    def visuals(body: ET.Element, name: str, T: np.ndarray | None = None) -> None:
        for k, p in enumerate(manifest.get(name, [])):
            pos, quat = np.array(p["xyz"]), _quat(p["rpy"])
            if T is not None:  # the finger meshes are placed in the TCP frame
                pos = T[:3, :3] @ pos + T[:3, 3]
                import mujoco

                R = np.zeros(9)
                mujoco.mju_quat2Mat(R, np.array(quat))
                quat = _mat_quat(T[:3, :3] @ R.reshape(3, 3))
            ET.SubElement(
                body,
                "geom",
                name=f"{name}_vis{k}",
                type="mesh",
                mesh=p["mesh"],
                pos=_f(*pos),
                quat=_f(*quat),
                rgba=_hex_rgba(p["color"]),
                **{"class": "visual"},
            )

    body = ET.SubElement(parent, "body", name="base_link")
    inertial(body, "base_link")
    visuals(body, "base_link")
    ET.SubElement(body, "geom", name="base_col", type="cylinder", size="0.06 0.04", pos="0 0 0.04")
    bodies = {}
    for i, jn in enumerate(ARM_JOINTS):
        child = f"link{i + 1}"
        j = joints[child]
        o = j.find("origin")
        body = ET.SubElement(
            body,
            "body",
            name=child,
            pos=_f(*_vec(o.get("xyz"))),
            quat=_f(*_quat(_vec(o.get("rpy")))),
            gravcomp="1",
        )
        bodies[child] = body
        lim = j.find("limit")
        ET.SubElement(
            body,
            "joint",
            name=jn,
            axis=j.find("axis").get("xyz"),
            range=f"{lim.get('lower')} {lim.get('upper')}",
            damping=str(_DAMPING[i]),
            armature=str(_ARMATURE[i]),
        )
        inertial(body, child)
        visuals(body, child)
    o = joints["gripper_end"].find("origin")
    tcp = ET.SubElement(
        body,
        "body",
        name="gripper_end",
        pos=_f(*_vec(o.get("xyz"))),
        quat=_f(*_quat(_vec(o.get("rpy")))),
        gravcomp="1",
    )
    inertial(tcp, "gripper_end")
    visuals(tcp, "gripper_end")
    ET.SubElement(tcp, "site", name="tcp", size="0.004")
    # the gripper housing behind the fingers; the camera is on link5, joint 6 doesn't turn it
    ET.SubElement(tcp, "geom", name="palm", type="box", size="0.042 0.09 0.034", pos="-0.115 0 0")
    T = camera_mount(cfg)
    mount = T[:3, 3] / 1000
    ET.SubElement(
        bodies["link5"],
        "geom",
        name="camera_body",
        type="box",
        size="0.013 0.045 0.0125",
        pos=_f(*(mount - T[:3, 2] * 0.013)),
        quat=_f(*_mat_quat(T[:3, :3])),
        rgba="0.1 0.1 0.12 1",
    )
    fovy = math.degrees(2 * math.atan(cfg.height / 2 / cfg.focal_px))
    # a MuJoCo camera looks along its -z with +y up: the optical frame turned 180° about x
    x, y = T[:3, 0], -T[:3, 1]
    ET.SubElement(
        bodies["link5"],
        "camera",
        name="wrist",
        pos=_f(*mount),
        xyaxes=_f(*x, *y),
        fovy=f"{fovy:.4f}",
        resolution=f"{cfg.width} {cfg.height}",
    )
    for side, T_f, sign in (("left", _FINGER_L, -1), ("right", _FINGER_R, 1)):
        f = ET.SubElement(tcp, "body", name=f"finger_{side}", gravcomp="1")
        ET.SubElement(
            f,
            "joint",
            name=f"finger_{side}",
            type="slide",
            axis=f"0 {sign} 0",
            range=f"0 {FINGER_TRAVEL_M}",
            damping=f"{_FINGER_DAMPING:.0f}" if side == "left" else "4",  # the motor's finger
            solreflimit="0.002 1",
        )
        ET.SubElement(
            f,
            "inertial",
            pos=_f(-0.04, sign * 0.015, 0),
            mass="0.075",
            diaginertia="2e-5 2e-5 2e-5",
        )
        visuals(f, f"gripper_{side}", T_f)
        # the fingertip pad, 60 mm long along the approach, its inner face at the finger's origin
        ET.SubElement(
            f,
            "geom",
            name=f"pad_{side}",
            type="box",
            size="0.03 0.006 0.012",
            pos=_f(-0.03, sign * 0.006, 0),
            friction="1.2 0.01 0.001",
            condim="4",
            **{"class": "grip"},
        )


def _hex_rgba(h: str) -> str:
    h = h.lstrip("#")
    return _f(*(int(h[i : i + 2], 16) / 255 for i in (0, 2, 4)), 1)


def box(parent: ET.Element, name: str, lo, hi, rgba: str, **attrs: str) -> ET.Element:
    """A static box geom between corners `lo` and `hi` (m)."""
    lo, hi = np.asarray(lo, dtype=float), np.asarray(hi, dtype=float)
    return ET.SubElement(
        parent,
        "geom",
        name=name,
        type="box",
        size=_f(*(hi - lo) / 2),
        pos=_f(*(hi + lo) / 2),
        rgba=rgba,
        **attrs,
    )


def tray(
    parent: ET.Element,
    name: str,
    inner: tuple[float, float, float, float],
    base_z: float,
    floor_z: float,
    rim_z: float,
    t: float,
    rgba: str,
    floor_rgba: str | None = None,
    dividers_x: tuple[float, ...] = (),
) -> None:
    """An open box standing on z = `base_z` (m): the inside `inner` = (x0, x1, y0, y1), its
    floor's top at `floor_z`, walls of thickness `t` up to `rim_z`, and walls across it at
    `dividers_x` (their centers)."""
    x0, x1, y0, y1 = inner
    box(
        parent,
        f"{name}_floor",
        (x0 - t, y0 - t, base_z - _SINK),
        (x1 + t, y1 + t, floor_z),
        floor_rgba or rgba,
        friction=CARDBOARD,
    )
    walls = (
        ((x0 - t, y0 - t), (x0, y1 + t)),
        ((x1, y0 - t), (x1 + t, y1 + t)),
        ((x0, y0 - t), (x1, y0)),
        ((x0, y1), (x1, y1 + t)),
        *(((x - t / 2, y0), (x + t / 2, y1)) for x in dividers_x),
    )
    for k, (lo, hi) in enumerate(walls):
        box(parent, f"{name}_wall{k}", (*lo, base_z), (*hi, rim_z), rgba, friction=CARDBOARD)


def _cloth(parent: ET.Element, it: ItemSpec, rng: np.random.Generator) -> None:
    if it.rest_m is not None:
        pts, tris = np.array(it.rest_m).reshape(-1, 3), grid_triangles()
    else:
        pts, tris = crumpled_sheet(rng, it.sheet_m, it.gather, it.fold_m)
    c, s = math.cos(it.yaw), math.sin(it.yaw)
    pts = pts @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    f = ET.SubElement(
        parent,
        "flexcomp",
        name=f"item{it.id}",
        type="direct",
        dim="2",
        radius="0.006",
        mass="0.05",
        rgba=_f(*it.rgb, 1),
        pos=_f(*it.pos),
        point=" ".join(f"{v:.5f}" for v in pts.ravel()),
        element=" ".join(map(str, tris)),
    )
    ET.SubElement(f, "edge", damping="0.05")
    ET.SubElement(
        f,
        "elasticity",
        young="1e5",
        poisson="0.2",
        thickness="0.004",
        damping="0.002",
        elastic2d="both",
    )
    # cloth touches the floor, the rover, the boxes and the fingers, but not other cloth:
    # flex-flex contacts cost ~10x the rest of the step, so items in a pile pass through each other
    ET.SubElement(
        f,
        "contact",
        condim="3",
        friction="0.6 0.01 0.001",  # cotton on cardboard; MuJoCo takes the larger of a pair
        selfcollide="none",
        solref="0.004 1",
        contype="2",
        conaffinity="1",
    )


def _rover(world: ET.Element, cfg: SimConfig) -> None:
    """The rover under the arm, as on the photos of the real one: wheels at the corners of
    `body`, side rails, the battery under the deck plate (top at z = 0), the equipment behind
    the arm, and the cardboard cargo box to its left."""
    lay = cfg.layout
    fz = lay.floor_z_mm / 1000
    x0, x1, y0, y1 = (v / 1000 for v in lay.body.bounds())
    r, w = lay.wheel_radius_mm / 1000, lay.wheel_width_mm / 1000
    for k, (x, y) in enumerate(((x0 + r, y0), (x1 - r, y0), (x0 + r, y1), (x1 - r, y1))):
        side = 1 if y == y0 else -1  # towards the rover's middle
        yc = y + side * w / 2
        ET.SubElement(
            world,
            "geom",
            name=f"wheel{k}",
            type="cylinder",
            size=_f(r, w / 2),
            pos=_f(x, yc, fz + r),
            quat=_f(*_quat((math.pi / 2, 0, 0))),
            rgba="0.09 0.09 0.1 1",
        )
        for part, (rr, hw, dy, rgba) in {
            "hub": (0.022, 0.004, -side * w / 2, "0.75 0.76 0.78 1"),  # the outer hub cap
            "motor": (0.02, 0.015, side * (w / 2 + 0.015), "0.8 0.81 0.83 1"),  # hub motor
        }.items():
            ET.SubElement(
                world,
                "geom",
                name=f"wheel{k}_{part}",
                type="cylinder",
                size=_f(rr, hw),
                pos=_f(x, yc + dy, fz + r),
                quat=_f(*_quat((math.pi / 2, 0, 0))),
                rgba=rgba,
                contype="0",
                conaffinity="0",
            )
    # the side rails (black aluminium extrusion) from wheel to wheel, over the hub motors
    ri = RAIL_INSET_MM / 1000
    for side, y in ((1, y0), (-1, y1)):
        yr = y + side * (w + 0.04)
        box(
            world,
            f"rail{0 if side > 0 else 1}",
            (x0 + ri, yr - 0.01, fz + r + 0.005),
            (x1 - ri, yr + 0.01, fz + r + 0.045),
            "0.05 0.05 0.06 1",
        )
    ry0, ry1 = y0 + w + 0.03, y1 - w - 0.03
    for k, x in enumerate((x0 + ri + 0.01, x1 - ri - 0.01)):
        box(
            world,
            f"crossbar{k}",
            (x - 0.01, ry0, fz + r + 0.005),
            (x + 0.01, ry1, fz + r + 0.045),
            "0.05 0.05 0.06 1",
        )
    dx0, dx1, dy0, dy1 = (v / 1000 for v in lay.deck.bounds())
    # the battery in its orange bag, hanging under the deck plate between the rails
    box(
        world,
        "battery",
        (dx0 + 0.02, dy0 + 0.03, fz + r),
        (dx1 - 0.03, dy1 - 0.03, -0.008),
        "0.95 0.36 0.08 1",
    )
    box(world, "deck", (dx0, dy0, -0.008), (dx1, dy1, 0.0), "0.06 0.06 0.07 1")
    for name, b in lay.equipment.items():
        bx0, bx1, by0, by1, bz0, bz1 = (v / 1000 for v in b.box_mm)
        box(world, name, (bx0, by0, bz0), (bx1, by1, bz1), _f(*b.rgba))
    cargo = lay.cargo
    cx0, cx1, cy0, cy1 = (v / 1000 for v in cargo.bounds())
    t = cargo.wall_t_mm / 1000
    inner = cargo.insides()
    dividers = tuple(
        (a.bounds()[1] + b.bounds()[0]) / 2000 for a, b in zip(inner, inner[1:], strict=False)
    )
    base, rim = cargo.base_z_mm / 1000, cargo.rim_z_mm / 1000
    tray(
        world,
        "cargo",
        (cx0, cx1, cy0, cy1),
        base,
        cargo.floor_z_mm / 1000,
        rim,
        t,
        CARDBOARD_RGBA,
        CARDBOARD_RGBA,
        dividers,
    )
    # black tape strips down from the rim of the box's outer (+y) wall, as on the real one
    for k in range(6):
        x = cx0 + (k + 0.5) * (cx1 - cx0) / 6
        box(
            world,
            f"cargo_tape{k}",
            (x - 0.007, cy1 + t, rim - 0.022),
            (x + 0.007, cy1 + t + 0.0005, rim),
            "0.05 0.05 0.05 1",
            contype="0",
            conaffinity="0",
        )


def _floor_view_extras(world: ET.Element, cfg: SimConfig, board: bool, board_z_mm: float):
    """The calibration's printed board and tape marks, on the floor view."""
    fv = cfg.layout.floor_view
    cx, cy = (v / 1000 for v in fv.center_mm)
    if board:
        from sorter.calibration.board import BOARD

        w, h = BOARD.size_m
        ET.SubElement(
            world,
            "geom",
            name="board",
            type="box",
            size=_f(w / 2, h / 2, 0.0005),
            pos=_f(cx, cy, board_z_mm / 1000 - 0.0005),
            material="board",
            contype="0",
            conaffinity="0",
        )
    if cfg.marks:  # dark tape squares, where the /calibrate page asks for them
        from sorter.calibration.marks import MARK_OFFSETS_MM

        z = cfg.layout.floor_z_mm / 1000 + 0.0012
        for name, (dx, dy) in MARK_OFFSETS_MM.items():
            ET.SubElement(
                world,
                "geom",
                name=f"mark_{name}",
                type="box",
                size=_f(MARK_MM / 2000, MARK_MM / 2000, 0.0002),
                pos=_f(cx + dx / 1000, cy + dy / 1000, z),
                rgba="0.08 0.08 0.1 1",
                contype="0",
                conaffinity="0",
            )


@dataclass(frozen=True)
class Scene:
    xml: str
    items: list[ItemSpec]  # item id = index


def build(cfg: SimConfig, board: bool = False, board_z_mm: float = 1.0) -> Scene:
    """MJCF of the whole scene: the base, then each scene of `sim.scenes`, then the cloth items
    they asked for. `board`: a ChArUco board over the floor view center, its top at `board_z_mm`
    (hand-eye calibration)."""
    root = ET.Element("mujoco", model="sorter")
    ET.SubElement(root, "compiler", angle="radian", meshdir=str(ASSETS_DIR), autolimits="true")
    ET.SubElement(
        root,
        "option",
        timestep=str(TIMESTEP),
        integrator="discrete",
        cone="pyramidal",  # impratio stays 1: stiffer friction wedges a dragged cloth into walls
    )
    vis = ET.SubElement(root, "visual")
    ET.SubElement(
        vis, "global", offwidth=str(max(cfg.width, 1280)), offheight=str(max(cfg.height, 960))
    )
    ET.SubElement(vis, "quality", shadowsize="2048")
    ET.SubElement(
        vis, "headlight", ambient="0.25 0.25 0.25", diffuse="0.2 0.2 0.2", specular="0 0 0"
    )
    ET.SubElement(vis, "map", znear="0.01")
    default = ET.SubElement(root, "default")
    v = ET.SubElement(default, "default", **{"class": "visual"})
    ET.SubElement(v, "geom", contype="0", conaffinity="0", group="2")
    g = ET.SubElement(default, "default", **{"class": "grip"})
    ET.SubElement(g, "geom", rgba="0.3 0.3 0.32 0")  # invisible: the finger meshes show

    asset = ET.SubElement(root, "asset")
    manifest = json.loads((ASSETS_DIR / "manifest.json").read_text())["links"]
    for mesh in sorted({p["mesh"] for parts in manifest.values() for p in parts}):
        ET.SubElement(asset, "mesh", name=mesh, file=mesh)
    # herringbone oak parquet, the pattern diagonal to the rover like on the photos
    ET.SubElement(asset, "texture", name="floor", type="2d", file=str(parquet_texture()))
    ET.SubElement(
        asset,
        "material",
        name="floor",
        texture="floor",
        texuniform="true",
        texrepeat=_f(*(2 * [1000 / floor_tile_mm()])),
    )
    if board:
        from sorter.calibration.board import board_texture

        png = board_texture()
        ET.SubElement(asset, "texture", name="board", type="2d", file=str(png))
        ET.SubElement(asset, "material", name="board", texture="board")

    world = ET.SubElement(root, "worldbody")
    # a diffuse lamp over the floor in front: no hard shadow of the arm
    fv = cfg.layout.floor_view.center_mm
    ET.SubElement(
        world,
        "light",
        name="lamp",
        pos=_f(fv[0] / 1000, fv[1] / 1000, 1.2),
        dir="0 0 -1",
        diffuse="0.5 0.5 0.49",
        castshadow="false",
    )
    ET.SubElement(
        world,
        "light",
        name="fill",
        pos="-0.6 -0.6 1.5",
        dir="0.4 0.4 -1",
        diffuse="0.25 0.25 0.25",
        castshadow="false",
    )
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="3 3 0.05",
        pos=_f(0, 0, cfg.layout.floor_z_mm / 1000),
        euler=_f(0, 0, math.pi / 4),
        material="floor",
        friction="0.8 0.01 0.001",
    )
    _rover(world, cfg)
    _floor_view_extras(world, cfg, board, board_z_mm)
    _arm(world, cfg)

    items: list[ItemSpec] = []
    for name in cfg.scenes:
        scene = importlib.import_module(f"sorter.sim.scenes.{name}.scene")
        rng = np.random.default_rng([cfg.seed, len(items), sum(map(ord, name))])
        items += scene.add(world, asset, cfg, rng)
    items = [replace(it, id=i) for i, it in enumerate(items)]
    rng = np.random.default_rng(cfg.seed + 1000)
    for it in items:
        _cloth(world, it, rng)

    contact = ET.SubElement(root, "contact")
    ET.SubElement(contact, "exclude", body1="finger_left", body2="finger_right")
    ET.SubElement(contact, "exclude", body1="base_link", body2="link1")
    eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "joint", joint1="finger_right", joint2="finger_left")
    # grip: every cloth vertex can be held by the gripper; PhysicsWorld switches these on for the
    # vertices pinched between the pads and sets where they are held
    for it in items:
        for k in range(CLOTH_N * CLOTH_N):
            ET.SubElement(
                eq,
                "connect",
                name=f"grip{it.id}_{k}",
                body1=f"item{it.id}_{k}",
                body2="gripper_end",
                anchor="0 0 0",
                active="false",
                solref="0.02 1",
            )
    act = ET.SubElement(root, "actuator")
    urdf = ET.parse(URDF).getroot()
    effort = {
        j.get("name"): float(j.find("limit").get("effort"))
        for j in urdf.findall("joint")
        if j.find("limit") is not None
    }
    for i, jn in enumerate(ARM_JOINTS):
        ET.SubElement(
            act,
            "position",
            name=jn,
            joint=jn,
            kp=str(_KP[i]),
            forcerange=f"{-effort[jn]} {effort[jn]}",
            inheritrange="1",
        )
    ET.SubElement(
        act,
        "motor",
        name="gripper",
        joint="finger_left",
        gear=f"{GRIPPER_N_PER_NM:.4f}",
        ctrlrange="-3 3",
    )
    return Scene(ET.tostring(root, encoding="unicode"), items)

"""The physics scene as MJCF: the arm from its URDF, the table from `sim.layout`, cloth items.

Everything physical lives here. The arm's joints carry position servos that stand in for the
RobStride position loop (with gravity compensation for its integral action); the gripper is a
force motor driven by the torque `rebot_b601` sends to motor 7. Cloth is a 2D flex with a
crumpled rest shape, so it keeps folds the fingers can pinch.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
from rebot_b601.assets import ASSETS_DIR
from rebot_b601.kinematics import _FINGER_L, _FINGER_R, FINGER_TRAVEL_M

from sorter.core.types import ColorClass
from sorter.sim.config import SimConfig
from sorter.sim.world import _PALETTE, camera_mount

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

_CARDBOARD = "0.5 0.01 0.001"  # the box and the bins
# the mat and the box / bin floors reach this far into the table: a cloth vertex pressed through
# a thin slab would sit between its underside and the table top, pushed both ways, and stay
# pinned there (the cloth then stretches from the gripper to the table and snaps back)
_SINK = 0.02
MARK_MM = 12  # a tape mark's side
CLOTH_N = 8  # vertices per side
CLOTH_SHEET_M = 0.14  # side of the flat sheet
CLOTH_GATHER = 0.6  # the rest shape is the sheet gathered to this fraction, with folds
CLOTH_FOLD_M = 0.03  # height of the folds


@dataclass(frozen=True)
class ItemSpec:
    id: int
    color: ColorClass
    rgb: tuple[float, float, float]
    pos: tuple[float, float, float]  # m, where the crumpled rest shape starts (drops from)
    yaw: float


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


def crumpled_sheet(rng: np.random.Generator) -> tuple[np.ndarray, list[int]]:
    """Points (N², 3) of a gathered, folded sheet (its lowest point at z = 0) and triangles."""
    n, side = CLOTH_N, CLOTH_SHEET_M
    u = np.linspace(-side / 2, side / 2, n)
    U, V = np.meshgrid(u, u)
    Z = np.zeros_like(U)
    for _ in range(6):
        k = rng.uniform(2, 6) * np.pi / side
        a, ph = rng.uniform(0, np.pi), rng.uniform(0, 2 * np.pi)
        Z += np.sin(k * (U * np.cos(a) + V * np.sin(a)) + ph)
    Z = CLOTH_FOLD_M * (Z - Z.min()) / np.ptp(Z)
    pts = np.c_[(U * CLOTH_GATHER).ravel(), (V * CLOTH_GATHER).ravel(), Z.ravel()]
    tris = []
    for i in range(n - 1):
        for j in range(n - 1):
            a, b, c, e = i * n + j, i * n + j + 1, (i + 1) * n + j, (i + 1) * n + j + 1
            tris += [a, b, e, a, e, c]
    return pts, tris


def item_specs(cfg: SimConfig) -> list[ItemSpec]:
    """The items piled in the box: random places inside it, dropped one above the other."""
    rng = np.random.default_rng(cfg.seed)
    box = cfg.layout.box
    (cx, cy), (w, h) = box.center_mm, box.size_mm
    reach = CLOTH_SHEET_M * CLOTH_GATHER * 1000 / math.sqrt(2) + 5  # a turned corner: off the walls
    out = []
    for i, color in enumerate(cfg.items):
        x = rng.uniform(cx - w / 2 + reach, cx + w / 2 - reach)
        y = rng.uniform(cy - h / 2 + reach, cy + h / 2 - reach)
        b, g, r = _PALETTE[color][int(rng.integers(len(_PALETTE[color])))]
        z = box.floor_z_mm / 1000 + 0.008 + 0.035 * i
        out.append(
            ItemSpec(
                i,
                color,
                (r / 255, g / 255, b / 255),
                (x / 1000, y / 1000, z),
                float(rng.uniform(0, 2 * np.pi)),
            )
        )
    return out


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
    # the gripper housing behind the fingers, and the camera on it
    ET.SubElement(tcp, "geom", name="palm", type="box", size="0.042 0.09 0.034", pos="-0.115 0 0")
    T = camera_mount(cfg)
    mount = np.array(cfg.camera_mount_mm) / 1000
    ET.SubElement(
        tcp,
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
        tcp,
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


def _open_box(
    parent: ET.Element, name: str, center, inner, floor_z, wall_h, t, rgba, floor_rgba=None
):
    cx, cy = center
    ix, iy = inner
    top = floor_z + wall_h
    ET.SubElement(
        parent,
        "geom",
        name=f"{name}_floor",
        type="box",
        size=_f(ix / 2 + t, iy / 2 + t, (floor_z + _SINK) / 2),
        pos=_f(cx, cy, (floor_z - _SINK) / 2),
        rgba=floor_rgba or rgba,
        friction=_CARDBOARD,
    )
    for k, (dx, dy, sx, sy) in enumerate(
        (
            (-(ix + t) / 2, 0, t / 2, iy / 2 + t),
            ((ix + t) / 2, 0, t / 2, iy / 2 + t),
            (0, -(iy + t) / 2, ix / 2, t / 2),
            (0, (iy + t) / 2, ix / 2, t / 2),
        )
    ):
        ET.SubElement(
            parent,
            "geom",
            name=f"{name}_wall{k}",
            type="box",
            size=_f(sx, sy, top / 2),
            pos=_f(cx + dx, cy + dy, top / 2),
            rgba=rgba,
            friction=_CARDBOARD,
        )


def _cloth(parent: ET.Element, it: ItemSpec, rng: np.random.Generator) -> None:
    pts, tris = crumpled_sheet(rng)
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
    # cloth touches the table, the box, the bins and the fingers, but not other cloth: flex-flex
    # contacts cost ~10x the rest of the step, so items in a pile pass through each other
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


def build_xml(cfg: SimConfig, board: bool = False) -> str:
    """MJCF of the whole scene. `board`: a ChArUco board lies on the mat (hand-eye calibration)."""
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
    ET.SubElement(
        asset,
        "texture",
        name="wood",
        type="2d",
        builtin="flat",
        rgb1="0.80 0.69 0.55",
        rgb2="0.74 0.62 0.48",
        mark="random",
        random="0.25",
        markrgb="0.70 0.58 0.44",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset, "material", name="wood", texture="wood", texrepeat="4 4", reflectance="0.05"
    )
    if board:
        from sorter.calibration.board import board_texture

        png = board_texture()
        ET.SubElement(asset, "texture", name="board", type="2d", file=str(png))
        ET.SubElement(asset, "material", name="board", texture="board")

    world = ET.SubElement(root, "worldbody")
    # a diffuse lamp: no hard shadow of the arm on the zones (block 1 asks the same of the rig)
    ET.SubElement(
        world,
        "light",
        name="lamp",
        pos="0.25 0 1.2",
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
    lay = cfg.layout
    ET.SubElement(
        world,
        "geom",
        name="table",
        type="box",
        size="0.5 1.2 0.02",  # 1 m deep from the back edge the arm is clamped to
        pos=_f(lay.edge_x_mm / 1000 + 0.5, 0, -0.02),
        material="wood",
        friction="0.8 0.01 0.001",
    )
    bg = lay.background
    ET.SubElement(
        world,
        "geom",
        name="mat",
        type="box",
        size=_f(bg.size_mm[0] / 2000, bg.size_mm[1] / 2000, (0.001 + _SINK) / 2),
        pos=_f(bg.center_mm[0] / 1000, bg.center_mm[1] / 1000, (0.001 - _SINK) / 2),
        rgba="0.49 0.49 0.48 1",
        friction="1.0 0.01 0.001",
    )
    if board:
        from sorter.calibration.board import BOARD

        w, h = BOARD.size_m
        ET.SubElement(
            world,
            "geom",
            name="board",
            type="box",
            size=_f(w / 2, h / 2, 0.0005),
            pos=_f(bg.center_mm[0] / 1000, bg.center_mm[1] / 1000, 0.0015),
            material="board",
            contype="0",
            conaffinity="0",
        )
    if cfg.marks:  # dark tape squares, where the /calibrate page asks for them
        from sorter.calibration.marks import MARK_OFFSETS_MM

        for name, (dx, dy) in MARK_OFFSETS_MM.items():
            ET.SubElement(
                world,
                "geom",
                name=f"mark_{name}",
                type="box",
                size=_f(MARK_MM / 2000, MARK_MM / 2000, 0.0002),
                pos=_f((bg.center_mm[0] + dx) / 1000, (bg.center_mm[1] + dy) / 1000, 0.0012),
                rgba="0.08 0.08 0.1 1",
                contype="0",
                conaffinity="0",
            )
    box = lay.box
    _open_box(
        world,
        "box",
        [c / 1000 for c in box.center_mm],
        [s / 1000 for s in box.size_mm],
        box.floor_z_mm / 1000,
        box.wall_mm / 1000,
        0.01,
        "0.82 0.67 0.49 1",
        "0.72 0.55 0.36 1",
    )
    bins = lay.bins
    inner = (bins.size_mm - 24) / 1000
    for color, (x, y) in bins.centers_mm.items():
        _open_box(
            world,
            f"bin_{color.value}",
            (x / 1000, y / 1000),
            (inner, inner),
            bins.floor_z_mm / 1000,
            bins.wall_mm / 1000,
            0.012,
            _BIN_RGB[color] + " 1",
        )
    _arm(world, cfg)
    rng = np.random.default_rng(cfg.seed + 1000)
    for it in item_specs(cfg):
        _cloth(world, it, rng)

    contact = ET.SubElement(root, "contact")
    ET.SubElement(contact, "exclude", body1="finger_left", body2="finger_right")
    ET.SubElement(contact, "exclude", body1="base_link", body2="link1")
    eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "joint", joint1="finger_right", joint2="finger_left")
    # grip: every cloth vertex can be held by the gripper; PhysicsWorld switches these on for the
    # vertices pinched between the pads and sets where they are held
    for it in item_specs(cfg):
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
    return ET.tostring(root, encoding="unicode")

"""MJCF of the navigation world: the Leo Rover 1.9 with an OAK-D, a room, socks and clutter.

The rover follows `leo_description` (Fictionlab, MIT): base_link 0.198 m above the floor, two
rockers joined by a differential (one goes up as much as the other goes down, an equality
constraint), four wheels on velocity servos with the motors' torque limit. The visual meshes are
the CAD, decimated (`assets/convert_leo.py`); contact uses the URDF's collision shapes (the
chassis and rocker outlines, the tyres as cylinders).

Textures (floors, walls, socks, clothes) and the cloth meshes are generated here from the
scenario's seed and handed to MuJoCo as in-memory assets.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np

from sorter.nav.config import NavConfig
from sorter.nav.scenario import Item, Obstacle, WorldSpec

ASSETS = Path(__file__).parent / "assets" / "leo"
BASE_Z = 0.19783  # base_link above the floor (URDF base_joint)
# contact groups: the rover touches the world and loose items, never itself
ROVER = 'contype="2" conaffinity="1" group="3"'
WORLD = 'contype="1" conaffinity="2"'
LOOSE = 'contype="1" conaffinity="3"'
VISUAL = 'contype="0" conaffinity="0" group="1"'


def build(spec: WorldSpec, cfg: NavConfig) -> tuple[str, dict[str, bytes]]:
    """(MJCF, assets) of the world; `mujoco.MjModel.from_xml_string(xml, assets)` loads it."""
    rng = np.random.default_rng([spec.seed, 99])
    assets: dict[str, bytes] = {}
    asset_xml: list[str] = []
    body_xml: list[str] = []

    _room(spec, rng, assets, asset_xml, body_xml, cfg.leo.tyre_friction)
    for i, it in enumerate(spec.socks):
        _item(f"sock{i}", it, assets, asset_xml, body_xml)
    for i, it in enumerate(spec.distractors):
        _item(f"clutter{i}", it, assets, asset_xml, body_xml)
    for i, ob in enumerate(spec.obstacles):
        body_xml.append(_obstacle(f"obstacle{i}", ob, rng, assets, asset_xml))
    rover_assets, rover_body, actuators = _rover(spec.rover, cfg, assets)
    asset_xml.append(rover_assets)
    body_xml.append(rover_body)

    light = _light(spec.light)
    xml = f"""<mujoco model="leo_nav">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{cfg.timestep_s}" cone="elliptic" integrator="implicitfast"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="4096"/>
    <headlight {light["head"]}/>
    <map znear="0.01" zfar="40"/>
  </visual>
  <default>
    <geom solref="0.004 1" solimp="0.95 0.99 0.001"/>
  </default>
  <asset>
{chr(10).join(asset_xml)}
  </asset>
  <worldbody>
    {light["lights"]}
    <camera name="overview" pos="0 0 {spec.arena_half_m * 2.6:.2f}" xyaxes="1 0 0 0 1 0"
      fovy="45"/>
{chr(10).join(body_xml)}
  </worldbody>
  <equality>
    <joint joint1="rocker_R" joint2="rocker_L" polycoef="0 1 0 0 0" solref="0.005 1"/>
  </equality>
  <actuator>
{actuators}
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>"""
    return xml, assets


# --- the rover ---


def _rover(pose, cfg: NavConfig, assets: dict[str, bytes]) -> tuple[str, str, str]:
    manifest = json.loads((ASSETS / "manifest.json").read_text())
    asset_lines = []
    for f in {e[0] for part in manifest.values() for e in part} | {
        "chassis_outline.stl",
        "rocker_outline.stl",
    }:
        assets[f"leo/{f}"] = (ASSETS / f).read_bytes()
        asset_lines.append(f'    <mesh name="{Path(f).stem}" file="leo/{f}"/>')

    def visuals(part: str) -> str:
        return "\n".join(
            f'<geom type="mesh" mesh="{Path(f).stem}" rgba="{_f(rgba)}" {VISUAL}/>'
            for f, rgba in manifest[part]
        )

    leo = cfg.leo
    fr = leo.tyre_friction

    def wheel(name: str, model: str, pos: str, axis: str) -> str:
        # the wheel link's origin is its outer face; the tyre extends 0.0 - 0.09 m inward (+y)
        return f"""
        <body name="wheel_{name}" pos="{pos}">
          <joint name="wheel_{name}" type="hinge" axis="{axis}" damping="0.02" armature="0.002"/>
          <inertial pos="0 0.030026 0" mass="0.283642"
            fullinertia="0.000391 0.0004716 0.000391 0.00000124 0.00000055 -0.00000208"/>
          {visuals("wheel" + model.lower())}
          <geom name="tyre_{name}" type="ellipsoid" pos="0 0.04485 0" size="0.0625 0.035 0.0625"
            friction="{fr} 0.01 0.0001" condim="3" {ROVER} rgba="0 0 0 0"/>
        </body>"""

    def rocker(side: str) -> str:
        y, quat = (0.14167, "0 0 0 1") if side == "L" else (-0.14167, "1 0 0 0")
        axis = "0 -1 0" if side == "L" else "0 1 0"  # both: positive = forward (URDF axes)
        front, rear = ("FL", "RL") if side == "L" else ("FR", "RR")
        fx = -0.15256 if side == "L" else 0.15256  # the left rocker is turned half round
        a, b = ("A", "A") if side == "L" else ("B", "B")
        return f"""
      <body name="rocker_{side}" pos="0.00263 {y} -0.04731" quat="{quat}">
        <joint name="rocker_{side}" type="hinge" axis="0 1 0" range="-0.24 0.24" damping="0.5"/>
        <inertial pos="0 0.01346 -0.06506" mass="1.387336"
          fullinertia="0.002956 0.02924 0.02832 -0.0000015 -0.0000081 0.0000711"/>
        {visuals("rocker")}
        <geom type="mesh" mesh="rocker_outline" {ROVER} rgba="0 0 0 0"/>
        {wheel(front, a, f"{fx} -0.08214 -0.08802", axis)}
        {wheel(rear, b, f"{-fx} -0.08214 -0.08802", axis)}
      </body>"""

    cam = cfg.camera
    p = math.radians(cam.pitch_deg)
    fwd = np.array([math.cos(p), 0.0, -math.sin(p)])
    up = np.array([math.sin(p), 0.0, math.cos(p)])
    right = np.cross(fwd, up)  # rover -y when level
    fovy = 2 * math.degrees(
        math.atan(math.tan(math.radians(cam.rgb_hfov_deg) / 2) * cam.height / cam.width)
    )
    mx, my, mz = cam.mount_xyz_m
    x, y, yaw = pose
    body = f"""
    <body name="base_link" pos="{x:.4f} {y:.4f} {BASE_Z + 0.004:.4f}" quat="{_f(_yaw_quat(yaw))}">
      <freejoint name="rover"/>
      <inertial pos="-0.019662 0.011643 -0.031802" mass="1.584994"
        fullinertia="0.01042 0.01045 0.01817 0.001177 -0.0008871 0.0002226"/>
      {visuals("chassis")}
      <geom type="mesh" mesh="chassis_outline" {ROVER} rgba="0 0 0 0"/>
      <site name="imu" pos="0.0628 -0.0314 -0.0393"/>
      <body name="antenna" pos="-0.0052 0.056 -0.0065">
        {visuals("antenna")}
      </body>
      <body name="oakd_bracket" pos="{mx - 0.005:.4f} {my:.4f} 0.006">
        <geom type="box" size="0.012 0.03 {(mz - 0.006 - 0.02) / 2:.4f}"
          pos="0 0 {(mz - 0.006 - 0.02) / 2:.4f}" rgba="0.15 0.15 0.16 1" {VISUAL}/>
      </body>
      <body name="oakd" pos="{mx:.4f} {my:.4f} {mz:.4f}" euler="0 {p:.4f} 0">
        <inertial pos="0 0 0" mass="0.115" diaginertia="0.0001 0.00003 0.0001"/>
        <geom type="box" size="0.0165 0.055 0.0273" pos="-0.0165 0 0" rgba="0.2 0.21 0.23 1"
          {VISUAL}/>
        <geom type="box" size="0.0167 0.0551 0.004" pos="-0.0165 0 0.0233" rgba="0.72 0.73 0.75 1"
          {VISUAL}/>
        <geom type="cylinder" size="0.008 0.001" pos="0.0001 0 0" euler="0 1.5708 0"
          rgba="0.02 0.02 0.03 1" {VISUAL}/>
        <geom type="cylinder" size="0.007 0.001" pos="0.0001 0.0375 0" euler="0 1.5708 0"
          rgba="0.02 0.02 0.03 1" {VISUAL}/>
        <geom type="cylinder" size="0.007 0.001" pos="0.0001 -0.0375 0" euler="0 1.5708 0"
          rgba="0.02 0.02 0.03 1" {VISUAL}/>
      </body>
      <camera name="oakd_rgb" pos="{mx + 0.002:.4f} {my:.4f} {mz:.4f}"
        xyaxes="{_f(right)} {_f(up)}" fovy="{fovy:.3f}"/>
      <camera name="chase" pos="-1.25 0 0.95" xyaxes="0 -1 0 0.6 0 0.8" fovy="55"/>
      {rocker("L")}
      {rocker("R")}
    </body>"""
    kv, tq = 8.0, leo.wheel_torque_nm  # the firmware's wheel PID tracks speed tightly
    actuators = "\n".join(
        f'    <velocity name="{w}" joint="wheel_{w}" kv="{kv}" forcerange="{-tq} {tq}" '
        f'ctrlrange="{-leo.max_wheel_rps} {leo.max_wheel_rps}"/>'
        for w in ("FL", "RL", "FR", "RR")
    )
    return "\n".join(asset_lines), body, actuators


# --- the room ---


def _room(spec: WorldSpec, rng, assets, asset_xml, body_xml, floor_mu: float) -> None:
    tile_m = {"wood": 1.2, "carpet": 0.8, "tile": 0.6, "concrete": 2.0, "plain": 2.0, "rug": 1.0}
    assets["floor.png"] = _png(_floor_texture(spec.floor, rng))
    asset_xml.append('    <texture name="floor" type="2d" file="floor.png"/>')
    asset_xml.append(
        f'    <material name="floor" texture="floor" texuniform="true" '
        f'texrepeat="{1 / tile_m[spec.floor]:.3f} {1 / tile_m[spec.floor]:.3f}" '
        f'reflectance="0.02"/>'
    )
    assets["wall.png"] = _png(_paint_texture(rng))
    asset_xml.append('    <texture name="wall" type="2d" file="wall.png"/>')
    asset_xml.append('    <material name="wall" texture="wall" texuniform="true" texrepeat="1 1"/>')
    h = spec.arena_half_m
    body_xml.append(
        f'    <geom name="floor" type="plane" size="{h + 1} {h + 1} 0.1" material="floor" '
        f'friction="{floor_mu} 0.01 0.0001" {WORLD}/>'
    )
    wall_h = 0.8
    for i, (cx, cy, sx, sy) in enumerate(
        [
            (h + 0.05, 0, 0.05, h + 0.1),
            (-h - 0.05, 0, 0.05, h + 0.1),
            (0, h + 0.05, h + 0.1, 0.05),
            (0, -h - 0.05, h + 0.1, 0.05),
        ]
    ):
        body_xml.append(
            f'    <geom name="wall{i}" type="box" pos="{cx} {cy} {wall_h / 2}" '
            f'size="{sx} {sy} {wall_h / 2}" material="wall" {WORLD}/>'
        )
        # a skirting board: a strong horizontal edge low on the wall
        body_xml.append(
            f'    <geom type="box" pos="{cx - 0.062 * np.sign(cx)} {cy - 0.062 * np.sign(cy)} '
            f'0.04" size="{sx if sx > 0.1 else 0.012} {sy if sy > 0.1 else 0.012} 0.04" '
            f'rgba="0.95 0.95 0.93 1" {VISUAL}/>'
        )


def _obstacle(name: str, ob: Obstacle, rng, assets, asset_xml) -> str:
    tex = f"{name}.png"
    assets[tex] = _png(_fabric_texture(ob.color, rng, grain=0.05))
    asset_xml.append(f'    <texture name="{name}" type="2d" file="{tex}"/>')
    asset_xml.append(
        f'    <material name="{name}" texture="{name}" texuniform="true" texrepeat="2 2"/>'
    )
    sx, sy, sz = ob.size
    if ob.kind == "box":
        geom = f'type="box" size="{sx:.3f} {sy:.3f} {sz:.3f}"'
    else:
        geom = f'type="cylinder" size="{sx:.3f} {sz:.3f}"'
    return (
        f'    <body name="{name}" pos="{ob.pos[0]:.3f} {ob.pos[1]:.3f} {sz:.3f}" '
        f'quat="{_f(_yaw_quat(ob.yaw))}">\n'
        f'      <geom {geom} material="{name}" {WORLD}/>\n    </body>'
    )


def _light(kind: str) -> dict[str, str]:
    if kind == "dim":
        return {
            "head": 'ambient="0.10 0.10 0.10" diffuse="0.15 0.15 0.15" specular="0 0 0"',
            "lights": '<light pos="1 -1 3" dir="-0.2 0.2 -1" diffuse="0.35 0.33 0.30" '
            'directional="true" castshadow="true"/>',
        }
    if kind == "side":
        return {
            "head": 'ambient="0.18 0.18 0.18" diffuse="0.2 0.2 0.2" specular="0 0 0"',
            "lights": '<light pos="-4 2 1.2" dir="1 -0.4 -0.35" diffuse="0.9 0.85 0.75" '
            'directional="true" castshadow="true"/>',
        }
    return {
        "head": 'ambient="0.30 0.30 0.30" diffuse="0.35 0.35 0.35" specular="0.05 0.05 0.05"',
        "lights": '<light pos="1 -1 4" dir="-0.25 0.25 -1" diffuse="0.6 0.6 0.58" '
        'directional="true" castshadow="true"/>'
        '<light pos="-2 2 3" dir="0.3 -0.3 -1" diffuse="0.2 0.2 0.22" '
        'directional="true" castshadow="false"/>',
    }


# --- loose items ---


def _item(name: str, it: Item, assets, asset_xml, body_xml) -> None:
    rng = np.random.default_rng(it.seed)
    pos = f"{it.pos[0]:.4f} {it.pos[1]:.4f}"
    quat = _f(_yaw_quat(it.yaw))
    if it.kind in ("sock_flat", "sock_bunched", "shirt", "towel"):
        if it.kind == "sock_flat":
            v, f, uv = _flat_sock(rng, it.scale)
            tex, mass = _sock_texture(it.color, rng), 0.04
        elif it.kind == "sock_bunched":
            v, f, uv = _bunched_sock(rng, it.scale)
            tex, mass = _sock_texture(it.color, rng), 0.04
        elif it.kind == "shirt":
            v, f, uv = _extrude(_shirt_outline(rng), 0.012, rng)
            tex, mass = _fabric_texture(it.color, rng, print_=True), 0.18
        else:
            w, h = rng.uniform(0.3, 0.45), rng.uniform(0.2, 0.3)
            v, f, uv = _extrude([(-w, -h), (w, -h), (w, h), (-w, h)], 0.01, rng)
            tex, mass = _fabric_texture(it.color, rng, stripes=True), 0.12
        assets[f"{name}.png"] = _png(tex)
        asset_xml.append(f'    <texture name="{name}" type="2d" file="{name}.png"/>')
        asset_xml.append(f'    <material name="{name}" texture="{name}"/>')
        asset_xml.append(
            f'    <mesh name="{name}" inertia="shell" vertex="{_f(v.ravel(), 4)}" '
            f'face="{" ".join(map(str, f.ravel()))}" texcoord="{_f(uv.ravel(), 4)}"/>'
        )
        body_xml.append(
            f'    <body name="{name}" pos="{pos} 0.002" quat="{quat}">\n'
            f"      <freejoint/>\n"
            f'      <geom name="{name}" type="mesh" mesh="{name}" material="{name}" mass="{mass}" '
            f'friction="0.9 0.01 0.001" {LOOSE}/>\n    </body>'
        )
        return
    c = _f(it.color)
    if it.kind == "ball":
        r = float(rng.uniform(0.05, 0.1))
        geoms = f'<geom type="sphere" size="{r:.3f}" pos="0 0 {r:.3f}" rgba="{c} 1" mass="0.1"'
        geoms += f" {LOOSE}/>"
    elif it.kind == "shoe":
        geoms = (
            f'<geom type="capsule" fromto="-0.09 0 0.045 0.09 0 0.04" size="0.04" rgba="{c} 1" '
            f'mass="0.3" {LOOSE}/><geom type="box" size="0.13 0.045 0.012" pos="0 0 0.012" '
            f'rgba="0.95 0.95 0.95 1" mass="0.1" {LOOSE}/>'
        )
    else:  # toy block
        s = float(rng.uniform(0.03, 0.06))
        geoms = f'<geom type="box" size="{s:.3f} {s:.3f} {s:.3f}" pos="0 0 {s:.3f}" '
        geoms += f'rgba="{c} 1" mass="0.05" {LOOSE}/>'
    body_xml.append(
        f'    <body name="{name}" pos="{pos} 0.001" quat="{quat}"><freejoint/>{geoms}</body>'
    )


def _flat_sock(rng, scale: float):
    """A sock lying flat on its side: the leg, a bend at the heel, the foot, a round toe.

    Built as a strip along its centerline (u along the sock, v across) with a domed top, so the
    knit texture's ribs run along the sock like the real thing. Faces are wound outward by
    construction: i runs along the sock, j from its right edge to its left.
    """
    w = rng.uniform(0.075, 0.09) * scale
    leg, foot = rng.uniform(0.13, 0.18) * scale, rng.uniform(0.14, 0.18) * scale
    turn = math.radians(rng.uniform(55, 80)) * (1 if rng.random() < 0.5 else -1)
    r_bend = w / 2 + 0.012
    # (length, curvature, width scale) pieces of the centerline; the toe closes like an ellipse
    pieces = [(leg, 0.0), (r_bend * abs(turn), turn / (r_bend * abs(turn))), (foot, 0.0)]
    ds = 0.012
    x, y, phi = 0.0, 0.0, math.pi / 2
    pts, heads, widths = [(x, y)], [phi], [w]
    for length, kappa in pieces:
        steps = max(2, int(length / ds))
        for _ in range(steps):
            h = length / steps
            phi += kappa * h / 2
            x, y = x + h * math.cos(phi), y + h * math.sin(phi)
            phi += kappa * h / 2
            pts.append((x, y))
            heads.append(phi)
            widths.append(w * (1.08 if kappa else 1.0))
    for k in np.linspace(0, 1, 7)[1:]:
        pts.append((x + k * w / 2 * math.cos(phi), y + k * w / 2 * math.sin(phi)))
        heads.append(phi)
        widths.append(max(w * math.sqrt(max(1 - k * k, 0)), 0.006))
    pts_a = np.array(pts)
    arc = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts_a, axis=0), axis=1))])
    u = arc / arc[-1]
    m = 8  # across
    thick = rng.uniform(0.008, 0.013) * scale
    top, bot, uvs = [], [], []
    wr = rng.normal(0, 1, 6)
    for i, ((px, py), hd, wd) in enumerate(zip(pts, heads, widths, strict=True)):
        rx, ry = math.sin(hd), -math.cos(hd)  # the sock's right
        for j in range(m + 1):
            s = 1 - 2 * j / m  # +1 right edge, -1 left edge
            wrinkle = 0.0015 * sum(wr[k] * math.sin((k + 1) * 7 * u[i] + wr[-k]) for k in range(3))
            z = thick * (0.45 + 0.55 * (1 - s * s)) + wrinkle
            vx, vy = px + rx * s * wd / 2, py + ry * s * wd / 2
            top.append((vx, vy, max(z, 0.002)))
            bot.append((vx, vy, 0.0))
            uvs.append((u[i], j / m))
    n = len(pts)
    nv = n * (m + 1)

    def idx(i, j):
        return i * (m + 1) + j

    faces = []
    for i in range(n - 1):
        for j in range(m):
            a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
            faces += [(a, b, c), (a, c, d), (nv + a, nv + c, nv + b), (nv + a, nv + d, nv + c)]
        a, b = idx(i, 0), idx(i + 1, 0)  # right side
        faces += [(a, nv + a, nv + b), (a, nv + b, b)]
        a, b = idx(i, m), idx(i + 1, m)  # left side
        faces += [(a, b, nv + b), (a, nv + b, nv + a)]
    for j in range(m):
        a, b = idx(0, j), idx(0, j + 1)  # the cuff end
        faces += [(a, b, nv + b), (a, nv + b, nv + a)]
        a, b = idx(n - 1, j), idx(n - 1, j + 1)  # the toe tip
        faces += [(a, nv + b, b), (a, nv + a, nv + b)]
    v = np.array(top + bot)
    v[:, :2] -= v[:nv, :2].mean(axis=0)
    return v, _clean(v, np.array(faces)), np.array(uvs + uvs)


def _bunched_sock(rng, scale: float):
    """A sock rolled into a lumpy ball, flat where it lies on the floor."""
    a, b, c = (
        rng.uniform(0.06, 0.08) * scale,
        rng.uniform(0.04, 0.055) * scale,
        rng.uniform(0.028, 0.038) * scale,
    )
    nu, nv = 28, 14
    k = rng.normal(0, 1, 8)
    v, uv = [], []
    for i in range(nv + 1):
        th = math.pi * i / nv
        for j in range(nu + 1):
            ph = 2 * math.pi * j / nu
            lump = 1 + 0.12 * (
                k[0] * math.sin(3 * ph + k[1]) * math.sin(2 * th)
                + 0.5 * k[2] * math.sin(5 * ph + k[3]) * math.sin(th)
            )
            x = a * lump * math.sin(th) * math.cos(ph)
            y = b * lump * math.sin(th) * math.sin(ph)
            z = c * lump * math.cos(th)
            v.append((x, y, z))
            uv.append((j / nu, i / nv))
    v = np.array(v)
    v[:, 2] = np.maximum(v[:, 2], -c * 0.55)
    v[:, 2] -= v[:, 2].min()
    faces = []
    for i in range(nv):
        for j in range(nu):
            p0, p1 = i * (nu + 1) + j, i * (nu + 1) + j + 1
            q0, q1 = p0 + nu + 1, p1 + nu + 1
            faces += [(p0, q0, p1), (p1, q0, q1)]
    return v, _clean(v, np.array(faces)), np.array(uv)


def _shirt_outline(rng) -> list[tuple[float, float]]:
    s = rng.uniform(0.9, 1.15)
    pts = [
        (-0.2, -0.3),
        (0.2, -0.3),
        (0.2, 0.12),
        (0.34, 0.02),
        (0.4, 0.12),
        (0.22, 0.3),
        (0.07, 0.3),
        (0.0, 0.26),
        (-0.07, 0.3),
        (-0.22, 0.3),
        (-0.4, 0.12),
        (-0.34, 0.02),
        (-0.2, 0.12),
    ]
    return [(x * s, y * s) for x, y in pts]


def _extrude(outline, thick: float, rng):
    """A flat cloth from a 2D outline (CCW): top and bottom caps (ear clipping) and the rim."""
    p = np.array(outline, dtype=float)
    tri = _ear_clip(p)
    n = len(p)
    lo, hi = p.min(axis=0), p.max(axis=0)
    uv2 = (p - lo) / (hi - lo)
    top = np.column_stack([p, np.full(n, thick) + rng.normal(0, 0.001, n)])
    bot = np.column_stack([p, np.zeros(n)])
    v = np.vstack([top, bot])
    faces = [tuple(t) for t in tri] + [(n + a, n + c, n + b) for a, b, c in tri]
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, n + i, n + j), (i, n + j, j)]
    v[:, :2] -= p.mean(axis=0)
    return v, _clean(v, np.array(faces)), np.vstack([uv2, uv2])


def _ear_clip(p: np.ndarray) -> list[tuple[int, int, int]]:
    idx = list(range(len(p)))
    if _area(p) < 0:
        idx.reverse()
    out = []
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = p[i0], p[i1], p[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 0:
                continue
            if any(_in_tri(p[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            out.append((i0, i1, i2))
            idx.pop(k)
            break
    out.append(tuple(idx))
    return out


def _area(p) -> float:
    return 0.5 * float(np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]))


def _in_tri(q, a, b, c) -> bool:
    def s(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    d1, d2, d3 = s(q, a, b), s(q, b, c), s(q, c, a)
    return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))


def _clean(v: np.ndarray, f: np.ndarray) -> np.ndarray:
    """Drop degenerate faces (the poles of the bunched sock, the toe tip)."""
    n = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    return f[np.linalg.norm(n, axis=1) > 1e-12]


# --- textures ---


def _png(rgb: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(
        ".png", cv2.cvtColor(np.clip(rgb, 0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR)
    )
    assert ok
    return buf.tobytes()


def _noise(rng, shape, blur: float) -> np.ndarray:
    n = rng.normal(0, 1, shape).astype(np.float32)
    if blur > 0:  # in the frequency domain, so the texture still tiles
        fy = np.fft.fftfreq(shape[0])[:, None]
        fx = np.fft.fftfreq(shape[1])[None, :]
        g = np.exp(-2 * (np.pi * blur) ** 2 * (fx**2 + fy**2))
        n = np.real(np.fft.ifft2(np.fft.fft2(n) * g)).astype(np.float32)
        n /= n.std() + 1e-6
    return n


def _floor_texture(kind: str, rng) -> np.ndarray:
    s = 512
    yy, xx = np.mgrid[0:s, 0:s].astype(np.float32)
    if kind == "wood":
        img = np.zeros((s, s, 3), np.float32)
        planks = 7
        pw = s / planks
        for k in range(planks):
            base = np.array([150, 105, 65]) * rng.uniform(0.8, 1.15)
            x0, x1 = int(k * pw), int((k + 1) * pw)
            grain = np.sin(
                yy[:, x0:x1] * rng.uniform(0.05, 0.09)
                + 3 * _noise(rng, (s, x1 - x0), 6)
                + xx[:, x0:x1] * 0.3
            )
            img[:, x0:x1] = base * (0.88 + 0.08 * grain[..., None])
            img[:, x0 : x0 + 2] *= 0.55
            cut = int(rng.integers(s))
            img[cut : cut + 2, x0:x1] *= 0.6
        img *= (1 + 0.04 * _noise(rng, (s, s), 1))[..., None]
    elif kind == "carpet":
        base = np.array(rng.choice([[110, 95, 85], [70, 80, 95], [120, 115, 105], [85, 60, 60]]))
        img = base * (
            1
            + 0.14 * _noise(rng, (s, s), 0.7)[..., None]
            + 0.05 * _noise(rng, (s, s), 12)[..., None]
        )
    elif kind == "tile":
        base = np.array(rng.choice([[215, 212, 205], [190, 180, 165], [160, 165, 170]]))
        img = base * (
            1 + 0.05 * _noise(rng, (s, s), 8)[..., None] + 0.02 * _noise(rng, (s, s), 1)[..., None]
        )
        g = (xx % (s // 2) < 5) | (yy % (s // 2) < 5)
        img[g] = base * 0.62
    elif kind == "concrete":
        img = np.array([150, 150, 148]) * (
            1
            + 0.08 * _noise(rng, (s, s), 20)[..., None]
            + 0.06 * _noise(rng, (s, s), 1.2)[..., None]
        )
    elif kind == "plain":
        img = np.array([205, 203, 198]) * (1 + 0.003 * _noise(rng, (s, s), 2)[..., None])
    else:  # rug: a bold geometric pattern
        c1, c2, c3 = (np.array(c) for c in ([150, 40, 45], [230, 210, 170], [40, 60, 110]))
        d = (np.abs((xx % 128) - 64) + np.abs((yy % 128) - 64)) / 64
        img = np.where((d < 0.5)[..., None], c1, np.where((d < 0.8)[..., None], c2, c3))
        img = img * (1 + 0.06 * _noise(rng, (s, s), 0.8)[..., None])
    return img


def _paint_texture(rng) -> np.ndarray:
    base = np.array(rng.choice([[225, 222, 212], [200, 210, 215], [215, 205, 190]]))
    return base * (1 + 0.025 * _noise(rng, (256, 256), 3)[..., None])


def _sock_texture(color, rng) -> np.ndarray:
    """Knit: ribs along the sock, a ribbed cuff, often a contrasting heel and toe or stripes.
    u (along the sock) is the image's x, v (across) its y."""
    w, h = 256, 64
    base = np.array(color) * 255
    uu, vv = np.meshgrid(np.linspace(0, 1, w), np.linspace(0, 1, h))
    rib = 0.93 + 0.07 * np.cos(2 * np.pi * vv * 14)
    img = base * rib[..., None]
    cuff = uu < 0.13
    img[cuff] = (base * (0.9 + 0.1 * np.cos(2 * np.pi * vv * 26))[..., None])[cuff]
    lum = base.mean()
    accent = (
        np.array(
            rng.choice(
                [[200, 40, 40], [40, 60, 150], [230, 200, 50], [30, 30, 30], [240, 240, 240]]
            )
        )
        if lum > 60
        else np.array([90, 90, 95])
    )
    style = rng.choice(["plain", "heel_toe", "stripes", "cuff_band"])
    if style in ("heel_toe", "stripes"):
        heel = (uu > 0.4) & (uu < 0.55)
        toe = uu > 0.88
        shade = accent if style == "heel_toe" else base * (0.75 if lum > 128 else 1.4)
        img[heel | toe] = (np.clip(shade, 0, 255) * rib[..., None])[heel | toe]
    if style == "stripes":
        band = ((uu * 10) % 1 < 0.35) & (uu > 0.14) & (uu < 0.4)
        img[band] = (accent * rib[..., None])[band]
    if style == "cuff_band":
        band = (uu > 0.03) & (uu < 0.08)
        img[band] = (accent * rib[..., None])[band]
    img = img * (1 + 0.05 * _noise(rng, (h, w), 0.6)[..., None])
    return img


def _fabric_texture(
    color, rng, grain: float = 0.06, print_: bool = False, stripes: bool = False
) -> np.ndarray:
    s = 256
    base = np.array(color) * 255
    img = base * (1 + grain * _noise(rng, (s, s), 0.8)[..., None])
    if stripes:
        yy = np.mgrid[0:s, 0:s][0]
        band = (yy // 24) % 2 == 0
        img[band] *= 0.75
    if print_ and rng.random() < 0.6:
        c = np.array(rng.uniform(0, 255, 3))
        cv2.circle(img, (s // 2, s // 2 - 20), int(rng.uniform(25, 50)), c.tolist(), -1)
    return img


# --- small helpers ---


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def _f(vals, nd: int = 6) -> str:
    return " ".join(f"{float(x):.{nd}g}" for x in vals)

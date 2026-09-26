"""Poses and zones for the table layout (`sim.layout`), computed with the arm's IK.

    uv run python -m sorter.sim.layout            # print them and check every pick
    uv run python -m sorter.sim.layout --write    # also write config/rig.yaml (poses, zones)

The look poses point the wrist camera straight down over the zone center, as high as the arm
can hold the gripper vertical. The check plans a full pick (above, down, up) on a grid over each
zone workspace and every move between the named poses.
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np
import yaml

from sorter.arm import kinematics as kin
from sorter.arm.config import POSE_NAMES, ArmConfig, ZoneConfig
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.errors import SorterError
from sorter.core.types import ArmPoint, Zone
from sorter.sim.config import RectConfig, SimConfig
from sorter.sim.world import camera_mount

LOOK_TCP_Z_MM = (120, 60)  # the highest TCP height tried for a look pose, then lower in 5 mm steps
BOX_MARGIN_MM = 25.0  # the grasp stays this far from the box walls (fingers + wrist camera)
BG_MARGIN_MM = 20.0
HOME_TCP_MM = (220.0, 0.0, 200.0)
HOME_APPROACH = (1.0, 0.0, -1.0)  # 45° down, forward
_SEED = (0.0, 1.2, 1.5, 0.0, 0.0, 0.0)  # elbow up


def _seed(x: float, y: float) -> np.ndarray:
    s = np.array(_SEED)
    s[0] = -math.atan2(y, x)  # joint 1 turns clockwise for a positive angle (URDF axis −z)
    return s


def _rect_polygon(r: RectConfig, margin: float) -> list[tuple[float, float]]:
    (cx, cy), (w, h) = r.center_mm, r.size_mm
    x0, x1, y0, y1 = (
        cx - w / 2 + margin,
        cx + w / 2 - margin,
        cy - h / 2 + margin,
        cy + h / 2 - margin,
    )
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _roll_error(q: np.ndarray, T_tcp_cam: np.ndarray) -> float:
    """Angle of the image u axis from the arm's x axis, folded to [-pi/2, pi/2]."""
    u = (kin.fk_tcp(q) @ T_tcp_cam)[:3, 0]
    a = math.atan2(u[1], u[0])
    return (a + math.pi / 2) % math.pi - math.pi / 2


def _square_up(q: np.ndarray, T_tcp_cam: np.ndarray, prefer: float | None = None) -> np.ndarray:
    """Turn the wrist (joint 6 turns about the approach axis) so the image's long side runs
    along the arm's x axis, like the long sides of the box and the mat. Of the two ways, the one
    nearest `prefer` (joint 6, rad), so the off-axis camera doesn't jump sides."""
    lo, hi = kin.JOINT_LIMITS[5]
    prefer = q[5] if prefer is None else prefer
    cands = []
    for k in (-1, 0, 1):
        for sign in (1, -1):
            c = q.copy()
            c[5] = q[5] + sign * _roll_error(q, T_tcp_cam) + k * math.pi
            if lo <= c[5] <= hi:
                cands.append((round(abs(_roll_error(c, T_tcp_cam)), 4), abs(c[5] - prefer), c))
    return min(cands, key=lambda t: t[:2])[2] if cands else q


def axis_hit(T_base_cam: np.ndarray, z_mm: float) -> np.ndarray | None:
    """Where the camera's optical axis meets the plane z = `z_mm` (x, y), or None."""
    o, d = T_base_cam[:3, 3], T_base_cam[:3, 2]
    if d[2] > -0.1:  # not looking down
        return None
    return (o + d * (z_mm - o[2]) / d[2])[:2]


def camera_over(
    arm: ArmConfig,
    center: Sequence[float],
    tcp_z: float,
    T_tcp_cam: np.ndarray,
    surface_z: float = 0.0,
) -> np.ndarray | None:
    """Joints with the gripper vertical, the TCP at `tcp_z`, and the camera's optical axis
    through `center` on the surface at `surface_z`, the image long side along the arm's x axis.
    None if the arm can't."""
    target = np.array([*center, tcp_z], dtype=float)
    q = _seed(*center)
    roll = None
    # the camera sits off the TCP: shift the TCP until the camera is centered
    for _ in range(10):
        q = kin.solve(target, "down", q, arm.z_min_mm)
        if q is None:
            return None
        q = _square_up(q, T_tcp_cam, roll)
        roll = q[5]
        hit = axis_hit(kin.fk_tcp(q) @ T_tcp_cam, surface_z)
        if hit is None:
            return None
        err = np.asarray(center, dtype=float) - hit
        if np.hypot(*err) < 0.5 and abs(_roll_error(q, T_tcp_cam)) < math.radians(0.5):
            return q
        target[:2] += err
    return None


def look_pose(sim: SimConfig, arm: ArmConfig, center: tuple[float, float]) -> np.ndarray:
    """Joints that put the camera straight down over `center`, as high as possible, the image
    long side along the arm's x axis."""
    T_tcp_cam = camera_mount(sim)
    for tcp_z in range(LOOK_TCP_Z_MM[0], LOOK_TCP_Z_MM[1] - 1, -5):
        q = camera_over(arm, center, tcp_z, T_tcp_cam)
        if q is not None:
            return q
    raise SystemExit(f"no look pose puts the camera straight down over {center}")


def zone_rois(cfg: Config, poses: dict[str, list[float]]) -> dict[str, dict]:
    """`views.<zone>.roi`: the box inside and the mat, projected into the image from the look
    poses, minus the rows the gripper hides at the top of the frame."""
    from sorter.sim.physics.camera import PhysicsCamera
    from sorter.sim.physics.world import PhysicsWorld

    sim = cfg.sim.model_copy(update={"items": [], "realtime": 0})
    world = PhysicsWorld(sim, poses)
    camera = PhysicsCamera(world)
    lay = cfg.sim.layout
    zones = {
        "box": (lay.box, lay.box.floor_z_mm, 0.0, "look_box"),
        "background": (lay.background, 0.0, 5.0, "look_bg"),
    }
    out = {}
    try:
        for zone, (rect, z, margin, pose) in zones.items():
            q = np.asarray(poses[pose], dtype=float)
            world.teleport_arm(q, finger_m=0.0)
            frame = camera.fresh(5.0)
            no_depth = (frame.depth_mm == 0).mean(axis=1) > 0.3
            top = int(np.argmin(no_depth)) + 10 if no_depth[0] else 0  # rows hidden by the gripper
            T = np.linalg.inv(kin.fk_tcp(q) @ camera_mount(cfg.sim))
            pts = []
            for x, y in _rect_polygon(rect, margin):
                c = T @ np.array([x, y, z, 1.0])
                u = sim.focal_px * c[0] / c[2] + sim.width / 2
                v = sim.focal_px * c[1] / c[2] + sim.height / 2
                pts.append(
                    (
                        int(np.clip(round(u), 0, sim.width - 1)),
                        int(np.clip(round(v), top, sim.height - 1)),
                    )
                )
            out[zone] = {"roi": [list(p) for p in pts]}
    finally:
        camera.close()
    return out


def compute_poses(sim: SimConfig, arm: ArmConfig) -> dict[str, list[float]]:
    lay = sim.layout
    poses: dict[str, np.ndarray] = {"rest": np.zeros(kin.N_JOINTS)}
    home = kin.solve(HOME_TCP_MM, HOME_APPROACH, _seed(*HOME_TCP_MM[:2]), arm.z_min_mm)
    if home is None:
        raise SystemExit(f"home {HOME_TCP_MM} is not reachable")
    poses["home"] = home
    poses["look_box"] = look_pose(sim, arm, lay.box.center_mm)
    poses["look_bg"] = look_pose(sim, arm, lay.background.center_mm)
    bx, by = lay.background.center_mm
    place = (bx, by, arm.place_release_height_mm)
    q = kin.solve(place, "down", _seed(bx, by), arm.z_min_mm)
    if q is None:
        raise SystemExit(f"place_bg {place} is not reachable with the gripper down")
    poses["place_bg"] = q
    z = lay.bins.floor_z_mm + lay.bins.wall_mm + arm.bin_release_height_mm
    for color, (x, y) in lay.bins.centers_mm.items():
        r = math.hypot(x, y)
        tilted = (x / r, y / r, -1.0)  # down and outwards
        for approach in ("down", tilted, None):
            q = kin.solve((x, y, z), approach, _seed(x, y), arm.z_min_mm)
            if q is not None:
                break
        else:
            raise SystemExit(f"bin_{color} ({x:.0f}, {y:.0f}, {z:.0f}) is not reachable")
        poses[f"bin_{color}"] = q
    return {name: [round(float(v), 4) for v in poses[name]] for name in POSE_NAMES}


def compute_zones(sim: SimConfig, arm: ArmConfig) -> dict[Zone, ZoneConfig]:
    lay = sim.layout
    return {
        Zone.BOX: ZoneConfig(
            workspace_mm=_rect_polygon(lay.box, BOX_MARGIN_MM),
            z_floor_mm=lay.box.floor_z_mm + 5,
        ),
        Zone.BACKGROUND: ZoneConfig(
            workspace_mm=_rect_polygon(lay.background, BG_MARGIN_MM),
            z_floor_mm=max(arm.z_min_mm, 3.0),
            lift_z_mm=70.0,  # no wall to clear; the far edge can't hold the gripper vertical higher
        ),
    }


def check(cfg: Config, step_mm: float = 20.0) -> list[str]:
    """Problems with `cfg.poses` / `cfg.zones` on the layout; [] = every pick and move plans."""
    from sorter.arm.controller import Controller
    from sorter.sim.driver import SimDriver
    from sorter.sim.world import SimWorld

    problems = []
    arm = Controller(SimDriver(SimWorld(cfg.sim, cfg.poses)), cfg.arm, cfg.poses, cfg.zones)
    q = {n: np.asarray(v) for n, v in cfg.poses.items()}
    moves = [
        ("rest", "home"),
        ("home", "look_bg"),
        ("look_bg", "look_box"),
        ("place_bg", "look_bg"),
    ]
    moves += [(b, "look_bg") for b in q if b.startswith("bin_")]
    moves += [("place_bg", b) for b in q if b.startswith("bin_")]
    for a, b in moves:
        try:
            kin.plan_joints(q[a], q[b], z_min_mm=cfg.arm.z_min_mm)
            kin.plan_joints(q[b], q[a], z_min_mm=cfg.arm.z_min_mm)
        except SorterError as e:
            problems.append(f"move {a} ↔ {b}: {e}")
    lay = cfg.sim.layout
    backs = {"box": lay.box.center_mm[0] - lay.box.size_mm[0] / 2 - 10}  # + the 10 mm wall
    backs["background"] = lay.background.center_mm[0] - lay.background.size_mm[0] / 2
    backs |= {f"bin_{c}": x - lay.bins.size_mm / 2 for c, (x, _) in lay.bins.centers_mm.items()}
    for name, x in backs.items():
        if x < lay.edge_x_mm:
            problems.append(f"{name} hangs {lay.edge_x_mm - x:.0f} mm behind the table edge")
    heights = {
        Zone.BOX: [lay.box.floor_z_mm + h for h in (15, 35, 55)],
        Zone.BACKGROUND: [25.0],
    }
    for zone, z in cfg.zones.items():
        xs = [p[0] for p in z.workspace_mm]
        ys = [p[1] for p in z.workspace_mm]
        gx = np.linspace(min(xs) + 1, max(xs) - 1, max(2, round((max(xs) - min(xs)) / step_mm) + 1))
        gy = np.linspace(min(ys) + 1, max(ys) - 1, max(2, round((max(ys) - min(ys)) / step_mm) + 1))
        look = q["look_box" if zone is Zone.BOX else "look_bg"]
        for x, y, h in itertools.product(gx, gy, heights[zone]):
            try:
                arm.plan_pick(ArmPoint(float(x), float(y), h), zone, q0=look)
            except SorterError as e:
                problems.append(f"pick {zone} ({x:.0f}, {y:.0f}, {h:.0f}): {e}")
    return problems


def _yaml(poses: dict[str, list[float]], zones: dict[Zone, ZoneConfig], views: dict) -> str:
    body = {
        "views": views,
        "poses": poses,
        "zones": {
            z.value: {
                "workspace_mm": [[round(x, 1), round(y, 1)] for x, y in zc.workspace_mm],
                "z_floor_mm": zc.z_floor_mm,
                "grasp_depth_mm": zc.grasp_depth_mm,
                "approach_mm": zc.approach_mm,
                "lift_z_mm": zc.lift_z_mm,
            }
            for z, zc in zones.items()
        },
    }
    head = (
        "# Fixed rig, committed (D-007). Computed for the table layout in `sim.layout` by\n"
        "# `python -m sorter.sim.layout --write`: poses (joint angles, rad) and zones (block 5),\n"
        "# views (pixel ROIs from the look poses, block 1). Re-teach / redraw them on the rig.\n"
    )
    return head + yaml.safe_dump(body, sort_keys=False, default_flow_style=None, width=100)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(
        prog="python -m sorter.sim.layout", description=__doc__.split("\n")[0]
    )
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    p.add_argument("--write", action="store_true", help="write poses and zones to rig.yaml")
    args = p.parse_args(argv)

    cfg = load_config(args.config_dir)
    poses = compute_poses(cfg.sim, cfg.arm)
    zones = compute_zones(cfg.sim, cfg.arm)
    rig = Path(args.config_dir) / "rig.yaml"
    views = zone_rois(cfg, poses)
    text = _yaml(poses, zones, views)
    print(text)
    cfg = cfg.model_copy(update={"poses": poses, "zones": zones})
    for name in ("look_box", "look_bg"):
        cam = (kin.fk_tcp(poses[name]) @ camera_mount(cfg.sim))[:3, 3]
        w = cfg.sim.width * cam[2] / cfg.sim.focal_px
        h = cfg.sim.height * cam[2] / cfg.sim.focal_px
        print(f"# {name}: camera at z {cam[2]:.0f} mm sees {w:.0f} x {h:.0f} mm of the table")
    problems = check(cfg)
    for msg in problems:
        print("PROBLEM", msg, file=sys.stderr)
    print(f"# {len(problems)} problems", file=sys.stderr)
    if args.write:
        rig.write_text(text)
        print(f"# wrote {rig}", file=sys.stderr)
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()

"""Poses, zones, views and the arm's keep-out for the rover layout (`sim.layout`), with the IK.

    uv run python -m sorter.sim.layout            # print them and check every pick and move
    uv run python -m sorter.sim.layout --write    # also write config/rig.yaml

The look poses point the wrist camera straight down over the zone center, as high as the arm
can hold the gripper vertical. The drop poses hold the TCP `arm.drop_height_mm` over the rim of
a cargo compartment / laundry bin. The check plans a full pick (above, down, up) on a grid over
each zone workspace and every move between the named poses, against the floor and the keep-out.
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
from sorter.arm.config import LOOK_POSES, POSE_NAMES, ArmConfig, ZoneConfig
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.errors import SorterError
from sorter.core.types import ArmPoint, ColorClass, Zone
from sorter.sim.config import RectConfig, SimConfig
from sorter.sim.rig import camera_mount, camera_pose

LOOK_TCP_Z_MM = (100, -120)  # the highest TCP height tried for a look pose, then lower by 5 mm
FLOOR_MARGIN_MM = 20.0
# the grasp stays this far from a compartment's walls: across the fingers, and along them (the
# open gripper is ~60 mm wide, + the keep-out margin)
CARGO_MARGIN_MM = (30.0, 45.0)
FLOOR_CLEARANCE_MM = 3.0  # arm.z_min_mm: this far above the floor
HOME_TCP_MM = (220.0, 0.0, 200.0)
HOME_APPROACH = (1.0, 0.0, -1.0)  # 45° down, forward
_SEED = (0.0, 1.2, 1.5, 0.0, 0.0, 0.0)  # elbow up


def _seed(x: float, y: float) -> np.ndarray:
    s = np.array(_SEED)
    s[0] = -math.atan2(y, x)  # joint 1 turns clockwise for a positive angle (URDF axis −z)
    return s


def rect_polygon(
    r: RectConfig, margin: float, margin_y: float | None = None
) -> list[tuple[float, float]]:
    """The corners of `r` shrunk by `margin` (along y by `margin_y`, if given)."""
    x0, x1, _, _ = r.bounds(-margin)
    _, _, y0, y1 = r.bounds(-(margin if margin_y is None else margin_y))
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


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
    T_link5_cam: np.ndarray,
    surface_z: float = 0.0,
    exact: bool = True,
) -> np.ndarray | None:
    """Joints with the gripper vertical, the TCP at `tcp_z`, and the camera's optical axis
    through `center` on the surface at `surface_z`. None if the arm can't. The camera is on
    link5 (D-027): joint 6 doesn't turn the image, so its turn is whatever the arm gives.
    `exact=False`: if the arm can't center the camera there (the TCP would have to go where it
    can't reach), the reachable pose whose camera axis comes closest to `center`."""
    center = np.asarray(center, dtype=float)
    target = np.array([*center, tcp_z], dtype=float)
    q, ok, best, misses = _seed(*center), None, None, 0
    # the camera sits off the TCP: shift the TCP until the camera is centered
    for _ in range(12):
        q_new = kin.solve(target, "down", q, arm.z_min_mm)
        if q_new is None:
            misses += 1  # a failed IK takes ~250 ms (restarts): back off twice at most
            if ok is None or misses > 2:
                break
            target = (target + ok) / 2  # out of reach: back off halfway
            continue
        q, ok = q_new, target.copy()
        hit = axis_hit(kin.fk_link5(q) @ T_link5_cam, surface_z)
        if hit is None:
            return None
        err = center - hit
        d = float(np.hypot(*err))
        if d < 0.5:
            return q
        if best is not None and d > best[0] - 0.5:  # no longer getting closer: the reach limit
            break
        if best is None or d < best[0]:
            best = (d, q)
        target[:2] += err
    return None if exact or best is None else best[1]


def look_pose(
    sim: SimConfig, arm: ArmConfig, center: tuple[float, float], surface_z: float
) -> np.ndarray:
    """Joints that put the camera straight down over `center`, as high as possible; if no height
    centers it exactly, the highest pose that comes closest."""
    T_link5_cam = camera_mount(sim)
    heights = range(LOOK_TCP_Z_MM[0], LOOK_TCP_Z_MM[1] - 1, -5)
    for exact in (True, False):
        for tcp_z in heights:
            q = camera_over(arm, center, tcp_z, T_link5_cam, surface_z, exact=exact)
            if q is not None:
                return q
    raise SystemExit(f"no look pose puts the camera down over {center}")


def zone_surfaces(sim: SimConfig) -> dict[Zone, tuple[RectConfig, float]]:
    """Each look zone's rectangle and its surface height (mm)."""
    lay = sim.layout
    return {
        Zone.FLOOR: (lay.floor_view, lay.floor_z_mm),
        Zone.CARGO: (lay.cargo, lay.cargo.floor_z_mm),
    }


def zone_rois(cfg: Config, poses: dict[str, list[float]]) -> dict[str, dict]:
    """`views.<zone>.roi`: the zone's rectangle projected into the image from its look pose,
    clipped to the image, minus the rows the gripper hides at the top of the frame."""
    from sorter.sim.physics.camera import PhysicsCamera
    from sorter.sim.physics.world import PhysicsWorld

    sim = cfg.sim.model_copy(update={"scenes": [], "realtime": 0})
    world = PhysicsWorld(sim, poses)
    camera = PhysicsCamera(world)
    out = {}
    try:
        for zone, (rect, z) in zone_surfaces(cfg.sim).items():
            q = np.asarray(poses[LOOK_POSES[zone]], dtype=float)
            world.teleport_arm(q, finger_m=0.0)
            frame = camera.fresh(5.0)
            no_depth = (frame.depth_mm == 0).mean(axis=1) > 0.3
            top = int(np.argmin(no_depth)) + 10 if no_depth[0] else 0  # rows hidden by the gripper
            T = np.linalg.inv(camera_pose(cfg.sim, q))
            pts = []
            for x, y in rect_polygon(rect, 0.0):
                c = T @ np.array([x, y, z, 1.0])
                u = sim.focal_px * c[0] / c[2] + sim.width / 2
                v = sim.focal_px * c[1] / c[2] + sim.height / 2
                pts.append(
                    (
                        int(np.clip(round(u), 0, sim.width - 1)),
                        int(np.clip(round(v), top, sim.height - 1)),
                    )
                )
            out[zone.value] = {"roi": [list(p) for p in pts]}
    finally:
        camera.close()
    return out


def keep_out(sim: SimConfig, margin_mm: float) -> list[list[float]]:
    """`arm.keep_out_mm`: the rover body up to the deck top (the margin grows it to z = 0, so a
    grasp may reach the cargo floor), and the cargo box's walls and dividers."""
    lay = sim.layout
    x0, x1, y0, y1 = lay.body.bounds()
    boxes = [[x0, x1, y0, y1, lay.floor_z_mm - 100.0, -margin_mm]]
    cargo = lay.cargo
    t = cargo.wall_t_mm
    cx0, cx1, cy0, cy1 = cargo.bounds()
    rim = cargo.rim_z_mm
    boxes += [
        [cx0 - t, cx0, cy0 - t, cy1 + t, 0.0, rim],
        [cx1, cx1 + t, cy0 - t, cy1 + t, 0.0, rim],
        [cx0, cx1, cy0 - t, cy0, 0.0, rim],
        [cx0, cx1, cy1, cy1 + t, 0.0, rim],
    ]
    rects = [cargo.compartment(c) for c in cargo.compartments]
    for a, b in zip(rects, rects[1:], strict=False):
        boxes.append([a.bounds()[1], b.bounds()[0], cy0, cy1, 0.0, rim])
    return [[round(v, 1) for v in b] for b in boxes]


def arm_for(sim: SimConfig, arm: ArmConfig) -> ArmConfig:
    """`arm` with the floor clearance and keep-out of the layout."""
    return arm.model_copy(
        update={
            "z_min_mm": sim.layout.floor_z_mm + FLOOR_CLEARANCE_MM,
            "keep_out_mm": [tuple(b) for b in keep_out(sim, arm.keep_out_margin_mm)],
        }
    )


def _drop_pose(arm: ArmConfig, xyz: tuple[float, float, float], name: str) -> np.ndarray:
    """Joints with the TCP at `xyz`: the gripper down if it can, else tilted outwards, else any
    way it reaches."""
    x, y, _ = xyz
    r = math.hypot(x, y)
    for approach in ("down", (x / r, y / r, -1.0), None):
        q = kin.solve(xyz, approach, _seed(x, y), arm.z_min_mm)
        if q is not None and kin.keep_out_hit(q, arm.keep_out_mm, arm.keep_out_margin_mm) is None:
            return q
    raise SystemExit(f"{name} ({x:.0f}, {y:.0f}, {xyz[2]:.0f}) is not reachable")


def compute_poses(sim: SimConfig, arm: ArmConfig) -> dict[str, list[float]]:
    lay = sim.layout
    poses: dict[str, np.ndarray] = {"rest": np.zeros(kin.N_JOINTS)}
    home = kin.solve(HOME_TCP_MM, HOME_APPROACH, _seed(*HOME_TCP_MM[:2]), arm.z_min_mm)
    if home is None:
        raise SystemExit(f"home {HOME_TCP_MM} is not reachable")
    poses["home"] = home
    for zone, (rect, z) in zone_surfaces(sim).items():
        poses[LOOK_POSES[zone]] = look_pose(sim, arm, rect.center_mm, z)
    cargo = lay.cargo
    for color in ColorClass:
        x, y = cargo.compartment(color).center_mm
        z = cargo.rim_z_mm + arm.drop_height_mm
        poses[f"cargo_{color.value}"] = _drop_pose(arm, (x, y, z), f"cargo_{color.value}")
    bins = lay.laundry
    for color in ColorClass:
        x, y = bins.centers_mm[color]
        z = lay.floor_z_mm + bins.height_mm + arm.drop_height_mm
        poses[f"laundry_{color.value}"] = _drop_pose(arm, (x, y, z), f"laundry_{color.value}")
    return {name: [round(float(v), 4) for v in poses[name]] for name in POSE_NAMES}


def compute_zones(sim: SimConfig, arm: ArmConfig) -> dict[Zone, ZoneConfig]:
    lay = sim.layout
    cargo = lay.cargo
    return {
        Zone.FLOOR: ZoneConfig(
            workspace_mm=rect_polygon(lay.floor_view, FLOOR_MARGIN_MM),
            z_floor_mm=lay.floor_z_mm + 8.0,  # the fingertips stop just above the floor
            lift_z_mm=lay.floor_z_mm + 150.0,
        ),
        Zone.CARGO: ZoneConfig(
            workspace_mm=rect_polygon(cargo, CARGO_MARGIN_MM[0]),
            z_floor_mm=cargo.floor_z_mm + 5.0,
            approach_mm=60.0,  # the move to above the grasp stays over the rim
            lift_z_mm=cargo.rim_z_mm + 30.0,
        ),
    }


class _PlanOnly:
    """An ArmDriver stand-in for planning: the arm at rest, nothing moves."""

    connected = True

    def joints(self) -> np.ndarray:
        return np.zeros(kin.N_JOINTS)


def check(cfg: Config, step_mm: float = 20.0) -> list[str]:
    """Problems with `cfg.poses` / `cfg.zones` / `cfg.arm.keep_out_mm` on the layout; [] = every
    pick and move plans."""
    from sorter.arm.controller import Controller

    problems = []
    arm = Controller(_PlanOnly(), cfg.arm, cfg.poses, cfg.zones)  # type: ignore[arg-type]
    q = {n: np.asarray(v) for n, v in cfg.poses.items()}
    moves = [("rest", "home"), *((p, "home") for p in POSE_NAMES if p not in ("rest", "home"))]
    moves += [("look_floor", "look_cargo")]
    for a, b in moves:
        try:
            arm._plan_joints(q[a], q[b])
            arm._plan_joints(q[b], q[a])
        except SorterError as e:
            problems.append(f"move {a} ↔ {b}: {e}")
    lay = cfg.sim.layout
    cargo = lay.cargo
    # where picks must work: the floor zone with any yaw; each compartment (less the margin),
    # the fingers opening along the compartment's long side
    along_y = cargo.size_mm[1] >= cargo.compartment(cargo.compartments[0]).size_mm[0]
    across, along = CARGO_MARGIN_MM
    regions = [(Zone.FLOOR, cfg.zones[Zone.FLOOR].workspace_mm, [lay.floor_z_mm + 15.0], None)]
    regions += [
        (
            Zone.CARGO,
            rect_polygon(cargo.compartment(c), *((across, along) if along_y else (along, across))),
            [cargo.floor_z_mm + h for h in (15, 35)],
            math.pi / 2 if along_y else 0.0,
        )
        for c in cargo.compartments
    ]
    for zone, poly, heights, yaw in regions:
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        gx = np.linspace(min(xs) + 1, max(xs) - 1, max(2, round((max(xs) - min(xs)) / step_mm) + 1))
        gy = np.linspace(min(ys) + 1, max(ys) - 1, max(2, round((max(ys) - min(ys)) / step_mm) + 1))
        look = q[LOOK_POSES[zone]]
        for x, y, h in itertools.product(gx, gy, heights):
            try:
                _, _, up = arm.plan_pick(ArmPoint(float(x), float(y), h), zone, yaw, q0=look)
                arm._plan_joints(up[-1], q["home"])  # and on to a drop, via home
            except SorterError as e:
                problems.append(f"pick {zone} ({x:.0f}, {y:.0f}, {h:.0f}): {e}")
    return problems


def _yaml(poses, zones: dict[Zone, ZoneConfig], views: dict, arm: ArmConfig) -> str:
    body = {
        "arm": {
            "z_min_mm": arm.z_min_mm,
            "keep_out_mm": [list(b) for b in arm.keep_out_mm],
        },
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
        "# Fixed rig, committed (D-007). Computed for the rover layout in `sim.layout` by\n"
        "# `python -m sorter.sim.layout --write`: the arm's floor clearance and keep-out, poses\n"
        "# (joint angles, rad) and zones, views (pixel ROIs from the look poses).\n"
        "# Re-teach / redraw them on the rig.\n"
    )
    return head + yaml.safe_dump(body, sort_keys=False, default_flow_style=None, width=100)


def main(argv: list[str] | None = None) -> None:
    p = argparse.ArgumentParser(
        prog="python -m sorter.sim.layout", description=__doc__.split("\n")[0]
    )
    p.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    p.add_argument("--write", action="store_true", help="write the results to rig.yaml")
    args = p.parse_args(argv)

    cfg = load_config(args.config_dir)
    arm = arm_for(cfg.sim, cfg.arm)
    poses = compute_poses(cfg.sim, arm)
    zones = compute_zones(cfg.sim, arm)
    rig = Path(args.config_dir) / "rig.yaml"
    views = zone_rois(cfg, poses)
    text = _yaml(poses, zones, views, arm)
    print(text)
    cfg = cfg.model_copy(update={"poses": poses, "zones": zones, "arm": arm})
    for zone, (_, z) in zone_surfaces(cfg.sim).items():
        name = LOOK_POSES[zone]
        cam = camera_pose(cfg.sim, poses[name])
        h = cam[2, 3] - z
        hit = axis_hit(cam, z)
        at = f"({hit[0]:.0f}, {hit[1]:.0f})" if hit is not None else "?"
        w, hh = (n * h / cfg.sim.focal_px for n in (cfg.sim.width, cfg.sim.height))
        print(
            f"# {name}: camera {h:.0f} mm above the {zone}, axis at {at}, "
            f"sees {w:.0f} x {hh:.0f} mm"
        )
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

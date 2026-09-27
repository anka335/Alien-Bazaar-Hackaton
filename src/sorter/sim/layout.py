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
from sorter.arm.config import LOOK_POSES, POSE_NAMES, SCAN_POSES, ArmConfig, ZoneConfig
from sorter.core.config import DEFAULT_CONFIG_DIR, Config, load_config
from sorter.core.errors import SorterError
from sorter.core.types import ArmPoint, ColorClass, Zone
from sorter.sim.config import RAIL_INSET_MM, RectConfig, SimConfig
from sorter.sim.rig import camera_body_points, camera_mount, camera_pose

LOOK_TCP_Z_MM = (100, -120)  # camera_over: the highest TCP height tried, then lower by 5 mm
LOOK_HEIGHTS_MM = tuple(range(400, 174, -10))  # the camera above what it looks at, tried in turn
REACH_EXTENT_MM = (-360.0, 540.0, -540.0, 540.0)  # x0, x1, y0, y1 of the floor reach map
REACH_STEP_MM = 30.0
REACH_CHECK_MM = 20.0  # the finer grid the floor zone is checked on (as `check` does)
REACH_RING_MM = (120.0, 560.0)  # the arm's floor reach lies within this ring
SCAN_VIEW_MM = 270.0  # the floor a scan view covers across (the camera ~350 mm up)
FLOOR_MARGIN_MM = 20.0
# the grasp stays this far from a compartment's walls: across the fingers, and along them (the
# open gripper is ~60 mm wide, + the keep-out margin)
CARGO_MARGIN_MM = (30.0, 50.0)
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
    sim: SimConfig,
    arm: ArmConfig,
    center: Sequence[float],
    surface_z: float,
    seed: Sequence[float] | None = None,
    home: Sequence[float] | None = None,
) -> np.ndarray:
    """Joints that point the camera at `center` on the surface at `surface_z` from as high as
    the arm safely reaches (up to LOOK_HEIGHTS_MM[0] above it), leaning up to 30° off vertical,
    and (with `home`) that a straight joint move from `home` reaches. The camera is on link5
    (D-027): joint 6 doesn't turn the image."""
    x, y = center[0], center[1]

    def from_home(q: np.ndarray) -> bool:
        from sorter.arm.controller import Controller

        poses = {n: home for n in POSE_NAMES}
        zones = compute_zones(sim, arm)
        ctl = Controller(_PlanOnly(), arm, poses, zones)  # type: ignore[arg-type]
        try:
            ctl.plan_move(home, q)
            ctl.plan_move(q, home)
        except SorterError:
            return False
        return True

    found = kin.camera_look(
        camera_mount(sim),
        (x, y, surface_z),
        _seed(x, y) if seed is None else seed,
        heights_mm=LOOK_HEIGHTS_MM,
        z_min_mm=arm.z_min_mm,
        keep_out=arm.keep_out_mm,
        keep_out_margin_mm=arm.keep_out_margin_mm,
        link5_points=arm.link5_points_mm,
        accept=None if home is None else from_home,
    )
    if found is None:
        raise SystemExit(f"no look pose points the camera at {tuple(center)}")
    return found[0]


def floor_workspace(
    sim: SimConfig, arm: ArmConfig, zones: dict[Zone, ZoneConfig]
) -> list[tuple[float, float]]:
    """The floor pick zone: the region where a pick plans with the fingers at any yaw and goes
    on to a drop via home. A map of reachable cells (REACH_STEP_MM), its largest region one
    cell in as a polygon, then points on a finer grid inside it that fail take their cell out
    until none fails. Cached in data/cache by the layout and the arm's limits (it takes
    minutes)."""
    import hashlib
    import json

    from sorter.calibration.board import CACHE

    base = [
        sim.layout.model_dump(mode="json"),
        arm.model_dump(mode="json"),
        zones[Zone.FLOOR].model_dump(mode="json"),
        REACH_EXTENT_MM,
        REACH_STEP_MM,
        REACH_RING_MM,
        2,  # the arm's planning: bump it when it changes
    ]

    def cached(name: str, extra: list) -> Path:
        key = json.dumps([*base, *extra], sort_keys=True)
        return CACHE / f"{name}_{hashlib.sha1(key.encode()).hexdigest()[:12]}"

    path = cached("floor_workspace", [REACH_CHECK_MM, 8]).with_suffix(".json")
    if path.is_file():
        return [tuple(p) for p in json.loads(path.read_text())]
    x0, x1, y0, y1 = REACH_EXTENT_MM
    xs = np.arange(x0, x1 + 1, REACH_STEP_MM)
    ys = np.arange(y0, y1 + 1, REACH_STEP_MM)
    reach_path = cached("floor_reach", []).with_suffix(".npy")
    if reach_path.is_file():
        ok = np.load(reach_path)
    else:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor() as pool:  # minutes on one core
            args = zip(*((sim, arm, zones, y, xs) for y in ys), strict=True)
            ok = np.array(list(pool.map(_reach_row, *args)), dtype=bool)
        reach_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(reach_path, ok)
    cx, cy = np.meshgrid(xs, ys)  # the cells' centers
    pick = _floor_pick(sim, arm, zones)
    tried: dict[tuple[float, float], bool] = {}  # the grid is fixed: each point once

    def fine(x: float, y: float) -> bool:
        if (x, y) not in tried:
            tried[x, y] = pick(x, y, (None, 0.0, math.pi / 2))
        return tried[x, y]

    for _ in range(30):
        poly = reach_polygon(ok)
        failed = [(x, y) for x, y in grid(poly, REACH_CHECK_MM) if not fine(x, y)]
        if not failed:
            break
        # the reachable cells near a failed point go: the polygon's edge moves in past it
        for x, y in failed:
            ok[np.hypot(cx - x, cy - y) <= 1.5 * REACH_STEP_MM] = False
    else:
        raise SystemExit("the floor zone keeps failing picks")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(poly))
    return poly


def _reach_row(
    sim: SimConfig, arm: ArmConfig, zones: dict[Zone, ZoneConfig], y: float, xs: np.ndarray
) -> list[bool]:
    """One row of the reach map: a pick at (x, y) plans with the fingers at any of 4 yaws."""
    pick = _floor_pick(sim, arm, zones)
    yaws = (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4)
    return [
        REACH_RING_MM[0] <= math.hypot(x, y) <= REACH_RING_MM[1] and pick(x, y, yaws) for x in xs
    ]


def _floor_pick(sim: SimConfig, arm: ArmConfig, zones: dict[Zone, ZoneConfig]):
    """f(x, y, yaws): a pick at (x, y) on the floor plans at each of `yaws` and goes on home."""
    from sorter.arm.controller import Controller

    x0, x1, y0, y1 = REACH_EXTENT_MM
    big = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    zones = {**zones, Zone.FLOOR: zones[Zone.FLOOR].model_copy(update={"workspace_mm": big})}
    home = kin.solve(HOME_TCP_MM, HOME_APPROACH, _seed(*HOME_TCP_MM[:2]), arm.z_min_mm)
    poses = {n: list(home) for n in POSE_NAMES}
    ctl = Controller(_PlanOnly(), arm, poses, zones)  # type: ignore[arg-type]
    z = sim.layout.floor_z_mm + 15.0

    def pick(x: float, y: float, yaws: Sequence[float]) -> bool:
        try:
            for yaw in yaws:
                _, _, up = ctl.plan_pick(ArmPoint(float(x), float(y), z), Zone.FLOOR, yaw, q0=home)
                ctl._plan_joints(up[-1], home)
        except SorterError:
            return False
        return True

    return pick


def grid(poly: Sequence[tuple[float, float]], step_mm: float) -> list[tuple[float, float]]:
    """The points of a fixed grid (`step_mm` apart, offset half a step from 0) inside `poly`."""
    from sorter.arm.controller import in_polygon

    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    h = step_mm / 2
    gx = np.arange(math.floor((min(xs) - h) / step_mm), math.ceil((max(xs) - h) / step_mm) + 1)
    gy = np.arange(math.floor((min(ys) - h) / step_mm), math.ceil((max(ys) - h) / step_mm) + 1)
    pts = ((float(i * step_mm + h), float(j * step_mm + h)) for i, j in itertools.product(gx, gy))
    return [(x, y) for x, y in pts if in_polygon(x, y, poly)]


def reach_polygon(ok: np.ndarray) -> list[tuple[float, float]]:
    """The largest region of reachable cells as a polygon through the centers of its edge cells
    (mm): half a cell in from where the reach ends."""
    import cv2

    x0, _, y0, _ = REACH_EXTENT_MM
    # an isolated cell or a one-cell spur is noise, not reach
    m = cv2.morphologyEx(ok.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise SystemExit("the arm reaches no floor at all")
    c = max(contours, key=cv2.contourArea)[:, 0, :]
    return [(float(x0 + j * REACH_STEP_MM), float(y0 + i * REACH_STEP_MM)) for j, i in c]


def scan_targets(workspace: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Where the scan poses point the camera: evenly along the ring of the floor workspace,
    from its right end to its left, halfway out."""
    pts = np.asarray(workspace, dtype=float)
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    r = np.hypot(pts[:, 0], pts[:, 1])
    a0, a1 = ang.min(), ang.max()
    rm = (np.percentile(r, 10) + np.percentile(r, 90)) / 2
    n = len(SCAN_POSES)
    # the end views sit half a view in from the ends of the ring
    half = SCAN_VIEW_MM / 2 / rm
    a = np.linspace(a0 + half, a1 - half, n) if a1 - a0 > 2 * half else np.full(n, (a0 + a1) / 2)
    return [(float(rm * math.cos(t)), float(rm * math.sin(t))) for t in a]


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
    """`arm.keep_out_mm`: the rover's middle (as wide as the deck, as long as the rails) up to
    the deck top (the margin grows it to z = 0), the wheels (not the one under the cargo box:
    the box's floor covers it), the equipment behind the arm, and the cargo box's walls and
    dividers."""
    lay = sim.layout
    fz = lay.floor_z_mm
    x0, x1, y0, y1 = lay.body.bounds()
    _, _, dy0, dy1 = lay.deck.bounds()
    boxes = [[x0 + RAIL_INSET_MM, x1 - RAIL_INSET_MM, dy0, dy1, fz - 100.0, -margin_mm]]
    cargo = lay.cargo
    r, w = lay.wheel_radius_mm, lay.wheel_width_mm
    for wx, wy in itertools.product((x0 + r, x1 - r), (y0 + w / 2, y1 - w / 2)):
        wheel = RectConfig(center_mm=(wx, wy), size_mm=(2 * r, w))
        if not _overlap(wheel, cargo, cargo.wall_t_mm):
            boxes.append([*wheel.bounds(), fz - 100.0, fz + 2 * r])
    boxes += [list(b.box_mm) for b in lay.equipment.values()]
    t = cargo.wall_t_mm
    cx0, cx1, cy0, cy1 = cargo.bounds()
    base, rim = cargo.base_z_mm, cargo.rim_z_mm
    boxes += [
        [cx0 - t, cx0, cy0 - t, cy1 + t, base, rim],
        [cx1, cx1 + t, cy0 - t, cy1 + t, base, rim],
        [cx0, cx1, cy0 - t, cy0, base, rim],
        [cx0, cx1, cy1, cy1 + t, base, rim],
    ]
    rects = cargo.insides()
    for a, b in zip(rects, rects[1:], strict=False):
        boxes.append([a.bounds()[1], b.bounds()[0], cy0, cy1, base, rim])
    return [[round(v, 1) for v in b] for b in boxes]


def _overlap(a: RectConfig, b: RectConfig, grow_b: float = 0.0) -> bool:
    ax0, ax1, ay0, ay1 = a.bounds()
    bx0, bx1, by0, by1 = b.bounds(grow_b)
    return ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1


def arm_for(sim: SimConfig, arm: ArmConfig) -> ArmConfig:
    """`arm` with the floor clearance and keep-out of the layout, and the camera body's points
    for the collision checks."""
    return arm.model_copy(
        update={
            "z_min_mm": sim.layout.floor_z_mm + FLOOR_CLEARANCE_MM,
            "keep_out_mm": [tuple(b) for b in keep_out(sim, arm.keep_out_margin_mm)],
            "link5_points_mm": camera_body_points(sim),
        }
    )


def _drop_pose(arm: ArmConfig, xyz: tuple[float, float, float], name: str) -> np.ndarray:
    """Joints with the TCP at `xyz`: the gripper down if it can, else tilted outwards, else any
    way it reaches."""
    x, y, _ = xyz
    r = math.hypot(x, y)
    for approach in ("down", (x / r, y / r, -1.0), None):
        q = kin.solve(xyz, approach, _seed(x, y), arm.z_min_mm)
        if q is None:
            continue
        pts = arm.link5_points_mm
        if kin.keep_out_hit(q, arm.keep_out_mm, arm.keep_out_margin_mm, pts, arm.z_min_mm) is None:
            return q
    raise SystemExit(f"{name} ({x:.0f}, {y:.0f}, {xyz[2]:.0f}) is not reachable")


def compute_poses(
    sim: SimConfig, arm: ArmConfig, floor_workspace: Sequence[tuple[float, float]]
) -> dict[str, list[float]]:
    lay = sim.layout
    poses: dict[str, np.ndarray] = {"rest": np.zeros(kin.N_JOINTS)}
    home = kin.solve(HOME_TCP_MM, HOME_APPROACH, _seed(*HOME_TCP_MM[:2]), arm.z_min_mm)
    if home is None:
        raise SystemExit(f"home {HOME_TCP_MM} is not reachable")
    poses["home"] = home
    for zone, (rect, z) in zone_surfaces(sim).items():
        poses[LOOK_POSES[zone]] = look_pose(sim, arm, rect.center_mm, z, home=home)
    # from the middle of the ring outwards, each seeded by its neighbour: the IK stays on one
    # branch (a fresh seed at the ends finds the arm upright, its tool by the base column)
    targets = scan_targets(floor_workspace)
    mid = len(targets) // 2
    poses[SCAN_POSES[mid]] = look_pose(
        sim, arm, targets[mid], lay.floor_z_mm, poses[LOOK_POSES[Zone.FLOOR]], home=home
    )
    for side in (range(mid - 1, -1, -1), range(mid + 1, len(targets))):
        seed = poses[SCAN_POSES[mid]]
        for k in side:
            seed = poses[SCAN_POSES[k]] = look_pose(
                sim, arm, targets[k], lay.floor_z_mm, seed, home=home
            )
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


def compute_zones(
    sim: SimConfig, arm: ArmConfig, floor_workspace: Sequence[tuple[float, float]] | None = None
) -> dict[Zone, ZoneConfig]:
    """The pick zones. The floor: `floor_workspace` (from `reach_polygon`), else the floor
    view."""
    lay = sim.layout
    cargo = lay.cargo
    if floor_workspace is None:
        floor_workspace = rect_polygon(lay.floor_view, FLOOR_MARGIN_MM)
    return {
        Zone.FLOOR: ZoneConfig(
            workspace_mm=list(floor_workspace),
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
    moves += list(zip(SCAN_POSES, SCAN_POSES[1:], strict=False))  # the scan sweeps along the ring
    for a, b in moves:
        try:
            arm.plan_move(q[a], q[b])
            arm.plan_move(q[b], q[a])
        except SorterError as e:
            problems.append(f"move {a} ↔ {b}: {e}")
    lay = cfg.sim.layout
    cargo = lay.cargo
    # where picks must work: the floor zone with any yaw; the box or each compartment (less the
    # margin), the fingers opening along its long side
    along_y = cargo.size_mm[1] >= cargo.insides()[0].size_mm[0]
    across, along = CARGO_MARGIN_MM
    regions = [(Zone.FLOOR, cfg.zones[Zone.FLOOR].workspace_mm, [lay.floor_z_mm + 15.0], None)]
    regions += [
        (
            Zone.CARGO,
            rect_polygon(r, *((across, along) if along_y else (along, across))),
            [cargo.floor_z_mm + h for h in (15, 35)],
            math.pi / 2 if along_y else 0.0,
        )
        for r in cargo.insides()
    ]
    for zone, poly, heights, yaw in regions:
        look = q[LOOK_POSES[zone]]
        for (x, y), h in itertools.product(grid(poly, step_mm), heights):
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
            "link5_points_mm": [list(p) for p in arm.link5_points_mm],
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
    print("# mapping where the arm reaches the floor …", file=sys.stderr, flush=True)
    workspace = floor_workspace(cfg.sim, arm, compute_zones(cfg.sim, arm))
    poses = compute_poses(cfg.sim, arm, workspace)
    zones = compute_zones(cfg.sim, arm, workspace)
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

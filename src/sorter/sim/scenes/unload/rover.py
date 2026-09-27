"""The rover as it is (stage B's working geometry until it becomes the committed layout).

    uv run python -m sorter.sim.scenes.unload.rover     # compute the rig and check every pick

`REAL_ROVER` overrides `sim.layout` (estimates from photos, to be measured): one undivided cargo
box 150 × 150 × 60 mm to the arm's left and a bit behind it, three boxes of the same size on the
floor in front of the rover. `rover_config()` is the committed config on that geometry with its
rig (poses, zones, the arm's keep-out) computed like `python -m sorter.sim.layout` does, cached
in `data/unload_rig/`. The committed `rig.yaml` is not touched: the cargo box is shared with
stage A, so the change needs their agreement.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from sorter.arm import kinematics as kin
from sorter.arm.config import LOOK_POSES, POSE_NAMES, ArmConfig
from sorter.box_detector import held
from sorter.box_detector.held import SHOW_POSE, show_pose
from sorter.core.config import Config, deep_merge, load_config
from sorter.core.errors import SorterError
from sorter.core.types import ArmPoint, ColorClass, Zone
from sorter.sim import layout as lay_tool
from sorter.sim.config import SimConfig
from sorter.sim.rig import camera_mount
from sorter.sim.scenes.unload.scene import ELECTRONICS_MM

REAL_ROVER: dict[str, Any] = {
    "floor_z_mm": -160,  # deck top above the floor (wheels ~115 mm)
    "body": {"center_mm": [-110, 0], "size_mm": [380, 300]},  # chassis + wheels
    "deck": {"center_mm": [-110, 0], "size_mm": [340, 260]},
    "cargo": {  # cardboard, 150 × 150 × 60 outside, on a bracket off the deck's left side
        "center_mm": [-130, 200],
        "size_mm": [140, 140],
        "floor_z_mm": 4,
        "wall_mm": 56,
        "wall_t_mm": 5,
        "compartments": [],
    },
    # the unload station in front of the rover (the load flow doesn't run on this geometry)
    "floor_view": {"center_mm": [270, 0], "size_mm": [200, 500]},
    "laundry": {  # the same boxes, in a row across the front of the rover
        "centers_mm": {"light": [270, 170], "dark": [270, 0], "colored": [270, -170]},
        "size_mm": 150,
        "height_mm": 60,
        "wall_t_mm": 5,
    },
}
RIG_CACHE = Path("data/unload_rig")
GRIPPER_OPEN = 0.6
CARGO_MARGIN_MM = 16.0  # the cargo workspace this far from the walls
CORNER_CUT_MM = 30.0  # off each corner of the cargo workspace


def chamfer(square: list[tuple[float, float]], cut: float) -> list[tuple[float, float]]:
    """The axis-aligned rectangle `square` (4 corners) with its corners cut by `cut` mm."""
    xs, ys = [p[0] for p in square], [p[1] for p in square]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    return [
        (x0 + cut, y0), (x1 - cut, y0), (x1, y0 + cut), (x1, y1 - cut),
        (x1 - cut, y1), (x0 + cut, y1), (x0, y1 - cut), (x0, y0 + cut),
    ]  # fmt: skip


def rover_overrides(overrides: dict[str, Any] | None = None) -> dict[str, Any]:
    """Config overrides for the real rover's geometry, with `overrides` on top."""
    base = {
        "sim": {"layout": REAL_ROVER, "scenes": ["unload"], "unload": {"rover_parts": True}},
        # the box is small: the fingers open to ±30 mm, not ±50, around a sock ~55 mm wide
        "arm": {"gripper": {"open": GRIPPER_OPEN}},
    }
    return deep_merge(base, overrides or {})


def arm_for(sim: SimConfig, arm: ArmConfig) -> ArmConfig:
    """`arm` with the floor clearance and keep-out of the rover, the electronics included."""
    arm = lay_tool.arm_for(sim, arm)
    return arm.model_copy(update={"keep_out_mm": [*arm.keep_out_mm, ELECTRONICS_MM]})


def compute_rig(cfg: Config) -> dict[str, Any]:
    """`arm` (floor clearance, keep-out), `poses` and `zones` for `cfg.sim.layout`, as YAML data."""
    sim = cfg.sim
    lay = sim.layout
    arm = arm_for(sim, cfg.arm)
    poses: dict[str, np.ndarray] = {"rest": np.zeros(kin.N_JOINTS)}
    home = kin.solve(
        lay_tool.HOME_TCP_MM,
        lay_tool.HOME_APPROACH,
        lay_tool._seed(*lay_tool.HOME_TCP_MM[:2]),
        arm.z_min_mm,
    )
    if home is None:
        raise SorterError(f"home {lay_tool.HOME_TCP_MM} is not reachable")
    poses["home"] = home
    for zone, (rect, z) in lay_tool.zone_surfaces(sim).items():
        poses[LOOK_POSES[zone]] = lay_tool.look_pose(sim, arm, rect.center_mm, z)
    cargo = lay.cargo
    for color in ColorClass:  # one box: every color drops into its middle
        x, y = cargo.center_mm
        name = f"cargo_{color.value}"
        poses[name] = lay_tool._drop_pose(arm, (x, y, cargo.rim_z_mm + arm.drop_height_mm), name)
    bins = lay.laundry
    for color in ColorClass:
        x, y = bins.centers_mm[color]
        z = lay.floor_z_mm + bins.height_mm + arm.drop_height_mm
        poses[f"laundry_{color.value}"] = lay_tool._drop_pose(arm, (x, y, z), f"laundry_{color}")
    show = show_pose(
        home,
        camera_mount(sim),
        arm.keep_out_mm,
        arm.keep_out_margin_mm,
        arm.z_min_mm,
        lay.body.bounds()[1],
        lay.floor_z_mm,
        sim.focal_px,
        (sim.width, sim.height),
        collides=_collision_check(sim),
    )
    if show is None:
        raise SorterError("no pose shows the camera what the gripper holds")
    zones = lay_tool.compute_zones(sim, arm)
    # the box is small: a sock against a wall is still graspable with the fingers along the
    # wall, so the workspace comes close to the walls (the pick's keep-out check says which
    # yaw fits); at a corner both walls are close, and none does
    square = lay_tool.rect_polygon(cargo, CARGO_MARGIN_MM)
    zones[Zone.CARGO].workspace_mm = chamfer(square, CORNER_CUT_MM)
    return {
        "arm": {"z_min_mm": arm.z_min_mm, "keep_out_mm": [list(b) for b in arm.keep_out_mm]},
        "poses": {
            **{n: [round(float(v), 4) for v in poses[n]] for n in POSE_NAMES},
            SHOW_POSE: [round(float(v), 4) for v in show],
        },
        "zones": {z.value: zc.model_dump(mode="json") for z, zc in zones.items()},
    }


def _collision_check(sim: SimConfig, clearance_mm: float = 10.0):
    """`collides(q)`: the arm at `q` comes within `clearance_mm` of itself or the scene (the
    MuJoCo model: no socks, the station where the layout puts it), past the contacts the arm
    always has."""
    import mujoco

    from sorter.sim.physics.model import ARM_JOINTS, build

    nominal = {"cargo": {}, "station_mm": 0.0, "station_deg": 0.0, "bin_mm": 0.0, "bin_deg": 0.0}
    empty = sim.model_copy(update={"unload": sim.unload.model_copy(update=nominal)})
    m = mujoco.MjModel.from_xml_string(build(empty).xml)
    m.geom_margin[:] = clearance_mm / 1000
    d = mujoco.MjData(m)
    adr = [m.jnt_qposadr[m.joint(j).id] for j in ARM_JOINTS]
    arm = {m.body(n).id for n in ("link2", "link3", "link4", "link5", "link6", "gripper_end")}

    def contacts(q) -> set[tuple[int, int]]:
        d.qpos[adr] = q
        mujoco.mj_forward(m, d)
        out = set()
        for c in d.contact[: d.ncon]:
            b = (int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2]))
            if b[0] in arm or b[1] in arm:
                out.add((min(c.geom1, c.geom2), max(c.geom1, c.geom2)))
        return out

    always = contacts(np.zeros(len(ARM_JOINTS)))  # e.g. the fingers' pads on each other

    def collides(q) -> bool:
        return bool(contacts(q) - always)

    return collides


def _key(cfg: Config) -> str:
    src = json.dumps(
        [cfg.sim.layout.model_dump(mode="json"), cfg.arm.model_dump(mode="json")],
        sort_keys=True,
    )
    src += repr(np.round(camera_mount(cfg.sim), 6).tolist()) + Path(__file__).read_text()
    src += Path(held.__file__).read_text()  # the show pose's search
    return hashlib.sha1(src.encode()).hexdigest()[:12]


def rover_config(overrides: dict[str, Any] | None = None, cache: Path | None = RIG_CACHE) -> Config:
    """The committed config on the real rover's geometry, with `overrides`, and its rig. The
    sim's camera sits where the rig's hand-eye calibration (`config/hand_eye.yaml`) found the
    real one, if there is one."""
    cfg = load_config(overrides=rover_overrides(overrides))
    measured = cfg.calibration.hand_eye
    if measured is not None and cfg.sim.camera_mount_T is None:
        mount = np.asarray(measured.T_link5_cam, dtype=float).round(6).tolist()
        cfg = load_config(
            overrides=rover_overrides(
                deep_merge(overrides or {}, {"sim": {"camera_mount_T": mount}})
            )
        )
    path = cache / f"{_key(cfg)}.yaml" if cache is not None else None
    if path is not None and path.is_file():
        rig = yaml.safe_load(path.read_text())
    else:
        rig = compute_rig(cfg)
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(yaml.safe_dump(rig, sort_keys=False, default_flow_style=None))
    data = cfg.model_dump(mode="json")
    data["arm"].update(rig["arm"])
    data["poses"] = rig["poses"]
    data["zones"] = rig["zones"]
    data["views"] = {}  # the loop projects the box itself
    return Config.model_validate(data)


def check(cfg: Config, step_mm: float = 20.0) -> list[str]:
    """Problems with the rig: every move between the named poses, and a pick on a grid over the
    cargo workspace (with some finger yaw, in 45° steps: near a wall only some fit), then on
    to home."""
    from sorter.arm.controller import Controller, in_polygon

    arm = Controller(lay_tool._PlanOnly(), cfg.arm, cfg.poses, cfg.zones)  # type: ignore[arg-type]
    q = {n: np.asarray(v) for n, v in cfg.poses.items()}
    problems = []
    for p in (*POSE_NAMES, SHOW_POSE):
        if p == "home":
            continue
        try:
            arm._plan_joints(q[p], q["home"])
            arm._plan_joints(q["home"], q[p])
        except SorterError as e:
            problems.append(f"move {p} ↔ home: {e}")
    poly = cfg.zones[Zone.CARGO].workspace_mm
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    grid = [
        np.linspace(min(v) + 1, max(v) - 1, max(2, round((max(v) - min(v)) / step_mm) + 1))
        for v in (xs, ys)
    ]
    floor = cfg.sim.layout.cargo.floor_z_mm
    for x, y, h in itertools.product(*grid, (15, 35)):
        if not in_polygon(x, y, poly):
            continue
        errors = []
        for yaw in np.radians([0, 90, 45, 135]):
            try:
                p = ArmPoint(float(x), float(y), floor + h)
                _, _, up = arm.plan_pick(p, Zone.CARGO, float(yaw), q0=q["look_cargo"])
                arm._plan_joints(up[-1], q["home"])
                break
            except SorterError as e:
                errors.append(str(e))
        else:
            problems.append(f"pick ({x:.0f}, {y:.0f}, {floor + h:.0f}), no yaw: {errors[0]}")
    return problems


def main() -> None:
    cfg = rover_config()
    print(yaml.safe_dump({"poses": cfg.poses}, default_flow_style=None, width=100))
    problems = check(cfg)
    for msg in problems:
        print("PROBLEM", msg, file=sys.stderr)
    print(f"# {len(problems)} problems", file=sys.stderr)
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()

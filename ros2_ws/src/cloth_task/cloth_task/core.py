"""ROS-free parts of the task supervisor: phases, config, launch-argument checks, named poses,
sweep order and the phase 2 geometry.

Kept free of rclpy so it can be unit-tested with plain pytest.
"""

from __future__ import annotations

import math
import os
import statistics
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from enum import StrEnum

import yaml

SPEED_MIN, SPEED_MAX = 0.1, 1.0
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
MODES = ("fixed_view", "search")

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w


class Phase(StrEnum):
    INIT = "init"
    GO_TO_VIEW = "go_to_view"  # fixed_view mode: ready pose → start pose over the box
    SEARCH = "search"  # search mode (phase 1): sweeping, watching /cloth_detection_status
    HALTING = "halting"  # search mode: stop sent, waiting for the arm to be still
    DETECT = "detect"  # arm still over the cloth: collect /cloth_target_pose samples
    APPROACH = "approach"  # phase 2: moving to the approach pose above the cloth
    AT_APPROACH = "at_approach"  # phase 2 done
    TRIAL_GRASP = "trial_grasp"  # phase 3: open, straight down, close
    VALIDATE = "validate"  # phase 4: lift straight up, check the fingers; retry on a miss
    HOLDING = "holding"  # phase 4 done: cloth in the gripper, lifted
    PLACE = "place"  # phase 5: carry to the place target, release
    DONE = "done"
    ERROR = "error"


class ConfigError(ValueError):
    pass


class TargetRejected(ValueError):
    """A target outside the workspace. No motion was started."""


@dataclass(frozen=True)
class SearchConfig:
    waypoints: tuple[tuple[float, ...], ...]  # joint positions, rad, ARM_JOINTS order
    sweep_speed_factor: float  # multiplies execution_speed while sweeping
    max_sweeps: int  # passes over the table before giving up; 0 = forever


@dataclass(frozen=True)
class DetectConfig:
    samples: int  # /cloth_target_pose messages to take the median of
    timeout_s: float
    max_spread_m: float  # samples further than this from the median → error


@dataclass(frozen=True)
class Workspace:
    x: tuple[float, float]
    y: tuple[float, float]
    z: tuple[float, float]

    def check(self, p: Vec3, what: str) -> None:
        for axis, v, (lo, hi) in zip("xyz", p, (self.x, self.y, self.z), strict=True):
            if not lo <= v <= hi:
                raise TargetRejected(
                    f"{what} {axis}={v:.3f} m is outside the workspace [{lo}, {hi}]"
                )


@dataclass(frozen=True)
class ApproachConfig:
    height_m: float  # TCP above the cloth surface
    min_height_m: float  # when height_m is out of reach (a high pile), lower down to this
    max_tilt_deg: float  # allowed angle between the gripper approach axis and straight down
    max_tilt_fallback_deg: float  # tried when max_tilt_deg is out of reach at every height
    max_roll_deg: float  # allowed turn about the approach axis at the goal, from the natural
    # wrist pose (joint6 ≈ 0, fingers across the radial direction); the path leaves it free
    position_tolerance_m: float
    keep_down_on_path: bool  # path constraint: stay within max_tilt the whole way
    planning_time_s: float
    planning_attempts: int
    workspace: Workspace  # the cloth point must be inside, base_link, m


@dataclass(frozen=True)
class BoxConfig:
    center: tuple[float, float]  # x, y of the box centre, base_link, m
    size: tuple[float, float]  # outer x, y, m
    height: float  # wall height above the table, m
    wall: float  # wall thickness, m


@dataclass(frozen=True)
class GraspConfig:
    depth_m: float  # TCP goes this far below the detected cloth surface
    floor_z_m: float  # but never below this (base_link z), whatever the detection says
    open_m: float  # finger position (joint_left, m) to open to before descending
    close_m: float  # finger position commanded to close (0 = fully closed)
    effort: float  # GripperCommand max_effort (the real gripper uses its own torque limit)
    lift_m: float  # straight up after closing
    empty_below_m: float  # finger position after the lift below this = nothing in the gripper
    max_attempts: int  # detect → approach → grasp → lift cycles before giving up
    linear_speed: float  # velocity scale for the straight descent and lift (× execution_speed)
    check_fingers: bool  # after the lift: fingers closed on nothing = missed → retry


@dataclass(frozen=True)
class PlaceConfig:
    max_tilt_deg: float  # xyz targets: gripper within this of straight down (cloth drops off)
    max_roll_deg: float  # xyz targets: turn about the gripper axis, from joint6 ≈ 0
    position_tolerance_m: float
    planning_time_s: float
    planning_attempts: int
    release_wait_s: float  # after opening, before moving away
    color_targets: dict[str, int]  # place_target:=color: detected class → place target id


PlaceTarget = Vec3 | str  # xyz in base_link (planned) or a named pose from poses.yaml (PTP)


@dataclass(frozen=True)
class TaskConfig:
    ready_pose: str | None  # named pose to unfold through before anything else
    planning_time_s: float  # joint-space moves
    still_velocity: float  # rad/s: every joint below this = the arm is still
    still_timeout_s: float
    search: SearchConfig
    detect: DetectConfig
    approach: ApproachConfig
    box: BoxConfig | None  # collision object for the planning scene
    grasp: GraspConfig
    place: PlaceConfig
    place_targets: dict[int, PlaceTarget]  # id → xyz (m, base_link) or a named pose


# --- launch arguments ---


def check_speed(value) -> float:
    """The execution_speed launch argument, as a float in [SPEED_MIN, SPEED_MAX]."""
    try:
        speed = float(value)
    except (TypeError, ValueError):
        speed = math.nan
    if isinstance(value, bool) or not SPEED_MIN <= speed <= SPEED_MAX:
        raise ConfigError(f"execution_speed must be in [{SPEED_MIN}, {SPEED_MAX}], got {value!r}")
    return speed


def check_mode(value) -> str:
    if value not in MODES:
        raise ConfigError(f"mode must be one of {MODES}, got {value!r}")
    return value


def check_place_target(cfg: TaskConfig, value) -> int | str:
    """The place_target launch argument: an id in place_targets (exact integer), or "color"
    (the target follows the detected class, `place.color_targets`)."""
    if value == "color":
        return "color"
    try:
        target_id = int(value)
    except (TypeError, ValueError):
        target_id = None
    if isinstance(value, bool) or target_id != value or target_id not in cfg.place_targets:
        raise ConfigError(
            f"place_target must be one of {sorted(cfg.place_targets)} or 'color', got {value!r}"
        )
    return target_id


def place_target_id(cfg: TaskConfig, choice: int | str, color: str | None) -> int:
    """The place target for this cloth: the fixed one, or the one for its color class."""
    if choice != "color":
        return choice
    if color not in cfg.place.color_targets:
        raise ValueError(
            f"place_target:=color but the detected class is {color!r} "
            f"(known: {sorted(cfg.place.color_targets)})"
        )
    return cfg.place.color_targets[color]


def majority(labels: Sequence[str]) -> str | None:
    """Most frequent label (first seen wins a tie); None for no labels."""
    if not labels:
        return None
    counts: dict[str, int] = {}
    for label in labels:
        counts[label] = counts.get(label, 0) + 1
    return max(counts, key=counts.get)


# --- task config ---


def _joints_rad(values, what: str) -> tuple[float, ...]:
    if len(values) != len(ARM_JOINTS):
        raise ConfigError(f"{what} needs {len(ARM_JOINTS)} joint values, got {len(values)}")
    return tuple(math.radians(float(v)) for v in values)


def _range(v) -> tuple[float, float]:
    lo, hi = (float(x) for x in v)
    if lo > hi:
        raise ConfigError(f"range {v} has min > max")
    return lo, hi


def parse_task_config(data: dict) -> TaskConfig:
    try:
        s, d, a = data["search"], data.get("detect", {}), data["approach"]
        search = SearchConfig(
            waypoints=tuple(_joints_rad(wp, "search.waypoints_deg") for wp in s["waypoints_deg"]),
            sweep_speed_factor=float(s.get("sweep_speed_factor", 0.5)),
            max_sweeps=int(s.get("max_sweeps", 0)),
        )
        detect = DetectConfig(
            samples=int(d.get("samples", 3)),
            timeout_s=float(d.get("timeout_s", 20.0)),
            max_spread_m=float(d.get("max_spread_m", 0.04)),
        )
        ws = a["workspace"]
        approach = ApproachConfig(
            height_m=float(a["height_m"]),
            min_height_m=float(a.get("min_height_m", a["height_m"])),
            max_tilt_deg=float(a.get("max_tilt_deg", 20.0)),
            max_tilt_fallback_deg=float(
                a.get("max_tilt_fallback_deg", a.get("max_tilt_deg", 20.0))
            ),
            max_roll_deg=float(a.get("max_roll_deg", 45.0)),
            position_tolerance_m=float(a.get("position_tolerance_m", 0.002)),
            keep_down_on_path=bool(a.get("keep_down_on_path", True)),
            planning_time_s=float(a.get("planning_time_s", 10.0)),
            planning_attempts=int(a.get("planning_attempts", 5)),
            workspace=Workspace(_range(ws["x"]), _range(ws["y"]), _range(ws["z"])),
        )
        b = data.get("scene", {}).get("box")
        box = (
            None
            if b is None
            else BoxConfig(
                center=tuple(float(v) for v in b["center"]),
                size=tuple(float(v) for v in b["size"]),
                height=float(b["height"]),
                wall=float(b.get("wall", 0.01)),
            )
        )
        g = data["grasp"]
        grasp = GraspConfig(
            depth_m=float(g.get("depth_m", 0.015)),
            floor_z_m=float(g.get("floor_z_m", 0.005)),
            open_m=float(g.get("open_m", 0.04)),
            close_m=float(g.get("close_m", 0.0)),
            effort=float(g.get("effort", 10.0)),
            lift_m=float(g.get("lift_m", 0.10)),
            empty_below_m=float(g.get("empty_below_m", 0.002)),
            max_attempts=int(g.get("max_attempts", 3)),
            linear_speed=float(g.get("linear_speed", 0.5)),
            check_fingers=bool(g.get("check_fingers", True)),
        )
        pl = data.get("place", {})
        place = PlaceConfig(
            max_tilt_deg=float(pl.get("max_tilt_deg", 45.0)),
            max_roll_deg=float(pl.get("max_roll_deg", 60.0)),
            position_tolerance_m=float(pl.get("position_tolerance_m", 0.01)),
            planning_time_s=float(pl.get("planning_time_s", 10.0)),
            planning_attempts=int(pl.get("planning_attempts", 5)),
            release_wait_s=float(pl.get("release_wait_s", 0.5)),
            color_targets={
                str(k): int(v)
                for k, v in pl.get("color_targets", {"light": 1, "dark": 2, "colored": 3}).items()
            },
        )
        cfg = TaskConfig(
            ready_pose=data.get("ready_pose"),
            planning_time_s=float(data.get("planning_time_s", 5.0)),
            still_velocity=float(data.get("still_velocity", 0.01)),
            still_timeout_s=float(data.get("still_timeout_s", 3.0)),
            search=search,
            detect=detect,
            approach=approach,
            box=box,
            grasp=grasp,
            place=place,
            place_targets={
                int(k): v if isinstance(v, str) else tuple(float(x) for x in v)
                for k, v in data["place_targets"].items()
            },
        )
    except (KeyError, TypeError, ValueError) as e:
        if isinstance(e, ConfigError):
            raise
        raise ConfigError(f"bad task config: {e!r}") from e

    if len(search.waypoints) < 2:
        raise ConfigError("search.waypoints_deg needs at least 2 waypoints")
    if not 0.0 < search.sweep_speed_factor <= 1.0:
        raise ConfigError("search.sweep_speed_factor must be in (0, 1]")
    if search.max_sweeps < 0:
        raise ConfigError("search.max_sweeps must be >= 0")
    if detect.samples < 1:
        raise ConfigError("detect.samples must be >= 1")
    if not 0 < approach.min_height_m <= approach.height_m:
        raise ConfigError("approach needs 0 < min_height_m <= height_m")
    if not 0.0 <= approach.max_tilt_deg <= approach.max_tilt_fallback_deg < 90.0:
        raise ConfigError("approach needs 0 <= max_tilt_deg <= max_tilt_fallback_deg < 90")
    if not 0.0 <= approach.max_roll_deg <= 180.0:
        raise ConfigError("approach.max_roll_deg must be in [0, 180]")
    if not grasp.close_m <= grasp.empty_below_m < grasp.open_m:
        raise ConfigError("grasp needs close_m <= empty_below_m < open_m")
    if grasp.depth_m < 0 or grasp.lift_m <= 0 or grasp.max_attempts < 1:
        raise ConfigError("grasp needs depth_m >= 0, lift_m > 0, max_attempts >= 1")
    if not 0.0 < grasp.linear_speed <= 1.0:
        raise ConfigError("grasp.linear_speed must be in (0, 1]")
    if box is not None and (len(box.center) != 2 or len(box.size) != 2):
        raise ConfigError("scene.box.center and scene.box.size need [x, y]")
    if any(not isinstance(t, str) and len(t) != 3 for t in cfg.place_targets.values()):
        raise ConfigError("every place target needs [x, y, z] or a pose name")
    if not 0.0 <= place.max_tilt_deg < 90.0:
        raise ConfigError("place.max_tilt_deg must be in [0, 90)")
    bad = {c: t for c, t in place.color_targets.items() if t not in cfg.place_targets}
    if bad:
        raise ConfigError(f"place.color_targets point at unknown place targets: {bad}")
    return cfg


def load_task_config(path: str) -> TaskConfig:
    with open(path) as f:
        return parse_task_config(yaml.safe_load(f))


# --- named poses (config/poses.yaml) ---

POSES_HEADER = """\
# Named arm poses: joint1..joint6 in degrees.
# Record the current arm pose:  ros2 run cloth_task record_pose <name>
# Move to a pose:               ros2 run cloth_task go_to_pose <name> [<name> ...]
# `ready` unfolds the arm from the folded home pose (shoulder first); the task goes through it.
"""


def load_poses(path: str) -> dict[str, tuple[float, ...]]:
    """Named poses in radians."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {
        str(name): _joints_rad(q, f"pose {name!r}") for name, q in data.get("poses", {}).items()
    }


def resolve_pose(poses: dict[str, tuple[float, ...]], name: str, path: str) -> tuple[float, ...]:
    if name not in poses:
        raise ConfigError(
            f"pose {name!r} is not in {path} (known: {sorted(poses)}); "
            f"record it with: ros2 run cloth_task record_pose {name}"
        )
    return poses[name]


def save_pose(path: str, name: str, q_deg: Sequence[float]) -> None:
    """Add or replace one named pose (degrees), keeping the others."""
    if not name.replace("_", "").isalnum():
        raise ConfigError(f"pose name must be letters, digits and _, got {name!r}")
    if len(q_deg) != len(ARM_JOINTS):
        raise ConfigError(f"a pose needs {len(ARM_JOINTS)} joint values, got {len(q_deg)}")
    data = {}
    if os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    poses = {str(k): [float(v) for v in q] for k, q in data.get("poses", {}).items()}
    poses[name] = [round(float(v), 2) for v in q_deg]
    lines = [POSES_HEADER, "poses:"]
    lines += [f"  {k}: [{', '.join(f'{v:.2f}' for v in q)}]" for k, q in poses.items()]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# --- phase 1 ---


def sweep_sequence(
    waypoints: Sequence[tuple[float, ...]], max_sweeps: int
) -> Iterator[tuple[int, tuple[float, ...]]]:
    """Yield (sweep number, waypoint), going back and forth over the waypoints.

    Sweep 1 is waypoints[0] … waypoints[-1] (the first one is the move to the start). Each
    later sweep runs the other way and skips the waypoint the arm is already at.
    `max_sweeps` = 0 means forever.
    """
    order = list(waypoints)
    for wp in order:
        yield 1, wp
    n = 2
    while max_sweeps == 0 or n <= max_sweeps:
        order.reverse()
        for wp in order[1:]:
            yield n, wp
        n += 1


# --- detection and phase 2 ---


def _median3(points: Sequence[Vec3]) -> Vec3:
    return tuple(statistics.median(p[i] for p in points) for i in range(3))


def median_point(points: Sequence[Vec3], max_spread_m: float) -> Vec3:
    """Median of the detections that agree. Samples further than max_spread_m from the per-axis
    median are dropped (a bad SAM3 frame, or another cloth that was the largest blob for one
    frame); raises ValueError if fewer than a majority are left."""
    if not points:
        raise ValueError("no detections")
    m = _median3(points)
    inliers = [p for p in points if math.dist(p, m) <= max_spread_m]
    if len(inliers) < len(points) // 2 + 1:
        raise ValueError(
            f"detections disagree: only {len(inliers)}/{len(points)} within "
            f"{max_spread_m * 1000:.0f} mm of the median"
        )
    return _median3(inliers)


def quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def down_orientation(yaw: float) -> Quat:
    """gripper_end orientation with its approach axis (+X) pointing straight down (−Z of
    base_link), turned by `yaw` about the vertical: Rz(yaw) · Ry(90°). yaw = atan2(y, x) of the
    target is the natural wrist (joint6 ≈ 0): at box_view (y = 0) TF shows exactly Ry(90°)."""
    rz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    ry = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    return quat_multiply(rz, ry)


def approach_axis(q: Quat) -> Vec3:
    """The gripper approach axis (+X of gripper_end) in base_link for orientation q."""
    x, y, z, w = q
    return (1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w))


def tilt_from_down_deg(q: Quat) -> float:
    """Angle between the approach axis and straight down."""
    ax = approach_axis(q)
    return math.degrees(math.acos(max(-1.0, min(1.0, -ax[2]))))


def approach_heights(cfg: ApproachConfig) -> tuple[float, ...]:
    """Heights above the cloth to try, highest first: height_m, halfway, min_height_m."""
    mid = (cfg.height_m + cfg.min_height_m) / 2
    return tuple(sorted({cfg.height_m, round(mid, 4), cfg.min_height_m}, reverse=True))


def approach_pose(
    cloth: Vec3, cfg: ApproachConfig, height: float | None = None
) -> tuple[Vec3, Quat]:
    """Phase 2 target for gripper_end: `height` (default height_m) straight above the cloth
    surface point, approach axis down. Raises TargetRejected if the cloth is outside the
    workspace."""
    cfg.workspace.check(cloth, "cloth")
    x, y, z = cloth
    h = cfg.height_m if height is None else height
    return (x, y, z + h), down_orientation(math.atan2(y, x))


# --- phases 3 and 4 ---


def grasp_point(cloth: Vec3, approach_xyz: Vec3, cfg: GraspConfig) -> Vec3:
    """Straight below the approach point: `depth_m` under the cloth surface, not below
    `floor_z_m`. XY stays the approach point's, so the descent is purely vertical."""
    return (approach_xyz[0], approach_xyz[1], max(cloth[2] - cfg.depth_m, cfg.floor_z_m))


def lifted(xyz: Vec3, cfg: GraspConfig) -> Vec3:
    return (xyz[0], xyz[1], xyz[2] + cfg.lift_m)


def holding_cloth(finger_m: float, cfg: GraspConfig) -> bool:
    """Phase 4 check: fingers stopped by the cloth, not closed on nothing."""
    return finger_m >= cfg.empty_below_m


def vertical_keep_roll(q: Quat) -> Quat:
    """`q` turned by the smallest rotation that points its approach axis straight down: the
    grasp orientation. Keeps the fingers' roll about the axis (the shortest-arc rotation adds
    none). A tilted gripper low over a table hits it with the side of its open fingers."""
    ax, ay, az = approach_axis(q)
    # axis = a × (0, 0, -1), angle = acos(a · (0, 0, -1))
    cx, cy = -ay, ax
    s = math.hypot(cx, cy)
    angle = math.atan2(s, -az)
    if s < 1e-9:
        return q if az < 0 else quat_multiply((1.0, 0.0, 0.0, 0.0), q)  # down already / up
    h = math.sin(angle / 2) / s
    align = (cx * h, cy * h, 0.0, math.cos(angle / 2))
    return quat_multiply(align, q)


def turn_about_approach(q: Quat, deg: float) -> Quat:
    """`q` rolled by `deg` about its own approach axis (gripper_end X): turns the line the
    fingers close along, e.g. to keep the open fingers off a box wall."""
    h = math.radians(deg) / 2
    return quat_multiply(q, (math.sin(h), 0.0, 0.0, math.cos(h)))


def slerp(q0: Quat, q1: Quat, t: float) -> Quat:
    """Spherical interpolation between unit quaternions (shortest way), t in [0, 1]."""
    dot = sum(a * b for a, b in zip(q0, q1, strict=True))
    if dot < 0:  # same rotation, other sign: take the short way
        q1, dot = tuple(-v for v in q1), -dot
    if dot > 0.9995:  # nearly equal: linear is fine
        q = tuple(a + t * (b - a) for a, b in zip(q0, q1, strict=True))
    else:
        theta = math.acos(dot)
        s0, s1 = math.sin((1 - t) * theta), math.sin(t * theta)
        q = tuple((s0 * a + s1 * b) / math.sin(theta) for a, b in zip(q0, q1, strict=True))
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)

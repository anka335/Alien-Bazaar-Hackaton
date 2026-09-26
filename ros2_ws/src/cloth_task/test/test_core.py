import copy
import math
import os
from itertools import islice

import pytest
import yaml

from cloth_task.core import (
    ConfigError,
    TargetRejected,
    approach_axis,
    approach_heights,
    approach_pose,
    check_mode,
    check_place_target,
    check_speed,
    down_orientation,
    grasp_point,
    holding_cloth,
    lifted,
    load_poses,
    load_task_config,
    majority,
    median_point,
    parse_task_config,
    place_target_id,
    quat_multiply,
    resolve_pose,
    save_pose,
    slerp,
    sweep_sequence,
    tilt_from_down_deg,
    turn_about_approach,
    vertical_keep_roll,
)
from cloth_task.sim_cloth_detector import in_view

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")

CFG = {
    "ready_pose": "ready",
    "search": {
        "waypoints_deg": [[-45, 59.4, 72.6, -73.2, 0, 0], [45, 59.4, 72.6, -73.2, 0, 0]],
    },
    "approach": {
        "height_m": 0.10,
        "workspace": {"x": [0.1, 0.45], "y": [-0.3, 0.3], "z": [-0.02, 0.15]},
    },
    "scene": {"box": {"center": [0.3, 0.0], "size": [0.3, 0.25], "height": 0.06}},
    "grasp": {},
    "place_targets": {1: [0.25, -0.3, 0.15], 2: [0.1, -0.35, 0.15], 3: [-0.05, -0.35, 0.15]},
}


def patched(**sections):
    data = copy.deepcopy(CFG)
    for section, values in sections.items():
        data[section] = {**data[section], **values}
    return data


# --- config ---


def test_shipped_configs_load():
    cfg = load_task_config(os.path.join(CONFIG_DIR, "task.yaml"))
    poses = load_poses(os.path.join(CONFIG_DIR, "poses.yaml"))
    assert cfg.ready_pose in poses
    assert {"home", "ready", "box_view"} <= set(poses)


def test_parse_converts_degrees_and_defaults():
    cfg = parse_task_config(CFG)
    assert cfg.search.waypoints[0][0] == pytest.approx(math.radians(-45))
    assert cfg.search.max_sweeps == 0
    assert cfg.grasp.check_fingers  # on unless the config turns it off
    assert cfg.approach.max_tilt_deg == 20.0
    assert cfg.detect.samples == 3
    assert cfg.box.wall == 0.01
    assert cfg.place_targets[check_place_target(cfg, 2)] == (0.1, -0.35, 0.15)


@pytest.mark.parametrize(
    "bad",
    [
        {"search": {"waypoints_deg": [[0, 0, 0, 0, 0, 0]]}},  # one waypoint is no sweep
        {"search": {"waypoints_deg": [[0, 0, 0], [1, 1, 1]]}},  # wrong joint count
        {"search": {"sweep_speed_factor": 0}},
        {"search": {"max_sweeps": -1}},
        {"approach": {"height_m": 0}},
        {"approach": {"min_height_m": 0.2}},  # above height_m
        {"approach": {"max_tilt_deg": 95}},
        {"approach": {"max_roll_deg": -1}},
        {"approach": {"workspace": {"x": [0.5, 0.1], "y": [0, 1], "z": [0, 1]}}},  # min > max
        {"scene": {"box": {"center": [0.3], "size": [0.3, 0.2], "height": 0.1}}},
        {"grasp": {"empty_below_m": 0.05}},  # not below open_m
        {"grasp": {"close_m": 0.003, "empty_below_m": 0.002}},  # closing can't read as empty
        {"grasp": {"max_attempts": 0}},
        {"grasp": {"lift_m": 0}},
        {"grasp": {"linear_speed": 1.5}},
    ],
)
def test_bad_config_rejected(bad):
    with pytest.raises(ConfigError):
        parse_task_config(patched(**bad))


def test_missing_section_rejected():
    data = copy.deepcopy(CFG)
    del data["approach"]
    with pytest.raises(ConfigError):
        parse_task_config(data)


# --- launch arguments ---


@pytest.mark.parametrize("speed", [0.1, 0.5, 1.0, 1])
def test_speed_accepted(speed):
    assert check_speed(speed) == speed


@pytest.mark.parametrize("speed", [0.0, 0.05, 1.5, -1, "abc", float("nan"), True])
def test_speed_rejected(speed):
    with pytest.raises(ConfigError):
        check_speed(speed)


def test_place_target_accepts_integral_float():
    assert check_place_target(parse_task_config(CFG), 3.0) == 3


@pytest.mark.parametrize("target", [0, 4, -1, 2.5, "x", True])
def test_unknown_place_target_rejected(target):
    with pytest.raises(ConfigError):
        check_place_target(parse_task_config(CFG), target)


def test_mode():
    assert check_mode("search") == "search"
    with pytest.raises(ConfigError):
        check_mode("sweep")


# --- named poses ---


def test_save_pose_adds_and_replaces(tmp_path):
    path = str(tmp_path / "poses.yaml")
    save_pose(path, "ready", [0, 20, 15, 0, 0, 0])
    save_pose(path, "box_view", [1.234, 2, 3, 4, 5, 6])
    save_pose(path, "ready", [0, 21, 15, 0, 0, 0])
    poses = load_poses(path)
    assert list(poses) == ["ready", "box_view"]  # order kept, replaced in place
    assert poses["ready"][1] == pytest.approx(math.radians(21))
    assert poses["box_view"][0] == pytest.approx(math.radians(1.23))
    with open(path) as f:
        text = f.read()
    assert yaml.safe_load(text)["poses"]["box_view"][0] == 1.23
    assert text.startswith("# Named arm poses")


@pytest.mark.parametrize("name,q", [("bad name", [0] * 6), ("ok", [0] * 5), ("", [0] * 6)])
def test_save_pose_rejects(tmp_path, name, q):
    with pytest.raises(ConfigError):
        save_pose(str(tmp_path / "poses.yaml"), name, q)


def test_resolve_unknown_pose_says_how_to_record():
    with pytest.raises(ConfigError, match="record_pose box_view"):
        resolve_pose({}, "box_view", "poses.yaml")


def test_missing_poses_file_is_empty(tmp_path):
    assert load_poses(str(tmp_path / "nope.yaml")) == {}


# --- phase 1 ---


def test_sweep_goes_back_and_forth_without_repeating_endpoints():
    seq = list(sweep_sequence(["a", "b", "c"], max_sweeps=3))
    assert seq == [(1, "a"), (1, "b"), (1, "c"), (2, "b"), (2, "a"), (3, "b"), (3, "c")]


def test_sweep_forever():
    assert len(list(islice(sweep_sequence(["a", "b"], max_sweeps=0), 100))) == 100


def test_in_view_frustum():
    fov = (69.0, 42.0, 0.1, 1.2)
    assert in_view((0.0, 0.0, 0.5), *fov)
    assert not in_view((0.0, 0.0, -0.5), *fov)  # behind the camera
    assert not in_view((0.0, 0.0, 2.0), *fov)  # too far
    assert not in_view((0.5, 0.0, 0.5), *fov)  # 45° right, outside the 34.5° half-FOV
    assert in_view((0.3, 0.0, 0.5), *fov)  # 31° right
    assert not in_view((0.0, 0.3, 0.5), *fov)  # 31° down, outside the 21° half-FOV


# --- detection and phase 2 ---


def test_median_point_drops_an_outlier():
    good = [(0.30, 0.0, 0.03), (0.301, 0.001, 0.03), (0.299, 0.0, 0.031), (0.30, -0.001, 0.029)]
    assert median_point(good, 0.02) == pytest.approx((0.30, 0.0, 0.03), abs=1e-3)
    # one bad frame (or the other cloth for one frame) no longer ends the run
    assert median_point([*good, (0.40, 0.0, 0.03)], 0.02) == pytest.approx(
        (0.30, 0.0, 0.03), abs=1e-3
    )
    # three samples, one off: the two that agree win
    assert median_point([(0.3, 0, 0.03), (0.5, 0.2, 0.1), (0.301, 0, 0.03)], 0.02) == (
        pytest.approx((0.3005, 0.0, 0.03), abs=1e-3)
    )


def test_median_point_needs_a_majority():
    with pytest.raises(ValueError, match="disagree"):  # two cloths, half the frames each
        median_point([(0.3, 0, 0.03), (0.3, 0, 0.03), (0.3, 0.2, 0.03), (0.3, 0.2, 0.03)], 0.02)
    with pytest.raises(ValueError):
        median_point([], 0.02)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.0, math.pi])
def test_down_orientation_points_the_approach_axis_down(yaw):
    q = down_orientation(yaw)
    assert approach_axis(q) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)
    assert tilt_from_down_deg(q) == pytest.approx(0.0, abs=1e-6)
    assert math.hypot(*q) == pytest.approx(1.0)


def test_tilt_of_the_identity_is_horizontal():
    # At the home pose gripper_end has the identity rotation: gripper pointing forward.
    assert tilt_from_down_deg((0.0, 0.0, 0.0, 1.0)) == pytest.approx(90.0)


def test_approach_pose_is_above_the_cloth():
    cfg = parse_task_config(CFG).approach
    xyz, q = approach_pose((0.30, 0.05, 0.03), cfg)
    assert xyz == pytest.approx((0.30, 0.05, 0.13))
    assert tilt_from_down_deg(q) == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("cloth", [(0.60, 0.0, 0.03), (0.30, 0.5, 0.03), (0.30, 0.0, 0.30)])
def test_approach_rejects_cloth_outside_the_workspace(cloth):
    with pytest.raises(TargetRejected):
        approach_pose(cloth, parse_task_config(CFG).approach)


# --- phases 3 and 4 ---


def test_grasp_point_is_straight_below_the_approach_point():
    g = parse_task_config(CFG).grasp
    assert grasp_point((0.30, 0.0, 0.05), (0.301, 0.001, 0.15), g) == pytest.approx(
        (0.301, 0.001, 0.05 - g.depth_m)
    )


def test_grasp_point_never_below_the_floor():
    g = parse_task_config(CFG).grasp
    assert grasp_point((0.30, 0.0, 0.01), (0.30, 0.0, 0.11), g)[2] == g.floor_z_m


def test_lift_and_hold_check():
    g = parse_task_config(CFG).grasp
    assert lifted((0.3, 0.0, 0.02), g) == pytest.approx((0.3, 0.0, 0.02 + g.lift_m))
    assert holding_cloth(0.004, g)  # stopped by the cloth
    assert not holding_cloth(0.0, g)  # closed on nothing
    assert not holding_cloth(g.empty_below_m - 1e-4, g)


@pytest.mark.parametrize("tilt_deg,yaw", [(0.0, 0.3), (16.8, 0.0), (20.0, 1.2), (45.0, -2.0)])
def test_vertical_keep_roll(tilt_deg, yaw):
    # tilt a down orientation about base_link x by tilt_deg
    t = math.radians(tilt_deg)
    tilted = quat_multiply((math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)), down_orientation(yaw))
    assert tilt_from_down_deg(tilted) == pytest.approx(tilt_deg, abs=1e-4)  # acos near 1
    v = vertical_keep_roll(tilted)
    assert tilt_from_down_deg(v) == pytest.approx(0.0, abs=1e-4)
    assert math.hypot(*v) == pytest.approx(1.0)
    if tilt_deg == 0.0:
        assert v == pytest.approx(tilted)


def test_turn_about_approach_keeps_the_axis():
    q = down_orientation(0.4)
    for deg in (90.0, -90.0, 30.0):
        t = turn_about_approach(q, deg)
        assert approach_axis(t) == pytest.approx(approach_axis(q), abs=1e-9)
    assert turn_about_approach(q, 0.0) == pytest.approx(q)


# --- phase 5 ---


def test_place_target_choice():
    cfg = parse_task_config(CFG)
    assert check_place_target(cfg, 2) == 2
    assert check_place_target(cfg, "color") == "color"
    assert place_target_id(cfg, 2, "dark") == 2  # fixed target ignores the class
    assert place_target_id(cfg, "color", "light") == 1
    assert place_target_id(cfg, "color", "dark") == 2
    assert place_target_id(cfg, "color", "colored") == 3
    with pytest.raises(ValueError):
        place_target_id(cfg, "color", None)  # no class detected
    with pytest.raises(ValueError):
        place_target_id(cfg, "color", "striped")


def test_place_target_can_be_a_pose_name():
    cfg = parse_task_config({**CFG, "place_targets": {1: "bin_1", 2: [0.1, -0.3, 0.15], 3: "b3"}})
    assert cfg.place_targets[1] == "bin_1"
    assert cfg.place_targets[2] == (0.1, -0.3, 0.15)


def test_color_targets_must_point_at_known_targets():
    with pytest.raises(ConfigError):
        parse_task_config({**CFG, "place": {"color_targets": {"light": 7}}})
    with pytest.raises(ConfigError):
        parse_task_config({**CFG, "place": {"max_tilt_deg": 90}})


def test_majority():
    assert majority(["dark", "light", "dark"]) == "dark"
    assert majority(["light", "dark"]) == "light"  # tie: first seen
    assert majority([]) is None


def test_slerp_halves_the_tilt():
    t = math.radians(30)
    tilted = quat_multiply((math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)), down_orientation(0.3))
    v = vertical_keep_roll(tilted)
    assert tilt_from_down_deg(slerp(tilted, v, 0.5)) == pytest.approx(15.0, abs=1e-3)
    assert slerp(tilted, v, 0.0) == pytest.approx(tilted)
    assert slerp(tilted, v, 1.0) == pytest.approx(v)
    neg = tuple(-x for x in v)  # same rotation, other sign
    assert tilt_from_down_deg(slerp(tilted, neg, 0.5)) == pytest.approx(15.0, abs=1e-3)


def test_approach_heights_go_down_to_the_minimum():
    cfg = parse_task_config(patched(approach={"min_height_m": 0.05})).approach
    assert approach_heights(cfg) == (0.10, 0.075, 0.05)
    xyz, _ = approach_pose((0.3, 0.1, 0.15), cfg, 0.05)
    assert xyz == pytest.approx((0.3, 0.1, 0.20))
    only = parse_task_config(CFG).approach  # min defaults to height_m: one height
    assert approach_heights(only) == (0.10,)

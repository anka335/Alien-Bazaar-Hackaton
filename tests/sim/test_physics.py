"""The rover scene on MuJoCo: the real arm loop, camera, calibration and detectors, and a
scripted (no vision) load and unload."""

import math

import numpy as np
import pytest

from sorter.app import build_system
from sorter.core.types import ArmPoint, ColorClass, Zone

pytest.importorskip("mujoco")


def _system(sim_config, scenes, **sim):
    cfg = sim_config.model_copy(deep=True)
    cfg.sim.scenes = scenes
    for k, v in sim.items():
        setattr(cfg.sim, k, v)
    system = build_system(cfg, sim=True)
    system.camera.start()
    system.arm.start()
    return system


@pytest.fixture
def load_system(sim_config):
    sim_config.sim.load.socks = [ColorClass.DARK]
    sim_config.sim.load.area = "view"
    system = _system(sim_config, ["load"])
    yield system
    system.camera.close()
    system.world.stop()


@pytest.fixture
def unload_system(sim_config):
    sim_config.sim.unload.cargo = {ColorClass.COLORED: 1}
    system = _system(sim_config, ["unload"])
    yield system
    system.camera.close()
    system.world.stop()


def _top(world, item) -> ArmPoint:
    """Where to grasp item `item`: its middle, at the top of the cloth there."""
    v = world.vertices(item)
    c = v.mean(axis=0)
    near = v[np.hypot(v[:, 0] - c[0], v[:, 1] - c[1]) < 25]
    return ArmPoint(float(c[0]), float(c[1]), float(near[:, 2].max()))


def test_scenes_put_the_socks_in_place(sim_config):
    sim_config.sim.load.socks = [ColorClass.LIGHT, ColorClass.DARK]
    sim_config.sim.load.area = "view"  # clear of the station's bins
    sim_config.sim.unload.cargo = {ColorClass.LIGHT: 1, ColorClass.COLORED: 1}
    world = _system(sim_config, ["load", "unload"]).world
    try:
        where = [(it.color, world.location(it.id)) for it in world.items]
        assert where == [
            (ColorClass.LIGHT, ("floor", None)),
            (ColorClass.DARK, ("floor", None)),
            (ColorClass.LIGHT, ("cargo", None)),  # one box, not split by color (D-035)
            (ColorClass.COLORED, ("cargo", None)),
        ]
    finally:
        world.stop()


def test_arm_reaches_the_look_pose(load_system):
    load_system.arm.look(Zone.CARGO)
    q = np.array(load_system.arm.joints())
    assert np.degrees(np.abs(q - load_system.cfg.poses["look_cargo"])).max() < 1.0
    assert load_system.world.looking_at is Zone.CARGO


def test_floor_detector_finds_the_sock(load_system):
    s = load_system
    obs = s.observer.observe(Zone.FLOOR)
    result = s.floor_detector.detect(obs.frame)
    assert [sock.color for sock in result.socks] == [ColorClass.DARK]
    p = s.calibration.to_arm(obs, result.socks[0].grasp)
    cloth = s.world.vertices(0)
    # the grasp point lies on the sock: within its thickness of some vertex
    assert np.linalg.norm(cloth - [p.x, p.y, p.z], axis=1).min() < 15


def test_scripted_load(load_system):
    """A sock from the floor into the cargo box, from its known position."""
    s, world = load_system, load_system.world
    result = s.arm.pick(_top(world, 0), Zone.FLOOR)
    assert world.location(0)[0] == "gripper" and not result.likely_empty
    s.arm.drop_to_cargo(ColorClass.DARK)
    s.arm.look(Zone.CARGO)  # let it land
    assert world.location(0) == ("cargo", None)


def test_scripted_unload(unload_system):
    """A sock from the cargo box into its laundry bin, from its known position."""
    s, world = unload_system, unload_system.world
    s.arm.look(Zone.CARGO)
    result = s.arm.pick(_top(world, 0), Zone.CARGO, math.pi / 2)
    assert world.location(0)[0] == "gripper" and not result.likely_empty
    s.arm.drop_to_laundry(ColorClass.COLORED)
    s.arm.look(Zone.FLOOR)  # let it land
    assert world.location(0) == ("laundry", ColorClass.COLORED)


def test_miss_prob_makes_a_grasp_catch_nothing(sim_config):
    sim_config.sim.load.socks = [ColorClass.DARK]
    sim_config.sim.load.area = "view"
    s = _system(sim_config, ["load"], miss_prob=1.0)
    try:
        s.arm.pick(_top(s.world, 0), Zone.FLOOR)  # likely_empty is a hint: cloth may be pinched
        s.arm.home()
        assert s.world.location(0)[0] == "floor"
    finally:
        s.camera.close()
        s.world.stop()


def test_twin_sends_the_scene_and_the_cloth(load_system):
    from sorter.dashboard.twin import Twin

    s = load_system
    twin = Twin(s.arm, s.calibration, s.cfg, s.world)
    layout = twin.layout()
    names = {p["name"] for p in layout["parts"]}
    assert {"floor", "deck", "cargo_floor"} <= names
    n = layout["cloth_n"]
    items = twin.state()["items"]
    assert len(items) == 1 and len(items[0]["vertices"]) == n * n * 3
    # the rig (no sim world) builds the same scene from the config
    assert Twin(s.arm, s.calibration, s.cfg).layout()["parts"] == layout["parts"]

"""The physics simulator: the real arm loop, camera, calibration and box detector on MuJoCo."""

import numpy as np
import pytest

from sorter.app import build_system
from sorter.core.config import load_config
from sorter.core.types import BoxStatus, Zone

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def system():
    cfg = load_config(
        overrides={
            "sim": {"engine": "physics", "realtime": 0, "items": ["light", "dark"]},
            "state_machine": {"save_runs": False},
        }
    )
    system = build_system(cfg, sim=True)
    system.camera.start()
    system.arm.start()
    yield system
    system.camera.close()
    system.world.stop()


def test_arm_reaches_the_look_pose(system):
    system.arm.look(Zone.BOX)
    q = np.array(system.arm.joints())
    assert np.degrees(np.abs(q - system.cfg.poses["look_box"])).max() < 1.0
    assert system.world.looking_at is Zone.BOX


def test_pixels_of_a_cloth_map_onto_it(system):
    obs = system.observer.observe(Zone.BOX)
    items = system.camera.segment(obs.frame.color)
    assert items, "no cloth in view"
    r = system.box_detector.detect(obs.frame)
    assert r.status is BoxStatus.GRASP
    p = system.calibration.to_arm(obs, r.grasp)
    cloth = np.vstack([system.world.vertices(it.id) for it in system.world.items])
    # the grasp point lies on a cloth: within its thickness of some vertex
    assert np.linalg.norm(cloth - [p.x, p.y, p.z], axis=1).min() < 15


def test_pick_from_the_box_and_release_on_the_mat(system):
    world = system.world
    obs = system.observer.observe(Zone.BOX)
    grasp = system.box_detector.detect(obs.frame).grasp
    result = system.arm.pick(system.calibration.to_arm(obs, grasp), Zone.BOX)
    held = [it.id for it in world.items if world.location(it.id)[0] == "gripper"]
    assert held and not result.likely_empty
    system.arm.place_on_background()
    system.arm.look(Zone.BACKGROUND)  # let it land
    assert all(world.location(i)[0] == "background" for i in held)


def test_twin_sends_the_cloth(system):
    from sorter.dashboard.twin import Twin

    twin = Twin(system.arm, system.calibration, system.cfg, system.world)
    n = twin.layout()["cloth_n"]
    items = twin.state()["items"]
    assert n and len(items) == len(system.world.items)
    assert all(len(it["vertices"]) == n * n * 3 for it in items)

import threading
import time

import numpy as np
import pytest

from sorter.app import build_system
from sorter.arm import kinematics as kin
from sorter.core.errors import CalibrationError, EStopped, TargetRejected
from sorter.core.types import ArmPoint, BoxStatus, ColorClass, GraspPoint, PixelPoint, Zone


@pytest.fixture
def system(sim_config):
    system = build_system(sim_config, sim=True)
    system.arm.start()
    return system


def test_observe_renders_the_zone(system):
    obs = system.observer.observe(Zone.BOX)
    view = system.world.views[Zone.BOX]
    cfg = system.cfg.sim
    assert obs.zone is Zone.BOX and obs.T_base_cam is not None
    assert obs.T_base_cam[2, 3] == pytest.approx(view.cam_z_mm)
    assert obs.T_base_cam[2, 2] == pytest.approx(-1, abs=1e-3)  # looking straight down
    assert obs.frame.color.shape == (cfg.height, cfg.width, 3)
    floor = view.cam_z_mm - cfg.layout.box.floor_z_mm
    assert obs.frame.depth_mm[240, 320] <= round(floor)  # the box floor or a cloth on it
    assert obs.frame.depth_mm.min() < floor - 10  # items stick out


def test_fresh_frames_are_newer(system):
    a, b = system.camera.fresh(), system.camera.fresh()
    assert b.seq > a.seq and b.timestamp >= a.timestamp


def test_calibration_roundtrip(system):
    obs = system.observer.observe(Zone.BACKGROUND)
    cam_z = system.world.views[Zone.BACKGROUND].cam_z_mm
    p = system.calibration.to_arm(obs, GraspPoint(PixelPoint(100, 50), 240.0))
    assert p.z == pytest.approx(cam_z - 240.0)
    assert system.calibration.to_pixel(obs, p) == PixelPoint(100, 50)
    assert system.calibration.to_pixel(obs, ArmPoint(0, 0, 0)) is None
    with pytest.raises(CalibrationError):
        system.calibration.to_arm(obs, GraspPoint(PixelPoint(1, 1), 0.0))


def test_box_detection_and_pick(system):
    obs = system.observer.observe(Zone.BOX)
    box = system.box_detector.detect(obs.frame)
    assert box.status is BoxStatus.GRASP
    system.cfg.sim.miss_prob = 0.0
    system.cfg.sim.double_prob = 0.0
    n_box = len(system.world.at("box"))
    result = system.arm.pick(system.calibration.to_arm(obs, box.grasp), Zone.BOX)
    assert not result.likely_empty
    assert len(system.world.at("box")) == n_box - 1 and len(system.world.at("gripper")) == 1
    # the grasped point is avoided on the next detection
    again = system.box_detector.detect(system.observer.observe(Zone.BOX).frame, [box.grasp.px])
    assert again.status is BoxStatus.GRASP and again.grasp.px != box.grasp.px


def test_a_gripper_closing_above_the_cloth_catches_nothing(system):
    it = max(system.world.at("box"), key=system.world.top_z)
    top = system.world.top_z(it)
    system.cfg.sim.miss_prob = 0.0
    assert system.world.grasp((it.x, it.y, top + 30)) == []
    assert system.world.grasp((it.x, it.y, top - 5)) != []


def test_pick_outside_workspace_is_rejected_without_motion(system):
    system.arm.look(Zone.BOX)
    q = system.arm.joints()
    with pytest.raises(TargetRejected, match="outside"):
        system.arm.pick(ArmPoint(0, 0, 0), Zone.BOX)
    with pytest.raises(TargetRejected, match="not reachable"):  # inside, but far too high
        system.arm.pick(ArmPoint(255, 0, 500), Zone.BOX)
    assert system.arm.joints() == q
    assert system.world.looking_at is Zone.BOX


def test_released_items_land_on_what_is_below(system):
    world = system.world
    a, b = world.items[:2]
    a.location = "gripper"
    world.release((600.0, 0.0, 100.0))  # beyond the mat: the bare table
    assert a.location == "table"
    a.location = "gripper"
    system.arm.drop_to_bin(ColorClass.DARK)
    assert a.location == "bin" and a.bin is ColorClass.DARK
    b.location = "gripper"
    system.arm.place_on_background()
    assert b.location == "background"


def test_hold_blocks_motion_until_recover(system):
    system.arm.hold()
    with pytest.raises(EStopped):
        system.arm.look(Zone.BOX)
    system.arm.recover()
    system.arm.look(Zone.BOX)


def test_hold_interrupts_a_motion(system):
    system.cfg.sim.time_scale = 1.0  # the real arm's speed: rest → home takes seconds
    threading.Timer(0.2, system.arm.hold).start()
    t0 = time.monotonic()
    with pytest.raises(EStopped):
        system.arm.home()
    assert time.monotonic() - t0 < 1.0
    q = np.array(system.arm.joints())
    assert not np.allclose(q, 0) and not np.allclose(q, system.cfg.poses["home"])
    time.sleep(0.1)
    assert np.allclose(system.arm.joints(), q)  # frozen where it was


def test_motions_take_the_real_arms_time_times_the_scale(system):
    system.cfg.sim.time_scale = 0.05
    wps = np.array([system.arm.joints(), system.cfg.poses["home"]])
    expected = kin.path_duration(wps, system.cfg.arm.speed_scale) * 0.05
    t0 = time.monotonic()
    system.arm.home()
    assert time.monotonic() - t0 == pytest.approx(expected, rel=0.3, abs=0.02)


def test_start_refills_the_box_when_everything_is_sorted(system):
    world = system.world
    for it in world.items:
        it.location, it.bin = "bin", it.color
    world.items[0].location, world.items[0].bin = "table", None  # dropped next to a bin
    system.arm.start()
    assert all(it.location == "box" and it.bin is None for it in world.items)


def test_camera_follows_the_arm(system):
    world = system.world
    system.arm.look(Zone.BACKGROUND)
    pose = world.camera_pose()
    view = world.views[Zone.BACKGROUND]
    assert (pose.x, pose.y, pose.z) == pytest.approx((*view.center_mm, view.cam_z_mm))
    assert view.center_mm == pytest.approx(system.cfg.sim.layout.background.center_mm, abs=1)
    system.arm.drop_to_bin(ColorClass.LIGHT)  # ends back at home
    assert world.looking_at is None
    x, y, z = world.tcp()
    assert np.allclose((x, y, z), kin.fk_tcp(system.cfg.poses["home"])[:3, 3], atol=1)

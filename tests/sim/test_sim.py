import threading

import pytest

from sorter.app import build_system
from sorter.core.errors import CalibrationError, EStopped, TargetRejected
from sorter.core.types import ArmPoint, BoxStatus, GraspPoint, PixelPoint, Zone


@pytest.fixture
def system(sim_config):
    return build_system(sim_config, sim=True)


def test_observe_renders_the_zone(system):
    obs = system.observer.observe(Zone.BOX)
    assert obs.zone is Zone.BOX and obs.T_base_cam is not None
    cfg = system.cfg.sim
    assert obs.frame.color.shape == (cfg.height, cfg.width, 3)
    assert obs.frame.depth_mm.max() == cfg.cam_height_mm  # the box floor
    assert obs.frame.depth_mm.min() < cfg.cam_height_mm  # items stick out


def test_fresh_frames_are_newer(system):
    a, b = system.camera.fresh(), system.camera.fresh()
    assert b.seq > a.seq and b.timestamp >= a.timestamp


def test_calibration_roundtrip(system):
    obs = system.observer.observe(Zone.BACKGROUND)
    p = system.calibration.to_arm(obs, GraspPoint(PixelPoint(100, 50), 380.0))
    assert p.z == pytest.approx(20.0)  # 380 mm below a camera 400 mm above the surface
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
    assert len(system.world.at("box")) == n_box - 1
    # the grasped point is avoided on the next detection
    again = system.box_detector.detect(system.observer.observe(Zone.BOX).frame, [box.grasp.px])
    assert again.status is BoxStatus.GRASP and again.grasp.px != box.grasp.px


def test_pick_outside_workspace_is_rejected_without_motion(system):
    system.arm.look(Zone.BOX)
    with pytest.raises(TargetRejected):
        system.arm.pick(ArmPoint(0, 0, 0), Zone.BOX)
    assert system.world.looking_at is Zone.BOX


def test_hold_blocks_motion_until_recover(system):
    system.arm.hold()
    with pytest.raises(EStopped):
        system.arm.look(Zone.BOX)
    system.arm.recover()
    system.arm.look(Zone.BOX)


def test_hold_interrupts_a_motion(system):
    system.cfg.sim.motion_s = 5.0
    threading.Timer(0.05, system.arm.hold).start()
    with pytest.raises(EStopped):
        system.arm.home()


def test_start_refills_the_box_when_everything_is_sorted(system):
    world = system.world
    for it in world.items:
        it.location, it.bin = "bin", it.color
    system.arm.start()
    assert all(it.location == "box" and it.bin is None for it in world.items)


def test_camera_follows_the_arm(system):
    system.arm.look(Zone.BACKGROUND)
    pose = system.world.camera.pose()
    view = system.world.views[Zone.BACKGROUND]
    assert (pose.x, pose.y, pose.z) == (*view.center_mm, view.cam_z_mm)
    system.arm.drop_to_bin(system.world.items[0].color)
    pose = system.world.camera.pose()
    assert (pose.x, pose.y) == system.world.bin_xy[system.world.items[0].color]

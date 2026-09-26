"""The arm controller (block 5) on a recording driver, the committed rig layout, the real
driver in dry-run mode."""

import numpy as np
import pytest

from sorter.arm import kinematics as kin
from sorter.arm.controller import MIN_SPEED_SCALE, Controller, in_polygon
from sorter.core.config import load_config
from sorter.core.errors import EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, Zone
from sorter.sim.layout import check
from sorter.sim.world import camera_pose


class FakeDriver:
    """Jumps to the end of every path and records it."""

    def __init__(self, q0, grip_after_close=0.3):
        self.q = np.asarray(q0, dtype=float)
        self.paths: list[np.ndarray] = []
        self.speeds: list[float] = []
        self.gripper_cmds: list[float] = []
        self.grip_after_close = grip_after_close
        self.stopped = False
        self.connected = False

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def joints(self):
        return self.q.copy()

    def gripper(self):
        return 0.0

    def execute(self, wps, speed_scale):
        if self.stopped:
            raise EStopped("held")
        self.paths.append(np.asarray(wps))
        self.speeds.append(speed_scale)
        self.q = np.asarray(wps[-1], dtype=float)

    def set_gripper(self, opening):
        if self.stopped:
            raise EStopped("held")
        self.gripper_cmds.append(opening)
        return self.grip_after_close if opening == 0 else opening

    def stop(self):
        self.stopped = True

    def resume(self):
        self.stopped = False

    def fault(self):
        return None

    def clear_fault(self):
        pass


@pytest.fixture
def cfg():
    return load_config()


@pytest.fixture
def arm(cfg):
    return Controller(FakeDriver(cfg.poses["look_box"]), cfg.arm, cfg.poses, cfg.zones)


def tcp_z(q) -> float:
    return float(kin.fk_tcp(q)[2, 3])


def test_committed_poses_and_zones_fit_the_layout(cfg):
    assert check(cfg) == []  # every pick on the zone grids and every move between poses plans
    for zone, name in ((Zone.BOX, "look_box"), (Zone.BACKGROUND, "look_bg")):
        T = camera_pose(cfg.sim, cfg.poses[name])
        center = getattr(cfg.sim.layout, zone.value).center_mm
        assert T[:2, 3] == pytest.approx(center, abs=2)  # the camera is over the zone center
        assert T[2, 2] == pytest.approx(-1, abs=1e-3)  # looking straight down


def test_pick_goes_above_down_and_up(arm, cfg):
    target = ArmPoint(255, 0, 50)
    result = arm.pick(target, Zone.BOX)
    assert not result.likely_empty and result.gripper_opening == 0.3
    above, down, up = arm.driver.paths
    zone = cfg.zones[Zone.BOX]
    assert tcp_z(above[-1]) == pytest.approx(50 + zone.approach_mm, abs=1.5)
    assert tcp_z(down[-1]) == pytest.approx(50 - zone.grasp_depth_mm, abs=1.5)
    assert tcp_z(up[-1]) == pytest.approx(zone.lift_z_mm, abs=1.5)
    for q in np.vstack([down, up]):  # straight down and up, gripper vertical
        T = kin.fk_tcp(q)
        assert T[:2, 3] == pytest.approx((255, 0), abs=2)
        assert T[2, 0] == pytest.approx(-1, abs=0.01)
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open, 0.0]


def test_grasp_depth_is_clamped_to_the_zone_floor(arm, cfg):
    arm.pick(ArmPoint(255, 0, 12), Zone.BOX)
    assert tcp_z(arm.driver.paths[1][-1]) == pytest.approx(cfg.zones[Zone.BOX].z_floor_mm, abs=1.5)


def test_empty_gripper_is_reported(cfg):
    arm = Controller(FakeDriver(cfg.poses["look_bg"], 0.0), cfg.arm, cfg.poses, cfg.zones)
    assert arm.pick(ArmPoint(180, 210, 25), Zone.BACKGROUND).likely_empty


@pytest.mark.parametrize(
    "target, zone",
    [
        (ArmPoint(180, 210, 25), Zone.BOX),  # the background, but asked for the box
        (ArmPoint(255, 0, 400), Zone.BOX),  # too high to reach with the gripper down
    ],
)
def test_rejected_pick_does_not_move(arm, target, zone):
    with pytest.raises(TargetRejected):
        arm.pick(target, zone)
    assert arm.driver.paths == [] and arm.driver.gripper_cmds == []


def test_place_and_drop_go_to_their_poses_and_open(arm, cfg):
    arm.place_on_background()
    assert np.allclose(arm.driver.q, cfg.poses["place_bg"])
    arm.drop_to_bin(ColorClass.COLORED)
    assert any(np.allclose(p[-1], cfg.poses["bin_colored"]) for p in arm.driver.paths)
    assert np.allclose(arm.driver.q, cfg.poses["home"])  # back out of the bin
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open] * 2


def test_look_is_a_no_op_when_already_there(arm):
    arm.look(Zone.BACKGROUND)
    arm.look(Zone.BACKGROUND)
    assert len(arm.driver.paths) == 1


def test_hold_until_recover(arm, cfg):
    arm.hold()
    with pytest.raises(EStopped):
        arm.home()
    with pytest.raises(EStopped):
        arm.pick(ArmPoint(255, 0, 50), Zone.BOX)
    arm.recover()  # lift, open above the background, home
    assert np.allclose(arm.driver.q, cfg.poses["home"])
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open]


def test_shutdown_rests_before_disabling(arm, cfg):
    arm.start()
    arm.hold()  # Ctrl+C holds first, then shuts down
    arm.shutdown()
    assert np.allclose(arm.driver.q, cfg.poses["rest"]) and not arm.driver.connected


def test_ee_pose_is_link5_the_camera_does_not_turn_with_joint6(arm, cfg):
    assert np.allclose(arm.ee_pose(), kin.fk_link5(cfg.poses["look_box"]))
    q = np.array(cfg.poses["look_box"])
    turned = q + [0, 0, 0, 0, 0, 1.0]
    assert np.allclose(kin.fk_link5(turned), kin.fk_link5(q))
    assert np.allclose(kin.fk_link5(q) @ kin.T_LINK5_TCP0, kin.fk_tcp(q), atol=0.1)  # q6 ≈ 0


def test_missing_pose_is_a_clear_error(cfg):
    poses = {k: v for k, v in cfg.poses.items() if k != "place_bg"}
    with pytest.raises(ValueError, match="place_bg"):
        Controller(FakeDriver(np.zeros(6)), cfg.arm, poses, cfg.zones)


def test_in_polygon():
    square = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert in_polygon(5, 5, square) and not in_polygon(15, 5, square)


def test_real_driver_dry_run():
    """rebot_b601's simulated motors: the same control loop and safety checks as the real arm."""
    from sorter.arm.driver import RebotDriver

    d = RebotDriver(dry_run=True)
    d.connect()
    try:
        q0 = d.joints()
        q1 = q0 + np.radians([2, 2, 2, 0, 0, 0])
        d.execute(np.array([q0, q1]), 0.5)
        assert np.allclose(d.joints(), q1, atol=np.radians(0.5))
        d.stop()
        with pytest.raises(EStopped):
            d.execute(np.array([q1, q0]), 0.5)
        d.resume()
        d.execute(np.array([q1, q0]), 0.5)
    finally:
        d.disconnect()


def test_real_driver_clears_a_fault_with_the_torque_on():
    from sorter.arm.driver import RebotDriver
    from sorter.core.errors import ArmError

    d = RebotDriver(dry_run=True)
    d.connect()
    try:
        q0 = d.joints()
        d.arm.backend.blocked = {5}  # joint6 stuck
        with pytest.raises(ArmError, match="joint6"):
            d.execute(np.array([q0, q0 + np.radians([0, 0, 0, 0, 0, 40])]), 0.5)
        assert "joint6" in d.fault()
        d.arm.backend.blocked = set()
        d.clear_fault()
        assert d.fault() is None and d.arm.status()["torque_enabled"]
        q1 = d.joints() + np.radians([0, 0, 0, 0, 0, 5])
        d.execute(np.array([d.joints(), q1]), 0.5)
    finally:
        d.disconnect()


def test_shutdown_after_a_failed_start_does_nothing(cfg):
    arm = Controller(FakeDriver(np.zeros(6)), cfg.arm, cfg.poses, cfg.zones)
    arm.shutdown()  # never connected: no motion, no error
    assert arm.driver.paths == [] and not arm.driver.connected


def test_speed_changes_from_the_next_motion_clamped(arm, cfg):
    assert arm.speed_scale == cfg.arm.speed_scale
    arm.home()
    assert arm.set_speed_scale(0.3) == 0.3
    arm.go_to("look_bg")
    assert arm.driver.speeds == [cfg.arm.speed_scale, 0.3]
    assert arm.set_speed_scale(5.0) == arm.max_speed_scale
    assert arm.set_speed_scale(0.0) == MIN_SPEED_SCALE
    with pytest.raises(ValueError):
        arm.set_speed_scale(float("nan"))

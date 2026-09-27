"""The arm controller (block 5) on a recording driver, the committed rover rig, the real
driver in dry-run mode."""

import math

import numpy as np
import pytest

from sorter.arm import kinematics as kin
from sorter.arm.config import LOOK_POSES
from sorter.arm.controller import MIN_SPEED_SCALE, Controller, in_polygon, speed_ceiling
from sorter.core.config import load_config
from sorter.core.errors import ArmError, EStopped, TargetRejected
from sorter.core.types import ArmPoint, ColorClass, Zone
from sorter.sim.layout import check, zone_surfaces
from sorter.sim.rig import camera_pose

FLOOR_TARGET = ArmPoint(300, 0, -185)  # a sock on the floor in front of the rover


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
    return Controller(FakeDriver(cfg.poses["look_floor"]), cfg.arm, cfg.poses, cfg.zones)


def tcp_z(q) -> float:
    return float(kin.fk_tcp(q)[2, 3])


def test_committed_rig_fits_the_layout(cfg):
    assert check(cfg) == []  # every pick on the zone grids and every move between poses plans
    for zone, (rect, _) in zone_surfaces(cfg.sim).items():
        T = camera_pose(cfg.sim, cfg.poses[LOOK_POSES[zone]])
        assert T[:2, 3] == pytest.approx(rect.center_mm, abs=40)  # the camera over the zone
        assert T[2, 2] == pytest.approx(-1, abs=1e-3)  # looking straight down


def test_pick_goes_above_down_and_up(arm, cfg):
    result = arm.pick(FLOOR_TARGET, Zone.FLOOR)
    assert not result.likely_empty and result.gripper_opening == 0.3
    above, down, up = arm.driver.paths
    zone = cfg.zones[Zone.FLOOR]
    z = FLOOR_TARGET.z
    assert tcp_z(above[-1]) == pytest.approx(z + zone.approach_mm, abs=1.5)
    assert tcp_z(down[-1]) == pytest.approx(max(z - zone.grasp_depth_mm, zone.z_floor_mm), abs=1.5)
    assert tcp_z(up[-1]) == pytest.approx(zone.lift_z_mm, abs=1.5)
    for q in np.vstack([down, up]):  # straight down and up, gripper vertical
        T = kin.fk_tcp(q)
        assert T[:2, 3] == pytest.approx((FLOOR_TARGET.x, FLOOR_TARGET.y), abs=2)
        assert T[2, 0] == pytest.approx(-1, abs=0.01)
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open, 0.0]


def test_grasp_stops_above_the_floor(arm, cfg):
    arm.pick(ArmPoint(300, 0, -195), Zone.FLOOR)
    z_floor = cfg.zones[Zone.FLOOR].z_floor_mm
    assert z_floor > cfg.sim.layout.floor_z_mm
    assert tcp_z(arm.driver.paths[1][-1]) == pytest.approx(z_floor, abs=1.5)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 4, math.pi / 2])
def test_pick_turns_the_fingers_to_the_yaw(arm, yaw):
    arm.pick(FLOOR_TARGET, Zone.FLOOR, yaw)
    for q in np.vstack(arm.driver.paths[1:]):  # down and up keep the yaw
        err = (kin.tool_yaw(q) - yaw + math.pi / 2) % math.pi - math.pi / 2
        assert abs(err) < math.radians(3)


def test_pick_from_a_cargo_compartment(cfg):
    arm = Controller(FakeDriver(cfg.poses["look_cargo"]), cfg.arm, cfg.poses, cfg.zones)
    x, y = cfg.sim.layout.cargo.compartment(ColorClass.DARK).center_mm
    arm.pick(ArmPoint(x, y, cfg.sim.layout.cargo.floor_z_mm + 20), Zone.CARGO, math.pi / 2)
    assert tcp_z(arm.driver.paths[1][-1]) == pytest.approx(cfg.zones[Zone.CARGO].z_floor_mm, abs=2)


def test_empty_gripper_is_reported(cfg):
    arm = Controller(FakeDriver(cfg.poses["look_floor"], 0.0), cfg.arm, cfg.poses, cfg.zones)
    assert arm.pick(FLOOR_TARGET, Zone.FLOOR).likely_empty


@pytest.mark.parametrize(
    "target, zone",
    [
        (FLOOR_TARGET, Zone.CARGO),  # the floor, but asked for the cargo box
        (ArmPoint(300, 0, 400), Zone.FLOOR),  # too high to reach with the gripper down
        (ArmPoint(40, 0, -185), Zone.FLOOR),  # under the rover
    ],
)
def test_rejected_pick_does_not_move(arm, target, zone):
    with pytest.raises(TargetRejected):
        arm.pick(target, zone)
    assert arm.driver.paths == [] and arm.driver.gripper_cmds == []


def test_no_motion_into_the_rover_or_the_cargo_walls(arm, cfg):
    lay = cfg.sim.layout
    # the TCP right into the rover body, next to the arm's base
    with pytest.raises(TargetRejected, match="keep-out"):
        arm.move_tcp((60.0, -120.0, -60.0))
    # down onto a cargo divider
    rects = [lay.cargo.compartment(c) for c in lay.cargo.compartments]
    x = (rects[0].bounds()[1] + rects[1].bounds()[0]) / 2
    with pytest.raises(TargetRejected, match="keep-out"):
        arm.move_tcp((x, lay.cargo.center_mm[1], 30.0))
    assert arm.driver.paths == []
    below = kin.solve((300, 0, lay.floor_z_mm - 30), "down", None, z_min_mm=-1000)
    assert below is not None
    with pytest.raises(ArmError, match="below"):  # the gripper through the floor
        arm.move_joints(below)


def test_drops_go_to_their_poses_via_home_and_open(arm, cfg):
    arm.drop_to_cargo(ColorClass.COLORED)
    assert any(np.allclose(p[-1], cfg.poses["cargo_colored"]) for p in arm.driver.paths)
    assert np.allclose(arm.driver.q, cfg.poses["home"])  # back out, clear of the walls
    arm.drop_to_laundry(ColorClass.LIGHT)
    assert any(np.allclose(p[-1], cfg.poses["laundry_light"]) for p in arm.driver.paths)
    assert np.allclose(arm.driver.q, cfg.poses["home"])
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open] * 2


def test_look_is_a_no_op_when_already_there(arm):
    arm.look(Zone.CARGO)
    arm.look(Zone.CARGO)
    assert len(arm.driver.paths) == 1
    with pytest.raises(ValueError):
        arm.look(Zone.LAUNDRY)  # drops only, no look pose


def test_hold_until_recover(arm, cfg):
    arm.hold()
    with pytest.raises(EStopped):
        arm.home()
    with pytest.raises(EStopped):
        arm.pick(FLOOR_TARGET, Zone.FLOOR)
    arm.recover()  # lift, open over the floor, home
    assert np.allclose(arm.driver.q, cfg.poses["home"])
    assert arm.driver.gripper_cmds == [cfg.arm.gripper.open]


def test_shutdown_rests_before_disabling(arm, cfg):
    arm.start()
    arm.hold()  # Ctrl+C holds first, then shuts down
    arm.shutdown()
    assert np.allclose(arm.driver.q, cfg.poses["rest"]) and not arm.driver.connected


def test_ee_pose_is_link5_the_camera_does_not_turn_with_joint6(arm, cfg):
    assert np.allclose(arm.ee_pose(), kin.fk_link5(cfg.poses["look_floor"]))
    q = np.array(cfg.poses["look_floor"])
    turned = q + [0, 0, 0, 0, 0, 1.0]
    assert np.allclose(kin.fk_link5(turned), kin.fk_link5(q))
    assert np.allclose(kin.fk_link5(q) @ kin.T_LINK5_TCP0, kin.fk_tcp(q), atol=0.1)  # q6 ≈ 0


def test_missing_pose_is_a_clear_error(cfg):
    poses = {k: v for k, v in cfg.poses.items() if k != "cargo_dark"}
    with pytest.raises(ValueError, match="cargo_dark"):
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
    arm.go_to("look_cargo")
    assert arm.driver.speeds == [cfg.arm.speed_scale, 0.3]
    assert arm.set_speed_scale(5.0) == arm.max_speed_scale
    assert arm.set_speed_scale(0.0) == MIN_SPEED_SCALE
    with pytest.raises(ValueError):
        arm.set_speed_scale(float("nan"))


def test_max_speed_stays_under_the_motors_limit(cfg):
    assert 1.4 < speed_ceiling() < 1.5  # 1.5 rad/s over joint 4's 60 deg/s
    fast = cfg.arm.model_copy(update={"max_speed_scale": 5.0, "speed_scale": 5.0})
    arm = Controller(FakeDriver(np.zeros(6)), fast, cfg.poses, cfg.zones)
    assert arm.max_speed_scale == arm.speed_scale == speed_ceiling()

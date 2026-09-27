import math

import numpy as np
import pytest
from leo_sim.model import World
from leo_sim.rover import LeoSim


@pytest.fixture
def sim():
    s = LeoSim(World())  # empty 6 x 5 m room, rover at (-2, -0.9) facing +x
    yield s
    s.close()


def test_move_is_closed_loop_on_odometry(sim):
    x0, y0, _ = sim.pose()
    assert sim.move(1.0) == "done"
    x, y, yaw = sim.pose()
    assert math.hypot(x - x0, y - y0) == pytest.approx(1.0, abs=0.03)
    assert abs(yaw) < math.radians(3)
    assert sim.odom()[0] == pytest.approx(1.0, abs=0.02)


def test_backwards(sim):
    x0 = sim.pose()[0]
    assert sim.move(-0.3) == "done"
    assert sim.pose()[0] - x0 == pytest.approx(-0.3, abs=0.02)


@pytest.mark.parametrize("degrees", [90, -45, 180])
def test_turn_in_place(sim, degrees):
    x0, y0, yaw0 = sim.pose()
    assert sim.turn(math.radians(degrees)) == "done"
    x, y, yaw = sim.pose()
    turned = math.degrees(math.remainder(yaw - yaw0, 2 * math.pi))
    assert turned == pytest.approx(degrees if degrees != 180 else math.copysign(180, turned), abs=3)
    assert math.hypot(x - x0, y - y0) < 0.08  # skid-steer: the centre wanders a few cm


def test_angular_multiplier_matches_commanded_rate(sim):
    """ANGULAR_MULTIPLIER is calibrated so the yaw rate follows cmd_vel on this sim."""
    for _ in range(50):
        sim.set_cmd_vel(0.0, 0.6)
        sim.step(0.02)
    yaw0, t0 = sim.pose()[2], sim.time
    for _ in range(50):
        sim.set_cmd_vel(0.0, 0.6)
        sim.step(0.02)
    rate = math.remainder(sim.pose()[2] - yaw0, 2 * math.pi) / (sim.time - t0)
    assert rate == pytest.approx(0.6, rel=0.15)


def test_cmd_vel_expires_like_the_firmware(sim):
    sim.set_cmd_vel(0.3, 0.0)
    sim.step(1.5)  # 0.5 s of command, then the timeout and braking
    x = sim.pose()[0]
    sim.step(1.0)
    assert abs(sim.pose()[0] - x) < 0.01
    assert abs(sim.velocity()[0]) < 0.01


def test_limits_and_bad_input(sim):
    sim.set_cmd_vel(5.0, 5.0)
    assert sim.cmd == (0.4, pytest.approx(math.radians(57)))
    with pytest.raises(ValueError):
        sim.set_cmd_vel(float("nan"), 0.0)
    with pytest.raises(ValueError):
        sim.start_move(float("inf"))


def test_driving_into_a_wall_is_seen_as_a_bump(sim):
    sim.turn(math.pi)  # face the west wall, 1 m away
    motion = sim.start_move(2.0)
    while motion.outcome == "running":
        sim.step(0.02)
    assert "wall" in motion.bumped
    assert sim.pose()[0] > -2.9  # did not go through
    assert sim.tilt() < 10


def test_stop_interrupts_a_motion(sim):
    motion = sim.start_move(2.0)
    sim.step(0.5)
    sim.stop()
    assert motion.outcome == "stopped"


def test_render_cameras(sim):
    for camera in ("leo", "oak", "chase"):
        image = sim.render(camera, 160, 120)
        assert image.shape == (120, 160, 3) and image.dtype == np.uint8
        assert image.std() > 5  # not a blank frame
    height, _ = sim.map_scale(200)
    assert sim.render_map(200, goal=(0.0, 0.0)).shape == (height, 200, 3)

"""The rover navigation sim: model, firmware, OAK-D depth, commands, episodes, the algorithm."""

import math
import os

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "egl")

from sorter.nav.camera import OakD  # noqa: E402
from sorter.nav.commands import FRONT_M, Rover  # noqa: E402
from sorter.nav.config import NavConfig  # noqa: E402
from sorter.nav.episode import Episode  # noqa: E402
from sorter.nav.model import BASE_Z  # noqa: E402
from sorter.nav.scenario import PRESETS, make  # noqa: E402
from sorter.nav.sim import RoverSim  # noqa: E402


@pytest.fixture
def rover():
    cfg = NavConfig()
    sim = RoverSim(make("easy", 0), cfg)
    cam = OakD(sim, cfg.camera)
    yield Rover(sim, cam)
    cam.close()


def _angle(a: float) -> float:
    return (a + 180) % 360 - 180


def test_scenarios_are_seeded():
    for name in PRESETS:
        assert make(name, 3) == make(name, 3)
    assert make("easy", 1) != make("easy", 2)


def test_rover_rests_on_its_wheels(rover):
    assert rover.sim.data.xpos[rover.sim._base][2] == pytest.approx(BASE_Z, abs=0.003)


def test_forward_and_turn_match_the_truth(rover):
    sim = rover.sim
    x0, y0, _ = sim.true_pose()
    res = rover.forward(0.8)
    x1, y1, yaw1 = sim.true_pose()
    assert res.blocked is None
    assert math.hypot(x1 - x0, y1 - y0) == pytest.approx(0.8, abs=0.02)
    rover.turn(90)
    _, _, yaw2 = sim.true_pose()
    assert _angle(math.degrees(yaw2 - yaw1)) == pytest.approx(90, abs=3)


def test_speed_limits_and_cmd_timeout(rover):
    sim = rover.sim
    sim.set_cmd(5.0, 5.0)
    assert sim.cmd == (0.4, 1.0)
    sim.set_cmd(0.3, 0.0)
    for _ in range(int(1.5 / sim.dt)):  # no new twist: the firmware stops after 0.5 s
        sim.tick()
    assert abs(sim.odom.v) < 0.01


def test_depth_agrees_with_the_floor(rover):
    f = rover.look().frame
    u, v = 320, 380  # floor right in front
    z = f.depth_at(u, v)
    assert z is not None
    p_depth, p_ray = f.point(u, v, z), f.floor_point(u, v)
    assert np.linalg.norm(p_depth[:2] - p_ray[:2]) < 0.03
    assert abs(p_depth[2]) < 0.02
    # the top rows are outside the stereo pair's vertical FOV: no depth
    assert not f.depth_mm[:10].any()


def test_extended_disparity_min_range():
    cfg = NavConfig()
    cfg.camera.depth_mode = "normal"  # MinZ ~0.7 m
    sim = RoverSim(make("easy", 0), cfg)
    cam = OakD(sim, cfg.camera)
    f = cam.capture()
    valid = f.depth_mm[f.depth_mm > 0]
    assert valid.min() > 650
    cam.close()


def test_go_to_pixel_brings_the_sock_into_the_zone(rover):
    from sorter.nav.detect import SegDetector

    det = SegDetector(rover.camera).detect(rover.look().frame)[0]
    rover.go_to_pixel(det.u, det.v)
    sx, sy = rover.sim.sock_in_rover(0)
    assert 0.33 <= sx <= 0.53 and abs(sy) <= 0.10


def test_guard_stops_before_a_wall(rover):
    # face the nearest wall and drive far: the depth guard must stop before touching it
    res = rover.forward(8.0, 0.4)
    assert res.blocked and "obstacle" in res.blocked
    assert rover.sim.collisions == 0
    assert rover._obstacle_ahead() is not None


def test_episode_resumes_where_it_stopped(tmp_path):
    ep, _ = Episode.new(tmp_path / "a", "easy", 0)
    ep.record(ep.rover.turn(30))
    pose = ep.sim.true_pose()
    ep.close()
    ep2 = Episode.load(tmp_path / "a")
    assert np.allclose(ep2.sim.true_pose(), pose, atol=1e-6)
    ep2.record(ep2.rover.forward(0.3))
    assert ep2.score().commands == 2
    ep2.close()


def test_approach_succeeds_on_easy():
    from sorter.nav.controller import Approach
    from sorter.nav.detect import SegDetector

    ep = Episode(make("easy", 2), NavConfig())
    ok = Approach(ep.rover, SegDetector(ep.camera), ep.cfg.goal).run()
    s = ep.score()
    assert ok and s.success, s
    assert s.sock_x_m > FRONT_M
    ep.close()

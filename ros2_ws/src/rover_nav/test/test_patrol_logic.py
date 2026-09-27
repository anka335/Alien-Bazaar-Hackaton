import math

import pytest

from rover_nav.patrol_logic import (
    Drive,
    Dwell,
    PatrolLogic,
    PatrolParams,
    Pose2D,
    Turn,
    plan_lap,
    route_points,
    wrap,
)

DT = 0.05  # 20 Hz, like the node
START = Pose2D(0.0, 2.5, -math.pi / 2)  # the ROS track's start: (0, 2.5) facing south


class SimRover:
    def __init__(self, pose=START, drift=0.0):
        self.x, self.y, self.yaw, self.drift = pose.x, pose.y, pose.yaw, drift

    def move(self, v, w):
        self.yaw = wrap(self.yaw + (w + self.drift) * DT)
        self.x += v * math.cos(self.yaw) * DT
        self.y += v * math.sin(self.yaw) * DT

    @property
    def pose(self):
        return Pose2D(self.x, self.y, self.yaw)


def run(logic, rover, max_s=600.0, on_step=None):
    t = 0.0
    while not logic.done:
        logic.odom(rover.pose, t)
        v, w = logic.step(t)
        if on_step:
            on_step(logic, t, v, w, rover)
        rover.move(v, w)
        t += DT
        assert t < max_s, f"stuck: {logic.message}"
    return t


def test_square_points_1m_counter_clockwise():
    pts = route_points(PatrolParams(shape="square"), Pose2D(0, 0, 0))
    assert pts == [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]


def test_circle_points_on_a_1m_circle_centre_to_the_left():
    p = PatrolParams(shape="circle", circle_stops=8)
    pts = route_points(p, Pose2D(0, 0, 0))
    assert len(pts) == 8 and pts[-1] == (0.0, 0.0)
    for x, y in pts:
        assert math.hypot(x, y - 0.5) == pytest.approx(0.5)
    assert pts[1][1] > 0  # the second stop is to the left: counter-clockwise


def test_route_follows_the_start_pose():
    pts = route_points(PatrolParams(), START)
    assert pts[0] == pytest.approx((0.0, 1.5))  # 1 m ahead of a rover facing south


def test_plan_has_a_full_scan_at_every_stop():
    p = PatrolParams(shape="square", scan_step_deg=90, dwell_s=2.0)
    steps = plan_lap(p, Pose2D(0, 0, 0), 1)
    dwells = [s for s in steps if isinstance(s, Dwell)]
    assert len(dwells) == 5 * 4  # the start + 4 stops, 4 looks each
    assert sum(isinstance(s, Drive) for s in steps) == 4
    assert isinstance(steps[-1], Turn) and steps[-1].yaw == pytest.approx(0.0)


@pytest.mark.parametrize("shape", ["square", "circle"])
def test_patrol_returns_to_the_start(shape):
    logic, rover = PatrolLogic(PatrolParams(shape=shape, dwell_s=0.5)), SimRover()
    run(logic, rover)
    assert math.hypot(rover.x - START.x, rover.y - START.y) < 0.03
    assert abs(math.degrees(wrap(rover.yaw - START.yaw))) < 3.0


def test_square_drives_through_the_corners_and_scans_there():
    looks = []

    def record(logic, t, v, w, rover):
        if logic.scanning and (not looks or looks[-1][2] != logic.message):
            looks.append((rover.x, rover.y, logic.message, rover.yaw))

    logic, rover = PatrolLogic(PatrolParams(dwell_s=0.5)), SimRover()
    run(logic, rover, on_step=record)
    corners = [(0.0, 2.5), (0.0, 1.5), (1.0, 1.5), (1.0, 2.5), (0.0, 2.5)]  # south, then left
    assert len(looks) == 5 * 4
    for i, (cx, cy) in enumerate(corners):
        for x, y, _, _ in looks[4 * i : 4 * i + 4]:
            assert math.hypot(x - cx, y - cy) < 0.05
        yaws = sorted(
            round(math.degrees(wrap(yaw - looks[4 * i][3]))) % 360
            for *_, yaw in looks[4 * i : 4 * i + 4]
        )
        assert yaws == pytest.approx([0, 90, 180, 270], abs=3)  # all four directions


def test_heading_hold_with_drift():
    logic, rover = PatrolLogic(PatrolParams(dwell_s=0.2, scan_at_start=False)), SimRover(drift=0.05)
    run(logic, rover)
    assert math.hypot(rover.x - START.x, rover.y - START.y) < 0.05


def test_two_laps_scan_only_once_at_the_start():
    p = PatrolParams(laps=2, dwell_s=0.2)
    logic, rover = PatrolLogic(p), SimRover()
    starts = []
    run(logic, rover, on_step=lambda lg, *_: starts.append(lg.lap))
    assert max(starts) == 2
    assert sum(isinstance(s, Dwell) for s in plan_lap(p, START, 2)) == 4 * 4  # no start scan


def test_stale_odometry_and_limits():
    p = PatrolParams(dwell_s=0.2)
    logic, rover = PatrolLogic(p), SimRover()
    assert logic.step(0.0) == (0.0, 0.0)  # no odometry yet
    cmds = []
    run(logic, rover, on_step=lambda lg, t, v, w, r: cmds.append((v, w)))
    assert max(v for v, _ in cmds) <= p.speed + 1e-9
    assert max(abs(w) for _, w in cmds) <= p.turn_speed + 1e-9
    assert all(v >= 0 for v, _ in cmds)
    logic2 = PatrolLogic(p)
    logic2.odom(START, 0.0)
    logic2.step(0.0)
    assert logic2.step(2.0) == (0.0, 0.0) and logic2.message == "waiting for odometry"


def test_bad_shape_is_refused():
    with pytest.raises(ValueError):
        route_points(PatrolParams(shape="triangle"), START)
    with pytest.raises(ValueError):
        route_points(PatrolParams(shape="circle", circle_stops=2), START)

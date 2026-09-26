import math
import os
import sys

import numpy as np
import pytest

from cloth_task.spectacles_session import Session, quat_to_R, rotvec


def _rebot():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import config, kinematics

    return kinematics, config


K, C = _rebot()
Q0 = np.radians([10.0, 50.0, 70.0, -20.0, 10.0, 5.0])
IDENTITY = [0.0, 0.0, 0.0, 1.0]


def quat_about(axis, deg):
    a = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    h = math.radians(deg) / 2
    return [*(a * math.sin(h)), math.cos(h)]


def teleop(
    seq, *, arm=False, position=(0.0, 0.0, 0.0), orientation=IDENTITY, gripper=0.5, base=False
):
    return {
        "v": 1,
        "type": "teleop",
        "seq": seq,
        "timestamp": 1000.0 + seq,
        "base": {"engaged": base, "vx": 0.3 if base else 0.0, "wz": 0.2 if base else 0.0},
        "arm": {
            "engaged": arm,
            "position": list(position),
            "orientation": list(orientation),
            "gripper": gripper,
        },
    }


class Motor:
    def __init__(self):
        self.joints: list[np.ndarray] = []
        self.gripper: list[float] = []


@pytest.fixture
def rig():
    motor = Motor()
    s = Session(K, C.JOINT_LIMITS_RAD, motor.joints.append, motor.gripper.append)
    s.on_joint_state(Q0)
    return s, motor


def assert_pose(q, p, R):
    p_q, R_q = K.fk(q)
    assert np.linalg.norm(p_q - p) <= 1e-3
    assert math.degrees(np.linalg.norm(rotvec(R @ R_q.T))) <= 3.0


def test_rising_right_clutch_latches_the_measured_pose(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, gripper=0.7))

    assert len(motor.joints) == 1
    assert_pose(motor.joints[0], *K.fk(Q0))
    assert motor.gripper == [0.7]
    assert s.status() == {
        "v": 1, "type": "status", "echoSeq": 0, "base": "idle", "arm": "tracking", "fault": None
    }


def test_later_sample_solves_the_full_pose_from_the_latch(rig):
    s, motor = rig
    p0, R0 = K.fk(Q0)
    s.on_teleop(teleop(0, arm=True))
    s.on_joint_state(Q0 + np.radians([2.0, -1.0, 1.0, 0.0, 3.0, 0.0]))  # arm moved: latch stays
    delta = np.array([0.03, -0.02, 0.02])
    twist = quat_about([0.2, 0.3, 1.0], 15.0)
    s.on_teleop(teleop(1, arm=True, position=delta, orientation=twist, gripper=0.2))

    assert len(motor.joints) == 2
    assert_pose(motor.joints[1], p0 + delta, quat_to_R(np.array(twist)) @ R0)
    assert motor.gripper[-1] == 0.2
    assert np.all(motor.joints[1] >= C.JOINT_LIMITS_RAD[:, 0])
    assert np.all(motor.joints[1] <= C.JOINT_LIMITS_RAD[:, 1])


def test_a_large_step_publishes_the_full_solution_not_an_intermediate(rig):
    s, motor = rig
    p0, R0 = K.fk(Q0)
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[-0.08, 0.10, 0.06]))

    assert_pose(motor.joints[-1], p0 + np.array([-0.08, 0.10, 0.06]), R0)


@pytest.mark.parametrize("gripper, sent", [(-0.4, 0.0), (1.7, 1.0), (0.0, 0.0), (1.0, 1.0)])
def test_gripper_is_clamped_to_closed_through_open(rig, gripper, sent):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, gripper=gripper))
    assert motor.gripper == [sent]


def test_release_publishes_nothing_more_and_holds(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[0.01, 0.0, 0.0]))
    sent = (len(motor.joints), len(motor.gripper))
    s.on_teleop(teleop(2, arm=False, position=[0.05, 0.0, 0.0], gripper=0.0))
    s.on_teleop(teleop(3, arm=False))

    assert (len(motor.joints), len(motor.gripper)) == sent
    assert s.status()["arm"] == "holding"
    assert s.status()["echoSeq"] == 3


def test_next_clutch_relatches_where_the_arm_is(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    s.on_teleop(teleop(1, arm=True, position=[0.04, 0.0, 0.0]))
    s.on_teleop(teleop(2))
    q_now = Q0 + np.radians([5.0, 3.0, -4.0, 2.0, 0.0, 0.0])
    s.on_joint_state(q_now)
    s.on_teleop(teleop(3, arm=True))

    assert_pose(motor.joints[-1], *K.fk(q_now))


def test_unreachable_sample_publishes_nothing_and_stays_tracking(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True))
    sent = (len(motor.joints), len(motor.gripper))
    s.on_teleop(teleop(1, arm=True, position=[1.5, 0.0, 0.0], gripper=0.9))

    assert (len(motor.joints), len(motor.gripper)) == sent
    assert s.status()["arm"] == "tracking"
    assert s.status()["echoSeq"] == 1


def test_left_clutch_stays_base_idle_and_moves_nothing(rig):
    s, motor = rig
    s.on_teleop(teleop(0, base=True))

    assert s.status()["base"] == "idle"
    assert s.status()["arm"] == "holding"
    assert motor.joints == [] and motor.gripper == []


def test_left_clutch_does_not_stop_the_arm(rig):
    s, motor = rig
    s.on_teleop(teleop(0, arm=True, base=True))
    s.on_teleop(teleop(1, arm=True, base=True, position=[0.0, 0.02, 0.0]))

    assert len(motor.joints) == 2
    assert s.status()["base"] == "idle"
    assert s.status()["arm"] == "tracking"


def test_status_before_any_teleop():
    s = Session(K, C.JOINT_LIMITS_RAD, lambda q: None, lambda g: None)
    assert s.status() == {
        "v": 1, "type": "status", "echoSeq": None, "base": "idle", "arm": "holding", "fault": None
    }

"""HardwareBackend sends: only the position references that changed, plus one motor in turn."""

import numpy as np

from rebot_b601 import arm as A


class Motor:
    def __init__(self):
        self.writes, self.mit = [], []

    def robstride_write_param_f32(self, param, value):
        self.writes.append((param, value))

    def send_mit(self, *args):
        self.mit.append(args)


def _backend():
    b = A.HardwareBackend()
    b.arm_motors = [Motor() for _ in range(6)]
    b.gripper = Motor()
    return b


def test_a_tick_writes_only_the_changed_references_and_one_in_turn():
    b = _backend()
    q = np.zeros(6)
    b.send_arm(q)  # first: every motor
    assert [len(m.writes) for m in b.arm_motors] == [1] * 6
    b.send_arm(q)  # nothing changed: one motor in turn
    assert sum(len(m.writes) for m in b.arm_motors) == 7
    q2 = q.copy()
    q2[4] = 0.1
    b.send_arm(q2)  # the turn is joint 3 now
    assert b.arm_motors[4].writes[-1] == (A.HardwareBackend._LOC_REF, 0.1)
    assert sum(len(m.writes) for m in b.arm_motors) == 9  # joint 5 + the one in turn
    for m in b.arm_motors:
        assert all(p == A.HardwareBackend._LOC_REF for p, _ in m.writes)


def test_the_gripper_torque_is_resent_only_when_it_changes_or_goes_stale(monkeypatch):
    b = _backend()
    t = [100.0]
    monkeypatch.setattr(A.time, "monotonic", lambda: t[0])
    b.send_gripper_torque(0.5)
    b.send_gripper_torque(0.5)
    assert len(b.gripper.mit) == 1
    t[0] += A.HardwareBackend._RESEND_S + 0.01
    b.send_gripper_torque(0.5)
    b.send_gripper_torque(0.7)
    assert len(b.gripper.mit) == 3

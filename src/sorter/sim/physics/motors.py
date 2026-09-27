"""MujocoBackend: the physics scene's motors behind `rebot_b601.arm.Arm`, in place of the CAN bus.

It has the interface of `rebot_b601`'s `HardwareBackend`: joint position setpoints for motors
1–6, a torque for the gripper (motor 7), and feedback of position, speed, torque. So the real
driver, control loop and safety checks run unchanged; only the bus is simulated.
"""

from __future__ import annotations

import math

import numpy as np
from rebot_b601.arm import Measurement
from rebot_b601.kinematics import FINGER_TRAVEL_M

from sorter.sim.physics.model import GRIPPER_OPEN_MOTOR_DEG
from sorter.sim.physics.world import PhysicsWorld

_RAD_PER_M = (
    math.radians(GRIPPER_OPEN_MOTOR_DEG) / FINGER_TRAVEL_M
)  # motor 7 angle per finger travel


class MujocoBackend:
    simulated = True

    def __init__(self, world: PhysicsWorld):
        self.world = world

    def connect(self, enable: bool) -> Measurement:
        return self.read()

    def enable(self, q_hold: np.ndarray) -> None:
        self.world.command(q=q_hold, gripper_nm=0.0)
        self.world.set_enabled(True)

    def disable(self) -> None:
        self.world.set_enabled(False)

    def close(self) -> None:
        self.world.set_enabled(False)

    def send_arm(self, q_cmd: np.ndarray) -> None:
        self.world.command(q=q_cmd)

    def send_gripper_torque(self, tau: float) -> None:
        self.world.command(gripper_nm=tau)

    def read(self) -> Measurement:
        q, dq, tau, finger, finger_v = self.world.arm_state()
        return Measurement(q, dq, tau, np.full(7, 35.0), finger * _RAD_PER_M, finger_v * _RAD_PER_M)

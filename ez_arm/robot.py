"""`Robot`: one object to steer the reBot B601 in the 3D base frame, without ROS.

    r = Robot(simulate=True)            # or Robot() for the real arm on can0
    r.go(0.30, 0.00, 0.15, "down")      # TCP to xyz, gripper pointing down
    r.rel(dz=-0.05)                     # straight line, 5 cm down
    r.grip(1.0); r.grip(0.0)            # open / close
    r.pose("look")                      # a named joint pose (poses.yaml)
    r.snap()                            # RGB-D frame + the joint angles it was taken at

Wraps `rebot_b601.arm.Arm` (IK, joint limits, workspace box, table guard, min-jerk trajectories,
tracking-error / temperature aborts, stop). Base frame: +x forward, +y left, +z up, origin on the
base plate (= table top), metres. TCP = the fingertip centre.
"""

from __future__ import annotations

import math
import os

import numpy as np
import yaml
from rebot_b601 import kinematics as K
from rebot_b601.arm import Arm, ArmError

from ez_arm.frames import CameraFrame

POSES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.yaml")

__all__ = ["Robot", "ArmError"]


def load_poses(path: str = POSES_FILE) -> dict[str, list[float]]:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return (yaml.safe_load(f) or {}).get("poses", {})


def save_pose(name: str, deg, path: str = POSES_FILE) -> None:
    poses = load_poses(path)
    poses[name] = [round(float(a), 2) for a in deg]
    with open(path, "w") as f:
        yaml.safe_dump({"poses": poses}, f, default_flow_style=None, sort_keys=False)


class Robot:
    def __init__(self, simulate: bool = False, enable: bool = True, camera: bool = False, speed: float = 0.3):
        self.arm = Arm()
        self.arm.connect(enable=enable, simulate=simulate)
        self.speed = speed
        self.camframe = CameraFrame()
        self.cam = None
        if camera:
            from ez_arm.camera import RealSense

            self.cam = RealSense()

    # ---- state -----------------------------------------------------------
    @property
    def q(self) -> np.ndarray:
        return np.radians(self.arm.status()["joints_deg"])

    @property
    def tcp(self) -> np.ndarray:
        return K.fk(self.q)[0]

    def state(self) -> dict:
        s = self.arm.status()
        return {k: s[k] for k in ("joints_deg", "tcp_xyz_m", "approach_axis", "gripper_opening", "torque_enabled", "fault", "max_motor_temp_c")}

    # ---- motion ----------------------------------------------------------
    def go(self, x, y, z, approach="down", linear=False, speed=None) -> dict:
        return self.arm.move_to_xyz(x, y, z, approach=approach, linear=linear, speed_scale=speed or self.speed)

    def rel(self, dx=0.0, dy=0.0, dz=0.0, approach=None, speed=None) -> dict:
        """Straight-line relative move; keeps the current gripper direction unless told otherwise."""
        if approach is None:
            approach = K.approach_vector(self.q).tolist()
        return self.arm.move_relative(dx, dy, dz, approach=approach, linear=True, speed_scale=speed or self.speed)

    def joints(self, deg, speed=None) -> dict:
        return self.arm.move_joints(deg, speed_scale=speed or self.speed)

    def pose(self, name: str, speed=None) -> dict:
        poses = load_poses()
        if name not in poses:
            raise ArmError(f"unknown pose {name!r}; have {sorted(poses)}")
        return self.joints(poses[name], speed)

    def wrist_roll(self, deg: float, speed=None) -> dict:
        """Set joint6 (rotation about the gripper axis) keeping the rest."""
        q = np.degrees(self.q)
        q[5] = deg
        return self.joints(q, speed)

    def plan(self, x, y, z, approach="down", linear=False) -> dict:
        return self.arm.plan_xyz(x, y, z, approach=approach, linear=linear)

    def grip(self, opening: float) -> dict:
        return self.arm.set_gripper(opening)

    def home(self, speed=None) -> dict:
        return self.arm.home(speed or self.speed)

    def stop(self) -> dict:
        return self.arm.stop()

    def record(self, name: str) -> list[float]:
        deg = self.arm.status()["joints_deg"]
        save_pose(name, deg)
        return deg

    # ---- camera ----------------------------------------------------------
    def snap(self):
        """(RGBD, q) with the arm still."""
        if self.cam is None:
            raise ArmError("camera not opened (Robot(camera=True))")
        q0 = self.q
        rgbd = self.cam.grab()
        q1 = self.q
        if np.max(np.abs(q1 - q0)) > math.radians(0.5):
            raise ArmError("the arm moved while the frame was taken")
        return rgbd, q1

    def close(self, go_home: bool = True):
        try:
            if self.arm.connected:
                self.arm.disconnect(go_home=go_home)
        finally:
            if self.cam is not None:
                self.cam.close()

"""Observer: move to a zone's look pose, take a fresh frame, attach the camera pose."""

from __future__ import annotations

from sorter.core.protocols import ArmController, Calibration, Camera
from sorter.core.types import Observation, Zone


class Observer:
    def __init__(self, camera: Camera, arm: ArmController, calibration: Calibration):
        self.camera = camera
        self.arm = arm
        self.calibration = calibration

    def observe(self, zone: Zone) -> Observation:
        self.arm.look(zone)  # blocks until the arm is still
        frame = self.camera.fresh()
        return Observation(
            frame=frame,
            zone=zone,
            T_base_cam=self.calibration.cam_pose(self.arm.ee_pose()),
            joints=self.arm.joints(),
        )

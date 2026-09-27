"""Observer: move the camera (a zone's look pose, a named pose, or aimed at a point), take a fresh
frame, attach the camera pose."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from sorter.core.protocols import ArmController, Calibration, Camera
from sorter.core.types import ArmPoint, Observation, Zone


class Observer:
    def __init__(self, camera: Camera, arm: ArmController, calibration: Calibration):
        self.camera = camera
        self.arm = arm
        self.calibration = calibration

    def observe(self, zone: Zone, pose: str | None = None) -> Observation:
        """From the zone's look pose, or from the named pose `pose` (the zone is then what the
        view is of)."""
        if pose is None:
            self.arm.look(zone)  # blocks until the arm is still
        else:
            self.arm.go_to(pose)
        return self._capture(zone)

    def observe_point(
        self, zone: Zone, target: ArmPoint, heights_mm: Sequence[float]
    ) -> Observation | None:
        """The camera aimed at `target` from the first of `heights_mm` the arm reaches; None (no
        motion) if none."""
        T_link5_cam = self.calibration.cam_pose(np.eye(4))  # the hand-eye result
        h = self.arm.aim_camera(T_link5_cam, (target.x, target.y, target.z), heights_mm)
        return None if h is None else self._capture(zone)

    def _capture(self, zone: Zone) -> Observation:
        frame = self.camera.fresh()
        return Observation(
            frame=frame,
            zone=zone,
            T_base_cam=self.calibration.cam_pose(self.arm.ee_pose()),
            joints=self.arm.joints(),
        )

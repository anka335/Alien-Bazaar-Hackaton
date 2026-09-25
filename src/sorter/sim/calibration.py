"""SimCalibration: a fixed linear pixel ↔ arm mapping per zone."""

from __future__ import annotations

import numpy as np

from sorter.core.errors import CalibrationError
from sorter.core.types import ArmPoint, GraspPoint, Observation, PixelPoint, Pose
from sorter.sim.world import SimWorld


class SimCalibration:
    def __init__(self, world: SimWorld):
        self.world = world

    def cam_pose(self, ee_pose: Pose) -> Pose:
        return np.array(ee_pose, dtype=np.float64)  # the sim camera sits at the flange

    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint:
        if obs.T_base_cam is None:
            raise CalibrationError("observation has no camera pose")
        if point.depth_mm <= 0:
            raise CalibrationError(f"no depth at {point.px}")
        x, y = self.world.views[obs.zone].to_xy(point.px.u, point.px.v)
        return ArmPoint(x, y, float(obs.T_base_cam[2, 3]) - point.depth_mm)

    def to_pixel(self, obs: Observation, p: ArmPoint) -> PixelPoint | None:
        view = self.world.views[obs.zone]
        u, v = view.to_px(p.x, p.y)
        if not (0 <= u < view.width and 0 <= v < view.height):
            return None
        return PixelPoint(int(u), int(v))

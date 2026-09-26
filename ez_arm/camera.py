"""RealSense D435i: colour + depth aligned to colour, straight from librealsense (no ROS)."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np


@dataclass
class RGBD:
    color: np.ndarray  # HxWx3 BGR uint8
    depth_m: np.ndarray  # HxW float32, 0 = no data
    K: tuple[float, float, float, float]  # fx, fy, cx, cy
    t: float


class RealSense:
    def __init__(self, width=640, height=480, fps=15, serial: str | None = None):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        if serial:
            cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        prof = self.pipe.start(cfg)
        self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = (intr.fx, intr.fy, intr.ppx, intr.ppy)
        for _ in range(10):  # auto exposure settles
            self.pipe.wait_for_frames(5000)

    def grab(self, n_depth: int = 5) -> RGBD:
        """Latest colour frame; depth = per-pixel median over n_depth frames (less noise/holes)."""
        depths, color = [], None
        for _ in range(n_depth):
            fs = self.align.process(self.pipe.wait_for_frames(5000))
            d, c = fs.get_depth_frame(), fs.get_color_frame()
            if not d or not c:
                continue
            depths.append(np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale)
            color = np.asanyarray(c.get_data()).copy()
        stack = np.stack(depths)
        stack[stack == 0] = np.nan
        depth = np.nan_to_num(np.nanmedian(stack, axis=0), nan=0.0).astype(np.float32)
        return RGBD(color, depth, self.K, time.time())

    def close(self):
        self.pipe.stop()

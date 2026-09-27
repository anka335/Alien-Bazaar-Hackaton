"""The real Luxonis OAK-D through depthai v3: RGB 640x480 + stereo depth aligned to it.

Same `Frame` as the simulated camera: RGB uint8, depth uint16 mm (0 = none), the RGB camera's
intrinsics from the device's own calibration, `T_rover_cam` from `nav.camera` (the mount on the
rover). The pipeline mirrors the sim's settings: extended disparity (MinZ ~0.2 m), 400P mono
pair, depth aligned to the RGB (CAM_A), RGB undistorted, frames paired by a Sync node.

Install on the machine the camera is plugged into: `uv sync --extra nav-hw` (depthai >= 3).
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from datetime import timedelta

import cv2
import numpy as np

from sorter.nav.camera import Frame, Intrinsics, _mount
from sorter.nav.config import OakDConfig, RealConfig

log = logging.getLogger(__name__)


class OakDError(RuntimeError):
    pass


class RealOakD:
    """`capture()` returns the newest RGB-D pair (waits for one newer than the last)."""

    def __init__(self, cam_cfg: OakDConfig, real_cfg: RealConfig):
        try:
            import depthai as dai
        except ImportError:
            raise OakDError("depthai is not installed: uv sync --extra nav-hw") from None
        self.dai = dai
        self.cfg = cam_cfg
        w, h, fps = cam_cfg.width, cam_cfg.height, real_cfg.oakd_fps
        try:
            device = (
                dai.Device(dai.DeviceInfo(real_cfg.oakd_device))
                if real_cfg.oakd_device
                else dai.Device()
            )
        except RuntimeError as e:
            raise OakDError(f"no OAK-D found: {e}") from None
        p = dai.Pipeline(device)
        rgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = p.create(dai.node.StereoDepth)
        sync = p.create(dai.node.Sync)
        stereo.setExtendedDisparity(cam_cfg.depth_mode == "extended")
        stereo.setLeftRightCheck(True)
        sync.setSyncThreshold(timedelta(seconds=1 / (2 * fps)))
        mono = (640, 400) if cam_cfg.depth_mode == "extended" else (1280, 800)
        rgb_out = rgb.requestOutput(
            size=(w, h), type=dai.ImgFrame.Type.BGR888i, fps=fps, enableUndistortion=True
        )
        left.requestOutput(size=mono, fps=fps).link(stereo.left)
        right.requestOutput(size=mono, fps=fps).link(stereo.right)
        rgb_out.link(sync.inputs["rgb"])
        rgb_out.link(stereo.inputAlignTo)
        stereo.depth.link(sync.inputs["depth"])
        self._queue = sync.out.createOutputQueue(maxSize=2, blocking=False)
        calib = device.readCalibration()
        m = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, w, h))
        self.K = Intrinsics(float(m[0, 0]), float(m[1, 1]), float(m[0, 2]), float(m[1, 2]), w, h)
        self.T_rover_cam = _mount(cam_cfg)
        self.pipeline = p
        p.start()
        self._index = 0
        self._t0 = time.monotonic()
        self._lock = threading.Lock()
        self._last_seq = -1

    def capture(self, timeout_s: float = 2.0) -> Frame:
        t_end = time.monotonic() + timeout_s
        while time.monotonic() < t_end:
            group = self._queue.tryGet()
            if group is None:
                time.sleep(0.005)
                continue
            rgb_msg, depth_msg = group["rgb"], group["depth"]
            if rgb_msg.getSequenceNum() == self._last_seq:
                continue
            self._last_seq = rgb_msg.getSequenceNum()
            bgr = rgb_msg.getCvFrame()
            depth = depth_msg.getFrame().astype(np.uint16)
            if depth.shape != bgr.shape[:2]:  # aligned depth normally matches; be safe
                depth = cv2.resize(
                    depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST
                )
            depth[depth > int(self.cfg.max_range_m * 1000)] = 0
            with self._lock:
                self._index += 1
                idx = self._index
            return Frame(
                idx,
                time.monotonic() - self._t0,
                cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
                depth,
                self.K,
                self.T_rover_cam,
            )
        raise OakDError(f"no frame from the OAK-D within {timeout_s} s")

    def close(self) -> None:
        with contextlib.suppress(Exception):  # closing anyway
            self.pipeline.stop()

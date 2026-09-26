"""RealSenseCamera: the D435i on the wrist, through `pyrealsense2` (block 1).

A capture thread keeps only the newest frame. Depth is aligned to color and converted to mm
(Z along the optical axis). After warm-up, auto exposure and white balance are locked so colors
stay comparable between frames. `fresh()` waits for a frame that arrived at least one frame
period after the call, so its exposure started after the call.

Needs `uv sync --extra camera` (pyrealsense2; on macOS the pyrealsense2-macosx
build, run as root: the macOS camera driver is stopped while the camera runs, see `macos.py`).
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time

import numpy as np

from sorter.camera.config import CameraConfig
from sorter.camera.macos import UvcAssistantFreeze
from sorter.core.errors import CameraError
from sorter.core.types import Frame, Intrinsics

log = logging.getLogger(__name__)


class RealSenseCamera:
    def __init__(self, cfg: CameraConfig):
        try:
            import pyrealsense2 as rs
        except ImportError as e:
            raise CameraError("pyrealsense2 is not installed: uv sync --extra camera") from e
        self.rs = rs
        self.cfg = cfg
        self._pipeline = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._cv = threading.Condition()
        self._latest: Frame | None = None
        self._error: str | None = None
        self._seq = 0
        self.intrinsics: Intrinsics | None = None
        self.serial = cfg.serial
        self._uvc = UvcAssistantFreeze()

    # --- Camera ---

    def start(self) -> None:
        if self._thread is not None:
            return
        cfg = self.cfg
        self._uvc.freeze()
        attempts = max(1, cfg.start_attempts)
        for attempt in range(1, attempts + 1):
            try:
                self._open()
                break
            except RuntimeError as e:
                if self._pipeline is not None:
                    with contextlib.suppress(RuntimeError):
                        self._pipeline.stop()
                    self._pipeline = None
                if attempt == attempts:
                    self._uvc.resume()
                    raise CameraError(f"RealSense did not start: {e}") from e
                log.warning(
                    "camera: start attempt %d/%d failed (%s), retrying", attempt, attempts, e
                )
                time.sleep(1.0)
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="camera", daemon=True)
        self._thread.start()
        log.info(
            "RealSense %s started, %dx%d @ %d fps", self.serial, cfg.width, cfg.height, cfg.fps
        )

    def _open(self) -> None:
        """Start the pipeline and warm up. On macOS the first frames sometimes never come."""
        rs, cfg = self.rs, self.cfg
        pipeline = rs.pipeline()
        rc = rs.config()
        if cfg.serial:
            rc.enable_device(cfg.serial)
        rc.enable_stream(rs.stream.color, cfg.width, cfg.height, rs.format.bgr8, cfg.fps)
        rc.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)
        profile = pipeline.start(rc)
        self._pipeline = pipeline
        device = profile.get_device()
        self.serial = device.get_info(rs.camera_info.serial_number)
        self._depth_mm = device.first_depth_sensor().get_depth_scale() * 1000.0
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsics = Intrinsics(i.fx, i.fy, i.ppx, i.ppy, i.width, i.height, tuple(i.coeffs))
        self._align = rs.align(rs.stream.color)
        self._color_sensor = device.first_color_sensor()
        self._warmup()

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        self._uvc.resume()

    def latest(self) -> Frame | None:
        with self._cv:
            return self._latest

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        t = time.monotonic() + 1.0 / self.cfg.fps  # the exposure of this frame started after t0
        with self._cv:
            ok = self._cv.wait_for(
                lambda: (
                    self._error is not None
                    or (self._latest is not None and self._latest.timestamp >= t)
                ),
                timeout_s,
            )
            if self._error:
                raise CameraError(self._error)
            if not ok:
                raise CameraError(f"no fresh frame within {timeout_s} s")
            return self._latest

    # --- capture ---

    def _warmup(self) -> None:
        rs, cfg, s = self.rs, self.cfg, self._color_sensor
        for _ in range(cfg.warmup_frames):
            self._pipeline.wait_for_frames(int(cfg.timeout_s * 1000))
        if not cfg.lock_exposure:
            return
        for opt, fixed in (
            (rs.option.exposure, cfg.exposure_us),
            (rs.option.white_balance, cfg.white_balance_k),
        ):
            value = fixed if fixed is not None else s.get_option(opt)  # the settled value
            auto = (
                rs.option.enable_auto_exposure
                if opt == rs.option.exposure
                else (rs.option.enable_auto_white_balance)
            )
            if s.supports(auto):
                s.set_option(auto, 0)
            if s.supports(opt):
                s.set_option(opt, value)
        log.info("camera: exposure and white balance locked")

    def _loop(self) -> None:
        while self._running.is_set():
            try:
                frames = self._pipeline.wait_for_frames(int(self.cfg.timeout_s * 1000))
            except RuntimeError as e:
                with self._cv:
                    self._error = f"camera lost: {e}"
                    self._cv.notify_all()
                return
            t = time.monotonic()
            aligned = self._align.process(frames)
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                continue
            depth_mm = np.asanyarray(depth.get_data()).astype(np.float32) * self._depth_mm
            frame = Frame(
                color=np.asanyarray(color.get_data()).copy(),
                depth_mm=np.clip(np.rint(depth_mm), 0, 65535).astype(np.uint16),
                intrinsics=self.intrinsics,
                timestamp=t,
                seq=self._seq,
            )
            self._seq += 1
            with self._cv:
                self._latest = frame
                self._cv.notify_all()

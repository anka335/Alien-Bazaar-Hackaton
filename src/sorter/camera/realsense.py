"""RealSenseCamera: the D435i on the wrist, through `pyrealsense2` (block 1).

A capture thread keeps only the newest frame. Depth is aligned to color and converted to mm
(Z along the optical axis). After warm-up, auto exposure and white balance are locked at what they
settled on (found by search, see `lock.py`) so colors stay comparable between frames. `fresh()`
waits for a frame that arrived at least one frame period after the call, so its exposure started
after the call.

Needs `uv sync --extra camera` (pyrealsense2; on macOS the pyrealsense2-macosx
build, run as root: the macOS camera driver is stopped while the camera runs, see `macos.py`).
"""

from __future__ import annotations

import contextlib
import logging
import math
import threading
import time

import numpy as np

from sorter.camera import lock
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
        rs, cfg = self.rs, self.cfg
        # The device keeps the manual mode of the last run, so the auto modes are turned on first.
        self._set(rs.option.enable_auto_exposure, 1)
        self._set(rs.option.enable_auto_white_balance, 1)
        ref = self._color(cfg.warmup_frames)
        if not cfg.lock_exposure:
            return
        wb = self._lock(
            rs.option.enable_auto_white_balance,
            rs.option.white_balance,
            cfg.white_balance_k,
            lock.red_blue,
            lock.red_blue(ref),
            geometric=False,
        )
        self._set(rs.option.gain, cfg.gain if cfg.gain is not None else self._default_gain())
        cap = self._range(rs.option.exposure)
        cap = (cap[0], min(cap[1], 10000 / cfg.fps))  # 100 µs units, no longer than a frame
        exposure = self._lock(
            rs.option.enable_auto_exposure,
            rs.option.exposure,
            cfg.exposure,
            lock.luma,
            lock.luma(ref),
            limits=cap,
        )
        gain = self._get(rs.option.gain)
        if cfg.exposure is None and cfg.gain is None and exposure >= 0.95 * cap[1]:
            # Too dark even at the longest exposure a frame allows: raise the gain too.
            limits = (gain, self._range(rs.option.gain)[1])
            gain = self._lock(None, rs.option.gain, None, lock.luma, lock.luma(ref), limits=limits)
        log.info(
            "camera: locked exposure %.0f (x100 µs), gain %.0f, white balance %.0f K",
            exposure,
            gain,
            wb,
        )

    def _lock(self, auto, opt, fixed, measure, target, geometric=True, limits=None) -> float:
        """Turn `auto` off, set `opt` to `fixed` or to where `measure` matches the auto frame."""
        s = self._color_sensor
        if auto is not None:
            self._set(auto, 0)
        if not s.supports(opt):
            return math.nan
        if fixed is None:
            lo, hi = limits or self._range(opt)

            def probe(v: float) -> float:
                s.set_option(opt, round(v))  # integer options on the D435i color sensor
                return measure(self._color(self.cfg.settle_frames))

            fixed = lock.bisect(probe, max(lo, 1), hi, target, self.cfg.lock_steps, geometric)
        s.set_option(opt, round(fixed))
        return self._get(opt)

    def _color(self, n: int) -> np.ndarray:
        """The color image of the n-th next frame (earlier ones may predate an option change)."""
        for _ in range(n - 1):
            self._pipeline.wait_for_frames(int(self.cfg.timeout_s * 1000))
        frames = self._pipeline.wait_for_frames(int(self.cfg.timeout_s * 1000))
        return np.asanyarray(frames.get_color_frame().get_data()).copy()

    def _set(self, opt, value: float) -> None:
        if self._color_sensor.supports(opt):
            self._color_sensor.set_option(opt, value)

    def _get(self, opt) -> float:
        s = self._color_sensor
        return s.get_option(opt) if s.supports(opt) else math.nan

    def _range(self, opt) -> tuple[float, float]:
        r = self._color_sensor.get_option_range(opt)
        return r.min, r.max

    def _default_gain(self) -> float:
        return self._color_sensor.get_option_range(self.rs.option.gain).default

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

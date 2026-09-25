"""UVC camera (the SO-101 wrist camera) read with OpenCV in a capture thread. RGB only (D-014)."""

from __future__ import annotations

import json
import logging
import math
import multiprocessing as mp
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from multiprocessing import shared_memory
from typing import Any

import cv2
import numpy as np

from sorter.camera.config import CameraConfig
from sorter.core.errors import CameraError
from sorter.core.types import Frame, Intrinsics

log = logging.getLogger(__name__)

OpenFn = Callable[[CameraConfig], Any]  # returns an object with read() / release() like cv2


def mac_camera_names() -> list[str] | None:
    """Names of the cameras macOS sees, or None if they can't be listed."""
    try:
        out = subprocess.run(
            ["system_profiler", "SPCameraDataType", "-json"],
            capture_output=True,
            text=True,
            timeout=10,
            check=True,
        ).stdout
        return [c.get("_name", "") for c in json.loads(out).get("SPCameraDataType", [])]
    except (OSError, subprocess.SubprocessError, ValueError):
        return None


def _check_present(cfg: CameraConfig) -> None:
    if sys.platform == "darwin" and cfg.name:
        # With the wrist camera unplugged, the index points at the built-in camera instead
        names = mac_camera_names()
        if names is not None and not any(cfg.name in n for n in names):
            raise CameraError(f"no {cfg.name!r} among the cameras {names}: wrist camera unplugged?")


def _cv_capture(cfg: CameraConfig) -> cv2.VideoCapture:
    api = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    cap = cv2.VideoCapture(cfg.index, api)
    if not cap.isOpened():
        raise CameraError(f"camera {cfg.index} can't be opened (plugged in? camera permission?)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
    cap.set(cv2.CAP_PROP_FPS, cfg.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # ignored by some backends; the thread drains it anyway
    return cap


def _capture_process(cfg: CameraConfig, shm_name: str, seq, stop) -> None:
    """Child process: read frames into shared memory (resized to the configured size)."""
    shm = shared_memory.SharedMemory(name=shm_name)
    try:
        buf = np.ndarray((cfg.height, cfg.width, 3), np.uint8, shm.buf)
        cap = _cv_capture(cfg)
        while not stop.is_set():
            ok, img = cap.read()
            if not ok or img is None:
                break  # the parent notices the silence and starts a new process
            if img.shape[:2] != (cfg.height, cfg.width):
                img = cv2.resize(img, (cfg.width, cfg.height))
            with seq.get_lock():
                buf[:] = img
                seq.value += 1
        cap.release()
    finally:
        shm.close()


class ProcessCapture:
    """OpenCV capture in a child process. After a USB drop (the wrist cable flexes), macOS
    OpenCV can't reopen the device in the same process, but a new process can: `release()`
    ends the child, and the camera thread opens a new one."""

    def __init__(self, cfg: CameraConfig, stall_s: float = 2.0):
        _check_present(cfg)
        self.cfg, self.stall_s = cfg, stall_s
        ctx = mp.get_context("spawn")
        self._shm = shared_memory.SharedMemory(create=True, size=cfg.height * cfg.width * 3)
        self._buf = np.ndarray((cfg.height, cfg.width, 3), np.uint8, self._shm.buf)
        self._seq = ctx.Value("q", 0)
        self._stop = ctx.Event()
        self._proc = ctx.Process(
            target=_capture_process,
            args=(cfg, self._shm.name, self._seq, self._stop),
            name="camera-capture",
            daemon=True,
        )
        self._proc.start()
        self._last = 0
        self._first_timeout_s = 10.0  # spawning + opening the device

    def read(self) -> tuple[bool, np.ndarray | None]:
        deadline = time.monotonic() + (self._first_timeout_s if self._last == 0 else self.stall_s)
        while time.monotonic() < deadline:
            with self._seq.get_lock():
                if self._seq.value != self._last:
                    self._last = self._seq.value
                    return True, self._buf.copy()
            if not self._proc.is_alive():
                return False, None
            time.sleep(0.005)
        return False, None

    def release(self) -> None:
        self._stop.set()
        self._proc.join(timeout=2)
        if self._proc.is_alive():
            self._proc.kill()
            self._proc.join(timeout=2)
        self._shm.close()
        self._shm.unlink()


def open_capture(cfg: CameraConfig) -> ProcessCapture:
    return ProcessCapture(cfg)


def approx_intrinsics(width: int, height: int, hfov_deg: float) -> Intrinsics:
    """Pinhole intrinsics from the field of view. Nothing here needs them to be exact: pixel →
    arm goes through the per-zone homography (block 2)."""
    f = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    return Intrinsics(f, f, width / 2, height / 2, width, height)


class UvcCamera:
    """`Camera` on an OpenCV capture. The thread reads continuously, so the driver buffer never
    holds stale frames; `fresh()` also skips `fresh_skip_frames` frames after the call."""

    def __init__(self, cfg: CameraConfig, open_fn: OpenFn = open_capture):
        self.cfg = cfg
        self._open = open_fn
        self._cond = threading.Condition()
        self._frame: Frame | None = None
        self._seq = 0
        self._error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._intrinsics: Intrinsics | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()
        try:
            self.fresh(timeout_s=max(5.0, self.cfg.warmup_s + 3.0))
        except CameraError as e:
            log.error("camera: %s", e)  # the loop reports it on the first observation
            return
        time.sleep(self.cfg.warmup_s)
        f = self._frame
        assert f is not None
        log.info("camera %d: %dx%d", self.cfg.index, f.color.shape[1], f.color.shape[0])

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def latest(self) -> Frame | None:
        return self._frame

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        deadline = time.monotonic() + timeout_s
        with self._cond:
            want = self._seq + self.cfg.fresh_skip_frames + 1
            while self._seq < want:
                left = deadline - time.monotonic()
                if left <= 0:
                    why = f": {self._error}" if self._error else ""
                    raise CameraError(f"no fresh frame within {timeout_s:.1f} s{why}")
                self._cond.wait(left)
            assert self._frame is not None
            return self._frame

    def _run(self) -> None:
        cap = None
        while not self._stop.is_set():
            if cap is None:
                try:
                    cap = self._open(self.cfg)
                    self._error = None
                except Exception as e:
                    self._error = str(e)
                    log.warning("camera: %s; retrying", e)
                    self._stop.wait(self.cfg.reconnect_s)
                    continue
            ok, img = cap.read()
            if not ok or img is None:
                self._error = "device returned no frame"
                log.warning("camera %d lost, reopening", self.cfg.index)
                cap.release()
                cap = None
                self._stop.wait(self.cfg.reconnect_s)
                continue
            t = time.monotonic()
            h, w = img.shape[:2]
            k = self._intrinsics
            if k is None or (k.width, k.height) != (w, h):
                k = self._intrinsics = approx_intrinsics(w, h, self.cfg.hfov_deg)
            with self._cond:
                self._seq += 1
                self._frame = Frame(
                    color=img, depth_mm=None, intrinsics=k, timestamp=t, seq=self._seq
                )
                self._cond.notify_all()
        if cap is not None:
            cap.release()

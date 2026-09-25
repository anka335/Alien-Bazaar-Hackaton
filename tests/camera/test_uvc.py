import threading
import time

import numpy as np
import pytest

from sorter.camera.config import CameraConfig
from sorter.camera.uvc import UvcCamera
from sorter.core.errors import CameraError


class FakeCapture:
    """Frames whose pixel value is the frame number, at ~200 fps."""

    def __init__(self, fail_after: int | None = None):
        self.n = 0
        self.fail_after = fail_after
        self.released = False

    def read(self):
        time.sleep(0.005)
        self.n += 1
        if self.fail_after is not None and self.n > self.fail_after:
            return False, None
        return True, np.full((48, 64, 3), self.n % 256, np.uint8)

    def release(self):
        self.released = True


def _cam(**kw):
    cfg = CameraConfig(warmup_s=0.0, reconnect_s=0.01, **kw)
    caps = []

    def open_fn(_cfg):
        caps.append(FakeCapture())
        return caps[-1]

    return UvcCamera(cfg, open_fn), caps


def test_fresh_returns_a_frame_captured_after_the_call():
    cam, _ = _cam(fresh_skip_frames=2)
    cam.start()
    try:
        before = cam.latest()
        assert before is not None and before.depth_mm is None
        t = time.monotonic()
        f = cam.fresh()
        assert f.seq >= before.seq + 3
        assert f.timestamp > t
        assert f.intrinsics.width == 64 and f.intrinsics.height == 48
    finally:
        cam.close()


def test_fresh_times_out_without_frames():
    def open_fn(_cfg):
        raise CameraError("no device")

    cam = UvcCamera(CameraConfig(warmup_s=0.0, reconnect_s=0.01), open_fn)
    thread = threading.Thread(target=cam.start)
    thread.start()  # start() waits for a first frame and logs the error
    thread.join(timeout=10)
    try:
        with pytest.raises(CameraError, match="no device"):
            cam.fresh(timeout_s=0.1)
    finally:
        cam.close()


def test_reopens_a_lost_device():
    cfg = CameraConfig(warmup_s=0.0, reconnect_s=0.01)
    caps = []

    def open_fn(_cfg):
        caps.append(FakeCapture(fail_after=5 if not caps else None))
        return caps[-1]

    cam = UvcCamera(cfg, open_fn)
    cam.start()
    try:
        deadline = time.monotonic() + 2
        while len(caps) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(caps) == 2 and caps[0].released
        assert cam.fresh().seq > 5
    finally:
        cam.close()

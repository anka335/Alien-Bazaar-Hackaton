"""Camera sources for the real rover besides a local OAK-D (`real_oakd`).

- `RosCamera`: the OAK-D's ROS driver on the rover, over rosbridge: RGB from
  `sensor_msgs/CompressedImage` (JPEG), depth from `compressedDepth` (16-bit PNG, mm), the
  intrinsics from `camera_info`. The RGB is scaled to `nav.camera.width` (keeping its aspect);
  the depth (aligned to the RGB by the driver) is resized to it.
- `NoCamera`: no camera at all: gray frames saying so, no depth (the obstacle guard sees
  nothing). For driving by the buttons only.

`open_camera(cfg, bridge)` picks one by `nav.real.camera` (`auto`: rosbridge, then a local
OAK-D, then none).
"""

from __future__ import annotations

import base64
import logging
import math
import threading
import time

import cv2
import numpy as np

from sorter.nav.camera import Frame, Intrinsics, _mount
from sorter.nav.config import NavConfig

log = logging.getLogger(__name__)

COMPRESSED_DEPTH_HEADER = 12  # ROS 2 compressed_depth_image_transport: format + 2 floats


class CameraUnavailable(RuntimeError):
    pass


def _bytes(data) -> bytes:
    """rosbridge sends uint8[] as base64 (ROS 2) or as a list of ints."""
    if isinstance(data, str):
        return base64.b64decode(data)
    return bytes(data)


def decode_depth(msg: dict) -> np.ndarray | None:
    """A compressedDepth message → uint16 mm (0 = none)."""
    raw = _bytes(msg["data"])
    fmt = msg.get("format", "")
    for skip in (COMPRESSED_DEPTH_HEADER, 0) if "compressedDepth" in fmt else (0,):
        img = cv2.imdecode(np.frombuffer(raw[skip:], np.uint8), cv2.IMREAD_UNCHANGED)
        if img is not None:
            if img.dtype == np.float32:  # 32FC1 in metres
                img = np.nan_to_num(img * 1000)
            return img.astype(np.uint16)
    return None


class RosCamera:
    def __init__(self, cfg: NavConfig, bridge, wait_s: float | None = None):
        self.cfg = cfg
        r = cfg.real
        self.bridge = bridge
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._rgb: np.ndarray | None = None
        self._src_size = (0, 0)
        self._rgb_seq = 0
        self._depth: np.ndarray | None = None
        self._info: dict | None = None
        self._taken = 0
        self._index = 0
        self._t0 = time.monotonic()
        self.T_rover_cam = _mount(cfg.camera)
        bridge.subscribe(r.rgb_topic, "sensor_msgs/msg/CompressedImage", self._on_rgb)
        bridge.subscribe(r.depth_topic, "sensor_msgs/msg/CompressedImage", self._on_depth)
        bridge.subscribe(r.camera_info_topic, "sensor_msgs/msg/CameraInfo", self._on_info)
        wait_s = r.camera_timeout_s if wait_s is None else wait_s
        with self._cv:
            if not self._cv.wait_for(lambda: self._rgb is not None, wait_s):
                raise CameraUnavailable(
                    f"no {r.rgb_topic} over rosbridge within {wait_s} s (is the OAK-D's ROS "
                    "driver running on the rover?)"
                )
            self.K = self._intrinsics()

    def _on_rgb(self, msg: dict) -> None:
        img = cv2.imdecode(np.frombuffer(_bytes(msg["data"]), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        w = self.cfg.camera.width
        h = round(img.shape[0] * w / img.shape[1])
        small = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        with self._cv:
            self._src_size = (img.shape[1], img.shape[0])
            self._rgb = rgb
            self._rgb_seq += 1
            self._cv.notify_all()

    def _on_depth(self, msg: dict) -> None:
        d = decode_depth(msg)
        if d is not None:
            with self._lock:
                self._depth = d

    def _on_info(self, msg: dict) -> None:
        with self._lock:
            self._info = msg

    def _intrinsics(self) -> Intrinsics:
        """The RGB intrinsics at the scaled size (call with the lock held)."""
        h, w = self._rgb.shape[:2]
        info = self._info
        k = (info.get("k") or info.get("K")) if info else None
        if k:
            sw = info.get("width") or self._src_size[0]
            sh = info.get("height") or self._src_size[1]
            sx, sy = w / sw, h / sh
            return Intrinsics(k[0] * sx, k[4] * sy, k[2] * sx, k[5] * sy, w, h)
        f = w / 2 / math.tan(math.radians(self.cfg.camera.rgb_hfov_deg) / 2)  # nominal
        return Intrinsics(f, f, w / 2, h / 2, w, h)

    def capture(self, timeout_s: float = 2.0) -> Frame:
        """The newest RGB frame newer than the last one returned (a frame from before a
        motion would mislead), with the newest depth."""
        with self._cv:
            if not self._cv.wait_for(lambda: self._rgb_seq > self._taken, timeout_s):
                raise CameraUnavailable(f"no new frame on {self.cfg.real.rgb_topic}")
            self._taken = self._rgb_seq
            rgb, depth = self._rgb.copy(), self._depth
            self.K = self._intrinsics()
        h, w = rgb.shape[:2]
        if depth is None:
            depth = np.zeros((h, w), np.uint16)
        elif depth.shape != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)
        self._index += 1
        return Frame(self._index, time.monotonic() - self._t0, rgb, depth, self.K, self.T_rover_cam)

    def close(self) -> None:
        pass  # the bridge belongs to the session


class NoCamera:
    """Drive without a camera: gray frames saying so, no depth."""

    def __init__(self, cfg: NavConfig, reason: str = ""):
        c = cfg.camera
        f = c.width / 2 / math.tan(math.radians(c.rgb_hfov_deg) / 2)
        self.K = Intrinsics(f, f, c.width / 2, c.height / 2, c.width, c.height)
        self.T_rover_cam = _mount(c)
        self.reason = reason
        self._index = 0

    def capture(self) -> Frame:
        h, w = self.K.height, self.K.width
        rgb = np.full((h, w, 3), 60, np.uint8)
        cv2.putText(
            rgb,
            "NO CAMERA - obstacle guard off",
            (40, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 200, 0),
            2,
            cv2.LINE_AA,
        )
        if self.reason:
            cv2.putText(
                rgb,
                self.reason[:70],
                (40, h // 2 + 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        self._index += 1
        return Frame(self._index, 0.0, rgb, np.zeros((h, w), np.uint16), self.K, self.T_rover_cam)

    def close(self) -> None:
        pass


def open_camera(cfg: NavConfig, bridge):
    """The camera `nav.real.camera` asks for; `auto` tries rosbridge, a local OAK-D, none."""
    mode = cfg.real.camera
    errors = []
    if mode in ("auto", "rosbridge"):
        try:
            return RosCamera(cfg, bridge)
        except CameraUnavailable as e:
            if mode == "rosbridge":
                raise
            errors.append(str(e))
    if mode in ("auto", "depthai"):
        try:
            from sorter.nav.real_oakd import RealOakD

            return RealOakD(cfg.camera, cfg.real)
        except Exception as e:  # noqa: BLE001 - try the next source
            if mode == "depthai":
                raise
            errors.append(f"local OAK-D: {e}")
    reason = "; ".join(errors)
    if reason:
        log.warning("no camera, driving blind: %s", reason)
    return NoCamera(cfg, reason)

"""PhysicsCamera: the wrist RGB-D camera, rendered by MuJoCo from where the arm really is.

A true pinhole at `sim.focal_px`, mounted on the gripper (`sim.camera_mount_mm`), with the
D435i's depth behavior: Z in mm, noise growing with distance, no data closer than the minimum
range (the fingers) or on random speckles. It has its own capture thread like the real camera
(block 1) and keeps only the newest frame. It also renders MuJoCo's segmentation, which stands in
for the SAM3 service in the simulator (`segment`).
"""

from __future__ import annotations

import itertools
import logging
import threading
import time

import mujoco
import numpy as np

from sorter.color_classifier.segmenter import Instance
from sorter.core.errors import CameraError
from sorter.core.types import Frame, Intrinsics
from sorter.sim.physics.world import PhysicsWorld

log = logging.getLogger(__name__)

MIN_RANGE_MM = 175  # D435i at 640x480
MAX_RANGE_MM = 3000
DROPOUT = 0.003  # share of pixels with no depth


class PhysicsCamera:
    def __init__(self, world: PhysicsWorld, fps: float = 15.0):
        self.world = world
        self.fps = fps
        cfg = world.cfg
        self.intrinsics = Intrinsics(
            cfg.focal_px, cfg.focal_px, cfg.width / 2, cfg.height / 2, cfg.width, cfg.height
        )
        self._rng = np.random.default_rng(cfg.seed + 7)
        self._seq = itertools.count()
        self._cv = threading.Condition()
        self._latest: tuple[Frame, np.ndarray] | None = None  # frame, flex id per pixel (-1 = none)
        self._want = 0.0  # a caller of fresh() waits for a frame started after this
        self._segs: dict[int, np.ndarray] = {}  # frame seq → segmentation, the last few
        self._thread: threading.Thread | None = None
        self._running = threading.Event()

    # --- Camera ---

    def start(self) -> None:
        if self._thread is None:
            self.world.start()
            self._running.set()
            self._thread = threading.Thread(target=self._loop, name="sim-camera", daemon=True)
            self._thread.start()

    def close(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def latest(self) -> Frame | None:
        self.start()
        with self._cv:
            return self._latest[0] if self._latest else None

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        self.start()
        t = time.monotonic()
        with self._cv:
            self._want = t
            self._cv.notify_all()
            ok = self._cv.wait_for(
                lambda: self._latest is not None and self._latest[0].timestamp > t, timeout_s
            )
            if not ok:
                raise CameraError(f"no fresh frame within {timeout_s} s")
            return self._latest[0]

    # --- the SAM3 stand-in ---

    def segment(self, bgr: np.ndarray) -> list[Instance]:
        """One instance per cloth visible in the frame whose color image is `bgr`."""
        with self._cv:
            seg = next((s for f, s in self._frames() if f.color is bgr), None)
        if seg is None:
            raise CameraError("no segmentation for this frame (too old?)")
        return [Instance(seg == f, 1.0) for f in np.unique(seg) if f >= 0 and (seg == f).sum() > 50]

    def _frames(self):
        return [(f, self._segs[f.seq]) for f in self._recent if f.seq in self._segs]

    # --- rendering ---

    def _loop(self) -> None:
        m = self.world.model
        cfg = self.world.cfg
        renderer = mujoco.Renderer(m, cfg.height, cfg.width)
        data = mujoco.MjData(m)
        self._recent: list[Frame] = []
        period = 1.0 / self.fps
        nxt = time.monotonic()
        try:
            while self._running.is_set():
                with self._cv:  # render on time, or at once for a fresh() caller
                    self._cv.wait_for(
                        lambda due=nxt: self._due(due), max(nxt - time.monotonic(), 0)
                    )
                nxt = time.monotonic() + period
                t = time.monotonic()
                with self.world.lock:
                    mujoco.mj_copyData(data, m, self.world.data)
                frame, seg = self._render(renderer, data, t)
                with self._cv:
                    self._latest = (frame, seg)
                    self._recent = [*self._recent[-7:], frame]
                    self._segs = {f.seq: self._segs.get(f.seq, seg) for f in self._recent}
                    self._segs[frame.seq] = seg
                    self._cv.notify_all()
        except Exception:
            log.exception("sim camera thread died")
        finally:
            renderer.close()

    def _due(self, due: float) -> bool:
        """Time for the next frame: on schedule, or a fresh() caller is waiting."""
        return (
            not self._running.is_set()
            or time.monotonic() >= due
            or (self._latest is not None and self._want > self._latest[0].timestamp)
        )

    def _render(
        self, r: mujoco.Renderer, data: mujoco.MjData, t: float
    ) -> tuple[Frame, np.ndarray]:
        r.update_scene(data, camera="wrist")
        rgb = r.render()
        r.enable_depth_rendering()
        r.update_scene(data, camera="wrist")
        z = r.render() * 1000.0  # m → mm, Z along the optical axis
        r.disable_depth_rendering()
        r.enable_segmentation_rendering()
        r.update_scene(data, camera="wrist")
        ids = r.render()
        r.disable_segmentation_rendering()

        rng = self._rng
        z = z + rng.normal(0, 1, z.shape) * (0.6 + 1.2e-5 * z**2)  # ~1.4 mm at 260 mm
        z[(z < MIN_RANGE_MM) | (z > MAX_RANGE_MM) | (rng.random(z.shape) < DROPOUT)] = 0
        color = rgb[..., ::-1].astype(np.float32) + rng.normal(0, 2.0, rgb.shape)
        flex = np.where(ids[..., 1] == int(mujoco.mjtObj.mjOBJ_FLEX), ids[..., 0], -1)
        item = np.full(flex.shape, -1, np.int16)
        for i, f in enumerate(self.world.item_flex):
            item[flex == f] = i
        frame = Frame(
            color=np.clip(color, 0, 255).astype(np.uint8),
            depth_mm=np.rint(z).astype(np.uint16),
            intrinsics=self.intrinsics,
            timestamp=t,
            seq=next(self._seq),
        )
        return frame, item

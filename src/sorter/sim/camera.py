"""SimCamera: what the wrist camera sees as the arm moves over the sim table (see `scene`)."""

from __future__ import annotations

import itertools
import threading
import time

from sorter.core.types import Frame, Intrinsics
from sorter.sim.scene import Scene
from sorter.sim.world import SimWorld


class SimCamera:
    def __init__(self, world: SimWorld):
        self.world = world
        self.scene = Scene(world)
        self._seq = itertools.count()
        self._lock = threading.Lock()

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    def latest(self) -> Frame | None:
        return self._render()

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        return self._render()

    def _render(self) -> Frame:
        with self._lock:  # the scene reuses buffers; one frame at a time
            t = time.monotonic()
            color, depth = self.scene.render(t)
            seq = next(self._seq)
        s = self.scene
        return Frame(
            color=color,
            depth_mm=depth,
            intrinsics=Intrinsics(s.f, s.f, s.W / 2, s.H / 2, s.W, s.H),
            timestamp=t,
            seq=seq,
        )

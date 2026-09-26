"""SimDriver: an ArmDriver that plays planned paths back on the SimWorld.

Motions take as long as on the real arm (rebot_b601's min-jerk timing) times `sim.time_scale`.
The gripper acts on the world: closing grabs the cloth under the fingertips, opening drops it.
"""

from __future__ import annotations

import threading

import numpy as np
from rebot_b601 import config as rc

from sorter.arm import kinematics as kin
from sorter.core.errors import EStopped
from sorter.sim.world import GRIPPER_S, HELD_OPENING, SimWorld


class SimDriver:
    def __init__(self, world: SimWorld):
        self.world = world
        self._stopped = threading.Event()
        self._opening = 0.0

    def connect(self) -> None:
        self.world.reset_if_sorted()

    def disconnect(self) -> None:
        pass

    @property
    def connected(self) -> bool:
        return True  # the sim arm is always there

    def joints(self) -> np.ndarray:
        return self.world.joints()

    def gripper(self) -> float:
        return self._opening

    def _wait(self, seconds: float) -> None:
        """Sleep `seconds` of sim time; a stop cuts it short and raises EStopped."""
        if seconds > 0 and self._stopped.wait(seconds * self.world.cfg.time_scale):
            raise EStopped("arm held during motion")
        if self._stopped.is_set():
            raise EStopped("arm is held")

    def execute(self, waypoints: np.ndarray, speed_scale: float) -> None:
        if self._stopped.is_set():
            raise EStopped("arm is held")
        seconds = kin.path_duration(waypoints, min(speed_scale, rc.MAX_SPEED_SCALE))
        with self.world.lock:
            self.world.motion.start(waypoints, seconds * self.world.cfg.time_scale)
        self._wait(seconds)

    def set_gripper(self, opening: float) -> float:
        if self._stopped.is_set():
            raise EStopped("arm is held")
        self._wait(GRIPPER_S)
        tcp = self.world.tcp()
        with self.world.lock:
            if opening < self._opening:
                held = self.world.grasp(tcp) if opening < HELD_OPENING else []
                self._opening = HELD_OPENING if held else opening
            else:
                self.world.release(tcp)
                self._opening = opening
        return self._opening

    def stop(self) -> None:
        self._stopped.set()
        with self.world.lock:
            self.world.motion.freeze()

    def resume(self) -> None:
        self._stopped.clear()

    def fault(self) -> str | None:
        return None

    def clear_fault(self) -> None:
        pass

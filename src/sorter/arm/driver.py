"""ArmDriver: the joint-level arm under the controller. Internal to block 5 (and the simulator).

Two implementations: `RebotDriver` (the real arm through `rebot_b601`, or its simulated motors
with `arm.dry_run`) and `sorter.sim.driver.SimDriver` (kinematic, drives the sim world).
"""

from __future__ import annotations

import threading
from typing import Protocol

import numpy as np

from sorter.core.errors import ArmError, EStopped


class ArmDriver(Protocol):
    def connect(self) -> None:
        """Enable the motors and hold the current pose. No-op if already connected."""
        ...

    def disconnect(self) -> None:
        """Disable the motors. The controller calls it only at the rest pose."""
        ...

    @property
    def connected(self) -> bool:
        """Motors enabled: `joints()` and motions work."""
        ...

    def joints(self) -> np.ndarray:
        """Measured joint angles, rad."""
        ...

    def gripper(self) -> float:
        """Measured opening, 0 = closed .. 1 = open."""
        ...

    def execute(self, waypoints: np.ndarray, speed_scale: float) -> None:
        """Run a planned joint path (M, 6), blocking until the arm is still. EStopped if the
        driver is stopped (before or during the motion), ArmError on a fault."""
        ...

    def set_gripper(self, opening: float) -> float:
        """Move the gripper, blocking. Returns the measured opening afterwards (cloth between
        the fingers keeps it from closing fully)."""
        ...

    def stop(self) -> None:
        """Thread-safe: abort the motion, hold where the arm is; later motions raise EStopped
        until `resume()`."""
        ...

    def resume(self) -> None: ...

    def fault(self) -> str | None:
        """A latched fault (joint blocked, lost feedback, overheating): motions fail until
        `clear_fault()`. None when fine."""
        ...

    def clear_fault(self) -> None:
        """Accept the fault and hold where the arm is now. Keeps the torque on if it still is;
        otherwise reconnects (the motors were already off)."""
        ...


class RebotDriver:
    """The reBot B601-RS through `rebot_b601.arm.Arm` (motorbridge, SocketCAN `can0`).

    `dry_run` uses rebot_b601's simulated motors: the same control loop, trajectories and
    safety checks, no bus. Needs `uv sync --extra hardware` for the real bus.
    """

    def __init__(self, dry_run: bool = False, *, arm=None, backend=None, own_loop: bool = True):
        """`arm`, `backend`, `own_loop`: for the physics simulator, which supplies the motors
        and runs the control loop in simulated time (`rebot_b601.arm.Arm.connect`)."""
        from rebot_b601.arm import Arm

        self.arm = arm or Arm()
        self.dry_run = dry_run
        self._backend = backend
        self._own_loop = own_loop
        self._stopped = threading.Event()

    def connect(self) -> None:
        if not self.arm.connected:
            self._call(
                self.arm.connect,
                enable=True,
                simulate=self.dry_run,
                backend=self._backend,
                own_loop=self._own_loop,
            )

    def disconnect(self) -> None:
        if self.arm.connected:
            self._call(self.arm.disconnect, go_home=False)

    @property
    def connected(self) -> bool:
        return bool(self.arm.connected)

    def joints(self) -> np.ndarray:
        return self.arm.joints()

    def gripper(self) -> float:
        return float(self.arm.status().get("gripper_opening") or 0.0)

    def execute(self, waypoints: np.ndarray, speed_scale: float) -> None:
        if self._stopped.is_set():
            raise EStopped("arm is held")
        self._call(self.arm.execute_path, waypoints, speed_scale)

    def set_gripper(self, opening: float) -> float:
        if self._stopped.is_set():
            raise EStopped("arm is held")
        self._call(self.arm.set_gripper, opening)
        return self.gripper()

    def stop(self) -> None:
        self._stopped.set()
        if self.arm.connected:
            self.arm.stop()

    def resume(self) -> None:
        self._stopped.clear()

    def fault(self) -> str | None:
        return self.arm.status().get("fault")

    def clear_fault(self) -> None:
        if self.arm.connected and self.arm.status()["torque_enabled"]:
            self._call(self.arm.clear_fault)
        else:  # torque already off: nothing more can drop
            self._call(self.arm.disconnect, go_home=False)
            self.connect()

    def _call(self, fn, *args, **kwargs):
        from rebot_b601.arm import ArmError as RebotError

        try:
            return fn(*args, **kwargs)
        except RebotError as e:
            if self._stopped.is_set():
                raise EStopped(f"arm held during motion ({e})") from None
            raise ArmError(str(e)) from None

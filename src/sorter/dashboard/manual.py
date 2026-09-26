"""Manual arm control for setting up the rig: the dashboard's `/manual` page, no state machine.

Named poses (with a tour through them), joint jog, gripper, re-teaching a pose into
`config/rig.yaml`, and clearing an arm fault (a blocked joint) without a restart. One motion at
a time, each in its own thread; Hold works throughout (`POST /api/command {"cmd": "hold"}` →
`arm.hold()`).
"""

from __future__ import annotations

import logging
import re
import threading
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from sorter.arm import kinematics as kin
from sorter.arm.config import POSE_NAMES
from sorter.arm.controller import Controller
from sorter.core.errors import SorterError

log = logging.getLogger(__name__)

# The tour starts where a cycle does, at the box, then visits every place the arm goes.
TOUR = ("look_box", "look_bg", "place_bg", "bin_light", "bin_dark", "bin_colored", "home")


class Busy(Exception):
    """A motion is already running."""


def write_pose(rig: Path, name: str, q: Sequence[float]) -> str:
    """Replace the `poses.<name>` line in rig.yaml, keeping the rest of the file. Returns it."""
    line = f"  {name}: [{', '.join(f'{v:.4f}' for v in q)}]"
    text = rig.read_text() if rig.is_file() else ""
    m = re.search(r"^poses:\n((?:  .*\n)*)", text, re.MULTILINE)
    if m is None:
        raise ValueError(f"no `poses:` block in {rig}; add it by hand:\nposes:\n{line}")
    block = m.group(1)
    new_block, n = re.subn(rf"^  {re.escape(name)}:.*$", line, block, flags=re.MULTILINE)
    if n == 0:
        new_block = block + line + "\n"
    rig.write_text(text[: m.start(1)] + new_block + text[m.end(1) :])
    return line


class ManualControl:
    def __init__(self, arm: Controller, rig_file: Path, gripper_open: float):
        self.arm = arm
        self.rig_file = Path(rig_file)
        self.gripper_open = gripper_open
        self._lock = threading.Lock()
        self._action: str | None = None
        self._error: str | None = None
        self._last: str | None = None
        self._tour_i = 0

    # --- runner ---

    def _submit(self, label: str, fn: Callable[[], None]) -> None:
        with self._lock:
            if self._action is not None:
                raise Busy(f"busy: {self._action}")
            self._action, self._error = label, None
        threading.Thread(target=self._work, args=(label, fn), name="manual", daemon=True).start()

    def _work(self, label: str, fn: Callable[[], None]) -> None:
        error = None
        try:
            fn()
        except (SorterError, ValueError) as e:
            error = f"{label}: {e}"
            log.warning("manual %s", error)
        except Exception as e:  # keep the page alive; the traceback goes to the log
            error = f"{label}: {e}"
            log.exception("manual %s failed", label)
        with self._lock:
            self._action, self._error = None, error
            self._last = label if error is None else self._last

    def wait(self, timeout_s: float = 30.0) -> None:
        """Block until no motion runs (tests)."""
        for t in [t for t in threading.enumerate() if t.name == "manual"]:
            t.join(timeout_s)

    # --- actions ---

    def _route(self, name: str) -> None:
        # A straight joint move between a bin and anywhere else sweeps through the bin walls:
        # go via home, like ArmController.drop_to_bin.
        at = self.arm.at or ""
        via_bin = name.startswith("bin_") or at.startswith("bin_")
        if via_bin and "home" not in (name, at):
            self.arm.go_to("home")
        self.arm.go_to(name)

    def go(self, name: str) -> None:
        if name not in POSE_NAMES:
            raise ValueError(f"unknown pose {name!r}")
        self._submit(f"go {name}", lambda: self._route(name))

    def tour_next(self) -> None:
        name = TOUR[self._tour_i]

        def step() -> None:
            self._route(name)
            self._tour_i = (self._tour_i + 1) % len(TOUR)

        self._submit(f"tour {name}", step)

    def tour_reset(self) -> None:
        self._tour_i = 0

    def jog(self, joint: int, delta_rad: float) -> None:
        if not 0 <= joint < 6:
            raise ValueError(f"joint {joint} out of range 0..5")
        q = np.array(self.arm.joints(), dtype=float)
        q[joint] += delta_rad
        label = f"jog J{joint + 1} {np.degrees(delta_rad):+.0f}°"
        self._submit(label, lambda: self.arm.move_joints(q))

    def gripper(self, open_: bool) -> None:
        opening = self.gripper_open if open_ else 0.0
        label = "open gripper" if open_ else "close gripper"
        self._submit(label, lambda: self.arm.set_gripper(opening))

    def release(self) -> None:
        self._submit("release hold", self.arm.release)

    def clear_fault(self) -> None:
        self._submit("clear fault", self.arm.clear_fault)

    def save_pose(self, name: str) -> str:
        """The current joints become pose `name`, now and in rig.yaml. Returns the YAML line."""
        if name not in POSE_NAMES:
            raise ValueError(f"unknown pose {name!r}")
        with self._lock:
            if self._action is not None:
                raise Busy(f"busy: {self._action}")
        q = [round(v, 4) for v in self.arm.joints()]
        line = write_pose(self.rig_file, name, q)
        self.arm.set_pose(name, q)
        log.info("pose %s saved to %s", name, self.rig_file)
        return line

    # --- status ---

    def state(self) -> dict[str, Any]:
        q = self.arm.joints()
        T = kin.fk_tcp(q)
        with self._lock:
            action, error, last = self._action, self._error, self._last
        return {
            "busy": action is not None,
            "action": action,
            "last": last,
            "error": error,
            "held": self.arm.held,
            "fault": self.arm.fault,
            "at": self.arm.at,
            "joints": [round(float(v), 4) for v in q],
            "gripper": round(float(self.arm.gripper_opening()), 3),
            "tcp_mm": [round(float(v), 1) for v in T[:3, 3]],
            "poses": {n: [round(float(v), 4) for v in self.arm.poses[n]] for n in POSE_NAMES},
            "tour": list(TOUR),
            "tour_next": TOUR[self._tour_i],
            "rig_file": str(self.rig_file),
        }

"""Interfaces between blocks. Contracts: docs/architecture.md → Contracts."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

from sorter.core.types import (
    ArmPoint,
    BackgroundResult,
    BoxResult,
    ColorClass,
    Frame,
    GraspPoint,
    Observation,
    PickResult,
    PixelPoint,
    Pose,
    Zone,
)


class Camera(Protocol):
    """Block 1. Runs its own capture thread and keeps only the newest frame."""

    def start(self) -> None: ...
    def close(self) -> None: ...

    def latest(self) -> Frame | None:
        """Non-blocking; for the live feed."""
        ...

    def fresh(self, timeout_s: float = 2.0) -> Frame:
        """Block until a frame whose exposure started AFTER the call. CameraError on timeout."""
        ...


class BoxDetector(Protocol):
    """Block 3. Stateless: failed grasps are passed as `avoid`."""

    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult: ...


class ColorClassifier(Protocol):
    """Block 4."""

    def classify(self, frame: Frame) -> BackgroundResult: ...


class Calibration(Protocol):
    """Block 2. The only place where pixel ↔ arm conversion happens."""

    def cam_pose(self, ee_pose: Pose) -> Pose:
        """T_base_link5 (`ArmController.ee_pose()`) → T_base_cam."""
        ...

    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint:
        """CalibrationError if obs.T_base_cam is None or depth is 0."""
        ...

    def to_pixel(self, obs: Observation, p: ArmPoint) -> PixelPoint | None:
        """None if outside the image."""
        ...


class ArmController(Protocol):
    """Block 5. Every motion method blocks until the arm is still."""

    def start(self) -> None:
        """Connect, enable motors, hold the current position."""
        ...

    def shutdown(self) -> None:
        """Move to the `rest` pose, then disable motors."""
        ...

    def home(self) -> None: ...

    def look(self, zone: Zone) -> None:
        """Go to look_box / look_bg; no-op if already there."""
        ...

    def pick(self, target: ArmPoint, zone: Zone) -> PickResult:
        """TargetRejected (no motion) if outside the zone workspace or IK fails."""
        ...

    def place_on_background(self) -> None: ...
    def drop_to_bin(self, color: ColorClass) -> None: ...

    def ee_pose(self) -> Pose:
        """T_base_link5 from FK of the measured joints: the link the camera is fixed to (it
        doesn't turn with joint 6, D-027)."""
        ...

    def joints(self) -> tuple[float, ...]: ...

    def hold(self) -> None:
        """Thread-safe. Freeze in place; every later motion raises EStopped until recover()."""
        ...

    def recover(self) -> None:
        """Leave hold: lift to safe Z, open the gripper above the background, home."""
        ...

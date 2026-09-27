"""The wired-up components, built by `sorter.app.build_system`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sorter.core.config import Config
from sorter.core.hub import Hub
from sorter.core.observer import Observer
from sorter.core.protocols import ArmController, BoxDetector, Calibration, Camera, FloorDetector


@dataclass
class System:
    cfg: Config
    camera: Camera
    arm: ArmController
    calibration: Calibration
    box_detector: BoxDetector
    floor_detector: FloorDetector
    observer: Observer
    hub: Hub
    world: Any = None  # the PhysicsWorld when the hardware is simulated

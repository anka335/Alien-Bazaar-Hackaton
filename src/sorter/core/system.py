"""The wired-up components, built by `sorter.app.build_system`."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sorter.core.config import Config
from sorter.core.hub import Hub
from sorter.core.observer import Observer
from sorter.core.protocols import ArmController, BoxDetector, Calibration, Camera, ColorClassifier


@dataclass
class System:
    cfg: Config
    camera: Camera
    arm: ArmController
    calibration: Calibration
    box_detector: BoxDetector
    color_classifier: ColorClassifier
    observer: Observer
    hub: Hub
    world: Any = None  # SimWorld when any component is simulated

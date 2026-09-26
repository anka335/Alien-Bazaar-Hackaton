"""Sim backend factory, used by `sorter.app` for components with `backends.<name>: sim`."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from sorter.arm.controller import Controller
from sorter.sim.calibration import SimCalibration
from sorter.sim.camera import SimCamera
from sorter.sim.driver import SimDriver
from sorter.sim.vision import SimBoxDetector, SimColorClassifier
from sorter.sim.world import SimWorld

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(name: str, cfg: Config, world: SimWorld) -> Any:
    match name:
        case "camera":
            return SimCamera(world)
        case "arm":  # the real controller on a simulated driver
            return Controller(SimDriver(world), cfg.arm, cfg.poses, cfg.zones)
        case "calibration":
            return SimCalibration(world)
        case "box_detector":
            return SimBoxDetector(world, cfg.box_detector.avoid_radius_px)
        case "color_classifier":
            return SimColorClassifier(world)
    raise ValueError(f"unknown component {name!r}")

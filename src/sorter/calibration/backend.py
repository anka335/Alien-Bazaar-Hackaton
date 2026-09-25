"""Real backend of block 2: per-zone plane homographies (D-014)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.calibration.plane import PlaneCalibration

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> PlaneCalibration:
    return PlaneCalibration(cfg.calibration)

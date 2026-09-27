"""Real arm backend (block 5): the controller on the reBot B601-RS (D-019)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.arm.controller import Controller
from sorter.arm.driver import RebotDriver

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> Controller:
    return Controller(
        RebotDriver(dry_run=cfg.arm.dry_run, max_speed_scale=cfg.arm.max_speed_scale),
        cfg.arm,
        cfg.poses,
        cfg.zones,
    )

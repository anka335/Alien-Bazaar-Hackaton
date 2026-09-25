"""Real backend of block 5: the SO-101 on its Feetech bus (D-014)."""

from __future__ import annotations

from typing import TYPE_CHECKING

from sorter.arm.bus import FeetechBus, find_port
from sorter.arm.controller import So101Arm

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> So101Arm:
    a = cfg.arm

    def bus() -> FeetechBus:
        return FeetechBus(find_port() if a.port == "auto" else a.port, a.baudrate)

    return So101Arm(a, cfg.poses, cfg.zones, bus)

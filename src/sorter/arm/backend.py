"""Real backend of block 5: the SO-101 on its Feetech bus (D-014)."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import TYPE_CHECKING

from sorter.arm.bus import FeetechBus, MockBus, find_port
from sorter.arm.controller import So101Arm

if TYPE_CHECKING:
    from sorter.core.config import Config


def create(cfg: Config) -> So101Arm:
    a = cfg.arm

    def bus() -> FeetechBus:
        return FeetechBus(find_port() if a.port == "auto" else a.port, a.baudrate)

    return So101Arm(a, cfg.poses, cfg.zones, bus)


def create_mock(
    cfg: Config,
    sleep: Callable[[float], None] = time.sleep,
    lag: float = 1.0,
    sag: dict[int, int] | None = None,
) -> So101Arm:
    """`So101Arm` on a `MockBus` (the MCP server's simulated arm, dry runs of the tools), at the
    `rest` pose if taught. `lag` and `sag` (ticks per servo) make the mock servos imperfect."""
    a = cfg.arm
    ranges = {sid: (648, 3448) for sid in a.ids} | {a.gripper_id: (1000, 3000)}
    bus = MockBus(ranges, lag=lag)
    bus.sag.update(sag or {})
    arm = So101Arm(a, cfg.poses, cfg.zones, lambda: bus, sleep=sleep)
    arm.connect()
    if "rest" in cfg.poses:
        bus.pos.update(zip(a.ids, arm.raw_from_q(cfg.poses["rest"]), strict=True))
        bus.goal.update(bus.pos)
    return arm

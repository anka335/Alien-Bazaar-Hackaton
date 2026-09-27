import logging

import pytest

from sorter.core.errors import WrongMode
from sorter.core.hub import Hub
from sorter.core.log import HubLogHandler
from sorter.core.types import Command, OperatorMode, Phase, Status


class _Camera:
    def latest(self):
        return "frame"


def test_hold_bypasses_the_queue():
    held = []
    hub = Hub(_Camera(), on_hold=lambda: held.append(True))
    hub.send("hold")
    assert held == [True]
    assert hub.next_command(0) is None


def test_commands_are_queued_in_order():
    hub = Hub(_Camera(), on_hold=lambda: None)
    hub.send(Command.START)
    hub.send("pause")
    assert hub.next_command(0) is Command.START
    assert hub.next_command(0.01) is Command.PAUSE
    assert hub.next_command(0.01) is None


def test_status_carries_log_events():
    hub = Hub(_Camera(), on_hold=lambda: None, max_events=2)
    hub.publish_status(Status(phase=Phase.SCAN))
    logger = logging.getLogger("sorter.test_hub")
    handler = HubLogHandler(hub)
    logger.addHandler(handler)
    try:
        logger.info("not an event")
        for i in range(3):
            logger.warning("w%d", i)
        logger.error("boom")
    finally:
        logger.removeHandler(handler)
    s = hub.status()
    assert s.phase is Phase.SCAN
    assert [(e.level, e.msg) for e in s.events] == [("warning", "w2"), ("error", "boom")]
    assert hub.live_frame() == "frame"


def test_run_commands_only_in_a_run_mode():
    hub = Hub(_Camera(), on_hold=lambda: None)
    assert hub.mode() is OperatorMode.LOAD
    hub.set_mode(OperatorMode.UNLOAD)
    hub.send(Command.START)
    assert hub.next_command(0) is Command.START
    hub.publish_status(Status())  # the state machine took it and is idle again
    hub.set_mode(OperatorMode.MANUAL)
    with pytest.raises(WrongMode):
        hub.send(Command.START)

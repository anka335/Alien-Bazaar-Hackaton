"""Logging → Hub events, so no module needs a Hub reference."""

from __future__ import annotations

import logging
import time

from sorter.core.hub import Hub
from sorter.core.types import Event


class HubLogHandler(logging.Handler):
    """Turns log records (warnings and errors by default) into Hub `Event`s."""

    def __init__(self, hub: Hub, level: int = logging.WARNING):
        super().__init__(level)
        self.hub = hub

    def emit(self, record: logging.LogRecord) -> None:
        try:
            if record.levelno >= logging.ERROR:
                level = "error"
            elif record.levelno >= logging.WARNING:
                level = "warning"
            else:
                level = "info"
            self.hub.add_event(
                Event(t=time.monotonic(), level=level, source=record.name, msg=record.getMessage())
            )
        except Exception:
            self.handleError(record)

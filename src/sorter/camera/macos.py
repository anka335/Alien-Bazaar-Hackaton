"""macOS: keep the system UVC driver off the RealSense while the camera runs.

UVCAssistant (the macOS driver for USB cameras, a user process) holds the D435i's video
interfaces. As root, libusb takes the device from it for a moment, but librealsense opens and
closes the interfaces several times while starting, and UVCAssistant grabs them back in ~30 ms:
`failed to set power state`. launchd restarts it at once if killed, so it is stopped (SIGSTOP)
while the camera runs and continued (SIGCONT) on close and at exit. Built-in Mac cameras use
another service. If the process dies hard, continue it by hand: `sudo killall -CONT UVCAssistant`.
"""

from __future__ import annotations

import atexit
import contextlib
import logging
import os
import signal
import subprocess
import sys

log = logging.getLogger(__name__)

UVC_ASSISTANT = "UVCAssistant"


class UvcAssistantFreeze:
    def __init__(self) -> None:
        self._pids: list[int] = []

    def freeze(self) -> None:
        if sys.platform != "darwin" or self._pids:
            return
        if os.geteuid() != 0:
            log.warning("camera: not root, macOS keeps the RealSense (run with sudo)")
            return
        out = subprocess.run(["pgrep", "-x", UVC_ASSISTANT], capture_output=True, text=True)
        self._pids = [int(p) for p in out.stdout.split()]
        for pid in self._pids:
            with contextlib.suppress(ProcessLookupError):
                os.kill(pid, signal.SIGSTOP)
        if self._pids:
            atexit.register(self.resume)
            log.info("camera: %s stopped while the camera runs", UVC_ASSISTANT)

    def resume(self) -> None:
        for pid in self._pids:
            with contextlib.suppress(ProcessLookupError):
                os.kill(pid, signal.SIGCONT)
        if self._pids:
            log.info("camera: %s continued", UVC_ASSISTANT)
        self._pids = []

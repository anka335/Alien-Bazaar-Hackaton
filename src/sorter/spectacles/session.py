"""The Spectacles teleop protocol v1 (the lens repo's protocol.md), mobile base only.

The lens sends `teleop` at 30 Hz; the left hand is the base's joystick (`base.vx`, `base.wz`).
This machine has no arm: the right hand is ignored and `status.arm` stays `holding` (`fault` on
a link timeout). No ROS and no network here: the server feeds messages and the clock in.

The stops (protocol.md, Deadman):
- link timeout: no valid `teleop` for 2 s -> base and arm `fault`, `fault: "timeout"`; the left
  hand must be seen open before it drives again;
- base stale: no valid `teleop` for 0.3 s -> base idle, resumes by itself;
- socket close: base idle at once;
- rover absent: no rover odometry for 0.5 s -> the left clutch is not accepted; if that happened
  while a socket was connected, the left hand must be seen open with the rover back.
A replacement socket needs a release too. The rover firmware's own 0.5 s stop is the last line.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

LINK_TIMEOUT_S = 2.0
BASE_STALE_S = 0.3
ROVER_ABSENT_S = 0.5

# the protocol's full scale: a launch limit may be lower, never higher
PROTOCOL_MAX_VX = 0.35
PROTOCOL_MAX_REVERSE = 0.15
PROTOCOL_MAX_WZ = 0.8


@dataclass(frozen=True)
class Limits:
    max_vx: float = 0.20
    max_reverse: float = 0.10
    max_wz: float = 0.6

    def __post_init__(self):
        for name, value, top in (
            ("max_vx", self.max_vx, PROTOCOL_MAX_VX),
            ("max_reverse", self.max_reverse, PROTOCOL_MAX_REVERSE),
            ("max_wz", self.max_wz, PROTOCOL_MAX_WZ),
        ):
            if not (0 < value <= top):
                raise ValueError(f"{name} {value} must be in (0, {top}] (protocol.md)")

    def clamp(self, vx: float, wz: float) -> tuple[float, float]:
        return (
            max(-self.max_reverse, min(self.max_vx, vx)),
            max(-self.max_wz, min(self.max_wz, wz)),
        )


@dataclass(frozen=True)
class Teleop:
    seq: int
    base_engaged: bool
    vx: float
    wz: float
    arm_engaged: bool
    reset: bool = False


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _finite_list(x, n: int) -> bool:
    return isinstance(x, list) and len(x) == n and all(_finite(v) for v in x)


def parse_teleop(msg) -> Teleop | None:
    """A valid v1 `teleop`, or None (malformed, or another type)."""
    if not isinstance(msg, dict) or msg.get("v") != 1 or msg.get("type") != "teleop":
        return None
    seq, base, arm = msg.get("seq"), msg.get("base"), msg.get("arm")
    if not (isinstance(seq, int) and not isinstance(seq, bool)) or not _finite(
        msg.get("timestamp")
    ):
        return None
    if not isinstance(base, dict) or not isinstance(arm, dict):
        return None
    if not isinstance(base.get("engaged"), bool) or not isinstance(arm.get("engaged"), bool):
        return None
    if not (_finite(base.get("vx")) and _finite(base.get("wz"))):
        return None
    orientation = arm.get("orientation")
    if not (_finite_list(arm.get("position"), 3) and _finite_list(orientation, 4)):
        return None
    if math.hypot(*orientation) < 1e-6 or not _finite(arm.get("gripper")):
        return None
    return Teleop(
        seq=seq,
        base_engaged=base["engaged"],
        vx=float(base["vx"]),
        wz=float(base["wz"]),
        arm_engaged=arm["engaged"],
        reset=msg.get("reset") is True,
    )


class BaseSession:
    """Protocol state for one bridge run. Thread-safe: the socket and the publisher share it."""

    def __init__(self, limits: Limits | None = None, clock: Callable[[], float] = time.monotonic):
        self.limits = limits or Limits()
        self._clock = clock
        self._lock = threading.Lock()
        self._sockets = 0  # accepted so far
        self._connected = False
        self._rx_t: float | None = None  # the link clock: accept, then every valid teleop
        self._last: Teleop | None = None
        self._echo: int | None = None
        self._fault: str | None = None
        self._left_needs_release = False
        self._odom_t: float | None = None

    # --- inputs -------------------------------------------------------------------------------

    def on_connect(self) -> None:
        """A socket was accepted; a second one (or any later one) needs both hands released."""
        with self._lock:
            now = self._clock()
            self._sockets += 1
            self._connected = True
            self._rx_t = now
            self._last = None
            self._echo = None
            self._fault = None
            if self._sockets > 1:
                self._left_needs_release = True

    def on_disconnect(self) -> None:
        """The accepted socket closed: the base stops now; the link clock keeps running."""
        with self._lock:
            self._connected = False
            self._last = None

    def on_message(self, msg) -> bool:
        """Any decoded JSON from the lens; True if it was a valid teleop."""
        t = parse_teleop(msg)
        if t is None:
            return False
        with self._lock:
            now = self._clock()
            self._update(now)
            self._fault = None
            if not t.base_engaged and self._present(now):
                self._left_needs_release = False
            self._last = t
            self._echo = t.seq
            self._rx_t = now
        return True

    def on_odom(self) -> None:
        with self._lock:
            now = self._clock()
            self._odom_t = now
            self._update(now)

    # --- outputs ------------------------------------------------------------------------------

    def command(self) -> tuple[bool, float, float]:
        """(driving, vx, wz): the twist to send now; zeros unless an accepted left clutch drives."""
        with self._lock:
            now = self._clock()
            self._update(now)
            if not self._driving(now):
                return False, 0.0, 0.0
            vx, wz = self.limits.clamp(self._last.vx, self._last.wz)
            return True, vx, wz

    def status(self) -> dict:
        with self._lock:
            now = self._clock()
            self._update(now)
            if self._fault:
                base = arm = "fault"
            else:
                base = "driving" if self._driving(now) else "idle"
                arm = "holding"
            return {
                "v": 1,
                "type": "status",
                "echoSeq": self._echo,
                "base": base,
                "arm": arm,
                "fault": self._fault,
            }

    def rover_present(self) -> bool:
        with self._lock:
            return self._present(self._clock())

    # --- internals (lock held) ----------------------------------------------------------------

    def _present(self, now: float) -> bool:
        return self._odom_t is not None and now - self._odom_t <= ROVER_ABSENT_S

    def _update(self, now: float) -> None:
        if self._rx_t is not None and not self._fault and now - self._rx_t >= LINK_TIMEOUT_S:
            self._fault = "timeout"
            self._left_needs_release = True
            self._last = None
        if self._connected and not self._present(now):
            self._left_needs_release = True

    def _driving(self, now: float) -> bool:
        t = self._last
        return (
            self._connected
            and not self._fault
            and t is not None
            and t.base_engaged
            and now - self._rx_t <= BASE_STALE_S
            and self._present(now)
            and not self._left_needs_release
        )

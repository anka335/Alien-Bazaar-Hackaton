"""Feetech STS3215 servo bus of the SO-101, and a mock of it for tests. Raw units: ticks 0..4095.

Only RAM registers are written (torque, goals, torque limit, acceleration). The EEPROM keeps the
LeRobot calibration: homing offset and the Min/Max position limits, which define joint zero.
"""

from __future__ import annotations

import glob
import sys
import threading
from typing import Protocol

from sorter.core.errors import ArmError

# STS3215 control table (address, bytes)
MIN_LIMIT = (9, 2)
MAX_LIMIT = (11, 2)
P_COEFFICIENT = (21, 1)
TORQUE_ENABLE = (40, 1)
ACCELERATION = (41, 1)
GOAL_POSITION = (42, 2)
GOAL_SPEED = (46, 2)
TORQUE_LIMIT = (48, 2)
PRESENT_POSITION = (56, 2)
PRESENT_VOLTAGE = (62, 1)

TICKS = 4096


class Bus(Protocol):
    def read_positions(self, ids: list[int]) -> list[int]: ...
    def write_goals(self, goals: dict[int, int]) -> None: ...
    def set_torque(self, ids: list[int], on: bool) -> None: ...
    def read_range(self, sid: int) -> tuple[int, int]: ...
    def write(self, sid: int, reg: tuple[int, int], value: int) -> None: ...
    def read(self, sid: int, reg: tuple[int, int]) -> int: ...
    def close(self) -> None: ...


def find_port() -> str:
    pattern = "/dev/cu.usbmodem*" if sys.platform == "darwin" else "/dev/ttyACM*"
    ports = sorted(glob.glob(pattern))
    if len(ports) != 1:
        raise ArmError(f"arm.port is auto, but {pattern} matches {ports or 'nothing'}; set it")
    return ports[0]


def _decode(raw: int) -> int:
    """Sign-magnitude (bit 15), as the STS reports positions past the homing offset."""
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


class FeetechBus:
    """scservo_sdk port + packet handler. Thread-safe: one transaction at a time."""

    def __init__(self, port: str, baudrate: int):
        import scservo_sdk as scs

        self._scs = scs
        self._lock = threading.Lock()
        self._port = scs.PortHandler(port)
        if not self._port.openPort():
            raise ArmError(f"can't open {port}")
        if not self._port.setBaudRate(baudrate):
            raise ArmError(f"can't set {baudrate} baud on {port}")
        self._ph = scs.PacketHandler(0)  # STS: protocol end 0
        self._reader = scs.GroupSyncRead(self._port, self._ph, *PRESENT_POSITION)
        self._reader_ids: tuple[int, ...] = ()

    def _check(self, what: str, result: int, error: int = 0) -> None:
        if result != self._scs.COMM_SUCCESS:
            raise ArmError(f"{what}: {self._ph.getTxRxResult(result)}")
        if error:
            raise ArmError(f"{what}: {self._ph.getRxPacketError(error)}")

    def read(self, sid: int, reg: tuple[int, int]) -> int:
        addr, n = reg
        with self._lock:
            fn = self._ph.read1ByteTxRx if n == 1 else self._ph.read2ByteTxRx
            value, result, error = fn(self._port, sid, addr)
        self._check(f"read {addr} of servo {sid}", result, error)
        return value

    def write(self, sid: int, reg: tuple[int, int], value: int) -> None:
        addr, n = reg
        with self._lock:
            fn = self._ph.write1ByteTxRx if n == 1 else self._ph.write2ByteTxRx
            result, error = fn(self._port, sid, addr, value)
        self._check(f"write {addr} of servo {sid}", result, error)

    def read_positions(self, ids: list[int]) -> list[int]:
        with self._lock:
            if tuple(ids) != self._reader_ids:
                self._reader.clearParam()
                for sid in ids:
                    self._reader.addParam(sid)
                self._reader_ids = tuple(ids)
            result = self._reader.txRxPacket()
            if result == self._scs.COMM_SUCCESS:
                return [_decode(self._reader.getData(sid, *PRESENT_POSITION)) for sid in ids]
            out = []  # sync read failed (a noisy packet): fall back to one read per servo
            for sid in ids:
                value, result, error = self._ph.read2ByteTxRx(self._port, sid, PRESENT_POSITION[0])
                if result != self._scs.COMM_SUCCESS:
                    raise ArmError(
                        f"read position of servo {sid}: {self._ph.getTxRxResult(result)}"
                    )
                out.append(_decode(value))
            return out

    def write_goals(self, goals: dict[int, int]) -> None:
        scs = self._scs
        with self._lock:
            w = scs.GroupSyncWrite(self._port, self._ph, *GOAL_POSITION)
            for sid, goal in goals.items():
                goal = max(0, min(TICKS - 1, int(goal)))
                w.addParam(sid, [scs.SCS_LOBYTE(goal), scs.SCS_HIBYTE(goal)])
            result = w.txPacket()
        self._check("write goals", result)

    def set_torque(self, ids: list[int], on: bool) -> None:
        for sid in ids:
            self.write(sid, TORQUE_ENABLE, 1 if on else 0)

    def read_range(self, sid: int) -> tuple[int, int]:
        return self.read(sid, MIN_LIMIT), self.read(sid, MAX_LIMIT)

    def close(self) -> None:
        with self._lock:
            self._port.closePort()


class MockBus:
    """Servos that reach their goal right away (or `lag` of the way per read), for tests."""

    def __init__(
        self,
        ranges: dict[int, tuple[int, int]],
        positions: dict[int, int] | None = None,
        lag: float = 1.0,
    ):
        self.ranges = dict(ranges)
        self.pos = {sid: (lo + hi) // 2 for sid, (lo, hi) in ranges.items()}
        self.pos.update(positions or {})
        self.goal = dict(self.pos)
        self.torque = dict.fromkeys(ranges, False)
        self.regs: dict[tuple[int, int], int] = {}
        self.lag = lag
        self.blocked: dict[int, int] = {}  # servo → position it can't go past (cloth, obstacle)
        self.sag: dict[int, int] = {}  # servo → ticks it stops short of its goal (gravity)
        self.start_delay = 0  # reads after a new goal before the servos start moving
        self._idle = 0
        self.goal_log: list[dict[int, int]] = []
        self.closed = False

    def _step(self) -> None:
        if self._idle > 0:
            self._idle -= 1
            return
        for sid in self.pos:
            if not self.torque[sid]:
                continue
            target = self.goal[sid] - self.sag.get(sid, 0)
            if sid in self.blocked:
                stop = self.blocked[sid]
                if (self.pos[sid] - stop) * (target - stop) < 0 or self.pos[sid] == stop:
                    target = stop
            self.pos[sid] += round((target - self.pos[sid]) * self.lag)

    def read_positions(self, ids: list[int]) -> list[int]:
        self._step()
        return [self.pos[sid] for sid in ids]

    def write_goals(self, goals: dict[int, int]) -> None:
        self.goal_log.append(dict(goals))
        self._idle = self.start_delay
        self.goal.update({sid: int(g) for sid, g in goals.items()})

    def set_torque(self, ids: list[int], on: bool) -> None:
        for sid in ids:
            self.torque[sid] = on

    def read_range(self, sid: int) -> tuple[int, int]:
        return self.ranges[sid]

    def write(self, sid: int, reg: tuple[int, int], value: int) -> None:
        if reg == TORQUE_ENABLE:
            self.torque[sid] = bool(value)
        elif reg == GOAL_POSITION:
            self.goal[sid] = value
        self.regs[(sid, reg[0])] = value

    def read(self, sid: int, reg: tuple[int, int]) -> int:
        if reg == PRESENT_POSITION:
            return self.pos[sid]
        if reg == PRESENT_VOLTAGE:
            return 74
        return self.regs.get((sid, reg[0]), 0)

    def close(self) -> None:
        self.closed = True

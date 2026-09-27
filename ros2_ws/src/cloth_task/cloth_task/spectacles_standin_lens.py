"""spectacles_standin_lens: plays the Spectacles lens over the WebSocket for tests, no glasses.

    python3 ros2_ws/src/cloth_task/cloth_task/spectacles_standin_lens.py \
        --url ws://127.0.0.1:9110 --scenario scenario.json --log standin_lens.jsonl

A scenario is a list of phases, played in order. Each phase lasts `duration` seconds and is one of:

  send        teleop v1 at 30 Hz with the phase's `base` and `arm` fields (merged over the
              disengaged defaults, BASE_OPEN and ARM_OPEN). Opens a socket first if none is open.
  silence     the socket stays open (opened first if none is) and nothing is sent.
  disconnect  the socket is closed (1000) and nothing is sent; the next send or silence phase
              opens a new socket, whose `seq` starts again at 0.

The frames are what the lens would compute (it does not run `teleopStep`): `seq` from 0 on each
socket, `timestamp` in Unix epoch ms. The JSON scenario file is a list of phase objects (or
{"phases": [...]}), with the keys of `Phase`, for example:

    [{"duration": 1.0},
     {"duration": 0.5, "base": {"engaged": true, "vx": 0.35}},
     {"duration": 0.5, "kind": "silence"},
     {"duration": 0.2, "kind": "disconnect"}]

Every event goes to a JSON-lines log (truncated at start), one object per line, with `t` from
time.monotonic() (system-wide on Linux, so comparable across processes) and `event`:
`phase` (index, name, kind), `connect` (url), `sent` (frame, and `t_send` taken just before
the send), `status` (a received status),
`recv` (any other received text), `close` (code, reason, by client or server), `error`
(a connection error), `fail` (reason) and last `done` (ok).

The run fails (exit 1) on a connection error, or when the server closes the socket during a
phase that does not set `expect_close` (the phase then ends at the close). Needs only
`websockets`, so it runs without ROS.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
import sys
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import websockets
from websockets.exceptions import ConnectionClosed, WebSocketException

PROTOCOL_VERSION = 1
RATE_HZ = 30.0
OPEN_TIMEOUT_S = 5.0
CLOSE_WAIT_S = 2.0

SEND, SILENCE, DISCONNECT = "send", "silence", "disconnect"
KINDS = (SEND, SILENCE, DISCONNECT)
# Protocol v1 with both clutches open
BASE_OPEN: dict[str, Any] = {"engaged": False, "vx": 0.0, "wz": 0.0}
ARM_OPEN: dict[str, Any] = {
    "engaged": False,
    "position": [0.0, 0.0, 0.0],
    "orientation": [0.0, 0.0, 0.0, 1.0],
    "gripper": 0.0,
}


@dataclass(frozen=True)
class Phase:
    duration: float  # s
    kind: str = SEND
    base: Mapping[str, Any] = field(default_factory=dict)  # over BASE_OPEN, send only
    arm: Mapping[str, Any] = field(default_factory=dict)  # over ARM_OPEN, send only
    name: str = ""
    expect_close: bool = False  # a close by the server ends this phase without failing

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"phase kind must be one of {KINDS} (got {self.kind!r})")
        if not (isinstance(self.duration, int | float) and math.isfinite(self.duration)):
            raise ValueError(f"phase duration must be a finite number (got {self.duration!r})")
        if self.duration < 0:
            raise ValueError(f"phase duration must be >= 0 (got {self.duration})")
        for block, defaults in (("base", BASE_OPEN), ("arm", ARM_OPEN)):
            fields = getattr(self, block)
            if fields and self.kind != SEND:
                raise ValueError(f"a {self.kind} phase sends nothing, so it takes no {block}")
            unknown = set(fields) - set(defaults)
            if unknown:
                raise ValueError(f"unknown {block} fields: {sorted(unknown)}")


def load_scenario(scenario: str | Path | Iterable[Phase | Mapping[str, Any]]) -> list[Phase]:
    """Phases from a JSON file path, or from a list of Phase or phase dicts. Raises ValueError."""
    if isinstance(scenario, str | Path):
        scenario = json.loads(Path(scenario).read_text())
    if isinstance(scenario, Mapping):
        scenario = scenario.get("phases")
    if isinstance(scenario, str | bytes) or not isinstance(scenario, Iterable):
        raise ValueError("a scenario is a list of phases")
    phases = []
    for p in scenario:
        if isinstance(p, Mapping):
            try:
                p = Phase(**p)
            except TypeError as e:
                raise ValueError(f"bad phase {dict(p)}: {e}") from e
        if not isinstance(p, Phase):
            raise ValueError(f"not a phase: {p!r}")
        phases.append(p)
    return phases


def teleop_frame(seq: int, phase: Phase, timestamp_ms: int) -> dict[str, Any]:
    """The teleop v1 message a send phase sends as frame `seq` of its socket."""
    return {
        "v": PROTOCOL_VERSION,
        "type": "teleop",
        "seq": seq,
        "timestamp": timestamp_ms,
        "base": {**BASE_OPEN, **phase.base},
        "arm": {**ARM_OPEN, **phase.arm},
    }


class JsonLog:
    """One JSON object per line with the time.monotonic() time, flushed per line."""

    def __init__(self, path: str | Path):
        self._f = open(path, "w")  # noqa: SIM115 (closed by close())

    def write(self, event: str, **fields: Any) -> None:
        self._f.write(json.dumps({"t": time.monotonic(), "event": event, **fields}) + "\n")
        self._f.flush()

    def close(self) -> None:
        self._f.close()


class _Player:
    def __init__(self, url: str, log: JsonLog, rate_hz: float):
        self.url, self.log, self.period = url, log, 1.0 / rate_hz
        self.ws: Any = None
        self.reader: asyncio.Future | None = None
        self.seq = 0
        self._closed_by_us: set[int] = set()

    async def play(self, phases: list[Phase]) -> bool:
        try:
            for i, p in enumerate(phases):
                self.log.write("phase", index=i, name=p.name, kind=p.kind)
                if not await self._phase(p):
                    return False
            return True
        except (OSError, WebSocketException) as e:
            self.log.write("error", error=repr(e))
            return False
        finally:
            await self._close("scenario over")

    async def _phase(self, p: Phase) -> bool:
        end = time.monotonic() + p.duration
        if p.kind == DISCONNECT:
            await self._close("disconnect phase")
            await asyncio.sleep(max(0.0, end - time.monotonic()))
            return True
        if self.ws is None:
            await self._open()
        if p.kind == SILENCE:
            await asyncio.wait({self.reader}, timeout=max(0.0, end - time.monotonic()))
        else:
            await self._send_until(p, end)
        if not self.reader.done():
            return True
        self.ws = self.reader = None  # the server closed it during this phase
        if not p.expect_close:
            self.log.write("fail", reason=f"the server closed the socket during phase {p.name!r}")
            return False
        return True

    async def _send_until(self, p: Phase, end: float) -> None:
        t0, k = time.monotonic(), 0
        while not self.reader.done():
            due = t0 + k * self.period
            if due >= end:
                return
            await asyncio.sleep(max(0.0, due - time.monotonic()))
            if self.reader.done():
                return
            frame = teleop_frame(self.seq, p, int(time.time() * 1000))
            t = time.monotonic()
            try:
                await self.ws.send(json.dumps(frame))
            except ConnectionClosed:
                await asyncio.wait({self.reader}, timeout=CLOSE_WAIT_S)
                return
            self.log.write("sent", frame=frame, t_send=t)
            self.seq += 1
            # a late frame does not make the next ones burst: skip the ticks already missed
            k = max(k + 1, math.floor((time.monotonic() - t0) / self.period))

    async def _open(self) -> None:
        self.log.write("connect", url=self.url)
        self.ws = await websockets.connect(self.url, open_timeout=OPEN_TIMEOUT_S)
        self.seq = 0
        self.reader = asyncio.ensure_future(self._read(self.ws))

    async def _read(self, ws: Any) -> None:
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (TypeError, ValueError):
                    msg = None
                if isinstance(msg, dict) and msg.get("type") == "status":
                    self.log.write("status", status=msg)
                else:
                    self.log.write("recv", raw=raw if isinstance(raw, str) else repr(raw))
        except ConnectionClosed:
            pass
        by = "client" if id(ws) in self._closed_by_us else "server"
        self.log.write("close", code=ws.close_code, reason=ws.close_reason, by=by)

    async def _close(self, reason: str) -> None:
        if self.ws is None:
            return
        ws, reader = self.ws, self.reader
        self.ws = self.reader = None
        if not reader.done():
            self._closed_by_us.add(id(ws))
        with contextlib.suppress(OSError, WebSocketException):
            await ws.close(1000, reason)
        await asyncio.wait({reader}, timeout=CLOSE_WAIT_S)


async def play(
    url: str,
    scenario: str | Path | Iterable[Phase | Mapping[str, Any]],
    log_path: str | Path,
    rate_hz: float = RATE_HZ,
) -> bool:
    """Plays the scenario against `url`, logging to `log_path`. True unless it failed."""
    phases = load_scenario(scenario)
    log = JsonLog(log_path)
    try:
        ok = await _Player(url, log, rate_hz).play(phases)
        log.write("done", ok=ok)
    finally:
        log.close()
    return ok


def run(
    url: str,
    scenario: str | Path | Iterable[Phase | Mapping[str, Any]],
    log_path: str | Path,
    rate_hz: float = RATE_HZ,
) -> bool:
    """play() on a fresh event loop."""
    return asyncio.run(play(url, scenario, log_path, rate_hz))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--url", required=True, help="the robot bridge, e.g. ws://127.0.0.1:9110")
    ap.add_argument("--scenario", required=True, help="JSON file: a list of phases")
    ap.add_argument("--log", default="spectacles_standin_lens.jsonl", help="JSON-lines log")
    ap.add_argument("--rate", type=float, default=RATE_HZ, help="teleop rate, Hz")
    args = ap.parse_args(argv)
    try:
        phases = load_scenario(args.scenario)
    except (OSError, ValueError) as e:
        ap.error(f"scenario {args.scenario}: {e}")
    ok = run(args.url, phases, args.log, args.rate)
    print(f"lens stand-in: {'ok' if ok else 'FAILED'}, log in {args.log}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

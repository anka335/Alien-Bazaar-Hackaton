"""Teleoperate the real Leo Rover's base from the Snap Spectacles lens.

    uv run python -m sorter.spectacles                      # on the rover or a laptop on its Wi-Fi
    uv run python -m sorter.spectacles probe --vx 0.1 --seconds 2    # drive without the glasses

The lens reaches this over wss: `ngrok http 9100 --url <the lens's ngrok domain>` alongside
(scripts/spectacles_base.sh starts both). See docs/rover/spectacles-teleop.md.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import sys
import time

from sorter.spectacles.server import LensServer, RoverLink, RoverTopics
from sorter.spectacles.session import BaseSession, Limits


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="python -m sorter.spectacles", description=__doc__.split("\n")[0]
    )
    sub = p.add_subparsers(dest="cmd")
    p.add_argument("--rosbridge", default="ws://10.0.0.1:9090", help="the rover's rosbridge")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument(
        "--port", type=int, default=9100, help="the lens's WebSocket (ngrok forwards here)"
    )
    p.add_argument("--max-vx", type=float, default=0.20, help="m/s forward (protocol max 0.35)")
    p.add_argument("--max-reverse", type=float, default=0.10, help="m/s back (protocol max 0.15)")
    p.add_argument("--max-wz", type=float, default=0.6, help="rad/s (protocol max 0.8)")
    p.add_argument("--cmd-vel-topic", default="cmd_vel")
    p.add_argument("--odom-topic", default="merged_odom")
    p.add_argument("--wheel-states-topic", default="firmware/wheel_states")
    p.add_argument("-v", "--verbose", action="store_true")
    probe = sub.add_parser("probe", help="act as the lens: drive the base through a running bridge")
    probe.add_argument("--url", default="ws://127.0.0.1:9100")
    probe.add_argument("--vx", type=float, default=0.0, help="m/s")
    probe.add_argument("--wz", type=float, default=0.0, help="rad/s")
    probe.add_argument("--seconds", type=float, default=2.0)
    args = p.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    if args.cmd == "probe":
        return asyncio.run(_probe(args.url, args.vx, args.wz, args.seconds))
    try:
        limits = Limits(args.max_vx, args.max_reverse, args.max_wz)
    except ValueError as e:
        p.error(str(e))
    session = BaseSession(limits)
    topics = RoverTopics(args.cmd_vel_topic, args.odom_topic, args.wheel_states_topic)
    link = RoverLink(session, args.rosbridge, topics)
    link.start()
    signal.signal(signal.SIGTERM, _interrupt)  # kill / systemd: stop the rover like Ctrl+C
    logging.getLogger("sorter.spectacles").info(
        "limits: +%.2f / -%.2f m/s, %.2f rad/s; Ctrl+C stops", *vars(limits).values()
    )
    try:
        asyncio.run(LensServer(session, args.host, args.port).serve())
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    return 0


def _interrupt(*_):
    raise KeyboardInterrupt


async def _probe(url: str, vx: float, wz: float, seconds: float) -> int:
    """A stand-in lens: an open left hand, the pinch for `seconds`, then open again."""
    try:  # websockets >= 13
        from websockets.asyncio.client import connect
    except ImportError:  # websockets 12
        from websockets.client import connect

    async with connect(url) as ws:
        seq, t_end, last = 0, None, None

        async def recv():
            nonlocal last
            async for raw in ws:
                last = json.loads(raw)

        reader = asyncio.ensure_future(recv())
        t0 = time.monotonic()
        while True:
            elapsed = time.monotonic() - t0
            engaged = 0.5 <= elapsed < 0.5 + seconds
            if t_end is None and elapsed >= 1.0 + seconds:
                t_end = elapsed
            msg = {
                "v": 1,
                "type": "teleop",
                "seq": seq,
                "timestamp": int(time.time() * 1000),
                "base": {
                    "engaged": engaged,
                    "vx": vx if engaged else 0,
                    "wz": wz if engaged else 0,
                },
                "arm": {
                    "engaged": False,
                    "position": [0, 0, 0],
                    "orientation": [0, 0, 0, 1],
                    "gripper": 0,
                },
            }
            await ws.send(json.dumps(msg))
            if seq % 10 == 0:
                print(f"{elapsed:5.2f}s engaged={engaged} status={last}", flush=True)
            seq += 1
            if t_end is not None:
                break
            await asyncio.sleep(1 / 30)
        reader.cancel()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""The bridge end to end, in process: a lens socket in, twists out to a fake rosbridge."""

import asyncio
import json
import time

from test_spectacles_session import teleop

from sorter.spectacles.server import LensServer, RoverLink
from sorter.spectacles.session import BaseSession, Limits


class FakeBridge:
    def __init__(self, url):
        self.url = url
        self.published: list[tuple[str, dict]] = []
        self.handlers = {}

    def advertise(self, topic, msg_type):
        pass

    def subscribe(self, topic, msg_type, handler, throttle_ms=0):
        self.handlers[topic] = handler

    def publish(self, topic, msg):
        self.published.append((topic, msg))

    def close(self):
        pass


def test_a_pinch_drives_cmd_vel_and_a_close_stops_it():
    from websockets.asyncio.client import connect

    session = BaseSession(Limits())
    bridges = []

    def factory(url):
        bridges.append(FakeBridge(url))
        return bridges[-1]

    link = RoverLink(session, "ws://rover:9090", bridge_factory=factory)
    link.start()

    async def run():
        ready, stop = asyncio.Event(), asyncio.Event()
        server = LensServer(session, "127.0.0.1", 0)
        task = asyncio.ensure_future(server.serve(ready, stop))
        await ready.wait()
        while not bridges:
            await asyncio.sleep(0.01)
        bridge = bridges[0]
        bridge.handlers["merged_odom"]({})  # the rover is present before the lens connects
        async with connect(f"ws://127.0.0.1:{server.port}") as ws:
            statuses = []
            for seq in range(20):
                bridge.handlers["merged_odom"]({})
                await ws.send(json.dumps(teleop(seq, True, 0.1, 0.2)))
                await asyncio.sleep(0.02)
            for _ in range(3):
                statuses.append(json.loads(await ws.recv()))
        await asyncio.sleep(0.4)
        stop.set()
        await task
        return bridge, statuses

    try:
        bridge, statuses = asyncio.run(run())
    finally:
        link.close()
    twists = [(m["linear"]["x"], m["angular"]["z"]) for t, m in bridge.published if t == "cmd_vel"]
    assert (0.1, 0.2) in twists
    assert twists[-1] == (0.0, 0.0)
    assert any(s["base"] == "driving" for s in statuses)
    time.sleep(0.1)
    assert session.command()[0] is False


def test_the_rosbridge_client_against_a_fake_rover():
    """The real client: advertise + subscribe on connect, odometry in, twists out."""
    import threading

    from websockets.sync.server import serve

    from sorter.spectacles.server import _rosbridge

    got: list[dict] = []

    def rover(ws):
        for raw in ws:
            op = json.loads(raw)
            got.append(op)
            if op["op"] == "subscribe" and op["topic"] == "merged_odom":
                ws.send(json.dumps({"op": "publish", "topic": "merged_odom", "msg": {}}))

    with serve(rover, "127.0.0.1", 0) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        port = server.socket.getsockname()[1]
        session = BaseSession(Limits())
        link = RoverLink(session, f"ws://127.0.0.1:{port}", bridge_factory=_rosbridge)
        link.start()
        deadline = time.monotonic() + 3
        while not session.rover_present() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert session.rover_present()
        session.on_connect()
        session.on_message(teleop(0, True, 0.1, 0.0))
        time.sleep(0.12)
        link.close()
        server.shutdown()
    ops = [(o["op"], o["topic"]) for o in got]
    assert ("advertise", "cmd_vel") in ops and ("subscribe", "merged_odom") in ops
    sent = [o["msg"]["linear"]["x"] for o in got if o["op"] == "publish"]
    assert 0.1 in sent and sent[-1] == 0.0

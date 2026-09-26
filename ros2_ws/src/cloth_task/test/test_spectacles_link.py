import asyncio
import json
import os
import sys
import threading

import numpy as np
import pytest

websockets = pytest.importorskip("websockets")

from cloth_task.spectacles_link import REFUSED_CODE, LensLink  # noqa: E402
from cloth_task.spectacles_session import Session  # noqa: E402


def _rebot():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import config, kinematics

    return kinematics, config


K, C = _rebot()
Q0 = np.radians([10.0, 50.0, 70.0, -20.0, 10.0, 5.0])


def teleop(seq, arm=False):
    return json.dumps(
        {
            "v": 1,
            "type": "teleop",
            "seq": seq,
            "timestamp": 0.0,
            "base": {"engaged": False, "vx": 0.0, "wz": 0.0},
            "arm": {
                "engaged": arm,
                "position": [0.0, 0.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "gripper": 0.5,
            },
        }
    )


class Rig:
    def __init__(self, teleop_mode=True):
        self.joints = []
        self.session = Session(K, C.JOINT_LIMITS_RAD, self.joints.append, lambda g: None)
        self.session.on_joint_state(Q0)
        self.session.on_teleop_mode(teleop_mode)
        self.link = LensLink(self.session, threading.Lock(), port=0, status_hz=20.0, log=str)

    @property
    def url(self):
        return f"ws://127.0.0.1:{self.link.port}"


def run(rig, scenario):
    async def main():
        server = asyncio.ensure_future(rig.link.serve())
        while not rig.link.ready.is_set():
            await asyncio.sleep(0.01)
        try:
            await asyncio.wait_for(scenario(rig), 10.0)
        finally:
            rig.link.stop()
            await server

    asyncio.run(main())


async def recv_status(ws):
    return json.loads(await asyncio.wait_for(ws.recv(), 2.0))


def test_accepted_lens_gets_a_status_per_teleop_and_moves_the_arm():
    async def scenario(rig):
        async with websockets.connect(rig.url) as ws:
            rig.session.on_joint_state(Q0)
            await ws.send(teleop(3, arm=True))
            st = await recv_status(ws)
            assert st["echoSeq"] == 3 and st["arm"] == "tracking" and st["fault"] is None
            assert len(rig.joints) == 1

    run(Rig(), scenario)


def test_undecodable_teleop_is_malformed():
    async def scenario(rig):
        async with websockets.connect(rig.url) as ws:
            await ws.send(teleop(1))
            assert (await recv_status(ws))["echoSeq"] == 1
            await ws.send("{not json")
            assert (await recv_status(ws))["echoSeq"] == 1

    run(Rig(), scenario)


def test_lens_is_refused_until_teleop_mode_is_on():
    async def scenario(rig):
        async with websockets.connect(rig.url) as ws:
            with pytest.raises(websockets.exceptions.ConnectionClosed) as e:
                await asyncio.wait_for(ws.recv(), 2.0)
            assert e.value.rcvd.code == REFUSED_CODE

    run(Rig(teleop_mode=False), scenario)


def test_silence_pushes_a_timeout_status():
    async def scenario(rig):
        async with websockets.connect(rig.url) as ws:
            await ws.send(teleop(0))
            await recv_status(ws)
            await asyncio.sleep(0.3)
            st = None
            for _ in range(5):
                st = await recv_status(ws)
                if st["fault"]:
                    break
            assert st == {
                "v": 1,
                "type": "status",
                "echoSeq": 0,
                "base": "fault",
                "arm": "fault",
                "fault": "timeout",
            }

    run(Rig(), scenario)


def test_a_new_lens_replaces_the_old_one():
    async def scenario(rig):
        async with websockets.connect(rig.url) as old:
            rig.session.on_joint_state(Q0)
            await old.send(teleop(0, arm=True))
            assert (await recv_status(old))["arm"] == "tracking"
            async with websockets.connect(rig.url) as new:
                with pytest.raises(websockets.exceptions.ConnectionClosed):
                    while True:
                        await asyncio.wait_for(old.recv(), 2.0)
                rig.session.on_joint_state(Q0)
                await new.send(teleop(0, arm=True))
                st = await recv_status(new)
                assert st["arm"] == "holding" and st["fault"] is None
                assert len(rig.joints) == 1

    run(Rig(), scenario)

"""The Mission tab's real-rover loop, with the nav sim behind `RealSession` and a stand-in arm."""

import dataclasses
import math
import time
from types import SimpleNamespace

from sorter.core.config import load_config
from sorter.core.types import ColorClass, Command, OperatorMode, Phase
from sorter.mission.control import MissionControl
from sorter.mission.mission import hide_sock, socks_in_reach
from sorter.nav.camera import OakD
from sorter.nav.real import RealSession
from sorter.nav.scenario import make
from sorter.nav.sim import RoverSim


class FakeArmRun:
    """The state machine as the mission sees it: START in the load mode loads every sock in the
    arm's zone; in the unload mode it sorts everything loaded so far."""

    def __init__(self, cfg, sim):
        self.cfg, self.sim = cfg, sim
        self.mode, self.phase, self.error = "idle", Phase.IDLE, None
        self.counters = dict.fromkeys(ColorClass, 0)
        self.on_floor = set(range(len(sim.spec.socks)))
        self.loaded = dict.fromkeys(ColorClass, 0)
        self.op = OperatorMode.LOAD
        self.hub = SimpleNamespace(set_mode=self.set_mode, send=self.send)

    def set_mode(self, mode):
        self.op = mode

    def send(self, cmd):
        if cmd is Command.START and self.op is OperatorMode.UNLOAD:
            self.counters = dict(self.loaded)
            self.phase, self.mode = Phase.DONE, "idle"
        elif cmd is Command.START:
            self.counters = dict.fromkeys(ColorClass, 0)
            for i, p in socks_in_reach(self.cfg, self.sim, self.on_floor):
                hide_sock(self.sim, i)
                self.on_floor.discard(i)
                self.counters[p.color] += 1
                self.loaded[p.color] += 1
            self.phase, self.mode = Phase.DONE, "idle"


def test_real_loop_collects_and_reaches_the_station():
    cfg = load_config()
    spec = make("mission", 1)
    sx, sy, _ = spec.station  # started 1.5 m in front of the station, facing it
    spec = dataclasses.replace(spec, rover=(sx + 1.5, sy + 0.2, math.pi))
    sim = RoverSim(spec, cfg.nav)
    arm = FakeArmRun(cfg, sim)
    system = SimpleNamespace(cfg=cfg, hub=arm.hub)
    mc = MissionControl(system, arm, sim=False)
    # the camera's renderer is made in the mission's thread: an OpenGL context is per thread
    mc.make_session = lambda nav: RealSession(nav, base=_Base(sim), camera=OakD(sim, nav.camera))
    b = cfg.nav.boxes
    cfg.nav.boxes = b.model_copy(update={"tag_size_m": b.station_tag_m, "memory": "/nonexistent"})
    mc.start(capacity=6, detector="classic")
    t0 = time.time()
    while mc.running and time.time() - t0 < 300:
        time.sleep(0.2)
    st = mc.status()
    assert st["loaded"] >= 3, st
    assert st["phase"] == "done", st
    assert sum(st["sorted"].values()) == st["loaded"], st


class _Base:
    """The nav sim as the real Leo base: `close` stops it."""

    def __init__(self, sim):
        self._sim = sim

    def __getattr__(self, name):
        return getattr(self._sim, name)

    def close(self):
        self._sim.set_cmd(0.0, 0.0)

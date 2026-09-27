"""The unload loop end to end on the simulator, on the real rover's geometry, with the station
off its place: judged by where the sock really lands."""

import math

import pytest

from sorter.app import build_system
from sorter.core.types import ColorClass, Command, OperatorMode, Phase

pytest.importorskip("mujoco")


def test_unloads_a_sock_into_its_bin_found_by_the_camera():
    from sorter.orchestrator.state_machine import StateMachine
    from sorter.sim.scenes.unload.bench import _truth_bins, _where
    from sorter.sim.scenes.unload.rover import rover_config

    cfg = rover_config(
        {
            "sim": {
                "realtime": 0,
                "seed": 3,
                "unload": {
                    "cargo": {"light": 0, "dark": 0, "colored": 1},
                    "sock_mm": [200, 90],
                    "station_mm": 30,
                    "station_deg": 5,
                    "bin_mm": 15,
                    "bin_deg": 8,
                },
            },
            "arm": {"speed_scale": 1.4},
            "state_machine": {"save_runs": False},
        }
    )
    s = build_system(cfg, sim=True)
    world = s.world
    try:
        s.camera.start()
        sm = StateMachine(s)
        truth = _truth_bins(world)
        s.hub.set_mode(OperatorMode.UNLOAD)
        s.hub.send(Command.START)
        sm.poll()
        t0 = world.time()
        while sm.mode != "idle" and sm.phase is not Phase.ERROR and world.time() - t0 < 200:
            sm.poll()
        assert sm.phase is Phase.DONE, sm.error
        assert _where(world, 0, truth) == ("laundry", "colored")
        assert sm.counters[ColorClass.COLORED] == 1
        loop = sm.loops[OperatorMode.UNLOAD]
        for color, b in loop.bins.items():
            assert b.found and math.dist(b.center, truth[color][:2]) < 5
    finally:
        s.camera.close()
        world.stop()

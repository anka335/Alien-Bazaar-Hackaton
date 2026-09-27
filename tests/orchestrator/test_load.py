"""The load loop on the simulator, end to end: scan all around, pick, drop, check."""

import numpy as np
import pytest

from sorter.app import build_system
from sorter.core.types import ArmPoint, ColorClass, Command, OperatorMode, Phase
from sorter.orchestrator.load import Target
from sorter.orchestrator.state_machine import StateMachine

pytest.importorskip("mujoco")


def _run(sim_config, socks, time_limit_s=400.0, **sim):
    cfg = sim_config.model_copy(deep=True)
    cfg.sim.scenes = ["load"]
    cfg.sim.load.socks = socks
    for k, v in sim.items():
        setattr(cfg.sim, k, v)
    system = build_system(cfg, sim=True)
    system.camera.start()
    sm = StateMachine(system)
    system.hub.set_mode(OperatorMode.LOAD)
    system.hub.send(Command.START)
    world = system.world
    t0 = world.time()
    try:
        while world.time() - t0 < time_limit_s:
            sm.poll()
            if sm.phase in (Phase.DONE, Phase.ERROR) and sm.mode != "running":
                break
        return sm, [world.location(it.id)[0] for it in world.items]
    finally:
        system.camera.close()
        world.stop()


def test_socks_all_around_end_up_in_the_box(sim_config):
    sm, where = _run(sim_config, [ColorClass.DARK, ColorClass.LIGHT], seed=3)
    assert sm.phase is Phase.DONE, sm.error
    assert where == ["cargo", "cargo"]
    assert sum(sm.counters.values()) == 2  # counted only what is in the box


def test_a_sock_that_never_comes_up_is_left_and_the_run_ends(sim_config):
    sim_config.load.max_attempts = 2
    sm, where = _run(sim_config, [ColorClass.COLORED], seed=3, miss_prob=1.0)
    assert sm.phase is Phase.DONE, sm.error
    assert where == ["floor"]
    assert sum(sm.counters.values()) == 0
    assert len(sm.loops[OperatorMode.LOAD].r.given_up) == 1


def _seen(x, y, cloth):
    return Target(ArmPoint(x, y, -190.0), None, ColorClass.LIGHT, 1.0, False, 0.0, np.array(cloth))


def test_a_sock_seen_twice_is_one_sock_though_its_grasp_point_moved():
    # a 200 mm sock along x: seen cut off (grasp near its end), then whole (grasp at its middle)
    cloth = [(x, 0.0) for x in range(200, 401, 10)]
    first, closer = _seen(380, 0, cloth[15:]), _seen(300, 5, cloth)
    assert first.same_sock(closer, 30.0) is not None  # 80 mm apart, one sock
    other = _seen(300, 150, [(x, 150.0) for x in range(200, 401, 10)])
    assert closer.same_sock(other, 30.0) is None  # the sock beside it

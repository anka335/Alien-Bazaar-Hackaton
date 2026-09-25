"""Smoke test: the loop sorts every sim item end to end."""

import time

import pytest

from sorter.app import build_system
from sorter.core.types import ColorClass, Command, Phase
from sorter.orchestrator.state_machine import StateMachine


def _run_to_done(sm, hub, timeout_s=10.0):
    hub.send(Command.START)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        sm.poll()
        if sm.phase is Phase.DONE or sm.phase is Phase.ERROR:
            return
    pytest.fail(f"no DONE within {timeout_s} s, phase {sm.phase}")


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_sorts_all_items(sim_config, seed):
    sim_config.sim.seed = seed
    system = build_system(sim_config, sim=True)
    sm = StateMachine(system)
    _run_to_done(sm, system.hub)

    status = system.hub.status()
    assert status.phase is Phase.DONE and status.mode == "idle", status.error
    world = system.world
    assert all(it.location == "bin" and it.bin is it.color for it in world.items)
    expected = {c: sum(it.color is c for it in world.items) for c in ColorClass}
    assert status.counters == expected

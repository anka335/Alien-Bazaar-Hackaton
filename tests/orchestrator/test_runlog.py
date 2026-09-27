"""Run log: files per decision, readable by blocks 3 and 4."""

# ruff: noqa: E402
import pytest

pytest.skip(
    "rover stage 0: drove the table loop; rewrite on the load / unload loops (stages A, B)",
    allow_module_level=True,
)

import json

from sorter.app import build_system
from sorter.core.io import load_observation
from sorter.core.types import Command, Phase, Zone
from sorter.orchestrator.state_machine import StateMachine


def _run(sim_config, tmp_path, save_runs=True):
    cfg = sim_config.model_copy(deep=True)
    cfg.sim.miss_prob = 0.0
    cfg.sim.double_prob = 0.0
    cfg.state_machine.save_runs = save_runs
    cfg.state_machine.runs_dir = tmp_path / "runs"
    system = build_system(cfg, sim=True)
    sm = StateMachine(system)
    system.hub.send(Command.START)
    for _ in range(2000):
        sm.poll()
        if sm.phase in (Phase.DONE, Phase.ERROR):
            break
    assert sm.phase is Phase.DONE, sm.error
    return sm


def test_run_log_files(sim_config, tmp_path):
    sm = _run(sim_config, tmp_path)
    run_dir = tmp_path / "runs" / sm.run_id

    run = json.loads((run_dir / "run.json").read_text())
    assert run["end_reason"] == "done"
    assert run["counters"] == {c.value: n for c, n in sm.counters.items()}
    assert run["config"]["state_machine"]["save_runs"] is True

    first = json.loads((run_dir / "0001_sense_bg.json").read_text())
    assert first["observation"] == "0001_sense_bg.npz"
    assert first["result"]["items"] == [] and first["next_phase"] == "look_box"
    assert load_observation(run_dir / "0001_sense_bg").zone is Zone.BACKGROUND
    assert (run_dir / "0001_sense_bg.png").is_file()

    records = [json.loads(p.read_text()) for p in sorted(run_dir.glob("0*.json"))]
    resolved = [r["resolved"] for r in records if r.get("resolved")]
    assert resolved.count("placed from the box") == len(sm.s.world.items)
    assert sum(r.startswith("verified drop") for r in resolved) == len(sm.s.world.items)

    # the box is confirmed empty twice within the last cycle: the second gets a suffix
    last = records[-1]
    assert last["phase"] == "sense_box" and last["next_phase"] == "done"
    assert (run_dir / f"{last['cycle']:04d}_sense_box_2.json").is_file()


def test_no_run_log_when_disabled(sim_config, tmp_path):
    _run(sim_config, tmp_path, save_runs=False)
    assert not (tmp_path / "runs").exists()

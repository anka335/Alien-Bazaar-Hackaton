"""Run log: a file per decision of the load loop, readable by the analysis tools."""

import json

import pytest

from sorter.app import build_system
from sorter.core.io import load_observation
from sorter.core.types import ColorClass, Command, OperatorMode, Phase, Zone
from sorter.orchestrator.state_machine import StateMachine

pytest.importorskip("mujoco")


def _run(sim_config, tmp_path, save_runs=True):
    cfg = sim_config.model_copy(deep=True)
    cfg.sim.scenes = ["load"]
    cfg.sim.seed = 3
    cfg.sim.load.socks = [ColorClass.DARK]
    cfg.sim.load.area = "view"
    cfg.state_machine.save_runs = save_runs
    cfg.state_machine.runs_dir = tmp_path / "runs"
    system = build_system(cfg, sim=True)
    system.camera.start()
    sm = StateMachine(system)
    system.hub.set_mode(OperatorMode.LOAD)
    system.hub.send(Command.START)
    t0 = system.world.time()
    try:
        while system.world.time() - t0 < 300:
            sm.poll()
            if sm.phase in (Phase.DONE, Phase.ERROR) and sm.mode != "running":
                break
    finally:
        system.camera.close()
        system.world.stop()
    assert sm.phase is Phase.DONE, sm.error
    return sm


def test_run_log_files(sim_config, tmp_path):
    sm = _run(sim_config, tmp_path)
    run_dir = tmp_path / "runs" / sm.run_id
    assert sm.run_id.endswith("load-" + sm.run_id.rsplit("-", 1)[1])

    run = json.loads((run_dir / "run.json").read_text())
    assert run["end_reason"] == "done"
    assert run["counters"] == {c.value: n for c, n in sm.counters.items()}
    assert run["counters"]["dark"] == 1
    assert run["config"]["state_machine"]["save_runs"] is True

    records = [json.loads(p.read_text()) for p in sorted(run_dir.glob("0*.json"))]
    first = records[0]
    assert first["phase"] == "sense_floor"
    stem = first["observation"].removesuffix(".npz")
    assert load_observation(run_dir / stem).zone is Zone.FLOOR
    assert (run_dir / f"{stem}.png").is_file()
    assert any(r["phase"] == "check_load" and r["summary"].startswith("verified") for r in records)
    assert records[-1]["next_phase"] == "done"


def test_no_run_log_when_disabled(sim_config, tmp_path):
    _run(sim_config, tmp_path, save_runs=False)
    assert not (tmp_path / "runs").exists()

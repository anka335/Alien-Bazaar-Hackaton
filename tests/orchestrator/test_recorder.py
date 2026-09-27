import csv
import json
import time

from sorter.app import build_system
from sorter.core.types import Decision, Overlay, Phase, Zone
from sorter.orchestrator.recorder import Recorder


def test_recorder_writes_the_session(sim_config, tmp_path):
    system = build_system(sim_config, sim=True)
    try:
        system.camera.start()
        rec = Recorder(system, tmp_path, video_fps=20).start(["test"])
        system.arm.start()
        obs = system.observer.observe(Zone.FLOOR)
        system.hub.publish_decision(Decision(Phase.SCAN, obs, Overlay(), "one sock"))
        time.sleep(0.5)
        rec.close()
    finally:
        system.arm.shutdown()
        system.camera.close()
        system.world.stop()

    d = rec.dir
    assert json.loads((d / "session.json").read_text())["argv"] == ["test"]
    events = [json.loads(line) for line in (d / "events.jsonl").read_text().splitlines()]
    calls = {e["call"] for e in events if e["type"] == "return"}
    assert {"arm.start", "arm.look", "observer.observe"} <= calls
    decision = next(e for e in events if e["type"] == "decision")
    assert decision["summary"] == "one sock" and (d / "decisions" / decision["image"]).is_file()
    rows = list(csv.DictReader((d / "telemetry.csv").open()))
    assert rows and rows[-1]["torque"] == "1"
    assert (d / "wrist_000.mp4").stat().st_size > 0
    assert "arm started" in (d / "sorter.log").read_text()

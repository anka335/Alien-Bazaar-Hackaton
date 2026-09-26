import math

from cloth_task.status_web import PIPELINE, snapshot


def test_snapshot_derives_phase_progress_and_arm_state():
    now = 1000.0
    state = {
        "status": {"phase": "trial_grasp", "phase_since": 990.0, "started": 900.0, "placed": 1},
        "status_t": 999.0,
        "joints": {**{f"joint{i}": math.radians(10 * i) for i in range(1, 7)}, "joint_left": 0.004},
        "joints_t": 999.5,
        "logs": [{"t": 1.0, "level": "info", "node": "task_supervisor_node", "msg": "hi"}],
    }
    d = snapshot(state, now)
    assert d["phase_index"] == PIPELINE.index("trial_grasp")
    assert d["phase_for_s"] == 10.0 and d["running_for_s"] == 100.0
    assert d["joints_deg"] == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert d["finger_mm"] == 4.0
    assert d["connected"] and d["logs"][0]["msg"] == "hi"
    assert d["live_age_s"] is None  # no camera image yet


def test_snapshot_before_the_supervisor_talks():
    d = snapshot({"logs": []}, 5.0)
    assert not d["connected"] and d["phase"] is None and d["phase_index"] == -1
    assert d["joints_deg"] == [] and d["finger_mm"] is None

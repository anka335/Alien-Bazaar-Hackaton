import json
import os
import socket
import sys
import threading
import time

import numpy as np
import pytest

pytest.importorskip("websockets")

from cloth_task.spectacles_link import REFUSED_CODE, LensLink  # noqa: E402
from cloth_task.spectacles_session import BASE_STALE_S, Session  # noqa: E402
from cloth_task.spectacles_standin_lens import (  # noqa: E402
    DISCONNECT,
    SILENCE,
    Phase,
    load_scenario,
    main,
    run,
)


def _rebot():
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    sys.path.insert(0, os.path.join(repo, "rebot_b601"))
    from rebot_b601 import config, kinematics

    return kinematics, config


K, C = _rebot()
Q0 = np.radians([10.0, 50.0, 70.0, -20.0, 10.0, 5.0])
LEFT = {"engaged": True, "vx": 0.1, "wz": 0.2}


class Rig:
    """A real LensLink and Session in a thread, with the rover present and the bridge's 50 Hz
    tick, as the robot bridge runs them."""

    def __init__(self, teleop_mode=True):
        self.base = []
        self.session = Session(
            K,
            C.JOINT_LIMITS_RAD,
            lambda q: None,
            lambda g: None,
            send_base=lambda vx, wz: self.base.append((vx, wz)),
            rover=True,
        )
        self.session.on_joint_state(Q0)
        self.session.on_odometry()
        self.session.on_teleop_mode(teleop_mode)
        self.lock = threading.Lock()
        self.link = LensLink(self.session, self.lock, port=0, status_hz=20.0, log=str)
        self._stop = threading.Event()

    @property
    def url(self):
        return f"ws://127.0.0.1:{self.link.port}"

    def __enter__(self):
        self._server = threading.Thread(target=self.link.run, daemon=True)
        self._server.start()
        assert self.link.ready.wait(5.0)
        self._ticker = threading.Thread(target=self._tick, daemon=True)
        self._ticker.start()
        return self

    def __exit__(self, *_):
        self._stop.set()
        self._ticker.join(2.0)
        self.link.stop()
        self._server.join(5.0)

    def _tick(self):
        while not self._stop.wait(0.02):
            with self.lock:
                self.session.on_joint_state(Q0)
                self.session.on_odometry()
                self.session.tick()


def read_log(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def events(log, name):
    return [e for e in log if e["event"] == name]


def collapse(values):
    out = []
    for v in values:
        if not out or out[-1] != v:
            out.append(v)
    return out


def test_idle_left_engaged_released_reports_idle_driving_idle(tmp_path):
    log_path = tmp_path / "lens.jsonl"
    with Rig() as rig:
        ok = run(
            rig.url,
            [Phase(0.3, name="idle"), Phase(0.4, base=LEFT, name="left"), Phase(0.3, name="open")],
            log_path,
        )
    log = read_log(log_path)
    assert ok and log[-1]["event"] == "done" and log[-1]["ok"] is True
    bases = [e["status"]["base"] for e in events(log, "status")]
    assert collapse(bases) == ["idle", "driving", "idle"]
    assert all(e["status"]["fault"] is None for e in events(log, "status"))
    assert (0.1, 0.2) in rig.base and rig.base[-1] == (0.0, 0.0)
    closes = events(log, "close")
    assert [(c["code"], c["by"]) for c in closes] == [(1000, "client")]


def test_frames_are_teleop_v1_at_30_hz_with_seq_up_by_one(tmp_path):
    log_path = tmp_path / "lens.jsonl"
    with Rig() as rig:
        before = time.time() * 1000
        assert run(rig.url, [Phase(0.5), Phase(0.5, base=LEFT)], log_path)
        after = time.time() * 1000
    sent = events(read_log(log_path), "sent")
    frames = [e["frame"] for e in sent]
    assert [f["seq"] for f in frames] == list(range(len(frames)))
    assert 27 <= len(frames) <= 31  # 1 s at 30 Hz
    assert all(before <= f["timestamp"] <= after for f in frames)
    assert frames[0] == {
        "v": 1,
        "type": "teleop",
        "seq": 0,
        "timestamp": frames[0]["timestamp"],
        "base": {"engaged": False, "vx": 0.0, "wz": 0.0},
        "arm": {
            "engaged": False,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "gripper": 0.0,
        },
    }
    assert frames[-1]["base"] == LEFT
    gaps = np.diff([e["t_send"] for e in sent])
    assert max(gaps) < 2.0 / 30


def test_silence_keeps_the_socket_open_and_sends_nothing(tmp_path):
    log_path = tmp_path / "lens.jsonl"
    silence = BASE_STALE_S + 0.2
    with Rig() as rig:
        assert run(
            rig.url,
            [Phase(0.3, base=LEFT), Phase(silence, kind=SILENCE), Phase(0.3, base=LEFT)],
            log_path,
        )
    log = read_log(log_path)
    assert len(events(log, "connect")) == 1
    sent = events(log, "sent")
    assert [e["frame"]["seq"] for e in sent] == list(range(len(sent)))  # no new socket
    gaps = np.diff([e["t_send"] for e in sent])
    assert max(gaps) >= silence - 0.01
    assert sum(g > 2.0 / 30 for g in gaps) == 1
    # statuses keep coming during the silence: base stale stops the base, with no fault
    i = int(np.argmax(gaps))
    t_last, t_next = sent[i]["t_send"], sent[i + 1]["t_send"]
    quiet = [
        e["status"] for e in events(log, "status") if t_last + BASE_STALE_S + 0.05 < e["t"] < t_next
    ]
    assert quiet and all(s["base"] == "idle" and s["fault"] is None for s in quiet)
    # and the base drives again, with no release
    bases = [e["status"]["base"] for e in events(log, "status")]
    assert collapse(bases)[-2:] == ["idle", "driving"]


def test_disconnect_closes_the_socket_and_the_next_phase_opens_a_new_one(tmp_path):
    log_path = tmp_path / "lens.jsonl"
    with Rig() as rig:
        assert run(
            rig.url,
            [Phase(0.3, base=LEFT), Phase(0.2, kind=DISCONNECT), Phase(0.2)],
            log_path,
        )
        stopped = list(rig.base)
    log = read_log(log_path)
    assert len(events(log, "connect")) == 2
    closes = events(log, "close")
    assert [(c["code"], c["by"]) for c in closes] == [(1000, "client"), (1000, "client")]
    seqs = [e["frame"]["seq"] for e in events(log, "sent")]
    restart = seqs.index(0, 1)
    assert seqs == list(range(restart)) + list(range(len(seqs) - restart))
    # nothing is sent between the close and the new socket
    t_close = closes[0]["t"]
    after = [e["t_send"] for e in events(log, "sent") if e["t_send"] > t_close]
    assert min(after) - t_close >= 0.2 - 0.01
    assert (0.1, 0.2) in stopped and stopped[-1] == (0.0, 0.0)


def test_a_close_by_the_server_fails_unless_expected(tmp_path):
    with Rig(teleop_mode=False) as rig:
        assert not run(rig.url, [Phase(0.5)], tmp_path / "a.jsonl")
        assert run(
            rig.url,
            [Phase(0.5, expect_close=True), Phase(0.1, kind=DISCONNECT)],
            tmp_path / "b.jsonl",
        )
        # a close during a silence counts the same
        assert not run(rig.url, [Phase(0.5, kind=SILENCE)], tmp_path / "c.jsonl")
    a = read_log(tmp_path / "a.jsonl")
    assert [(c["code"], c["by"]) for c in events(a, "close")] == [(REFUSED_CODE, "server")]
    assert events(a, "fail") and a[-1] == {"t": a[-1]["t"], "event": "done", "ok": False}
    b = read_log(tmp_path / "b.jsonl")
    assert [(c["code"], c["by"]) for c in events(b, "close")] == [(REFUSED_CODE, "server")]
    assert not events(b, "fail") and b[-1]["ok"] is True


def test_a_connection_error_fails(tmp_path):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]  # nothing listens here
    log_path = tmp_path / "lens.jsonl"
    assert not run(f"ws://127.0.0.1:{port}", [Phase(0.2)], log_path)
    log = read_log(log_path)
    assert events(log, "error") and log[-1]["ok"] is False


def test_the_command_line_plays_a_json_scenario(tmp_path, capsys):
    scenario = tmp_path / "scenario.json"
    scenario.write_text(
        json.dumps(
            [
                {"duration": 0.2, "name": "idle"},
                {"duration": 0.3, "base": LEFT, "name": "left"},
                {"duration": 0.2, "kind": "silence"},
                {"duration": 0.1, "kind": "disconnect"},
            ]
        )
    )
    log_path = tmp_path / "lens.jsonl"
    with Rig() as rig:
        code = main(["--url", rig.url, "--scenario", str(scenario), "--log", str(log_path)])
    nobody = ["--url", "ws://127.0.0.1:1", "--scenario", str(scenario)]
    failed = main([*nobody, "--log", str(tmp_path / "x.jsonl")])
    assert (code, failed) == (0, 1)
    assert "lens stand-in: ok" in capsys.readouterr().out
    log = read_log(log_path)
    assert [e["name"] for e in events(log, "phase")] == ["idle", "left", "", ""]
    assert "driving" in [e["status"]["base"] for e in events(log, "status")]


@pytest.mark.parametrize(
    "bad",
    [
        [{"duration": 1.0, "kind": "wave"}],
        [{"duration": -1.0}],
        [{"duration": float("nan")}],
        [{"duration": 1.0, "base": {"engaged": True, "vy": 0.1}}],
        [{"duration": 1.0, "kind": "silence", "base": LEFT}],
        [{"duration": 1.0, "speed": 3}],
        [{"kind": "send"}],
        "not a list",
    ],
)
def test_a_bad_scenario_is_refused(bad):
    with pytest.raises(ValueError):
        load_scenario(bad if not isinstance(bad, str) else {"phases": bad})

import signal
import subprocess

from sorter.camera import macos


def test_freeze_stops_and_resume_continues(monkeypatch):
    sent = []
    monkeypatch.setattr(macos.sys, "platform", "darwin")
    monkeypatch.setattr(macos.os, "geteuid", lambda: 0)
    monkeypatch.setattr(
        macos.subprocess,
        "run",
        lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout="101\n202\n"),
    )
    monkeypatch.setattr(macos.os, "kill", lambda pid, sig: sent.append((pid, sig)))
    monkeypatch.setattr(macos.atexit, "register", lambda f: None)
    f = macos.UvcAssistantFreeze()
    f.freeze()
    f.resume()
    f.resume()  # a second resume (close, then atexit) sends nothing
    assert sent == [
        (101, signal.SIGSTOP),
        (202, signal.SIGSTOP),
        (101, signal.SIGCONT),
        (202, signal.SIGCONT),
    ]


def test_freeze_needs_root(monkeypatch):
    monkeypatch.setattr(macos.sys, "platform", "darwin")
    monkeypatch.setattr(macos.os, "geteuid", lambda: 501)
    monkeypatch.setattr(macos.os, "kill", lambda *a: (_ for _ in ()).throw(AssertionError))
    macos.UvcAssistantFreeze().freeze()

"""Commands of the state machine after an arm fault and during a run, on a fake arm driver."""

from types import SimpleNamespace

import numpy as np
import pytest

from sorter.arm.controller import Controller
from sorter.core.config import load_config
from sorter.core.errors import ArmError, EStopped
from sorter.core.hub import Hub
from sorter.core.types import Command, OperatorMode, Phase
from sorter.orchestrator.state_machine import StateMachine


class Driver:
    """Jumps to the end of every path; a fault latches until clear_fault()."""

    def __init__(self, q0):
        self.q = np.asarray(q0, dtype=float)
        self.stopped = False
        self.connected = True
        self.latched: str | None = None

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def joints(self):
        return self.q.copy()

    def gripper(self):
        return 0.0

    def execute(self, wps, speed_scale):
        if self.stopped:
            raise EStopped("held")
        if self.latched:
            raise ArmError(f"arm is in a fault state: {self.latched}")
        self.q = np.asarray(wps[-1], dtype=float)

    def wait(self, seconds):
        if self.stopped:
            raise EStopped("held")

    def set_gripper(self, opening):
        if self.latched:
            raise ArmError(f"arm is in a fault state: {self.latched}")
        return opening

    def stop(self):
        self.stopped = True

    def resume(self):
        self.stopped = False

    def fault(self):
        return self.latched

    def clear_fault(self):
        self.latched = None


class _Camera:
    def latest(self):
        return None


@pytest.fixture
def rig():
    cfg = load_config(overrides={"state_machine": {"save_runs": False}})
    driver = Driver(cfg.poses["home"])
    arm = Controller(driver, cfg.arm, cfg.poses, cfg.zones)
    hub = Hub(_Camera(), on_hold=arm.hold, speed=arm, mode=OperatorMode.LOAD)
    sm = StateMachine(SimpleNamespace(cfg=cfg, arm=arm, hub=hub))
    return sm, arm, driver, hub


def _command(sm, hub, cmd):
    hub.send(cmd)
    sm._apply(hub.next_command(0))


def _fail_on_start(sm, hub, driver):
    _command(sm, hub, Command.START)
    driver.latched = "lost motor feedback"
    sm._run_phase()  # STARTING: home fails
    assert sm.phase is Phase.ERROR and sm.mode == "paused"


@pytest.mark.parametrize("cmd", [Command.RESET, Command.RESUME])
def test_reset_or_resume_clears_the_arm_fault(rig, cmd):
    sm, arm, driver, hub = rig
    _fail_on_start(sm, hub, driver)
    _command(sm, hub, cmd)
    assert driver.latched is None and sm.error is None
    assert sm.mode == "running" and sm.next is sm.loop.first


def test_stop_after_an_error_ends_the_run(rig):
    sm, arm, driver, hub = rig
    _fail_on_start(sm, hub, driver)
    _command(sm, hub, Command.STOP)
    assert sm.phase is Phase.IDLE and sm.mode == "idle" and sm.error is None
    assert driver.latched is None and not arm.held


def test_stop_ends_the_run_even_if_the_arm_cant_recover(rig, monkeypatch):
    sm, arm, driver, hub = rig
    _command(sm, hub, Command.START)

    def broken():
        raise ArmError("no path home")

    monkeypatch.setattr(arm, "recover", broken)
    _command(sm, hub, Command.STOP)
    assert sm.phase is Phase.IDLE and sm.mode == "idle" and "no path home" in sm.error


def test_stop_holds_the_arm_at_once_during_a_run(rig):
    sm, arm, driver, hub = rig
    hub.send(Command.STOP)  # nothing running: no hold
    assert not arm.held
    sm._apply(hub.next_command(0))
    _command(sm, hub, Command.START)
    hub.send(Command.STOP)
    assert arm.held  # the phase under way aborts with EStopped
    sm._apply(hub.next_command(0))
    assert sm.phase is Phase.IDLE and not arm.held

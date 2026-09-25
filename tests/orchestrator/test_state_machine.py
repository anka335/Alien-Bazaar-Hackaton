"""State machine on the simulator: happy path, every failure path, controls."""

import pytest

from sorter.app import build_system
from sorter.core.errors import TargetRejected
from sorter.core.types import (
    BoxResult,
    BoxStatus,
    ColorClass,
    Command,
    Overlay,
    Phase,
    PickResult,
    Zone,
)
from sorter.orchestrator.state_machine import StateMachine


@pytest.fixture
def make(sim_config):
    """Build (system, sm) on a deterministic sim: no misses, no double grasps by default."""

    def _make(**sim):
        cfg = sim_config.model_copy(deep=True)
        cfg.sim.miss_prob = 0.0
        cfg.sim.double_prob = 0.0
        for k, v in sim.items():
            setattr(cfg.sim, k, v)
        system = build_system(cfg, sim=True)
        return system, StateMachine(system)

    return _make


@pytest.fixture
def decisions(monkeypatch):
    """Returns a function that records every Decision the system publishes."""

    def _record(system):
        seen = []
        publish = system.hub.publish_decision

        def spy(d):
            seen.append(d)
            publish(d)

        monkeypatch.setattr(system.hub, "publish_decision", spy)
        return seen

    return _record


def run_until(sm, pred, max_polls=2000):
    for _ in range(max_polls):
        if pred(sm):
            return
        sm.poll()
    pytest.fail(f"condition not reached, phase {sm.phase} / {sm.mode}, error {sm.error}")


def is_stopped(sm):
    return sm.phase in (Phase.DONE, Phase.ERROR, Phase.HELD)


def run_to_done(sm, system):
    system.hub.send(Command.START)
    run_until(sm, is_stopped)
    assert sm.phase is Phase.DONE, sm.error


def assert_all_sorted(system):
    world = system.world
    assert all(it.location == "bin" and it.bin is it.color for it in world.items)
    expected = {c: sum(it.color is c for it in world.items) for c in ColorClass}
    status = system.hub.status()
    assert status.counters == expected
    assert status.phase is Phase.DONE and status.mode == "idle"


def patch_once(monkeypatch, obj, name, replacement, when=lambda *a, **k: True):
    """Replace obj.name with `replacement(original, *args)` on the first call where `when` holds."""
    original = getattr(obj, name)
    used = []

    def wrapper(*args, **kwargs):
        if not used and when(*args, **kwargs):
            used.append(True)
            return replacement(original, *args, **kwargs)
        return original(*args, **kwargs)

    monkeypatch.setattr(obj, name, wrapper)
    return used


def is_box(target, zone):
    return zone is Zone.BOX


def is_bg(target, zone):
    return zone is Zone.BACKGROUND


# --- happy path ---


def test_happy_path_sorts_everything(make, decisions):
    system, sm = make()
    system.cfg.state_machine.empty_confirmations = 3
    seen = decisions(system)
    run_to_done(sm, system)

    assert_all_sorted(system)
    assert sm.failures == 0 and sm.avoid == []
    box = [d for d in seen if d.phase is Phase.SENSE_BOX]
    assert [d.summary for d in box[-3:]] == [f"box empty ({i}/3)" for i in (1, 2, 3)]
    assert sm.last_cycle_s is not None and sm.last_cycle_s >= 0


def test_restart_after_done_resets_counters(make):
    system, sm = make()
    run_to_done(sm, system)
    first_run = sm.run_id
    system.hub.send(Command.START)
    sm.poll()
    assert sm.run_id != first_run
    assert all(n == 0 for n in sm.counters.values())


# --- failure paths ---


def test_missed_grasp_from_box_goes_to_avoid(make, monkeypatch):
    """The gripper reports an item, but it slipped: the background stays empty after the place."""
    system, sm = make()
    world = system.world

    def slip(pick, target, zone):
        pick(target, zone)
        for it in world.at("gripper"):
            it.location = "box"
        return PickResult(gripper_opening=0.3, likely_empty=False)

    patch_once(monkeypatch, system.arm, "pick", slip, when=is_box)
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.avoid or is_stopped(sm))
    assert len(sm.avoid) == 1 and sm.failures == 1

    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_empty_gripper_after_box_pick_retries_elsewhere(make, monkeypatch):
    system, sm = make()
    patch_once(
        monkeypatch,
        system.arm,
        "pick",
        lambda pick, t, z: PickResult(gripper_opening=0.0, likely_empty=True),
        when=is_box,
    )
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.avoid or is_stopped(sm))
    assert sm.failures == 1 and sm.next is Phase.LOOK_BOX

    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_failed_drop_is_not_counted_and_retried(make, monkeypatch):
    """The item falls back onto the background: blob count unchanged → no counter, retry."""
    system, sm = make()
    world = system.world

    def fall_back(drop, color):
        for it in world.at("gripper"):
            it.location = "background"

    patch_once(monkeypatch, system.arm, "drop_to_bin", fall_back)
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.failures == 1 or is_stopped(sm))
    assert sum(sm.counters.values()) == 0

    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_empty_gripper_after_bg_pick_retries(make, monkeypatch):
    system, sm = make()
    patch_once(
        monkeypatch,
        system.arm,
        "pick",
        lambda pick, t, z: PickResult(gripper_opening=0.0, likely_empty=True),
        when=is_bg,
    )
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.failures == 1 or is_stopped(sm))
    assert sm.next is Phase.LOOK_BG

    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_double_grasp_sorts_one_item_per_cycle(make, decisions):
    system, sm = make(double_prob=1.0)
    seen = decisions(system)
    run_to_done(sm, system)

    assert_all_sorted(system)
    assert any(d.summary.endswith("(2 on bg)") for d in seen)


def test_target_rejected_in_box_resenses_same_observation(make, monkeypatch, decisions):
    system, sm = make()
    seen = decisions(system)

    def reject(pick, target, zone):
        raise TargetRejected("outside the workspace")

    patch_once(monkeypatch, system.arm, "pick", reject, when=is_box)
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.avoid or is_stopped(sm))
    assert sm.next is Phase.SENSE_BOX and sm.failures == 1
    sm.poll()  # SENSE_BOX again

    first, second = [d for d in seen if d.phase is Phase.SENSE_BOX][:2]
    assert second.obs is first.obs
    assert first.summary != second.summary  # the rejected point is avoided

    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_target_rejected_on_background_is_an_error(make, monkeypatch):
    system, sm = make()

    def reject(pick, target, zone):
        raise TargetRejected("outside the workspace")

    patch_once(monkeypatch, system.arm, "pick", reject, when=is_bg)
    system.hub.send(Command.START)
    run_until(sm, is_stopped)
    assert sm.phase is Phase.ERROR and sm.mode == "paused"
    assert "outside the workspace" in system.hub.status().error


def test_no_grasp_clears_avoid_once(make, monkeypatch, decisions):
    system, sm = make()
    seen = decisions(system)
    patch_once(
        monkeypatch,
        system.arm,
        "pick",
        lambda pick, t, z: PickResult(gripper_opening=0.0, likely_empty=True),
        when=is_box,
    )
    patch_once(
        monkeypatch,
        system.box_detector,
        "detect",
        lambda detect, frame, avoid=(): BoxResult(BoxStatus.NO_GRASP, None, 0.5, Overlay()),
        when=lambda frame, avoid=(): bool(avoid),
    )
    run_to_done(sm, system)

    assert any(d.summary.startswith("no grasp, clearing 1") for d in seen)
    assert_all_sorted(system)


def test_no_grasp_without_avoid_is_an_error(make, monkeypatch):
    system, sm = make()
    monkeypatch.setattr(
        system.box_detector,
        "detect",
        lambda frame, avoid=(): BoxResult(BoxStatus.NO_GRASP, None, 0.5, Overlay()),
    )
    system.hub.send(Command.START)
    run_until(sm, is_stopped)
    assert sm.phase is Phase.ERROR and "no grasp candidate" in sm.error


def test_too_many_failures_pause_in_error_then_resume(make):
    system, sm = make(miss_prob=1.0)
    limit = system.cfg.state_machine.max_consecutive_failures
    system.hub.send(Command.START)
    run_until(sm, is_stopped)
    assert sm.phase is Phase.ERROR and sm.mode == "paused"
    assert sm.failures == limit and "consecutive failures" in sm.error

    system.world.cfg.miss_prob = 0.0
    system.hub.send(Command.RESUME)
    sm.poll()
    assert sm.mode == "running" and sm.failures == 0 and sm.error is None
    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_unexpected_exception_is_an_error_not_a_crash(make, monkeypatch):
    system, sm = make()
    patch_once(monkeypatch, system.color_classifier, "classify", lambda *a: 1 / 0)
    system.hub.send(Command.START)
    run_until(sm, is_stopped)
    assert sm.phase is Phase.ERROR and "division by zero" in sm.error

    system.hub.send(Command.RESET)
    sm.poll()
    run_until(sm, is_stopped)
    assert_all_sorted(system)


# --- controls ---


def test_hold_then_reset(make):
    system, sm = make()
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.next is Phase.PICK_FROM_BOX)
    system.hub.send(Command.HOLD)  # arm.hold() right away, from the "web thread"
    sm.poll()
    assert sm.phase is Phase.HELD and sm.mode == "paused"

    system.hub.send(Command.RESUME)  # not allowed while held
    sm.poll()
    assert sm.phase is Phase.HELD and sm.mode == "paused"

    system.hub.send(Command.RESET)
    sm.poll()
    assert sm.mode == "running"
    run_until(sm, is_stopped)
    assert_all_sorted(system)


def test_pause_step_stop(make):
    system, sm = make()
    hub = system.hub
    hub.send(Command.START)
    sm.poll()
    assert sm.phase is Phase.STARTING

    hub.send(Command.PAUSE)
    sm.poll()
    assert sm.mode == "paused" and sm.phase is Phase.STARTING and sm.next is Phase.LOOK_BG
    sm.poll()
    assert sm.phase is Phase.STARTING  # nothing runs while paused

    hub.send(Command.STEP)
    sm.poll()
    assert sm.phase is Phase.LOOK_BG and sm.next is Phase.SENSE_BG and sm.mode == "paused"
    assert hub.status().next_phase is Phase.SENSE_BG

    hub.send(Command.RESUME)
    run_until(sm, lambda sm: sum(sm.counters.values()) >= 1)
    counters = dict(sm.counters)
    hub.send(Command.STOP)
    sm.poll()
    assert sm.phase is Phase.IDLE and sm.mode == "idle" and sm.next is None
    assert hub.status().counters == counters  # kept until the next START
    assert system.world.looking_at is None  # homed


def test_stop_while_held_recovers(make):
    system, sm = make()
    system.hub.send(Command.START)
    run_until(sm, lambda sm: sm.next is Phase.DROP_TO_BIN)
    system.hub.send(Command.HOLD)
    sm.poll()
    assert sm.phase is Phase.HELD
    system.hub.send(Command.STOP)
    sm.poll()
    assert sm.phase is Phase.IDLE
    assert not system.world.at("gripper")  # recover() opened the gripper above the background

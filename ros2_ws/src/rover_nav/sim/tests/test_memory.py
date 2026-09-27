import math

import pytest
from leo_sim.agent import Agent, Asker
from leo_sim.jevomir import OracleScorer
from leo_sim.memory import Tracker
from leo_sim.runner import SimRunner

R = math.radians


def test_no_context_before_the_first_move():
    assert Tracker("the red ball").context() == ""


def test_last_seen_bearing_follows_the_turns():
    t = Tracker("the red ball")
    # seen in the left half (~30 deg left), then the rover turned 20 deg left
    t.update(0, "left", "left", R(20), "done", (0, 0, 0), (0, 0, R(20)))
    text = t.context()
    assert "in the previous photo, in the left half of the photo" in text
    assert "roughly straight ahead of where the robot now faces" in text  # 30 - 20 < 12
    assert "turned 20° left" in text
    # then 90 deg right without seeing it: now it is ~80 deg to the left
    t.update(1, "none", "right", R(90), "done", (0, 0, R(20)), (0, 0, R(-70)))
    assert "2 photos ago" in t.context()
    assert "about 100° to the left" in t.context()


def test_patterns():
    t = Tracker("the blue box")
    heading = 0.0
    for i, side in enumerate(["left", "right", "left", "right"]):
        turn = R(20) if side == "left" else -R(20)
        t.update(i, side, side, R(20), "done", (0, 0, heading), (0, 0, heading + turn))
        heading += turn
    assert "turning back and forth" in t.context()
    t = Tracker("the blue box")
    for i in range(9):
        h = i * R(40)
        t.update(i, "none", "left", R(40), "done", (0, 0, h), (0, 0, h + R(40)))
    text = t.context()
    assert "has not seen the blue box yet" in text
    t.update(9, "center", "forward", 0.4, "stalled", (0, 0, 0), (0.1, 0, 0))
    assert "blocked by an obstacle" in t.context()
    assert "drove 0.40 m forward (blocked)" in t.context()


def test_context_goes_before_every_question():
    class Echo:
        def __init__(self):
            self.questions = []

        def score(self, question, options, images, kind):
            self.questions.append(question)
            return [1.0 / len(options)] * len(options)

    echo = Echo()
    ask = Asker(echo, None, both_orders=True, context="Robot memory: x.")
    ask("where", "Where?", ["a", "b"])
    assert echo.questions == ["Robot memory: x.\nWhere?"] * 2
    assert ask.records[0]["question"] == "Where?"


def test_agent_with_memory_logs_it():
    runner = SimRunner(seed=None, realtime=0)
    try:
        agent = Agent(
            runner,
            OracleScorer(runner, "red_ball"),
            "red_ball",
            policy="direct",
            both_orders=False,
            max_steps=30,
            memory="prompt",
        )
        summary = agent.run()
    finally:
        runner.close()
    assert summary["status"] == "success" and summary["memory"] == "prompt"
    assert agent.steps[0]["memory"] == ""
    assert agent.steps[1]["memory"].startswith("Robot memory from its wheel odometry (step 2)")


class D:  # a Decision stand-in
    def __init__(self, action, amount, seen):
        self.action, self.amount, self.seen, self.reason = action, amount, seen, ""


def test_control_searches_where_it_was_last_seen():
    t = Tracker("the red ball")
    t.update(0, "right", "right", R(20), "done", (0, 0, 0), (0, 0, -R(20)))  # seen 30 right
    t.update(1, "none", "left", R(40), "done", (0, 0, -R(20)), (0, 0, R(20)))  # lost it
    d = t.adjust(D("left", R(40), "none"))  # the policy searches left by default
    assert d.action == "right"  # it was last seen ~50 deg to the right
    assert math.degrees(d.amount) == pytest.approx(40)


def test_control_halves_back_and_forth():
    t = Tracker("the red ball")
    t.update(0, "left", "left", R(20), "done", (0, 0, 0), (0, 0, R(20)))
    d = t.adjust(D("right", R(20), "right"))
    assert math.degrees(d.amount) == pytest.approx(10)
    assert "back and forth" in d.reason


def test_control_explores_after_a_full_circle():
    t = Tracker("the red ball")
    for i in range(9):
        t.update(i, "none", "left", R(40), "done", (0, 0, i * R(40)), (0, 0, (i + 1) * R(40)))
    d = t.adjust(D("left", R(40), "none"))
    assert (d.action, d.amount) == ("forward", 0.6)
    assert t.adjust(D("left", R(40), "none")).action == "left"  # counts again from here


def test_control_looks_back_once_then_keeps_sweeping():
    t = Tracker("the red ball")
    t.update(0, "center", "forward", 0.4, "done", (0, 0, 0), (0.4, 0, 0))  # seen ahead
    t.update(1, "none", "left", R(40), "done", (0.4, 0, 0), (0.4, 0, R(40)))  # lost it
    first = t.adjust(D("left", R(40), "none"))
    assert first.action == "right"  # back to where it was
    t.update(2, "none", "right", first.amount, "done", (0.4, 0, R(40)), (0.4, 0, 0))
    second = t.adjust(D("left", R(40), "none"))  # the policy wants left again
    assert second.action == "right"  # no swinging back: keep sweeping right

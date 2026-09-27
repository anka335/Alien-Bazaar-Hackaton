import json

import pytest
from leo_sim.agent import DIRECT_PROMPTS, Agent, Asker, DirectPolicy, GuidedPolicy
from leo_sim.jevomir import OracleScorer, pick_action
from leo_sim.runner import SimRunner


@pytest.fixture
def runner():
    r = SimRunner(seed=None, realtime=0)
    yield r
    r.close()


class Scripted:
    """Answers with fixed probabilities per question kind; records the option orders."""

    def __init__(self, answers):
        self.answers, self.calls = answers, []

    def score(self, question, options, images, kind):
        self.calls.append((kind, list(options)))
        return self.answers[kind](options)


def by_text(table):
    return lambda options: [table[o] for o in options]


def test_both_orders_averages_a_letter_bias():
    first_letter_bias = Scripted({"where": lambda options: [0.6, 0.2, 0.2]})
    ask = Asker(first_letter_bias, image=None, both_orders=True)
    probs = ask("where", "Where?", ["Left side", "Center", "Right side"])
    assert probs == pytest.approx([0.4, 0.2, 0.4])  # the bias cancels out
    assert first_letter_bias.calls[1][1] == ["Right side", "Center", "Left side"]
    assert ask.records[0]["answer"] in ("Left side", "Right side")


def test_guided_policy_rules():
    def decide(visible, where=None, distance=None):
        answers = {"visible": by_text({"Yes": visible, "No": 1 - visible})}
        if where:
            answers["where"] = by_text(where)
        if distance:
            answers["distance"] = by_text(distance)
        return GuidedPolicy("the red ball").decide(Asker(Scripted(answers), None, False))

    assert decide(0.1).action == "left"  # not visible: search
    left = {"Left side": 0.8, "Center": 0.1, "Right side": 0.1}
    assert decide(0.9, left).action == "left"
    centre = {"Left side": 0.1, "Center": 0.8, "Right side": 0.1}
    near = {"Less than half a meter": 0.7, "About one meter": 0.2, "Two meters or more": 0.1}
    far = {"Less than half a meter": 0.1, "About one meter": 0.2, "Two meters or more": 0.7}
    assert decide(0.9, centre, near).action == "done"
    far_move = decide(0.9, centre, far)
    assert (far_move.action, far_move.amount) == ("forward", 0.7)


def test_direct_policy_takes_the_models_action():
    policy = DirectPolicy("the red ball")
    options = [o.format(t="the red ball") for o in policy.options]
    right = next(o for o in options if "right" in o.lower())
    scorer = Scripted({"action": lambda o: [0.9 if x == right else 0.025 for x in o]})
    decision = policy.decide(Asker(scorer, None, False))
    assert decision.action == "right"
    assert scorer.calls[0][1] == options


@pytest.mark.parametrize("prompt", list(DIRECT_PROMPTS))
def test_oracle_understands_every_direct_prompt(prompt):
    question, options = DIRECT_PROMPTS[prompt]
    texts = [o.format(t="the red ball") for o in options]
    for action in ("search", "left", "right", "done", "forward"):
        chosen = list(options.values())[pick_action(action, texts)][0]
        assert chosen == action or (action == "search" and chosen == "left"), (prompt, action)


def test_oracle_drives_to_the_red_ball(runner, tmp_path):
    agent = Agent(
        runner,
        OracleScorer(runner, "red_ball"),
        "red_ball",
        both_orders=False,
        max_steps=30,
        log_dir=tmp_path,
    )
    summary = agent.run()
    assert summary["status"] == "success", summary
    assert summary["bumps"] == 0
    assert summary["distance_m"] < 0.5
    lines = (tmp_path / "log.jsonl").read_text().splitlines()
    assert len(lines) == summary["steps"]
    assert json.loads(lines[-1])["action"] == "done"
    assert (tmp_path / "map.png").is_file() and (tmp_path / "step_000.jpg").is_file()


def test_agent_reports_scorer_errors(runner):
    class Broken:
        def score(self, *args):
            raise RuntimeError("API down")

    summary = Agent(runner, Broken(), "red_ball", max_steps=3).run()
    assert summary["status"] == "error"
    assert "API down" in summary["error"]

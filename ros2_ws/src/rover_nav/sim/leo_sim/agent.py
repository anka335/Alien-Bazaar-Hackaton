"""jevomir drives the simulated rover: look with the camera, ask closed-choice questions, move.

The model never sees the map or the ground truth, only the rover's camera, like on the real
rover. Two policies:

- `guided` (default): the model answers perception questions about the target (visible?
  where? how far?) and fixed rules turn the answers into moves. Each answer is one API call.
- `direct`: the model picks the action itself ("Drive forward", "Turn left", ...).

Options are shown to the model as letters, and it prefers some letters regardless of content
(jevomir/API.md), so with `both_orders` every question is also asked with the options
reversed and the two probabilities per option are averaged.

A move that ends `stalled` (wheels blocked, as leo-rover-mcp reports it) is followed by a
short back-up and a turn. Bumps are measured by the sim for the score only; the policy
doesn't get them, since the real rover has no bump sensor.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .jevomir import Scorer, target_view
from .memory import Tracker
from .runner import SimRunner, jpeg

IMAGE_SIZE = (448, 336)  # the API letterboxes to 448x448
SUCCESS_M = 0.5  # front of the rover within this of the target's surface
SEARCH_TURN = math.radians(40)
AIM_TURN = math.radians(15)


@dataclass
class Decision:
    action: str  # forward | back | left | right | done
    amount: float = 0.0  # metres or radians
    reason: str = ""
    questions: list[dict] = field(default_factory=list)
    # the model's answer about the target in this photo, for the memory: "left", "right",
    # "center", "center close", "none" (not in view), or None (not asked)
    seen: str | None = None


class Asker:
    """Asks one question (both orders if set) and records it for the log."""

    def __init__(self, scorer: Scorer, image: np.ndarray, both_orders: bool, context: str = ""):
        self.scorer, self.image, self.both_orders = scorer, image, both_orders
        self.context = context  # the rover's memory, put before every question
        self.records: list[dict] = []

    def __call__(self, kind: str, question: str, options: list[str]) -> list[float]:
        asked = f"{self.context}\n{question}" if self.context else question  # logged apart
        probs = self.scorer.score(asked, options, [self.image], kind)
        if self.both_orders:
            reverse = self.scorer.score(asked, options[::-1], [self.image], kind)[::-1]
            probs = [(a + b) / 2 for a, b in zip(probs, reverse, strict=True)]
        best = int(np.argmax(probs))
        self.records.append(
            {
                "kind": kind,
                "question": question,
                "options": options,
                "probabilities": [round(p, 4) for p in probs],
                "answer": options[best],
            }
        )
        return probs


class GuidedPolicy:
    name = "guided"

    def __init__(self, target_label: str):
        self.target = target_label
        self.search_turns = 0

    def decide(self, ask: Asker) -> Decision:
        t = self.target
        yes, _ = ask("visible", f"Is {t} visible in this photo?", ["Yes", "No"])
        if yes < 0.5:
            self.search_turns += 1
            if self.search_turns % 9 == 0:  # a full circle without it: go somewhere else
                clear, _ = ask(
                    "clear",
                    "Is the floor straight ahead free of obstacles for at least one meter?",
                    ["Yes", "No"],
                )
                if clear >= 0.5:
                    return Decision(
                        "forward", 0.6, "not found after a full circle: explore", seen="none"
                    )
            return Decision("left", SEARCH_TURN, f"{t} not visible: search", seen="none")
        self.search_turns = 0
        left, centre, right = ask(
            "where", f"Where is {t} in this photo?", ["Left side", "Center", "Right side"]
        )
        if left > max(centre, right):
            return Decision("left", AIM_TURN, f"{t} on the left", seen="left")
        if right > max(centre, left):
            return Decision("right", AIM_TURN, f"{t} on the right", seen="right")
        near, mid, far = ask(
            "distance",
            f"How far is {t} from the camera?",
            ["Less than half a meter", "About one meter", "Two meters or more"],
        )
        if near > max(mid, far):
            return Decision("done", 0.0, f"{t} ahead and close", seen="center close")
        return Decision("forward", 0.35 if mid > far else 0.7, f"{t} ahead", seen="center")


SEARCH = "search"  # a direct option meaning "not in view": turn left like the guided search

# Direct prompts: question, and option -> (action, amount). {t} is the target's label.
# tools/eval_direct.py scores them on labelled camera frames; DIRECT_PROMPT is the best one.
DIRECT_PROMPTS = {
    "plain": (
        "This photo is from the front camera of a small wheeled robot. The robot must drive "
        "up to {t}. What should it do next?",
        {
            "Drive forward": ("forward", 0.4),
            "Turn left": ("left", math.radians(30)),
            "Turn right": ("right", math.radians(30)),
            "Back up": ("back", 0.25),
            "Stop, it has arrived": ("done", 0.0),
        },
    ),
    # each move says what it means in the picture, so the model matches what it sees
    "reasons": (
        "This photo is from the front camera of a small wheeled robot that must drive up to "
        "{t}. What should the robot do next?",
        {
            "Drive forward: {t} is straight ahead": ("forward", 0.4),
            "Turn left: {t} is on the left side of the photo": ("left", math.radians(20)),
            "Turn right: {t} is on the right side of the photo": ("right", math.radians(20)),
            "Stop: {t} is right in front of the robot, very close": ("done", 0.0),
            "Turn to search: {t} is not in the photo": (SEARCH, 0.0),
        },
    ),
    # only where the target is; the options map to moves
    "describe": (
        "This photo is from the front camera of a small wheeled robot. Where is {t}?",
        {
            "In the middle of the photo, still far away": ("forward", 0.4),
            "On the left side of the photo": ("left", math.radians(20)),
            "On the right side of the photo": ("right", math.radians(20)),
            "In the middle of the photo and very close, large and low in the picture": (
                "done",
                0.0,
            ),
            "Not in the photo": (SEARCH, 0.0),
        },
    ),
    # "describe" with halves instead of sides: targets 20-45 deg off read as "middle"
    "describe_half": (
        "This photo is from the front camera of a small wheeled robot. Where is {t}?",
        {
            "In the center of the photo, still far away": ("forward", 0.4),
            "In the left half of the photo": ("left", math.radians(20)),
            "In the right half of the photo": ("right", math.radians(20)),
            "In the center of the photo and very close, large and low in the picture": (
                "done",
                0.0,
            ),
            "Not in the photo": (SEARCH, 0.0),
        },
    ),
    # "describe" without the robot framing
    "describe_short": (
        "Where is {t} in this photo?",
        {
            "In the middle of the photo, still far away": ("forward", 0.4),
            "On the left side of the photo": ("left", math.radians(20)),
            "On the right side of the photo": ("right", math.radians(20)),
            "In the middle of the photo and very close, large and low in the picture": (
                "done",
                0.0,
            ),
            "Not in the photo": (SEARCH, 0.0),
        },
    ),
    # second person, "reasons" options
    "you": (
        "You are driving a small wheeled robot and this is its front camera. You must reach "
        "{t}. Which move should you make now?",
        {
            "Drive forward: {t} is straight ahead": ("forward", 0.4),
            "Turn left: {t} is on the left side of the photo": ("left", math.radians(20)),
            "Turn right: {t} is on the right side of the photo": ("right", math.radians(20)),
            "Stop: {t} is right in front of the robot, very close": ("done", 0.0),
            "Turn to search: {t} is not in the photo": (SEARCH, 0.0),
        },
    ),
}
DIRECT_PROMPT = "describe_half"


class DirectPolicy:
    name = "direct"

    def __init__(self, target_label: str, prompt: str = ""):
        self.target = target_label
        self.question, self.options = DIRECT_PROMPTS[prompt or DIRECT_PROMPT]

    def decide(self, ask: Asker) -> Decision:
        options = [o.format(t=self.target) for o in self.options]
        probs = ask("action", self.question.format(t=self.target), options)
        best = int(np.argmax(probs))
        action, amount = list(self.options.values())[best]
        if action == SEARCH:
            return Decision("left", SEARCH_TURN, "not in view: search", seen="none")
        seen = {"left": "left", "right": "right", "forward": "center", "done": "center close"}
        return Decision(action, amount, "chosen by the model", seen=seen.get(action))


POLICIES = {"guided": GuidedPolicy, "direct": DirectPolicy}


class Agent:
    def __init__(
        self,
        runner: SimRunner,
        scorer: Scorer,
        target: str,
        policy: str = "guided",
        camera: str = "leo",
        max_steps: int = 60,
        both_orders: bool = True,
        log_dir: Path | None = None,
        on_step: Callable[[dict], None] | None = None,
        memory: str = "off",  # off | control | prompt (leo_sim/memory.py)
    ):
        self.runner, self.scorer, self.target, self.camera = runner, scorer, target, camera
        label = runner.call(lambda s: s.world.obj(target).label) or target.replace("_", " ")
        self.label = label
        self.policy = POLICIES[policy](label)
        if memory not in ("off", "control", "prompt"):
            raise ValueError(f"memory must be off, control or prompt, not {memory!r}")
        self.memory_mode = memory
        self.memory = Tracker(label) if memory != "off" else None
        self.max_steps, self.both_orders = max_steps, both_orders
        self.log_dir, self.on_step = log_dir, on_step
        self.steps: list[dict] = []
        self.images: list[bytes] = []  # JPEG the model saw, per step
        self.status = "idle"
        self.summary: dict[str, Any] = {}

    def run(self, cancel: threading.Event | None = None) -> dict[str, Any]:
        cancel = cancel or threading.Event()
        self.status = "running"
        self.runner.goal = self.runner.call(lambda s: s.object_position(self.target))
        if self.log_dir:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        wall0, sim0 = time.monotonic(), self.runner.call(lambda s: s.time)
        said_done, error = False, None
        try:
            for index in range(self.max_steps):
                if cancel.is_set() or self._tipped():
                    break
                said_done = self._step(index, cancel)
                if said_done:
                    break
        except Exception as exc:  # noqa: BLE001 - reported in the summary and the UI
            error = f"{type(exc).__name__}: {exc}"
        view = self.runner.call(lambda s: target_view(s, self.target, self.camera))
        success = said_done and view["surface_m"] < SUCCESS_M
        self.status = (
            "error"
            if error
            else "stopped"
            if cancel.is_set()
            else "tipped"
            if self._tipped()
            else "success"
            if success
            else "failed"
        )
        self.summary = {
            "status": self.status,
            "target": self.target,
            "policy": self.policy.name,
            "memory": self.memory_mode,
            "steps": len(self.steps),
            "said_done": said_done,
            "error": error,
            "distance_m": round(view["surface_m"], 3),
            "questions": sum(len(s["questions"]) for s in self.steps)
            * (2 if self.both_orders else 1),
            "bumps": sum(bool(s["result"]["bumped"]) for s in self.steps if s.get("result")),
            "sim_s": round(self.runner.call(lambda s: s.time) - sim0, 1),
            "wall_s": round(time.monotonic() - wall0, 1),
        }
        if self.log_dir:
            (self.log_dir / "summary.json").write_text(json.dumps(self.summary, indent=1) + "\n")
            Image.fromarray(
                self.runner.call(lambda s: s.render_map(640, goal=self.runner.goal))
            ).save(self.log_dir / "map.png")
        return self.summary

    def _step(self, index: int, cancel: threading.Event) -> bool:
        image = self.runner.render(self.camera, *IMAGE_SIZE)
        before = self.runner.call(lambda s: s.odom())
        context = self.memory.context() if self.memory_mode == "prompt" else ""
        ask = Asker(self.scorer, image, self.both_orders, context)
        decision = self.policy.decide(ask)
        if self.memory_mode == "control":
            decision = self.memory.adjust(decision, explore=self.policy.name == "direct")
        record: dict[str, Any] = {
            "step": index,
            "action": decision.action,
            "amount": round(decision.amount, 3),
            "reason": decision.reason,
            "memory": context,
            "questions": ask.records,
            "state": self.runner.state(),
        }
        if decision.action != "done":
            record["result"] = self._execute(decision, cancel)
        if self.memory:
            after = self.runner.call(lambda s: s.odom())
            outcome = (record.get("result") or {}).get("outcome", "done")
            self.memory.update(
                index, decision.seen, decision.action, decision.amount, outcome, before, after
            )
        self.images.append(jpeg(image))
        self.steps.append(record)
        if self.log_dir:
            (self.log_dir / f"step_{index:03d}.jpg").write_bytes(self.images[-1])
            with (self.log_dir / "log.jsonl").open("a") as log:
                log.write(json.dumps(record) + "\n")
        if self.on_step:
            self.on_step(record)
        return decision.action == "done"

    def _tipped(self) -> bool:
        return bool(self.steps) and (self.steps[-1].get("result") or {}).get("outcome") == "tilted"

    def _execute(self, d: Decision, cancel: threading.Event) -> dict:
        kind, amount = {
            "forward": ("move", d.amount),
            "back": ("move", -d.amount),
            "left": ("turn", d.amount),
            "right": ("turn", -d.amount),
        }[d.action]
        result = self.runner.motion(kind, amount, cancel=cancel)
        if result["outcome"] == "stalled" and not cancel.is_set():
            back = self.runner.motion("move", -0.2, cancel=cancel)
            self.runner.motion("turn", math.radians(45), cancel=cancel)
            result["recovery"] = back["outcome"]
            result["bumped"] = sorted(set(result["bumped"]) | set(back["bumped"]))
        return result

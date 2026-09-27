"""The rover's memory of its own path, from its wheel odometry and the model's own answers.

jevomir answers about one photo at a time. The tracker keeps what the real rover would know
too (never the sim's ground truth): the moves and their outcomes, the odometry pose, and
where the target was last seen, carried through the turns since (when the model says "left
half", the target is ~30 deg left of that photo's heading; after the rover turns, that bearing
is re-expressed from the new heading: dead reckoning on the odometry).

Two ways to use it (Agent `memory=`):
- "control": the agent, not the model, acts on it (`adjust`): a lost target is searched
  towards where it was last seen instead of always to the left; turns that go back and forth
  are halved; a full circle without the target is followed by a short drive elsewhere.
- "prompt": `context()` sentences go before every question. Measured: it makes jevomir worse
  (82% -> 40-64% right moves on the labelled frames, 7/12 -> 2/12 rooms), even when the
  sentences are true; the model copies "not seen yet" and stops looking. Kept to compare.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# Bearing of the target in the photo for each answer (the rover camera's HFOV is 109 deg,
# so the middle of a half is ~27 deg off the centre). Positive = left.
SIDE_BEARING = {"left": math.radians(30), "right": math.radians(-30), "center": 0.0}
SIDE_TEXT = {
    "left": "in the left half of the photo",
    "right": "in the right half of the photo",
    "center": "in the center of the photo",
}
MOVE_TEXT = {
    "forward": "drove {m:.2f} m forward",
    "back": "backed up {m:.2f} m",
    "left": "turned {d:.0f}° left",
    "right": "turned {d:.0f}° right",
}


EXPLORE_M = 0.6  # drive after a full circle without the target
MIN_TURN = math.radians(7)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Tracker:
    target: str  # the target's label, e.g. "the red ball"
    moves: list[dict] = field(default_factory=list)
    seen_heading: float | None = None  # odometry heading towards the target when last seen
    seen_step: int | None = None
    seen_side: str = ""
    seen_close: bool = False
    turned_unseen: float = 0.0  # radians turned since the target was last seen
    sweep: str = ""  # search direction after one turn back to where it was last seen
    driven: float = 0.0
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)  # odometry x, y, heading

    def update(
        self,
        step: int,
        seen: str | None,
        action: str,
        amount: float,
        outcome: str,
        before: tuple[float, float, float],
        after: tuple[float, float, float],
    ) -> None:
        """After one step: `seen` is the model's answer about the target in that step's photo
        ("left", "right", "center", "center close", "none", or None if not asked)."""
        if seen and seen != "none":
            side = seen.split()[0]
            self.seen_heading = before[2] + SIDE_BEARING[side]
            self.seen_step, self.seen_side = step, side
            self.seen_close = seen.endswith("close")
            self.turned_unseen = 0.0
            self.sweep = ""
        elif seen == "none":
            self.turned_unseen += abs(_wrap(after[2] - before[2]))
        self.driven += math.dist(before[:2], after[:2])
        self.pose = after
        self.moves.append({"step": step, "action": action, "amount": amount, "outcome": outcome})

    def context(self) -> str:
        """A few sentences for the start of the question, or "" before the first move."""
        if not self.moves:
            return ""
        t = self.target
        step = self.moves[-1]["step"] + 1
        lines = [f"Robot memory from its wheel odometry (step {step + 1}):"]
        recent = [self._move_text(m) for m in self.moves[-3:]]
        lines.append("Last moves, oldest first: " + "; ".join(recent) + ".")
        if self.moves[-1]["outcome"] == "stalled":
            lines.append(
                "The last move was blocked by an obstacle, so the robot backed up and "
                "turned 45° left."
            )
        turns = [m["action"] for m in self.moves[-4:] if m["action"] in ("left", "right")]
        if len(turns) == 4 and all(a != b for a, b in zip(turns, turns[1:], strict=False)):
            lines.append(
                f"It has been turning back and forth, so {t} is probably almost straight ahead."
            )
        if self.seen_heading is None:
            lines.append(f"It has not seen {t} yet.")
        else:
            ago = step - 1 - self.seen_step
            when = "in the previous photo" if ago == 0 else f"{ago + 1} photos ago"
            where = SIDE_TEXT[self.seen_side] + (" and very close" if self.seen_close else "")
            lines.append(f"It last saw {t} {when}, {where}; that is {self._bearing_text()}.")
        if self.turned_unseen >= 2 * math.pi - 0.1:
            lines.append(f"Since then it has turned a full circle without seeing {t}.")
        x, y, h = self.pose
        lines.append(
            f"Position: x {x:.2f} m, y {y:.2f} m, heading {math.degrees(h):.0f}° from "
            f"the start; {self.driven:.1f} m driven in total."
        )
        return " ".join(lines)

    def last_seen_bearing(self) -> float | None:
        """Where the target was last seen, relative to the current heading (rad, + = left)."""
        if self.seen_heading is None:
            return None
        return _wrap(self.seen_heading - self.pose[2])

    def adjust(self, decision, explore: bool = True):
        """Change a policy's Decision with what the path tells (memory="control")."""
        d = decision
        if d.seen == "none":
            if explore and self.turned_unseen >= 2 * math.pi - 0.1:
                self.turned_unseen = 0.0
                d.action, d.amount = "forward", EXPLORE_M
                d.reason += "; a full circle without it: explore"
                return d
            if d.action not in ("left", "right"):
                return d
            rel = self.last_seen_bearing()
            if not self.sweep and rel is not None and abs(rel) > math.radians(12):
                # once: turn back to where it was last seen, then keep sweeping that way (a
                # second look there, and back again, would swing between two empty views)
                d.action = self.sweep = "left" if rel > 0 else "right"
                d.amount = min(d.amount, abs(rel) + math.radians(10))
                d.reason += f"; last seen {math.degrees(abs(rel)):.0f}° {d.action}: search there"
            elif self.sweep and d.action != self.sweep:
                d.action = self.sweep
                d.reason += f"; keep searching {self.sweep}"
            elif not self.sweep:
                self.sweep = d.action
            return d
        turns = [m["action"] for m in self.moves[-2:] if m["action"] in ("left", "right")]
        if d.action in ("left", "right") and turns and turns[-1] != d.action:
            d.amount = max(d.amount / 2, MIN_TURN)
            d.reason += "; back and forth: half turn"
        return d

    def _bearing_text(self) -> str:
        rel = math.degrees(_wrap(self.seen_heading - self.pose[2]))
        if abs(rel) < 12:
            return "roughly straight ahead of where the robot now faces"
        if abs(rel) > 150:
            return "behind the robot now"
        side = "left" if rel > 0 else "right"
        return f"about {abs(rel):.0f}° to the {side} of where the robot now faces"

    @staticmethod
    def _move_text(m: dict) -> str:
        text = MOVE_TEXT[m["action"]].format(m=m["amount"], d=math.degrees(m["amount"]))
        return text + (" (blocked)" if m["outcome"] == "stalled" else "")

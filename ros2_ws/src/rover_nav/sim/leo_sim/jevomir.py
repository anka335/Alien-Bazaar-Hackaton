"""Scorers: who answers the agent's closed-choice questions.

- `JevomirClient`: the jevomir scoring API (Qwen3.5-4B, one forward pass per question, see
  jevomir/API.md). The key is read from JEVOMIR_API_KEY or a key file and never logged.
- `OracleScorer`: answers from the sim's ground truth, for tests and for trying the UI
  without a GPU. It knows only the question kinds the policies in agent.py ask.

Both return one probability per option, in the order given.
"""

from __future__ import annotations

import base64
import http.client
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Protocol

import mujoco
import numpy as np

from .model import CAMERAS
from .runner import SimRunner, jpeg

DEFAULT_URL = "http://127.0.0.1:8100"  # jevomir's api_server.py on this machine
KEY_FILE = Path(__file__).resolve().parents[1] / ".api-key"  # git-ignored


class Scorer(Protocol):
    def score(
        self, question: str, options: list[str], images: list[np.ndarray], kind: str
    ) -> list[float]: ...


class ScorerError(RuntimeError):
    pass


class RateLimiter:
    """Blocks until a call fits in `per_minute` calls per sliding minute (free ngrok: ~100)."""

    def __init__(self, per_minute: int):
        self.per_minute, self.calls, self.lock = per_minute, deque(), threading.Lock()

    def wait(self) -> None:
        if self.per_minute <= 0:
            return
        with self.lock:
            while True:
                now = time.monotonic()
                while self.calls and now - self.calls[0] >= 60:
                    self.calls.popleft()
                if len(self.calls) < self.per_minute:
                    self.calls.append(now)
                    return
                time.sleep(60 - (now - self.calls[0]))


def load_key(key_file: Path | None = None) -> str:
    key = os.environ.get("JEVOMIR_API_KEY", "").strip()
    path = key_file or KEY_FILE
    if not key and path.is_file():
        key = path.read_text().strip()
    return key


class JevomirClient:
    def __init__(
        self,
        url: str = "",
        key_file: Path | None = None,
        timeout: float = 60.0,
        max_per_minute: int = 90,
    ):
        self.url = (url or os.environ.get("JEVOMIR_API_URL") or DEFAULT_URL).rstrip("/")
        self.key = load_key(key_file)
        if not self.key:
            where = key_file or KEY_FILE
            raise ScorerError(f"no API key: set JEVOMIR_API_KEY or write it to {where}")
        self.timeout = timeout
        self.limiter = RateLimiter(max_per_minute)
        self.last_timing: dict | None = None

    def _request(self, method: str, path: str, body: dict | None = None) -> dict:
        self.limiter.wait()
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(
            self.url + path,
            data=data,
            method=method,
            headers={
                "X-API-Key": self.key,
                "Content-Type": "application/json",
                "ngrok-skip-browser-warning": "1",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read())
        except urllib.error.HTTPError as error:
            detail = error.read().decode(errors="replace")[:300]
            raise ScorerError(f"jevomir API {error.code}: {detail}") from None
        except (OSError, http.client.HTTPException) as error:
            raise ScorerError(
                f"jevomir API unreachable at {self.url}: {getattr(error, 'reason', error)}"
            ) from None

    def info(self) -> dict:
        return self._request("GET", "/v1/info")

    def score(
        self, question: str, options: list[str], images: list[np.ndarray], kind: str = ""
    ) -> list[float]:
        encoded = [base64.b64encode(jpeg(image, 90)).decode() for image in images]
        result = self._request(
            "POST",
            "/v1/score",
            {"question": question, "options": options, "images": encoded, "id": kind or None},
        )
        self.last_timing = result.get("timing")
        return [float(o["probability"]) for o in result["options"]]


class OracleScorer:
    """Ground-truth answers for the question kinds of agent.py (visible, where, distance,
    clear, action). `confidence` goes to the true option, the rest is spread evenly."""

    def __init__(
        self, runner: SimRunner, target: str, camera: str = "leo", confidence: float = 0.9
    ):
        self.runner, self.target, self.camera, self.confidence = runner, target, camera, confidence

    def truth(self) -> dict:
        return self.runner.call(lambda s: target_view(s, self.target, self.camera))

    def score(
        self, question: str, options: list[str], images: list[np.ndarray], kind: str
    ) -> list[float]:
        t = self.truth()
        index = self._answer(kind, t, options)
        rest = (1 - self.confidence) / (len(options) - 1)
        return [self.confidence if i == index else rest for i in range(len(options))]

    @staticmethod
    def _answer(kind: str, t: dict, options: list[str]) -> int:
        def pick(word: str) -> int:
            return next(i for i, o in enumerate(options) if word.lower() in o.lower())

        if kind == "visible":
            return pick("yes" if t["visible"] else "no")
        if kind == "where":
            side = (
                "left" if t["bearing_deg"] > 18 else "right" if t["bearing_deg"] < -18 else "center"
            )
            return pick(side)
        if kind == "distance":
            d = t["surface_m"]
            return pick("less than" if d < 0.5 else "about one" if d < 1.5 else "two meters")
        if kind == "clear":
            return pick("yes" if t["clear_m"] > 1.0 else "no")
        if kind == "action":
            return pick_action(oracle_action(t), options)
        raise ValueError(f"oracle cannot answer question kind {kind!r}")


def target_view(sim, target: str, camera: str = "leo") -> dict:
    """Where `target` is for the rover's camera: bearing, distances, line of sight."""
    m, d = sim.model, sim.data
    cam_id = m.camera(camera).id
    cam = d.cam_xpos[cam_id].copy()
    body = m.body(target).id
    obj = sim.world.obj(target)
    goal = d.xpos[body].copy()
    goal[2] = max(goal[2], 0.05)
    x, y, yaw = sim.pose()
    bearing = math.degrees(math.atan2(goal[1] - y, goal[0] - x) - yaw)
    bearing = (bearing + 180) % 360 - 180
    groups = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)  # world geoms only, not the rover
    geom = np.array([-1], dtype=np.int32)
    ray = goal - cam
    hit = mujoco.mj_ray(m, d, cam, ray / np.linalg.norm(ray), groups, 1, -1, geom)
    line_of_sight = (
        hit < 0
        or geom[0] < 0
        or m.geom_bodyid[geom[0]] == body
        or hit >= np.linalg.norm(ray) - obj.radius
    )
    half_fov = math.degrees(CAMERAS[camera]["hfov"]) / 2 if camera in CAMERAS else 30
    ahead = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    start = np.array([x, y, 0.08]) + ahead * 0.3
    clear = mujoco.mj_ray(m, d, start, ahead, groups, 1, -1, geom)
    centre = math.hypot(goal[0] - x, goal[1] - y)
    return {
        "bearing_deg": bearing,
        "visible": bool(abs(bearing) < half_fov - 5 and line_of_sight),
        "centre_m": centre,
        "surface_m": max(0.0, centre - obj.radius - 0.25),  # front of the rover to the object
        "clear_m": clear if clear >= 0 else math.inf,
    }


def oracle_action(view: dict) -> str:
    """The right move for a target_view(): search | left | right | done | forward."""
    if not view["visible"]:
        return "search"
    if abs(view["bearing_deg"]) > 15:
        return "left" if view["bearing_deg"] > 0 else "right"
    return "done" if view["surface_m"] < 0.5 else "forward"


ACTION_WORDS = {  # how the direct prompts word each move
    "search": ("not in", "not visible", "turn left"),
    "left": ("left",),
    "right": ("right",),
    "done": ("stop", "very close"),
    "forward": ("forward", "ahead", "far away"),
}


def pick_action(action: str, options: list[str]) -> int:
    for word in ACTION_WORDS[action]:
        for i, option in enumerate(options):
            text = option.lower()
            if word in text and (action == "search" or "not " not in text):
                return i
    raise ValueError(f"no option for {action!r} in {options}")

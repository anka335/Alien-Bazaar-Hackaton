"""Web UI: watch the simulated rover, drive it by hand, or let jevomir drive it.

The page (web/index.html) polls camera frames and state; the API key stays in this process.
"""

from __future__ import annotations

import json
import math
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .agent import POLICIES, Agent
from .jevomir import JevomirClient, OracleScorer, ScorerError
from .runner import SimRunner

WEB = Path(__file__).resolve().parent / "web"
MANUAL = {
    "forward": ("move", 0.25),
    "back": ("move", -0.25),
    "left": ("turn", math.radians(20)),
    "right": ("turn", math.radians(-20)),
}
MAX_BODY = 64 * 1024


class App:
    def __init__(self, runner: SimRunner, client: JevomirClient | None, client_error: str = ""):
        self.runner, self.client, self.client_error = runner, client, client_error
        self.agent: Agent | None = None
        self.cancel = threading.Event()
        self.thread: threading.Thread | None = None
        self.manual: threading.Thread | None = None
        self.lock = threading.Lock()

    def busy(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def world(self) -> list[dict]:
        return self.runner.call(
            lambda s: [{"name": o.name, "label": o.label} for o in s.world.objects]
        )

    def state(self) -> dict[str, Any]:
        agent = self.agent
        return {
            "sim": self.runner.state(),
            "seed": self.runner.seed,
            "jevomir": {
                "url": self.client.url if self.client else None,
                "error": self.client_error,
            },
            "agent": None
            if agent is None
            else {
                "status": agent.status,
                "target": agent.target,
                "label": agent.label,
                "policy": agent.policy.name,
                "summary": agent.summary,
                "steps": [{k: v for k, v in s.items() if k != "state"} for s in agent.steps],
            },
        }

    def start_agent(self, body: dict) -> dict:
        with self.lock:
            if self.busy():
                raise ValueError("jevomir is already driving")
            target = str(body.get("target", ""))
            names = {o["name"] for o in self.world()}
            if target not in names:
                raise ValueError(f"unknown target {target!r}")
            policy = str(body.get("policy", "guided"))
            if policy not in POLICIES:
                raise ValueError(f"unknown policy {policy!r}")
            if body.get("scorer") == "oracle":
                scorer = OracleScorer(self.runner, target)
            elif self.client is None:
                raise ValueError(f"jevomir API not configured: {self.client_error}")
            else:
                scorer = self.client
            steps = max(1, min(int(body.get("max_steps", 40)), 200))
            self.cancel = threading.Event()
            self.agent = Agent(
                self.runner,
                scorer,
                target,
                policy=policy,
                max_steps=steps,
                both_orders=bool(body.get("both_orders", True)),
                memory=str(body.get("memory", "off")),
            )
            self.thread = threading.Thread(target=self.agent.run, args=(self.cancel,), daemon=True)
            self.thread.start()
            return {"ok": True}

    def stop(self) -> dict:
        self.cancel.set()
        self.runner.call(lambda s: s.stop())
        return {"ok": True}

    def drive(self, action: str) -> dict:
        if action == "stop":
            return self.stop()
        if action not in MANUAL:
            raise ValueError(f"unknown action {action!r}")
        if self.busy():
            raise ValueError("jevomir is driving: stop it first")
        if self.manual is not None and self.manual.is_alive():
            self.runner.call(lambda s: s.stop())
            self.manual.join(2)
        kind, amount = MANUAL[action]
        self.manual = threading.Thread(target=self.runner.motion, args=(kind, amount), daemon=True)
        self.manual.start()
        return {"ok": True}

    def reset(self, seed: Any) -> dict:
        self.stop()
        if self.thread:
            self.thread.join(5)
        self.agent = None
        self.runner.reset(None if seed in (None, "", "fixed") else int(seed))
        return {"ok": True}


def make_handler(app: App):
    class Handler(BaseHTTPRequestHandler):
        def _send(
            self, status: int, data: bytes, content_type: str = "application/json; charset=utf-8"
        ):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _json(self, status: int, payload: Any):
            self._send(status, json.dumps(payload).encode())

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(
                    200, (WEB / "index.html").read_bytes(), "text/html; charset=utf-8"
                )
            if path == "/api/state":
                return self._json(200, app.state())
            if path == "/api/world":
                return self._json(200, {"objects": app.world(), "policies": list(POLICIES)})
            if match := re.fullmatch(r"/api/frame/(leo|oak|chase|map)\.jpg", path):
                data = app.runner.frame(match[1])
                return (
                    self._send(200, data, "image/jpeg")
                    if data
                    else self._json(503, {"detail": "no frame yet"})
                )
            if match := re.fullmatch(r"/api/step/(\d+)\.jpg", path):
                agent, index = app.agent, int(match[1])
                if agent and index < len(agent.images):
                    return self._send(200, agent.images[index], "image/jpeg")
            self._json(404, {"detail": "not found"})

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 <= length <= MAX_BODY:
                return self._json(413, {"detail": "body too large"})
            try:
                body = json.loads(self.rfile.read(length) or b"{}")
                if self.path == "/api/drive":
                    return self._json(200, app.drive(str(body.get("action", ""))))
                if self.path == "/api/agent/start":
                    return self._json(200, app.start_agent(body))
                if self.path == "/api/agent/stop":
                    return self._json(200, app.stop())
                if self.path == "/api/reset":
                    return self._json(200, app.reset(body.get("seed")))
            except (ValueError, TypeError, json.JSONDecodeError) as error:
                return self._json(400, {"detail": str(error)})
            self._json(404, {"detail": "not found"})

        def log_message(self, fmt, *args):
            pass

    return Handler


def serve(
    host: str, port: int, seed: int | None, api_url: str, key_file: Path | None, max_per_minute: int
) -> None:
    client, error = None, ""
    try:
        client = JevomirClient(api_url, key_file, max_per_minute=max_per_minute)
    except ScorerError as exc:
        error = str(exc)
        print(f"jevomir API off ({error}); manual driving and the oracle still work", flush=True)
    runner = SimRunner(seed=seed)
    app = App(runner, client, error)
    server = ThreadingHTTPServer((host, port), make_handler(app))
    target = client.url if client else "no API"
    print(f"Leo Rover sim on http://{host}:{port} (jevomir: {target})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()
        runner.close()

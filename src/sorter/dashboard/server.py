"""Placeholder dashboard from block 0: status JSON, commands, a bare page. Block 7 replaces it.

The HTTP API is in docs/architecture.md → Dashboard HTTP API.
"""

from __future__ import annotations

import dataclasses

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from sorter.core.hub import Hub
from sorter.core.types import Command
from sorter.dashboard.config import DashboardConfig

_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Sorter</title>
<style>body{font:16px system-ui;margin:2em}button{font-size:1.1em;margin:.2em}
#hold{background:#c00;color:#fff;font-weight:bold}pre{background:#eee;padding:1em}</style>
</head><body>
<h1>Sorter <small>(placeholder dashboard, block 7 replaces it)</small></h1>
<div id="buttons"></div><pre id="status">…</pre>
<script>
const cmds = ["start","pause","resume","step","stop","reset","hold"];
for (const c of cmds) {
  const b = document.createElement("button"); b.textContent = c.toUpperCase(); b.id = c;
  b.onclick = () => fetch("/api/command", {method: "POST",
    headers: {"Content-Type": "application/json"}, body: JSON.stringify({cmd: c})});
  document.getElementById("buttons").append(b);
}
setInterval(async () => {
  const s = await (await fetch("/api/status")).json();
  document.getElementById("status").textContent = JSON.stringify(s, null, 2);
}, 300);
</script></body></html>"""


class CommandRequest(BaseModel):
    cmd: str


def create_app(hub: Hub, cfg: DashboardConfig) -> FastAPI:
    app = FastAPI(title="Sorter")

    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        return _PAGE

    @app.get("/api/status")
    def status() -> dict:
        return dataclasses.asdict(hub.status())

    @app.post("/api/command")
    def command(req: CommandRequest) -> dict:
        try:
            cmd = Command(req.cmd)
        except ValueError:
            raise HTTPException(400, f"unknown command {req.cmd!r}") from None
        hub.send(cmd)
        return {"ok": True}

    return app

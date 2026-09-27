import json
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest
from leo_sim.runner import SimRunner
from leo_sim.server import App, make_handler


@pytest.fixture
def base():
    runner = SimRunner(seed=None, realtime=0)
    app = App(runner, client=None, client_error="no API key")
    server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(app))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{server.server_port}"
    server.shutdown()
    app.stop()
    runner.close()


def get(url):
    with urllib.request.urlopen(url, timeout=10) as r:
        return r.status, r.headers["Content-Type"], r.read()


def post(url, body):
    request = urllib.request.Request(
        url, json.dumps(body).encode(), {"Content-Type": "application/json"}
    )
    try:
        with urllib.request.urlopen(request, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_page_state_and_frames(base):
    assert get(base + "/")[0] == 200
    status, _, body = get(base + "/api/state")
    state = json.loads(body)
    assert state["jevomir"]["error"] == "no API key" and state["agent"] is None
    names = [o["name"] for o in json.loads(get(base + "/api/world")[2])["objects"]]
    assert "red_ball" in names
    for _ in range(50):  # the first request starts rendering that camera
        try:
            status, kind, data = get(base + "/api/frame/leo.jpg")
            break
        except urllib.error.HTTPError:
            time.sleep(0.1)
    assert kind == "image/jpeg" and data[:2] == b"\xff\xd8"


def test_jevomir_without_key_is_refused_but_oracle_runs(base):
    status, body = post(base + "/api/agent/start", {"target": "red_ball"})
    assert status == 400 and "not configured" in body["detail"]
    assert post(base + "/api/agent/start", {"target": "nope", "scorer": "oracle"})[0] == 400
    body = {"target": "red_ball", "scorer": "oracle", "both_orders": False}
    assert post(base + "/api/agent/start", body)[0] == 200
    for _ in range(300):
        agent = json.loads(get(base + "/api/state")[2])["agent"]
        if agent["status"] != "running":
            break
        time.sleep(0.1)
    assert agent["status"] == "success"
    assert get(base + "/api/step/0.jpg")[1] == "image/jpeg"


def test_manual_drive(base):
    assert post(base + "/api/drive", {"action": "forward"})[0] == 200
    assert post(base + "/api/drive", {"action": "sideways"})[0] == 400
    assert post(base + "/api/reset", {"seed": 3})[0] == 200
    assert json.loads(get(base + "/api/state")[2])["seed"] == 3

import base64
import io
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pytest
from leo_sim.jevomir import JevomirClient, ScorerError
from PIL import Image


class FakeApi:
    """Answers /v1/score like jevomir's api_server.py: the first option wins."""

    def __init__(self, status=200):
        self.requests = []
        api = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                api.requests.append(
                    {"key": self.headers.get("X-API-Key"), "body": body, "path": self.path}
                )
                if status != 200:
                    data = b'{"detail": "invalid API key"}'
                else:
                    n = len(body["options"])
                    probs = [0.7] + [0.3 / (n - 1)] * (n - 1)
                    data = json.dumps(
                        {
                            "options": [
                                {"letter": chr(65 + i), "text": o, "probability": p}
                                for i, (o, p) in enumerate(zip(body["options"], probs, strict=True))
                            ],
                            "timing": {"total_s": 0.07},
                        }
                    ).encode()
                self.send_response(status)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def close(self):
        self.server.shutdown()


@pytest.fixture
def key_file(tmp_path, monkeypatch):
    monkeypatch.delenv("JEVOMIR_API_KEY", raising=False)
    path = tmp_path / ".api-key"
    path.write_text("jev_test\n")
    return path


def test_score_sends_the_api_request(key_file):
    api = FakeApi()
    try:
        client = JevomirClient(api.url, key_file, max_per_minute=0)
        image = np.zeros((30, 40, 3), np.uint8)
        image[:, :20] = (255, 0, 0)
        probs = client.score("Where?", ["Left side", "Center"], [image], "where")
    finally:
        api.close()
    assert probs == [0.7, pytest.approx(0.3)]
    request = api.requests[0]
    assert request["path"] == "/v1/score"
    assert request["key"] == "jev_test"
    assert request["body"]["question"] == "Where?"
    assert request["body"]["options"] == ["Left side", "Center"]
    decoded = Image.open(io.BytesIO(base64.b64decode(request["body"]["images"][0])))
    assert decoded.size == (40, 30)


def test_http_errors_become_scorer_errors(key_file):
    api = FakeApi(status=401)
    try:
        client = JevomirClient(api.url, key_file, max_per_minute=0)
        with pytest.raises(ScorerError, match="401"):
            client.score("q", ["a", "b"], [np.zeros((8, 8, 3), np.uint8)])
    finally:
        api.close()


def test_unreachable_api(key_file):
    client = JevomirClient("http://127.0.0.1:9", key_file, timeout=2, max_per_minute=0)
    with pytest.raises(ScorerError, match="unreachable"):
        client.score("q", ["a", "b"], [np.zeros((8, 8, 3), np.uint8)])


def test_missing_key(tmp_path, monkeypatch):
    monkeypatch.delenv("JEVOMIR_API_KEY", raising=False)
    with pytest.raises(ScorerError, match="no API key"):
        JevomirClient("http://127.0.0.1:9", tmp_path / "nothing")


def test_env_key_wins(key_file, monkeypatch):
    monkeypatch.setenv("JEVOMIR_API_KEY", "jev_env")
    assert JevomirClient("http://x", key_file).key == "jev_env"

import json
import urllib.request

import numpy as np
import pytest

from rebot_b601.arm import Arm
from rebot_b601.viewer import start_viewer


def get(url, path):
    with urllib.request.urlopen(url + path, timeout=3) as r:
        return r.status, r.read()


@pytest.fixture
def twin(tmp_path):
    arm = Arm(max_speed_scale=1.0)
    server, url = start_viewer(arm, 0, assets_dir=tmp_path)     # port 0 = any free port; empty assets = skeleton page
    yield arm, url
    server.shutdown()
    server.server_close()
    if arm.connected:
        arm._stop_thread.set()
        arm.backend.close()
        arm.backend = None


def test_page_and_state_when_disconnected(twin):
    arm, url = twin
    status, body = get(url, "")
    assert status == 200 and b"<canvas" in body and b"/state" in body
    st = json.loads(get(url, "state")[1])
    assert st["connected"] is False
    assert st["tcp"] == pytest.approx([0.3017, 0.0, 0.2177], abs=1e-3)      # shows the home pose
    assert len(st["points"]) == 8                                            # base, 6 joints, TCP


def test_state_follows_the_arm(twin):
    arm, url = twin
    arm.connect(simulate=True)
    arm.move_to_xyz(0.30, 0.05, 0.08, approach="down", speed_scale=1.0)
    st = json.loads(get(url, "state")[1])
    assert st["connected"] and st["simulated"] and not st["moving"]
    assert st["tcp"] == pytest.approx([0.30, 0.05, 0.08], abs=3e-3)
    assert st["target"] == pytest.approx([0.30, 0.05, 0.08], abs=1e-6)
    assert st["tcp_axes"][0][2] < -0.99                                      # approach axis points down
    assert np.allclose(st["points"][-1], st["tcp"], atol=1e-3)


def test_unknown_path_is_404(twin):
    _, url = twin
    with pytest.raises(urllib.error.HTTPError) as e:
        get(url, "nope")
    assert e.value.code == 404


def test_state_has_link_frames(twin):
    arm, url = twin
    st = json.loads(get(url, "state")[1])
    assert set(st["links"]) == {"base_link", "link1", "link2", "link3", "link4", "link5", "link6",
                                "gripper_end", "gripper_left", "gripper_right"}
    assert all(len(m) == 16 for m in st["links"].values())
    assert st["links"]["gripper_end"][3::4][:3] == pytest.approx(st["tcp"], abs=1e-3)   # translation column


def test_mesh_page_and_asset_serving(tmp_path):
    (tmp_path / "three").mkdir()
    (tmp_path / "three" / "three.module.min.js").write_text("export {}")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "part.STL").write_bytes(b"\0" * 84)
    (tmp_path / "manifest.json").write_text(json.dumps({"links": {}}))
    (tmp_path.parent / "secret.js").write_text("nope")
    arm = Arm()
    server, url = start_viewer(arm, 0, assets_dir=tmp_path)
    try:
        page = get(url, "")[1]
        assert b"importmap" in page and b"STLLoader" in page              # CAD page, not the skeleton
        assert get(url, "assets/three/three.module.min.js")[0] == 200
        assert get(url, "assets/meshes/part.STL")[1] == b"\0" * 84
        for bad in ("assets/../secret.js", "assets/%2e%2e/secret.js", "assets/manifest.txt", "assets/meshes"):
            with pytest.raises(urllib.error.HTTPError) as e:
                get(url, bad)
            assert e.value.code == 404
    finally:
        server.shutdown()
        server.server_close()

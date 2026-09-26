"""Manual control (setup mode) on the kinematic sim: poses, tour, jog, hold, saving a pose."""

import shutil

import numpy as np
import pytest
import yaml
from fastapi.testclient import TestClient

from sorter.app import build_system
from sorter.core.config import DEFAULT_CONFIG_DIR
from sorter.dashboard.manual import TOUR, ManualControl, write_pose
from sorter.dashboard.server import create_app


@pytest.fixture
def rig(tmp_path):
    path = tmp_path / "rig.yaml"
    shutil.copy(DEFAULT_CONFIG_DIR / "rig.yaml", path)
    return path


@pytest.fixture
def system(sim_config):
    s = build_system(sim_config, sim=True)
    s.arm.start()
    return s


@pytest.fixture
def manual(system, rig):
    return ManualControl(system.arm, rig, system.cfg.arm.gripper.open)


def test_write_pose_keeps_the_rest(rig):
    before = yaml.safe_load(rig.read_text())
    write_pose(rig, "look_box", [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    after = yaml.safe_load(rig.read_text())
    assert after["poses"]["look_box"] == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    after["poses"]["look_box"] = before["poses"]["look_box"]
    assert after == before
    assert rig.read_text().startswith("# Fixed rig")


def test_go_and_bins_via_home(manual, system):
    manual.go("look_box")
    manual.wait()
    assert system.arm.at == "look_box" and manual.state()["error"] is None

    visited = []
    go_to = system.arm.go_to
    system.arm.go_to = lambda name: (visited.append(name), go_to(name))
    manual.go("bin_dark")
    manual.wait()
    manual.go("look_bg")
    manual.wait()
    assert visited == ["home", "bin_dark", "home", "look_bg"]


def test_tour_starts_at_the_box_and_wraps(manual, system):
    assert manual.state()["tour_next"] == "look_box"
    for name in TOUR:
        manual.tour_next()
        manual.wait()
        assert system.arm.at == name
    assert manual.state()["tour_next"] == TOUR[0]


def test_jog_and_gripper(manual, system):
    manual.go("home")
    manual.wait()
    q0 = np.array(system.arm.joints())
    manual.jog(0, np.radians(5))
    manual.wait()
    assert np.array(system.arm.joints()) - q0 == pytest.approx([np.radians(5), 0, 0, 0, 0, 0])
    assert system.arm.at is None
    manual.gripper(True)
    manual.wait()
    assert manual.state()["gripper"] == pytest.approx(system.cfg.arm.gripper.open, abs=0.05)


def test_hold_then_release(manual, system):
    system.arm.hold()
    manual.go("home")
    manual.wait()
    s = manual.state()
    assert s["held"] and "held" in s["error"]
    manual.release()
    manual.wait()
    manual.go("home")
    manual.wait()
    assert system.arm.at == "home" and not manual.state()["held"]


def test_save_pose(manual, system, rig):
    manual.go("home")
    manual.wait()
    manual.jog(0, np.radians(10))
    manual.wait()
    q = system.arm.joints()
    manual.save_pose("look_bg")
    assert yaml.safe_load(rig.read_text())["poses"]["look_bg"] == pytest.approx(q, abs=1e-4)
    manual.go("home")
    manual.wait()
    manual.go("look_bg")
    manual.wait()
    assert system.arm.joints() == pytest.approx(q, abs=1e-4)


def test_api(system, manual):
    with TestClient(create_app(system.hub, system.cfg.dashboard, manual=manual)) as c:
        assert c.get("/", follow_redirects=False).headers["location"] == "/manual"
        assert c.get("/manual").status_code == 200
        assert c.get("/static/manual.js").status_code == 200
        assert c.post("/api/manual", json={"action": "go", "pose": "home"}).status_code == 200
        manual.wait()
        s = c.get("/api/manual").json()
        assert s["at"] == "home" and len(s["joints"]) == 6 and len(s["tcp_mm"]) == 3
        assert c.post("/api/manual", json={"action": "go", "pose": "moon"}).status_code == 400
        assert c.post("/api/manual", json={"action": "fly"}).status_code == 400


def test_api_off_without_manual(system):
    with TestClient(create_app(system.hub, system.cfg.dashboard)) as c:
        assert c.get("/api/manual").status_code == 404
        assert "__PHASE_LABELS__" not in c.get("/").text

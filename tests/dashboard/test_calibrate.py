"""The camera calibration page on the physics sim: tape marks drawn on the mat, clicks, the
fitted mount against the true one, look poses, and the files it writes."""

import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
from fastapi.testclient import TestClient

from sorter.app import build_system
from sorter.arm import kinematics as kin
from sorter.calibration.marks import deproject, project
from sorter.core.config import DEFAULT_CONFIG_DIR, load_config
from sorter.dashboard.calibrate import CalibrateControl, coverage, write_views
from sorter.dashboard.manual import ManualControl
from sorter.dashboard.server import create_app

pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def system():
    cfg = load_config(
        overrides={
            "sim": {"engine": "physics", "realtime": 0, "items": [], "marks": True},
            "state_machine": {"save_runs": False},
        }
    )
    system = build_system(cfg, sim=True)
    system.camera.start()
    system.arm.start()
    yield system
    system.camera.close()
    system.world.stop()


@pytest.fixture
def rig(tmp_path):
    path = tmp_path / "rig.yaml"
    shutil.copy(DEFAULT_CONFIG_DIR / "rig.yaml", path)
    return path


@pytest.fixture
def cal(system, rig, tmp_path):
    from sorter.sim.physics.backend import hand_eye

    manual = ManualControl(system.arm, rig, system.cfg.arm.gripper.open)
    return CalibrateControl(
        manual,
        system.camera,
        system.calibration,
        system.cfg,
        tmp_path / "hand_eye.yaml",
        hand_eye(system.cfg),
        fixed_marks=True,
    )


def _click_all(cal, system):
    """Click every mark in view where the true mount puts it (a careful user)."""
    frame = system.camera.fresh()
    k = frame.intrinsics
    T_cam = system.arm.ee_pose() @ cal.true_mount
    n = 0
    for name in cal.marks:
        (p,) = project(T_cam, k, [cal.marks[name].xyz])
        if p is None or not (20 <= p[0] < k.width - 20 and 20 <= p[1] < k.height - 20):
            continue
        if deproject(frame, *p) is None:  # under the gripper: too close for depth
            continue
        assert frame.color[int(p[1]), int(p[0])].mean() < 90  # the tape is drawn there
        cal.click(name, p[0] + 7, p[1] - 6)  # a bit off: it snaps to the square
        n += 1
    return n


def test_the_tip_goes_over_a_mark(cal, system):
    cal.goto_mark("M1")
    cal.manual.wait()
    assert cal.manual.state()["error"] is None
    tip = kin.fk_tcp(system.arm.joints())[:3, 3]
    x, y, z = cal.marks["M1"].xyz
    assert np.hypot(tip[0] - x, tip[1] - y) < 3.0
    assert 0 < tip[2] - z < 20


def test_clicks_from_views_find_the_true_mount(cal, system):
    for i in range(3):
        cal.goto_view(i)
        cal.manual.wait()
        assert cal.manual.state()["error"] is None
        assert _click_all(cal, system) >= 3
    fit = cal.state()["fit"]
    assert fit["marks"] == 6
    assert fit["true_error_mm"] < 3.0 and fit["true_error_deg"] < 1.0
    assert fit["rmse_mm"] < 3.0

    path = cal.save_mount()
    he = yaml.safe_load(Path(path).read_text())["calibration"]["hand_eye"]
    assert np.allclose(he["T_link5_cam"], cal.mount, atol=1e-5)
    assert he["method"].startswith("marks (6 marks")


def test_a_second_click_on_a_mark_replaces_the_first(cal, system):
    cal.goto_view(0)
    cal.manual.wait()
    cal.clear_clicks()  # the marks it found by itself
    cal.click("M1", 320, 240)
    cal.click("M1", 330, 250)
    clicks = cal.state()["clicks"]
    assert len(clicks) == 1
    assert np.hypot(clicks[0]["px"][0] - 330, clicks[0]["px"][1] - 250) < 15  # snapped to M1
    cal.delete_click(0)
    assert cal.state()["clicks"] == []


def test_look_poses_see_the_whole_zone(cal, system, rig):
    look = cal.compute_look()
    assert look["look_box"]["fits"], look["look_box"]
    # joint 6 doesn't turn the camera (D-022): over the mat, off to the side, the image is
    # turned against it and the arm can't lift the camera high enough to see all of it
    assert look["look_bg"]["coverage"] >= 0.9, look["look_bg"]
    for name in ("look_box", "look_bg"):
        assert look[name]["camera_mm"] >= 200
    cal.goto_look("look_box")
    cal.manual.wait()
    assert cal.manual.state()["error"] is None

    before = yaml.safe_load(rig.read_text())
    cal.save_look()
    after = yaml.safe_load(rig.read_text())
    assert after["poses"]["look_box"] == look["look_box"]["q"]
    assert after["zones"] == before["zones"]
    assert after["poses"]["home"] == before["poses"]["home"]
    for zone in ("box", "background"):
        roi = after["views"][zone]["roi"]
        assert len(roi) == 4 and all(0 <= u < 640 and 0 <= v < 480 for u, v in roi)
    assert np.allclose(system.arm.poses["look_box"], look["look_box"]["q"])
    assert rig.read_text().startswith("# Fixed rig")


def test_write_views_keeps_the_rest(rig):
    before = yaml.safe_load(rig.read_text())
    write_views(rig, {"box": [[1, 2], [3, 4], [5, 6], [7, 8]]})
    after = yaml.safe_load(rig.read_text())
    assert after["views"]["box"]["roi"] == [[1, 2], [3, 4], [5, 6], [7, 8]]
    after["views"]["box"] = before["views"]["box"]
    assert after == before


def test_coverage():
    assert coverage([(10, 10), (100, 10), (100, 100), (10, 100)], 640, 480) == 1.0
    half = coverage([(-100, 10), (100, 10), (100, 100), (-100, 100)], 640, 480)
    assert half == pytest.approx(0.5, abs=0.01)
    assert coverage([(10, 10), None, (100, 100), (10, 100)], 640, 480) == 0.0


def test_http_api(cal, system):
    app = create_app(system.hub, system.cfg.dashboard, manual=cal.manual, calibrate=cal)
    client = TestClient(app)
    post = lambda body: client.post("/api/calibrate", json=body).status_code  # noqa: E731
    s = client.get("/api/calibrate").json()
    assert {m["name"] for m in s["marks"]} == set(cal.marks)
    assert s["image"] == {"width": 640, "height": 480}
    assert post({"action": "goto_mark", "mark": "M9"}) == 400
    assert post({"action": "save_mount"}) == 400
    assert post({"action": "nope"}) == 400
    assert client.get("/calibrate").status_code == 200


def test_off_outside_the_setup_mode(system):
    client = TestClient(create_app(system.hub, system.cfg.dashboard))
    assert client.get("/api/calibrate").status_code == 404


def test_calibrate_saves_the_mount_and_the_look_poses(cal, system, rig):
    for i in range(2):
        cal.goto_view(i)
        cal.manual.wait()
        _click_all(cal, system)
    assert {c["pose"] for c in cal.state()["clicks"]} == {0, 1}
    out = cal.calibrate()
    assert yaml.safe_load(Path(out["file"]).read_text())["calibration"]["hand_eye"]
    poses = yaml.safe_load(rig.read_text())["poses"]
    assert np.allclose(system.arm.poses["look_box"], poses["look_box"])
    s = cal.state()
    assert s["mount"]["saved"] and s["look_saved"] == str(rig)


def test_marks_where_the_tip_stopped_survive_a_restart(system, rig, tmp_path):
    manual = ManualControl(system.arm, rig, system.cfg.arm.gripper.open)
    file = tmp_path / "marks.yaml"
    args = (manual, system.camera, system.calibration, system.cfg, tmp_path / "he.yaml")
    cal = CalibrateControl(*args, marks_file=file)
    cal.goto_mark("M2")
    manual.wait()
    tip = kin.fk_tcp(system.arm.joints())[:3, 3]
    assert np.allclose(cal._mark("M2")[:2], tip[:2], atol=2.0)  # where the tip stopped
    again = CalibrateControl(*args, marks_file=file)
    assert again._mark("M2") == cal._mark("M2")
    assert again._mark("M1") == again.marks["M1"].xyz


def test_a_view_finds_and_names_the_marks_by_itself(cal, system):
    cal.clear_clicks()
    for i in range(2):
        cal.goto_view(i)
        cal.manual.wait()
        assert cal.manual.state()["error"] is None
        assert len(cal.state()["detected"]["marks"]) >= 5
    s = cal.state()
    assert {c["pose"] for c in s["clicks"]} == {0, 1}
    assert s["fit"]["true_error_mm"] < 3.0 and s["fit"]["true_error_deg"] < 1.0

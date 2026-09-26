import pytest

from cloth_task.leader import JOINTS, public_angle, to_follower, unwrap

ZERO = dict.fromkeys(JOINTS, 0.0)


def test_zero_pose_maps_to_home_with_gripper_closed():
    q, opening = to_follower(ZERO, 240.0)
    assert q == pytest.approx([0.0] * 6)
    assert opening == 0.0


def test_shoulder_lift_up_is_positive_on_our_arm():
    # leader servo +90 → LeRobot public -90 (direction -1) → RS motor +90 = our joint2 +90
    q, _ = to_follower({**ZERO, "shoulder_lift": 90.0}, 240.0)
    assert q[1] == pytest.approx(90.0)


def test_elbow_and_wrist_directions():
    q, _ = to_follower({**ZERO, "elbow_flex": -60.0, "wrist_flex": 30.0, "wrist_roll": 20.0}, 240)
    assert q[2] == pytest.approx(60.0)  # direction +1, then RS -1
    assert q[3] == pytest.approx(-30.0)
    assert q[5] == pytest.approx(20.0)  # direction -1, then RS -1


def test_gripper_scale_and_opening():
    # leader +20 → public -120 (×-6) → motor +120 → opening 120/240
    _, opening = to_follower({**ZERO, "gripper": 20.0}, 240.0)
    assert opening == pytest.approx(0.5)
    _, opening = to_follower({**ZERO, "gripper": 80.0}, 240.0)  # clamped at the range
    assert opening == 1.0


def test_multi_turn_is_unwrapped_and_clamped():
    assert unwrap(370.0, -180.0, 180.0) == pytest.approx(10.0)
    assert public_angle("shoulder_pan", 360.0 + 10.0) == pytest.approx(-10.0)
    assert public_angle("wrist_flex", 120.0) == 90.0  # clamped to LeRobot's range


def test_flip_reverses_one_joint_only():
    raw = {**ZERO, "shoulder_lift": 30.0, "elbow_flex": -40.0}
    q, _ = to_follower(raw, 240.0)
    qf, _ = to_follower(raw, 240.0, frozenset({"elbow_flex"}))
    assert qf[1] == pytest.approx(q[1])
    assert qf[2] == pytest.approx(-q[2])


def test_roi_save_keeps_comments_and_other_keys(tmp_path):
    import yaml

    from cloth_task.roi_tool import save_roi

    path = tmp_path / "detector.yaml"
    path.write_text("# box region\nroi: []\nother: 3\n")
    save_roi(str(path), [(10, 20), (300, 20), (300, 200)])
    text = path.read_text()
    assert text.startswith("# box region")
    data = yaml.safe_load(text)
    assert data["roi"] == [[10, 20], [300, 20], [300, 200]] and data["other"] == 3

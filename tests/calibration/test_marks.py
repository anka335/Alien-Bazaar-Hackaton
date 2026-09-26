"""Hand-eye calibration from tape marks: the fit, depth at a click, projection."""

import dataclasses

import cv2
import numpy as np

from sorter.calibration.marks import (
    Click,
    deproject,
    fit_mount,
    marks,
    mount_change,
    project,
    view_rmse,
)
from sorter.core.types import Frame, Intrinsics

K = Intrinsics(615.0, 615.0, 320.0, 240.0, 640, 480)


def _pose(rvec, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, dtype=float))[0]
    T[:3, 3] = t
    return T


# a camera ~140 mm behind the flange, looking along the flange's x (like the rig's mount)
X_TRUE = _pose([0.05, 1.5, 0.03], [-60.0, 5.0, 50.0])


def _flange_looking_at(target, height, turn):
    """A flange pose whose camera (X_TRUE) looks straight down at `target` from `height`."""
    T_cam = np.eye(4)
    T_cam[:3, :3] = (
        cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0] @ cv2.Rodrigues(np.array([0.0, 0.0, turn]))[0]
    )
    T_cam[:3, 3] = [target[0], target[1], height]
    return T_cam @ np.linalg.inv(X_TRUE)


def _clicks(noise_mm=0.0, seed=0):
    rng = np.random.default_rng(seed)
    ms = marks((180.0, 210.0), 1.0)
    out = []
    for dx, height, turn in ((0, 260, 0.0), (30, 300, 0.4), (-30, 230, -0.4)):
        F = _flange_looking_at((180 + dx, 210), height, turn)
        for m in ms:
            p = np.linalg.inv(F @ X_TRUE) @ np.array([*m.xyz, 1.0])
            p = p[:3] + rng.normal(0, noise_mm, 3)
            out.append(Click(m.name, m.xyz, F, tuple(p), (0.0, 0.0)))
    return out


def test_fit_finds_the_mount():
    fit, _ = fit_mount(_clicks())
    assert fit is not None
    dist, angle = mount_change(X_TRUE, fit.T_flange_cam)
    assert dist < 1e-6 and angle < 1e-3
    assert fit.rmse_mm < 1e-6


def test_fit_with_noisy_depth_stays_close():
    fit, _ = fit_mount(_clicks(noise_mm=1.0))
    dist, angle = mount_change(X_TRUE, fit.T_flange_cam)
    assert dist < 3.0 and angle < 1.0
    assert 0.3 < fit.rmse_mm < 3.0


def test_view_rmse_tells_an_fk_error_from_a_bad_click():
    clicks = _clicks()
    # the arm's pose off by 15 mm in view 2 (FK): the mount fit is poor, every view alone agrees
    shift = np.eye(4)
    shift[:3, 3] = [15.0, 0.0, 0.0]
    views = [clicks[i * 6 : (i + 1) * 6] for i in range(3)]
    views[1] = [dataclasses.replace(c, T_base_flange=shift @ c.T_base_flange) for c in views[1]]
    fit, _ = fit_mount([c for v in views for c in v])
    assert fit.rmse_mm > 5.0
    assert all(view_rmse(v) < 1e-6 for v in views)
    # a click on the wrong tape: its view disagrees by itself
    bad = [dataclasses.replace(views[0][1], mark="M3", p_base=views[0][2].p_base), *views[0][2:]]
    assert view_rmse(bad) > 5.0
    assert view_rmse(views[0][:2]) is None


def test_fit_needs_three_marks_not_in_a_line():
    clicks = _clicks()
    two = [c for c in clicks if c.mark in ("M1", "M2")]
    assert fit_mount(two) == (None, [])
    line = [c for c in clicks if c.mark in ("M1", "M6")]  # M1 and M6 are on one x line
    line += [c for c in clicks if c.mark == "M1"]
    assert fit_mount(line) == (None, [])


def test_deproject_inverts_project():
    T_cam = _pose([np.pi, 0.0, 0.0], [200.0, 180.0, 260.0])  # looking straight down
    p = (215.0, 170.0, 1.0)
    ((u, v),) = project(T_cam, K, [p])
    depth = np.zeros((480, 640), np.uint16)
    depth[:] = round(260 - 1)
    frame = Frame(np.zeros((480, 640, 3), np.uint8), depth, K, 0.0, 0)
    c = np.array(deproject(frame, u, v))
    back = (T_cam @ np.array([*c, 1.0]))[:3]
    assert np.linalg.norm(back - p) < 1.0


def test_deproject_without_depth():
    frame = Frame(np.zeros((480, 640, 3), np.uint8), np.zeros((480, 640), np.uint16), K, 0.0, 0)
    assert deproject(frame, 320, 240) is None


def test_project_behind_the_camera():
    T_cam = _pose([np.pi, 0.0, 0.0], [0.0, 0.0, 100.0])
    assert project(T_cam, K, [(0.0, 0.0, 200.0)]) == [None]


def test_a_click_snaps_to_the_tape_square():
    from sorter.calibration.marks import snap

    img = np.full((480, 640, 3), 120, np.uint8)
    img[200:224, 300:324] = 25  # a tape square centered at (311.5, 211.5)
    u, v = snap(img, 318, 205)
    assert abs(u - 311.5) < 0.6 and abs(v - 211.5) < 0.6
    assert snap(img, 500, 400) is None  # nothing dark near
    assert snap(np.full((480, 640, 3), 120, np.uint8), 300, 200) is None


def test_a_view_clicked_mirrored_is_fixed():
    """M2↔M3 and M4↔M5 swapped in one view fit as well, with the camera flipped under the
    table: the fit with the camera above it is the right one, no prior needed."""
    from sorter.calibration.marks import mirrored

    ms = marks((180.0, 210.0), 1.0)
    positions = {m.name: m.xyz for m in ms}
    clicks = _clicks()
    views = [i // len(ms) for i in range(len(clicks))]
    swapped = [mirrored(c, positions) if v == 0 else c for c, v in zip(clicks, views, strict=True)]
    for n in (len(ms), len(clicks)):  # one view, all views
        fit, fixed = fit_mount(swapped[:n], views[:n], positions)
        dist, angle = mount_change(X_TRUE, fit.T_flange_cam)
        assert fixed == [0] and dist < 1e-3 and angle < 1e-3


def test_identify_names_the_marks_from_their_distances_alone():
    from sorter.calibration.marks import identify

    ms = marks((180.0, 210.0), 1.0)
    positions = {m.name: m.xyz for m in ms}
    T_cam = _pose([np.pi, 0.0, 0.4], [170.0, 200.0, 260.0])  # over the mat, turned
    order = ["M4", "M1", "M6", "M2", "M5"]  # M3 out of view, and in any order
    cam = [(np.linalg.inv(T_cam) @ np.array([*positions[n], 1.0]))[:3] for n in order]
    cam.append(np.array([40.0, -30.0, 255.0]))  # a shadow
    names, T = identify(np.array(cam), positions)
    assert names == dict(enumerate(order))
    assert np.allclose(T, T_cam, atol=1e-6)


def test_plausible_ignores_the_turn_about_the_optical_axis():
    from sorter.calibration.marks import plausible

    turned = X_TRUE @ _pose([0.0, 0.0, np.radians(135)], [0.0, 0.0, 0.0])  # about its axis
    assert plausible(X_TRUE, turned)
    tilted = X_TRUE @ _pose([np.radians(70), 0.0, 0.0], [0.0, 0.0, 0.0])  # looking elsewhere
    assert not plausible(X_TRUE, tilted)
    moved = X_TRUE @ _pose([0.0, 0.0, 0.0], [300.0, 0.0, 0.0])
    assert not plausible(X_TRUE, moved)

import math

import cv2
import numpy as np
import pytest

from rover_nav.boxes import (
    BoxConfig,
    ahead_and_left,
    bearing_deg,
    detect_markers,
    from_xyz_rpy,
    label_boxes,
    refine_with_depth,
)

W, H = 640, 360
K = np.array([[466.0, 0, W / 2], [0, 466.0, H / 2], [0, 0, 1]])  # OAK-D color, ~69° wide
SIZE = 0.04  # the boxes' markers: black square 4 cm


def facing(x, distance, yaw_deg=0.0):
    """A marker `distance` ahead and `x` to the right, facing the camera."""
    T = from_xyz_rpy((x, 0.0, distance), (math.pi, 0.0, 0.0))
    return T @ from_xyz_rpy((0, 0, 0), (0.0, math.radians(yaw_deg), 0.0))


def render(markers, dictionary="DICT_4X4_50"):
    """Grey background with the given (id, T_cam_marker) markers pasted in perspective."""
    out = np.full((H, W), 110, np.uint8)
    for marker_id, T in markers:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        inner = cv2.aruco.drawMarker(d, marker_id, 600)
        img = cv2.copyMakeBorder(inner, 150, 150, 150, 150, cv2.BORDER_CONSTANT, value=255)
        n = img.shape[0]
        s = SIZE * n / 600 / 2
        corners = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
        px, _ = cv2.projectPoints(corners, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, None)
        e = -0.5
        src = np.float32([[e, e], [n + e, e], [n + e, n + e], [e, n + e]])
        Hm = cv2.getPerspectiveTransform(src, px.reshape(4, 2).astype(np.float32))
        warped = cv2.warpPerspective(img, Hm, (W, H), borderValue=0)
        mask = cv2.warpPerspective(np.full_like(img, 255), Hm, (W, H), borderValue=0) > 0
        out[mask] = warped[mask]
    return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)


THREE = [(3, facing(-0.25, 0.7)), (8, facing(0.0, 0.7)), (5, facing(0.25, 0.7))]


def test_three_boxes_by_order_left_to_right():
    markers = detect_markers(render(THREE), K, None, SIZE)
    assert sorted(m.marker_id for m in markers) == [3, 5, 8]
    boxes = label_boxes(markers, BoxConfig())
    assert [(b.label, b.marker.marker_id, b.by_id) for b in boxes] == [
        ("dark", 3, False),
        ("colored", 8, False),
        ("light", 5, False),
    ]
    for b, (_, T) in zip(boxes, THREE, strict=True):
        assert np.linalg.norm(b.marker.T_cam_marker[:3, 3] - T[:3, 3]) < 0.02


def test_known_ids_work_with_one_box_in_view():
    cfg = BoxConfig(ids={"dark": [3], "colored": [8], "light": [5]})
    boxes = label_boxes(detect_markers(render([THREE[2]]), K, None, SIZE), cfg)
    assert [(b.label, b.by_id) for b in boxes] == [("light", True)]


def test_order_fallback_needs_all_three():
    boxes = label_boxes(detect_markers(render(THREE[:2]), K, None, SIZE), BoxConfig())
    assert boxes == []  # two unknown markers: can't tell which two boxes they are


def test_mixed_known_and_unknown():
    cfg = BoxConfig(ids={"colored": [8]})
    boxes = label_boxes(detect_markers(render(THREE), K, None, SIZE), cfg)
    assert [(b.label, b.marker.marker_id, b.by_id) for b in boxes] == [
        ("dark", 3, False),
        ("colored", 8, True),
        ("light", 5, False),
    ]


def test_other_dictionaries_are_found():
    markers = detect_markers(render([(17, facing(0.0, 0.6))], "DICT_5X5_100"), K, None, SIZE)
    assert [(m.dictionary, m.marker_id) for m in markers] == [("DICT_5X5_100", 17)]


def depth_of(T, noise=0.0, seed=0):
    """Depth image (m) of the marker's plane, like the OAK-D's aligned depth; 0 elsewhere."""
    v, u = np.mgrid[0:H, 0:W]
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    n, p0 = T[:3, 2], T[:3, 3]  # plane normal and a point on it
    z = (n @ p0) / (rays @ n)  # depth along the optical axis
    rng = np.random.default_rng(seed)
    return z * (1 + noise * rng.standard_normal(z.shape))


@pytest.mark.parametrize("distance", [0.4, 0.6, 0.8, 1.0])
def test_4cm_marker_distance_from_depth(distance):
    T = facing(0.0, distance, yaw_deg=20)
    (m,) = detect_markers(render([(3, T)]), K, None, SIZE)
    size_based_error = abs(m.distance - distance) / distance
    refined = refine_with_depth(m, depth_of(T, noise=0.01))
    assert refined.from_depth
    assert abs(refined.distance - distance) / distance < 0.015  # within 1.5 % with 1 % noise
    assert abs(refined.distance - distance) < abs(m.distance - distance) or size_based_error < 0.015


def test_no_depth_keeps_the_marker():
    T = facing(0.0, 0.6)
    (m,) = detect_markers(render([(3, T)]), K, None, SIZE)
    assert refine_with_depth(m, np.zeros((H, W))) is m


def test_size_based_distance_is_too_long_far_away():
    # why depth is used: without it the distance is ~10-16 % too long beyond ~1 m
    (m,) = detect_markers(render([(3, facing(0.0, 1.6))]), K, None, SIZE)
    assert m.distance > 1.6 * 1.08


def test_rover_frame_helpers():
    T = from_xyz_rpy((1.27, 0.3, 0.1), (0, 0, 0))
    ahead, left = ahead_and_left(T, front_m=0.27)
    assert ahead == pytest.approx(1.0) and left == pytest.approx(0.3)
    assert bearing_deg(T) == pytest.approx(math.degrees(math.atan2(0.3, 1.27)))

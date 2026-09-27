"""The floor detector on synthetic frames: size in mm from depth, the grasp yaw, cut-off masks."""

import math

import cv2
import numpy as np

from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import ColorClass, Frame, Intrinsics
from sorter.floor_detector.config import FloorDetectorConfig
from sorter.floor_detector.detector import SockDetector, mask_axes

W, H, F = 640, 480, 615.0
DEPTH = 350  # mm: the camera over the floor in a scan pose


def _frame(rects: list[tuple[tuple[float, float], tuple[float, float], float]], bgr=(40, 40, 200)):
    """A floor frame with rotated rectangles ((cx, cy), (w, h) px, angle°) of cloth on it."""
    color = np.full((H, W, 3), (60, 110, 170), np.uint8)  # the wooden floor
    depth = np.full((H, W), DEPTH, np.uint16)
    masks = []
    for r in rects:
        m = np.zeros((H, W), np.uint8)
        cv2.fillPoly(m, [cv2.boxPoints(r).astype(np.int32)], 1)
        m = m.astype(bool)
        color[m] = bgr
        depth[m] = DEPTH - 8  # the cloth's thickness
        masks.append(m)
    k = Intrinsics(F, F, W / 2, H / 2, W, H)
    frame = Frame(color=color, depth_mm=depth, intrinsics=k, timestamp=0.0, seq=0)
    return frame, masks


def _detector(masks, **cfg):
    seg = lambda _bgr: [Instance(m, 0.9) for m in masks]  # noqa: E731
    classifier = Sam3ColorClassifier(ColorClassifierConfig(), seg)
    return SockDetector(FloorDetectorConfig(**cfg), classifier)


def _px(mm: float) -> float:
    return mm * F / DEPTH


def test_mask_axes_finds_the_long_side():
    _, (m,) = _frame([((320, 240), (_px(200), _px(90)), 30.0)])
    angle, long, short = mask_axes(m)
    assert abs((math.degrees(angle) - 30 + 90) % 180 - 90) < 2
    assert long > 2 * short


def test_a_sock_lying_flat_is_found_with_its_size_and_yaw():
    frame, masks = _frame([((300, 250), (_px(200), _px(90)), 30.0)])
    result = _detector(masks).detect(frame)
    assert len(result.socks) == 1
    s = result.socks[0]
    assert s.color is ColorClass.COLORED
    assert 200 * 90 * 0.85 < s.stats["area_mm2"] < 200 * 90 * 1.15
    assert 170 < s.stats["length_mm"] < 230
    # the fingers close across the sock: the yaw is square to its long side (mod π)
    assert abs((math.degrees(s.grasp_angle_rad) - 120 + 90) % 180 - 90) < 3
    assert not s.touches_roi_edge
    assert s.mask is not None and s.mask.sum() == s.area_px


def test_not_socks_are_rejected():
    scrap = ((150, 150), (_px(30), _px(30)), 0.0)  # 9 cm²: too small
    towel = ((400, 300), (_px(300), _px(200)), 0.0)  # 600 cm²: too big
    frame, masks = _frame([scrap, towel])
    result = _detector(masks).detect(frame)
    assert result.socks == []


def test_a_sock_cut_off_by_the_frame_edge_says_so():
    frame, masks = _frame([((20, 240), (_px(200), _px(90)), 90.0)])
    (s,) = _detector(masks).detect(frame).socks
    assert s.touches_roi_edge


def test_best_view_first_and_avoided_points():
    near = ((330, 250), (_px(180), _px(80)), 0.0)
    far = ((560, 90), (_px(180), _px(80)), 0.0)
    frame, masks = _frame([far, near])
    socks = _detector(masks).detect(frame).socks
    assert [s.grasp.px.u > 450 for s in socks] == [False, True]
    avoided = _detector(masks).detect(frame, avoid=[socks[0].grasp.px]).socks
    assert len(avoided) == 1 and avoided[0].grasp.px.u > 450

"""Color classifier on rendered sim frames, with a SAM3 stand-in: items = depth above the mat."""

import time

import cv2
import numpy as np
import pytest

from sorter.app import build_system
from sorter.color_classifier import classifier as clf
from sorter.color_classifier.classifier import (
    Sam3ColorClassifier,
    color_stats,
    decide,
    regrasp_point,
)
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance, SamSegmenter
from sorter.core.config import Backend
from sorter.core.types import ColorClass, Command, Frame, Intrinsics, Phase, Zone
from sorter.orchestrator.state_machine import StateMachine
from sorter.sim.world import BG_ITEM_HEIGHT_MM


def depth_segmenter(frame_depth: dict):
    """Instances = connected regions standing out of the mat in depth. Reads the frame's depth."""

    def segment(color: np.ndarray) -> list[Instance]:
        d = frame_depth["d"]
        valid = d > 0
        mat = np.median(d[valid])
        up = (valid & (d < mat - 10)).astype(np.uint8)
        n, labels = cv2.connectedComponents(up)
        return [Instance(labels == i, 0.9) for i in range(1, n)]

    return segment


def sim_frame(sim_config, colors: list[ColorClass], seed: int = 0) -> tuple[Frame, list]:
    sim_config.sim.seed = seed
    sim_config.sim.items = [c.value for c in colors] or ["light"]
    system = build_system(sim_config, sim=True)
    world = system.world
    placed = []
    for it in world.at("box")[: len(colors)]:
        it.x, it.y = world.free_point_on_background()
        it.location, it.height_mm = "background", BG_ITEM_HEIGHT_MM
        placed.append(it)
    system.arm.start()
    system.arm.look(Zone.BACKGROUND)
    return system.camera.fresh(), placed


def classify(frame: Frame, cfg: ColorClassifierConfig | None = None, roi=()):
    c = Sam3ColorClassifier(
        cfg or ColorClassifierConfig(), depth_segmenter({"d": frame.depth_mm}), roi
    )
    return c.classify(frame)


@pytest.mark.parametrize("seed", range(4))
@pytest.mark.parametrize("color", list(ColorClass))
def test_each_class(sim_config, color, seed):
    frame, placed = sim_frame(sim_config, [color], seed)
    bg = classify(frame)
    assert [it.color for it in bg.items] == [color], [it.stats for it in bg.items]
    it = bg.items[0]
    assert 0.5 <= it.confidence <= 1.0
    assert it.grasp.depth_mm > 0
    assert frame.depth_mm[it.grasp.px.v, it.grasp.px.u] > 0
    assert {"L", "a", "b", "chroma"} <= it.stats.keys()


def test_empty_background(sim_config):
    frame, _ = sim_frame(sim_config, [])
    bg = classify(frame)
    assert bg.items == []


@pytest.mark.parametrize("seed", range(3))
def test_three_items_largest_first(sim_config, seed):
    colors = [ColorClass.LIGHT, ColorClass.DARK, ColorClass.COLORED]
    frame, placed = sim_frame(sim_config, colors, seed)
    bg = classify(frame)
    assert sorted(it.color for it in bg.items) == sorted(colors)
    areas = [it.area_px for it in bg.items]
    assert areas == sorted(areas, reverse=True)
    assert bg.overlay.mask is not None and bg.overlay.mask.any()
    assert len(bg.overlay.markers) == 3 and len(bg.overlay.polygons) == 3


def _synthetic(depth_fn=None) -> Frame:
    color = np.full((120, 160, 3), 124, np.uint8)
    depth = np.full((120, 160), 400, np.uint16)
    color[30:90, 40:120] = (40, 40, 200)  # red cloth
    depth[30:90, 40:120] = 380
    if depth_fn:
        depth_fn(depth)
    return Frame(color, depth, Intrinsics(100, 100, 80, 60, 160, 120), 0.0, 0)


def _const(masks):
    return lambda color: [Instance(m, s) for m, s in masks]


def _rect(y0, y1, x0, x1):
    m = np.zeros((120, 160), bool)
    m[y0:y1, x0:x1] = True
    return m


def test_duplicate_and_small_instances_dropped():
    frame = _synthetic()
    cfg = ColorClassifierConfig(min_area_px=100)
    masks = [(_rect(30, 90, 40, 120), 0.9), (_rect(30, 60, 40, 80), 0.6), (_rect(0, 5, 0, 5), 0.8)]
    bg = Sam3ColorClassifier(cfg, _const(masks)).classify(frame)
    assert len(bg.items) == 1 and bg.items[0].color is ColorClass.COLORED


def test_pixels_without_depth_are_not_cloth():
    def fingers(d):
        d[80:120, 0:40] = 0

    frame = _synthetic(fingers)
    cfg = ColorClassifierConfig(min_area_px=100)
    bg = Sam3ColorClassifier(cfg, _const([(_rect(80, 120, 0, 40), 0.9)])).classify(frame)
    assert bg.items == []


def test_roi_edge_and_clip():
    frame = _synthetic()
    cfg = ColorClassifierConfig(min_area_px=100)
    roi = [(60, 0), (159, 0), (159, 119), (60, 119)]  # cuts the cloth at x = 60
    bg = Sam3ColorClassifier(cfg, _const([(_rect(30, 90, 40, 120), 0.9)]), roi).classify(frame)
    (it,) = bg.items
    assert it.touches_roi_edge
    assert it.grasp.px.u >= 60
    inside = Sam3ColorClassifier(cfg, _const([(_rect(30, 90, 40, 120), 0.9)])).classify(frame)
    assert not inside.items[0].touches_roi_edge


def test_regrasp_is_highest_point_inside():
    mask = _rect(30, 90, 40, 120)
    depth = np.full((120, 160), 380, np.uint16)
    depth[35:45, 100:110] = 340  # a fold standing up
    depth[31, 41] = 200  # higher, but on the edge of the blob
    g = regrasp_point(mask, depth, inset_px=5, window_px=5)
    assert 35 <= g.px.v < 45 and 100 <= g.px.u < 110
    assert g.depth_mm == pytest.approx(340)


def test_regrasp_on_flat_cloth_is_central():
    mask = _rect(30, 90, 40, 120)
    depth = np.full((120, 160), 380, np.uint16)
    g = regrasp_point(mask, depth, inset_px=5, window_px=5)
    assert abs(g.px.v - 60) <= 2 and 40 + 25 <= g.px.u <= 120 - 25


@pytest.mark.parametrize(
    ("bgr", "expected"),
    [
        ((240, 240, 240), ColorClass.LIGHT),
        ((200, 220, 235), ColorClass.LIGHT),  # cream
        ((30, 30, 30), ColorClass.DARK),
        ((70, 40, 20), ColorClass.DARK),  # navy
        ((40, 40, 200), ColorClass.COLORED),
        ((40, 200, 230), ColorClass.COLORED),  # yellow
    ],
)
def test_decide(bgr, expected):
    color = np.full((20, 20, 3), bgr, np.uint8)
    stats = color_stats(color, np.ones((20, 20), bool), erode_px=2)
    assert decide(stats, ColorClassifierConfig())[0] is expected


@pytest.mark.parametrize("seed", [0, 1])
def test_loop_with_real_classifier(sim_config, monkeypatch, seed):
    sim_config.sim.seed = seed
    sim_config.backends.color_classifier = Backend.REAL
    sim_config.color_classifier.sam.api_key = "test"
    last = {}
    monkeypatch.setattr(SamSegmenter, "segment", lambda self, color: depth_segmenter(last)(color))
    real_classify = clf.Sam3ColorClassifier.classify

    def classify(self, frame):
        last["d"] = frame.depth_mm
        return real_classify(self, frame)

    monkeypatch.setattr(clf.Sam3ColorClassifier, "classify", classify)

    system = build_system(sim_config)
    assert isinstance(system.color_classifier, clf.Sam3ColorClassifier)
    sm = StateMachine(system)
    system.hub.send(Command.START)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline and sm.phase not in (Phase.DONE, Phase.ERROR):
        sm.poll()

    status = system.hub.status()
    assert status.phase is Phase.DONE, status.error
    assert all(it.location == "bin" and it.bin is it.color for it in system.world.items)
    expected = {c: sum(it.color is c for it in system.world.items) for c in ColorClass}
    assert status.counters == expected


def test_rgb_only_frame_classifies_and_grasps_the_middle():
    """The SO-101 wrist camera has no depth (D-014): grasp = deepest point inside the blob."""
    f = _synthetic()
    rgb = Frame(f.color, None, f.intrinsics, 0.0, 0)
    bg = Sam3ColorClassifier(
        ColorClassifierConfig(min_area_px=100), _const([(_rect(30, 90, 40, 120), 0.9)])
    ).classify(rgb)
    (it,) = bg.items
    assert it.color is ColorClass.COLORED
    assert it.grasp.depth_mm is None
    assert (it.grasp.px.u, it.grasp.px.v) == (pytest.approx(80, abs=12), pytest.approx(60, abs=2))

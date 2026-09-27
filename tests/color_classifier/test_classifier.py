"""Color classifier: the floor detector's colors on the rendered floor scene (parquet, the wrist
camera's noise), and its building blocks on synthetic frames."""

import numpy as np
import pytest

from sorter.app import build_system
from sorter.color_classifier.classifier import (
    Sam3ColorClassifier,
    color_stats,
    decide,
    regrasp_point,
)
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.color_classifier.segmenter import Instance
from sorter.core.types import ColorClass, Frame, Intrinsics, Zone


def _floor_socks(sim_config, colors: list[ColorClass], seed: int):
    """Socks of `colors` in the floor view, seen from `look_floor` and detected."""
    cfg = sim_config.model_copy(deep=True)
    cfg.sim.scenes = ["load"]
    cfg.sim.seed = seed
    cfg.sim.load.socks = colors
    cfg.sim.load.area = "view"
    system = build_system(cfg, sim=True)
    system.camera.start()
    system.arm.start()
    try:
        obs = system.observer.observe(Zone.FLOOR)
        return system.floor_detector.detect(obs.frame)
    finally:
        system.camera.close()
        system.world.stop()


@pytest.mark.parametrize("seed", range(3))
@pytest.mark.parametrize("color", list(ColorClass))
def test_each_class_on_the_floor(sim_config, color, seed):
    result = _floor_socks(sim_config, [color], seed)
    assert [s.color for s in result.socks] == [color], [s.stats for s in result.socks]
    sock = result.socks[0]
    assert 0.5 <= sock.confidence <= 1.0
    assert {"L", "a", "b", "chroma", "area_mm2"} <= sock.stats.keys()


def test_three_socks_on_the_floor(sim_config):
    colors = [ColorClass.LIGHT, ColorClass.DARK, ColorClass.COLORED]
    result = _floor_socks(sim_config, colors, 1)
    assert sorted(s.color for s in result.socks) == sorted(colors)
    assert result.overlay.mask is not None and result.overlay.mask.any()


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
        ((95, 58, 33), ColorClass.DARK),  # navy under a lamp: L* ~25, chroma ~25
        ((30, 30, 140), ColorClass.COLORED),  # a deep red: dim but bold
        ((40, 40, 200), ColorClass.COLORED),
        ((40, 200, 230), ColorClass.COLORED),  # yellow
    ],
)
def test_decide(bgr, expected):
    color = np.full((20, 20, 3), bgr, np.uint8)
    stats = color_stats(color, np.ones((20, 20), bool), erode_px=2)
    assert decide(stats, ColorClassifierConfig())[0] is expected

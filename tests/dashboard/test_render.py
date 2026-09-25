import numpy as np

from sorter.core.types import (
    Decision,
    Frame,
    Intrinsics,
    Marker,
    Observation,
    Overlay,
    Phase,
    PixelPoint,
    Zone,
)
from sorter.dashboard.render import draw_overlay, placeholder, render_decision

GRAY = (128, 128, 128)


def _frame(w=640, h=480) -> Frame:
    color = np.full((h, w, 3), GRAY, np.uint8)
    depth = np.zeros((h, w), np.uint16)
    return Frame(color, depth, Intrinsics(500, 500, w / 2, h / 2, w, h), 0.0, 0)


def test_overlay_is_drawn_on_a_copy():
    f = _frame()
    img = draw_overlay(f.color, Overlay(markers=[Marker(PixelPoint(300, 200), "", "grasp")]))
    assert (f.color == GRAY).all()
    assert (img[200, 300] != GRAY).any()  # crosshair center
    assert (img[10, 10] == GRAY).all()


def test_mask_and_roi():
    f = _frame()
    mask = np.zeros((480, 640), bool)
    mask[100:150, 100:150] = True
    roi = [(20, 20), (620, 20), (620, 460), (20, 460)]
    img = draw_overlay(f.color, Overlay(mask=mask), roi)
    assert (img[120, 120] != GRAY).any()  # tinted
    assert (img[300, 300] == GRAY).all()  # outside the mask
    assert (img[20, 320] != GRAY).any()  # on the ROI edge


def test_mask_of_wrong_shape_is_ignored():
    f = _frame()
    img = draw_overlay(f.color, Overlay(mask=np.ones((10, 10), bool)))
    assert (img == GRAY).all()


def test_decision_gets_a_caption_strip():
    obs = Observation(_frame(), Zone.BACKGROUND, None, None)
    d = Decision(Phase.SENSE_BG, obs, Overlay(), "colored 0.93")
    img = render_decision(d, {})
    assert img.shape[1] == 640 and img.shape[0] > 480
    assert (img[:480] == GRAY).all()  # no overlay, no ROI: the frame is untouched


def test_placeholder_size():
    assert placeholder("x", 320, 240).shape == (240, 320, 3)

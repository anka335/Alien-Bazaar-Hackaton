"""Server-side drawing: the decision frame with its ROI and overlay, the live feed, placeholders.

Everything is drawn onto the frame the overlay was computed on, so the two always match.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence

import cv2
import numpy as np

from sorter.core.types import Decision, Marker, Overlay, Phase, Zone

# What the audience reads: plain words, not enum names. The page gets the same labels.
PHASE_LABELS: dict[Phase, str] = {
    Phase.IDLE: "Ready",
    Phase.STARTING: "Starting up",
    Phase.SCAN: "Looking at the floor",
    Phase.SENSE_FLOOR: "Finding socks",
    Phase.AIM: "A closer look at the sock",
    Phase.PICK_FROM_FLOOR: "Picking up the sock",
    Phase.DROP_TO_CARGO: "Into the cargo box",
    Phase.CHECK_LOAD: "Checking it landed in the box",
    Phase.LOOK_CARGO: "Looking into the cargo box",
    Phase.SENSE_CARGO: "Choosing what to grab",
    Phase.PICK_FROM_CARGO: "Grabbing from the cargo box",
    Phase.DROP_TO_LAUNDRY: "Into the laundry bin",
    Phase.DONE: "All done",
    Phase.HELD: "Held",
    Phase.ERROR: "Stopped on an error",
}

# BGR. Bright colors that stand out on the floor and in the cargo box.
_GRASP = (80, 220, 40)
_CANDIDATE = (0, 210, 255)
_AVOID = (40, 40, 230)
_INFO = (255, 255, 255)
_ROI = (255, 255, 255)
_POLY = (255, 200, 0)
_MASK = np.array([255, 190, 0], np.float32)
_GLASS = (35, 24, 15)  # the page's control panel
_LCD = (251, 247, 233)  # its display text
_LCD_DIM = (171, 149, 127)
_FONT = cv2.FONT_HERSHEY_DUPLEX


def _scale(img: np.ndarray) -> float:
    return img.shape[1] / 640


def _label(
    img: np.ndarray,
    text: str,
    org: tuple[int, int],
    color: tuple[int, int, int],
    flip_x: int | None = None,
) -> None:
    """Text on a dark box so it reads on any background. `org` is the box's top-left.

    If the box doesn't fit to the right and `flip_x` is set, it ends at `flip_x` instead.
    """
    s = _scale(img)
    fs, th = 0.5 * s, max(1, round(s))
    (w, h), base = cv2.getTextSize(text, _FONT, fs, th)
    pad = round(4 * s)
    x = org[0]
    if flip_x is not None and x + w + 2 * pad > img.shape[1]:
        x = flip_x - w - 2 * pad
    x = min(max(x, 0), max(img.shape[1] - w - 2 * pad, 0))
    y = min(max(org[1], 0), max(img.shape[0] - h - base - 2 * pad, 0))
    cv2.rectangle(img, (x, y), (x + w + 2 * pad, y + h + base + 2 * pad), (20, 20, 20), -1)
    cv2.putText(img, text, (x + pad, y + pad + h), _FONT, fs, color, th, cv2.LINE_AA)


def _marker(img: np.ndarray, m: Marker) -> None:
    s = _scale(img)
    p = (m.px.u, m.px.v)
    r, th = round(14 * s), max(2, round(2 * s))
    if m.kind == "grasp":
        cv2.circle(img, p, r, (0, 0, 0), th + 2, cv2.LINE_AA)
        cv2.circle(img, p, r, _GRASP, th, cv2.LINE_AA)
        cv2.drawMarker(img, p, _GRASP, cv2.MARKER_CROSS, round(2.6 * r), th, cv2.LINE_AA)
        color = _GRASP
    elif m.kind == "avoid":
        cv2.drawMarker(img, p, _AVOID, cv2.MARKER_TILTED_CROSS, round(1.6 * r), th + 1, cv2.LINE_AA)
        color = _AVOID
    elif m.kind == "candidate":
        cv2.circle(img, p, round(r * 0.6), _CANDIDATE, th, cv2.LINE_AA)
        color = _CANDIDATE
    else:
        cv2.circle(img, p, round(4 * s), _INFO, -1, cv2.LINE_AA)
        color = _INFO
    if m.label:
        _label(img, m.label, (p[0] + r + 4, p[1] - r), color, flip_x=p[0] - r - 4)


def _polyline(img: np.ndarray, pts: Sequence, color: tuple[int, int, int], th: int) -> None:
    arr = np.array([(p[0], p[1]) for p in pts], np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [arr], True, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.polylines(img, [arr], True, color, th, cv2.LINE_AA)


def draw_overlay(
    color: np.ndarray, overlay: Overlay, roi: Sequence[tuple[int, int]] = ()
) -> np.ndarray:
    """A copy of `color` (BGR) with the zone ROI and the overlay drawn on it."""
    img = color.copy()
    s = _scale(img)
    mask = overlay.mask
    if mask is not None and mask.shape == img.shape[:2]:
        m = mask.astype(bool)
        img[m] = (img[m] * 0.55 + _MASK * 0.45).astype(np.uint8)
    if len(roi) >= 3:
        _polyline(img, roi, _ROI, max(1, round(s)))
    for pts, label in overlay.polygons:
        if len(pts) >= 2:
            _polyline(img, [(p.u, p.v) for p in pts], _POLY, max(2, round(2 * s)))
            if label:
                _label(img, label, (pts[0].u, pts[0].v - round(24 * s)), _POLY)
    for m in overlay.markers:
        _marker(img, m)
    y = round(6 * s)
    for line in overlay.text:
        _label(img, line, (round(6 * s), y), _INFO)
        y += round(24 * s)
    return img


def _caption(img: np.ndarray, title: str, detail: str) -> np.ndarray:
    """Add a strip under the image. It is part of the frame, so it can't go out of sync."""
    s = _scale(img)
    h = round(44 * s)
    strip = np.full((h, img.shape[1], 3), _GLASS, np.uint8)
    fs, th = 0.7 * s, max(1, round(1.5 * s))
    (tw, th_px), _ = cv2.getTextSize(title, _FONT, fs, th)
    base_y = (h + th_px) // 2
    x = round(10 * s)
    cv2.putText(strip, title, (x, base_y), _FONT, fs, _LCD, th, cv2.LINE_AA)
    if detail:
        dx = x + tw + round(16 * s)
        cv2.putText(strip, detail, (dx, base_y), _FONT, 0.6 * s, _LCD_DIM, th, cv2.LINE_AA)
    return np.vstack([img, strip])


def render_decision(d: Decision, rois: Mapping[Zone, Sequence[tuple[int, int]]]) -> np.ndarray:
    img = draw_overlay(d.obs.frame.color, d.overlay, rois.get(d.obs.zone, ()))
    return _caption(img, PHASE_LABELS.get(d.phase, d.phase.value), d.summary)


def placeholder(text: str, width: int = 640, height: int = 480) -> np.ndarray:
    img = np.full((height, width, 3), (24, 17, 11), np.uint8)
    s = width / 640
    fs, th = 0.8 * s, max(1, round(1.5 * s))
    (tw, h), _ = cv2.getTextSize(text, _FONT, fs, th)
    cv2.putText(
        img, text, ((width - tw) // 2, (height + h) // 2), _FONT, fs, _LCD_DIM, th, cv2.LINE_AA
    )
    return img


def encode_jpeg(img: np.ndarray, quality: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return buf.tobytes()

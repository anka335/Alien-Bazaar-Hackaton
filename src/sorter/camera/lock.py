"""Lock the color sensor at what auto exposure and auto white balance settled on.

The sensor does not report the values its auto modes picked (`get_option` returns the last manual
value, and on macOS the frames carry no exposure metadata), so they are found by search: with auto
off, white balance is bisected until the red/blue ratio matches the auto frame, then exposure
(and gain, if the exposure cap is not enough) until the brightness matches.
"""

from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np


def red_blue(img: np.ndarray) -> float:
    """Mean red over mean blue of a BGR image: rises with the white balance temperature."""
    b, _, r = img.reshape(-1, 3).mean(axis=0)
    return float(r) / max(float(b), 1e-3)


def luma(img: np.ndarray) -> float:
    """Mean brightness of a BGR image."""
    b, g, r = img.reshape(-1, 3).mean(axis=0)
    return float(0.114 * b + 0.587 * g + 0.299 * r)


def bisect(
    measure: Callable[[float], float],
    lo: float,
    hi: float,
    target: float,
    steps: int,
    geometric: bool = False,
) -> float:
    """The value in [lo, hi] where the rising `measure` is closest to `target`."""
    best, best_err = lo, math.inf
    for _ in range(steps):
        mid = math.sqrt(lo * hi) if geometric else (lo + hi) / 2
        m = measure(mid)
        if abs(m - target) < best_err:
            best, best_err = mid, abs(m - target)
        if m < target:
            lo = mid
        else:
            hi = mid
    return best

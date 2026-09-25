import numpy as np
import pytest

from sorter.core.types import Frame, Intrinsics


def test_frame_arrays_are_read_only():
    k = Intrinsics(fx=500, fy=500, cx=2, cy=1, width=4, height=2)
    f = Frame(np.zeros((2, 4, 3), np.uint8), np.zeros((2, 4), np.uint16), k, 0.0, 0)
    with pytest.raises(ValueError):
        f.color[0, 0, 0] = 1
    with pytest.raises(ValueError):
        f.depth_mm[0, 0] = 1

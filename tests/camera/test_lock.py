import numpy as np
import pytest

from sorter.camera import lock


@pytest.mark.parametrize("geometric", [False, True])
def test_bisect_finds_where_a_rising_measure_meets_the_target(geometric):
    calls = []

    def measure(v):
        calls.append(v)
        return 2.0 * v

    v = lock.bisect(measure, 1, 1000, target=500, steps=12, geometric=geometric)
    assert v == pytest.approx(250, rel=0.02)
    assert len(calls) == 12


def test_bisect_clamps_to_the_range_end():
    assert lock.bisect(lambda v: v, 1, 100, target=1e6, steps=10) == pytest.approx(100, rel=0.01)


def test_red_blue_and_luma():
    img = np.zeros((2, 2, 3), np.uint8)
    img[..., 0], img[..., 2] = 50, 100  # BGR
    assert lock.red_blue(img) == pytest.approx(2.0)
    assert lock.luma(np.full((2, 2, 3), 80, np.uint8)) == pytest.approx(80)

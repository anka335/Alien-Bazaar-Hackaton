import pytest

from rebot_b601 import kinematics as K


@pytest.fixture(autouse=True)
def _urdf_base_frame():
    """The URDF base frame: a caller (the sorter's config) may have turned it (set_base)."""
    K.set_base(0.0)
    yield
    K.set_base(0.0)

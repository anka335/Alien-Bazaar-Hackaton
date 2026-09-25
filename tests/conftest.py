import pytest

from sorter.core.config import Config, load_config


@pytest.fixture
def sim_config() -> Config:
    """The committed config, with instant arm motions."""
    return load_config(overrides={"sim": {"motion_s": 0.0}})

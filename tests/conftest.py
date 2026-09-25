import pytest

from sorter.core.config import Config, load_config


@pytest.fixture
def sim_config() -> Config:
    """The committed config, with instant arm motions and no run logs."""
    return load_config(overrides={"sim": {"motion_s": 0.0}, "state_machine": {"save_runs": False}})

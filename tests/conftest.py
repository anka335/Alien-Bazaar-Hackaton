import pytest

from sorter.core.config import Config, load_config


@pytest.fixture
def sim_config() -> Config:
    """The committed config on the kinematic sim with every component simulated, instant arm
    motions and vision, and no run logs."""
    cfg = load_config(
        overrides={
            "sim": {"engine": "kinematic", "time_scale": 0.0, "vision_s": 0.0},
            "backends": dict.fromkeys(
                ("camera", "arm", "calibration", "box_detector", "color_classifier"), "sim"
            ),
            "state_machine": {"save_runs": False},
        }
    )
    cfg.views = {}  # the ROIs are for the pinhole camera; the kinematic renderer is top-down
    return cfg

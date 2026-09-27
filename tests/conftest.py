import pytest

from sorter.core.config import Config, load_config


@pytest.fixture
def sim_config() -> Config:
    """The committed config on the physics sim, as fast as the CPU allows, with the hardware
    and the vision simulated (the floor detector on the render's segmentation), the empty scene
    (the base: floor, rover, cargo box, arm; tests add what they need to `sim.scenes`), the
    arm at full speed, and no run logs."""
    return load_config(
        overrides={
            "sim": {"realtime": 0, "scenes": []},
            "backends": dict.fromkeys(("camera", "arm"), "sim"),
            "arm": {"speed_scale": 0.6},
            "state_machine": {"save_runs": False},
        }
    )

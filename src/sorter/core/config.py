"""Config loading: YAML files deep-merged in order, validated with pydantic.

Each block owns the model of its section in `sorter/<package>/config.py`.
"""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field

from sorter.arm.config import ArmConfig, ZoneConfig
from sorter.box_detector.config import BoxDetectorConfig
from sorter.calibration.config import CalibrationConfig
from sorter.camera.config import CameraConfig, ViewConfig
from sorter.color_classifier.config import ColorClassifierConfig
from sorter.core.types import Zone
from sorter.dashboard.config import DashboardConfig
from sorter.orchestrator.config import StateMachineConfig
from sorter.sim.config import SimConfig

DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[3] / "config"
CONFIG_FILES = ("default.yaml", "rig.yaml", "calibration.yaml", "local.yaml")


class Backend(StrEnum):
    REAL = "real"
    SIM = "sim"


class BackendsConfig(BaseModel):
    """`backends`: real or sim, per component."""

    model_config = ConfigDict(extra="forbid")

    camera: Backend = Backend.SIM
    arm: Backend = Backend.SIM
    calibration: Backend = Backend.SIM
    box_detector: Backend = Backend.SIM
    color_classifier: Backend = Backend.SIM


class Config(BaseModel):
    model_config = ConfigDict(extra="forbid")

    backends: BackendsConfig = Field(default_factory=BackendsConfig)
    sim: SimConfig = Field(default_factory=SimConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    views: dict[Zone, ViewConfig] = Field(default_factory=dict)
    calibration: CalibrationConfig = Field(default_factory=CalibrationConfig)
    box_detector: BoxDetectorConfig = Field(default_factory=BoxDetectorConfig)
    color_classifier: ColorClassifierConfig = Field(default_factory=ColorClassifierConfig)
    arm: ArmConfig = Field(default_factory=ArmConfig)
    poses: dict[str, list[float]] = Field(default_factory=dict)  # joint angles, rad
    zones: dict[Zone, ZoneConfig] = Field(default_factory=dict)
    state_machine: StateMachineConfig = Field(default_factory=StateMachineConfig)
    dashboard: DashboardConfig = Field(default_factory=DashboardConfig)


def deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    """Merge `over` into a copy of `base`. Nested dicts merge; everything else is replaced."""
    out = dict(base)
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def load_config(
    config_dir: str | Path = DEFAULT_CONFIG_DIR, overrides: dict[str, Any] | None = None
) -> Config:
    """Load CONFIG_FILES from `config_dir` (missing ones are skipped, except default.yaml)."""
    config_dir = Path(config_dir)
    if not (config_dir / CONFIG_FILES[0]).is_file():
        raise FileNotFoundError(f"{config_dir / CONFIG_FILES[0]} not found")
    data: dict[str, Any] = {}
    for name in CONFIG_FILES:
        path = config_dir / name
        if path.is_file():
            data = deep_merge(data, yaml.safe_load(path.read_text()) or {})
    if overrides:
        data = deep_merge(data, overrides)
    return Config.model_validate(data)


def update_yaml(path: str | Path, updates: dict[str, Any]) -> None:
    """Deep-merge `updates` into a YAML file (created if missing), keeping its leading comments.

    Used by the setup tools to write `rig.yaml` and `calibration.yaml`. Other comments are lost.
    """
    path = Path(path)
    text = path.read_text() if path.is_file() else ""
    header = []
    for line in text.splitlines():
        if not line.startswith("#"):
            break
        header.append(line)
    data = deep_merge(yaml.safe_load(text) or {}, updates)
    body = yaml.safe_dump(data, sort_keys=False, default_flow_style=None, width=100)
    path.write_text("\n".join(header) + ("\n" if header else "") + body)

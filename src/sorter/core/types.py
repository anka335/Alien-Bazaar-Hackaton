"""Shared data types. Contracts: docs/architecture.md → Contracts."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from typing import Literal

import numpy as np

Pose = np.ndarray  # 4x4 float64, homogeneous transform, translation in mm


def readonly(a: np.ndarray) -> np.ndarray:
    """Mark an array read-only in place and return it."""
    a.flags.writeable = False
    return a


# --- Basic types -------------------------------------------------------------


class Zone(StrEnum):
    BOX = "box"
    BACKGROUND = "background"


class ColorClass(StrEnum):
    LIGHT = "light"
    DARK = "dark"
    COLORED = "colored"


@dataclass(frozen=True)
class PixelPoint:
    u: int
    v: int


@dataclass(frozen=True)
class ArmPoint:
    x: float  # mm, arm base frame
    y: float
    z: float


# --- Camera (block 1) --------------------------------------------------------


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    coeffs: tuple[float, ...] = ()  # distortion; empty = already rectified


@dataclass(frozen=True)
class Frame:
    color: np.ndarray  # HxWx3 uint8, BGR
    depth_mm: np.ndarray  # HxW uint16, Z in mm, aligned to color; 0 = no data
    intrinsics: Intrinsics
    timestamp: float  # monotonic, when the frame arrived
    seq: int

    def __post_init__(self) -> None:
        readonly(self.color)
        readonly(self.depth_mm)


# --- Observation (block 0) ---------------------------------------------------


@dataclass(frozen=True)
class Observation:
    frame: Frame
    zone: Zone
    T_base_cam: Pose | None  # None only in recordings made without the arm
    joints: tuple[float, ...] | None  # arm joints at capture, rad

    def __post_init__(self) -> None:
        if self.T_base_cam is not None:
            readonly(self.T_base_cam)


# --- Vision: shared types ----------------------------------------------------


@dataclass(frozen=True)
class GraspPoint:
    px: PixelPoint
    depth_mm: float  # robust Z of the cloth SURFACE at px; never 0


@dataclass
class Marker:
    px: PixelPoint
    label: str
    kind: Literal["grasp", "candidate", "avoid", "info"]


@dataclass
class Overlay:
    """Drawn by the dashboard on top of the frame it came from."""

    markers: list[Marker] = field(default_factory=list)
    polygons: list[tuple[list[PixelPoint], str]] = field(default_factory=list)
    mask: np.ndarray | None = None  # HxW bool
    text: list[str] = field(default_factory=list)


# --- Box detector (block 3) --------------------------------------------------


class BoxStatus(StrEnum):
    GRASP = "grasp"
    EMPTY = "empty"
    NO_GRASP = "no_grasp"


@dataclass
class BoxResult:
    status: BoxStatus
    grasp: GraspPoint | None  # set only when status == GRASP
    coverage: float  # 0..1, share of the ROI covered by cloth
    overlay: Overlay


# --- Color classifier (block 4) ----------------------------------------------


@dataclass
class ItemResult:
    color: ColorClass
    confidence: float  # 0..1
    grasp: GraspPoint  # re-grasp point
    area_px: int
    touches_roi_edge: bool  # item partly outside the background ROI
    stats: dict[str, float]  # e.g. median L, a, b, chroma


@dataclass
class BackgroundResult:
    items: list[ItemResult]  # one per blob, largest first; [] = background empty
    overlay: Overlay


# --- Arm controller (block 5) ------------------------------------------------


@dataclass(frozen=True)
class PickResult:
    gripper_opening: float  # 0 = fully closed .. 1 = fully open, after closing
    likely_empty: bool  # a hint; the camera is the truth


# --- Hub: state machine ↔ dashboard (block 0) --------------------------------


class Phase(StrEnum):
    IDLE = "idle"
    STARTING = "starting"
    LOOK_BG = "look_bg"
    SENSE_BG = "sense_bg"
    PICK_FROM_BG = "pick_from_bg"
    DROP_TO_BIN = "drop_to_bin"
    LOOK_BOX = "look_box"
    SENSE_BOX = "sense_box"
    PICK_FROM_BOX = "pick_from_box"
    PLACE_ON_BG = "place_on_bg"
    DONE = "done"
    HELD = "held"
    ERROR = "error"


class Command(StrEnum):
    START = "start"
    PAUSE = "pause"
    RESUME = "resume"
    STEP = "step"
    STOP = "stop"
    HOLD = "hold"
    RESET = "reset"


Mode = Literal["idle", "running", "paused"]


@dataclass(frozen=True)
class Event:
    t: float  # monotonic
    level: Literal["info", "warning", "error"]
    source: str
    msg: str


def _zero_counters() -> dict[ColorClass, int]:
    return {c: 0 for c in ColorClass}


@dataclass(frozen=True)
class Status:
    phase: Phase = Phase.IDLE
    next_phase: Phase | None = None  # what STEP will run
    mode: Mode = "idle"
    run_id: str | None = None
    cycle: int = 0
    counters: dict[ColorClass, int] = field(default_factory=_zero_counters)
    failures: int = 0  # consecutive
    last_cycle_s: float | None = None
    error: str | None = None
    health: list[str] = field(default_factory=list)  # startup problems; empty = OK
    events: list[Event] = field(default_factory=list)  # last ~50


@dataclass(frozen=True)
class Decision:
    """What the state machine decided on, for the dashboard."""

    phase: Phase
    obs: Observation
    overlay: Overlay
    summary: str  # e.g. "grasp (412, 230) depth 540 mm" / "colored 0.93"

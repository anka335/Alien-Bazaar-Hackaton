"""Config models for the simulator (`sim`, stage 0) and the rover layout the rig comes from."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass
from sorter.sim.scenes.load.config import LoadSceneConfig
from sorter.sim.scenes.unload.config import UnloadSceneConfig


class RectConfig(BaseModel):
    """An axis-aligned rectangle in the arm frame, mm."""

    model_config = ConfigDict(extra="forbid")

    center_mm: tuple[float, float]
    size_mm: tuple[float, float]  # along x, along y

    def bounds(self, grow: float = 0.0) -> tuple[float, float, float, float]:
        """x0, x1, y0, y1, grown by `grow` mm on every side (negative: shrunk)."""
        (cx, cy), (w, h) = self.center_mm, self.size_mm
        return cx - w / 2 - grow, cx + w / 2 + grow, cy - h / 2 - grow, cy + h / 2 + grow

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        x0, x1, y0, y1 = self.bounds(-margin)
        return x0 <= x <= x1 and y0 <= y <= y1


class CargoLayout(RectConfig):
    """The cargo box on the rover deck: `size_mm` is the inside, split along x into one
    compartment per color, in the order of `compartments` (from −x to +x)."""

    floor_z_mm: float = 5.0  # the inside floor, above the deck
    wall_mm: float = 50.0  # wall height above the inside floor (the gripper held down reaches
    # ~100 mm above the deck at most)
    wall_t_mm: float = 6.0  # wall and divider thickness
    compartments: list[ColorClass] = Field(
        default_factory=lambda: [ColorClass.LIGHT, ColorClass.DARK, ColorClass.COLORED]
    )

    @property
    def rim_z_mm(self) -> float:
        return self.floor_z_mm + self.wall_mm

    def compartment(self, color: ColorClass) -> RectConfig:
        """The inside of the compartment for `color`."""
        n = len(self.compartments)
        (cx, cy), (w, h) = self.center_mm, self.size_mm
        inner = (w - (n - 1) * self.wall_t_mm) / n
        k = self.compartments.index(color)
        x = cx - w / 2 + inner / 2 + k * (inner + self.wall_t_mm)
        return RectConfig(center_mm=(x, cy), size_mm=(inner, h))


class LaundryLayout(BaseModel):
    """The unload station's laundry bins, on the floor, where the rover parks next to them."""

    model_config = ConfigDict(extra="forbid")

    centers_mm: dict[ColorClass, tuple[float, float]] = Field(
        default_factory=lambda: {
            ColorClass.LIGHT: (-140.0, -330.0),
            ColorClass.DARK: (100.0, -330.0),
            ColorClass.COLORED: (340.0, -300.0),
        }
    )
    size_mm: float = 220.0  # square, outside
    height_mm: float = 150.0  # rim above the floor
    wall_t_mm: float = 10.0


class RoverLayout(BaseModel):
    """The rover around the arm, arm base frame (+x forward, +y left, z up), mm.

    The arm stands on the rover's deck plate; the deck top is z = 0 and the floor is
    `floor_z_mm` below it. The committed `poses`, `zones`, `views` and the arm's keep-out in
    `rig.yaml` are computed from this layout (`python -m sorter.sim.layout --write`).
    """

    model_config = ConfigDict(extra="forbid")

    floor_z_mm: float = -200.0  # the deck is 200 mm above the floor
    # the whole rover seen from above (chassis + wheels): nothing of the arm goes in there below
    # the deck
    body: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(-150.0, 80.0), size_mm=(500.0, 480.0))
    )
    # the deck plate the arm is bolted to (its top is z = 0)
    deck: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(-150.0, 80.0), size_mm=(480.0, 470.0))
    )
    # to the arm's left: joint 1 turns ±145°, so nothing right behind the arm is reachable
    cargo: CargoLayout = Field(
        default_factory=lambda: CargoLayout(center_mm=(-100.0, 220.0), size_mm=(320.0, 180.0))
    )
    # the patch of floor in front of the rover that `look_floor` frames; the calibration marks
    # and board lie here too
    floor_view: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(300.0, 0.0), size_mm=(280.0, 240.0))
    )
    laundry: LaundryLayout = Field(default_factory=LaundryLayout)


class SimConfig(BaseModel):
    """`sim`: the MuJoCo simulator world."""

    model_config = ConfigDict(extra="forbid")

    realtime: float = 1.0  # simulated seconds per wall second; 0 = as fast as possible
    board: bool = False  # a ChArUco board lies on the floor view (hand-eye calibration)
    marks: bool = False  # tape marks on the floor view (the /calibrate page; on in setup mode)
    use_sam3: bool = False  # the real SAM3 service segments the rendered frames
    seed: int = 0
    # which scene files add their things to the base (floor, rover, cargo box, arm)
    scenes: list[str] = Field(default_factory=lambda: ["load", "unload"])
    load: LoadSceneConfig = Field(default_factory=LoadSceneConfig)  # stage A's scene
    unload: UnloadSceneConfig = Field(default_factory=UnloadSceneConfig)  # stage B's scene
    miss_prob: float = 0.0  # a closing gripper catches nothing although cloth is between the pads
    width: int = 640
    height: int = 480
    focal_px: float = 615.0  # RealSense D435i color at 640x480
    # the wrist camera in the TCP frame (x = approach); it looks along the approach axis
    camera_mount_mm: tuple[float, float, float] = (-140.0, 0.0, 55.0)
    # or the whole mount, T_link5_cam (4x4, mm), e.g. the rig's measured hand-eye result: it
    # replaces `camera_mount_mm` and its turn when set
    camera_mount_T: list[list[float]] | None = None
    layout: RoverLayout = Field(default_factory=RoverLayout)

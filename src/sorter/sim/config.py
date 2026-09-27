"""Config models for the simulator (`sim`, stage 0) and the rover layout the rig comes from."""

from pydantic import BaseModel, ConfigDict, Field

from sorter.core.types import ColorClass
from sorter.sim.scenes.load.config import LoadSceneConfig
from sorter.sim.scenes.unload.config import UnloadSceneConfig

RAIL_INSET_MM = 20.0  # the rover's rails and crossbars end this far inside the wheels' outer edges


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
    """The cargo box on the rover: `size_mm` is the inside. With `compartments` it is split
    along x into one compartment per color, in that order (from −x to +x); without, it is one
    box every sock goes into (D-040)."""

    floor_z_mm: float = -35.0  # the inside floor (the deck top is z = 0)
    wall_mm: float = 60.0  # wall height above the inside floor (the gripper held down reaches
    # ~100 mm above the deck at most)
    wall_t_mm: float = 4.0  # wall, bottom and divider thickness
    compartments: list[ColorClass] = Field(default_factory=list)

    @property
    def rim_z_mm(self) -> float:
        return self.floor_z_mm + self.wall_mm

    @property
    def base_z_mm(self) -> float:
        """The underside of the box."""
        return self.floor_z_mm - self.wall_t_mm

    def insides(self) -> list[RectConfig]:
        """The compartments from −x to +x, or the whole inside if the box isn't split."""
        return [self.compartment(c) for c in self.compartments or [ColorClass.LIGHT]]

    def compartment(self, color: ColorClass) -> RectConfig:
        """The inside of the compartment for `color`: the whole box if it isn't split."""
        n = len(self.compartments)
        if n == 0:
            return RectConfig(center_mm=self.center_mm, size_mm=self.size_mm)
        (cx, cy), (w, h) = self.center_mm, self.size_mm
        inner = (w - (n - 1) * self.wall_t_mm) / n
        k = self.compartments.index(color)
        x = cx - w / 2 + inner / 2 + k * (inner + self.wall_t_mm)
        return RectConfig(center_mm=(x, cy), size_mm=(inner, h))


class LaundryLayout(BaseModel):
    """The unload station's laundry bins: cardboard boxes like the cargo box, on the floor in a
    row across the front of the rover, where it parks next to them (D-043)."""

    model_config = ConfigDict(extra="forbid")

    centers_mm: dict[ColorClass, tuple[float, float]] = Field(
        default_factory=lambda: {
            ColorClass.LIGHT: (290.0, 220.0),
            ColorClass.DARK: (290.0, 0.0),
            ColorClass.COLORED: (290.0, -220.0),
        }
    )
    size_mm: float = 190.0  # square, outside
    sizes_mm: dict[ColorClass, float] = Field(default_factory=dict)  # bins not `size_mm`

    def size(self, color: ColorClass) -> float:
        """The outside side of the bin of `color` (mm)."""
        return self.sizes_mm.get(color, self.size_mm)

    height_mm: float = 75.0  # rim above the floor
    wall_t_mm: float = 4.0


class Block(BaseModel):
    """A solid box on the rover, arm frame, mm: [x0, x1, y0, y1, z0, z1]."""

    model_config = ConfigDict(extra="forbid")

    box_mm: tuple[float, float, float, float, float, float]
    rgba: tuple[float, float, float, float] = (0.6, 0.6, 0.62, 1.0)


def _equipment() -> dict[str, Block]:
    # what stands behind the arm (estimated from photos of the rover): the electronics case,
    # the power supply and the power strip on it
    return {
        "electronics": Block(box_mm=(-200, -55, -100, 95, -75, -5), rgba=(0.72, 0.73, 0.75, 1)),
        # the power supply on the rover's left, the power strip under it (photo from above)
        "psu": Block(box_mm=(-245, 10, 120, 270, -10, 60), rgba=(0.72, 0.73, 0.75, 1)),
        # the Leo's raised top cover behind the arm, higher than the box's rim: it hid the box
        # from a look over the rover's middle (seen by the wrist camera, D-050)
        "leo_top": Block(box_mm=(-300, -50, -128, 128, -10, 50), rgba=(0.8, 0.8, 0.82, 1)),
    }


class RoverLayout(BaseModel):
    """The rover around the arm, arm base frame (+x the rover's forward, +y left, z up), mm;
    the arm itself stands turned by `arm.base_yaw_deg`.

    The arm stands on the rover's deck plate; the deck top is z = 0 and the floor is
    `floor_z_mm` below it. The committed `poses`, `zones`, `views` and the arm's keep-out in
    `rig.yaml` are computed from this layout (`python -m sorter.sim.layout --write`).
    """

    model_config = ConfigDict(extra="forbid")

    floor_z_mm: float = -160.0  # the deck is 160 mm above the floor
    # the whole rover seen from above (chassis + wheels): nothing of the arm goes in there below
    # the deck; the wheels stand at its corners
    body: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(-10.0, 0.0), size_mm=(360.0, 350.0))
    )
    wheel_radius_mm: float = 60.0
    wheel_width_mm: float = 50.0
    # the plate the arm is bolted to (its top is z = 0)
    deck: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(15.0, 0.0), size_mm=(210.0, 190.0))
    )
    equipment: dict[str, Block] = Field(default_factory=_equipment)
    # to the arm's right and a bit behind (D-045): joint 1 turns ±145°, so nothing right behind
    # the arm is reachable
    cargo: CargoLayout = Field(
        default_factory=lambda: CargoLayout(center_mm=(-50.0, -200.0), size_mm=(150.0, 150.0))
    )
    # the patch of floor in front of the rover that `look_floor` frames; the calibration marks
    # and board lie here too
    floor_view: RectConfig = Field(
        default_factory=lambda: RectConfig(center_mm=(310.0, 0.0), size_mm=(280.0, 240.0))
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
    # cloth collides with cloth: a sock dropped on others lies on top (a pile costs ~5x the step)
    cloth_collisions: bool = True
    width: int = 640
    height: int = 480
    focal_px: float = 615.0  # RealSense D435i color at 640x480
    # the wrist camera in the TCP frame (x = approach); it looks along the approach axis
    camera_mount_mm: tuple[float, float, float] = (-140.0, 0.0, 55.0)
    # the simulated camera's mount, T_link5_cam (4x4, mm). None: where the real camera was
    # calibrated (config/hand_eye.yaml, filled in by the config loader), else the nominal mount
    # from `camera_mount_mm`
    camera_T_link5_cam: list[list[float]] | None = None
    layout: RoverLayout = Field(default_factory=RoverLayout)

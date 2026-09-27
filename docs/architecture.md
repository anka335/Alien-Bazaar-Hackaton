# Architecture

Single source of truth for the contracts between the stages. The shared types are in `src/sorter/core/`; if code and this file diverge, fix both in the same PR (see `AGENTS.md`).

## Physical setup

- **Rover** ([D-032](decisions.md)): built and driven by others. The arm is bolted to its deck plate, **200 mm above the floor** (measured, [D-042](decisions.md)). For our code the rover stands still: in load mode it has stopped next to socks, in unload mode it is parked at the station. There is no rover interface yet.
- **Leo Rover, ROS 2 track** ([D-037](decisions.md)): separately from the stages, `ros2_ws/src/rover_nav` drives a real Leo Rover (LeoOS, ROS 2 Jazzy) with the laptop on board through one room on a saved map; see [Rover navigation](#rover-navigation-ros-2-track). Not part of the sorter loop.
- **Arm:** Seeed reBot Arm B601-RS (6 DoF + parallel gripper, [D-011](decisions.md)), driven through `rebot_b601/` ([D-019](decisions.md)). Joint 1 turns ±145°, so nothing right behind the arm is reachable; with the gripper pointing down the TCP reaches ~100 mm above the deck at most, and the floor from ~140 to ~450 mm out.
- **Camera:** Intel RealSense D435i RGB-D on the wrist, fixed to link5, looking along the gripper ([D-006](decisions.md), [D-027](decisions.md)).
- **Cargo box:** one cardboard box, 190 × 190 mm outside and 75 deep, to the arm's right and a bit behind ([D-045](decisions.md)), its underside 45 mm below the deck; every sock goes into it, not split by color ([D-040](decisions.md)). `sim.layout.cargo.compartments` can still split it along x.
- **On the rover** around the arm: the electronics case behind it and the power supply on the left (keep-out). The arm stands turned on the deck (joint 1 = 0 to the rover's right) and leans ~6° forward ([D-049](decisions.md), [D-050](decisions.md)).
- **Unload station:** 3 laundry bins (cardboard boxes like the cargo box) on the floor in a row in front of the rover, one per color ([D-043](decisions.md)); the rover parks there within a tolerance, the loop finds each bin with the camera.
- The layout is `sim.layout` (the rover's frame at the arm base, mm), measured on the rover except the equipment and the box's position ([D-042](decisions.md)):

| Part | Where |
| --- | --- |
| floor | z = −192 (`floor_z_mm`, fitted to the floor the camera saw; 200 by tape) |
| rover body (chassis + wheels, from above) | 420 × 420: x −300..120, y −210..210; wheels Ø120 (not measured) × 130 at its corners; below the deck (z < 0) |
| deck plate | 300 × 185: x −240..60, y −92.5..92.5, top z = 0; the arm at its front edge |
| equipment (`equipment`) | electronics case x −200..−55, z −75..−5; power supply (and the power strip under it) x −245..10, y 120..270, z −10..60 (photo); the Leo's raised top cover x −300..−50, y ±128, up to z 50 (higher than the box's rim) |
| cargo box, inside | 182 × 182 (190 outside) around (−185, −225), over the right rear wheel (its corner seen by the wrist camera, [D-050](decisions.md)): x −276..−94, y −316..−134, floor z −41 (underside −45), rim z 34, walls 4 mm |
| floor view (what `look_floor` frames) | 280 × 240 centered at (310, 0) |
| floor pick zone (`zones.floor`, computed) | the ring the arm reaches: ~150–490 mm out, from the rover's right side round the front to ~50° left of forward (joint 1's ±145° with the arm turned) |
| cargo pick zone (`zones.cargo`, computed) | the box's inside 16 mm off the walls, corners cut by 45 mm; a pick plans with the fingers at 0, 90, 45 or 135° |
| laundry bins (not measured) | 190 mm square outside, 75 high, walls 4 mm, at (290, 220) light, (290, 0) dark, (290, −220) colored |

## Components

| Component | Owner | Responsibility |
| --- | --- | --- |
| Camera | shared | Capture thread. `latest()` for the live feed, `fresh()` for decisions |
| Calibration | shared | Hand-eye transform. Pixel + depth + camera pose → arm point, and back. The only place where this conversion happens ([D-002](decisions.md)) |
| Arm controller | shared | Named poses, `look` / `pick` / `drop_to_cargo` / `drop_to_laundry`, floor limit and keep-out, hold, recover. One `Controller` on an `ArmDriver`: the real arm or the simulator |
| Observer | shared | `observe(zone)`: move to the look pose, take a fresh frame, attach the camera pose |
| Hub | shared | Status, decision frames, commands and the operator mode between the state machine and the dashboard |
| State machine | shared | Commands, hold, errors, status, run log; runs the loop of the operator mode |
| Load loop | A | `orchestrator/load.py`: socks from the floor into the cargo box |
| Unload loop | B | `orchestrator/unload.py`: the cargo box into the laundry bins |
| Floor detector | A | Socks on the floor: color, grasp point (pixels), grasp angle, mask |
| Box detector | B | Grasp point in the cargo box (pixels), or "empty" |
| Color classifier | A | Block 4's library: SAM3 masks ([D-013](decisions.md)) + color statistics. The floor detector's baseline uses it |
| Dashboard | shared (panels: A, B; Rover tab: N) | Web UI: decision frame, live wrist feed, 3D view, state, counters, controls; the Rover tab drives the nav sim or the real Leo |
| Simulator | base shared, scenes A / B | MuJoCo: the arm's motors, the wrist RGB-D camera, the rover, cloth socks |

## Coordinate frames and units

| Frame | Units | Notes |
| --- | --- | --- |
| Pixel `(u, v)` | px, int | Color stream, origin top-left. Depth is aligned to color |
| Camera | mm | Optical frame (x right, y down, z forward). **Depth values are Z along the optical axis** |
| Camera link | mm | URDF `link5`: joint 6 turns the gripper, not the camera ([D-027](decisions.md)). `ee_pose()` returns `T_base_link5` |
| TCP | mm | URDF `gripper_end`, the fingertips; +x is the approach axis, the fingers open along +y |
| Arm base | mm | Origin at the arm's base on the deck (deck top z = 0), +x the rover's forward, +y left, z up. The floor is at `sim.layout.floor_z_mm` (−192). The arm stands turned on the deck: joint 1 = 0 points `arm.base_yaw_deg` (−90: to the rover's right) from +x, and its base leans `arm.base_tilt_deg` (about x, then y; `sorter.calibration.level` measures it from the floor the camera saw, [D-050](decisions.md)); `rebot_b601.kinematics.set_base` turns FK, IK and every check into this frame (`load_config` sets it, [D-049](decisions.md)) |

- `Pose` = `np.ndarray` 4×4 float64, translation in **mm**. Joint angles in **rad**. Time is `time.monotonic()`.
- Only `sorter.arm.kinematics` and the arm driver convert to `rebot_b601` units (m, rad).
- `T_base_cam = T_base_link5(q at capture) · T_link5_cam` (the hand-eye result).
- **Gripper yaw:** the direction the fingers open along, projected on the floor, as an angle from +x in the arm frame (rad, mod π: the gripper is symmetric). With the gripper pointing down, joint 6 sets it.

## The loops

`sorter.orchestrator.state_machine.StateMachine` is the part both loops share: commands, hold, errors, status, run log. At `START` it takes the loop of the operator mode (`LOAD` → `LoadLoop`, `UNLOAD` → `UnloadLoop`). `STARTING` (arm on, home) and `DONE` (home, idle) are shared; every other phase is a method `_<phase>` of the loop that returns the next phase. A loop keeps its own memory (`reset()` at the start of a run) and uses the shared run state on `sm`: `counters`, `failures`, `obs`, `cycle`, `new_cycle()`, `decide(result, overlay, summary, next_phase)` (publishes the decision frame and writes the run log).

**Load (stage A)** ([D-041](decisions.md)): the arm looks at the floor from the scan poses `scan_1` … `scan_7` (a ring around it) one after another and picks the nearest sock as soon as a view has one. A sock cut off by the frame or far off-center gets a closer look first (`AIM`: the camera aimed at it). After the drop, `CHECK_LOAD` looks at the spot (the sock must be gone) and into the box from `look_cargo` (its surface must have risen where the sock landed, or one more sock is told apart there): only then is it counted. A sock that fails `load.max_attempts` times is left and its spot avoided. A whole round of views without a sock (`load.empty_rounds`) → `DONE`.

```text
STARTING → SCAN(k) → SENSE_FLOOR ─sock─► [AIM] → PICK_FROM_FLOOR → DROP_TO_CARGO → CHECK_LOAD → SCAN(k) …
                         └─no sock─► SCAN(k + 1) … every view empty → DONE
```

**Unload (stage B):** only the wrist camera tells where things are ([D-043](decisions.md)). Once per run the bins are found (a look at each bin's layout place, a second one centered on the estimate if the bin was cut by the image edge). Per sock: the sock on top of the pile, its grasp and finger directions (the open fingers kept off other socks); after the pick a second look into the box (it is also the next cycle's look), the `show_held` pose (is anything held, and its color from how it hangs); the color from the sock seen hanging, else the box's best match ([D-044](decisions.md): the one sock gone, of several the one nearest the grasp, none gone the grasp's target); socks of different colors hanging: everything back into the box, and again; the drop into the found bin via home, then a look into the bin: only a seen drop is counted.

```text
STARTING → LOOK_CARGO (the bins, once) → SENSE_CARGO ─sock─► PICK_FROM_CARGO → DROP_TO_LAUNDRY(color) → LOOK_CARGO …
                                              └─box empty × empty_confirmations─► DONE
```

The unload loop takes the socks out of the one box with the wrist camera only and tells each one's color on the way ([D-043](decisions.md)).

**Shared rules for both loops:**

- Commands are applied **between phases**; every phase is atomic. `STEP` runs one phase, then pauses. `STOP` during a run: the Hub holds the arm at once (the phase under way aborts), then `IDLE` and `arm.recover()`; the run ends even if the arm can't get home ([D-048](decisions.md)).
- `HOLD` is not queued: the Hub calls `arm.hold()` from the web thread; the blocked arm call raises `EStopped` → phase `HELD`. `RESET` → `arm.recover()` → the loop's `first` phase.
- `failures ≥ state_machine.max_consecutive_failures` → `ERROR`, paused. `RESUME` from `ERROR` recovers the arm (like `RESET`) and resets `failures`. Any `SorterError` or unexpected exception in a phase → `ERROR`, paused; the loop thread never dies.
- **Counters go up only for what ended in the right place.** Load counts what landed in the cargo box (by color), unload per bin.

## Contracts

Shared types are in `sorter.core.types`, errors in `sorter.core.errors`, the `Protocol`s in `sorter.core.protocols`. Array fields are read-only; consumers never modify them.

### Errors

```python
class SorterError(Exception): ...


class CameraError(SorterError): ...  # no frame within timeout, device lost


class CalibrationError(SorterError): ...  # no camera pose, invalid depth, file missing


class ArmError(SorterError): ...  # bus fault, motion failed, path hits the floor / keep-out


class TargetRejected(
    ArmError
): ...  # outside the zone workspace, IK or path check failed; NO motion


class EStopped(ArmError): ...  # arm is held; every motion raises this until recover()


class WrongMode(SorterError): ...  # needs another operator mode, or the mode can't change now
```

"Nothing found" is never an exception: vision returns it as a result value.

### Basic types

```python
class Zone(StrEnum):
    FLOOR = "floor"      # in front of the rover: socks to load
    CARGO = "cargo"      # the cargo box on the rover (one box, D-040)
    LAUNDRY = "laundry"  # the station's bins, one per ColorClass (drops only, no look pose)

class ColorClass(StrEnum):
    LIGHT, DARK, COLORED

PixelPoint(u: int, v: int)
ArmPoint(x: float, y: float, z: float)  # mm, arm base frame
```

### Camera

`Intrinsics(fx, fy, cx, cy, width, height, coeffs=())`, `Frame(color: HxWx3 uint8 BGR, depth_mm: HxW uint16 Z in mm aligned to color (0 = no data), intrinsics, timestamp, seq)`.

```python
class Camera(Protocol):
    def start(self) -> None: ...
    def close(self) -> None: ...
    def latest(self) -> Frame | None: ...  # non-blocking; the live feed
    def fresh(self, timeout_s: float = 2.0) -> Frame: ...  # exposure started after the call
```

The real backend is `RealSenseCamera` (`pyrealsense2`, the `camera` extra). The camera knows nothing about the arm.

### Observation and Observer

```python
Observation(frame, zone, T_base_cam: Pose | None, joints: tuple[float, ...] | None)

Observer(camera, arm, calibration).observe(zone, pose=None)  # look pose (or `pose`), fresh frame
Observer.observe_point(zone, target: ArmPoint, heights_mm)  # the camera aimed at target, or None
```

### Vision: shared types

`GraspPoint(px, depth_mm)`: the cloth **surface** at `px` (robust Z, never 0); how deep the gripper goes below it is the arm's business. `Overlay(markers, polygons, mask, text)`, drawn by the dashboard on the frame it came from. Zone ROIs are `views.<zone>.roi` in config.

### Floor detector (stage A)

```python
@dataclass
class Sock:
    color: ColorClass
    confidence: float  # 0..1, of the color
    grasp: GraspPoint
    grasp_angle_rad: float | None  # gripper yaw across the sock at the grasp, image frame
    area_px: int
    touches_roi_edge: bool  # cut off by the frame or by pixels without depth
    mask: np.ndarray | None  # HxW bool
    stats: dict[str, float]


@dataclass
class FloorResult:
    socks: list[Sock]  # nearest the image center first; [] = no sock in view
    overlay: Overlay


class FloorDetector(Protocol):
    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> FloorResult: ...
```

Stateless: failed grasps are passed as `avoid`. `SockDetector(cfg.floor_detector, classifier)` searches the whole frame: the classifier's segmentation (SAM3 / the render) gives the masks; a mask is a sock if its area and length in mm (pixels scaled by the depth) are sock-like; the color is the classifier's; the grasp is the pixel deepest inside the mask (its widest part); the yaw is across the sock there (PCA of the mask within `local_axis_mm` of the grasp). Whether a sock lies on the floor, in reach, is the loop's business (arm coordinates): if the grasp is out of reach, the loop takes the mask's deepest pixel that is in reach.

### Box detector (stage B)

```python
class BoxStatus(StrEnum): GRASP, EMPTY, NO_GRASP
BoxResult(status, grasp: GraspPoint | None, coverage: float, overlay)

class BoxDetector(Protocol):
    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult: ...
```

`DepthBoxDetector(cfg, roi)`: the floor is a high percentile of the ROI depth, cloth is what stands `cloth_height_mm` above it, the grasp is the top of the smoothed height map, `wall_margin_mm` inside the ROI. It sees walls and dividers as cloth, so its ROI must be one compartment.

The unload loop's vision works on arm-frame points (depth + camera pose, `box_detector.geometry.points`), not on image heuristics:

- `cargo.find_sock(obs, box, floor_z, rim_z, workspace, classifier_cfg, segment, avoid) -> CargoView(target: SockTarget | None, cloth_px, seen: list[SockSeen], overlay)`: cloth is what stands over the box floor inside its walls; socks are the segmentation's instances; the target is the top of the pile, grasped at its highest point inside the workspace, with finger directions to try. `taken(before, after)`: the socks of one view missing in a later one from the same pose (matched by color and pixel overlap).
- `station.find_bin(obs, guess, floor_z, size, height, wall) -> BinFit(center, yaw, score, seen, complete, overlay)`: a square ring of the bin's size matched to the wall points on a top-down grid.
- `held.find_held(frame, segment, classifier_cfg) -> HeldView(color | None, …)`: the instance mostly nearer than 330 mm (or without depth) from `show_held`; `show_pose(...)` searches that pose.

### Color classifier (library, used by A)

`ColorClassifier.classify(frame) -> BackgroundResult(items: list[ItemResult], overlay)`; `ItemResult(color, confidence, grasp, area_px, touches_roi_edge, stats)`. `Sam3ColorClassifier(cfg.color_classifier, segment, roi)`: masks from `segment(bgr)` (the SAM3 service, [D-013](decisions.md), or the sim's render), color from Lab statistics of the eroded mask. A failing service raises `SegmentationError` (a `SorterError`).

### Calibration

```python
class Calibration(Protocol):
    def cam_pose(self, ee_pose: Pose) -> Pose: ...  # T_base_link5 → T_base_cam
    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint: ...
    def to_pixel(
        self, obs: Observation, p: ArmPoint
    ) -> PixelPoint | None: ...  # None outside the image
```

- The hand-eye result `T_link5_cam` is in `config/hand_eye.yaml` (committed, [D-007](decisions.md)).
- Two ways to compute it: the **calibration page** (tape marks on the floor view, [D-026](decisions.md)), or `python -m sorter.calibration.hand_eye` (a ChArUco board on the floor under `look_floor`, its top at `calibration.board_z_mm` = −144, [D-028](decisions.md)).

### Arm controller

```python
PickResult(gripper_opening: float, likely_empty: bool)  # likely_empty is a hint; the camera is the truth

class ArmController(Protocol):
    def start(self) -> None: ...      # connect, enable motors, hold the current position
    def shutdown(self) -> None: ...   # rest pose, then disable motors
    def home(self) -> None: ...
    def look(self, zone: Zone) -> None: ...  # look_floor / look_cargo; no-op if already there
    def go_to(self, name: str) -> None: ...  # a named pose (scan_k)
    def aim_camera(self, T_link5_cam, target, heights_mm, tilts_deg=(0, 10, 20, 30)) -> float | None:
        ...  # the camera at `target` from the highest safe height; None and no motion if none
    def pick(self, target: ArmPoint, zone: Zone, yaw_rad: float | None = None) -> PickResult: ...
    def drop_to_cargo(self, color: ColorClass) -> None: ...    # home → high over cargo_<color>, straight down under the rim, settle, open, back out, up, home
    def drop_to_laundry(self, color: ColorClass) -> None: ...  # via home → laundry_<color>, open, home
    def ee_pose(self) -> Pose: ...    # T_base_link5
    def joints(self) -> tuple[float, ...]: ...
    def hold(self) -> None: ...       # thread-safe; freeze; every motion raises EStopped until recover()
    def recover(self) -> None: ...    # leave hold: lift, open over the floor view, home
```

- **Blocking:** every motion returns once the arm is still.
- **`pick(target, zone, yaw_rad)`**, `target` = the cloth surface point: rejected with no motion (`TargetRejected`) if `target` XY is outside `zones.<zone>.workspace_mm` or any of the three moves fails to plan. Then: to `target.z + approach_mm` with the gripper down (turned to `yaw_rad` by joint 6 if given), open, straight down to `max(target.z − grasp_depth_mm, z_floor_mm)`, close, straight up to `lift_z_mm`. The IK above the target is seeded from where the arm is, else elbow up towards the target.
- **Every planned path** is checked against the floor (`arm.z_min_mm`, = floor + 3) and the **keep-out boxes** (`arm.keep_out_mm`: the rover's middle below the deck, all four wheels (also one partly under the cargo box), the equipment behind the arm, the cargo box's walls, grown by `arm.keep_out_margin_mm`). Points are sampled along the links, on the gripper, on its housing (`kinematics.HOUSING_MM`: the finger rail, 184 mm across, and the motor behind it, from rebot_b601's meshes, turning with joint 6; the sim's `palm*` geoms are the same boxes) and on the camera's body (`arm.link5_points_mm`, from the hand-eye mount; also checked against the floor). A joint move (`plan_move`) goes straight if that's clear, else turns joint 1 first or last, else the same way via `home`.
- **Drops** hold still `arm.drop_settle_s` over the drop pose before opening: the hanging sock stops swinging.
- **Into the cargo box** the TCP comes in `arm.cargo_drop_above_mm` over `cargo_<color>` (the hanging sock clears the walls; coming in at the drop height dragged it over the wall), goes straight down to `arm.cargo_drop_depth_mm` below it (under the rim), lets go, backs out along the tool `arm.cargo_drop_back_mm` (the fingers out of the cloth) and goes up. The gripper is tilted outwards by the first of `arm.cargo_drop_tilts_deg` × `arm.cargo_drop_azimuths_deg` (turned about the vertical off straight away from the base) that plans: pointing straight down it gets only ~110 mm over a box this close to the base. Without that path: the plain drop. `sim.layout`'s check plans this path from `home` too.
- **The camera's poses** (`kin.camera_look`): the optical axis through a target from the highest safe height, up to 30° off vertical (the camera is ~100 mm off the gripper's axis: straight down, it can't get high over every spot).
- **Hold vs disable:** disabling the motors makes the arm fall; the software stop is `hold()` ([D-009](decisions.md)).
- **Driver faults** latch until `clear_fault()` ([D-025](decisions.md)); `recover()` clears one. A tracking fault (a joint behind its setpoint; rebot_b601 allows `TRACKING_LAG_S` of its speed) is cleared once per motion and the rest of the path run at half speed ([D-048](decisions.md)). Motor feedback frozen bit for bit (0.5 s moving, 1.5 s still; the adapter stopped receiving while commands still go out) is a fault on the hardware: the arm holds its last setpoint (not the frozen pose it would jerk back to) and the fault can't be cleared: quit, replug the adapter, start again.
- Beyond the protocol, `Controller` has `plan_pick`, `gripper_opening()`, and for the dashboard's setup modes and speed control `held`, `at`, `go_to`, `move_joints`, `move_tcp`, `lift`, `set_gripper`, `release`, `fault`, `clear_fault`, `set_pose`, `speed_scale`, `max_speed_scale`, `set_speed_scale`.

### Hub: state machine ↔ dashboard

```python
class Phase(StrEnum):
    IDLE, STARTING,
    SCAN, SENSE_FLOOR, AIM, PICK_FROM_FLOOR, DROP_TO_CARGO, CHECK_LOAD,  # load
    LOOK_CARGO, SENSE_CARGO, PICK_FROM_CARGO, DROP_TO_LAUNDRY,      # unload
    DONE, HELD, ERROR

class Command(StrEnum): START, PAUSE, RESUME, STEP, STOP, HOLD, RESET

class OperatorMode(StrEnum):
    LOAD, UNLOAD      # RUN_MODES: the state machine runs that loop
    MANUAL, CALIBRATE # the dashboard's setup modes

Status(phase, next_phase, mode: idle|running|paused, run_id, cycle, counters, failures,
       last_cycle_s, error, health, events)
Decision(phase, obs, overlay, summary)
```

- Run commands (all but HOLD) are queued in `LOAD` / `UNLOAD` only (`WrongMode` otherwise); HOLD works in every mode. A run mode is left only while no run is going. The default mode is `LOAD`.
- Modules log with `logging`; `HubLogHandler` turns warnings and errors into `Event`s.
- **3D view data** (`sorter.dashboard.twin.Twin`, `TwinSource`): `layout()` = `parts` (every box / cylinder / plane geom of the MuJoCo world body: floor, rover, cargo box, bins, whatever a scene adds; on the rig the same scene is built from the config), `floor_z_mm`, `zones` (`polygon`, `z_mm`), camera intrinsics, `cloth_n`; `state()` = link poses, gripper, TCP, camera pose, and on the sim every cloth item with its vertices, `location` and `in` (compartment / bin color).

### Dashboard HTTP API

| Endpoint | Content |
| --- | --- |
| `GET /`, `/load`, `/unload`, `/manual`, `/calibrate`, `/3d`, `/rover` | The admin panel (React, `frontend/`, built into `src/sorter/dashboard/web/`) |
| `GET /api/meta` | `{"phase_labels", "modes", "calibrate"}` |
| `GET` / `POST /api/mode` | `{"mode": "load" \| "unload" \| "manual" \| "calibrate", "busy"}`; 409 if a run is going or a manual motion runs |
| `GET /api/status`, `WS /ws` | `Status` as JSON + `now`, `speed`, `operator` |
| `GET` / `POST /api/speed` | the arm's `speed_scale` ([D-030](decisions.md)) |
| `GET /stream/decision.mjpg`, `/stream/live.mjpg`, `/snapshot/*.jpg` | the decision frame (ROI, overlay, caption) and the live wrist feed |
| `POST /api/command` | `{"cmd": "start" \| "pause" \| "resume" \| "step" \| "stop" \| "hold" \| "reset"}`; 409 for all but `hold` outside `load` / `unload` |
| `GET /api/twin/layout`, `/api/twin/state` | `Twin.layout()` / `.state()` |
| `GET` / `POST /api/manual` | manual control: named poses, a tour `look_floor → look_cargo → cargo_* → laundry_* → home`, jog, gripper, save a pose into `rig.yaml`, release, clear fault |
| `GET` / `POST /api/calibrate` | the calibration page: marks on the floor view, clicks, the mount fit, look poses (`look_floor`, `look_cargo`) |

**The front end is updated only in part**: its tabs are Load / Unload (one run page, the tab says which loop it switches to), Manual, Calibrate, 3D view; the run page's phase strip and the 3D view are still the table's (the old layout JSON). The rest (the 3D view from `parts`, the rover phases, the load panel) is task A6; B adds its panel on top (B5). The Rover tab (`/rover`, `RoverPage.tsx`) is stage N's and works already.

### Rover navigation API (stage N)

The Rover tab talks to `sorter.nav.server` (mounted by `create_app(..., nav=)`, also standalone with `python -m sorter.nav serve`). One thread owns the nav sim and its renderer and runs one command at a time in real time.

| Route | What |
| --- | --- |
| `GET /api/nav/commands` | every command with its parameters (name, type, default, required) and doc, from the signatures: the tab's command panel |
| `GET /api/nav/state` | episode, busy, last results, detections, odometry, ground truth (debug), score |
| `GET /api/nav/view/{rgb,depth,chase,overview}.jpg`, `/api/nav/stream/{…}.mjpg` | the OAK-D's RGB (goal zone drawn) and depth; chase and overview are debug views |
| `POST /api/nav/reset` `{scenario, seed, overrides}` | a new episode (with `serve --real`: reconnect the real rover) |
| `POST /api/nav/command` `{name, args}` | one command of `sorter.nav.commands.Rover.COMMANDS` |
| `POST /api/nav/auto` / `detect` `{detector}` | the approach algorithm / detections on the last frame (`classic`, `sam3`, `seg`) |
| `POST /api/nav/run` `{detector, gap_m}` | RUN ROBOT: find the nearest sock, stop the bumper `gap_m` before it (default 0.30; `sam3` on the real rover) |
| `POST /api/nav/box` `{target, stop_m}` | GO TO BOX: drive until the bumper is `stop_m` from AprilTag `target` (`nav.boxes`) |
| `POST /api/nav/boxes/remember` | save the AprilTag boxes in view and their layout (`nav.boxes.memory`) |
| `POST /api/nav/stop` | stops the running command or algorithm |

## Threads and process

One Python process ([D-005](decisions.md)): camera capture thread (on the sim, its render thread); the arm's 50 Hz control loop inside `rebot_b601.arm.Arm` (on the sim, ticked by the physics thread on simulated time); the physics thread (sim only); the state machine thread; the web server (uvicorn).

`python -m sorter run [--sim] [--mode load|unload|manual|calibrate]` wires everything (`sorter/app.py`), starting in `--mode` (default `load`). Ctrl+C → `arm.hold()`, stop the loop, `arm.shutdown()`.

## Wiring

`sorter.app.build_system(cfg, sim=False) -> System` creates every component per `backends` (`camera`, `arm`, `calibration`, `box_detector`, `floor_detector`: `real` | `sim`). `sim=True` makes the camera and the arm sim (MuJoCo); the rest run their real code on the rendered frames, except that calibration uses the sim's exact camera mount (= the real hand-eye result, see Simulator) and the floor detector the render's segmentation (unless `sim.use_sam3`). `System` holds `cfg`, `camera`, `arm`, `calibration`, `box_detector`, `floor_detector`, `observer`, `hub`, `world` (the `PhysicsWorld` or `None`).

Real backends: `sorter.<package>.backend.create(cfg)`. Package `__init__.py` files stay empty. Import driver SDKs inside `backend.create`.

## Config

YAML, deep-merged: `config/default.yaml` → `config/rig.yaml` → `config/hand_eye.yaml` → `config/local.yaml` (gitignored). Validated by `sorter.core.config.load_config()`; unknown keys are an error. Each package defines its section's model in `src/sorter/<package>/config.py`.

| Key | Owner | Content |
| --- | --- | --- |
| `backends` | shared | `real` \| `sim` per component |
| `sim` | shared; `load`: A, `unload`: B | `realtime` (0 = as fast as possible), `seed`, `scenes` (which scene files add to the base), `load` (`socks`, `area` reach / zone (`zone_mm`: rig.yaml's floor zone, what `watch` and `bench` use) / view, `reach_mm`, `reach_deg`, `margin_mm`, `sock_mm`, `bunched_prob`), `unload` (`cargo`: socks per color, in the box), `miss_prob`, `use_sam3`, `board`, `marks`, image size, `focal_px`, `camera_mount_mm` (the nominal mount), `camera_T_link5_cam` (default: the hand-eye result), `layout` (`floor_z_mm`, `body`, `wheel_radius_mm`, `wheel_width_mm`, `deck`, `equipment`, `cargo`, `floor_view`, `laundry`) |
| `camera` | shared | the D435i's settings; on the rig a fixed `white_balance_k` and `color_gains` (B, G, R on every frame) for the room's light (`python -m sorter.camera.wb`, [D-050](decisions.md)) |
| `views.<zone>.roi` | layout tool | pixel polygon of the zone from its look pose (`rig.yaml`) |
| `calibration` | shared | `hand_eye`, the board tool's `poses`, `tilt_deg`, `shift_mm`, `board_z_mm`; `marks_z_mm` (the floor) |
| `box_detector` | B | depth thresholds and margins |
| `color_classifier` | A | the SAM3 service (`sam`, API key in `local.yaml` or `SAM3_API_KEY`) and the color thresholds |
| `floor_detector` | A | `min_area_mm2`, `max_area_mm2`, `max_length_mm`, `edge_px`, `avoid_radius_px`, `local_axis_mm` |
| `load` | A | the load loop: `max_attempts`, `empty_rounds`, `aim_off_center`, `aim_heights_mm`, `same_sock_mm`, `max_sock_height_mm`, `raised_mm`, `min_raised_mm2`, `cargo_margin_mm` |
| `arm` | shared | `base_yaw_deg`, `base_tilt_deg` (the arm turned and leaning on the rover, [D-049](decisions.md), [D-050](decisions.md)), `speed_scale` (at start; `POST /api/speed` changes it), `max_speed_scale` (at most `speed_ceiling()` ≈ 1.43, the motors' velocity limit, [D-033](decisions.md)), `approach`, `safe_z_mm`, `z_min_mm`, `drop_height_mm`, `drop_settle_s`, `cargo_drop_above_mm` / `_tilts_deg` / `_azimuths_deg` / `_back_mm` / `_depth_mm`, `keep_out_mm`, `keep_out_margin_mm`, `link5_points_mm`, gripper |
| `poses` | layout tool | `rest`, `home`, `look_floor`, `look_cargo`, `scan_1` … `scan_7`, `cargo_<color>`, `laundry_<color>`, `show_held` (`rig.yaml`) |
| `zones.<zone>` | layout tool | `floor`, `cargo`: `workspace_mm`, `z_floor_mm`, `grasp_depth_mm`, `approach_mm`, `lift_z_mm` (`rig.yaml`) |
| `state_machine` | shared | `empty_confirmations`, `max_consecutive_failures`, `low_confidence`, `save_runs`, `runs_dir` |
| `dashboard` | shared | `host`, `port`, `stream_fps`, `jpeg_quality`, `status_hz` |
| `nav` | N | the navigation world ([D-046](decisions.md)): `leo` (the rover and its firmware), `camera` (the OAK-D, its mount and depth mode), `goal` (where the sock must end up), `real` (`rosbridge_url`, topics, speed caps, `stop_lead_s`, the OAK-D device), `boxes` (AprilTag size, target id, stop distance, memory file), `timestep_s`, `control_hz`, `realtime`, `stream_fps`, `max_command_s`, `detector`, `sam_prompt` |

`rig.yaml` is computed: `uv run python -m sorter.sim.layout --write` after any change to `sim.layout`, `arm.drop_height_mm` or the hand-eye result (it prints the poses and checks every pick and move; 0 problems or it exits 1). The rig's `arm.z_min_mm`, `arm.keep_out_mm` and `arm.link5_points_mm` come from there too. The floor zone is mapped: a pick at one of 4 yaws at least (`FLOOR_PICK_YAWS`; the load loop turns the grasp off the sock's yaw where that one doesn't plan, [D-051](decisions.md)) on a 30 mm grid, the reached region as a polygon, cells taken out where a pick on a 20 mm grid inside it fails (minutes the first time; cached in `data/cache/`). The scan poses aim the camera along that ring, from the middle outwards, each seeded by its neighbour, each reachable from `home`.

## Recording format

`sorter.core.io.save_observation` / `load_observation`: one `.npz` per observation (+ `.png`). Run logs (`sorter.orchestrator.runlog`): `data/runs/<run_id>/` (the run id ends with the mode), one record per `decide()`: the observation and a `.json` with the result, summary, counters and `next_phase`; `run.json` with the config and the end. The session recorder (`sorter.orchestrator.recorder`, `run --record`, on by default on the rig) writes `data/sessions/<stamp>/`: `sorter.log` (DEBUG), `telemetry.csv` (the arm at 50 Hz and the phase), `events.jsonl` (status changes, arm / detector / observer calls with durations, decisions, operator marks), `decisions/*.jpg`, `wrist_NNN.mp4` + `wrist_frames.csv`; it wraps the components' methods on the instance and reads `rebot_b601.Arm`'s measurement, and changes nothing the system does. `data/` is gitignored.

## Simulator

`--sim` simulates the **hardware only** ([D-021](decisions.md)), in MuJoCo (`sorter.sim.physics`); there is no other engine.

- **Scene** (`sorter.sim.physics.model.build(cfg.sim) -> Scene(xml, items)`): the **base** (floor plane with herringbone parquet, the rover's wheels, rails and battery, the deck plate, the equipment (the power supply on the left), the cardboard cargo box, the arm from its URDF with the wrist camera, its base turned and tilted like the rig's; the calibration board and tape marks when asked), then each scene in `sim.scenes` in order: `sorter.sim.scenes.<name>.scene.add(world, asset, cfg, rng) -> list[ItemSpec]` may add geoms and assets and returns the cloth items to add. `load` (A) scatters sock-shaped `sim.load.socks` over the floor the arm reaches (or the floor view); `unload` (B) adds the laundry bins, moved off their layout places by `sim.unload.station_mm` / `station_deg` (the whole row: parking) and `bin_mm` / `bin_deg` (each bin), by seed, and `sim.unload.cargo` socks piled in the box, each a limp `sim.unload.sock_mm` sheet (`sock_young` / `sock_thickness_mm`, [D-044](decisions.md)). Helpers for scenes: `box`, `tray`, `palette_rgb`, `ItemSpec(color, rgb, pos, yaw, sheet_m, gather, young, thickness_m, fold_m, rest_m)`.
- **Cloth:** 8 × 8 flex grids with a crumpled rest shape, or a scene's own (`rest_m`: the load scene's sock outline); they don't collide with each other (cost). Cloth is expensive: 3 socks run at ~2.6× real time, 6 at ~1.3×.
- **Grip:** when a close stalls the fingers on cloth touching both pads, the vertices between them are attached to the gripper until an open (a stand-in for friction). `sim.miss_prob` makes a close catch nothing.
- `PhysicsWorld`: `step`, `teleport_arm`, `joints`, `tcp`, `looking_at`, `vertices(item)`, `location(item)` → `gripper` / `cargo` + color (None: one box) / `laundry` + color / `floor` / `other`, `at(location, color=None)`.
- The arm: `rebot_b601.arm.Arm` runs unchanged on `MujocoBackend`; `PhysicsCamera` renders the D435i (depth noise, no depth under 175 mm) and `segment()` gives MuJoCo's segmentation as SAM3-style instances.
- **The camera sits where the real one was calibrated:** `sim.camera_T_link5_cam`, filled by the config loader from `config/hand_eye.yaml` ([D-041](decisions.md)); the nominal `camera_mount_mm` only without a hand-eye result. So the look poses, ROIs and pixel → arm math the sim runs are the rig's.
- **The load benchmark:** `uv run python -m sorter.sim.scenes.load.bench -n 50 [--socks 1-4] [--workers 3]`: seeded scenes through the real load loop, headless, `realtime: 0`; `data/bench/<run_id>/` gets `scenes.jsonl` and `summary.json` (socks in the box / on the floor / elsewhere, counted vs in the box, sim time per sock).
- `sorter.sim.layout` computes the rig (poses, zones, views, keep-out) with the arm's IK and checks it, B's too: the cargo pick zone and `show_held` ([D-043](decisions.md)); `sorter.sim.rig` has the sim camera mount.
- **The unload benchmark:** `uv run python -m sorter.sim.scenes.unload.bench -n 20`: seeded scenarios through the real unload loop, the station off its place, judged by the simulator's ground truth; report in `data/bench/unload-<run_id>/`.

## Navigation world (stage N)

`sorter.nav` is a separate MuJoCo world, not the arm scene ([D-046](decisions.md)): the Leo Rover 1.9 (`model.py`, meshes from `leo_description` in `assets/leo/`), its firmware (`sim.py`: `cmd_vel` → wheel speeds, timeout, odometry from encoders + gyro), the OAK-D (`camera.py`: RGB + aligned stereo depth), scenarios (`scenario.py`), commands (`commands.py`), detectors (`detect.py`), the approach algorithm (`controller.py`), episodes and scoring (`episode.py`). The rover frame has its origin on the floor under the rover's center, x forward, y left, z up. Operators and the algorithm decide from the camera only; ground truth is used for scoring and the debug views.

The commands (`sorter.nav.commands.Rover`) need a base with `set_cmd(v, w)`, `tick()`, `odom`, `t`, `dt`, `moving()`, `ref`, `cfg` and a camera with `capture() → Frame`, `K`, `T_rover_cam`: `RoverSim` + `OakD` in the sim, `real_leo.LeoBase` (rosbridge: `/cmd_vel` out every tick, `/merged_odom` in) + `real_oakd.RealOakD` (depthai v3) on the hardware. A `Frame` is RGB uint8, depth uint16 mm aligned to it (0 = none), the RGB intrinsics and the camera's pose in the rover frame.

## Full mission (stage C)

`sorter.mission` ([D-052](decisions.md)): `run_mission()` / `python -m sorter.mission` alternates the nav world and the arm world. Drive (`nav.controller.Approach`, nav goal zone) → hand over the socks in the arm's floor zone (Leo frame → arm base frame: `+ sim.layout.body.center_mm`; color class from the sock's RGB) as `sim.load.placed` → a fresh arm world runs the load loop (`StateMachine`, mode load) to DONE → socks in the cargo box leave the nav world → repeat until `capacity` or no sock → drive home on the odometry → `approach_box` on the station's tag 13. Report: `mission.json` (socks, loaded, cargo by class, stops, station gap).

## Rover navigation (ROS 2 track)

`ros2_ws/src/rover_nav` ([D-037](decisions.md), [D-038](decisions.md)). Ready-made nodes: RTAB-Map for SLAM, Nav2 for driving, a Nav2 keepout filter for the forbidden half of the room. Two launches: **mapping** (RTAB-Map mapping, keyboard teleop, done once) and **navigation** (RTAB-Map localization on the saved database, Nav2, keepout filter).

| Interface | Direction | Notes |
| --- | --- | --- |
| `/rover_nav/oak/rgb/image_rect`, `/rover_nav/oak/stereo/image_raw`, `/rover_nav/oak/rgb/camera_info` | in | `nav_camera:=oak` (default): OAK-D on the rover's front, DepthAI driver started by `rover_nav` in `/rover_nav`; TF `leo/base_link` → `oak` → `oak_rgb_camera_optical_frame` from the driver's description |
| `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `.../color/camera_info` | in | `nav_camera:=wrist`: the RealSense driver as started by `cloth_task` (640×480, 15 fps, TF off); reused, never started twice |
| TF `leo/base_link` → arm `base_link` → … → `camera_color_optical_frame` | in | Rover → arm base: static, measured mount, published by `rover_nav` (always). Arm → camera: the arm stack's `robot_state_publisher` with the arm in the `drive` pose, or a static transform in the drive pose while the arm stack isn't available |
| `/leo/merged_odom` (`nav_msgs/Odometry`, 100 Hz) + `leo/odom` → `leo/base_footprint` TF | in | The rover's `odom_filter` (LeoOS, wheel odometry + IMU), over the rover's Wi-Fi |
| `/leo/cmd_vel` (`geometry_msgs/Twist`) | out | To the rover firmware. Nav2, or `teleop_twist_keyboard` while mapping |
| `/map` + `map` → `leo/odom` TF | internal | RTAB-Map |
| Keepout mask (`.pgm` + `.yaml`) | internal | Generated from the saved 2D map and a dividing line by `rover_nav`'s mask tool |

- **Names:** the arm owns the plain names (`base_link`, `/joint_states`, `/robot_description`). The rover runs with LeoOS's `ROBOT_NAMESPACE=leo`: frames `leo/…`, topics `/leo/…` (`rover_nav/scripts/setup_rover.sh`). TF tree: `map` → `leo/odom` → `leo/base_footprint` → `leo/base_link` → `base_link` (arm) → … → camera.
- With `nav_camera:=wrist`, the arm stays in `drive` while the rover moves: moving it breaks mapping and localization. With `oak` the arm is free.
- **Sim** (`rover_nav/sim`, [D-039](decisions.md)): a standalone MuJoCo Leo Rover from the official `leo_description` (uv project, no ROS): `cmd_vel` with the firmware's timeout, wheel + gyro odometry, the rover and OAK-D cameras. Not wired to ROS yet; the jevomir VLM drives it through its scoring API.

## Repo layout

| Path | Owner |
| --- | --- |
| `src/sorter/core/`, `app.py`, `__main__.py` | shared |
| `src/sorter/arm/`, `rebot_b601/` | shared |
| `src/sorter/camera/`, `src/sorter/calibration/` | shared |
| `src/sorter/sim/config.py`, `rig.py`, `layout.py`, `physics/` (the base scene) | shared |
| `src/sorter/sim/scenes/load/`, `config/default.yaml` → `sim.load`, `sim.layout.floor_view` | A |
| `src/sorter/sim/scenes/unload/`, `config/default.yaml` → `sim.unload`, `sim.layout.laundry` | B |
| `src/sorter/orchestrator/state_machine.py`, `runlog.py`, `recorder.py` | shared |
| `src/sorter/orchestrator/load.py`, `src/sorter/floor_detector/`, `src/sorter/color_classifier/` | A |
| `src/sorter/orchestrator/unload.py`, `src/sorter/box_detector/` | B |
| `src/sorter/dashboard/`, `frontend/` (the shared parts: A6) | shared; the load panel A, the unload panel B |
| `config/rig.yaml` | the layout tool (and the setup pages on the rig) |
| `config/hand_eye.yaml` | calibration |
| `tests/<package>/` | same as the package; `tests/conftest.py` shared |
| `src/sorter/nav/`, `tests/nav/`, `frontend/src/pages/RoverPage.tsx`, `config/default.yaml` → `nav` | N |
| `src/sorter/mission/`, `tests/mission/` | C |
| `ros2_ws/` | the ROS 2 track ([D-014](decisions.md)), outside these stages |
| `ros2_ws/src/rover_nav/` (incl. `sim/`, the MuJoCo Leo Rover) | the ROS 2 rover navigation track ([D-037](decisions.md), [D-039](decisions.md)); brief: [rover/ros2-navigation.md](rover/ros2-navigation.md) |

"Shared" means: change it only through the contract rules in [AGENTS.md](../AGENTS.md).

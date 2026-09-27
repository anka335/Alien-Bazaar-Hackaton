# Architecture

Single source of truth for the contracts between the stages. The shared types are in `src/sorter/core/`; if code and this file diverge, fix both in the same PR (see `AGENTS.md`).

## Physical setup

- **Rover** ([D-032](decisions.md)): built and driven by others. The arm is bolted to its deck plate, **200 mm above the floor**. For our code the rover stands still: in load mode it has stopped next to socks, in unload mode it is parked at the station. There is no rover interface yet.
- **Arm:** Seeed reBot Arm B601-RS (6 DoF + parallel gripper, [D-011](decisions.md)), driven through `rebot_b601/` ([D-019](decisions.md)). Joint 1 turns ±145°, so nothing right behind the arm is reachable; with the gripper pointing down the TCP reaches ~100 mm above the deck at most, and the floor from ~140 to ~450 mm out.
- **Camera:** Intel RealSense D435i RGB-D on the wrist, fixed to link5, looking along the gripper ([D-006](decisions.md), [D-027](decisions.md)).
- **Cargo box** on the deck, to the arm's left: 3 compartments in a row along x (light, dark, colored).
- **Unload station:** 3 laundry bins on the floor to the rover's right, one per color.
- The layout is `sim.layout` (arm base frame, mm), placeholders until the real rover is measured ([D-034](decisions.md)):

| Part | Where |
| --- | --- |
| floor | z = −200 (`floor_z_mm`) |
| rover body (chassis + wheels, from above) | x −400..100, y −160..320, below the deck (z < 0) |
| cargo box, inside | x −260..60, y 130..310, floor z 5, walls 50 high, dividers 6 mm; compartments ~103 × 180 |
| floor view (what `look_floor` frames; pick zone) | 280 × 240 centered at (300, 0) |
| laundry bins | 220 mm square, 150 high, at (−140, −330) light, (100, −330) dark, (340, −300) colored |

## Components

| Component | Owner | Responsibility |
| --- | --- | --- |
| Camera | shared | Capture thread. `latest()` for the live feed, `fresh()` for decisions |
| Calibration | shared | Hand-eye transform. Pixel + depth + camera pose → arm point, and back. The only place where this conversion happens ([D-002](decisions.md)) |
| Arm controller | shared | Named poses, `look` / `pick` / `drop_to_cargo` / `drop_to_laundry`, floor limit and keep-out, hold, recover. One `Controller` on an `ArmDriver`: the real arm or the simulator |
| Observer | shared | `observe(zone)`: move to the look pose, take a fresh frame, attach the camera pose |
| Hub | shared | Status, decision frames, commands and the operator mode between the state machine and the dashboard |
| State machine | shared | Commands, hold, errors, status, run log; runs the loop of the operator mode |
| Load loop | A | `orchestrator/load.py`: socks from the floor into their compartments |
| Unload loop | B | `orchestrator/unload.py`: every compartment into its laundry bin |
| Floor detector | A | Socks on the floor: color, grasp point (pixels), grasp angle, mask |
| Box detector | B | Grasp point in a cargo compartment (pixels), or "empty" |
| Color classifier | A | Block 4's library: SAM3 masks ([D-013](decisions.md)) + color statistics. The floor detector's baseline uses it |
| Dashboard | shared (panels: A, B) | Web UI: decision frame, live wrist feed, 3D view, state, counters, controls |
| Simulator | base shared, scenes A / B | MuJoCo: the arm's motors, the wrist RGB-D camera, the rover, cloth socks |

## Coordinate frames and units

| Frame | Units | Notes |
| --- | --- | --- |
| Pixel `(u, v)` | px, int | Color stream, origin top-left. Depth is aligned to color |
| Camera | mm | Optical frame (x right, y down, z forward). **Depth values are Z along the optical axis** |
| Camera link | mm | URDF `link5`: joint 6 turns the gripper, not the camera ([D-027](decisions.md)). `ee_pose()` returns `T_base_link5` |
| TCP | mm | URDF `gripper_end`, the fingertips; +x is the approach axis, the fingers open along +y |
| Arm base | mm | Origin at the arm's base on the deck (deck top z = 0), +x forward, +y left, z up. The floor is at `sim.layout.floor_z_mm` (−200) |

- `Pose` = `np.ndarray` 4×4 float64, translation in **mm**. Joint angles in **rad**. Time is `time.monotonic()`.
- Only `sorter.arm.kinematics` and the arm driver convert to `rebot_b601` units (m, rad).
- `T_base_cam = T_base_link5(q at capture) · T_link5_cam` (the hand-eye result).
- **Gripper yaw:** the direction the fingers open along, projected on the floor, as an angle from +x in the arm frame (rad, mod π: the gripper is symmetric). With the gripper pointing down, joint 6 sets it.

## The loops

`sorter.orchestrator.state_machine.StateMachine` is the part both loops share: commands, hold, errors, status, run log. At `START` it takes the loop of the operator mode (`LOAD` → `LoadLoop`, `UNLOAD` → `UnloadLoop`). `STARTING` (arm on, home) and `DONE` (home, idle) are shared; every other phase is a method `_<phase>` of the loop that returns the next phase. A loop keeps its own memory (`reset()` at the start of a run) and uses the shared run state on `sm`: `counters`, `failures`, `obs`, `cycle`, `new_cycle()`, `decide(result, overlay, summary, next_phase)` (publishes the decision frame and writes the run log).

**Load (stage A), baseline:** observation-driven ([D-008](decisions.md)): a sock is counted once the next look shows one fewer.

```text
STARTING → SCAN → SENSE_FLOOR ─sock─► PICK_FROM_FLOOR → DROP_TO_CARGO(color) → SCAN …
                      └─no sock × empty_confirmations─► DONE
```

**Unload (stage B), baseline:** the depth box detector on each compartment's image area (the compartment rectangle projected from `look_cargo`), the first grasp found; the sock goes to that compartment's bin, fingers along the compartment's long side (yaw π/2).

```text
STARTING → LOOK_CARGO → SENSE_CARGO ─grasp in c─► PICK_FROM_CARGO → DROP_TO_LAUNDRY(c) → LOOK_CARGO …
                             └─all empty × empty_confirmations─► DONE
```

The baselines are starting points, not the goal: stages A and B replace them.

**Shared rules for both loops:**

- Commands are applied **between phases**; every phase is atomic. `STEP` runs one phase, then pauses. `STOP` → `arm.home()`, `IDLE`.
- `HOLD` is not queued: the Hub calls `arm.hold()` from the web thread; the blocked arm call raises `EStopped` → phase `HELD`. `RESET` → `arm.recover()` → the loop's `first` phase.
- `failures ≥ state_machine.max_consecutive_failures` → `ERROR`, paused. `RESUME` resets `failures`. Any `SorterError` or unexpected exception in a phase → `ERROR`, paused; the loop thread never dies.
- **Counters go up only for what ended in the right place.** Load counts per compartment, unload per bin.

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
    CARGO = "cargo"      # the cargo box, one compartment per ColorClass
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

Observer(camera, arm, calibration).observe(zone)  # arm.look(zone), camera.fresh(), cam pose
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
    grasp_angle_rad: float | None  # gripper yaw across the sock, image frame; None = any
    area_px: int
    touches_roi_edge: bool  # partly outside the view
    mask: np.ndarray | None  # HxW bool
    stats: dict[str, float]


@dataclass
class FloorResult:
    socks: list[Sock]  # best target first; [] = no sock in view
    overlay: Overlay


class FloorDetector(Protocol):
    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> FloorResult: ...
```

Stateless: failed grasps are passed as `avoid`. The baseline `ClassifierFloorDetector` wraps the color classifier over `views.floor.roi` (no grasp angle, no mask).

### Box detector (stage B)

```python
class BoxStatus(StrEnum): GRASP, EMPTY, NO_GRASP
BoxResult(status, grasp: GraspPoint | None, coverage: float, overlay)

class BoxDetector(Protocol):
    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult: ...
```

`DepthBoxDetector(cfg, roi)`: the floor is a high percentile of the ROI depth, cloth is what stands `cloth_height_mm` above it, the grasp is the top of the smoothed height map, `wall_margin_mm` inside the ROI. It sees walls and dividers as cloth, so its ROI must be one compartment.

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
- Two ways to compute it: the **calibration page** (tape marks on the floor view, [D-026](decisions.md)), or `python -m sorter.calibration.hand_eye` (a ChArUco board on the floor under `look_floor`, its top at `calibration.board_z_mm` = −184, [D-028](decisions.md)).

### Arm controller

```python
PickResult(gripper_opening: float, likely_empty: bool)  # likely_empty is a hint; the camera is the truth

class ArmController(Protocol):
    def start(self) -> None: ...      # connect, enable motors, hold the current position
    def shutdown(self) -> None: ...   # rest pose, then disable motors
    def home(self) -> None: ...
    def look(self, zone: Zone) -> None: ...  # look_floor / look_cargo; no-op if already there
    def pick(self, target: ArmPoint, zone: Zone, yaw_rad: float | None = None) -> PickResult: ...
    def drop_to_cargo(self, color: ColorClass) -> None: ...    # via home → cargo_<color>, open, home
    def drop_to_laundry(self, color: ColorClass) -> None: ...  # via home → laundry_<color>, open, home
    def ee_pose(self) -> Pose: ...    # T_base_link5
    def joints(self) -> tuple[float, ...]: ...
    def hold(self) -> None: ...       # thread-safe; freeze; every motion raises EStopped until recover()
    def recover(self) -> None: ...    # leave hold: lift, open over the floor view, home
```

- **Blocking:** every motion returns once the arm is still.
- **`pick(target, zone, yaw_rad)`**, `target` = the cloth surface point: rejected with no motion (`TargetRejected`) if `target` XY is outside `zones.<zone>.workspace_mm` or any of the three moves fails to plan. Then: to `target.z + approach_mm` with the gripper down (turned to `yaw_rad` by joint 6 if given), open, straight down to `max(target.z − grasp_depth_mm, z_floor_mm)`, close, straight up to `lift_z_mm`.
- **Every planned path** is checked against the floor (`arm.z_min_mm`, = floor + 3) and the **keep-out boxes** (`arm.keep_out_mm`: the rover body below the deck, the cargo box's walls and dividers, grown by `arm.keep_out_margin_mm`). Points are sampled along the links and on the gripper. A move to a named pose that would hit something goes via `home`.
- **Hold vs disable:** disabling the motors makes the arm fall; the software stop is `hold()` ([D-009](decisions.md)).
- **Driver faults** latch until `clear_fault()` ([D-025](decisions.md)).
- Beyond the protocol, `Controller` has `plan_pick`, `gripper_opening()`, and for the dashboard's setup modes and speed control `held`, `at`, `go_to`, `move_joints`, `move_tcp`, `lift`, `set_gripper`, `release`, `fault`, `clear_fault`, `set_pose`, `speed_scale`, `max_speed_scale`, `set_speed_scale`.

### Hub: state machine ↔ dashboard

```python
class Phase(StrEnum):
    IDLE, STARTING,
    SCAN, SENSE_FLOOR, PICK_FROM_FLOOR, DROP_TO_CARGO,              # load
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
| `GET /`, `/load`, `/unload`, `/manual`, `/calibrate`, `/3d` | The admin panel (React, `frontend/`, built into `src/sorter/dashboard/web/`) |
| `GET /api/meta` | `{"phase_labels", "modes", "calibrate"}` |
| `GET` / `POST /api/mode` | `{"mode": "load" \| "unload" \| "manual" \| "calibrate", "busy"}`; 409 if a run is going or a manual motion runs |
| `GET /api/status`, `WS /ws` | `Status` as JSON + `now`, `speed`, `operator` |
| `GET` / `POST /api/speed` | the arm's `speed_scale` ([D-030](decisions.md)) |
| `GET /stream/decision.mjpg`, `/stream/live.mjpg`, `/snapshot/*.jpg` | the decision frame (ROI, overlay, caption) and the live wrist feed |
| `POST /api/command` | `{"cmd": "start" \| "pause" \| "resume" \| "step" \| "stop" \| "hold" \| "reset"}`; 409 for all but `hold` outside `load` / `unload` |
| `GET /api/twin/layout`, `/api/twin/state` | `Twin.layout()` / `.state()` |
| `GET` / `POST /api/manual` | manual control: named poses, a tour `look_floor → look_cargo → cargo_* → laundry_* → home`, jog, gripper, save a pose into `rig.yaml`, release, clear fault |
| `GET` / `POST /api/calibrate` | the calibration page: marks on the floor view, clicks, the mount fit, look poses (`look_floor`, `look_cargo`) |

**The front end is not updated yet**: it still has an "Auto" tab and draws the table from the old layout JSON. Updating it (tabs Load / Unload, the 3D view from `parts`) is task A6; B adds its panel on top (B5).

## Threads and process

One Python process ([D-005](decisions.md)): camera capture thread (on the sim, its render thread); the arm's 50 Hz control loop inside `rebot_b601.arm.Arm` (on the sim, ticked by the physics thread on simulated time); the physics thread (sim only); the state machine thread; the web server (uvicorn).

`python -m sorter run [--sim] [--mode load|unload|manual|calibrate]` wires everything (`sorter/app.py`), starting in `--mode` (default `load`). Ctrl+C → `arm.hold()`, stop the loop, `arm.shutdown()`.

## Wiring

`sorter.app.build_system(cfg, sim=False) -> System` creates every component per `backends` (`camera`, `arm`, `calibration`, `box_detector`, `floor_detector`: `real` | `sim`). `sim=True` makes the camera and the arm sim (MuJoCo); the rest run their real code on the rendered frames, except that calibration uses the sim's exact camera mount and the floor detector the render's segmentation (unless `sim.use_sam3`). `System` holds `cfg`, `camera`, `arm`, `calibration`, `box_detector`, `floor_detector`, `observer`, `hub`, `world` (the `PhysicsWorld` or `None`).

Real backends: `sorter.<package>.backend.create(cfg)`. Package `__init__.py` files stay empty. Import driver SDKs inside `backend.create`.

## Config

YAML, deep-merged: `config/default.yaml` → `config/rig.yaml` → `config/hand_eye.yaml` → `config/local.yaml` (gitignored). Validated by `sorter.core.config.load_config()`; unknown keys are an error. Each package defines its section's model in `src/sorter/<package>/config.py`.

| Key | Owner | Content |
| --- | --- | --- |
| `backends` | shared | `real` \| `sim` per component |
| `sim` | shared; `load`: A, `unload`: B | `realtime` (0 = as fast as possible), `seed`, `scenes` (which scene files add to the base), `load` (`socks`, `margin_mm`), `unload` (`cargo`: socks per compartment), `miss_prob`, `use_sam3`, `board`, `marks`, image size, `focal_px`, `camera_mount_mm`, `layout` (`floor_z_mm`, `body`, `deck`, `cargo`, `floor_view`, `laundry`) |
| `camera` | shared | the D435i's settings |
| `views.<zone>.roi` | layout tool | pixel polygon of the zone from its look pose (`rig.yaml`) |
| `calibration` | shared | `hand_eye`, the board tool's `poses`, `tilt_deg`, `shift_mm`, `board_z_mm`; `marks_z_mm` (the floor) |
| `box_detector` | B | depth thresholds and margins |
| `color_classifier` | A | the SAM3 service (`sam`, API key in `local.yaml` or `SAM3_API_KEY`) and the color thresholds |
| `arm` | shared | `speed_scale` (at start; `POST /api/speed` changes it), `max_speed_scale` (at most `speed_ceiling()` ≈ 1.43, the motors' velocity limit, [D-033](decisions.md)), `approach`, `safe_z_mm`, `z_min_mm`, `drop_height_mm`, `keep_out_mm`, `keep_out_margin_mm`, gripper |
| `poses` | layout tool | `rest`, `home`, `look_floor`, `look_cargo`, `cargo_<color>`, `laundry_<color>` (`rig.yaml`) |
| `zones.<zone>` | layout tool | `floor`, `cargo`: `workspace_mm`, `z_floor_mm`, `grasp_depth_mm`, `approach_mm`, `lift_z_mm` (`rig.yaml`) |
| `state_machine` | shared | `empty_confirmations`, `max_consecutive_failures`, `low_confidence`, `save_runs`, `runs_dir` |
| `dashboard` | shared | `host`, `port`, `stream_fps`, `jpeg_quality`, `status_hz` |

`rig.yaml` is computed: `uv run python -m sorter.sim.layout --write` after any change to `sim.layout` or `arm.drop_height_mm` (it prints the poses and checks every pick and move; 0 problems or it exits 1). The rig's `arm.z_min_mm` and `arm.keep_out_mm` come from there too.

## Recording format

`sorter.core.io.save_observation` / `load_observation`: one `.npz` per observation (+ `.png`). Run logs (`sorter.orchestrator.runlog`): `data/runs/<run_id>/` (the run id ends with the mode), one record per `decide()`: the observation and a `.json` with the result, summary, counters and `next_phase`; `run.json` with the config and the end. `data/` is gitignored.

## Simulator

`--sim` simulates the **hardware only** ([D-021](decisions.md)), in MuJoCo (`sorter.sim.physics`); there is no other engine.

- **Scene** (`sorter.sim.physics.model.build(cfg.sim) -> Scene(xml, items)`): the **base** (floor plane, rover chassis and wheels, deck plate, cargo box with dividers, the arm from its URDF with the wrist camera; the calibration board and tape marks when asked), then each scene in `sim.scenes` in order: `sorter.sim.scenes.<name>.scene.add(world, asset, cfg, rng) -> list[ItemSpec]` may add geoms and assets and returns the cloth items to add. `load` (A) scatters `sim.load.socks` over the floor view; `unload` (B) adds the laundry bins and `sim.unload.cargo` socks per compartment. Helpers for scenes: `box`, `tray`, `palette_rgb`, `ItemSpec(color, rgb, pos, yaw, sheet_m, gather)`.
- **Cloth:** 8 × 8 flex grids with a crumpled rest shape; they don't collide with each other (cost). Cloth is expensive: 3 socks run at ~2.6× real time, 6 at ~1.3×.
- **Grip:** when a close stalls the fingers on cloth touching both pads, the vertices between them are attached to the gripper until an open (a stand-in for friction). `sim.miss_prob` makes a close catch nothing.
- `PhysicsWorld`: `step`, `teleport_arm`, `joints`, `tcp`, `looking_at`, `vertices(item)`, `location(item)` → `gripper` / `cargo` + color / `laundry` + color / `floor` / `other`, `at(location, color=None)`.
- The arm: `rebot_b601.arm.Arm` runs unchanged on `MujocoBackend`; `PhysicsCamera` renders the D435i (depth noise, no depth under 175 mm) and `segment()` gives MuJoCo's segmentation as SAM3-style instances.
- `sorter.sim.layout` computes the rig (poses, zones, views, keep-out) with the arm's IK and checks it; `sorter.sim.rig` has the sim camera mount.

## Repo layout

| Path | Owner |
| --- | --- |
| `src/sorter/core/`, `app.py`, `__main__.py` | shared |
| `src/sorter/arm/`, `rebot_b601/` | shared |
| `src/sorter/camera/`, `src/sorter/calibration/` | shared |
| `src/sorter/sim/config.py`, `rig.py`, `layout.py`, `physics/` (the base scene) | shared |
| `src/sorter/sim/scenes/load/`, `config/default.yaml` → `sim.load`, `sim.layout.floor_view` | A |
| `src/sorter/sim/scenes/unload/`, `config/default.yaml` → `sim.unload`, `sim.layout.laundry` | B |
| `src/sorter/orchestrator/state_machine.py`, `runlog.py` | shared |
| `src/sorter/orchestrator/load.py`, `src/sorter/floor_detector/`, `src/sorter/color_classifier/` | A |
| `src/sorter/orchestrator/unload.py`, `src/sorter/box_detector/` | B |
| `src/sorter/dashboard/`, `frontend/` (the shared parts: A6) | shared; the load panel A, the unload panel B |
| `config/rig.yaml` | the layout tool (and the setup pages on the rig) |
| `config/hand_eye.yaml` | calibration |
| `tests/<package>/` | same as the package; `tests/conftest.py` shared |
| `ros2_ws/` | the ROS 2 track ([D-014](decisions.md)), outside these stages |

"Shared" means: change it only through the contract rules in [AGENTS.md](../AGENTS.md).

# Architecture

Single source of truth for the contracts between blocks. Block 0 implements the shared types in `src/sorter/core/`; if code and this file diverge, fix both in the same PR (see `AGENTS.md`).

## Physical setup

- **Arm:** Seeed reBot Arm B601-RS (RobStride motors: RS-06 on joints 1–3, RS-00 on joints 4–6 and the gripper; 6 DoF + parallel gripper; reach roughly 0.6–0.7 m from the shoulder axis, from the URDF; see [D-011](decisions.md)). USB→CAN (PEAK PCAN-USB, SocketCAN `can0` at 1 Mbit/s on Linux), driven through `rebot_b601/` (`motorbridge`, own IK/FK, metres + radians; [D-014](decisions.md)).
- **Camera:** Intel RealSense D435i RGB-D, **mounted on the wrist** behind the gripper, looking along the gripper axis (eye-in-hand, [D-006](decisions.md)).
- **Zones at fixed positions** ([D-007](decisions.md)): mixed box, background area (mid-gray), 3 bins (light / dark / colored). The clothes inside the box lie arbitrarily. The layout is `sim.layout` ([D-015](decisions.md)), sized for the arm's top-down reach and all in front of it (the arm is clamped to the table's back edge, [D-017](decisions.md)): a low tray straight ahead (+x), the mat front-left, the bins farther out on both sides.
- The arm looks at a zone from a fixed **look pose** (`look_box`, `look_bg`): the camera straight down, ~260 mm above the table. Because look poses are repeatable, pixel ROIs of each zone are constants in config.

## Components

| Component | Block | Responsibility |
| --- | --- | --- |
| Camera | 1 | Background capture thread. `latest()` for the live feed, `fresh()` for decisions |
| Calibration | 2 | Hand-eye transform. Pixel + depth + camera pose → arm point, and back. The only place where this conversion happens |
| Box detector | 3 | Grasp point in the box (pixels), or "box empty" |
| Color classifier | 4 | All items on the background: color, re-grasp point (pixels), area. Empty list = background empty. Masks from the remote SAM3 service ([D-013](decisions.md)) |
| Arm controller | 5 | Named poses, `look` / `pick` / `place_on_background` / `drop_to_bin`, workspace checks, hold, recover. One `Controller` on an `ArmDriver`: the real arm or the simulator |
| Observer | 0 | `observe(zone)`: move to look pose, take a fresh frame, attach camera pose |
| Hub | 0 | Status, decision frames, and commands between the state machine and the dashboard |
| State machine | 6 | Main loop, failure handling, counters, run logs |
| Dashboard | 7 | Web UI: decision frame with overlays, live wrist feed, state, counters, controls |
| Simulator | 0 | The hardware without hardware: a MuJoCo physics scene with the arm's motors and the wrist RGB-D camera, so the real code of every other block runs on it; a quick kinematic world for tests |

## Coordinate frames and units

| Frame | Units | Notes |
| --- | --- | --- |
| Pixel `(u, v)` | px, int | Color stream, origin top-left. Depth is aligned to color |
| Camera | mm | Optical frame (x right, y down, z forward). **Depth values are Z along the optical axis**, not ray length |
| Flange | mm | URDF `link6` (the wrist roll output) |
| Camera link | mm | URDF `link5`, the link before the wrist roll: the camera is fixed to it, joint 6 turns the gripper but not the camera ([D-022](decisions.md)). `ee_pose()` returns `T_base_link5` from FK (`sorter.arm.kinematics.fk_link5`) |
| TCP | mm | URDF `gripper_end`, the end of the gripper; +x is the approach axis (wrist → fingertips). `T_flange_tcp` is fixed by the URDF (`sorter.arm.kinematics.T_FLANGE_TCP`) |
| Arm base | mm | What `pick()` targets are expressed in |

- `Pose` = `np.ndarray` 4×4 float64, homogeneous transform, translation in **mm**.
- Joint angles in **radians** (URDF joint angles = motor angles after the zero calibration). Durations in seconds. Time is `time.monotonic()`.
- Only `sorter.arm.kinematics` and the arm driver convert to `rebot_b601` units (m, rad).
- `T_base_cam = T_base_link5(q at capture) · T_link5_cam`, where `T_link5_cam` is the hand-eye result (block 2).
- **Rule ([D-002](decisions.md)):** vision outputs pixels + depth only. Calibration owns every pixel ↔ arm conversion.

## Main loop (state machine walkthrough)

The loop is **observation-driven** ([D-008](decisions.md)). Every cycle starts by looking at the background, and the next action follows from what the camera sees, not from what was supposed to happen. The state machine's memory is only: `counters`, `avoid` (failed grasp pixels in the box view), `pending` (last action, to verify it), `failures` (consecutive), `empty_streak`.

```text
IDLE ─START─► STARTING ─► LOOK_BG ─► SENSE_BG ──items──► PICK_FROM_BG ─► DROP_TO_BIN ─┐
                             ▲           │                                            │
                             │           └─empty─► LOOK_BOX ─► SENSE_BOX ─grasp─► PICK_FROM_BOX ─► PLACE_ON_BG ─┐
                             │                                    │  └─empty ×N─► DONE                          │
                             └────────────────────────────────────┴─────────────────────────────────────────────┘
```

| Phase | Calls | Outcomes |
| --- | --- | --- |
| `STARTING` | `arm.start()` (enable, hold), `arm.home()` | → `LOOK_BG`. Counters reset when a run starts from `IDLE`/`DONE` |
| `LOOK_BG` | `obs = observer.observe(BACKGROUND)` | → `SENSE_BG` |
| `SENSE_BG` | `bg = classifier.classify(obs.frame)`; resolve `pending` (below) | `bg.items` non-empty → `PICK_FROM_BG` with `items[0]`. Empty → `LOOK_BOX` |
| `PICK_FROM_BG` | `p = calibration.to_arm(obs, item.grasp)`; `r = arm.pick(p, BACKGROUND)` | OK → `DROP_TO_BIN`. `r.likely_empty` → failure, → `LOOK_BG`. `TargetRejected` → `ERROR` |
| `DROP_TO_BIN` | `arm.drop_to_bin(item.color)`; `pending = Dropped(color, n_before=len(bg.items))` | → `LOOK_BG` |
| `LOOK_BOX` | `obs = observer.observe(BOX)` | → `SENSE_BOX` |
| `SENSE_BOX` | `box = box_detector.detect(obs.frame, avoid)` | `GRASP` → `PICK_FROM_BOX`, `empty_streak = 0`. `EMPTY` → `empty_streak += 1`; `≥ N` → `DONE`, else → `LOOK_BOX`. `NO_GRASP` → if `avoid` non-empty: clear it and retry, else `ERROR` |
| `PICK_FROM_BOX` | `p = calibration.to_arm(obs, box.grasp)`; `r = arm.pick(p, BOX)` | OK → `PLACE_ON_BG`. `TargetRejected` (no motion happened) → add to `avoid`, failure, → `SENSE_BOX` on the **same** `obs`. `r.likely_empty` → add to `avoid`, failure, → `LOOK_BOX` |
| `PLACE_ON_BG` | `arm.place_on_background()`; `pending = PlacedFromBox(px)` | → `LOOK_BG` |
| `DONE` | `arm.home()` | mode `idle` |

**Resolving `pending` in `SENSE_BG`:**

- `PlacedFromBox(px)` and the background is empty → **missed grasp from box**: add `px` to `avoid`, failure. Items present → success: clear `avoid`, `failures = 0`.
- `Dropped(color, n_before)` and `len(items) < n_before` → **verified drop**: `counters[color] += 1`, `failures = 0`. Otherwise the pick from the background failed: failure, the item is retried.

**Invariants and what they give us:**

- We pick from the box only when the background is empty, so "empty background after place" always means a missed grasp.
- **Double grasp needs no special code:** the classifier returns every blob, and they are sorted one per cycle.
- **Counters go up only after a verified drop.**
- After a hold, error, or restart, the loop resumes from `LOOK_BG` and re-derives the situation from the camera.
- `avoid` is in pixels of the box view. That works because `look_box` is repeatable. It is cleared after every successful place.

**Failures and control:**

- `failures ≥ state_machine.max_consecutive_failures` → phase `ERROR`, mode `paused`, error shown on the dashboard. `RESUME` resets `failures` and continues from `LOOK_BG`.
- `STOP` → finish the current phase, `arm.home()`, phase `IDLE`. Counters are kept until the next `START`.
- Commands are applied **between phases**. Every phase is atomic, so pause and step never stop the arm mid-air. `STEP` runs exactly one phase, then pauses.
- `HOLD` is not queued: the Hub calls `arm.hold()` directly from the web thread. The blocked arm call in the state machine raises `EStopped` → phase `HELD`. `RESET` → `arm.recover()` → `LOOK_BG`.
- Any `ArmError` / `CameraError` / `CalibrationError`, or an unexpected exception from a module → phase `ERROR`, mode `paused`; the loop thread never dies. `RESET` → `arm.recover()` → `LOOK_BG`. `pending` survives a hold or error: it is set only after its action completed, so the next `SENSE_BG` still verifies it.
- Low-confidence color (below `state_machine.low_confidence`): sort into the most likely class and log a warning.
- Every decision is saved to the run log (see _Recording format_).

## Contracts

All shared types live in `sorter.core.types`, errors in `sorter.core.errors`, the `Protocol`s (`Camera`, `BoxDetector`, `ColorClassifier`, `Calibration`, `ArmController`) in `sorter.core.protocols`. Data types are frozen dataclasses unless they carry mutable debug data. Array fields are read-only (`flags.writeable = False`); consumers never modify them.

### Errors

```python
class SorterError(Exception): ...


class CameraError(SorterError): ...  # no frame within timeout, device lost


class CalibrationError(SorterError): ...  # no camera pose, invalid depth, file missing


class ArmError(SorterError): ...  # SDK/bus fault, motion failed or timed out


class TargetRejected(ArmError): ...  # outside the zone workspace or IK failed; NO motion happened


class EStopped(ArmError): ...  # arm is held; every motion raises this until recover()
```

"Nothing found" is never an exception. Vision returns it as a result value.

### Basic types

```python
Pose = np.ndarray  # 4x4 float64, mm


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
    x: float
    y: float
    z: float  # mm, arm base frame
```

### Camera (block 1)

```python
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


class Camera(Protocol):
    def start(self) -> None: ...
    def close(self) -> None: ...
    def latest(self) -> Frame | None: ...  # non-blocking; for the live feed
    def fresh(self, timeout_s: float = 2.0) -> Frame: ...

    # Blocks until a frame whose exposure started AFTER the call. Raises CameraError on timeout.
```

- The camera runs its own capture thread and keeps only the newest frame.
- Auto exposure and white balance are locked after warm-up if the SDK allows.
- The camera module knows nothing about the arm.
- The real backend is `RealSenseCamera` (`pyrealsense2`, the `camera` extra; Linux / Windows): depth aligned to color, rectified (`coeffs` empty).

### Observation and Observer (block 0)

```python
@dataclass(frozen=True)
class Observation:
    frame: Frame
    zone: Zone
    T_base_cam: Pose | None           # None only in recordings made without the arm
    joints: tuple[float, ...] | None  # arm joints at capture, rad

class Observer:
    def __init__(self, camera: Camera, arm: ArmController, calibration: Calibration): ...
    def observe(self, zone: Zone) -> Observation:
        # arm.look(zone)  → blocks until the arm is still
        # frame = camera.fresh()
        # T_base_cam = calibration.cam_pose(arm.ee_pose())
```

### Vision: shared types

```python
@dataclass(frozen=True)
class GraspPoint:
    px: PixelPoint
    depth_mm: float  # robust (median over a small window) Z of the cloth SURFACE at px; never 0


@dataclass
class Marker:
    px: PixelPoint
    label: str
    kind: Literal["grasp", "candidate", "avoid", "info"]


@dataclass
class Overlay:  # drawn by the dashboard on top of the frame it came from
    markers: list[Marker] = field(default_factory=list)
    polygons: list[tuple[list[PixelPoint], str]] = field(default_factory=list)
    mask: np.ndarray | None = None  # HxW bool
    text: list[str] = field(default_factory=list)
```

- Vision returns the **surface** point. How deep the gripper goes below it is the arm's business (per-zone `grasp_depth_mm`).
- Vision works on `Frame` only. The zone ROI comes from config (`views.<zone>.roi`).

### Box detector (block 3)

```python
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


class BoxDetector(Protocol):
    def detect(self, frame: Frame, avoid: Sequence[PixelPoint] = ()) -> BoxResult: ...
```

- **Stateless.** Failed grasps are passed as `avoid`. Candidates within `box_detector.avoid_radius_px` of any of them are skipped.
- `NO_GRASP` = cloth is present, but no valid candidate is left.
- The grasp point keeps `wall_margin_mm` from the ROI edge on the box floor (an open finger and its pad), converted to pixels with the floor depth.
- `DepthBoxDetector` (the real backend): the floor is a high percentile of the ROI depth, cloth is what stands `cloth_height_mm` above it, the grasp is the top of the smoothed height map inside the cloth.
- The ROI excludes the gripper fingers. They sit at fixed pixels in every frame.

### Color classifier (block 4)

```python
@dataclass
class ItemResult:
    color: ColorClass
    confidence: float  # 0..1
    grasp: GraspPoint  # re-grasp point
    area_px: int
    touches_roi_edge: bool  # item partly outside the background ROI
    stats: dict[str, float]  # e.g. median L, a, b, chroma, for tuning and the dashboard


@dataclass
class BackgroundResult:
    items: list[ItemResult]  # one per blob, largest first; [] = background empty
    overlay: Overlay


class ColorClassifier(Protocol):
    def classify(self, frame: Frame) -> BackgroundResult: ...
```

- The real backend sends the color image to the SAM3 service ([D-013](decisions.md)). If the service fails, `classify` raises `SegmentationError` (`sorter.color_classifier.segmenter`, a `SorterError`), so the state machine goes to `ERROR`.
- `stats` holds `L` (0..100), `a`, `b`, `chroma` (median over the eroded mask), `px` (pixels used), `score` (SAM3 instance score).

### Calibration (block 2)

```python
class Calibration(Protocol):
    def cam_pose(self, ee_pose: Pose) -> Pose: ...  # T_base_link5 → T_base_cam
    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint: ...
    def to_pixel(
        self, obs: Observation, p: ArmPoint
    ) -> PixelPoint | None: ...  # None if outside the image
```

- `to_arm`: deproject `(u, v, depth)` with `obs.frame.intrinsics`, then apply `obs.T_base_cam`. It raises `CalibrationError` if `T_base_cam` is `None` or depth is 0.
- The hand-eye result `T_link5_cam` is stored in `config/hand_eye.yaml`: 4×4 in mm, `rmse_mm`, `method`, `camera_serial`, `created`. It is **committed** ([D-007](decisions.md)), because it depends only on the camera mount, which travels with the arm.
- It must be computed against the same frame that `ee_pose()` returns: `link5`. A file with the old key `T_flange_cam` (link6) fails to load: recalibrate.
- Two ways to compute it:
  - the **calibration page** (`/calibrate`, setup mode, [D-021](decisions.md)), no printed board: the tip points at spots around the mat center (`sorter.calibration.marks`, `calibration.marks_z_mm` high) and tape marks go under it, where the tip really stopped (FK of the measured joints); marks clicked in the live image + depth give camera-frame points, and a rigid fit (Kabsch) of all clicks, from any poses, gives `T_link5_cam`;
  - `python -m sorter.calibration.hand_eye` ([D-023](decisions.md)): a ChArUco board (7 × 5 squares of 45 mm, 32 mm 6×6 markers; `python -m sorter.calibration.board` prints it) flat under `look_bg`, its top `calibration.board_z_mm` above the table; `calibration.poses` views around `look_bg`, Park's method as the start, then a fit of every corner's reprojection over the mount and the board lying flat at that height.
- The real backend (`HandEyeCalibration`) raises `CalibrationError` at start if there is no result. Setup mode starts without one, on the nominal mount (`sim.camera_mount_mm`).

### Arm controller (block 5)

```python
@dataclass(frozen=True)
class PickResult:
    gripper_opening: float  # 0 = fully closed .. 1 = fully open, after closing
    likely_empty: bool  # gripper_opening < arm.gripper.empty_below; a hint, the camera is the truth


class ArmController(Protocol):
    def start(self) -> None: ...  # connect, enable motors, hold the current position
    def shutdown(self) -> None: ...  # move to the `rest` pose, then disable motors
    def home(self) -> None: ...
    def look(self, zone: Zone) -> None: ...  # go to look_box / look_bg; no-op if already there
    def pick(self, target: ArmPoint, zone: Zone) -> PickResult: ...
    def place_on_background(self) -> None: ...
    def drop_to_bin(self, color: ColorClass) -> None: ...
    def ee_pose(self) -> Pose: ...  # T_base_link5 (the camera link) from FK of the measured joints
    def joints(self) -> tuple[float, ...]: ...
    def hold(self) -> None: ...  # thread-safe; freeze in place; see below
    def recover(
        self,
    ) -> None: ...  # leave hold: lift to safe Z, open gripper above the background, home
```

- **Blocking:** every motion method returns only once the arm is still. A fault during a motion raises `ArmError`.
- **`pick(target, zone)`, with `target` = the cloth surface point where the TCP should go:**
  1. Reject (`TargetRejected`, no motion) if `target` XY is outside `zones.<zone>.workspace_mm`, or if IK or a path check fails for any of the three moves below. The whole pick is planned before anything moves (`Controller.plan_pick`).
  2. Go to `target.z + approach_mm` with the gripper pointing down (`arm.approach`), open the gripper.
  3. Descend in a straight line to `max(target.z - grasp_depth_mm, z_floor_mm)`. Z is clamped, XY is never clamped: a clamped XY means grasping a box wall.
  4. Close the gripper, lift in a straight line to `zones.<zone>.lift_z_mm`.
- **`place_on_background` / `drop_to_bin`:** joint moves to the fixed poses (`place_bg`, `bin_<color>`), then open the gripper. `place_bg` holds the TCP `arm.place_release_height_mm` above the mat so the cloth lands crumpled, which makes the re-grasp easier.
- **Hold vs disable:** disabling the motors makes **the arm fall**. The software stop (dashboard button, Ctrl+C) is `hold()`: abort the motion and hold the joints where they are. After `hold()`, every motion raises `EStopped` until `recover()`. Cutting power is the job of the hardware e-stop switch ([D-009](decisions.md)). `shutdown()` releases a hold, lifts, goes to `rest`, and only then disables.
- **Driver faults:** `rebot_b601` latches a fault when a joint stays >12° from its setpoint for 0.4 s (blocked, collision, weak motor), on lost feedback or overheating: the motion aborts, the torque stays on holding the measured pose, and every motion raises `ArmError` until the fault is cleared. `driver.fault()` returns the message (with the commanded and measured angle), `clear_fault()` accepts it with the torque still on (`Arm.clear_fault`), or reconnects if the torque is already off ([D-020](decisions.md)).
- **Inside block 5** ([D-014](decisions.md)): `sorter.arm.controller.Controller` implements the protocol on an `ArmDriver` (`sorter.arm.driver`: `connect`, `disconnect`, `joints`, `gripper`, `execute(waypoints, speed_scale)`, `set_gripper`, `stop`, `resume`, `fault`, `clear_fault`). `RebotDriver` wraps `rebot_b601.arm.Arm` (the CAN bus, or its simulated motors with `arm.dry_run`); the simulator has `SimDriver`. Planning (`sorter.arm.kinematics`: IK, straight-line and joint paths, joint limits, table clearance `arm.z_min_mm`) is shared. The controller also has `gripper_opening()` for the 3D view, and for the dashboard's setup mode `held`, `at`, `go_to(pose)`, `move_joints(q)`, `set_gripper(opening)`, `release()` (leave hold without moving), `fault`, `clear_fault()` and `set_pose(name, q)`, and for the dashboard's speed control (`Hub(speed=)`, D-025) `speed_scale`, `max_speed_scale`, `set_speed_scale(scale)`; none are in the protocol. Block 2 uses `sorter.arm.kinematics` for FK.

### Hub: state machine ↔ dashboard (block 0)

```python
class Phase(StrEnum):
    (
        IDLE,
        STARTING,
        LOOK_BG,
        SENSE_BG,
        PICK_FROM_BG,
        DROP_TO_BIN,
    )
    LOOK_BOX, SENSE_BOX, PICK_FROM_BOX, PLACE_ON_BG, DONE, HELD, ERROR


class Command(StrEnum):
    START, PAUSE, RESUME, STEP, STOP, HOLD, RESET


@dataclass(frozen=True)
class Event:
    t: float
    level: Literal["info", "warning", "error"]
    source: str
    msg: str


@dataclass(frozen=True)
class Status:
    phase: Phase
    next_phase: Phase | None  # what STEP will run
    mode: Literal["idle", "running", "paused"]
    run_id: str | None
    cycle: int
    counters: dict[ColorClass, int]
    failures: int  # consecutive
    last_cycle_s: float | None
    error: str | None
    health: list[str]  # startup problems; empty = OK
    events: list[Event]  # last ~50


@dataclass(frozen=True)
class Decision:  # what the state machine decided on, for the dashboard
    phase: Phase
    obs: Observation
    overlay: Overlay
    summary: str  # e.g. "grasp (412, 230) depth 540 mm" / "colored 0.93"


class TwinSource(Protocol):  # the 3D view's data
    def layout(self) -> dict[str, Any]: ...
    def state(self) -> dict[str, Any]: ...


class SpeedControl(Protocol):  # the arm's speed, set from the dashboard (block 5's Controller)
    speed_scale: float  # read-only
    max_speed_scale: float  # read-only
    def set_speed_scale(self, scale: float) -> float: ...  # clamped; returns the speed set


class Hub:
    def __init__(
        self,
        camera: Camera,
        on_hold: Callable[[], None],
        twin: TwinSource | None = None,
        speed: SpeedControl | None = None,
    ): ...
    # state machine side
    def publish_status(self, s: Status) -> None: ...
    def publish_decision(self, d: Decision) -> None: ...
    def next_command(self, timeout_s: float) -> Command | None: ...
    # dashboard side
    def status(self) -> Status: ...
    def decision(self) -> Decision | None: ...
    def live_frame(self) -> Frame | None: ...  # proxy to camera.latest()
    def twin(self) -> TwinSource | None: ...
    def speed(self) -> SpeedControl | None: ...
    def send(self, cmd: Command) -> None: ...  # HOLD → on_hold() immediately; others → queue
```

- Modules log with the standard `logging` module. `HubLogHandler` turns warnings and errors into `Event`s, so no module needs a Hub reference.
- Blocks 6 and 7 depend only on the Hub, never on each other.
- `build_system` gives the Hub a `sorter.dashboard.twin.Twin`: the table layout (`sim.layout`, zone workspaces, camera intrinsics) and the live state (link poses from FK of `arm.joints()`, gripper opening, TCP, `T_base_cam` from `calibration.cam_pose(arm.ee_pose())`, and the sim items with their location when there is a sim world).

### Dashboard HTTP API (block 7)

| Endpoint | Content |
| --- | --- |
| `GET /` | Single page, no build step |
| `GET /api/status` | `Status` as JSON, plus `now` (`time.monotonic()`, the clock of `Event.t`) for event ages and `speed` (as `GET /api/speed`, null without a speed control) |
| `WS /ws` | The same JSON, pushed on change (a speed change too), at most `dashboard.status_hz` (5 Hz) |
| `GET /api/speed` | `{"speed_scale", "max_speed_scale"}` of the arm ([D-025](decisions.md)); 404 without a speed control |
| `POST /api/speed` | `{"speed_scale": float}` → the same JSON. Works in any mode; the next motion uses it (one under way keeps its speed); clamped to [0.05, `max_speed_scale`]; not saved (a restart goes back to `arm.speed_scale`) |
| `GET /stream/decision.mjpg` | Last decision frame with the zone ROI (`views.<zone>.roi`), the overlay, and a caption strip (phase + `summary`) drawn server-side |
| `GET /stream/live.mjpg` | Live wrist camera (`hub.live_frame()`), at `dashboard.stream_fps` |
| `GET /snapshot/decision.jpg`, `GET /snapshot/live.jpg` | One JPEG of the same images |
| `POST /api/command` | `{"cmd": "start" \| "pause" \| "resume" \| "step" \| "stop" \| "hold" \| "reset"}` |
| `GET /twin` | 3D view (three.js): the arm's CAD meshes posed from FK, the table, the items, the wrist camera's view on the table. `?embed` for the dashboard's main screen, which switches between the decision frame and this view |
| `GET /api/twin/layout`, `GET /api/twin/state` | `TwinSource.layout()` / `.state()` as JSON; 404 without a source. Link and camera poses are 4×4 row-major in metres, the rest in mm |
| `GET /twin-assets/...` | The CAD meshes and three.js vendored in `rebot_b601/rebot_b601/viewer_assets/` |
| `GET /manual` | Manual control page for setting up the rig (setup mode, see below) |
| `GET /api/manual` | `ManualControl.state()`: `busy`, `action`, `last`, `error`, `held`, `fault` (driver fault message or null), `at` (named pose or null), `joints` (rad), `gripper`, `tcp_mm`, `poses`, `tour`, `tour_next`, `rig_file`; 404 outside setup mode |
| `GET /calibrate` | Camera calibration page (setup mode, see below) |
| `GET /api/calibrate` | `CalibrateControl.state()`: `arm` (the `/api/manual` state), `image` (`width`, `height`), `overlay` (`marks`: name + pixel or null, `zones`: box / background outline pixels, from where the arm is, with the fitted mount if any else the one in use), `marks` (`name`, `xyz`, `placed`), `clicks` (`mark`, `px`, `here`, `pose` (the view it was clicked from, 0-based), `residual_mm`), `fit` (`rmse_mm`, `change_mm`, `change_deg` from the mount in use, `marks`; on the sim `true_error_mm`, `true_error_deg`) or null, `mount` (`method`, `saved`), `look` (per look pose: `tcp_z`, `camera_mm`, `sees_mm`, `coverage`, `depth_ok`, `fits`, `top`) or null, `look_saved`, `hand_eye_file`, `rig_file`, `views`, `hover_mm`, `detected` (`squares`, `marks` of the last detection) or null; 404 outside setup mode |
| `POST /api/calibrate` | `{"action": "goto_mark", "mark"}` \| `{"action": "goto_view", "index"}` \| `{"action": "detect"}` → `{"ok", "marks"}` \| `{"action": "click", "mark", "u", "v"}` \| `{"action": "delete_click", "index"}` \| `{"action": "clear_clicks" \| "compute_look"}` \| `{"action": "goto_look", "pose"}` \| `{"action": "save_mount"}` → `{"ok", "file"}` \| `{"action": "save_look"}` → `{"ok", "lines"}` \| `{"action": "calibrate"}` (save_mount + compute_look + save_look) → `{"ok", "file", "lines"}`. 409 while a motion runs, 400 on a bad action or when it can't be done (no depth at the click, no fit yet, unreachable) |
| `POST /api/manual` | `{"action": "go", "pose"}` \| `{"action": "tour_next" \| "tour_reset" \| "release" \| "clear_fault"}` \| `{"action": "jog", "joint": 0..5, "delta_deg"}` \| `{"action": "gripper", "open": bool}` \| `{"action": "save", "pose"}` → `{"ok", "line"}`. 409 while a motion runs, 400 on a bad action or pose |

**Setup mode** (`python -m sorter manual [--sim]`, `create_app(..., manual=ManualControl)`): no state machine; `/` redirects to `/manual`, where the operator moves the arm by hand while watching the wrist camera and the 3D view. `sorter.dashboard.manual.ManualControl` runs one motion at a time in its own thread: named poses (a move to or from a bin goes via `home`), a tour `look_box → look_bg → place_bg → bin_light → bin_dark → bin_colored → home` one pose per press, joint jog, the gripper, and `save` (the current joints become that pose in memory and in `config/rig.yaml`, the rest of the file kept). Hold is the usual `POST /api/command {"cmd": "hold"}`; `release` leaves it without moving. A driver fault shows as a banner with **Clear fault** (`clear_fault`, after a confirmation), which also leaves hold. The page switcher (`static/nav.js`) greys out the page of the other mode: Dashboard in setup mode, Manual and Calibrate under `run`.

**Calibration page** (`/calibrate`, setup mode, `create_app(..., calibrate=CalibrateControl)`, [D-021](decisions.md)): `sorter.dashboard.calibrate.CalibrateControl` runs its motions through the same `ManualControl` (one at a time, same busy / hold rules). The page is a 4-step wizard (Marks → Click → Calibrate → Check); its **Calibrate** button is the `calibrate` action. (1) **Marks**: `goto_mark` lifts, closes the gripper, goes above the mark and straight down to `HOVER_MM` above it, gripper vertical; the tip's measured position becomes the mark's, kept in `data/calibration_marks.yaml` across restarts (not on the sim, whose marks are drawn at their nominal spots). (2) **Camera mount**: `goto_view` puts the camera over the marks, aimed with the saved mount (the nominal one before the first save), as close as the arm can (3 views, shifted; clicks within `SAME_POSE_RAD` = 0.01 rad of each other are one view, since the measured joints wander a few mrad at a pose) and then finds the marks by itself (`detect`, also an action): `find_squares` (dark, square blobs of the tape's size at their depth) and `identify` (which mark each is, from the 3D distances between them, the mirrored match rejected as it puts the camera under the table) make the clicks of that pose; each manual `click` snaps to the center of the dark tape square within 35 px (`snap`), then takes a fresh frame's depth around it (`deproject`) and the link5 pose; the fit updates with every click; the mark pattern is symmetric about the M1–M6 line, so each view is also tried with M2↔M3, M4↔M5 swapped and the fit that has the camera above the table and looking down from every view wins (a mirrored view flips it under the table; the view's labels are fixed); a fit whose camera is more than 200 mm from the nominal one or whose optical axis is more than 45° off is not saved (the turn about the optical axis is free: the real camera may be mounted any way round), and the views are always aimed with the nominal mount (a one-view fit can lose the marks); `save_mount` writes `hand_eye.yaml` (the sim: `data/hand_eye_sim.yaml`) and sets the live calibration's `T_link5_cam` (the 3D view and the overlay follow). (3) **Look poses**: `compute_look` finds, for the mount in use, the lowest TCP height (60–200 mm, 10 mm steps, stopping where the coverage starts to drop or the arm can't reach) with the camera's axis through the zone center or as close as the arm can put it, the gripper vertical (the image's turn is whatever the arm gives: joint 6 doesn't turn the camera), the whole zone in the image and the camera ≥ 200 mm above it (depth); `goto_look` previews one and measures the image rows the gripper hides (after a save it rewrites the ROI without them); `save_look` writes both poses (`write_pose`) and `views.<zone>.roi` (`write_views`) into `rig.yaml` and uses them at once.

The decision frame is the main panel: the wrist feed moves with the arm, and overlays only match the frame they were computed on. The caption is part of the image for the same reason. Each decision is rendered and encoded once. An optional fixed scene webcam for the audience can be added by block 7 (`dashboard.scene_camera`, not implemented), with no effect on the loop.

## Threads and process

One Python process ([D-005](decisions.md)):

- camera capture thread (block 1; in the physics sim, its render thread);
- arm control loop thread, 50 Hz, inside `rebot_b601.arm.Arm` (block 5). In the physics sim the loop is ticked by the physics thread every 20 ms of simulated time; the kinematic sim driver has none;
- physics thread (physics sim only): steps MuJoCo at `sim.realtime`;
- state machine thread (block 6);
- web server, uvicorn (block 7).

`python -m sorter manual [--sim]` (setup mode) has no state machine thread: camera, arm, web server. `python -m sorter run [--sim]` wires everything (block 0, `sorter/app.py`). Ctrl+C → `arm.hold()`, stop the loop, `arm.shutdown()` (rest pose, then disable).

## Wiring (block 0)

`sorter.app.build_system(cfg, sim=False) -> System` creates every component per `backends`. `--sim` (`sim=True`) makes the hardware sim: the camera and the arm. With `sim.engine: physics` (default) the other components keep their real backends and run on the rendered frames; only the calibration comes from the sim camera mount (exact) instead of `hand_eye.yaml`, and the classifier's SAM3 client is replaced by the render's segmentation unless `sim.use_sam3`. With `sim.engine: kinematic` (tests) every backend set to `sim` stays sim. `System` (`sorter.core.system`) holds `cfg`, `camera`, `arm`, `calibration`, `box_detector`, `color_classifier`, `observer`, `hub`, and `world` (the `SimWorld`, or `None` when nothing is simulated).

Entry points each block provides:

| Block | Entry point |
| --- | --- |
| 1, 2, 3, 4, 5 | `sorter.<package>.backend.create(cfg: Config) -> <Protocol>`: the real backend. Until the module exists, `backends.<name>: real` fails with a clear error |
| 6 | `sorter.orchestrator.state_machine.StateMachine(system)` with `run(stop: threading.Event)`, the loop for the state machine thread |
| 7 | `sorter.dashboard.server.create_app(hub, cfg.dashboard, views=cfg.views) -> FastAPI` (`views` gives the ROIs drawn on the decision frame) |

Package `__init__.py` files stay empty: `sorter.core.config` imports every block's config model, so an import in an `__init__` can create a cycle. Import driver SDKs inside `backend.create`, so sim runs don't need them.

## Config

YAML, loaded and deep-merged in this order: `config/default.yaml` → `config/rig.yaml` → `config/hand_eye.yaml` → `config/local.yaml` (gitignored, machine overrides). Missing files are skipped, except `default.yaml`. Validated with pydantic by `sorter.core.config.load_config()`: unknown top-level sections are an error.

Each block defines the model for its own section in `src/sorter/<package>/config.py` (e.g. `BoxDetectorConfig` in `sorter/box_detector/config.py`). Block 0 created them as placeholders that accept any key; the owner adds typed fields.

| Key | Owner | Content |
| --- | --- | --- |
| `backends` | 0 | Per component `real` \| `sim`: `camera`, `arm`, `calibration`, `box_detector`, `color_classifier`. Swap stubs one at a time during integration |
| `sim` | 0 | Simulator world: `engine` (`physics` \| `kinematic`), `realtime` (physics: simulated s per wall s, 0 = as fast as possible), `use_sam3`, `board` (a ChArUco board on the mat), `seed`, `items` (colors in the box), `miss_prob`, `double_prob`, `time_scale` (arm motion time × this; 0 = instant), `vision_s` (sim vision delay), image size, `focal_px`, `camera_mount_mm` (wrist camera in the TCP frame), `item_radius_mm`, `layout` (`edge_x_mm`: the table's back edge; `box`: `center_mm`, `size_mm`, `floor_z_mm`, `wall_mm`; `background`: `center_mm`, `size_mm`; `bins`: `centers_mm.<color>`, `size_mm`, `wall_mm`, `floor_z_mm`) |
| `camera` | 1 | `serial` (empty = the first D435i), `width`, `height`, `fps`, `warmup_frames`, `lock_exposure`, `exposure_us`, `white_balance_k`, `timeout_s`, `start_attempts` |
| `views.<zone>.roi` | 1 | Pixel polygon of the zone in its look pose, excluding the gripper fingers (`rig.yaml`) |
| `calibration` | 2 | `hand_eye` (the transform, from `hand_eye.yaml`); hand-eye tool: `poses`, `tilt_deg`, `shift_mm`; calibration page: `marks_z_mm` (the mat top) |
| `box_detector` | 3 | `avoid_radius_px`, `wall_margin_mm`, `floor_percentile` / `floor_depth_mm`, `cloth_height_mm`, `min_cloth_px`, `inset_px`, `smooth_px`, `depth_window_px` |
| `color_classifier` | 4 | `sam` (SAM3 service: `url`, `api_key`, `prompts`, `threshold`, `mask_threshold`, `timeout_s`), class thresholds (`lightness_dark`, `chroma_colored`, `lightness_light`, `confidence_margin`), `erode_px`, `min_area_px`, `max_area_frac`, `overlap_max`, re-grasp (`grasp_inset_px`, `grasp_depth_tol_mm`, `depth_window_px`). The API key goes in `local.yaml` or env `SAM3_API_KEY`, never committed |
| `arm` | 5 | `dry_run`, `speed_scale` (of `rebot_b601`'s joint speeds, capped at 0.6; the start value, `POST /api/speed` changes it at runtime), `approach`, `safe_z_mm` (recover / shutdown lift), `z_min_mm` (table clearance), `place_release_height_mm`, `bin_release_height_mm` (both used by the layout tool), gripper (`open`, `empty_below`) |
| `poses` | 5 | Joint angles (rad): `rest`, `home`, `look_box`, `look_bg`, `place_bg`, `bin_light`, `bin_dark`, `bin_colored` (`rig.yaml`; computed for `sim.layout` by `python -m sorter.sim.layout --write`, re-taught on the rig) |
| `zones.<zone>` | 5 | `workspace_mm` (XY polygon, arm frame), `z_floor_mm`, `grasp_depth_mm`, `approach_mm`, `lift_z_mm` (`rig.yaml`, same tool) |
| `state_machine` | 6 | `empty_confirmations`, `max_consecutive_failures`, `low_confidence`, `save_runs`, `runs_dir` |
| `dashboard` | 7 | `host`, `port`, `stream_fps`, `jpeg_quality`, `status_hz` |

## Recording format (block 0)

`sorter.core.io.save_observation(path, obs)` / `load_observation(path)`: one `.npz` per observation (`color`, `depth_mm`, intrinsics, `timestamp`, `zone`, `T_base_cam`, `joints`), plus a `.png` of the color image for browsing.

- **Datasets** (block 1): `data/datasets/<name>/`. Record from the real look poses so the viewpoint matches runtime.
- **Run logs** (block 6, `sorter.orchestrator.runlog`): one record per sense phase in `data/runs/<run_id>/`: `<cycle:04d>_<phase>.npz` (+ `.png`) and a `.json` with the vision result (without arrays), the resolved `pending`, `avoid`, counters, the summary, and `next_phase`. A phase repeated within a cycle gets a suffix (`0003_sense_box_2`); a re-sense of the same observation writes only a `.json` whose `observation` names the existing `.npz`. `run.json` holds the config at start and the counters and `end_reason` (`done` / `stopped`) at the end. Run logs double as test data for blocks 3 and 4.
- `data/` is gitignored. A few small fixtures for tests live in `tests/fixtures/`.

## Simulator (block 0)

`--sim` simulates the **hardware only** (camera, arm), so everything above it runs its real code and moving to the rig is `run` without `--sim` ([D-016](decisions.md)). Two engines, `sim.engine`:

**`physics` (default), `sorter.sim.physics`:** a MuJoCo scene built from `sim.layout` and the arm's URDF.

- `PhysicsWorld` steps the scene in its own thread in simulated time (`sim.realtime` × wall time; 0 = as fast as the CPU allows). Items are cloth (2D flex, crumpled rest shape) that fall into the box, fold, hang from the gripper and land where they are dropped. Cloth does not collide with other cloth (it costs ~10× the rest of the step), so a pile interpenetrates.
- The arm: `rebot_b601.arm.Arm` runs unchanged with `MujocoBackend` in place of the CAN bus (joint setpoints for motors 1–6, torque for motor 7, feedback of position, speed, torque), and its 50 Hz loop runs on the simulated clock. The joints have position servos with gravity compensation (the RobStride position loop); the gripper is a force motor (`GRIPPER_N_PER_NM`, an assumption) with the damping rebot sets on motor 7. Tracking-error, limit and fault checks are the real ones.
- Grip: when a close command has stalled the fingers on an item that touches both pads, its vertices between the pads are attached to the gripper until an open command (a stand-in for fabric friction, which soft contacts underestimate).
- `PhysicsCamera` renders the wrist D435i: pinhole at `sim.focal_px` from the `wrist` camera on the `link5` body (`camera_mount()`: `sim.camera_mount_mm` off the TCP with joint 6 at 0, image long side radial), depth with noise growing with distance, no data under 175 mm (the fingers) and on random speckles, its own capture thread. `segment()` returns MuJoCo's segmentation as SAM3-style instances.
- `sorter.sim.physics.backend.hand_eye(cfg)` is the true `T_link5_cam` of the sim mount; `python -m sorter.calibration.hand_eye --sim` must recover it from views of the rendered board, and the calibration page on `manual --sim` from the tape marks, which `sim.marks` (on in setup mode) draws on the mat.
- `connect()` puts the items back in the box when none is left in the box or on the mat.

**`kinematic`, `sorter.sim.world` (tests, the color-classifier stats tool):** no physics.

- `SimWorld` holds items as points (position, color, height, location) and the arm's joints over time. The arm is the real `Controller` on `SimDriver`: planned paths play back with `rebot_b601`'s min-jerk timing × `sim.time_scale`. Closing the gripper grabs the highest cloth under the fingertips if the TCP got within 12 mm of its top (misses with `sim.miss_prob`; from the box it drags a second item with `sim.double_prob`); opening it drops what it holds onto whatever is below.
- `SimCamera` renders sprites straight down from the camera pose (`sorter.sim.scene`). `SimCalibration` is a linear pixel ↔ XY mapping per zone; sim vision (`SimBoxDetector`, `SimColorClassifier`) reads the world directly, after `sim.vision_s`.

Both: `world.looking_at` is the zone whose look pose the joints are at. `python -m sorter.sim.layout [--write]` computes `poses`, `zones` and `views.<zone>.roi` for the layout with the arm's IK and the physics camera, and checks every pick and pose-to-pose move and that nothing lies behind `layout.edge_x_mm` ([D-015](decisions.md), [D-017](decisions.md)).

## Repo layout

| Path | Owner |
| --- | --- |
| `pyproject.toml`, `uv.lock`, `.gitignore`, tooling config | 0 |
| `src/sorter/core/` (types, protocols, errors, config loader, io, hub, observer, `HubLogHandler`, `System`) | 0 |
| `src/sorter/app.py`, `src/sorter/__main__.py` (wiring, CLI) | 0 |
| `src/sorter/sim/` | 0 |
| `src/sorter/camera/` (drivers, record tool, ROI tool) | 1 |
| `src/sorter/calibration/` (hand-eye: board tool, tape marks fit; verification tool) | 2 |
| `src/sorter/box_detector/` | 3 |
| `src/sorter/color_classifier/` (incl. stats tool) | 4 |
| `src/sorter/arm/` (kinematics, driver, controller, pose teaching tool) | 5 |
| `rebot_b601/` (standalone RS driver, IK, planner, MCP server, 3D assets; a path dependency) | 5 |
| `src/sorter/orchestrator/` (state machine, run log) | 6 |
| `src/sorter/dashboard/` (server, renderer, manual and calibration pages, `static/` pages) | 7 |
| `src/sorter/<package>/config.py` | The block that owns the package |
| `tests/<package>/` | Same as the package |
| `tests/conftest.py` (shared fixtures) | 0 |
| `config/default.yaml` | Each block its own section; structure by 0 |
| `config/rig.yaml` | `views`: 1; `poses`, `zones`: 5 |
| `config/hand_eye.yaml` | 2 |
| `docs/demo.md` | 8 |

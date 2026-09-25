# Architecture

Single source of truth for the contracts between blocks. Block 0 implements the shared types in `src/sorter/core/`; if code and this file diverge, fix both in the same PR (see `AGENTS.md`).

## Physical setup

- **Arm:** Seeed reBot Arm B601-RS (RobStride motors: RS-06 on joints 1–3, RS-00 on joints 4–6 and the gripper; 6 DoF + parallel gripper; reach roughly 0.6–0.7 m from the shoulder axis, from the URDF; see [D-011](decisions.md)). USB→CAN (PEAK PCAN-USB, SocketCAN `can0` at 1 Mbit/s on Linux), Python SDK [`reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py) (Pinocchio IK/FK, metres + radians).
- **Camera:** RGB-D, **mounted on the wrist** behind the gripper, looking along the gripper axis (eye-in-hand, [D-006](decisions.md)). Model _TBD_ (Seeed demos support Orbbec Gemini 2, RealSense D435i / D405).
- **Zones at fixed positions** ([D-007](decisions.md)): mixed box, background area (mid-gray), 3 bins (light / dark / colored). The clothes inside the box lie arbitrarily.
- The arm looks at a zone from a fixed **look pose** (`look_box`, `look_bg`). Because look poses are repeatable, pixel ROIs of each zone are constants in config.

## Components

| Component | Block | Responsibility |
| --- | --- | --- |
| Camera | 1 | Background capture thread. `latest()` for the live feed, `fresh()` for decisions |
| Calibration | 2 | Hand-eye transform. Pixel + depth + camera pose → arm point, and back. The only place where this conversion happens |
| Box detector | 3 | Grasp point in the box (pixels), or "box empty" |
| Color classifier | 4 | All items on the background: color, re-grasp point (pixels), area. Empty list = background empty. Masks from the remote SAM3 service ([D-013](decisions.md)) |
| Arm controller | 5 | Named poses, `look` / `pick` / `place_on_background` / `drop_to_bin`, workspace checks, hold, recover |
| Observer | 0 | `observe(zone)`: move to look pose, take a fresh frame, attach camera pose |
| Hub | 0 | Status, decision frames, and commands between the state machine and the dashboard |
| State machine | 6 | Main loop, failure handling, counters, run logs |
| Dashboard | 7 | Web UI: decision frame with overlays, live wrist feed, state, counters, controls |
| Simulator | 0 | Fake world, camera, arm, vision, calibration, so the loop runs without hardware |

## Coordinate frames and units

| Frame | Units | Notes |
| --- | --- | --- |
| Pixel `(u, v)` | px, int | Color stream, origin top-left. Depth is aligned to color |
| Camera | mm | Optical frame (x right, y down, z forward). **Depth values are Z along the optical axis**, not ray length |
| Flange | mm | SDK `end_link`. `ee_pose()` returns `T_base_flange` from FK |
| TCP | mm | Midpoint between the fingertips. `T_flange_tcp` = `arm.tcp_offset_mm` (config) |
| Arm base | mm | What `pick()` targets are expressed in |

- `Pose` = `np.ndarray` 4×4 float64, homogeneous transform, translation in **mm**.
- Joint angles in **radians** (raw SDK values). Durations in seconds. Time is `time.monotonic()`.
- Only the arm driver converts to SDK units (m, rad).
- `T_base_cam = T_base_flange(q at capture) · T_flange_cam`, where `T_flange_cam` is the hand-eye result (block 2).
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
- The grasp point keeps a margin from the box walls that covers the gripper **and the camera** footprint (the camera descends with the gripper).
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
    def cam_pose(self, ee_pose: Pose) -> Pose: ...  # T_base_flange → T_base_cam
    def to_arm(self, obs: Observation, point: GraspPoint) -> ArmPoint: ...
    def to_pixel(
        self, obs: Observation, p: ArmPoint
    ) -> PixelPoint | None: ...  # None if outside the image
```

- `to_arm`: deproject `(u, v, depth)` with `obs.frame.intrinsics`, then apply `obs.T_base_cam`. It raises `CalibrationError` if `T_base_cam` is `None` or depth is 0.
- The hand-eye result `T_flange_cam` is stored in `config/hand_eye.yaml`: 4×4 in mm, `rmse_mm`, `method`, `camera_serial`, `created`. It is **committed** ([D-007](decisions.md)), because it depends only on the camera mount, which travels with the arm.
- It must be computed against the same flange frame (`end_link`) that `ee_pose()` returns.

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
    def ee_pose(self) -> Pose: ...  # T_base_flange from FK of the measured joints
    def joints(self) -> tuple[float, ...]: ...
    def hold(self) -> None: ...  # thread-safe; freeze in place; see below
    def recover(
        self,
    ) -> None: ...  # leave hold: lift to safe Z, open gripper above the background, home
```

- **Blocking:** every motion method returns only once the arm is still (joint velocity below tolerance). A motion that doesn't finish within its timeout raises `ArmError`. The SDK's `move_to_traj` is non-blocking, so the driver waits for it itself.
- **`pick(target, zone)`, with `target` = the cloth surface point where the TCP should go:**
  1. Reject (`TargetRejected`, no motion) if `target` XY is outside `zones.<zone>.workspace_mm` or IK fails.
  2. Go to `target.z + approach_mm` with the top-down orientation (`arm.grasp_rpy_deg`), open the gripper.
  3. Descend linearly to `max(target.z - grasp_depth_mm, z_floor_mm)`. Z is clamped, XY is never clamped: a clamped XY means grasping a box wall.
  4. Close the gripper, lift linearly to `arm.safe_z_mm`.
- **`place_on_background` / `drop_to_bin`:** fixed joint poses (`place_bg`, `bin_<color>`), open the gripper, lift. Release from `arm.place_release_height_mm` above the background so the cloth lands crumpled, which makes the re-grasp easier.
- **Hold vs disable:** the SDK's `estop()` disables the motors, and **the arm falls**. The software stop (dashboard button, Ctrl+C) is `hold()`: freeze the joint targets at the current position. After `hold()`, every motion raises `EStopped` until `recover()`. Cutting power is the job of the hardware e-stop switch ([D-009](decisions.md)).
- Low-level `ArmDriver` (SDK wrapper) and its mock are internal to block 5. Block 2 uses the driver for FK and gravity-compensation teaching, not the state machine.

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


class Hub:
    def __init__(self, camera: Camera, on_hold: Callable[[], None]): ...
    # state machine side
    def publish_status(self, s: Status) -> None: ...
    def publish_decision(self, d: Decision) -> None: ...
    def next_command(self, timeout_s: float) -> Command | None: ...
    # dashboard side
    def status(self) -> Status: ...
    def decision(self) -> Decision | None: ...
    def live_frame(self) -> Frame | None: ...  # proxy to camera.latest()
    def send(self, cmd: Command) -> None: ...  # HOLD → on_hold() immediately; others → queue
```

- Modules log with the standard `logging` module. `HubLogHandler` turns warnings and errors into `Event`s, so no module needs a Hub reference.
- Blocks 6 and 7 depend only on the Hub, never on each other.

### Dashboard HTTP API (block 7)

| Endpoint | Content |
| --- | --- |
| `GET /` | Single page, no build step |
| `GET /api/status` | `Status` as JSON, plus `now` (`time.monotonic()`, the clock of `Event.t`) for event ages |
| `WS /ws` | The same JSON, pushed on change, at most `dashboard.status_hz` (5 Hz) |
| `GET /stream/decision.mjpg` | Last decision frame with the zone ROI (`views.<zone>.roi`), the overlay, and a caption strip (phase + `summary`) drawn server-side |
| `GET /stream/live.mjpg` | Live wrist camera (`hub.live_frame()`), at `dashboard.stream_fps` |
| `GET /snapshot/decision.jpg`, `GET /snapshot/live.jpg` | One JPEG of the same images |
| `POST /api/command` | `{"cmd": "start" \| "pause" \| "resume" \| "step" \| "stop" \| "hold" \| "reset"}` |

The decision frame is the main panel: the wrist feed moves with the arm, and overlays only match the frame they were computed on. The caption is part of the image for the same reason. Each decision is rendered and encoded once. An optional fixed scene webcam for the audience can be added by block 7 (`dashboard.scene_camera`, not implemented), with no effect on the loop.

## Threads and process

One Python process ([D-005](decisions.md)):

- camera capture thread (block 1);
- SDK control loop thread, 500 Hz, inside the arm driver (block 5);
- state machine thread (block 6);
- web server, uvicorn (block 7).

`python -m sorter run [--sim]` wires everything (block 0, `sorter/app.py`). Ctrl+C → `arm.hold()`, stop the loop, `arm.shutdown()` (rest pose, then disable).

## Wiring (block 0)

`sorter.app.build_system(cfg, sim=False) -> System` creates every component per `backends` (`--sim` forces all of them to sim). `System` (`sorter.core.system`) holds `cfg`, `camera`, `arm`, `calibration`, `box_detector`, `color_classifier`, `observer`, `hub`, and `world` (the `SimWorld`, or `None` when nothing is simulated).

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
| `sim` | 0 | Simulator world: `seed`, `items` (colors in the box), `miss_prob`, `double_prob`, `motion_s` (per path segment), `vision_s` (sim vision delay), image size, `cam_height_mm`, `item_radius_mm`, `zones.<zone>` (`center_mm`, `width_mm`, `surface_z_mm`) |
| `camera` | 1 | Device type, serial, resolution, fps, exposure / white balance |
| `views.<zone>.roi` | 1 | Pixel polygon of the zone in its look pose, excluding the gripper fingers (`rig.yaml`) |
| `calibration` | 2 | `hand_eye` (the transform, from `hand_eye.yaml`) |
| `box_detector` | 3 | Thresholds, `avoid_radius_px`, wall margin |
| `color_classifier` | 4 | `sam` (SAM3 service: `url`, `api_key`, `prompt`, `threshold`, `mask_threshold`, `timeout_s`), class thresholds (`lightness_dark`, `chroma_colored`, `lightness_light`, `confidence_margin`), `erode_px`, `min_area_px`, `max_area_frac`, `overlap_max`, re-grasp (`grasp_inset_px`, `grasp_depth_tol_mm`, `depth_window_px`). The API key goes in `local.yaml` or env `SAM3_API_KEY`, never committed |
| `arm` | 5 | SDK config path, speeds, `tcp_offset_mm`, `grasp_rpy_deg`, `safe_z_mm`, `place_release_height_mm`, gripper (`open`, `close_kp`, `empty_below`), timeouts |
| `poses` | 5 | Joint angles (rad): `rest`, `home`, `look_box`, `look_bg`, `place_bg`, `bin_light`, `bin_dark`, `bin_colored` (`rig.yaml`) |
| `zones.<zone>` | 5 | `workspace_mm` (XY polygon, arm frame), `z_floor_mm`, `grasp_depth_mm`, `approach_mm` (`rig.yaml`) |
| `state_machine` | 6 | `empty_confirmations`, `max_consecutive_failures`, `low_confidence`, `save_runs`, `runs_dir` |
| `dashboard` | 7 | `host`, `port`, `stream_fps`, `jpeg_quality`, `status_hz` |

## Recording format (block 0)

`sorter.core.io.save_observation(path, obs)` / `load_observation(path)`: one `.npz` per observation (`color`, `depth_mm`, intrinsics, `timestamp`, `zone`, `T_base_cam`, `joints`), plus a `.png` of the color image for browsing.

- **Datasets** (block 1): `data/datasets/<name>/`. Record from the real look poses so the viewpoint matches runtime.
- **Run logs** (block 6, `sorter.orchestrator.runlog`): one record per sense phase in `data/runs/<run_id>/`: `<cycle:04d>_<phase>.npz` (+ `.png`) and a `.json` with the vision result (without arrays), the resolved `pending`, `avoid`, counters, the summary, and `next_phase`. A phase repeated within a cycle gets a suffix (`0003_sense_box_2`); a re-sense of the same observation writes only a `.json` whose `observation` names the existing `.npz`. `run.json` holds the config at start and the counters and `end_reason` (`done` / `stopped`) at the end. Run logs double as test data for blocks 3 and 4.
- `data/` is gitignored. A few small fixtures for tests live in `tests/fixtures/`.

## Simulator (block 0)

`SimWorld` holds items (position in arm frame, color, height), the bins, and the zone whose look pose the arm is at. All sim components share one world. No physics, no 3D.

- `SimArm` implements `ArmController` directly: `pick` grabs the highest item near the target, misses with `sim.miss_prob`, grabs two items from the box with `sim.double_prob`; `place_on_background` / `drop_to_bin` move the gripper content. A target outside the zone view raises `TargetRejected`. `hold()` interrupts a motion and blocks motions until `recover()`. Each motion is a camera path of one or more segments (a pick: above the target, down, up), `sim.motion_s` seconds each. `start()` puts every item back in the box when all of them are in bins, so the next run has something to sort.
- `SimCamera` renders what the wrist camera sees wherever the arm is (`sorter.sim.scene`): a table rendered once (wood, a cardboard box, the gray mat, three bins), crumpled cloth sprites with shadows, the gripper fingers at the bottom of the frame (no depth there) and the item they hold, plus depth. The camera follows the arm's path smoothly, so the live feed moves between poses. At a look pose the image matches `ZoneView` exactly, which sim vision and sim calibration use.
- `SimCalibration` is a fixed linear pixel ↔ XY mapping per zone; Z comes from depth.
- Sim vision (`SimBoxDetector`, `SimColorClassifier`) reads the world directly and returns pixels, with perfect colors, after `sim.vision_s`.

Each sim component is selected independently through `backends`, so a real component can run against the rest of the sim (e.g. real vision on sim frames).

## Repo layout

| Path | Owner |
| --- | --- |
| `pyproject.toml`, `uv.lock`, `.gitignore`, tooling config | 0 |
| `src/sorter/core/` (types, protocols, errors, config loader, io, hub, observer, `HubLogHandler`, `System`) | 0 |
| `src/sorter/app.py`, `src/sorter/__main__.py` (wiring, CLI) | 0 |
| `src/sorter/sim/` | 0 |
| `src/sorter/camera/` (drivers, record tool, ROI tool) | 1 |
| `src/sorter/calibration/` (hand-eye, verification tool) | 2 |
| `src/sorter/box_detector/` | 3 |
| `src/sorter/color_classifier/` (incl. stats tool) | 4 |
| `src/sorter/arm/` (driver, mock driver, controller, pose teaching tool) | 5 |
| `src/sorter/orchestrator/` (state machine, run log) | 6 |
| `src/sorter/dashboard/` (server, renderer, `static/` page) | 7 |
| `src/sorter/<package>/config.py` | The block that owns the package |
| `tests/<package>/` | Same as the package |
| `tests/conftest.py` (shared fixtures) | 0 |
| `config/default.yaml` | Each block its own section; structure by 0 |
| `config/rig.yaml` | `views`: 1; `poses`, `zones`: 5 |
| `config/hand_eye.yaml` | 2 |
| `docs/demo.md` | 8 |

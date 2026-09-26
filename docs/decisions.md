# Decision Log

Short records of design decisions. Newest at the bottom. To reverse a decision, add a new entry that references the old one; don't edit history.

Template:

```markdown
## D-NNN: Title (YYYY-MM-DD)
**Context:** why a decision was needed.
**Decision:** what we chose.
**Consequences:** what follows from it, trade-offs accepted.
```

---

## D-001: Classic CV + depth instead of trained models (2026-09-25)

**Context:** Hackathon time budget, no dataset, and a fully controlled scene (fixed camera, lamp, uniform background).
**Decision:** Box detection and color classification use classic computer vision (reference-image differencing, depth, color statistics, thresholds), not trained ML models.
**Consequences:** No data collection or training. Thresholds are tuned on the actual demo clothes. A model may be tried only if time remains and the baseline works.

## D-002: Vision outputs pixel coordinates (2026-09-25)

**Context:** The original plan had vision blocks output arm coordinates.
**Decision:** Vision modules return pixel coordinates (plus depth where relevant). Conversion to arm coordinates lives only in the calibration module.
**Consequences:** Vision can be developed and tested on recorded frames without the arm or calibration. There is one place to fix when the calibration changes.

## D-003: Block 0, contracts and stubs first (2026-09-25)

**Context:** The original dependency list was contradictory: blocks 3–5 depended on 1–2, yet 4, 5, 7 were supposed to start right away.
**Decision:** Add block 0, which fixes the stack, repo layout, interfaces, and stubs/mocks before parallel work starts.
**Consequences:** Blocks 3–7 run in parallel against stubs. Blocks 1–2 are needed only for real-hardware integration.

## D-004: Depth camera (2026-09-25)

**Context:** The team has an RGB-D camera.
**Decision:** Use the depth stream for grasp selection in the box (pick point and grasp depth) and for empty-box detection. Calibration may use a 3D camera→arm transform instead of a planar homography (final choice in block 2).
**Consequences:** Removes the "unknown grasp depth" and "parallax on pile height" problems of a 2D setup. Depth must be aligned to the color stream.

## D-005: Stack: Python, single process (2026-09-25)

**Context:** The arm SDK (`reBotArm_control_py`) is Python with a uv project; the dashboard can run on the same laptop.
**Decision:** Python 3.12, uv, pytest, ruff; FastAPI + a single static page for the dashboard. One process with threads (camera, SDK control loop, state machine, web server).
**Consequences:** No IPC or serialization between modules; the dashboard's hold reaches the arm by a direct call. The SDK control loop shares the GIL with vision and streaming. In `posvel` mode the motors hold their targets on their own, so jitter only affects smoothness.

## D-006: Camera on the wrist (eye-in-hand) (2026-09-25)

**Context:** D-004 assumed an overhead camera. The Seeed grasping demo for this arm is eye-in-hand and ships a hand-eye calibration script (ArUco, Tsai). An overhead stand is extra mechanics, and any bump or transport breaks its calibration.
**Decision:** Mount the RGB-D camera rigidly on the wrist behind the gripper. The arm moves to a fixed look pose per zone before each decision frame. Pixel → arm conversion is `T_base_flange(q) · T_flange_cam · deproject(u, v, depth)`. Supersedes the overhead mounting and the planar-homography option of D-004; depth use from D-004 stays.
**Consequences:** Block 2 reuses the Seeed hand-eye script instead of writing calibration from scratch; the result survives transport. No camera stand. The live feed moves with the arm, so the dashboard's main panel is the decision frame. The camera descends with the gripper, so grasp points need a wall margin that covers the camera. The mount must be rigid.

## D-007: Fixed zones; rig config committed (2026-09-25)

**Context:** The box, background area, and bins are always at the same positions; only the clothes inside the box vary. There is one physical rig.
**Decision:** Look poses, place and bin poses, zone workspaces (`config/rig.yaml`), and the hand-eye transform (`config/hand_eye.yaml`) are constants in committed config. This is the exception allowed by `AGENTS.md` for these two files.
**Consequences:** Pixel ROIs per zone are constants (look poses are repeatable). Place and drop need no vision. Recalibration is only needed if the camera mount or the zone layout changes.

## D-008: Observation-driven loop, stateless vision (2026-09-25)

**Context:** The loop has to handle missed grasps, double grasps, failed drops, and restarts after a hold without special cases for each.
**Decision:** Every cycle starts by looking at the background; the box is visited only when the background is empty. The color classifier returns all blobs; a drop is verified when the blob count decreases. The box detector is stateless: failed grasp pixels are passed in as `avoid`.
**Consequences:** Double grasps are sorted one item per cycle. After any hold or error the loop resumes from `LOOK_BG`. Vision is testable on single frames. An item dropped outside the camera's view (on the way to a bin) is not detected.

## D-009: Software stop is hold, not disable (2026-09-25)

**Context:** The SDK's `estop()` disables the motors, and the arm falls onto the table and the clothes.
**Decision:** The dashboard stop button and Ctrl+C call `hold()` (freeze joint targets at the current position). A hardware e-stop switch cuts power when needed. Shutdown moves to a `rest` pose before disabling.
**Consequences:** Stopping is safe for the hardware in normal cases. After a hold, `recover()` is needed before any motion.

## D-010: Python 3.11; the arm SDK is added by block 5 (2026-09-25)

**Context:** D-005 chose Python 3.12, but the arm SDK `reBotArm_control_py` declares `requires-python = ">=3.10,<3.12"`. It also has no `[build-system]` and keeps `config/` and `urdf/` next to the package, so it can't be a plain git dependency.
**Decision:** Python 3.11 for the whole project (supersedes the version in D-005; the rest of D-005 stays). `pyproject.toml` does not depend on the SDK yet: block 5 adds it (a clone next to the repo, installed as a path / editable dependency) together with the real arm backend.
**Consequences:** One process with the SDK stays possible. Sim runs and blocks 0–4, 6, 7 install on any laptop without Pinocchio or the USB-CAN stack.

## D-011: The arm is the B601-RS (RobStride), not the DM (2026-09-25)

**Context:** The docs named the Seeed reBot Arm B601-DM (Damiao motors) and used DM specs. The team's arm is the B601-RS: RobStride motors, RS-06 on joints 1–3 and RS-00 on joints 4–6 and the gripper.
**Decision:** Every reference to the arm means the B601-RS. Block 5 runs the SDK with `config/rebotarm_rs.yaml`; the transport is SocketCAN through a PEAK PCAN-USB (`can0`, 1 Mbit/s) on Linux, not a serial port. DM-only numbers (~767 mm reach, ±0.2 mm repeatability) are removed; the URDF gives a reach of roughly 0.6–0.7 m from the shoulder axis.
**Consequences:** D-009 and D-010 still hold (the SDK's `estop()` disables the RS motors too; the Python < 3.12 limit is the SDK's). With the tool pointing straight down the TCP only reaches z ≲ 0.14 m above the base plate (URDF, not yet verified on the hardware), which constrains the look poses and the approach height (block 5 *Notes & risks*). `rebot_b601/` (standalone RS driver, IK, simulator, MCP server, 3D twin) can seed block 5's `ArmDriver`.

## D-012: Dashboard is a static page with plain JS (2026-09-25)

**Context:** React was considered for the dashboard. The page has one data source (`Status` over a WebSocket), two MJPEG `<img>` tags, a few buttons, and an event list. Overlays are drawn server-side.
**Decision:** One static page (`index.html`, `app.js`, `style.css`) served by FastAPI, no framework, no build step, no CDN (it must work offline at the venue; the Barlow font, OFL, is vendored in `static/fonts/`). `websockets` is a dependency so uvicorn serves `/ws`; the page falls back to polling `/api/status` when the socket is down. If the page outgrows plain DOM code, Preact + htm vendored into `static/` is the next step, still without a build.
**Consequences:** No Node toolchain on the demo laptop. The HTTP API stays the same whatever the frontend becomes.

## D-013: Background items are segmented by a remote SAM3 service (2026-09-25)

**Context:** The ground rule is classic CV + depth for vision. On the background, a gray mat, classic segmentation fails on mid-gray and patterned clothes and on shadows, and one crumpled item can split into several blobs, which breaks the blob-count contract (D-008). The team runs a SAM3 segmentation service (text prompt → instance masks; FastAPI `POST /segment` behind ngrok, API key in `X-API-Key`).
**Decision:** Block 4 gets its masks from that service (`color_classifier.sam`, prompts `clothing` and `sock`). SAM3 prompts are object categories: on real photos `clothing` finds a T-shirt (score 0.95) but no socks, and `sock` finds socks (0.9+) only, so there is one request per prompt, in parallel, and overlapping masks are merged. Everything after segmentation stays classic: ROI and depth filters, de-duplication of overlapping instances, median Lab color stats, and the re-grasp point from depth. The client uses only the standard library (multipart POST, COCO RLE decoding), so it adds no dependency. If the service is unreachable or refuses the request, `classify` raises `SegmentationError` (a `SorterError`): the loop goes to `ERROR`, paused, and Reset retries. There is no silent classic-CV fallback.
**Consequences:** The demo needs network access to the service and a key in `config/local.yaml` or `SAM3_API_KEY`. Each classification is one round trip per prompt, in parallel (~0.1 s inference each; 1.3–1.8 s per frame through ngrok in tests). A missed instance leaves an item on the background that the loop never sorts, so the prompt and threshold have to be checked on the demo clothes. The box detector (block 3) still uses classic CV.

## D-014: The arm runs on `rebot_b601`, one controller for the real arm and the simulator (2026-09-26)

**Context:** Block 5 was to wrap Seeed's `reBotArm_control_py` (Pinocchio, Python < 3.12, no `[build-system]`, D-010). The repo already has `rebot_b601/`: an RS driver over `motorbridge`, IK, min-jerk trajectories, safety checks and a simulated motor backend (D-011). The simulator's arm moved a camera along straight lines, with no kinematics, so it could not show whether a pose or a pick is reachable.
**Decision:** Block 5's `ArmDriver` is `rebot_b601.arm.Arm` (`RebotDriver`; `arm.dry_run` swaps the CAN bus for its simulated motors). `sorter.arm.controller.Controller` implements `ArmController` on any `ArmDriver`: IK and path planning come from `rebot_b601` (`plan_path`), named poses from `rig.yaml`, the pick is planned in full before anything moves. The simulator runs the same controller on `SimDriver`, which plays the planned paths back with the real arm's timing (× `sim.time_scale`) and makes the gripper act on the sim world. `rebot-b601` is a path dependency; `motorbridge` is the `hardware` extra. The Seeed SDK is not used.
**Consequences:** Moving from the sim to the arm is `backends.arm: real` (first with `arm.dry_run: true`). Everything the sim loop does is kinematically reachable with the real joint limits. `rebot_b601`'s hardware backend is still unverified on the arm (its README). No collision model: joint-space moves between poses can sweep through a box wall; the layout keeps the walls low.

## D-015: The table is laid out for the arm's top-down reach (2026-09-26)

**Context:** With the gripper pointing straight down the B601-RS reaches a TCP height of only ~120 mm, at 0.20–0.35 m from the base (joint 4 at its −80° limit). The old sim table (a 130 mm box, bins 0.65 m away, a camera 400 mm up) was not reachable.
**Decision:** `sim.layout` is the table to build: a low tray (inside 240 × 180 mm, walls 60 mm), the gray mat (240 × 180 mm), three bins (180 mm, walls 150 mm); where they stand: [D-017](#d-017-everything-stands-in-front-of-the-arm-2026-09-26). Look poses point the wrist camera straight down from ~260 mm (camera 140 mm behind the TCP, 55 mm off-axis; it sees 271 × 203 mm). `python -m sorter.sim.layout --write` computes `poses` and `zones` from the layout with the arm's IK and checks that every pick on a 20 mm grid of each workspace and every move between poses plans. The lift after a pick is per zone (`zones.<zone>.lift_z_mm`: 100 mm over the box wall, 70 mm on the mat, whose far edge can't hold the gripper vertical higher).
**Consequences:** The real rig copies the layout and the camera mount, then re-teaches the poses (block 5). Only about two items fit on the mat at once. If the real camera must be tilted at the look poses, the sim renderer (straight-down views only) needs a perspective warp.

## D-016: The simulator is a MuJoCo physics scene that simulates only the hardware (2026-09-26)

**Context:** The kinematic simulator replaced the camera, calibration and vision with stand-ins that read the world directly, so `run --sim` said nothing about whether the real blocks 1–4 work, and moving to the rig meant switching five backends at once. The goal is that the rig runs the code the sim ran, without changes.
**Decision:** `--sim` simulates the hardware only: the arm and the camera. `sim.engine: physics` (default) is a MuJoCo 3 scene (`mujoco` dependency) built from the layout and the URDF: `rebot_b601.arm.Arm` runs unchanged on a `MujocoBackend` (position servos with gravity compensation for motors 1–6, a force motor with rebot's damping for the gripper), its 50 Hz loop ticks on the simulated clock; the wrist camera is rendered with D435i-like depth; items are cloth (2D flex with a crumpled rest shape). The real box detector, the real color classifier (with the render's segmentation instead of the SAM3 service, unless `sim.use_sam3`) and the real pixel → arm calibration (with the exact sim camera mount) run on it. Physics shortcuts, each a known gap to reality: cloth does not collide with cloth (~10× faster steps); a grip attaches the item pressed by both pads when a close command stalls the fingers on it (only that item: the others in the pile pass through it), and detaches on an open command (soft contacts let fabric slip far more than real friction does); the gripper's force per motor torque (`GRIPPER_N_PER_NM` = 15) is assumed, not measured. `sim.engine: kinematic` stays for the unit tests.
**Consequences:** `run --sim` exercises blocks 1–6 end to end with real timing, tracking-error faults and collisions (a joint-space move that sweeps the gripper through a bin wall faults, as it would on the rig; `drop_to_bin` goes via `home` both ways). The physics sim needs about one CPU core; with `sim.realtime: 0` a laptop sorts an item in about a minute. Grip success, cloth behavior and the gripper force need checking on the rig; the hand-eye tool can be rehearsed in the sim (`--sim` renders the board).

## D-017: Everything stands in front of the arm (2026-09-26)

**Context:** The arm is clamped to the back edge of the table, so there is no room behind it or beside its base; the D-015 layout had the box beside the base and a bin behind it. Top-down picks reach a TCP 160–360 mm from the base; drops over the bins (TCP ~245 mm up, tilted outwards) reach up to ~590 mm.
**Decision:** `sim.layout.edge_x_mm` (−70, the back of the base) is the table's back edge; the physics table starts there and `python -m sorter.sim.layout` reports anything behind it. The box stands straight ahead (center 255, 0), the mat front-left (180, 210), the bins farther out where only the tilted drop reaches: light (150, −330), dark (380, −250), colored (430, 220).
**Consequences:** Gaps are tight: 20 mm between the box and the mat, 30–60 mm around the bins. The real rig copies the layout and re-teaches the poses.

## D-018: A setup mode moves the arm by hand from the dashboard (2026-09-26)

**Context:** The poses in `rig.yaml` are computed for the sim layout; on the real rig each has to be checked through the wrist camera and re-taught, before the state machine may drive the arm.
**Decision:** `python -m sorter manual [--sim]` starts the camera, the arm and the dashboard without the state machine; `/manual` is a remote on top of the same `Controller` (named poses, a tour through them, joint jog, gripper, hold / release) with the same path checks as the loop, one motion at a time. A move to or from a bin goes via `home`. **Save current** rewrites that pose's line in `config/rig.yaml` in place, so the re-taught poses are committed like the computed ones (D-007). No Cartesian jog and no teach-by-hand (motors off): the arm falls without torque.
**Consequences:** Re-teaching needs no code or YAML editing. Running `python -m sorter.sim.layout --write` afterwards overwrites taught poses. A pose in `config/local.yaml` shadows the saved one.

## D-019: On macOS the camera pauses the system UVC driver (2026-09-26)

**Context:** Intel does not support RealSense on macOS. With the community `pyrealsense2-macosx` build, UVCAssistant (the macOS driver for USB cameras, a user process) holds the D435i's video interfaces. As root libusb captures the device, but librealsense opens and closes the interfaces several times while starting and UVCAssistant takes them back within ~30 ms, so ~9 starts in 10 failed (`failed to set power state`). Killing it does not help: launchd restarts it at once. libusb 1.0.30 behaves the same as the bundled 1.0.26.
**Decision:** As root on macOS, `RealSenseCamera.start()` sends SIGSTOP to UVCAssistant and `close()` (and an atexit hook) sends SIGCONT (`sorter/camera/macos.py`). The start is retried `camera.start_attempts` times for the remaining starts that get no frames.
**Consequences:** 4 starts in 5 work on the first try. Other external USB cameras don't work while the sorter runs; built-in Mac cameras use another service. After a hard crash UVCAssistant stays paused until `sudo killall -CONT UVCAssistant`. Linux and Windows are unaffected.

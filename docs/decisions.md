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

## D-014: ROS 2 + MoveIt track for the cloth pick-and-place task (2026-09-25) — proposed

**Context:** A node plan for a single-cloth pick-and-place task was proposed: RealSense driver, a cloth detector, MoveIt 2 `move_group` with Octomap, a task supervisor state machine and a grasp validator, started from one launch file with `execution_speed` and `place_target` arguments. It conflicts with D-005 (single Python process, no IPC) and, for the detector, with D-001 (the plan says "inference").
**Decision (proposed, not yet agreed by the team):** Build it as a separate track in `ros2_ws/` (ROS 2 Jazzy, `ament_python`) next to `src/sorter/`. `rebot_b601_moveit_config` (MoveIt + ros2_control mock hardware, built from the `rebot_b601` URDF) and `cloth_task`: the supervisor (go to a recorded look pose, detect, approach, straight-down grasp, lift, finger check, retry), `arm_bridge` (the real arm through the team's `rebot_b601` driver, which keeps its safety checks; it serves MoveIt's FollowJointTrajectory and a GripperCommand in place of ros2_control) and `cloth_detector` (RealSense frames through block 4's SAM3 classifier, imported from `src/`, same config and key). Motion uses the MoveGroup action from Python (no `moveit_py` on Jazzy apt): Pilz PTP for named poses, OMPL with orientation constraints for the approach, Pilz LIN for the descent and lift. The approach is 10 cm above the cloth with up to 20° tilt, not 15 cm at exactly 90°, because of the arm's top-down reach.
**Consequences:** Two stacks share the arm and the camera; only one can drive them at a time. The team must pick one stack for the demo, or define how they split. The detector process needs pydantic 2, which Jazzy's Python lacks: `ros2_ws/scripts/setup_deps.sh` installs it into the gitignored `ros2_ws/.pydeps`. The arm driver's `motorbridge` comes from `rebot_b601/.venv` (same Python 3.12 as Jazzy), so D-010's Python < 3.12 limit (Seeed's SDK) does not apply to this track.

## D-015: Spectacles teleop solves the full end-effector pose in a ROS-free session (2026-09-26)

**Context:** Spectacles teleop ([#27](https://github.com/anka335/Alien-Bazaar-Hackaton/issues/27)) streams an end-effector delta: a position plus an orientation quaternion, relative to the pose latched when the right clutch closed. `arm_bridge` only takes absolute joint targets. The driver's `solve_ik` leaves roll about the approach axis free and uses random restarts, which can jump to another branch between samples.
**Decision:** `cloth_task/spectacles_session.py` owns the session and is ROS-free: it is fed teleop messages and joint measurements, and returns status, joint targets and gripper commands. Its solve is a full six-degree-of-freedom damped least squares on the driver's `fk`/`jacobian`. It is seeded from the measured joints on every sample, has no restarts, and stays within the driver's soft joint limits. It converges at 1 mm and 3°, the driver's own IK tolerance; a sample that does not converge publishes nothing. There is no Cartesian clamp or servo planner, and `arm_bridge`'s joint slew and pose guard are the only limits.
**Consequences:** The fingertip path under fast motion is whatever the joint slew produces, not a straight line. Tests exercise the session without ROS, ngrok or the motor node.

## D-016: The Spectacles link watchdog runs on the bridge's receive clock inside the session (2026-09-26)

**Context:** A lost link or a second lens must stop the arm ([#30](https://github.com/anka335/Alien-Bazaar-Hackaton/issues/30)). The lens timestamp comes from another clock, and a hand still squeezing a clutch when the link comes back must not start moving the arm.
**Decision:** The session takes an injected receive clock (monotonic by default). Once a socket is accepted, 200 ms without a valid teleop times out: publishing stops, `arm` and `base` report `fault`, `fault` is `timeout`. Only valid teleops refresh the timer; the lens timestamp and seq gaps are ignored. The next valid teleop clears the fault. After a timeout or a replacement socket (arm stopped, `fault` null, `echoSeq` reset), each hand must be seen open before either commands again, and a clutch held while blocked consumes that hand's release, so tracking always restarts on a fresh rising edge. The first socket of a session is not blocked.
**Consequences:** The bridge node must call `on_connect()` on every accepted socket and `tick()` well under 200 ms, besides `status()`. The watchdog is unit-tested with a fake clock.

## D-017: A driver fault or a stale measurement holds the arm without a link fault (2026-09-26)

**Context:** The driver can fault, or its joint measurements can stop arriving, while the lens link is healthy ([#31](https://github.com/anka335/Alien-Bazaar-Hackaton/issues/31)). Solving from a stale measurement, or commanding a faulted driver, is unsafe.
**Decision:** The session is fed the driver-fault flag, the joint measurements and the result of enabling teleop mode. While the flag is true, or after 100 ms without a measurement on the receive clock, it publishes nothing and reports `arm: "holding"` with `fault` null: this is not a link fault, and the lens raises a disagreement from its own tracking state. Tracking resumes only on a right clutch pressed after the arm is healthy again. `on_connect()` refuses a socket until teleop mode is on and a measurement has arrived, so a refused enable (including an already faulted driver) never accepts a lens.
**Consequences:** The bridge node must close a socket that `on_connect()` refuses and call `tick()` well under 100 ms. The fault flag's latching (until the arm reconnects) stays in the driver; the session only mirrors it.

## D-018: Spectacles teleop is a launch switch with its own bridge node on `websockets` (2026-09-26)

**Context:** The session has to reach the arm ([#32](https://github.com/anka335/Alien-Bazaar-Hackaton/issues/32)): a WebSocket for the lens (through ngrok), `arm_bridge`'s teleop service and command topic, and the driver's fault state, without the cloth task or the leader arm fighting over the arm.
**Decision:** `spectacles:=true` in `task.launch.py` starts `spectacles_bridge` on `127.0.0.1:9100`. It needs `hardware:=real` and fails with the cloth task. The node enables `arm_bridge`'s teleop mode once and never turns it off: stops are the session not publishing, and teleop mode keeps MoveIt goals refused. `arm_bridge` publishes `/arm_bridge/driver_fault` as a latched `std_msgs/Bool`. The WebSocket side (`spectacles_link.py`) is ROS-free and uses the `websockets` library (`python3-websockets`, 10.4 on Jazzy; the API used also works on current releases) in an asyncio thread; one lock guards the session. ngrok is started by hand and its token never enters the repo. As for the task, `arm_bridge` unfolds a folded arm to `park_via_deg` at start, since MoveIt goals are refused from then on and a solve from the folded pose starts near a singularity.
**Consequences:** One new system dependency. The arm moves on its own at start with `spectacles:=true`. The link is tested with a real client and no ROS; the node and the launch wiring are not covered by tests.

## D-019: The arm runs on `rebot_b601`, one controller for the real arm and the simulator (2026-09-26)

**Context:** Block 5 was to wrap Seeed's `reBotArm_control_py` (Pinocchio, Python < 3.12, no `[build-system]`, D-010). The repo already has `rebot_b601/`: an RS driver over `motorbridge`, IK, min-jerk trajectories, safety checks and a simulated motor backend (D-011). The simulator's arm moved a camera along straight lines, with no kinematics, so it could not show whether a pose or a pick is reachable.
**Decision:** Block 5's `ArmDriver` is `rebot_b601.arm.Arm` (`RebotDriver`; `arm.dry_run` swaps the CAN bus for its simulated motors). `sorter.arm.controller.Controller` implements `ArmController` on any `ArmDriver`: IK and path planning come from `rebot_b601` (`plan_path`), named poses from `rig.yaml`, the pick is planned in full before anything moves. The simulator runs the same controller on `SimDriver`, which plays the planned paths back with the real arm's timing (× `sim.time_scale`) and makes the gripper act on the sim world. `rebot-b601` is a path dependency; `motorbridge` is the `hardware` extra. The Seeed SDK is not used.
**Consequences:** Moving from the sim to the arm is `backends.arm: real` (first with `arm.dry_run: true`). Everything the sim loop does is kinematically reachable with the real joint limits. `rebot_b601`'s hardware backend is still unverified on the arm (its README). No collision model: joint-space moves between poses can sweep through a box wall; the layout keeps the walls low.

## D-020: The table is laid out for the arm's top-down reach (2026-09-26)

**Context:** With the gripper pointing straight down the B601-RS reaches a TCP height of only ~120 mm, at 0.20–0.35 m from the base (joint 4 at its −80° limit). The old sim table (a 130 mm box, bins 0.65 m away, a camera 400 mm up) was not reachable.
**Decision:** `sim.layout` is the table to build: a low tray (inside 240 × 180 mm, walls 60 mm), the gray mat (240 × 180 mm), three bins (180 mm, walls 150 mm); where they stand: [D-022](#d-017-everything-stands-in-front-of-the-arm-2026-09-26). Look poses point the wrist camera straight down from ~260 mm (camera 140 mm behind the TCP, 55 mm off-axis; it sees 271 × 203 mm). `python -m sorter.sim.layout --write` computes `poses` and `zones` from the layout with the arm's IK and checks that every pick on a 20 mm grid of each workspace and every move between poses plans. The lift after a pick is per zone (`zones.<zone>.lift_z_mm`: 100 mm over the box wall, 70 mm on the mat, whose far edge can't hold the gripper vertical higher).
**Consequences:** The real rig copies the layout and the camera mount, then re-teaches the poses (block 5). Only about two items fit on the mat at once. If the real camera must be tilted at the look poses, the sim renderer (straight-down views only) needs a perspective warp.

## D-021: The simulator is a MuJoCo physics scene that simulates only the hardware (2026-09-26)

**Context:** The kinematic simulator replaced the camera, calibration and vision with stand-ins that read the world directly, so `run --sim` said nothing about whether the real blocks 1–4 work, and moving to the rig meant switching five backends at once. The goal is that the rig runs the code the sim ran, without changes.
**Decision:** `--sim` simulates the hardware only: the arm and the camera. `sim.engine: physics` (default) is a MuJoCo 3 scene (`mujoco` dependency) built from the layout and the URDF: `rebot_b601.arm.Arm` runs unchanged on a `MujocoBackend` (position servos with gravity compensation for motors 1–6, a force motor with rebot's damping for the gripper), its 50 Hz loop ticks on the simulated clock; the wrist camera is rendered with D435i-like depth; items are cloth (2D flex with a crumpled rest shape). The real box detector, the real color classifier (with the render's segmentation instead of the SAM3 service, unless `sim.use_sam3`) and the real pixel → arm calibration (with the exact sim camera mount) run on it. Physics shortcuts, each a known gap to reality: cloth does not collide with cloth (~10× faster steps); a grip attaches the item pressed by both pads when a close command stalls the fingers on it (only that item: the others in the pile pass through it), and detaches on an open command (soft contacts let fabric slip far more than real friction does); the gripper's force per motor torque (`GRIPPER_N_PER_NM` = 15) is assumed, not measured. `sim.engine: kinematic` stays for the unit tests.
**Consequences:** `run --sim` exercises blocks 1–6 end to end with real timing, tracking-error faults and collisions (a joint-space move that sweeps the gripper through a bin wall faults, as it would on the rig; `drop_to_bin` goes via `home` both ways). The physics sim needs about one CPU core; with `sim.realtime: 0` a laptop sorts an item in about a minute. Grip success, cloth behavior and the gripper force need checking on the rig; the hand-eye tool can be rehearsed in the sim (`--sim` renders the board).

## D-022: Everything stands in front of the arm (2026-09-26)

**Context:** The arm is clamped to the back edge of the table, so there is no room behind it or beside its base; the D-020 layout had the box beside the base and a bin behind it. Top-down picks reach a TCP 160–360 mm from the base; drops over the bins (TCP ~245 mm up, tilted outwards) reach up to ~590 mm.
**Decision:** `sim.layout.edge_x_mm` (−70, the back of the base) is the table's back edge; the physics table starts there and `python -m sorter.sim.layout` reports anything behind it. The box stands straight ahead (center 255, 0), the mat front-left (180, 210), the bins farther out where only the tilted drop reaches: light (150, −330), dark (380, −250), colored (430, 220).
**Consequences:** Gaps are tight: 20 mm between the box and the mat, 30–60 mm around the bins. The real rig copies the layout and re-teaches the poses.

## D-023: A setup mode moves the arm by hand from the dashboard (2026-09-26)

**Context:** The poses in `rig.yaml` are computed for the sim layout; on the real rig each has to be checked through the wrist camera and re-taught, before the state machine may drive the arm.
**Decision:** `python -m sorter manual [--sim]` starts the camera, the arm and the dashboard without the state machine; `/manual` is a remote on top of the same `Controller` (named poses, a tour through them, joint jog, gripper, hold / release) with the same path checks as the loop, one motion at a time. A move to or from a bin goes via `home`. **Save current** rewrites that pose's line in `config/rig.yaml` in place, so the re-taught poses are committed like the computed ones (D-007). No Cartesian jog and no teach-by-hand (motors off): the arm falls without torque.
**Consequences:** Re-teaching needs no code or YAML editing. Running `python -m sorter.sim.layout --write` afterwards overwrites taught poses. A pose in `config/local.yaml` shadows the saved one.

## D-024: On macOS the camera pauses the system UVC driver (2026-09-26)

**Context:** Intel does not support RealSense on macOS. With the community `pyrealsense2-macosx` build, UVCAssistant (the macOS driver for USB cameras, a user process) holds the D435i's video interfaces. As root libusb captures the device, but librealsense opens and closes the interfaces several times while starting and UVCAssistant takes them back within ~30 ms, so ~9 starts in 10 failed (`failed to set power state`). Killing it does not help: launchd restarts it at once. libusb 1.0.30 behaves the same as the bundled 1.0.26.
**Decision:** As root on macOS, `RealSenseCamera.start()` sends SIGSTOP to UVCAssistant and `close()` (and an atexit hook) sends SIGCONT (`sorter/camera/macos.py`). The start is retried `camera.start_attempts` times for the remaining starts that get no frames.
**Consequences:** 4 starts in 5 work on the first try. Other external USB cameras don't work while the sorter runs; built-in Mac cameras use another service. After a hard crash UVCAssistant stays paused until `sudo killall -CONT UVCAssistant`. Linux and Windows are unaffected.

## D-025: A driver fault is cleared with the torque on (2026-09-26)

**Context:** On the first real-arm run a tracking-error fault (joint6 13.6° behind its setpoint) latched in `rebot_b601`, and every later command failed with "disconnect and connect again". The setup page had no way to do that, so the only way out was restarting the process; and a disconnect cuts the torque, so the arm sags.
**Decision:** `rebot_b601.arm.Arm.clear_fault()` accepts a fault while the motors are still enabled: the setpoint becomes the measured pose, the fault is cleared, motion is allowed again. After a fault that cut the torque (135 °C, `emergency_disable`) it refuses; the driver then reconnects (the arm is already limp). The setup page shows the fault with a **Clear fault** button behind a confirmation. The fault message now has the commanded and measured angle.
**Consequences:** A fault no longer needs a restart or a sagging arm. A cause that persists faults again on the next move. The state machine (`run`) still does not clear driver faults: `recover()` fails on a faulted arm.

## D-026: The camera is calibrated from tape marks the arm points at (2026-09-26)

**Context:** The 3D view showed the camera seeing more than it does: without `hand_eye.yaml` the setup mode uses the nominal mount (140 mm behind the tip), and the look poses were computed for it. The board tool (D-006) needs a printed, measured ChArUco board and tilted views over the mat.
**Decision:** A setup-mode page, `/calibrate`: the arm points its tip at 6 spots around the mat center and the operator sticks tape under it; the marks are clicked in the live image from a few views; click + depth + the flange pose give point pairs, and a rigid fit (Kabsch) gives `T_flange_cam`, saved to `config/hand_eye.yaml`. A mark is where the tip really stopped (FK of the measured joints), since the arm sags a few mm at full reach. The page then recomputes `look_box` / `look_bg` for that mount (the lowest pose that sees the whole zone with depth) and writes them with the zone ROIs to `rig.yaml`. The overlay of marks and zone outlines on the live image shows at every step whether mount, arm and tape agree. The board tool stays as the alternative.
**Consequences:** No board to print; the result depends on the depth at the clicks (the D435i needs ~18 cm), FK accuracy and click accuracy: on the sim it finds the mount within ~2 mm, 0.4°. The marks lie on a plane, so views from different poses matter. The pattern is mirror-symmetric about the M1–M6 line (M2↔M3 and M4↔M5 look alike), and a view clicked mirrored fits as well with the camera flipped; the fit tries both per view and keeps the one with the camera above the table in every view. No prior on the camera's turn about its axis. Each view also finds the marks by itself: dark squares of the tape's size, named from the 3D distances between them (a mirrored naming would put the camera under the table), so clicking by hand is only for marks it misses. `sim.marks` draws the tape on the physics mat to rehearse it with `manual --sim`.

## D-027: The camera is on link5, not on the flange (2026-09-26)

**Context:** On the rig the calibration page never fit better than ~16 mm RMSE, though each view alone agreed with the tape to 1–2 mm. The views differ in joint 6, and the camera doesn't turn with the wrist roll: it is fixed to the link before it. With the camera on `link5` the same clicks fit to 1.9 mm and predict the marks in frames the fit didn't use to 0–6 px; with it on `link6`, 11–95 px.
**Decision:** The camera frame is `link5`. `ArmController.ee_pose()` returns `T_base_link5`, `Calibration.cam_pose()` takes it, the hand-eye result is `T_link5_cam` (config key renamed from `T_flange_cam`, so a result for the old frame fails to load). The sim mounts its camera on the `link5` body. Look and view poses no longer turn the image with joint 6; the calibration views are shifted only.
**Consequences:** Joint 6 is free for the grasp angle and moves nothing in the image. The image's turn over a zone follows from joint 1 alone, so over the mat, off to the side of the arm, it is turned against the mat and doesn't see all of it (~91% on the sim). `config/rig.yaml` look poses and ROIs were recomputed for the sim; on the rig the calibration page recomputes them. The sim's nominal mount is turned 90° about the optical axis, so the image's long side runs radially.

## D-028: The board tool fits corner reprojection with the board at its measured height (2026-09-26)

**Context:** A ChArUco board was printed for the rig: 7 × 5 squares of 45 mm, 32 mm DICT_6X6 markers (the tool expected 30 mm and 4×4). Over the mat the arm can tilt the camera only ~12°, so AX = XB (Park) from those views leaves the mount's translation off by ~17 mm on the sim, while each view's board pose is good.
**Decision:** `sorter.calibration.board` describes the printed board. `hand_eye` takes Park's result as the start of a Levenberg-Marquardt fit (numpy, central differences) of every detected corner's reprojection over `T_link5_cam` and the board's pose, with the board flat at `calibration.board_z_mm` (the board's top above the table, 16 mm on the rig): 9 unknowns instead of AX = XB's weak translation. A view needs 6 corners (the 45 mm squares are big in the look-pose view); views are no longer turned about the gripper axis (D-027). The `/calibrate` page stays as it is; its `compute_look` / `save_look` actions (`POST /api/calibrate`) recompute the look poses for the mount the tool saved.
**Consequences:** On the sim the mount comes out 0.1 mm, 0.1° from the truth (0.3 px). An error in `board_z_mm` goes into the camera's height, so it has to be measured. The tool runs as its own process: `manual` has to be stopped first.

## D-029: The box and the mat are as small as the rig camera sees whole (2026-09-26)

**Context:** On the rig the camera is ~100 mm out from the fingertips (D-027 calibration). With the gripper vertical the arm can't lift it above ~230 mm over a zone, and the image runs across the arm (~110° from radial): the look poses saw ~75% of the 240 × 180 box and ~84% of the 240 × 180 mat.
**Decision:** Shrink the zones to what the camera sees whole: the box inside 140 × 200 mm (long side along y, across the arm) at (300, 0), the mat 150 × 120 mm at (180, 210); `sim.item_radius_mm` 22; the calibration marks ±50 × ±35 mm. The sim's nominal camera image runs across the arm like the rig's. A tilted look pose was the alternative (keeps the zones, more work, gain unknown).
**Consequences:** Only small clothes fit (the pickable area in the box is ~50 × 110 mm with the 45 mm wall margin). `rig.yaml` poses and zones recomputed; on the rig the calibration page's Calibrate recomputes the look poses for the real mount. Supersedes the zone sizes of D-020.

## D-030: The arm's speed is set at runtime from the dashboard (2026-09-26)

**Context:** The demo should run faster than `arm.speed_scale` 0.5, and trying speeds meant editing the config and restarting.
**Decision:** `Controller` keeps its own `speed_scale` (starts at `arm.speed_scale`), set with `set_speed_scale()` through `Hub(speed=)` and `GET/POST /api/speed`. It applies from the next motion, clamped to [0.05, `rebot_b601`'s `MAX_SPEED_SCALE`], and isn't saved. One knob for all backends: the real arm, `dry_run`, both sims (their motions are timed from the same `speed_scale`). The hard cap stays 0.6 (`REBOT_MAX_SPEED` env raises it): joint 6 lagged ~20° at 45 °/s on the rig. Sim-only waits (`sim.vision_s`, `sim.time_scale`, the gripper's 0.5 s) stay in the config.
**Consequences:** From 0.5 the knob gives at most ×1.2 until the cap is raised after a test on the rig. The admin panel's bar has the slider (D-031).

## D-031: One admin panel in React; the operator modes switch at runtime (2026-09-26)

**Context:** The dashboard had grown into four loose pages (dashboard, manual, calibrate, 3D) with a switcher in the corner, and `run` and `manual` were separate processes: going from calibration to a run meant a restart. The plain-JS pages (D-012) were hard to keep consistent.
**Decision:** One page in React + TypeScript (Vite, `frontend/`), tabs on top switched without a reload: Auto, Manual, Calibrate (each tab also switches the operator mode, one control instead of two) and 3D view; the speed and Hold in the same bar on every tab. The build goes to `src/sorter/dashboard/web/`, not in git: building needs Node, running doesn't. three.js is an npm dependency, bundled (still no CDN); the arm's meshes stay in `rebot_b601`. One process serves the operator modes `auto` (the state machine), `manual` and `calibrate`: the mode lives in the Hub, run commands are queued in auto only, auto is left only between runs (Stop first), a setup mode is left only when no manual motion runs, HOLD works in every mode. `python -m sorter manual` is `run --mode manual`. The sim's tape marks are on the mat only in calibrate, so they don't clutter the vision in auto. Supersedes D-012 and D-023's separate process.
**Consequences:** Every checkout that serves the dashboard needs `cd frontend && npm install && npm run build` once, and again after a front-end change (the server says so when the build is missing). Without `config/hand_eye.yaml` the real rig runs on the nominal camera mount, auto included: calibrate first.

## D-032: The arm rides on a rover; two modes, load and unload, built on the simulator first (2026-09-27)

**Context:** The setup changed: the arm is mounted on a rover that drives to socks on the floor. The rover itself is built by others. The table with the mixed box, the gray mat and the bins is gone.
**Decision:** Two operator modes. **Load:** the rover stops next to a sock; the arm scans the floor, classifies the sock's color on the spot, picks it and drops it into the matching compartment of a 3-compartment cargo box on the rover, then stows and releases the rover. **Unload:** at a station the arm moves every sock from compartment *c* to laundry bin *c*; no classification there. The rover is behind a `Rover` interface with a stub; the arm moves only while the rover is stopped, the rover drives only with the arm stowed. The work is three stages ([plan.md](plan.md)): 0 preparation, then A loading and B unloading in parallel; each is accepted on a seeded sim benchmark before any hardware work. Supersedes the table layout of D-020, D-022 and D-029, and the background-first loop of D-008.
**Consequences:** New contracts (zones `FLOOR` / `CARGO` / `LAUNDRY`, `FloorDetector`, `Rover`, modes `load` / `unload`) and a new MuJoCo scene come first. Arm control, calibration, the camera, the box detector and the color logic are reused. Floor grasps in the sim are optimistic (vertices stick to the pads), so the benchmark adds misses on purpose. Rover dimensions and its real interface are open until stage 3.

## D-033: Faster arm: a speed ceiling from the motors' limit and a cruise-speed profile (2026-09-27)

**Context:** The speed slider (D-030) went only up to `rebot_b601`'s 0.6, and every move ran one min-jerk profile over the whole path, whose mean speed is the peak / 1.875. A sort cycle took ~48 s of arm motion per item at 0.6.
**Decision:** The cap is `arm.max_speed_scale` (1.4), passed to `rebot_b601.Arm`; the controller never goes above `speed_ceiling()` = `REBOT_MOTOR_VLIM` (1.5 rad/s) / the fastest joint's `JOINT_SPEED_DPS` (60 °/s) ≈ 1.43, so no joint is asked for more than the motors are programmed for. `arm.speed_scale` starts at 1.0. `rebot_b601` moves at the limiting joint's peak speed between raised-cosine ramps (acceleration rises and falls smoothly), with a peak acceleration of `JOINT_ACCEL` (4, `REBOT_JOINT_ACCEL`) × `JOINT_SPEED_DPS` per second; a short move is two ramps with a lower peak. Both sims play the same profile. Raising `REBOT_MOTOR_VLIM` for more was left out until the arm is tested.
**Consequences:** Arm motion per item on the kinematic sim: ~48 s → ~20 s at 1.4 (~30 s at 0.6). On the physics sim at 1.4 the joints stay within 2.2° of the setpoint (fault at 12°). Short moves (the pick's descent and lift) are limited by the acceleration, not the speed. Acceleration at 1.4 is higher than the arm has seen so far (0.6 min-jerk peaked at ~2.5): watch the first fast runs. joint6 peaks at 56 °/s at 1.4, above the 45 °/s where it lagged on the rig.

## D-034: Stage 0 gives A and B a thin shared base; each stage owns its sim scene (2026-09-27)

**Context:** Two agents build loading (A) and unloading (B) in parallel on the simulator. They must not edit the same files, and the rover is not measured yet.
**Decision:** Stage 0 fixes only what both need: the contracts (zones `FLOOR` / `CARGO` / `LAUNDRY`, `FloorDetector`, modes `load` / `unload`, `pick(..., yaw_rad)`, `drop_to_cargo` / `drop_to_laundry`), the shared state machine with one loop file per stage, a MuJoCo base scene (floor 200 mm below the deck, rover body, cargo box) with one scene file per stage, the arm's floor limit and keep-out boxes, and a layout tool that computes and checks the rig. The rover is not behind an interface: for our code it stands still. The kinematic sim is removed; MuJoCo is the only simulator. Geometry follows the arm's reach: joint 1 turns ±145° and the gripper held down reaches ~100 mm above the deck, so the cargo box sits on the deck to the arm's left (compartments along x, walls 50 mm) and the laundry bins are low (150 mm) and to the right. All dimensions are placeholders in `sim.layout`. Tuning the scenes, the vision, the benchmarks and the dashboard panels is left to A and B.
**Consequences:** A and B each own a scene, a loop, a detector and a panel; a change to the base, the arm or the contracts goes through AGENTS.md's contract rules. The table flow (D-008's background-first loop, the table layout of D-020 / D-022 / D-029, the kinematic sim of D-021) is gone from the code. Tests that relied on it are skipped with a note naming the stage that rewrites them.

## D-035: Navigation, the full mission and a sim-to-real check are stages of their own (2026-09-27)

**Context:** A and B assume a rover standing still. The full idea also needs the rover to find socks, stop within reach, return and dock at the station, and the sim is optimistic (exact calibration, ideal segmentation, vertices glued to the gripper).
**Decision:** Three more sim stages. **N (navigation):** a driving rover in a room scene, finding socks from afar, approach and stop within reach, docking on a station marker, room coverage; parallel to A and B. **C (full mission):** the `Rover` interface and interlock dropped from stage 0, a stow pose, the mission loop, repositioning for socks out of reach, unloading when the cargo box is full, a mission benchmark. **D (sim-to-real):** calibration error, D435i-like depth, friction grip, the real SAM3 on renders, rig-like arm lag; the benchmarks must hold up. Then the hardware stage. Out of scope: SLAM, dynamic obstacles, the battery.
**Consequences:** Order A ∥ B ∥ N → C → D → hardware. If the other team keeps the real rover's navigation, N stands in for it in the sim and only docking and the interface stay ours. N makes the rover base movable, a change to the shared base scene.

## D-036: Far detection is a stage of its own, separate from navigation (2026-09-27)

**Context:** D-035 put "finding socks from afar" inside navigation (N2). It is a vision problem of its own: a slanted view from a raised search pose, socks a few dozen pixels big at 0.5–3 m, depth unreliable past ~2 m. Stage A's floor detector only works looking straight down at arm's reach.
**Decision:** Stage F (far detection) finds socks from search poses and hands N a target list on the floor (arm frame, at the time of the frame); N turns targets into the room frame, drives there, and re-checks close up; A's floor detector does the final stop. F runs in parallel with A, B and N, on its own scene with the rover standing still; N uses the sim's ground truth as targets until F is ready.
**Consequences:** C depends on F too. F adds search poses to the shared layout tool.

## D-037: Rover navigation with RTAB-Map + Nav2 and a keepout half (2026-09-26)

**Context:** The arm, the laptop and the one RGB-D camera go on a Leo Rover, which has to drive autonomously in one fixed room; laundry search in the room comes later. There is no lidar: the only depth sensor is the D435i on the wrist (link4, D-014), with a narrow field of view (~87°). Mapping time doesn't matter (the room never changes), hackathon time does. The rover may only use the first half of the room.
**Decision:** A ROS 2 Jazzy package `ros2_ws/src/rover_nav` (block 9) built from ready-made nodes. SLAM is **RTAB-Map** (RGB-D + the rover's wheel odometry): it is made for a single RGB-D camera, does both mapping and localization on a saved database, and installs from apt. slam_toolbox was rejected: it needs a wide laser scan, and a scan faked from one D435i depth image is too narrow for reliable scan matching. The room is mapped **once by keyboard teleop**, not by autonomous exploration (m-explore is not in the Jazzy binaries and struggles with a narrow view). **Nav2** drives on the saved map, with goals from RViz for now; the forbidden half is a Nav2 **keepout filter** mask generated from the saved map by a small script. While the rover moves, the arm holds a fixed `drive` pose, so the camera is effectively rigid on the rover; its TF comes from the arm stack's `robot_state_publisher`, or from a static transform measured in the drive pose while the arm stack isn't available. The laptop rides on the rover and talks to it over the rover's Wi-Fi access point (its USB-C port is a host port); SLAM and Nav2 run on the laptop, the rover runs only its stock LeoOS driver.
**Consequences:** No extra sensor to buy or mount. Navigation and the arm stack share the camera driver and the TF tree. The arm keeps the plain names; the rover runs under LeoOS's `ROBOT_NAMESPACE=leo` (frames `leo/…`, topics `/leo/…`), and the Panthera arm in LeoOS's stock model is removed (its links clash with the reBot arm's). The rover has no internet, so the laptop serves it time (chrony). Obstacles beside the rover are invisible while it turns, so speeds stay low. Moving the arm while driving breaks localization. Waypoints (Nav2 waypoint follower) are the next step, for laundry search.

## D-038: The OAK-D on the rover's front is the default navigation camera (2026-09-26)

**Context:** D-037 used the arm's wrist D435i for SLAM, with the arm holding a `drive` pose. That ties navigation to the arm (pose from the arm team, arm must not move while driving, one camera driver shared by two stacks). A Luxonis OAK-D is available and can be fixed on the rover's front; the D435i is currently not available to the navigation work.
**Decision:** `rover_nav` takes `nav_camera:=oak|wrist`, default `oak`. The OAK-D runs with the apt `depthai_ros_driver` (depthai-ros 2.12) in the `/rover_nav` namespace (its own `robot_description` would otherwise replace the arm's), hung under `leo/base_link` by the driver's description with the offset from `config/mounts.yaml`. Stereo at 400p with extended disparity (depth from ~0.2 m instead of ~0.7 m at 800p), aligned to a 640×360 color image at 15 fps. The wrist D435i stays as `nav_camera:=wrist`, unchanged from D-037. Supersedes D-037's choice of camera; the rest of D-037 stays.
**Consequences:** Navigation works without the arm stack and without a `drive` pose; the arm and the D435i stay with the cloth task. The OAK-D's field of view is narrower (~70° vs ~87°). A map made with one camera is used with the same camera. The laptop needs a udev rule for Luxonis devices (`scripts/setup_oak.sh`).

## D-039: A MuJoCo sim of the Leo Rover, driven by the jevomir VLM (2026-09-27)

**Context:** Rover work needs the real rover, the room and the Wi-Fi link at once, and every experiment risks the hardware. We also want to try the jevomir VLM (a scoring API: Qwen3.5-4B answers closed-choice questions about an image in one forward pass) as the rover's driver before trusting it near people. Gazebo (LeoOS's own sim) needs the ROS stack and doesn't expose a simple Python loop; the project already uses MuJoCo for the arm (D-021).
**Decision:** `ros2_ws/src/rover_nav/sim/`: a separate uv project (MuJoCo, numpy, pillow; no ROS). The rover model is built from the official `leo_description` 3.2.0 (frames, masses, inertias, joints from its xacro; its COLLADA meshes converted to decimated STL by a script). It takes `cmd_vel` with the firmware's 0.5 s timeout, reports wheel + gyro odometry, and runs closed-loop moves and turns with `leo-rover-mcp`'s outcomes (done / stalled / stopped / timeout, plus tilted). jevomir sees only the rover's camera and never the map; the default `guided` policy asks it perception questions (visible? where? how far?) and fixed rules turn the answers into moves, because a 4B VLM picking actions directly is weak; `direct` asks one question whose options each map to one move, worded as what the camera shows ("In the left half of the photo"), not as moves (a prompt worded as moves made the model always drive forward). Every question is also asked with the options reversed (letter bias). An oracle scorer answers from ground truth for tests and GPU-less runs.
**Consequences:** Driving logic and jevomir can be tried and scored on seeded rooms without hardware (first real runs: `guided` 5/12, `direct` 7/12; oracle 9/12 and 10/12). Physics shortcuts: ellipsoid tires, friction 0.4 off the floor, a turn gain of 2.2 against the firmware's 1.76. Not yet connected to ROS (no `/leo/cmd_vel` bridge), so RTAB-Map / Nav2 can't run on it. A run through the real API costs ~2–7 calls per step (both orders on). The rover's path (odometry + the model's past answers) is used by the agent, not told to the model: text before the question made jevomir worse even when true (7/12 → 2/12 rooms); acting on it (`--memory control`) did not help either (6/12).

## D-040: Stage B unloads on the real rover's geometry and finds everything with the wrist camera (2026-09-27)

**Context:** The real rover (photos, 2026-09-27) has one undivided cardboard box 150 × 150 × 60 mm to the arm's left and a bit behind it, so the color is classified at unload, and three boxes of the same size on the floor in front of it. The station is never where the layout says: the rover parks within a tolerance and the bins are put down by hand.
**Decision:** `sorter.sim.scenes.unload.rover` lays `REAL_ROVER` over the committed layout and computes its rig in memory (cached in `data/unload_rig/`), with the electronics in the keep-out and a `show_held` pose; `config/rig.yaml` and the committed layout stay as they are until stage A agrees. The loop uses only the wrist camera: each bin is found once per run by matching a square ring of the bin's size to the wall points on a top-down grid (a second look, centered on the estimate, if the bin was cut by the image edge). Per sock: the sock on top of the pile (the segmentation's instances inside the box, geometry from depth), grasped at its highest point inside the workspace, finger directions tried in turn (the pick's keep-out check says which fits); a shake right after the pick lets a neighbor stuck to a finger fall back; the held sock's color comes from the `show_held` pose (side-lit: only chroma and coarse L* hold there, one class per hanging sock), else from exactly one sock gone from the box (a second look, pixels compared with the look before); socks of different classes hanging, or no color at all → everything back into the box, on top of the pile, and again; the `show_held` pose and the gripper's opening say whether anything is held; the drop goes into the found bin (the fingers below its rim), via home; a look into the bin confirms it, and only a seen drop is counted. `sorter.sim.scenes.unload.bench` judges the loop by the simulator's ground truth.
**Consequences:** Bins are found to ~1 mm and ~0.5° with the station shifted ±30 mm, ±5° and each bin ±15 mm, ±8°. A second look into the box per sock (it is also the next cycle's look). Known sim gaps show up as failures: cloth passes through cloth (colliding cloth is ~15× slower), so a pick may take the sock under the target and a pile settles anew once a sock is pulled out of it; fingers drag a neighboring sock out of the box. Benchmark (8 × 6 socks, before the held-first color): 64.6 % in the right bin, 6 in a wrong one, 11 lost.

## D-041: The sim's camera sits where the rig's hand-eye calibration put the real one (2026-09-27)

**Context:** The sim mounted the camera at a nominal `sim.camera_mount_mm` (−140, 0, 55) mm off the TCP, looking along the fingers. The rig's calibration (`config/hand_eye.yaml`, 4.7 mm RMSE) has it at (−130, +18, +99) mm, tilted 6.6°, the image turned ~180° from the sim's; the ROS 2 track's `camera_mount.yaml` (set by eye) differs from both.
**Decision:** `sim.camera_mount_T` (4 × 4 `T_link5_cam`, mm) replaces `camera_mount_mm` when set. Stage B's rover config sets it from `config/hand_eye.yaml`; the committed default stays nominal, so stage A's rig doesn't change until A moves to it too.
**Consequences:** Look poses and views on the sim match the rig's. The real camera doesn't see the fingertips (~37° off its axis): what the gripper holds is visible only where it hangs 60 mm or more below the fingers, from a pose found by a search with a collision check (`show_held`).

## D-042: Limp socks in the unload scene; the held sock's color falls back to the box's best match (2026-09-27)

**Context:** The rig camera sees nothing closer than ~55 mm below the fingers from `show_held` (the TCP is ~40° off its axis: no pose does better). The base's cloth (1e5 Pa, 4 mm) holds its crumpled shape: pinched, it stays bunched at the fingers (−31..+59 mm around the TCP), out of view. The loop then relied on exactly one sock gone from the box, but the pile settles anew (0 or 3 socks "gone"), so it put the sock back again and again. A real sock hangs ~half its length.
**Decision:** `ItemSpec` takes the cloth's `young` and `thickness_m` (the base's defaults unchanged, so stage A's scene is the same); the unload scene's socks are limp (`sim.unload.sock_young` 1e4 Pa, `sock_thickness_mm` 1): pinched in the middle a 200 × 90 mm sheet hangs ~100 mm, at an end ~150 mm. When the held sock still isn't seen, the color is the box's best match instead of a put-back: the one sock gone from the box; of several gone, the one nearest the grasp; none gone, the grasp's target. Socks of different colors seen hanging still go back.
**Consequences:** The held sock hangs in view (82–167 mm on the checked picks) and its side-lit class matched the sim's truth. The best match can be wrong (the pinched sock isn't always the target), so a sock may land in a wrong bin instead of going back into the box.

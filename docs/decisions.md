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

## D-019: Rover navigation with RTAB-Map + Nav2 and a keepout half (2026-09-26)

**Context:** The arm, the laptop and the one RGB-D camera go on a Leo Rover, which has to drive autonomously in one fixed room; laundry search in the room comes later. There is no lidar: the only depth sensor is the D435i on the wrist (link4, D-014), with a narrow field of view (~87°). Mapping time doesn't matter (the room never changes), hackathon time does. The rover may only use the first half of the room.
**Decision:** A ROS 2 Jazzy package `ros2_ws/src/rover_nav` (block 9) built from ready-made nodes. SLAM is **RTAB-Map** (RGB-D + the rover's wheel odometry): it is made for a single RGB-D camera, does both mapping and localization on a saved database, and installs from apt. slam_toolbox was rejected: it needs a wide laser scan, and a scan faked from one D435i depth image is too narrow for reliable scan matching. The room is mapped **once by keyboard teleop**, not by autonomous exploration (m-explore is not in the Jazzy binaries and struggles with a narrow view). **Nav2** drives on the saved map, with goals from RViz for now; the forbidden half is a Nav2 **keepout filter** mask generated from the saved map by a small script. While the rover moves, the arm holds a fixed `drive` pose, so the camera is effectively rigid on the rover; its TF comes from the arm stack's `robot_state_publisher`, or from a static transform measured in the drive pose while the arm stack isn't available. The laptop rides on the rover and talks to it over the rover's Wi-Fi access point (its USB-C port is a host port); SLAM and Nav2 run on the laptop, the rover runs only its stock LeoOS driver.
**Consequences:** No extra sensor to buy or mount. Navigation and the arm stack share the camera driver and the TF tree. The arm keeps the plain names; the rover runs under LeoOS's `ROBOT_NAMESPACE=leo` (frames `leo/…`, topics `/leo/…`), and the Panthera arm in LeoOS's stock model is removed (its links clash with the reBot arm's). The rover has no internet, so the laptop serves it time (chrony). Obstacles beside the rover are invisible while it turns, so speeds stay low. Moving the arm while driving breaks localization. Waypoints (Nav2 waypoint follower) are the next step, for laundry search.

## D-020: The OAK-D on the rover's front is the default navigation camera (2026-09-26)

**Context:** D-019 used the arm's wrist D435i for SLAM, with the arm holding a `drive` pose. That ties navigation to the arm (pose from the arm team, arm must not move while driving, one camera driver shared by two stacks). A Luxonis OAK-D is available and can be fixed on the rover's front; the D435i is currently not available to the navigation work.
**Decision:** `rover_nav` takes `nav_camera:=oak|wrist`, default `oak`. The OAK-D runs with the apt `depthai_ros_driver` (depthai-ros 2.12) in the `/rover_nav` namespace (its own `robot_description` would otherwise replace the arm's), hung under `leo/base_link` by the driver's description with the offset from `config/mounts.yaml`. Stereo at 400p with extended disparity (depth from ~0.2 m instead of ~0.7 m at 800p), aligned to a 640×360 color image at 15 fps. The wrist D435i stays as `nav_camera:=wrist`, unchanged from D-019. Supersedes D-019's choice of camera; the rest of D-019 stays.
**Consequences:** Navigation works without the arm stack and without a `drive` pose; the arm and the D435i stay with the cloth task. The OAK-D's field of view is narrower (~70° vs ~87°). A map made with one camera is used with the same camera. The laptop needs a udev rule for Luxonis devices (`scripts/setup_oak.sh`).

## D-021: Room search by viewpoints on the saved map, pick with the wrist camera (2026-09-26) — proposed

**Context:** The rover (block 9) has to find clothes on the floor of the allowed area, bring each to the arm and then to the washing machine, which is always at the same place. Hackathon time: the approach must be simple, and obstacle avoidance too. The segmentation (SAM3, block 4) is done. The arm reaches the floor top-down only 0.25–0.40 m ahead of its base (`rebot_b601` IK), closer than the OAK-D sees the floor (~0.4 m).
**Decision (proposed):** Block 10. Search = a fixed grid of viewpoints generated from the saved map and keepout mask, visited nearest-first; at each, 4 headings with the SAM3 detector on the OAK-D. Found clothes become `map` points; the rover parks with the cloth 0.32 m ahead of the arm's base, and the arm stack picks with its wrist camera and its own pipeline, called as a service. The machine pose is recorded once; the arm places into the drum, also as a service. Obstacle avoidance is Nav2's own (local costmap from depth, recoveries) plus "retry once, then skip". No exploration, no dynamic replanning of the search order beyond nearest-first.
**Consequences:** Every piece but the ROS node is plain Python and testable offline. Coverage depends on the viewpoint spacing and SAM3's range from a low camera (to test first). The arm stack needs floor picks (lower safety limit, no table plane) and a pick / place split with `carry`, `drive`, `floor_view` and `place_machine` poses; until then a fake arm service lets the search and machine trips be tested.

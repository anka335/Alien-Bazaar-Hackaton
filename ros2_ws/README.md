# ROS 2 track: cloth pick-and-place

**New here? Start with [TUTORIAL.md](TUTORIAL.md):** install, connect, calibrate, run, troubleshoot.

ROS 2 Jazzy + MoveIt 2 version of a single-cloth pick-and-place task ([D-014](../docs/decisions.md), proposed). Separate from the `sorter` package: it only borrows block 4's SAM3 classifier (`src/sorter/color_classifier`) for the real cloth detector.

| Package | What |
| --- | --- |
| `rebot_b601_moveit_config` | MoveIt 2 + ros2_control for the reBot B601-RS. The robot description is built at launch from `rebot_b601/…/reBot_Lite_RS_with_gripper.urdf` (symlinked): the driver's soft joint limits, box collision shapes (real meshes for the fingers), a table, the wrist camera from `config/camera_mount.yaml`, a `<ros2_control>` block (mock hardware) |
| `cloth_task` | `task_supervisor` (state machine), `arm_bridge` (real arm), `cloth_detector` (real camera + SAM3), `sim_cloth_detector` / `sim_gripper` (simulation), `record_pose` / `go_to_pose`, `leader_teleop`, `spectacles_bridge` (Spectacles teleop), `rover_standin` (a Leo Rover stand-in for tests), `task.launch.py`, `config/task.yaml`, `config/poses.yaml` |

## Status

| Step | State |
| --- | --- |
| Go to a recorded pose over the box (`mode:=fixed_view`, default) | Done |
| 1. Search & halt (`mode:=search`) | Done in simulation; kept for later |
| Detect: SAM3 on the RealSense frames, median, tf2 → `base_link` | Done; tested with the real SAM3 service on synthetic frames, not yet with the real camera |
| 2. Approach above the cloth | Done |
| 3. Trial grasp: open, straight down, close | Done |
| 4. Lift, check the fingers, retry on a miss | Done (finger position; no separate validator node). Off by default: `grasp.check_fingers: false` |
| 5. Place: carry to the target (fixed, or by SAM3 color class), release; repeat until the box is empty | Done |
| Real arm (`arm_bridge`) | Tested on the driver's simulated arm (`driver_sim:=true`), **not yet on the real arm** |
| Octomap | Not configured |

## One-time setup

```bash
sudo ros2_ws/scripts/setup_hardware.sh   # installs realsense2_camera + can-utils; brings can0 up (again after every reboot / replug)
ros2_ws/scripts/setup_deps.sh            # pydantic 2 into ros2_ws/.pydeps for the detector (no sudo)

source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

The SAM3 key goes in `config/local.yaml` (gitignored; never commit it) as `color_classifier.sam.api_key`, or in `SAM3_API_KEY`. Stop `motorbridge-gateway` / Motorbridge Studio before using the arm: they hold the CAN bus. If the script reports `couldn't set bitrate (err -32)`, the PCAN-USB adapter stalled: replug it straight into the laptop (not the hub) and run it again.

## Bring-up on the real rig, in this order

Each step is safe to stop with Ctrl+C. Keep a hand on the hardware e-stop from step 3 on.

```bash
# 1. Camera + SAM3 on a SIMULATED arm: point the camera at the box; watch /cloth_detector/debug_image in RViz
ros2 launch cloth_task task.launch.py camera:=real grasp:=false use_rviz:=true

# 2. Real arm READ-ONLY, no task (run_task:=false): motors stay limp; RViz shows the real joint
#    angles. Move the arm by hand to check the model matches; record the view pose from the encoders:
ros2 launch cloth_task task.launch.py hardware:=real camera:=real run_task:=false use_rviz:=true
ros2 run cloth_task record_pose box_view

# 3. Motors ON, stop above the cloth: check the TCP really ends 10 cm above it (camera mount!)
ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true grasp:=false execution_speed:=0.2 use_rviz:=true

# 4. The whole grasp: open, down, close, lift, check, retry
ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true execution_speed:=0.2 use_rviz:=true
```

**Ctrl+C with the motors on** parks the arm: stop, lift clear if low, `ready`, home, torque off (~45 s at `park_speed` 0.2). It ignores further Ctrl+C until done; the hardware e-stop stops it. After a driver fault it leaves the motors as they are: support the arm and cut the power.

### Box region (do this once per `box_view`)

The detector only accepts clothing inside a polygon marked on the camera image, so a sleeve of someone at the table is never a target. With the arm at `box_view` and the camera running:

```bash
ros2 run cloth_task roi_tool      # click the box corners, Enter = save to config/detector.yaml
```

Without a region the detector warns at startup and uses the whole image. Cloth points above 30 cm are rejected too (beyond top-down reach).

## Launch arguments (`task.launch.py`)

| Argument | Default | Meaning |
| --- | --- | --- |
| `hardware` | `mock` | `mock`: ros2_control mock hardware. `real`: `arm_bridge` over CAN |
| `enable_motors` | `false` | `hardware:=real`: `false` only reads the encoders (arm limp, every goal refused) |
| `driver_sim` | `false` | `hardware:=real`: the `rebot_b601` driver's simulated arm, no CAN (tests the real code path) |
| `camera` | `sim` | `sim`: `sim_cloth_detector`. `real`: `realsense2_camera` + `cloth_detector` (SAM3) |
| `camera_profile` | `640,480,15` | RealSense color/depth profile. The camera is on a USB 2 port here: keep 15 fps |
| `camera_serial` | `''` | RealSense serial with a leading underscore when there are several |
| `grasp` | `true` | `false`: stop above the cloth |
| `execution_speed` | `0.3` | MoveIt velocity scaling 0.1 … 1.0. The real arm caps at 0.6 (`arm_bridge` slows faster trajectories and warns) |
| `place_target` | `1` | 1, 2, 3 → `place_targets` in `config/task.yaml`, or `color`: light → 1, dark → 2, colored → 3 (`place.color_targets`). Anything else stops the node at startup |
| `cycles` | `1` | Cloths to pick and place; `0` = until no cloth is detected (then `ready` and `done`) |
| `mode` | `fixed_view` | `fixed_view`: go to `start_pose` and look. `search`: sweep the table (phase 1) |
| `start_pose` | `box_view` | Named pose to look at the cloth from |
| `sim_cloth_xyz` / `sim_colors` / `sim_misses` | `[0.30, 0.00, 0.03]` / `[colored]` / `1` | Simulation: virtual cloths (x, y, z per cloth, flat) and their classes; grasps that miss before one holds |
| `task_config`, `poses_file`, `repo_dir` | in `cloth_task` / the repo | Config files; `repo_dir` finds `config/`, `src/`, `rebot_b601/` |
| `use_rviz` | `false` | RViz with the planning scene and markers |
| `web` / `web_port` | `true` / `8080` | Status page at http://localhost:8080 (`status_web`): phase pipeline, counters, cloth, arm, live and detection images, log |
| `run_task` | `true` | `false`: arm, camera and MoveIt only, nothing moves by itself (record poses, `go_to_pose`) |
| `run_stack` | `true` | `false`: only the task supervisor, on a stack already started with `run_task:=false` (position the arm with the leader first, then start the task) |
| `spectacles` | `false` | `true`: Spectacles teleop, `spectacles_bridge` on 127.0.0.1:`spectacles_port`. Needs `hardware:=real` and `run_task:=false` |
| `spectacles_port` | `9100` | Port of `spectacles_bridge`'s WebSocket |
| `rover` | `false` | `true`: the lens's left clutch drives the Leo Rover on `/leo/cmd_vel`. Needs `spectacles:=true` |
| `base_max_vx` / `base_max_reverse` / `base_max_wz` | `0.20` / `0.10` / `0.6` | Rover limits: m/s forward, m/s back, rad/s. Each must be > 0 and at most protocol v1's 0.35 / 0.15 / 0.8, or the launch (and the node) refuses to start |

## Teleop with the leader arm (StarArm102 / reBot Arm 102, "Spark")

Record checkpoints (`box_view`, `bin_1`, …) by driving the robot with the leader instead of by hand. The leader's 7 FashionStar servos (ids 0–6, 1 Mbaud, `motorbridge-smart-servo`) are mapped with LeRobot's conventions (`cloth_task/leader.py`), so a leader calibrated in LeRobot works as is. The leader is streamed at 200 Hz (a read of all 7 servos takes ~4.5 ms). `arm_bridge` applies each command as it arrives, up to `teleop_max_dps` (80°/s, just under the motors' own 86°/s) on every joint, runs the driver loop at `control_hz` (100 Hz), checks every step with the driver's `pose_is_safe` (table, base), holds when commands stop, and refuses MoveIt goals while teleop is on. On the driver's simulated arm, a 70°/s leader motion lags by 40–75 ms. With `driver_sim:=true` each joint's teleop cap is also held to 90% of the simulated joint's speed (32°/s for joints 2 and 3), which the simulated arm can follow without a tracking-error fault.

```bash
sudo ros2_ws/scripts/setup_hardware.sh   # once: udev rule → /dev/rebot_leader readable by plugdev
ros2_ws/scripts/setup_deps.sh            # once: motorbridge-smart-servo into .pydeps

ros2 launch cloth_task task.launch.py hardware:=real enable_motors:=true run_task:=false use_rviz:=true
ros2 run cloth_task leader_teleop --calibrate   # once, if the leader was never calibrated: leader folded like the robot at home, gripper closed
ros2 run cloth_task leader_teleop               # Enter to engage; then: name + Enter = save the arm's pose, Enter = show, q = quit
ros2 run cloth_task leader_teleop --flip wrist_roll   # if one joint moves the wrong way
```

## Teleop with Spectacles ([#27](https://github.com/anka335/Alien-Bazaar-Hackaton/issues/27))

The robot-side peer of the lens (teleop v1): `spectacles_bridge` serves the lens on a WebSocket at `127.0.0.1:9100` (`cloth_task/spectacles_link.py`) and runs the ROS-free session `cloth_task/spectacles_session.py` ([D-015](../docs/decisions.md), [D-018](../docs/decisions.md)). Not yet tried on the real arm.

```bash
sudo apt install python3-websockets   # once (or: rosdep install --from-paths src -y)
ros2 launch cloth_task task.launch.py hardware:=real enable_motors:=true run_task:=false spectacles:=true
ngrok http 127.0.0.1:9100   # by hand, in another terminal, with your own token
```

Simulated run with the arm twin: the driver's simulated arm (no CAN) and RViz showing its measured joints (`/joint_states` from `arm_bridge`, through `robot_state_publisher`) moving under the lens commands. Start ngrok as above.

```bash
ros2 launch cloth_task task.launch.py hardware:=real driver_sim:=true enable_motors:=true run_task:=false spectacles:=true use_rviz:=true
```

- `enable_motors:=true` is needed even with the simulated arm: without it `arm_bridge` refuses teleop mode and every lens is closed with 1013.
- Stop the Lens Studio Preview before testing on the glasses: each new socket replaces the previous one (closed with 1000), so the Preview and the glasses keep replacing each other.

- `spectacles:=true` needs `hardware:=real` and `run_task:=false`; with the cloth task (or `run_stack:=false`) the launch fails. The leader arm is not started. MoveIt and the camera or simulated detector come up but don't command the arm: `arm_bridge` refuses MoveIt goals while teleop mode is on.
- With the motors on and the arm folded at home, `arm_bridge` first unfolds it to `park_via_deg` (as for the task), because MoveIt can't move it once teleop mode is on. Clear the space around the arm before launching.
- With `spectacles:=true`, `arm_bridge` runs on a single-threaded executor (`single_threaded`): the multi-threaded one published `/joint_states` up to 150 ms late while teleop commands streamed (D-040).
- At start the bridge turns `arm_bridge`'s teleop mode on and leaves it on; every stop below just stops publishing to `/arm_bridge/teleop_command`. With `enable_motors:=false`, or a driver already faulted, teleop mode is refused and every lens is refused (closed with 1013).
- `arm_bridge` publishes `/arm_bridge/driver_fault` (`std_msgs/Bool`, latched): false at start, true once the driver faults, until it reconnects. The bridge passes it to the session as the driver-fault flag.
- Each teleop frame is answered with a status, and a status is also pushed at 10 Hz so a timeout reaches the lens.

- The right clutch's rising edge latches the measured `gripper_end` pose in `base_link`. Each engaged sample targets `p_ee + position` and `orientation ⊗ q_ee`, solved for the full pose from the measured joints (damped least squares, soft joint limits, no restarts). The result goes out as joint targets plus the gripper opening, clamped to 0 (closed) through 1 (open), for `/arm_bridge/teleop_command`.
- A solve that misses 1 mm / 3° publishes nothing and `arm` stays `tracking`. Releasing the clutch stops publishing (`holding`), so `arm_bridge` holds its last setpoint.
- Mobile base, only with `rover:=true` (D-041): the bridge publishes `geometry_msgs/Twist` on `/leo/cmd_vel` (`linear.x` = `vx`, `angular.z` = `wz`, everything else 0) and takes rover presence from the arrival of `nav_msgs/Odometry` on `/leo/merged_odom` (parameters `cmd_vel_topic`, `odom_topic`). With `rover:=false` (the default) it creates neither, the left clutch is never accepted, and `base` is `idle` unless the link faults. The left clutch is accepted while rover odometry arrived within 0.5 s, the socket is open, the hands are not blocked, and the left hand was seen open since the rover was last absent. It then drives (`base: driving`) while the latest valid teleop, under 0.3 s old, has it engaged: every tick sends that `vx`, `wz` clamped to the limits (default 0.20 m/s forward, 0.10 m/s back, 0.6 rad/s), and the call where driving ends (release, 0.3 s without a valid teleop, socket closed, replacement, timeout, rover absent) sends one zero. After a 0.3 s pause the next engaged teleop drives again with no release. The tick runs at 50 Hz, so a driving base gets 50 Twists a second. Stopping the bridge (Ctrl+C or SIGTERM) publishes one more zero before the node is destroyed: the bridge turns rclpy's signal handlers off so the context is still up for it.
- Link watchdog: once a socket is accepted, 2 s without a valid teleop on the bridge's receive clock stops publishing and reports `arm` and `base` as `fault`, `fault: "timeout"`. Malformed teleops don't refresh the timer or `echoSeq`; the lens timestamp and skipped seqs don't matter. The next valid teleop clears the fault. 2 s because ngrok round trips stall for 300 ms to 1.2 s (D-040).
- After a timeout, or when a new socket replaces the old one (arm stopped, `fault` null), both hands must be seen open before either can command; a clutch held while blocked, including on the clearing frame, doesn't count. The first socket accepts the first right clutch straight away.
- Driver health: while the driver-fault flag is true, or when no joint measurement has arrived for 500 ms, publishing stops and the arm reports `holding` with `fault` null, so the lens raises a disagreement. Tracking resumes only on a right clutch pressed after the arm is healthy again.
- A socket is accepted only once teleop mode has been enabled and a joint measurement has arrived. If enabling teleop is refused (for example the driver is already faulted), no socket is accepted.

```bash
# no ROS needed; the link tests need websockets (skipped without it)
PYTHONPATH=ros2_ws/src/cloth_task python -m pytest ros2_ws/src/cloth_task/test/test_spectacles_session.py ros2_ws/src/cloth_task/test/test_spectacles_link.py
```

### Rover stand-in

`rover_standin` plays the Leo Rover for tests of the base, with no hardware. No launch file starts it. Run it only on a private ROS domain, because it publishes on `/leo/*`:

```bash
ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 run cloth_task rover_standin --ros-args -p csv_path:=/tmp/rover_standin.csv
```

- It takes `geometry_msgs/Twist` on `/leo/cmd_vel` and publishes `nav_msgs/Odometry` on `/leo/merged_odom` at 100 Hz (frame `leo/odom`, child `leo/base_footprint`; it publishes no TF), integrating a differential-drive model. It stops 0.5 s after the last Twist, like the firmware.
- It appends every received Twist to the CSV at `csv_path` (default `rover_standin.csv` in the working directory), one row per Twist and no header: receive time from `time.monotonic()` (system-wide on Linux, so comparable across processes), `linear.x`, `angular.z`.
- Kill it to make the rover absent.

## Named poses

`config/poses.yaml` holds joint poses in degrees (`home`, `ready`, `box_view`, …). `ready` unfolds the arm from the folded home pose; the task always passes through it.

```bash
ros2 run cloth_task record_pose box_view                      # from /joint_states (real encoders with hardware:=real)
ros2 run cloth_task record_pose box_view --deg 0 87.7 72.1 -74.4 0 0
ros2 run cloth_task go_to_pose --list
ros2 run cloth_task go_to_pose ready box_view --speed 0.3     # needs MoveIt (and motors on, on the real arm)
```

## Interfaces

| Topic / action / service | Type | Who |
| --- | --- | --- |
| `/cloth_detector/enable` | `std_srvs/SetBool` | supervisor → detector: on only while the arm is still at the look pose (each frame is a SAM3 request) |
| `/cloth_target_pose` | `geometry_msgs/PoseStamped`, camera optical frame, image stamp | detector → supervisor |
| `/cloth_detection_status` | `std_msgs/Bool` | detector → supervisor (search mode) |
| `/cloth_detector/color`, `/cloth_detector/debug_image` | `std_msgs/String`, `sensor_msgs/Image` | real detector: light / dark / colored; masks and grasp point |
| `/task_supervisor/phase` | `std_msgs/String`, latched | supervisor → anyone |
| `/move_action` | `moveit_msgs/action/MoveGroup` | supervisor, `go_to_pose` → move_group |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | move_group → ros2_control (mock) or `arm_bridge` (real) |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand`, position = `joint_left` in m (0 = closed) | supervisor → `sim_gripper` or `arm_bridge` |
| `/joint_states` | `sensor_msgs/JointState` | ros2_control or `arm_bridge` (50 Hz; fingers from the gripper motor angle) |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | supervisor adds the box walls (`scene.box`) |
| `/trajectory_execution_event` | `std_msgs/String` (`"stop"`) | supervisor → move_group (halt) |
| `/task_markers` | `visualization_msgs/MarkerArray` | cloth point, approach arrow → RViz |

Frames: `base_link` (arm base, table top at z = 0), `gripper_end` (fingertip centre, +X = approach axis), `camera_link` / `camera_color_optical_frame` (from `camera_mount.yaml`; the RealSense driver's own TF is off).

## The pipeline

Phases on `/task_supervisor/phase`: `init → go_to_view → detect → approach → trial_grasp → validate → holding → place`, back to `go_to_view` after a miss (up to `grasp.max_attempts`) and for the next cloth (`cycles`), then `done`; or `error`.

1. **init**: check the launch arguments, `task.yaml` and the named poses; add the box walls to the planning scene.
2. **go_to_view**: `ready`, then `start_pose` (Pilz PTP).
3. **detect**: wait until the arm is still, switch the detector on, take the median of `detect.samples` cloth poses from images taken after that, transformed to `base_link` at each image's time. Fails if none come within `detect.timeout_s` or they disagree by more than `detect.max_spread_m`.
4. **approach (phase 2)**: reject a cloth point outside `approach.workspace`; `gripper_end` to `approach.height_m` above it, tilt ≤ `max_tilt_deg` (a cone), roll within `max_roll_deg` of joint6 ≈ 0. OMPL plans around the planning scene; the tilt limit is also a path constraint when the start pose satisfies it.
5. **trial_grasp (phase 3)**: open to `grasp.open_m`; Pilz LIN straight down to `grasp.depth_m` under the cloth top (not below `grasp.floor_z_m`), straightening to vertical on the way (tilted, the side of the open fingers is lower than the tips). Far out, where vertical is out of reach, half the approach tilt, then all of it; each also with the fingers turned 90° either way (fingers against a wall); close. A straight move refused on a joint acceleration limit is retried at half and a quarter of the speed.
6. **validate (phase 4)**: LIN straight up `grasp.lift_m` (back to the approach tilt). With `grasp.check_fingers: false` (the current `task.yaml`) that's it → **holding**. With `true` it reads the finger position: ≥ `grasp.empty_below_m` → **holding**; less → missed: open, back to the look pose, detect again.
7. **place (phase 5)**: the target is `place_target`, or with `place_target:=color` the majority SAM3 class of the detection samples mapped by `place.color_targets`. An `[x, y, z]` target is planned with OMPL, no downward path constraint, gripper within `place.max_tilt_deg` of down at the target; a pose-name target is a joint move (record bins with `record_pose bin_1`, put `1: bin_1` in `place_targets`). Open, wait `place.release_wait_s`, back to the look pose for the next cloth. When the first detection of a later cycle finds nothing: `ready`, **done**.

In simulation: approach 2.5–8 s, TCP 1–2 mm from the target; a scripted miss is retried and the next grasp holds; after `max_attempts` misses it stops in `error`. Three cloths (light, dark, colored) with `place_target:=color cycles:=0`: each picked and dropped at target 1, 2, 3, then `done`; the same run through `arm_bridge` on the driver's simulated arm passes the driver's path checks. With the real `cloth_detector` + SAM3 on a synthetic T-shirt frame: detected ("colored 1.00", 0.7–1.2 s per frame) and grasped on the first attempt.

## Review status (2026-09-26)

Fixed: teleop no longer lets a step go deeper once the arm is unsafe; one bad detection frame no longer ends the run (outliers dropped, bad looks retried); search mode switches the detector on; move results time out after 180 s; Ctrl+C on the supervisor halts the current move; `place_target:=color` without a class is retried before grasping; the arm must settle before the gripper closes or releases; the default simulated cloth is in view of the recorded `box_view`; MoveIt plans with the driver's real speed cap; the real arm with the simulated camera is refused; box region + 30 cm cap; dead code removed (`gravity.py`, unused helpers, stray poses).

Open: the SRDF false collisions (folded home link2–link4, `bin_1` gripper_end–link4; two `disable_collisions` lines after checking on the real arm) and with them the `unfold_on_start` / point-target workarounds; the descent retry stack could be replaced by planning the grasp pose first; `arm_bridge` uses private members of the driver (the driver should expose them); three path-guessing mechanisms that depend on `--symlink-install`; `rgbd_camera/` duplicates the camera setup; the detection handshake (on/off service + separate pose and color topics) could be one request/response.

## Risks and notes

- **Not yet run on the real arm or with the real camera.** `arm_bridge` wraps the team's `rebot_b601` driver, whose own README says its hardware backend has not been run on a real arm. Follow the bring-up order.
- **Camera mount is a placeholder** (`rebot_b601_moveit_config/config/camera_mount.yaml`). Measure it: 1 cm off there is 1 cm off at the grasp. Step 3 of the bring-up (`grasp:=false`) is the check.
- **Camera distance.** The D435i sees depth only beyond ~17.5 cm at 640×480 (docs/tasks/01-setup-camera.md). From the placeholder `box_view` the camera is ~19 cm above the cloth: record the real `box_view` with the camera ≥ 25 cm above the cloth, with the whole box in view (the view covers ~0.26 × 0.15 m at 19 cm).
- **`grasp.empty_below_m` (2 mm) is a guess** (only used with `grasp.check_fingers: true`). Watch `fingers at … mm` in the log for a real miss and a real hold, and set it between them.
- **Top-down reach.** Vertical, the TCP reaches only z ≈ 0.12 m (x = 0.25–0.30 m); a 20° tilt allows about 0.20 m. Hence the 10 cm approach, not 15 cm. Keep the box within x ≈ 0.20–0.40 m of the base.
- **Box, `box_view` and place targets are placeholders** in `task.yaml` / `poses.yaml`: measure and record them.
- **Collision shapes are boxes** (except the fingers, link1 and link5, which use their meshes). On the real arm the folded home pose (link2–link4) and the recorded `bin_1` pose (gripper_end–link4) are flagged as self-collisions although the real parts don't touch: MoveIt won't plan from or to them. Start the task from an unfolded pose (drive the arm there with the leader, then `run_stack:=false`), and place target 1 is `bin_1`'s TCP point instead of the pose.
- **Reach.** Top-down grasps are reliable within ~0.40 m of the arm base; at 0.45 m vertical is out of reach and the descent falls back to a tilted gripper.
- **Speed on the real arm.** The driver caps joint speed at 0.6 of its maximum; `execution_speed` above that is slowed down (MoveIt allows 2× for the real arm). Start at 0.2.
- move_group logs `No default projection is set` from OMPL with a path constraint; planning still succeeds.

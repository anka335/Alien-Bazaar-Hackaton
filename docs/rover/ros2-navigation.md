# ROS 2 track: rover navigation on the real Leo Rover

**Status:** in progress · **Owner:** Slava + Claude · **Branch:** `block/09-rover-navigation`
**Owned paths:** `ros2_ws/src/rover_nav/`

Formerly block 9. The rover plan's stage [N: Navigation](n-navigation.md) drives the rover in the sorter's sim; this track drives the real rover with ROS 2 and has a standalone MuJoCo sim of it (`rover_nav/sim`, [D-039](../decisions.md)) whose model, odometry and cameras N1 can reuse.

## Goal

The Leo Rover carries the laptop and the arm, and drives autonomously inside one room on a saved SLAM map, staying out of a forbidden half of the room ([D-037](../decisions.md)). The navigation camera is an OAK-D fixed on the rover's front by default, or the arm's wrist D435i ([D-038](../decisions.md)). Searching the room for laundry comes later and builds on this.

## Scope

- [x] Rover setup: namespace `leo`, model without the Panthera arm (`scripts/setup_rover.sh`, applied).
- [x] Rover clock synced to the laptop (`scripts/setup_time_sync.sh`, applied: ~70 ms right after, converging).
- [x] ROS 2 package `ros2_ws/src/rover_nav` (`ament_python`, launch + config + small tools), built with the rest of `ros2_ws`. Usage: [rover_nav/README.md](../../ros2_ws/src/rover_nav/README.md).
- [x] **Camera option** `nav_camera:=oak|wrist` (D-038): OAK-D via `depthai_ros_driver` (`config/oak.yaml`, udev `scripts/setup_oak.sh`), or the wrist D435i.
- [x] **Mapping launch** (written, not yet run): RTAB-Map (RGB-D + the rover's wheel odometry) in mapping mode; the room is mapped **once by keyboard teleop** (`teleop_twist_keyboard` → `/cmd_vel`).
- [ ] Save the result: the RTAB-Map database (localization) and the 2D occupancy grid (`map_saver_cli` → `.pgm` + `.yaml`, the base for the keepout mask).
- [x] **Keepout mask tool** (`make_keepout`, plain Python, 11 tests; `--forbid-rect` for extra rectangles such as furniture): from the saved map and a dividing line (two points in the `map` frame) plus which side is allowed, write a Nav2 keepout filter mask (`.pgm` + `.yaml`) that marks the other side forbidden.
- [x] **Navigation launch** (written, not yet run): RTAB-Map in localization mode on the saved database + Nav2 (planner, controller, behaviors, BT navigator) + keepout filter (filter mask server + `costmap_filter_info_server`, `KeepoutFilter` in both costmaps). Goals from RViz ("Nav2 Goal").
- [ ] Nav2 footprint covers the rover **with the arm in its drive pose** and the laptop (placeholder in `config/nav2.yaml`).
- [x] OAK-D mount measured (`config/mounts.yaml`: x 0.13, z −0.035 from `leo/base_link`, ~10° down by eye; verify the tilt with the floor in RViz).
- [ ] Measure the arm mount, and the wrist camera in the drive pose for `nav_camera:=wrist` (placeholders).
- [ ] First real run: map the room, make the mask, navigate; tune.
- [ ] Rover interface (topic names, frames) in one config file. Verified on the real rover: `/leo/merged_odom`, `leo/odom` → `leo/base_footprint`, `/leo/cmd_vel`.
- [x] `rover_nav/README.md`: install, map once, make the mask, navigate. (`ros2_ws/README.md` belongs to the ROS track: a pointer there is requested, not edited.)
- [x] **MuJoCo sim** (`sim/`, plain Python + uv, no ROS, [D-039](../decisions.md)): the Leo Rover from the official `leo_description` 3.2.0 in a furnished room, `cmd_vel` / odometry / closed-loop moves like the real rover, the rover's camera and the OAK-D rendered, web UI. The jevomir VLM (scoring API) drives it from the camera alone (`guided` / `direct` policies); an oracle scorer runs the same loop without a GPU; `bench` scores seeded rooms, `tools/eval_direct.py` scores prompts on labelled frames, `--memory` uses the rover's path. 62 tests. Usage: [sim/README.md](../../ros2_ws/src/rover_nav/sim/README.md).

## The laundry boxes ([D-047](../decisions.md))

Three boxes, left → right dark / colored / light, each with a 4 cm ArUco marker. `box_detector` (`boxes.launch.py`): markers in the OAK-D's image, distance from its depth image, each box in the rover's frame on `/boxes/<label>`, the chosen box's distance on `/boxes/target_distance`. Labelled by marker id once `config/boxes.yaml` has them (the log shows every marker's dictionary + id), else by left-to-right order. `boxes.py`, 12 tests (range, order and id labelling, depth: within 1.5 % to 1 m; the marker's own size alone is 4–10 % too long). Checked end to end with synthetic OAK-D frames: the three boxes at the expected positions, the target distance right. Not yet run with the camera; the ids are still to fill in. Range ~1 m with 4 cm markers.

## Out of scope

- Waypoints / patrols (later, with laundry search; Nav2's waypoint follower).
- Autonomous exploration for mapping (the room is mapped once by hand).
- Searching for laundry and picking from the floor.
- Running the sorter (`src/sorter/`) at the same time: only one stack owns the camera and the arm at a time (D-014).

## Depends on / Unblocks

- Depends on: ROS 2 Jazzy, RTAB-Map, Nav2 and the camera driver on the laptop. With `nav_camera:=oak` (default) nothing from other blocks. With `nav_camera:=wrist`: the RealSense driver and the arm holding a **`drive` pose** (a fixed arm pose while driving; the rover plan's `stow` pose, stage 0 P4, is the candidate; until the arm stack runs, a static transform measured in that pose).
- Unblocks: laundry search in the room (future block).

## Acceptance criteria

- A saved map of the whole room; after a restart, the rover localizes on it (RTAB-Map localization mode) without re-mapping.
- From RViz goals, the rover reaches any reachable point in the allowed half without hitting anything.
- A goal in the forbidden half is refused or ends at the border; no planned path ever enters the forbidden half.
- The keepout mask tool has unit tests (line split, allowed side, map metadata copied).

## Notes & risks

- **One forward camera.** OAK-D ~70° (depth aligned to color), D435i ~87°; both give depth from ~0.2 m in the chosen modes (OAK-D: 400p + extended disparity; at 800p it would be ~0.7 m). Nothing beside or right in front of the rover is seen while turning; the saved map covers the fixed room, but new obstacles on the sides are missed. Drive slowly (Nav2 speed limits low).
- **`nav_camera:=wrist`: camera on the arm (link4).** The camera is only where TF says it is if the arm really is in the drive pose. A moved arm during navigation corrupts the map/localization. The drive pose must not change between mapping and navigation.
- **Rover namespace `leo`** (`scripts/setup_rover.sh`): the arm is the base of the naming, so the rover gives way. With LeoOS's `ROBOT_NAMESPACE=leo`, every rover frame is `leo/…` and every rover topic `/leo/…`; the arm keeps `base_link`, `/joint_states` and `/robot_description`. The script also deploys `rover/robot.urdf.xacro`: the stock model without the Panthera arm LeoOS shipped with (its `link1`–`link6` and its mount on plain `base_link` clashed with the reBot arm). Originals are backed up on the rover (`--restore`). The rover's launch passes the model as a string that ROS first tries as YAML: no `: ` in the model's comments.
- **`nav_camera:=wrist`: camera driver is shared.** `cloth_task` starts `realsense2_camera` (`/camera/camera/...`, 640×480 at 15 fps, TF off, IMU off). The navigation launch must be able to use an already-running driver instead of starting a second one.
- **Rover link is the rover's Wi-Fi** (`LeoRover-…` access point, rover at `10.0.0.1`), not USB: the blue USB-C port on the rover's top is a host port for accessories; a laptop plugged into it shows no USB device at all. The laptop's internet (SAM3, apt) goes over its other link (Ethernet / USB tethering), so both work at once. ROS discovery works out of the box (`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` on both). Only odometry, TF and `/cmd_vel` cross the Wi-Fi; camera images stay on the laptop.
- **Rover clock** (no internet on the rover) was ~2.2 days behind the laptop; TF and messages from the two machines don't line up until the clocks agree. `scripts/setup_time_sync.sh` makes the laptop an NTP server (chrony, `local stratum 10`) and points the rover's `systemd-timesyncd` at the laptop's address on the rover network (stable: networkd's DHCP server derives it from the MAC).
- **Rover (checked 2026-09-26):** LeoOS on Ubuntu 24.04, ROS 2 Jazzy. `odom_filter` (leo_filters) fuses `/leo/wheel_odom` and `/leo/imu/data` into `/leo/merged_odom` (100 Hz) and publishes the `leo/odom` → `leo/base_footprint` TF. The rover's model: `leo/base_footprint` → `leo/base_link` → `leo/camera_frame`, `leo/imu_frame`, rockers, wheels. `/leo/cmd_vel` goes straight to the firmware; `heading_controller` publishes on it only when it gets input on `/leo/heading_controller/cmd_vel`, so Nav2 publishes on `/leo/cmd_vel`. The rover also has its own RGB camera (`/leo/camera/image_raw`, no depth).
- **The arm's model adds a `table` link** under `base_link` (`rebot_b601_moveit_config`, a collision box for MoveIt). On the rover there is no table; for navigation it's only an extra frame, but MoveIt would plan around a phantom table (ROS track).
- Nav2 uses its own node list (controller, planner, behaviors, velocity smoother, BT navigator), not `nav2_bringup`'s `navigation_launch.py`, so no config is needed for docking, collision monitor or route server. Checked against the installed Jazzy packages: the default behavior tree needs only these servers (`GridBased`, `FollowPath`, Spin / BackUp / Wait), all plugins and both keepout filters load, and the RTAB-Map parameters exist in RTAB-Map 0.23.7.
- The rover's firmware stops the wheels 0.5 s after the last `/leo/cmd_vel` (`controller.input_timeout`).
- **OAK-D:** runs in `/rover_nav` (its `robot_description` would replace the arm's); needs the Luxonis udev rule; the original OAK-D may need its own 5 V supply. Tested on the laptop (2026-09-26): color + aligned depth 640×360 at ~15 Hz, identical timestamps, depth from 0.34 m, at USB 2 speed (a USB 3 port/cable is safer). The stereo resolution must be written `400P`. After a hard kill the next start can need ~40 s and a device crash before it reconnects.
- Payload and power: check the rover's payload rating against arm + laptop + camera, and how the arm is powered on the rover.
- **Sim vs. the real rover** (`sim/`): tires collide as ellipsoids, furniture has friction 0.4 (so the wheels don't climb it), and the turn gain is 2.2 against the firmware's 1.76 (MuJoCo's skid-steer slips more); a 180° turn moves the centre up to ~6 cm. Driving into a wall spins the wheels and odometry keeps counting, as on the real rover. jevomir (Qwen3.5-4B on a remote GPU, `127.0.0.1:8100` through an SSH tunnel) reaches the target in 5–7 of 12 rooms; it misses targets it doesn't recognise (the sim's laundry basket is a bare white box) and has no notion of obstacles. Any text before its question makes it worse.

## Open questions

- Where the forbidden half starts (the dividing line), decided after the map exists.

## Requests from other blocks

_None yet._

## Log

- 2026-09-26: block created (D-037): RTAB-Map + Nav2 + keepout filter, map once by teleop, camera on the wrist in a `drive` pose, laptop on the rover over USB-C. Waypoints deferred.
- 2026-09-26: rover checked over its Wi-Fi: LeoOS, Jazzy, `/merged_odom` + `odom` → `base_footprint` from `odom_filter`, `/cmd_vel` to the firmware; the USB-C port is a host port, so the link is Wi-Fi; rover clock unsynchronized.
- 2026-09-26: rover renamed to namespace `leo` (frames `leo/…`, topics `/leo/…`) and the Panthera arm removed from its model (`scripts/setup_rover.sh`); time sync script added.
- 2026-09-26: `rover_nav` package: mapping and navigation launches (RTAB-Map, Nav2 with RPP controller, keepout filter), `make_keepout` with tests, `config/mounts.yaml` (placeholders), README. Built with colcon; not yet run (RTAB-Map, Nav2, RealSense driver not installed yet).
- 2026-09-26: OAK-D on the rover's front as the default navigation camera, wrist D435i as `nav_camera:=wrist` (D-038).
- 2026-09-26: OAK-D tested on the laptop with `config/oak.yaml` (stereo resolution fixed to `400P`): 640×360 color + depth at ~15 Hz, synced, depth from 0.34 m, USB 2.
- 2026-09-26: first test on the rover (OAK-D): mapping stack up, RTAB-Map at 1 Hz (~0.1 s per update), TF `map` → … → `oak_rgb_camera_optical_frame` complete; floor plane from depth: tilt error +0.9°, roll −0.8°, height −0.1 cm, so the measured mount is right. The camera needed ~40 s and two USB errors to connect on a normal start (USB 2). After a rover reboot the clock re-synced within a minute of the laptop joining its Wi-Fi.
- 2026-09-26: final room map saved (`~/rover_nav_maps/room.*`, ~8.8 × 8.6 m). Keepout: y < −3.5 forbidden, plus the row of chairs at x = −1, y −2.5 … −1 (`--forbid-rect -1.3 -2.6 -0.7 -0.9`, new option). Bottles near (−0.5, −3.5) are in the map as obstacles; they stay in the saved map after they are removed from the room until that area is mapped again.
- 2026-09-27: MuJoCo sim of the Leo Rover (`sim/`, D-039) from the official `leo_description` 3.2.0, web UI, and the jevomir VLM driving it through its scoring API (guided / direct policies, oracle for tests). `colcon test` runs only `test/` (`setup.cfg`).
- 2026-09-27: jevomir on the real API (Brev H100 over an SSH tunnel): `guided` 5/12 rooms, `direct` 7/12 after a new prompt (asks where the target is; 82% right moves on 50 labelled frames against 20% for the first prompt). Oracle: 9/12 and 10/12. `bench` command and `tools/eval_direct.py` added.
- 2026-09-27: path memory (`leo_sim/memory.py`, `--memory`): told to jevomir before each question it made things worse (direct 7/12 → 2/12; 82% → 40–64% on the labelled frames, even with true hints); used by the agent instead (search where last seen, halve back-and-forth turns, explore after a full circle) 6/12. Default stays off.
- 2026-09-27: merged with main: brief moved from `docs/tasks/09-rover-navigation.md`, decisions renumbered D-019/D-020/D-034 → D-037/D-038/D-039.
- 2026-09-27: the laundry boxes from their 4 cm ArUco markers, distance from the OAK-D's depth (D-047): `box_detector`, `boxes.launch.py`, `config/boxes.yaml`.

# Block 9: Rover Navigation

**Status:** in progress · **Owner:** Slava + Claude · **Branch:** `block/09-rover-navigation`
**Owned paths:** `ros2_ws/src/rover_nav/`

## Goal

The Leo Rover carries the laptop and the arm, and drives autonomously inside one room on a saved SLAM map, staying out of a forbidden half of the room ([D-019](../decisions.md)). The navigation camera is an OAK-D fixed on the rover's front by default, or the arm's wrist D435i ([D-020](../decisions.md)). Searching the room for laundry comes later and builds on this.

## Scope

- [x] Rover setup: namespace `leo`, model without the Panthera arm (`scripts/setup_rover.sh`, applied).
- [x] Rover clock synced to the laptop (`scripts/setup_time_sync.sh`, applied: ~70 ms right after, converging).
- [x] ROS 2 package `ros2_ws/src/rover_nav` (`ament_python`, launch + config + small tools), built with the rest of `ros2_ws`. Usage: [rover_nav/README.md](../../ros2_ws/src/rover_nav/README.md).
- [x] **Camera option** `nav_camera:=oak|wrist` (D-020): OAK-D via `depthai_ros_driver` (`config/oak.yaml`, udev `scripts/setup_oak.sh`), or the wrist D435i.
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

## Out of scope

- Waypoints / patrols (later, with laundry search; Nav2's waypoint follower).
- Autonomous exploration for mapping (the room is mapped once by hand).
- Searching for laundry and picking from the floor.
- Running the sorter (`src/sorter/`) at the same time: only one stack owns the camera and the arm at a time (D-014).

## Depends on / Unblocks

- Depends on: ROS 2 Jazzy, RTAB-Map, Nav2 and the camera driver on the laptop. With `nav_camera:=oak` (default) nothing from other blocks. With `nav_camera:=wrist`: the RealSense driver and the arm holding a **`drive` pose** (requested from block 5; until the arm stack runs, a static transform measured in the drive pose).
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

## Open questions

- Where the forbidden half starts (the dividing line), decided after the map exists.

## Requests from other blocks

_None yet._

## Log

- 2026-09-26: block created (D-019): RTAB-Map + Nav2 + keepout filter, map once by teleop, camera on the wrist in a `drive` pose, laptop on the rover over USB-C. Waypoints deferred.
- 2026-09-26: rover checked over its Wi-Fi: LeoOS, Jazzy, `/merged_odom` + `odom` → `base_footprint` from `odom_filter`, `/cmd_vel` to the firmware; the USB-C port is a host port, so the link is Wi-Fi; rover clock unsynchronized.
- 2026-09-26: rover renamed to namespace `leo` (frames `leo/…`, topics `/leo/…`) and the Panthera arm removed from its model (`scripts/setup_rover.sh`); time sync script added.
- 2026-09-26: `rover_nav` package: mapping and navigation launches (RTAB-Map, Nav2 with RPP controller, keepout filter), `make_keepout` with tests, `config/mounts.yaml` (placeholders), README. Built with colcon; not yet run (RTAB-Map, Nav2, RealSense driver not installed yet).
- 2026-09-26: OAK-D on the rover's front as the default navigation camera, wrist D435i as `nav_camera:=wrist` (D-020).
- 2026-09-26: OAK-D tested on the laptop with `config/oak.yaml` (stereo resolution fixed to `400P`): 640×360 color + depth at ~15 Hz, synced, depth from 0.34 m, USB 2.
- 2026-09-26: first test on the rover (OAK-D): mapping stack up, RTAB-Map at 1 Hz (~0.1 s per update), TF `map` → … → `oak_rgb_camera_optical_frame` complete; floor plane from depth: tilt error +0.9°, roll −0.8°, height −0.1 cm, so the measured mount is right. The camera needed ~40 s and two USB errors to connect on a normal start (USB 2). After a rover reboot the clock re-synced within a minute of the laptop joining its Wi-Fi.
- 2026-09-26: final room map saved (`~/rover_nav_maps/room.*`, ~8.8 × 8.6 m). Keepout: y < −3.5 forbidden, plus the row of chairs at x = −1, y −2.5 … −1 (`--forbid-rect -1.3 -2.6 -0.7 -0.9`, new option). Bottles near (−0.5, −3.5) are in the map as obstacles; they stay in the saved map after they are removed from the room until that area is mapped again.

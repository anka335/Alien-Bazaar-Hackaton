# rover_nav: Leo Rover navigation in one room

ROS 2 rover navigation ([brief](../../../docs/rover/ros2-navigation.md), [D-037](../../../docs/decisions.md), [D-038](../../../docs/decisions.md)). The Leo Rover carries the laptop and the reBot arm. RTAB-Map builds a map of the room from one RGB-D camera and the rover's odometry (once, driven by keyboard); after that Nav2 drives on the saved map and stays out of a forbidden half (keepout filter). Everything runs on the laptop; the rover runs only its stock LeoOS driver.

**Status:** rover setup applied and checked; launch files and configs written and built, **not yet run** (RTAB-Map, Nav2 and the camera drivers still to install). Mount values in `config/mounts.yaml` and the Nav2 footprint are placeholders.

**Simulator:** [sim/](sim/README.md) is a MuJoCo sim of the rover (official `leo_description` model) with a web UI, where the jevomir VLM drives from the rover's camera ([D-039](../../../docs/decisions.md)). Plain Python + uv, no ROS.

## Navigation camera (`nav_camera:=`)

| | `oak` (default) | `wrist` |
| --- | --- | --- |
| Camera | Luxonis OAK-D, fixed on the rover's front | the arm's RealSense D435i, arm holding its `drive` pose |
| Needs | `ros-jazzy-depthai-ros`, `scripts/setup_oak.sh` (udev) | `ros-jazzy-realsense2-camera`; the arm in `drive` (or `camera_tf:=static`) |
| Depth range | from 0.34 m (measured), up to 3 m used (400p stereo, extended disparity) | ~0.2–3 m used |
| Field of view | ~70° (depth aligned to the color camera) | ~87° |
| Arm | free: doesn't matter what the arm does | must not move while driving |

Map and navigate **with the same camera**: the map is made of what that camera saw. With `oak`, the arm and its camera stay entirely with the cloth task (no shared driver).

## Names and frames

The arm owns the plain names (`base_link`, `/joint_states`, `/robot_description`). The rover runs under LeoOS's `ROBOT_NAMESPACE=leo`: topics `/leo/…`, frames `leo/…`. The OAK-D driver runs in `/rover_nav` so its own `robot_description` doesn't replace the arm's.

```text
map ─(RTAB-Map)→ leo/odom ─(rover odom_filter)→ leo/base_footprint → leo/base_link
   ├─(mounts.yaml: oak_mount, DepthAI driver)→ oak → oak_rgb_camera_frame → oak_rgb_camera_optical_frame
   └─(mounts.yaml: arm_mount)→ base_link (arm) → … → camera_link → camera_color_optical_frame
```

The arm mount is always published (the arm rides on the rover). With `nav_camera:=wrist`, the arm → camera part comes from the arm stack's `robot_state_publisher` (`camera_tf:=arm`) or from `camera_in_drive_pose` (`camera_tf:=static`).

| Topic | Who |
| --- | --- |
| `/leo/merged_odom` + TF `leo/odom` → `leo/base_footprint` | rover (wheel odometry + IMU), 100 Hz |
| `/leo/cmd_vel` | Nav2 (through `velocity_smoother`) or `teleop_twist_keyboard` → rover |
| `/rover_nav/oak/rgb/image_rect`, `/rover_nav/oak/stereo/image_raw`, `/rover_nav/oak/rgb/camera_info` | DepthAI driver (`oak`): color, depth aligned to color |
| `/camera/camera/color/image_raw`, `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/color/camera_info` | RealSense driver (`wrist`; `camera:=start` here, or cloth_task's with `camera:=external`) |
| `/map`, TF `map` → `leo/odom` | RTAB-Map |
| `/rover_nav/obstacle_cloud` | depth → sparse cloud for Nav2's obstacle layers |
| `/keepout_filter_mask`, `/costmap_filter_info` | keepout filter servers |

## One-time setup

```bash
sudo apt install -y ros-jazzy-rtabmap-ros ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-depthai-ros                  # OAK-D; for the wrist camera: ros-jazzy-realsense2-camera
sudo ros2_ws/src/rover_nav/scripts/setup_oak.sh    # OAK-D USB permissions, then replug it
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
```

Laptop on the rover's Wi-Fi (`LeoRover-…`, rover at `10.0.0.1`), internet over its other link. Then, once per rover:

```bash
ssh-copy-id pi@10.0.0.1                                  # key login for the scripts
src/rover_nav/scripts/setup_rover.sh                     # rover namespace leo, model without the Panthera arm
src/rover_nav/scripts/setup_time_sync.sh                 # laptop serves time to the rover (sudo on both)
```

`setup_rover.sh --restore` puts the original LeoOS config back. ROS discovery between laptop and rover needs no configuration (`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, the Jazzy default).

**Measure** and write into `config/mounts.yaml`: where the OAK-D sits on the rover (`oak_mount`: centre of the housing, and its tilt), where the arm base sits (`arm_mount`), and for the wrist camera without the arm stack, the camera in `drive` (`camera_in_drive_pose`). Put the footprint of rover + arm + laptop + camera into `config/nav2.yaml` (both costmaps).

Check the OAK-D alone first: `ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D use_rviz:=true` shows color and depth.

## 1. Map the room (once)

Maps go to `~/rover_nav_maps` (or `ROVER_NAV_MAPS`, or `maps_dir:=`).

```bash
ros2 launch rover_nav mapping.launch.py                   # OAK-D; nav_camera:=wrist for the D435i
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leo/cmd_vel   # 2nd terminal
```

In teleop, press `z` a few times first (slower: ~0.2 m/s, turns ~0.5 rad/s). Drive slowly along the walls and through the middle, and come back to places you've already seen: that's where RTAB-Map closes loops. Watch the map grow in RViz. Then, **still running**, save the 2D map (3rd terminal):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rover_nav_maps/room --ros-args -p map_subscribe_transient_local:=true
```

Ctrl+C the launch: RTAB-Map writes `~/rover_nav_maps/room.db`. Launching mapping again extends the same map; `new_map:=true` starts over.

## 2. Forbid half of the room

Pick the border in the map frame: open `room.pgm`, or in RViz use **Publish Point** on the map and read `ros2 topic echo /clicked_point`. Two points on the border, one point on the allowed side:

```bash
ros2 run rover_nav make_keepout ~/rover_nav_maps/room.yaml --line X1 Y1 X2 Y2 --keep X Y
```

Furniture the rover must not drive through (chairs: legs and gaps look passable on the map) can be forbidden too: add `--forbid-rect X1 Y1 X2 Y2` (two opposite corners, repeatable), with `--line`/`--keep` or alone. Our room:

```bash
ros2 run rover_nav make_keepout ~/rover_nav_maps/room.yaml --line 0 -3.5 1 -3.5 --keep 0 0 \
  --forbid-rect -1.3 -2.6 -0.7 -0.9      # y < -3.5 forbidden + the row of chairs at x = -1
```

It writes `~/rover_nav_maps/keepout.pgm` + `keepout.yaml` (black = forbidden) and prints the forbidden share. Look at `keepout.pgm` to check the right areas are black. Running it again overwrites the mask; restart the navigation launch to use it.

## 3. Navigate

```bash
ros2 launch rover_nav navigation.launch.py                # same nav_camera as for mapping
```

RViz: wait until the rover appears on the map (RTAB-Map has to recognise a place first; if it doesn't, drive a little with teleop where it was mapped), then **Nav2 Goal**. A goal in the forbidden half is refused. Without `keepout.yaml` the launch says so and the whole map is allowed.

Stop: Ctrl+C the launch; the rover stops 0.5 s after `/leo/cmd_vel` goes quiet (firmware `controller.input_timeout`). Keep a hand near the rover's power switch on the first runs.

## Hard-coded patrol: a 1 m square or circle ([D-046](../../../docs/decisions.md))

From where the rover stands, counter-clockwise, back to the start: a **square** (side 1 m, stops at the corners) or a **circle** (1 m across, 8 stops). At every stop, and once at the start, it turns once all the way around in 90° steps, pausing 2 s at each. On the rover's odometry, no map.

```bash
ros2 launch rover_nav patrol.launch.py                    # terminal 1: rover on; nothing moves yet
ros2 launch rover_nav patrol.launch.py shape:=circle      # or: size_m:=0.8 laps:=2 dwell_s:=3 ...
ros2 run rover_nav patrol_ctl start                       # terminal 2: go
ros2 run rover_nav patrol_ctl stop                        # any time: stop and end
ros2 run rover_nav patrol_ctl status                      # what it's doing now
```

- Try it without the rover: `patrol.launch.py fake_rover:=true`. On the real rover, `dry_run:=true` computes everything but sends no drive commands.
- `/patrol/scanning` (`std_msgs/Bool`) is true while it pauses during a look-around: the moment for a detector (not connected yet). `/patrol/phase` says what it's doing.
- Settings: `config/patrol.yaml` (shape, size, stops, laps, pause, speeds).
- Nothing avoids obstacles: keep the area clear (about 1.3 × 1.3 m incl. the rover). A look-around takes ~20 s; the square ~2.5 min, the circle ~4 min.

## Config

| File | What |
| --- | --- |
| `config/patrol.yaml` | The hard-coded patrol: shape, size, stops, laps, look-around pauses, speeds |
| `config/mounts.yaml` | OAK-D and arm on the rover; wrist camera in the drive pose. Placeholders |
| `config/oak.yaml` | DepthAI driver: color 640×360 at 15 fps, 400p stereo with extended disparity, depth aligned to color, synced, no IMU / NN |
| `config/nav2.yaml` | Nav2: speeds (0.2 m/s, 0.6 rad/s), footprint (placeholder), costmaps (obstacles from depth 5–90 cm above the floor), keepout filter |
| `rover_nav/stack.py` | Topic and frame names per camera, RealSense settings (same as cloth_task), RTAB-Map parameters |
| `rover/robot.urdf.xacro` | The rover's model, deployed by `setup_rover.sh` |

## Tests

```bash
cd ros2_ws/src/rover_nav && python3 -m pytest test -q      # keepout mask tool, no ROS needed
```

## Risks

- One camera looking forward (~70° OAK-D, ~87° D435i): the sides are blind while turning, and obstacles closer than ~20 cm aren't seen. The saved map covers the fixed room; keep people and new objects away on the first runs.
- OAK-D: on the test laptop it ran at **USB 2** speed (`USB SPEED: HIGH` in the log) and still delivered 640×360 color + depth at ~15 Hz. Prefer a USB 3 port and cable (`USB SPEED: SUPER`). The original OAK-D has a separate 5 V power input; if it resets or drops out on USB power alone, give it its own supply.
- After the driver is killed hard, the next start can take ~40 s (`Device already closed or disconnected`, once `Device crashed`) before it reconnects by itself. Stop it with Ctrl+C and wait; replug the camera if it never reconnects.
- `wrist`: the arm must not move while driving (map and localization assume a fixed camera).
- RTAB-Map needs texture: a bare wall or a dark room gives few features. Keep the room lit as when mapping.
- The rover's time is served by the laptop (`setup_time_sync.sh`); if the laptop's address on the rover network changes, run it again.

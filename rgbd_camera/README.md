# Intel RealSense D435i bring-up (ROS 2 Jazzy)

This directory contains a camera-only ROS 2 Jazzy package for an Intel
RealSense D435i. It does not contain or launch YOLO, segmentation, or any other
object-detection node.

The launch file starts the maintained `realsense2_camera` driver with:

- 640x480 RGB at 30 FPS
- 640x480 depth at 30 FPS
- depth aligned to the RGB image
- synchronized RGB and depth streams
- gyroscope and accelerometer streams
- a combined IMU stream using linear interpolation
- camera calibration, diagnostics, and TF

## Requirements

- ROS 2 Jazzy on Ubuntu 24.04, or a Jazzy/Noble Docker image
- Intel RealSense D435i connected over USB 3.x
- Git, `vcstool`, `rosdep`, and `colcon`

The pinned RealSense wrapper source is recorded in `d435i_jazzy.repos`.

## Native Ubuntu 24.04

If ROS 2 Jazzy is already installed:

```bash
cd rgbd_camera
chmod +x setup_jazzy.sh
./setup_jazzy.sh
```

If Jazzy is not installed yet, run `install_ros2_jazzy.sh` first.

## Docker

Mount this directory at `/ws` in a Jazzy desktop container and pass through
the camera devices:

```bash
docker run -it \
  --name d435i-jazzy \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v "$PWD":/ws \
  -w /ws \
  osrf/ros:jazzy-desktop \
  bash
```

Inside the container:

```bash
chmod +x setup_jazzy.sh
./setup_jazzy.sh
source install/setup.bash
ros2 launch rgbd_camera_bringup d435i.launch.py
```

When Docker runs through WSL 2, attach the camera to WSL before starting the
container:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rgbd_camera_bringup d435i.launch.py
```

To select one camera by serial number:

```bash
ros2 launch rgbd_camera_bringup d435i.launch.py serial_no:=_831612073525
```

## Published topics

Important topics include:

- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/aligned_depth_to_color/camera_info`
- `/camera/camera/imu`
- `/tf_static`
- `/diagnostics`

Topic availability depends on the connected camera and driver version. Use
`ros2 topic list` to inspect the active system.

## Preview

With `web_video_server` installed:

```bash
ros2 run web_video_server web_video_server --ros-args -p address:=0.0.0.0
```

Open `http://localhost:8080/` and select the RGB or aligned-depth topic.

## Guarded clothing-follow integration test

The separate test `../rebot_b601/manual_tests/clothing_follow.py` consumes this
launch file's color and aligned-depth topics. It performs a small lateral arm
sweep and then uses the remote SAM3 `clothing` mask to center a garment with
bounded base-frame Y steps. It defaults to camera-only preview. It is not a hand
tracking or person-safe controller; for hardware testing, place the garment on
a stand and keep every person outside the arm workspace.

Run the test inside the Jazzy container so it can import `rclpy`, `sensor_msgs`,
and `cv_bridge`. Mount the repository root at `/repo`, install the standalone
arm package into a venv that can see ROS packages, and set the SAM3 key:

```bash
apt-get update && apt-get install -y python3-venv ros-jazzy-cv-bridge
python3 -m venv --system-site-packages /tmp/rebot-follow-venv
/tmp/rebot-follow-venv/bin/pip install -e /repo/rebot_b601
source /opt/ros/jazzy/setup.bash
source /repo/rgbd_camera/install/setup.bash
export SAM3_API_KEY='<key>'
/tmp/rebot-follow-venv/bin/python \
  /repo/rebot_b601/manual_tests/clothing_follow.py
```

This first command uses the real camera and segmentation service without
creating an arm object or opening CAN. Hardware mode, its direction check, limits, and confirmation procedure are
documented in `../rebot_b601/manual_tests/README.md`.

On Windows/WSL, if the camera disappears after its startup reset, run
`usbipd list` in PowerShell and reattach the RealSense bus with
`usbipd attach --wsl --busid <BUSID>`. Export `SAM3_API_KEY` in the same
container shell that runs the test; a separate `docker exec` does not inherit
variables exported in another shell.

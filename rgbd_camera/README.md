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

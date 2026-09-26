"""roi_tool: mark the box in the camera image, so the detector ignores clothing outside it.

    (arm at box_view, camera running, e.g. the run_task:=false stack)
    ros2 run cloth_task roi_tool

Click the box corners in the window (3 or more), Enter = save to config/detector.yaml,
r = start over, q / Esc = quit without saving. Needs a display.
"""

from __future__ import annotations

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

WINDOW = "box region: click corners, Enter = save, r = reset, q = quit"


def default_config_file() -> str:
    path = os.path.join(get_package_share_directory("cloth_task"), "config", "detector.yaml")
    return os.path.realpath(path)  # symlink-install: the file in src/


def save_roi(path: str, roi: list[tuple[int, int]]) -> None:
    data = {}
    if os.path.exists(path):
        with open(path) as f:
            head = [line for line in f if line.startswith("#")]
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    else:
        head = []
    data["roi"] = [[int(u), int(v)] for u, v in roi]
    with open(path, "w") as f:
        f.writelines(head)
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)


def main(argv=None) -> int:
    topic = "/camera/camera/color/image_raw"
    path = default_config_file()
    rclpy.init()
    node = Node("roi_tool")
    bridge, latest = CvBridge(), {}
    node.create_subscription(
        Image, topic, lambda m: latest.update(img=bridge.imgmsg_to_cv2(m, "bgr8")),
        qos_profile_sensor_data,
    )  # fmt: skip
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    deadline = time.monotonic() + 10.0
    while "img" not in latest:
        if time.monotonic() > deadline:
            print(f"no image on {topic}: is the camera running?", file=sys.stderr)
            return 1
        time.sleep(0.05)
    corners: list[tuple[int, int]] = []
    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(
        WINDOW, lambda ev, u, v, *_: corners.append((u, v)) if ev == cv2.EVENT_LBUTTONDOWN else None
    )
    saved = 1
    while True:
        img = latest["img"].copy()
        if corners:
            pts = np.array(corners, np.int32)
            cv2.polylines(img, [pts], len(corners) > 2, (0, 255, 0), 2)
            for u, v in corners:
                cv2.circle(img, (u, v), 4, (0, 255, 0), -1)
        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 10):  # Enter
            if len(corners) < 3:
                print("need at least 3 corners")
                continue
            save_roi(path, corners)
            print(f"saved {len(corners)} corners → {path}")
            saved = 0
            break
        if key == ord("r"):
            corners.clear()
        if key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return saved

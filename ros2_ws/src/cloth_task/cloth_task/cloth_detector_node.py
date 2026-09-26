"""cloth_detector_node: RealSense color + aligned depth → the team's SAM3 classifier → cloth pose.

Reuses block 4 as-is (`sorter.color_classifier`: SAM3 masks, the re-grasp point = highest cloth
point inside the blob, the light/dark/colored class) with the sorter's own config loader, so the
SAM3 URL, prompts, thresholds and the API key (config/local.yaml or SAM3_API_KEY) are the same
for both stacks. The launch file puts `src/` and ros2_ws/.pydeps (pydantic 2) on PYTHONPATH.

Runs only while enabled (/cloth_detector/enable, std_srvs/SetBool; the supervisor enables it
while the arm is still), since every frame is a round trip to the SAM3 service. For each frame
taken after enabling:

  /cloth_detection_status  std_msgs/Bool
  /cloth_target_pose       geometry_msgs/PoseStamped, camera optical frame, image stamp (if found)
  /cloth_detector/color    std_msgs/String: light / dark / colored + confidence (if found)
  /cloth_detector/debug_image  sensor_msgs/Image: masks, grasp point, class
"""

from __future__ import annotations

import contextlib
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from sorter.color_classifier.classifier import Sam3ColorClassifier
from sorter.color_classifier.segmenter import SamSegmenter, SegmentationError
from sorter.core.config import load_config
from sorter.core.types import Frame, Intrinsics
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def load_roi(path: str) -> list[tuple[int, int]]:
    """The box polygon in image pixels (from roi_tool); [] = no file or no region."""
    if not path or not os.path.exists(path):
        return []
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return [(int(u), int(v)) for u, v in data.get("roi") or []]


def deproject(u: float, v: float, depth_m: float, k: tuple[float, float, float, float]):
    """Pixel + depth (Z along the optical axis) → point in the optical frame, m."""
    fx, fy, cx, cy = k
    return ((u - cx) / fx * depth_m, (v - cy) / fy * depth_m, depth_m)


class ClothDetector(Node):
    def __init__(self):
        super().__init__("cloth_detector_node")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("enabled", False)
        self.declare_parameter("detector_config", "")  # yaml with `roi`: the box in the image
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        cfg = load_config(p("config_dir"))
        roi = load_roi(p("detector_config"))
        if roi:
            self.get_logger().info(f"box region: {len(roi)}-corner polygon {roi}")
        else:
            self.get_logger().warn(
                "no box region set: clothing ANYWHERE in the image is a target, including on "
                "people. Mark the box once: ros2 run cloth_task roi_tool"
            )
        self.classifier = Sam3ColorClassifier(
            cfg.color_classifier, SamSegmenter(cfg.color_classifier.sam).segment, roi
        )
        self.get_logger().info(
            f"SAM3 at {cfg.color_classifier.sam.url}, prompts {cfg.color_classifier.sam.prompts}"
        )
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest: tuple[Image, Image] | None = None
        self._k: tuple[float, float, float, float] | None = None
        self._enabled_at: Time | None = self.get_clock().now() if p("enabled") else None
        self._seq = 0

        self.status_pub = self.create_publisher(Bool, "/cloth_detection_status", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/cloth_target_pose", 10)
        self.color_pub = self.create_publisher(String, "/cloth_detector/color", 10)
        self.debug_pub = self.create_publisher(Image, "/cloth_detector/debug_image", 1)
        self.create_subscription(
            CameraInfo, p("info_topic"), self._on_info, qos_profile_sensor_data
        )
        self._sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, p("color_topic"), qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, p("depth_topic"), qos_profile=qos_profile_sensor_data),
            ],
            queue_size=5,
            slop=0.05,
        )
        self._sync.registerCallback(self._on_frames)
        self.create_service(SetBool, "/cloth_detector/enable", self._on_enable)
        threading.Thread(target=self._worker, daemon=True).start()

    def _on_info(self, msg: CameraInfo) -> None:
        self._k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_frames(self, color: Image, depth: Image) -> None:
        with self._lock:
            self._latest = (color, depth)

    def _on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self._enabled_at = self.get_clock().now() if req.data else None
        res.success, res.message = True, "enabled" if req.data else "disabled"
        self.get_logger().info(f"detection {res.message}")
        return res

    def _worker(self) -> None:
        last_stamp = None
        while rclpy.ok():
            with self._lock:
                frames = self._latest
            enabled_at = self._enabled_at
            if enabled_at is None or frames is None or self._k is None:
                time.sleep(0.05)
                continue
            stamp = Time.from_msg(frames[0].header.stamp)
            if stamp <= enabled_at or stamp == last_stamp:  # only frames taken after enabling
                time.sleep(0.02)
                continue
            last_stamp = stamp
            try:
                self._detect(*frames)
            except SegmentationError as e:
                self.get_logger().error(f"SAM3: {e}")
                time.sleep(1.0)
            except Exception as e:  # keep the node alive; report
                self.get_logger().error(f"detection failed: {e!r}")
                time.sleep(1.0)

    def _detect(self, color_msg: Image, depth_msg: Image) -> None:
        color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.dtype != np.uint16:  # 32FC1 in metres from some drivers
            depth = np.nan_to_num(depth * 1000.0).astype(np.uint16)
        h, w = depth.shape
        fx, fy, cx, cy = self._k
        self._seq += 1
        frame = Frame(color, depth, Intrinsics(fx, fy, cx, cy, w, h), time.monotonic(), self._seq)
        t0 = time.monotonic()
        result = self.classifier.classify(frame)
        dt = time.monotonic() - t0
        if self._enabled_at is None:  # disabled while SAM3 was answering: nobody wants it
            return
        found = bool(result.items)
        self.status_pub.publish(Bool(data=found))
        if found:
            item = result.items[0]  # largest first
            u, v = item.grasp.px.u, item.grasp.px.v
            x, y, z = deproject(u, v, item.grasp.depth_mm / 1000.0, self._k)
            pose = PoseStamped()
            pose.header = color_msg.header  # optical frame + the image's capture time
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.color_pub.publish(String(data=f"{item.color.value} {item.confidence:.2f}"))
            self.get_logger().info(
                f"cloth: {len(result.items)} item(s); grasp px ({u}, {v}) depth "
                f"{item.grasp.depth_mm:.0f} mm → ({x:.3f}, {y:.3f}, {z:.3f}) m in "
                f"{color_msg.header.frame_id}; {item.color.value} {item.confidence:.2f} "
                f"[{dt:.1f} s]"
            )
        else:
            self.get_logger().info(f"no cloth in view [{dt:.1f} s]")
        self._publish_debug(color, result, color_msg.header)

    def _publish_debug(self, color: np.ndarray, result, header) -> None:
        img = color.copy()
        ov = result.overlay
        if ov.mask is not None and ov.mask.any():
            tint = img.copy()
            tint[ov.mask] = (0, 200, 255)
            img = cv2.addWeighted(tint, 0.35, img, 0.65, 0)
        for poly, _label in ov.polygons:
            if poly:
                pts = np.array([(pt.u, pt.v) for pt in poly], np.int32)
                cv2.polylines(img, [pts], True, (0, 200, 255), 2)
        for i, m in enumerate(ov.markers):
            c = (0, 255, 0) if i == 0 else (0, 160, 255)
            cv2.drawMarker(img, (m.px.u, m.px.v), c, cv2.MARKER_CROSS, 24, 2)
            cv2.putText(img, m.label, (m.px.u + 8, m.px.v - 8), 0, 0.6, c, 2)
        msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        msg.header = header
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ClothDetector()
    except Exception as e:  # missing key, bad config: one clear line
        rclpy.logging.get_logger("cloth_detector_node").fatal(f"cannot start: {e}")
        rclpy.try_shutdown()
        raise SystemExit(1) from e
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):  # Ctrl+C arrives twice under launch
            node.destroy_node()
            rclpy.try_shutdown()

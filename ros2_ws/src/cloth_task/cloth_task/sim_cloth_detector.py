"""sim_cloth_detector: stand-in for cloth_detector_node while there is no camera.

A virtual cloth lies at `cloth_xyz` (base_link). At `rate_hz` the node looks up the wrist
camera's optical frame in TF and "sees" the cloth when its centre is inside the camera's field
of view and range. It publishes the same contract as the real detector:

  /cloth_detection_status  std_msgs/Bool
  /cloth_target_pose       geometry_msgs/PoseStamped, in the camera optical frame (only when seen)

plus /task_markers (the cloth in RViz: green when seen, grey otherwise).
"""

from __future__ import annotations

import contextlib
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _quat_to_R(x, y, z, w) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def in_view(p_cam, hfov_deg: float, vfov_deg: float, min_range: float, max_range: float) -> bool:
    """Is a point in the optical frame (z forward, x right, y down) inside the view frustum?"""
    x, y, z = p_cam
    if not min_range <= z <= max_range:
        return False
    return (
        abs(math.atan2(x, z)) <= math.radians(hfov_deg) / 2
        and abs(math.atan2(y, z)) <= math.radians(vfov_deg) / 2
    )


class SimClothDetector(Node):
    def __init__(self):
        super().__init__("cloth_detector_node")
        # one or more cloths: x, y, z per cloth, flattened (base_link, m)
        self.declare_parameter("cloth_xyz", [0.38, 0.25, 0.03])  # in view from box_view
        self.declare_parameter("colors", ["colored"])  # class per cloth (cycled if shorter)
        self.declare_parameter("cloth_size", [0.20, 0.15])
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        # RealSense D435 color stream: 69° × 42°; depth works from ~0.1 m.
        self.declare_parameter("hfov_deg", 69.0)
        self.declare_parameter("vfov_deg", 42.0)
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 1.2)
        self.declare_parameter("rate_hz", 15.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        flat = [float(v) for v in p("cloth_xyz")]
        if not flat or len(flat) % 3:
            raise ValueError(f"cloth_xyz needs x, y, z per cloth, got {len(flat)} numbers")
        colors = list(p("colors")) or ["colored"]
        self.cloths = [
            (np.array(flat[i : i + 3]), colors[(i // 3) % len(colors)])
            for i in range(0, len(flat), 3)
        ]
        self.size = p("cloth_size")
        self.base, self.cam = p("base_frame"), p("camera_frame")
        self.fov = (p("hfov_deg"), p("vfov_deg"), p("min_range"), p("max_range"))
        self._lock = threading.RLock()  # _on_phase redraws markers while holding it
        self._reported: int | None = None  # index of the cloth last reported
        self._phase = ""
        self._target: int | None = None  # the cloth reported while the supervisor detected

        self.status_pub = self.create_publisher(Bool, "/cloth_detection_status", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/cloth_target_pose", 10)
        self.color_pub = self.create_publisher(String, "/cloth_detector/color", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/task_markers", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / p("rate_hz"), self._tick)
        # same service as the real detector; the sim one is cheap, so it always runs
        self.create_service(SetBool, "/cloth_detector/enable", self._on_enable)
        # a cloth carried away (supervisor in PLACE) leaves the box
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/task_supervisor/phase", self._on_phase, latched)
        self._seen: bool | None = None
        self.get_logger().info(f"sim: {len(self.cloths)} cloth(s)")

    def _on_enable(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        res.success, res.message = True, "sim detector always runs"
        return res

    def _on_phase(self, msg: String) -> None:
        with self._lock:
            self._phase = msg.data
            # the cloth being carried is the one detected from the look pose, not whatever the
            # camera happens to see from the lifted pose
            if msg.data == "place" and self._target is not None:
                xyz, color = self.cloths.pop(self._target)
                self._target = self._reported = None
                self.get_logger().info(
                    f"cloth ({xyz[0]:.2f}, {xyz[1]:.2f}) {color} picked: {len(self.cloths)} left"
                )
                self._publish_markers(None, clear=True)

    def _tick(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.cam, self.base, Time())
        except TransformException:
            return
        t, r = tf.transform.translation, tf.transform.rotation
        R = _quat_to_R(r.x, r.y, r.z, r.w)
        with self._lock:
            seen = None
            for i, (xyz, _color) in enumerate(self.cloths):
                p_cam = R @ xyz + np.array([t.x, t.y, t.z])
                if in_view(p_cam, *self.fov):
                    seen = (i, p_cam)
                    break
            self._reported = seen[0] if seen else None
            if self._phase == "detect" and seen:
                self._target = seen[0]
            color = self.cloths[seen[0]][1] if seen else None

        self.status_pub.publish(Bool(data=seen is not None))
        if seen:
            msg = PoseStamped()
            msg.header.stamp = tf.header.stamp
            msg.header.frame_id = self.cam
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, seen[1])
            msg.pose.orientation.w = 1.0
            self.pose_pub.publish(msg)
            self.color_pub.publish(String(data=f"{color} 1.00"))
        if (seen is not None) != self._seen:
            self.get_logger().info(f"cloth {'IN' if seen else 'out of'} view")
            self._seen = seen is not None
        self._publish_markers(seen[0] if seen else None)

    def _publish_markers(self, seen_index: int | None, clear: bool = False) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = []
        if clear:
            gone = Marker(ns="sim_cloth", action=Marker.DELETEALL)
            gone.header.frame_id, gone.header.stamp = self.base, stamp
            markers.append(gone)
        with self._lock:
            cloths = list(self.cloths)
        for i, (xyz, _color) in enumerate(cloths):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.base, stamp
            m.ns, m.id, m.type, m.action = "sim_cloth", i, Marker.CUBE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(self.size[0]), float(self.size[1]), 0.01
            m.color.r, m.color.g, m.color.b, m.color.a = (
                (0.1, 0.9, 0.2, 0.9) if i == seen_index else (0.6, 0.6, 0.6, 0.9)
            )
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = SimClothDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):  # Ctrl+C arrives twice under launch
            node.destroy_node()
            rclpy.try_shutdown()

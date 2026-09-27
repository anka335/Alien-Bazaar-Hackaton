"""box_detector: the laundry boxes from their ArUco markers, in the rover's frame (D-047).

Boxes left → right: dark, colored, light (rover_nav/boxes.py; config/boxes.yaml).

    in   color + aligned depth + camera_info of the OAK-D (camera: oak) or the wrist D435i (wrist)
    out  /boxes/dark, /boxes/colored, /boxes/light   geometry_msgs/PoseStamped in base_frame
         /boxes/markers                              visualization_msgs/MarkerArray (RViz)
         /boxes/target_distance                      std_msgs/Float32, only with target_box set:
                                                     that box's distance ahead of the rover's front
Logs once a second what it sees, including markers that aren't assigned to a box yet (dictionary +
id), so the ids for config/boxes.yaml can be read off the log.
"""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker as RvizMarker
from visualization_msgs.msg import MarkerArray

from rover_nav import boxes as bx

CAMERAS = {
    "oak": (
        "/rover_nav/oak/rgb/image_rect",
        "/rover_nav/oak/stereo/image_raw",
        "/rover_nav/oak/rgb/camera_info",
        True,  # rectified: no distortion
    ),
    "wrist": (
        "/camera/camera/color/image_raw",
        "/camera/camera/aligned_depth_to_color/image_raw",
        "/camera/camera/color/camera_info",
        False,
    ),
}
COLORS = {"dark": (0.1, 0.1, 0.1), "colored": (0.9, 0.4, 0.1), "light": (0.95, 0.95, 0.95)}


def _color_image(msg: Image) -> np.ndarray:
    ch = 1 if msg.encoding == "mono8" else 3
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    img = img[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
    if ch == 1:
        return img[:, :, 0]
    return img if msg.encoding == "bgr8" else img[:, :, ::-1].copy()


def _depth_m(msg: Image) -> np.ndarray:
    if msg.encoding in ("16UC1", "mono16"):
        d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)
        return d[:, : msg.width].astype(np.float64) / 1000.0
    if msg.encoding == "32FC1":
        d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)
        return d[:, : msg.width].astype(np.float64)
    raise ValueError(f"unsupported depth encoding {msg.encoding}")


class BoxDetector(Node):
    def __init__(self):
        super().__init__("box_detector")
        p = self.declare_parameter
        self.cfg = bx.BoxConfig(
            marker_size_m=p("marker_size_m", 0.04).value,
            dictionaries=tuple(p("dictionaries", list(bx.DEFAULT_DICTIONARIES)).value),
            ids={
                label: [int(i) for i in p(f"ids_{label}", [-1]).value if int(i) >= 0]
                for label in bx.CLASSES
            },
        )
        camera = p("camera", "oak").value
        self.base_frame = p("base_frame", "leo/base_link").value
        self.front_m = p("front_m", 0.27).value
        self.period = 1.0 / p("rate_hz", 5.0).value
        self.target = p("target_box", "").value
        self.use_depth = p("use_depth", True).value
        if camera not in CAMERAS:
            raise ValueError(f"camera must be one of {', '.join(CAMERAS)}")
        if self.target and self.target not in bx.CLASSES:
            raise ValueError(f"target_box must be one of {', '.join(bx.CLASSES)} or empty")
        color_t, depth_t, info_t, self.rectified = CAMERAS[camera]

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.info = None
        self.create_subscription(CameraInfo, info_t, self._on_info, qos_profile_sensor_data)
        sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Image, color_t, qos_profile=qos_profile_sensor_data),
                Subscriber(self, Image, depth_t, qos_profile=qos_profile_sensor_data),
            ],
            queue_size=5,
            slop=0.05,
        )
        sync.registerCallback(self._on_images)
        self.pubs = {c: self.create_publisher(PoseStamped, f"/boxes/{c}", 10) for c in bx.CLASSES}
        self.rviz = self.create_publisher(MarkerArray, "/boxes/markers", 10)
        self.target_pub = self.create_publisher(Float32, "/boxes/target_distance", 10)
        self._last = 0.0
        self._last_log = 0.0
        known = {k: v for k, v in self.cfg.ids.items() if v}
        self.get_logger().info(
            f"{camera} camera, {self.cfg.marker_size_m * 100:.1f} cm markers, "
            + (f"ids {known}" if known else "no ids yet: labelled by left-to-right order")
            + (f", target box: {self.target}" if self.target else "")
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_images(self, color: Image, depth: Image) -> None:
        try:
            self._process(color, depth)
        except Exception as e:  # noqa: BLE001  one bad frame must not stop the detector
            self._log_once(self._now(), f"frame skipped: {type(e).__name__}: {e}")

    def _process(self, color: Image, depth: Image) -> None:
        now = self._now()
        if self.info is None or now - self._last < self.period:
            return
        self._last = now
        K = np.array(self.info.k)
        dist = None if self.rectified else np.array(self.info.d)
        markers = bx.detect_markers(
            _color_image(color), K, dist, self.cfg.marker_size_m, self.cfg.dictionaries
        )
        if self.use_depth and markers:
            dm = _depth_m(depth)
            markers = [bx.refine_with_depth(m, dm) for m in markers]
        boxes = bx.label_boxes(markers, self.cfg)
        try:
            t = self.tf.lookup_transform(
                self.base_frame, color.header.frame_id, rclpy.time.Time()
            ).transform
        except Exception as e:  # noqa: BLE001
            self._log_once(now, f"no TF {self.base_frame} ← {color.header.frame_id}: {e}")
            return
        T_base_cam = bx.from_quaternion(
            (t.translation.x, t.translation.y, t.translation.z),
            (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
        )
        array = MarkerArray()
        summary = []
        for k, b in enumerate(boxes):
            T = T_base_cam @ b.marker.T_cam_marker
            self.pubs[b.label].publish(self._pose(T, color.header.stamp))
            array.markers.append(self._rviz(k, b.label, T, color.header.stamp))
            ahead, left = bx.ahead_and_left(T, self.front_m)
            summary.append(
                f"{b.label} ({'id' if b.by_id else 'order'} {b.marker.marker_id}): "
                f"{ahead:.2f} m ahead of the front, {left:+.2f} m left"
                + ("" if b.marker.from_depth else " [no depth]")
            )
            if b.label == self.target:
                self.target_pub.publish(Float32(data=float(ahead)))
        self.rviz.publish(array)
        labelled = {b.marker.marker_id for b in boxes}
        others = [
            f"{m.dictionary} id {m.marker_id} at {m.distance:.2f} m"
            for m in markers
            if m.marker_id not in labelled
        ]
        if summary or others:
            text = "; ".join(summary) if summary else "no box labelled"
            if others:
                text += " | not assigned to a box: " + ", ".join(others)
            self._log_once(now, text)

    def _log_once(self, now: float, text: str) -> None:
        if now - self._last_log >= 1.0:
            self._last_log = now
            self.get_logger().info(text)

    def _pose(self, T, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = stamp, self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in T[:3, 3])
        _, (roll, pitch, yaw) = bx.to_xyz_rpy(T)
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
        q = msg.pose.orientation
        q.w = float(cr * cp * cy + sr * sp * sy)
        q.x = float(sr * cp * cy - cr * sp * sy)
        q.y = float(cr * sp * cy + sr * cp * sy)
        q.z = float(cr * cp * sy - sr * sp * cy)
        return msg

    def _rviz(self, k: int, label: str, T, stamp) -> RvizMarker:
        m = RvizMarker()
        m.header.stamp, m.header.frame_id = stamp, self.base_frame
        m.ns, m.id, m.type, m.action = "boxes", k, RvizMarker.CUBE, RvizMarker.ADD
        m.pose = self._pose(T, stamp).pose
        m.scale.x = m.scale.y = 0.08
        m.scale.z = 0.01
        m.color.r, m.color.g, m.color.b = COLORS[label]
        m.color.a = 0.9
        m.lifetime.nanosec = 500_000_000
        return m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

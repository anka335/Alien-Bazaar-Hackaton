#!/usr/bin/env python3
"""Guarded RGB-D clothing-follow test for the reBot B601-RS.

The node consumes the D435i ROS 2 color and aligned-depth topics, asks the
configured SAM3 service for a ``clothing`` mask, and keeps the mask centroid
near the horizontal image center using small base-frame Y moves.  It never
moves toward the observed object; depth is a clearance guard only.

Camera/segmentation preview is the default and does not create an arm object.
Real motion requires --hardware and an exact interactive confirmation.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "rebot_b601"))

from rebot_b601.arm import Arm, ArmError  # noqa: E402


class FollowError(RuntimeError):
    """A camera, segmentation, clearance, or configuration failure."""


def _counts_from_string(value: str) -> list[int]:
    counts: list[int] = []
    pos = 0
    while pos < len(value):
        x = 0
        shift = 0
        more = True
        while more:
            c = ord(value[pos]) - 48
            x |= (c & 0x1F) << (5 * shift)
            more = bool(c & 0x20)
            pos += 1
            shift += 1
            if not more and c & 0x10:
                x |= -1 << (5 * shift)
        if len(counts) > 2:
            x += counts[-2]
        counts.append(x)
    return counts


def decode_rle(rle: dict | str) -> np.ndarray:
    """Decode a compressed or uncompressed COCO RLE mask."""
    if isinstance(rle, str):
        rle = ast.literal_eval(rle)
    height, width = rle["size"]
    counts = rle["counts"]
    if isinstance(counts, str):
        counts = _counts_from_string(counts)
    flat = np.zeros(height * width, dtype=bool)
    offset = 0
    for index, count in enumerate(counts):
        if index % 2:
            flat[offset : offset + count] = True
        offset += count
    if offset != flat.size:
        raise FollowError("SAM3 returned a malformed RLE mask")
    return flat.reshape((height, width), order="F")


def _multipart(fields: dict[str, str], png: bytes) -> tuple[bytes, str]:
    boundary = uuid.uuid4().hex
    parts: list[bytes] = []
    for name, value in fields.items():
        parts.append(
            (
                f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"'
                f"\r\n\r\n{value}\r\n"
            ).encode()
        )
    parts.append(
        (
            f'--{boundary}\r\nContent-Disposition: form-data; name="image"; '
            'filename="frame.png"\r\nContent-Type: image/png\r\n\r\n'
        ).encode()
        + png
        + b"\r\n"
    )
    parts.append(f"--{boundary}--\r\n".encode())
    return b"".join(parts), f"multipart/form-data; boundary={boundary}"


class SamClient:
    def __init__(self, url: str, api_key: str, prompt: str, threshold: float, timeout_s: float):
        if not api_key:
            raise FollowError("SAM3_API_KEY is not set")
        self.url = url.rstrip("/") + "/segment"
        self.api_key = api_key
        self.prompt = prompt
        self.threshold = threshold
        self.timeout_s = timeout_s

    def largest_mask(self, bgr: np.ndarray, min_area_px: int) -> np.ndarray:
        ok, encoded = cv2.imencode(".png", bgr)
        if not ok:
            raise FollowError("could not encode the color frame")
        body, content_type = _multipart(
            {
                "prompt": self.prompt,
                "threshold": str(self.threshold),
                "mask_threshold": "0.5",
                "output": "json",
            },
            encoded.tobytes(),
        )
        request = urllib.request.Request(
            self.url,
            data=body,
            headers={"Content-Type": content_type, "X-API-Key": self.api_key},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                payload = json.loads(response.read())
            masks = [decode_rle(item["mask_rle"]) for item in payload["instances"]]
        except (urllib.error.URLError, TimeoutError, OSError, ValueError, KeyError) as error:
            raise FollowError(f"SAM3 request failed: {error}") from None
        masks = [mask for mask in masks if int(mask.sum()) >= min_area_px]
        if not masks:
            raise FollowError(f"no '{self.prompt}' mask above {min_area_px} pixels")
        mask = max(masks, key=lambda candidate: int(candidate.sum()))
        if mask.shape != bgr.shape[:2]:
            raise FollowError(f"mask shape {mask.shape} does not match color frame {bgr.shape[:2]}")
        return mask


class RosRgbdSource:
    """Keep the newest color and aligned-depth ROS messages."""

    def __init__(self, color_topic: str, depth_topic: str):
        try:
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as error:
            raise FollowError(
                "ROS 2 Python dependencies are missing; run this inside the Jazzy camera container"
            ) from error

        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node = rclpy.create_node("rebot_clothing_follow_test")
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._color: tuple[float, np.ndarray] | None = None
        self._depth: tuple[float, np.ndarray] | None = None
        self.node.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        self.node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self._thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()

    @staticmethod
    def _stamp(message) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def _on_color(self, message) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        with self._lock:
            self._color = (self._stamp(message), np.asarray(image).copy())

    def _on_depth(self, message) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        depth = np.asarray(image)
        if depth.dtype == np.float32:
            depth = np.rint(depth * 1000.0).astype(np.uint16)
        elif depth.dtype != np.uint16:
            return
        with self._lock:
            self._depth = (self._stamp(message), depth.copy())

    def frame(self, timeout_s: float, max_skew_s: float) -> tuple[np.ndarray, np.ndarray]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                color, depth = self._color, self._depth
            if color is not None and depth is not None:
                if color[1].shape[:2] != depth[1].shape:
                    raise FollowError("color and aligned-depth frame sizes differ")
                if abs(color[0] - depth[0]) <= max_skew_s:
                    return color[1], depth[1]
            time.sleep(0.02)
        raise FollowError("timed out waiting for synchronized color and aligned-depth frames")

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()
        self._thread.join(timeout=2.0)


def observe_clothing(
    source: RosRgbdSource,
    sam: SamClient,
    args: argparse.Namespace,
) -> tuple[float, float, int, int]:
    color, depth = source.frame(args.frame_timeout, args.max_frame_skew)
    mask = sam.largest_mask(color, args.min_area_px)
    ys, xs = np.nonzero(mask)
    valid_depth = depth[mask]
    valid_depth = valid_depth[valid_depth > 0]
    if valid_depth.size < max(100, int(mask.sum() * 0.25)):
        raise FollowError("clothing mask has insufficient valid aligned depth")
    nearest_mm = float(np.percentile(valid_depth, 10))
    median_mm = float(np.median(valid_depth))
    if nearest_mm < args.min_clearance_mm:
        raise FollowError(
            f"object is too close: {nearest_mm:.0f} mm < {args.min_clearance_mm:.0f} mm"
        )
    if median_mm > args.max_depth_mm:
        raise FollowError(f"object is too far away: {median_mm:.0f} mm")
    return float(np.median(xs)), median_mm, color.shape[1], int(mask.sum())


def move_xyz(arm: Arm, target: list[float], approach: list[float], speed: float) -> None:
    arm.move_to_xyz(*target, approach=approach, linear=True, speed_scale=speed)


def initial_sweep(
    arm: Arm,
    source: RosRgbdSource,
    sam: SamClient,
    args: argparse.Namespace,
) -> tuple[list[float], list[float], int]:
    """Sweep laterally and infer which base-Y sign follows image-right."""
    arm.home(speed_scale=args.speed)
    state = arm.status()
    center = list(state["tcp_xyz_m"])
    approach = list(state["approach_axis"])
    offset = args.sweep_mm / 1000.0
    right = [center[0], center[1] - offset, center[2]]
    left = [center[0], center[1] + offset, center[2]]
    arm.plan_xyz(*right, approach=approach, linear=True)
    arm.plan_xyz(*left, approach=approach, linear=True)
    print("Initial sweep: right")
    move_xyz(arm, right, approach, args.speed)
    time.sleep(args.settle_s)
    u_right, _, _, _ = observe_clothing(source, sam, args)
    print("Initial sweep: left")
    move_xyz(arm, left, approach, args.speed)
    time.sleep(args.settle_s)
    u_left, _, _, _ = observe_clothing(source, sam, args)
    print("Initial sweep: center")
    move_xyz(arm, center, approach, args.speed)

    pixel_shift = u_left - u_right
    if abs(pixel_shift) < args.min_sweep_shift_px:
        raise FollowError(
            f"camera target shifted only {pixel_shift:+.1f} px during the physical sweep; "
            "arm motion or eye-in-hand response was not verified"
        )
    inferred_sign = -1 if pixel_shift > 0 else 1
    if args.y_sign is not None and args.y_sign != inferred_sign:
        raise FollowError(
            f"--y-sign {args.y_sign} disagrees with sweep-inferred sign {inferred_sign}"
        )
    print(
        f"Sweep verification: clothing shifted {pixel_shift:+.1f} px; "
        f"image-right follows base-Y sign {inferred_sign:+d}"
    )
    return center, approach, inferred_sign


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", action="store_true", help="enable the real arm (default: camera preview only)")
    parser.add_argument(
        "--y-sign",
        type=int,
        choices=(-1, 1),
        help="optional expected direction; the physical sweep infers and verifies it",
    )
    parser.add_argument("--sweep-mm", type=float, default=5.0)
    parser.add_argument("--speed", type=float, default=0.02)
    parser.add_argument("--step-mm", type=float, default=3.0)
    parser.add_argument("--max-offset-mm", type=float, default=30.0)
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--deadband-px", type=float, default=40.0)
    parser.add_argument("--min-sweep-shift-px", type=float, default=4.0)
    parser.add_argument("--min-clearance-mm", type=float, default=350.0)
    parser.add_argument("--max-depth-mm", type=float, default=1500.0)
    parser.add_argument("--min-area-px", type=int, default=1500)
    parser.add_argument("--frame-timeout", type=float, default=5.0)
    parser.add_argument("--max-frame-skew", type=float, default=0.08)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--sam-url", default="https://exes-shape-zoning.ngrok-free.dev")
    parser.add_argument("--sam-prompt", default="clothing")
    parser.add_argument("--sam-threshold", type=float, default=0.5)
    parser.add_argument("--sam-timeout", type=float, default=10.0)
    return parser.parse_args()


def validate(args: argparse.Namespace) -> None:
    if not 1.0 <= args.sweep_mm <= 20.0:
        raise FollowError("--sweep-mm must be between 1 and 20")
    if not 0.01 <= args.speed <= 0.05:
        raise FollowError("--speed must be between 0.01 and 0.05")
    if not 1.0 <= args.step_mm <= 5.0:
        raise FollowError("--step-mm must be between 1 and 5")
    if not 5.0 <= args.max_offset_mm <= 50.0:
        raise FollowError("--max-offset-mm must be between 5 and 50")
    if not 1 <= args.max_steps <= 30:
        raise FollowError("--max-steps must be between 1 and 30")
    if args.min_clearance_mm < 300.0:
        raise FollowError("--min-clearance-mm cannot be below 300")


def confirm_hardware(args: argparse.Namespace) -> bool:
    if not args.hardware:
        return True
    queue_path = Path("/sys/class/net/can0/tx_queue_len")
    try:
        queue_length = int(queue_path.read_text().strip())
    except (OSError, ValueError):
        raise FollowError("can0 is unavailable") from None
    if queue_length < 100:
        raise FollowError(f"can0 tx_queue_len is only {queue_length}; configure it to 1000")
    print("WARNING: this test will move the physical arm near a presented garment.")
    print("No person may enter the arm workspace. Hold clothing on a stand, not in your hand.")
    print("Keep access to the physical power switch. Lost vision stops, then returns home.")
    return input("Type FOLLOW_CLOTHING to enable hardware: ").strip() == "FOLLOW_CLOTHING"


def supervised_stop(arm: Arm, *, allow_home: bool) -> bool:
    """Keep streaming the hold target until the operator chooses a safe exit.

    Returns True when normal homing should run.  Returning False means the
    operator confirmed physical support and this function already disabled.
    """
    if arm.simulated:
        return True
    print("The physical arm is being held; keep this process running.", file=sys.stderr)
    if allow_home:
        print("Type HOME to return to rest, or support the arm and type SUPPORTED.", file=sys.stderr)
    else:
        print("The fault blocks homing. Physically support the arm, then type SUPPORTED.", file=sys.stderr)
    while True:
        try:
            choice = input("Recovery choice: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("No safe choice received; continuing to hold.", file=sys.stderr)
            time.sleep(1.0)
            continue
        if allow_home and choice == "HOME":
            return True
        if choice == "SUPPORTED":
            arm.disconnect(go_home=False)
            return False
        print("Enter only HOME or SUPPORTED as described above.", file=sys.stderr)


def main() -> int:
    args = parse_args()
    source: RosRgbdSource | None = None
    arm: Arm | None = None
    completed = False
    return_home = False
    try:
        validate(args)
        sam = SamClient(
            args.sam_url,
            os.environ.get("SAM3_API_KEY", ""),
            args.sam_prompt,
            args.sam_threshold,
            args.sam_timeout,
        )
        source = RosRgbdSource(args.color_topic, args.depth_topic)
        print("Waiting for aligned RGB-D and validating the clothing target ...")
        u, depth_mm, width, area = observe_clothing(source, sam, args)
        print(f"Clothing: u={u:.1f}/{width}, median depth={depth_mm:.0f} mm, area={area} px")
        if not args.hardware:
            print("Preview passed. No arm object was created and no CAN connection was opened.")
            completed = True
            return 0
        if not confirm_hardware(args):
            print("Cancelled; arm torque was not enabled.")
            return 1

        arm = Arm(max_speed_scale=0.05)
        arm.connect(enable=True, simulate=False)
        return_home = True
        center, approach, inferred_sign = initial_sweep(arm, source, sam, args)
        current_y = center[1]
        y_sign = inferred_sign

        for step in range(1, args.max_steps + 1):
            time.sleep(args.settle_s)
            u, depth_mm, width, area = observe_clothing(source, sam, args)
            error_px = u - width / 2.0
            print(
                f"Track {step}/{args.max_steps}: error={error_px:+.1f} px, "
                f"depth={depth_mm:.0f} mm, area={area} px"
            )
            if abs(error_px) <= args.deadband_px:
                print("Clothing is centered; tracking complete.")
                completed = True
                return 0
            delta_y = y_sign * np.sign(error_px) * args.step_mm / 1000.0
            target_y = current_y + float(delta_y)
            max_offset = args.max_offset_mm / 1000.0
            if abs(target_y - center[1]) > max_offset:
                raise FollowError("tracking would exceed the bounded lateral workspace")
            target = [center[0], target_y, center[2]]
            arm.plan_xyz(*target, approach=approach, linear=True)
            move_xyz(arm, target, approach, args.speed)
            current_y = target_y

        raise FollowError("maximum tracking steps reached without centering the clothing")
    except KeyboardInterrupt:
        if arm is not None and arm.connected:
            arm.stop()
            return_home = supervised_stop(arm, allow_home=True)
        print("Interrupted: arm stopped and holding.", file=sys.stderr)
        return 130
    except FollowError as error:
        if arm is not None and arm.connected:
            arm.stop()
        print(f"Test stopped: {error}", file=sys.stderr)
        return 1
    except ArmError as error:
        if arm is not None and arm.connected:
            arm.stop()
            return_home = supervised_stop(arm, allow_home=False)
        print(f"Arm fault: {error}", file=sys.stderr)
        return 1
    finally:
        if arm is not None and arm.connected:
            if return_home:
                try:
                    arm.disconnect(go_home=True, speed_scale=args.speed)
                except (ArmError, KeyboardInterrupt) as error:
                    print(f"Arm shutdown failed: {error}; torque may still be ON.", file=sys.stderr)
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())

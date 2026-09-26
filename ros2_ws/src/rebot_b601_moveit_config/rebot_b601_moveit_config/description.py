"""Build the robot_description for MoveIt from the Seeed B601-RS URDF.

The source URDF (rebot_b601/rebot_b601/viewer_assets, symlinked into urdf/) is kept untouched.
At launch we patch a copy in memory:

* mesh URIs point at this package (the source uses a package name that does not exist here);
* collision geometry is one box per link: the bounding box of the link's visual meshes. The
  source's collision meshes for link2–link5 and the gripper do not exist (the visuals are
  split into CAD parts), and boxes are much cheaper to check than the 35 MB of STL. The two
  fingers keep their real meshes (~1 MB each): they go into the box, where a loose box around
  the open fingers makes cloth near a wall ungraspable;
* joint limits are the soft limits of the arm driver (rebot_b601/config.py), not the wider URDF
  ones, and the velocities are realistic (the URDF says 50 rad/s);
* a `table` link (collision box, top at z = 0) so plans never go through the table;
* the wrist camera frames `camera_link` / `camera_color_optical_frame` (placeholder offset until
  the hand-eye calibration, block 2, gives the real one), with a RealSense-sized collision box;
* a <ros2_control> block: mock hardware for simulation, or a named real hardware plugin.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

SOURCE_MESH_PREFIX = "package://reBot_Lite_RS_with_gripper/meshes/"
MESH_PREFIX = "package://rebot_b601_moveit_config/meshes/"

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINTS = ("joint_left", "joint_right")

# (lower, upper) deg: intersection of URDF and driver soft limits, from rebot_b601/config.py.
SOFT_LIMITS_DEG = {
    "joint1": (-145.0, 145.0),
    "joint2": (0.0, 170.0),
    "joint3": (0.0, 180.0),
    "joint4": (-80.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-130.0, 130.0),
}
# Peak joint speed at speed scale 1, deg/s (rebot_b601/config.py JOINT_SPEED_DPS).
MAX_VELOCITY_DPS = {
    "joint1": 40.0,
    "joint2": 30.0,
    "joint3": 30.0,
    "joint4": 60.0,
    "joint5": 60.0,
    "joint6": 90.0,
}

# Default mount of the RealSense on the wrist, relative to gripper_end (whose +X is the approach
# axis): 12 cm back from the fingertips and 6 cm off-axis, looking along +X. A placeholder: the
# launch file reads the measured one from config/camera_mount.yaml.
CAMERA_XYZ = (-0.12, 0.0, 0.06)
CAMERA_RPY = (0.0, 0.0, 0.0)

MESH_COLLISION_LINKS = ("gripper_left", "gripper_right", "link1", "link5")
CAMERA_BOX = (0.025, 0.09, 0.025)  # D435 body in camera_link (x forward), m

TABLE_SIZE = (1.2, 1.2, 0.02)  # m; top face at z = 0 (the arm base plate sits on the table)
TABLE_XY = (0.3, 0.0)


def _fmt(values) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _rpy_matrix(roll, pitch, yaw) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _matrix_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of _rpy_matrix (URDF fixed-axis roll, pitch, yaw)."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), pitch, 0.0


def _joint_transform(joint) -> np.ndarray:
    origin = joint.find("origin")
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*[float(v) for v in origin.get("rpy", "0 0 0").split()])
    T[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    return T


def _mount_in_parent(robot, parent: str, xyz, rpy):
    """(xyz, rpy) of a mount given relative to gripper_end at zero joints, in `parent`'s frame."""
    by_child = {j.find("child").get("link"): j for j in robot.findall("joint")}
    T_parent_ge = np.eye(4)  # parent → gripper_end with every joint at zero
    link = "gripper_end"
    while link != parent:
        if link not in by_child:
            raise ValueError(f"camera parent {parent!r} is not between base_link and gripper_end")
        joint = by_child[link]
        T_parent_ge = _joint_transform(joint) @ T_parent_ge
        link = joint.find("parent").get("link")
    T_mount = np.eye(4)
    T_mount[:3, :3] = _rpy_matrix(*rpy)
    T_mount[:3, 3] = xyz
    T = T_parent_ge @ T_mount
    return tuple(T[:3, 3]), _matrix_rpy(T[:3, :3])


def _stl_vertices(path: str) -> np.ndarray:
    """Vertices of a binary STL, N×3."""
    with open(path, "rb") as f:
        data = f.read()
    n = int.from_bytes(data[80:84], "little")
    tri = np.dtype([("normal", "<3f4"), ("v", "<9f4"), ("attr", "<u2")])
    return np.frombuffer(data[84 : 84 + n * 50], dtype=tri)["v"].reshape(-1, 3).astype(float)


def _visual_bbox(link, mesh_dir: str):
    """(size, center) of the link's visual meshes in the link frame, or None."""
    pts = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        path = os.path.join(mesh_dir, os.path.basename(mesh.get("filename")))
        if not os.path.exists(path):
            continue
        origin = visual.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        pts.append(_stl_vertices(path) @ _rpy_matrix(*rpy).T + xyz)
    if not pts:
        return None
    all_pts = np.vstack(pts)
    lo, hi = all_pts.min(axis=0), all_pts.max(axis=0)
    return tuple(hi - lo), tuple((hi + lo) / 2)


def _set_box_collision(link, size, center) -> None:
    for old in link.findall("collision"):
        link.remove(old)
    col = ET.SubElement(link, "collision")
    ET.SubElement(col, "origin", xyz=_fmt(center), rpy="0 0 0")
    ET.SubElement(ET.SubElement(col, "geometry"), "box", size=_fmt(size))


def _set_mesh_collision(link) -> None:
    """Collision = the link's visual meshes, as they are."""
    for old in link.findall("collision"):
        link.remove(old)
    for visual in link.findall("visual"):
        col = ET.SubElement(link, "collision")
        for tag in ("origin", "geometry"):
            el = visual.find(tag)
            if el is not None:
                col.append(ET.fromstring(ET.tostring(el)))


def _add_fixed_link(robot, name, parent, xyz, rpy, box=None):
    link = ET.SubElement(robot, "link", name=name)
    if box is not None:
        size, origin = box
        for tag in ("visual", "collision"):
            el = ET.SubElement(link, tag)
            ET.SubElement(el, "origin", xyz=_fmt(origin), rpy="0 0 0")
            ET.SubElement(ET.SubElement(el, "geometry"), "box", size=_fmt(size))
    joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=name)
    ET.SubElement(joint, "origin", xyz=_fmt(xyz), rpy=_fmt(rpy))


def _add_ros2_control(robot, hardware_plugin: str, initial: dict[str, float]):
    rc = ET.SubElement(robot, "ros2_control", name="rebot_b601", type="system")
    hw = ET.SubElement(rc, "hardware")
    ET.SubElement(hw, "plugin").text = hardware_plugin
    for name in ARM_JOINTS + GRIPPER_JOINTS:
        j = ET.SubElement(rc, "joint", name=name)
        ET.SubElement(j, "command_interface", name="position")
        state = ET.SubElement(j, "state_interface", name="position")
        ET.SubElement(state, "param", name="initial_value").text = f"{initial.get(name, 0.0):.6g}"
        ET.SubElement(j, "state_interface", name="velocity")


def build_urdf(
    urdf_path: str,
    hardware_plugin: str = "mock_components/GenericSystem",
    initial_positions: dict[str, float] | None = None,
    camera_xyz=CAMERA_XYZ,
    camera_rpy=CAMERA_RPY,
    camera_parent: str = "gripper_end",
) -> str:
    """camera_xyz / camera_rpy: camera_link relative to gripper_end with every joint at zero
    (easy to measure: from the fingertips, wrist straight). camera_parent: the link the camera
    is bolted to; the mount is converted to that link's frame, so e.g. a camera on the joint5
    motor (link4) does not turn with joints 5 and 6."""
    tree = ET.parse(urdf_path)
    robot = tree.getroot()
    mesh_dir = os.path.join(os.path.dirname(os.path.dirname(urdf_path)), "meshes")

    for link in robot.findall("link"):
        if link.get("name") in MESH_COLLISION_LINKS:
            _set_mesh_collision(link)
            continue
        bbox = _visual_bbox(link, mesh_dir)
        if bbox is None:
            raise FileNotFoundError(f"no visual meshes found for link {link.get('name')}")
        _set_box_collision(link, *bbox)

    for mesh in robot.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith(SOURCE_MESH_PREFIX):
            mesh.set("filename", MESH_PREFIX + fn[len(SOURCE_MESH_PREFIX) :])

    for joint in robot.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in SOFT_LIMITS_DEG and limit is not None:
            lo, hi = SOFT_LIMITS_DEG[name]
            limit.set("lower", f"{math.radians(lo):.6f}")
            limit.set("upper", f"{math.radians(hi):.6f}")
            limit.set("velocity", f"{math.radians(MAX_VELOCITY_DPS[name]):.6f}")
        elif name in GRIPPER_JOINTS and limit is not None:
            limit.set("velocity", "0.05")

    sx, sy, sz = TABLE_SIZE
    _add_fixed_link(
        robot,
        "table",
        "base_link",
        (0, 0, 0),
        (0, 0, 0),
        box=((sx, sy, sz), (TABLE_XY[0], TABLE_XY[1], -sz / 2)),
    )
    _add_fixed_link(
        robot,
        "camera_link",
        camera_parent,
        *_mount_in_parent(robot, camera_parent, camera_xyz, camera_rpy),
        box=(CAMERA_BOX, (0, 0, 0)),
    )
    # REP-103 optical frame: z forward, x right, y down.
    _add_fixed_link(
        robot,
        "camera_color_optical_frame",
        "camera_link",
        (0, 0, 0),
        (-math.pi / 2, 0, -math.pi / 2),
    )

    _add_ros2_control(robot, hardware_plugin, initial_positions or {})
    return ET.tostring(robot, encoding="unicode")

"""Cloth task: MoveIt + arm (mock or real) + camera/detector (sim or real) + task supervisor.

    ros2 launch cloth_task task.launch.py use_rviz:=true             # all simulated
    ros2 launch cloth_task task.launch.py camera:=real use_rviz:=true  # real camera+SAM3, mock arm
    ros2 launch cloth_task task.launch.py hardware:=real camera:=real  # real arm, READ-ONLY
    ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true \
        grasp:=false                                                 # stops above the cloth
    ros2 launch cloth_task task.launch.py hardware:=real camera:=real enable_motors:=true
    ros2 launch cloth_task task.launch.py hardware:=real enable_motors:=true run_task:=false \
        spectacles:=true                                             # lens on 127.0.0.1:9100

hardware:=real replaces ros2_control with arm_bridge (the rebot_b601 driver over CAN).
camera:=real starts realsense2_camera and cloth_detector_node (SAM3; key in config/local.yaml).
spectacles:=true starts spectacles_bridge, which turns arm_bridge's teleop mode on and leaves it
on (MoveIt goals are then refused). ngrok is started by hand; the launch never holds its token.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(pkg: str) -> str:
    return get_package_share_directory(pkg)


def _default_repo_dir() -> str:
    # with --symlink-install this resolves to <repo>/ros2_ws/src/cloth_task/config/task.yaml
    path = os.path.realpath(os.path.join(_share("cloth_task"), "config", "task.yaml"))
    return os.environ.get("ALIEN_BAZAAR_REPO", os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(path))))))  # fmt: skip


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    hardware, camera = arg("hardware"), arg("camera")
    if hardware not in ("mock", "real") or camera not in ("sim", "real"):
        raise RuntimeError(
            f"hardware must be mock|real, camera sim|real (got {hardware}, {camera})"
        )
    spectacles = arg("spectacles") == "true"
    if spectacles and (arg("run_task") == "true" or arg("run_stack") != "true"):
        raise RuntimeError(
            "spectacles:=true drives the arm from the lens: it cannot run with the cloth task "
            "(add run_task:=false; run_stack:=false starts only the task)"
        )
    if spectacles and hardware != "real":
        raise RuntimeError("spectacles:=true needs hardware:=real (arm_bridge takes the commands)")
    if (
        hardware == "real"
        and camera == "sim"
        and arg("enable_motors") == "true"
        and arg("run_task") == "true"
        and arg("sim_camera_ok") != "true"
    ):
        raise RuntimeError(
            "hardware:=real enable_motors:=true with camera:=sim would make the real arm grasp a "
            "virtual cloth: add camera:=real (or sim_camera_ok:=true if that is really meant)"
        )
    repo = arg("repo_dir")
    ros2_ws = os.path.join(repo, "ros2_ws")
    if arg("run_stack") != "true":  # only the task, on a stack that is already running
        return [_supervisor(arg), *_web(arg)]
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_share("rebot_b601_moveit_config"), "launch", "moveit.launch.py")
            ),
            launch_arguments={
                "use_rviz": arg("use_rviz"),
                "use_ros2_control": "true" if hardware == "mock" else "false",
            }.items(),
        )
    ]

    if hardware == "real":
        rebot_dir = os.path.join(repo, "rebot_b601")
        if not os.path.isdir(os.path.join(rebot_dir, "rebot_b601")):
            raise RuntimeError(f"rebot_b601 driver not found in {rebot_dir} (set repo_dir:=...)")
        actions.append(
            Node(
                package="cloth_task",
                executable="arm_bridge",
                parameters=[
                    {
                        "rebot_dir": rebot_dir,
                        "enable_motors": arg("enable_motors") == "true",
                        "driver_sim": arg("driver_sim") == "true",
                        # the task starts right away, and Spectacles teleop refuses MoveIt
                        # goals: unfold out of home first
                        "unfold_on_start": arg("run_task") == "true" or spectacles,
                        # no MoveIt goals while Spectacles teleop is on: the single-threaded
                        # executor keeps /joint_states on time
                        "single_threaded": spectacles,
                    }
                ],
                output="screen",
                # parking on Ctrl+C (lift, via pose, home, torque off) takes a while
                sigterm_timeout="60",
                sigkill_timeout="10",
            )
        )
    else:
        actions.append(
            Node(
                package="cloth_task",
                executable="sim_gripper",
                parameters=[
                    {"misses": ParameterValue(LaunchConfiguration("sim_misses"), value_type=int)}
                ],
                output="screen",
            )
        )

    if camera == "real":
        try:
            rs_launch = os.path.join(_share("realsense2_camera"), "launch", "rs_launch.py")
        except PackageNotFoundError:
            raise RuntimeError(
                "realsense2_camera is not installed: sudo apt install ros-jazzy-realsense2-camera"
            ) from None
        pydeps = os.path.join(ros2_ws, ".pydeps")
        if not os.path.isdir(os.path.join(pydeps, "pydantic")):
            raise RuntimeError(f"{pydeps} is missing: run {ros2_ws}/scripts/setup_deps.sh")
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    "camera_name": "camera",
                    "camera_namespace": "camera",
                    "serial_no": arg("camera_serial"),
                    "initial_reset": "true",
                    "enable_color": "true",
                    "enable_depth": "true",
                    "rgb_camera.color_profile": arg("camera_profile"),
                    "depth_module.depth_profile": arg("camera_profile"),
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    # the camera's frames come from the URDF (camera_mount.yaml), not the driver
                    "publish_tf": "false",
                }.items(),
            ),
            Node(
                package="cloth_task",
                executable="cloth_detector",
                name="cloth_detector_node",
                parameters=[
                    {
                        "config_dir": os.path.join(repo, "config"),
                        "detector_config": os.path.realpath(
                            os.path.join(_share("cloth_task"), "config", "detector.yaml")
                        ),
                    }
                ],
                additional_env={
                    "PYTHONPATH": os.pathsep.join(
                        [os.path.join(repo, "src"), pydeps, os.environ.get("PYTHONPATH", "")]
                    )
                },
                output="screen",
            ),
        ]
    else:
        actions.append(
            Node(
                package="cloth_task",
                executable="sim_cloth_detector",
                name="cloth_detector_node",
                parameters=[
                    {
                        "cloth_xyz": LaunchConfiguration("sim_cloth_xyz"),
                        "colors": LaunchConfiguration("sim_colors"),
                    }
                ],
                output="screen",
            )
        )

    if spectacles:
        actions.append(
            Node(
                package="cloth_task",
                executable="spectacles_bridge",
                parameters=[
                    {
                        "rebot_dir": os.path.join(repo, "rebot_b601"),
                        "host": "127.0.0.1",
                        "port": 9100,
                    }
                ],
                output="screen",
            )
        )

    if arg("run_task") != "true":  # bring-up only: nothing moves by itself
        return actions
    return [*actions, _supervisor(arg), *_web(arg)]


def _web(arg) -> list:
    """The status page (status_web), next to the task: http://localhost:<web_port>."""
    if arg("web") != "true":
        return []
    return [
        Node(
            package="cloth_task",
            executable="status_web",
            parameters=[{"port": ParameterValue(LaunchConfiguration("web_port"), value_type=int)}],
            output="screen",
        )
    ]


def _supervisor(arg) -> Node:
    return Node(
        package="cloth_task",
        executable="task_supervisor",
        name="task_supervisor_node",
        parameters=[
            {
                "execution_speed": ParameterValue(
                    LaunchConfiguration("execution_speed"), value_type=float
                ),
                # "1" → int, "color" → str (YAML parse; the supervisor checks it)
                "place_target": LaunchConfiguration("place_target"),
                "cycles": ParameterValue(LaunchConfiguration("cycles"), value_type=int),
                "mode": arg("mode"),
                "start_pose": arg("start_pose"),
                "grasp": arg("grasp") == "true",
                "task_config": arg("task_config"),
                "poses_file": arg("poses_file"),
            }
        ],
        output="screen",
    )


def generate_launch_description():
    share = _share("cloth_task")
    args = [
        ("execution_speed", "0.3", "MoveIt velocity scaling, 0.1 … 1.0 (real arm caps at 0.6)"),
        ("place_target", "1", "1, 2, 3 (config/task.yaml place_targets) or color (by SAM3 class)"),
        ("cycles", "1", "Cloths to pick and place; 0 = until no cloth is detected"),
        ("mode", "fixed_view", "fixed_view: go to start_pose and look; search: sweep the table"),
        ("start_pose", "box_view", "Named pose in poses.yaml to look at the cloth from"),
        ("grasp", "true", "false: stop above the cloth (first runs on the real arm)"),
        ("hardware", "mock", "mock: ros2_control mock hardware; real: arm_bridge over CAN"),
        ("enable_motors", "false", "hardware:=real only: false = read the encoders, arm limp"),
        ("driver_sim", "false", "hardware:=real only: the rebot driver's simulated arm, no CAN"),
        ("camera", "sim", "sim: sim_cloth_detector; real: RealSense + SAM3 cloth_detector"),
        ("camera_serial", "''", "RealSense serial with a leading underscore; '' = the only one"),
        ("camera_profile", "640,480,15", "RealSense color/depth profile (15 fps: USB 2 works)"),
        (
            "sim_cloth_xyz",
            "[0.38, 0.25, 0.03]",  # in view from the recorded box_view, and reachable
            "Virtual cloth(s): x,y,z per cloth, flat (camera:=sim)",
        ),
        ("sim_colors", "[colored]", "Class of each virtual cloth (camera:=sim)"),
        ("sim_misses", "1", "Simulated grasps that miss before one holds (hardware:=mock)"),
        ("sim_camera_ok", "false", "Allow the real arm with the simulated camera"),
        ("task_config", os.path.join(share, "config", "task.yaml"), "Task config"),
        (
            "poses_file",
            os.path.realpath(os.path.join(share, "config", "poses.yaml")),
            "Named poses (with --symlink-install: the file in src/ that record_pose edits)",
        ),
        ("repo_dir", _default_repo_dir(), "Repo root: config/, src/ (sorter), rebot_b601/"),
        ("use_rviz", "false", "RViz with the planning scene and markers"),
        ("run_task", "true", "false: arm, camera, MoveIt only (record poses, go_to_pose)"),
        ("web", "true", "Status page at http://localhost:<web_port>"),
        ("web_port", "8080", "Port of the status page"),
        ("run_stack", "true", "false: only the task, on an already running run_task:=false stack"),
        (
            "spectacles",
            "false",
            "true: Spectacles teleop on 127.0.0.1:9100 (hardware:=real, run_task:=false)",
        ),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=_setup)]
    )

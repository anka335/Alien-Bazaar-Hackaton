from glob import glob

from setuptools import find_packages, setup

package_name = "cloth_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alien Bazaar team",
    maintainer_email="prog74194@gmail.com",
    description="Cloth pick-and-place task: supervisor state machine and sim detector",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_supervisor = cloth_task.supervisor_node:main",
            "sim_cloth_detector = cloth_task.sim_cloth_detector:main",
            "record_pose = cloth_task.pose_tools:record_main",
            "go_to_pose = cloth_task.pose_tools:go_main",
            "sim_gripper = cloth_task.sim_gripper:main",
            "arm_bridge = cloth_task.arm_bridge:main",
            "cloth_detector = cloth_task.cloth_detector_node:main",
            "leader_teleop = cloth_task.leader_teleop:main",
            "roi_tool = cloth_task.roi_tool:main",
            "status_web = cloth_task.status_web:main",
            "spectacles_bridge = cloth_task.spectacles_bridge:main",
            "rover_standin = cloth_task.rover_standin:main",
            "spectacles_standin_lens = cloth_task.spectacles_standin_lens:main",
        ],
    },
)

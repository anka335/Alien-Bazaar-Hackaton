import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rebot_b601_moveit_config"


def tree(src):
    """data_files entries for every file under src (follows the meshes symlink)."""
    out = {}
    for path in glob(os.path.join(src, "**", "*"), recursive=True):
        if os.path.isfile(path):
            dest = os.path.join("share", package_name, os.path.dirname(path))
            out.setdefault(dest, []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *tree("config"),
        *tree("launch"),
        *tree("urdf"),
        *tree("meshes"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alien Bazaar team",
    maintainer_email="prog74194@gmail.com",
    description="MoveIt 2 config and ros2_control setup for the reBot B601-RS",
    license="MIT",
)

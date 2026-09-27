from glob import glob

from setuptools import find_packages, setup

package_name = "rover_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rover", glob("rover/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alien Bazaar team",
    maintainer_email="slava@lute.one",
    description="Leo Rover navigation: RTAB-Map + Nav2 + keepout filter",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "make_keepout = rover_nav.keepout:main",
            "patrol = rover_nav.patrol_node:main",
            "patrol_ctl = rover_nav.patrol_ctl:main",
            "fake_rover = rover_nav.fake_rover:main",
        ],
    },
)

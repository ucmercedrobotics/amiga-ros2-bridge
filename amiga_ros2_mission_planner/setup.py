import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_mission_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*launch.[pxy][yma]*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="LLM-powered mission planner A2A agent",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_planner = amiga_ros2_mission_planner.mission_planner_node:main",
        ],
    },
)
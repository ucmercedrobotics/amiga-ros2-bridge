import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*config.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "include"),
            glob(os.path.join("include", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "amiga_streams = amiga_ros2_bridge.amiga_streams:main",
            "twist_control = amiga_ros2_bridge.twist_control:main",
            "twist_wasd = amiga_ros2_bridge.twist_wasd:main",
            "clock = amiga_ros2_bridge.clock:main",
        ],
    },
)

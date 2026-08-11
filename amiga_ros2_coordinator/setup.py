import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_coordinator"

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
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="Contract-net coordination for the Amiga fleet: task announce/bid/grant, peer registry, hazard propagation and safe preemption",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coordinator = amiga_ros2_coordinator.nodes.coordinator_node:main",
            "coordinator_sim = amiga_ros2_coordinator.nodes.sim_node:main",
            "escalate = amiga_ros2_coordinator.nodes.escalate:main",
        ],
    },
)

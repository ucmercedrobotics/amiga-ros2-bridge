import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_comms"

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
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="Robot-to-robot comms for the Amiga: LoRa serial bridge, coordination codec, and reliable delivery",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lora_bridge = amiga_ros2_comms.lora.bridge_node:main",
            "lora_sim = amiga_ros2_comms.lora.sim_node:main",
            "lora_reliability = amiga_ros2_comms.reliability.node:main",
        ],
    },
)

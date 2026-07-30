import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_agents"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="A2A agents for the Amiga robot (LTL generation, ...)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ltl_gen = amiga_ros2_agents.ltl_gen_node:main",
            "dummy_agent = amiga_ros2_agents.dummy_node:main",
        ],
    },
)

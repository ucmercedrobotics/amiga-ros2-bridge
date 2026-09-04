import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amiga_ros2_agents"


def prompt_data_files():
    """Install prompts/<agent>/*.j2 into share/, preserving the directory layout.

    Jinja templates live outside the Python package, so find_packages() doesn't
    see them — they have to be listed as data_files explicitly. With
    `colcon build --symlink-install` they're symlinked, so editing a prompt takes
    effect on the next node restart without a rebuild.
    """
    grouped = {}
    for path in glob(os.path.join("prompts", "**", "*.j2"), recursive=True):
        dest = os.path.join("share", package_name, os.path.dirname(path))
        grouped.setdefault(dest, []).append(path)
    return sorted(grouped.items())


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
    ]
    + prompt_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="appuser",
    maintainer_email="appuser@todo.todo",
    description="LLM agents for the Amiga robot — mission planning, arbitration, world state",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_planner = amiga_ros2_agents.replanning.mission_planner_node:main",
            "arbiter = amiga_ros2_agents.replanning.arbiter_node:main",
            "world_state = amiga_ros2_agents.replanning.world_state_node:main",
            "triage = amiga_ros2_agents.coordination.triage_node:main",
            "note = amiga_ros2_agents.coordination.note_node:main",
            "mission_bridge = amiga_ros2_agents.coordination.mission_bridge_node:main",
        ],
    },
)

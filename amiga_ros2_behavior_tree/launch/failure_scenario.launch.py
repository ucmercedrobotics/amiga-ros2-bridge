"""One robot, one real mission, one failure that looks like a real one.

The chain this exercises starts at the behaviour tree and ends at an auction:

    bt_runner --(a node fails)--> /bt/status_change
        --> mission_planner + arbiter (local recovery)
        --> triage (escalation) --> /coordination/infeasible
        --> coordinator --> TASK_ANNOUNCE

Everything below the tree only has something to do because the tree failed, so
until this existed, every stage below it had only ever run against a fault
published by hand.

The mock action servers are the *real* servers' stand-ins, and when they fail
they fail the way those servers do: same result code, same log text, same node
name, same timing. See ``include/.../mocks/failure_modes.hpp`` for why that
matters and, just as importantly, what it still does not prove.

    ros2 launch amiga_ros2_behavior_tree failure_scenario.launch.py \\
        mission:=sample_leafs.xml fail_goals:="[60]" failure_mode:=nav_failed

Add ``agents:=true`` to bring up the planner, arbiter and triage as well, which
needs a model endpoint. Without it this is the tree and its mocks only, which
is enough to watch a fault reach ``/bt/status_change``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "amiga_ros2_behavior_tree"


def _mission_path(name: str) -> str:
    """An example mission by name, or an absolute path taken as given."""
    if os.path.isabs(name):
        return name
    return os.path.join(get_package_share_directory(PACKAGE), "examples", name)


def _publish_mission(context, *args, **kwargs):
    """Publish the mission once the tree is up.

    A plain `ros2 topic pub --once` rather than a node: /mission/xml is a
    std_msgs/String and the arbiter is its only other publisher, so anything
    more would be a second writer to a topic that deliberately has one.
    """
    mission = _mission_path(LaunchConfiguration("mission").perform(context))
    with open(mission) as handle:
        xml = handle.read()
    return [
        Node(
            package="amiga_ros2_behavior_tree",
            executable="mission_publisher",
            name="mission_publisher",
            parameters=[{"mission_xml": xml, "delay_sec": 3.0}],
            output="screen",
        )
    ]


def generate_launch_description():
    fail_goals = LaunchConfiguration("fail_goals")
    fail_after_n = LaunchConfiguration("fail_after_n")
    failure_mode = LaunchConfiguration("failure_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission",
                default_value="sample_leafs.xml",
                description=(
                    "An example mission by filename, or an absolute path. Real "
                    "missions on purpose: a fixture written for a test is a "
                    "place to encode the same assumption twice."
                ),
            ),
            DeclareLaunchArgument(
                "fail_goals",
                default_value="[60]",
                description=(
                    "Tree ids whose navigation fails. [0] for the arm, which "
                    "takes no goal argument. Empty means nothing fails."
                ),
            ),
            DeclareLaunchArgument(
                "fail_after_n",
                default_value="0",
                description=(
                    "Attempts at each of those goals that succeed first. 2 with "
                    "a RetryUntilSuccessful of 3 is a transient fault the tree "
                    "recovers from by itself, which should never reach the "
                    "radio."
                ),
            ),
            DeclareLaunchArgument(
                "failure_mode",
                default_value="nav_failed",
                description=(
                    "nav_failed | tree_info_empty | tree_info_unavailable | "
                    "no_waypoint | no_point_cloud | no_leaves. Each is a real "
                    "failure path, and they are not interchangeable: "
                    "tree_info_empty means the tree does not exist, so no "
                    "robot can do the work, while nav_failed means this robot "
                    "could not get there and a peer might."
                ),
            ),
            DeclareLaunchArgument(
                "xml_validation",
                default_value="true",
                description=(
                    "Validate the mission against amiga_btcpp.xsd, as a robot "
                    "does. Left on: a scenario that had to disable it would be "
                    "running a mission this robot would refuse."
                ),
            ),
            DeclareLaunchArgument(
                "agents",
                default_value="false",
                description=(
                    "Also start the planner, arbiter and triage. Needs a model "
                    "endpoint; without it the tree and its mocks come up alone."
                ),
            ),
            Node(
                package=PACKAGE,
                executable="bt_runner",
                name="bt_runner",
                parameters=[{"xml_validation": LaunchConfiguration("xml_validation")}],
                output="screen",
            ),
            # Under the real node's name, because the "[node]" prefix is part
            # of the evidence the triage agent reads.
            Node(
                package=PACKAGE,
                executable="dummy_tree_id_server",
                name="waypoint_follower",
                arguments=["--mock-node-name", "waypoint_follower"],
                parameters=[
                    {
                        "fail_goals": fail_goals,
                        "fail_after_n": fail_after_n,
                        "failure_mode": failure_mode,
                    }
                ],
                output="screen",
            ),
            Node(
                package=PACKAGE,
                executable="dummy_segment_leaves_server",
                name="pistachio_leaf_segmentation",
                arguments=["--mock-node-name", "pistachio_leaf_segmentation"],
                parameters=[
                    {
                        "fail_goals": fail_goals,
                        "fail_after_n": fail_after_n,
                        "failure_mode": failure_mode,
                    }
                ],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("amiga_ros2_agents"),
                        "launch",
                        "agents.launch.py",
                    )
                ),
                condition=IfCondition(LaunchConfiguration("agents")),
            ),
            OpaqueFunction(function=_publish_mission),
        ]
    )

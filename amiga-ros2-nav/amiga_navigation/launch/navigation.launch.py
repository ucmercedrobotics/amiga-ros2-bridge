import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


# nav2_params.yaml topic values that are absolute (leading "/") and therefore
# NOT touched by PushRosNamespace — these are the only VALUES that need
# per-robot rewriting; everything else in the file is relative and
# namespaces correctly on its own.
ABSOLUTE_NAV2_TOPICS = [
    "/odometry/filtered/global",
    "/odometry/filtered/local",
    "/oak0/points",
]

# Top-level keys in nav2_params.yaml, each naming a node. ROS 2's yaml-params
# loader only applies a section to a node whose fully qualified name matches
# the section's key (or a **/ wildcard) — a bare key like "controller_server"
# only matches a root-namespace node. RewrittenYaml's root_key looked like
# the "correct" nav2 mechanism for this, but empirically it does NOT nest
# the file the way this needed (verified: with root_key=ns, robot2's
# controller_server silently fell back to nav2's built-in DWB default
# instead of the RegulatedPurePursuitController this file actually
# specifies) — so, same as ros2_controllers_sim.yaml and the EKF configs,
# rewrite these directly with a wildcard instead: "**/<key>", NOT /**/<key>.
# - No leading slash: rcl resolves a params-file key as <node
#   namespace>/<key>, so a leading slash produces a double slash
#   ("/amiga2//**/...") that rcl's arg parser rejects outright — confirmed:
#   with the leading slash, every Nav2 node for robot2 crashed at startup
#   with RCLInvalidROSArgsError.
# - Quoted: plain YAML treats a leading "*" as an alias reference, so
#   **/key: unquoted is a YAML syntax error ("while scanning an alias"),
#   not a wildcard — confirmed by launch itself failing to parse the file.
NAV2_PARAMS_TOP_LEVEL_KEYS = [
    "bt_navigator",
    "controller_server",
    "local_costmap",
    "global_costmap",
    "planner_server",
    "collision_monitor",
]


def namespace_nav2_params(content: str, ns: str) -> str:
    if not ns:
        return content
    for topic in ABSOLUTE_NAV2_TOPICS:
        content = content.replace(topic, f"/{ns}{topic}")
    for key in NAV2_PARAMS_TOP_LEVEL_KEYS:
        content = re.sub(rf"(?m)^{re.escape(key)}:", f'"**/{key}":', content)
    return content


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("namespace").perform(context)

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    amiga_navigation_dir = get_package_share_directory("amiga_navigation")
    params_dir = os.path.join(amiga_navigation_dir, "config")
    nav2_params = os.path.join(params_dir, "nav2_params.yaml")

    if ns:
        with open(nav2_params) as f:
            resolved_params = namespace_nav2_params(f.read(), ns)
        tmp_params = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", prefix=f"{ns}_nav2_params_", delete=False
        )
        tmp_params.write(resolved_params)
        tmp_params.flush()
        nav2_params = tmp_params.name

    # root_key intentionally always "" (a no-op) — see NAV2_PARAMS_TOP_LEVEL_KEYS
    # comment above for why; the "**/" rewrite in namespace_nav2_params() is
    # what actually makes robot2's sections match its namespaced nodes.
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    # TODO: RViz and MapViz

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "True",
            "log_level": "info",
        }.items(),
    )

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            configured_params,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        # nav2_bringup applies this remap to every node IT creates, but this
        # node is created directly by us, outside that mechanism — without
        # it, tf2_ros's hardcoded-absolute /tf would bypass namespacing.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    collision_monitor_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="collision_monitor_lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            {"autostart": True},
            {"node_names": ["collision_monitor"]},
        ],
    )

    nav_actions = [nav2_launch, collision_monitor_node, collision_monitor_lifecycle]

    if ns:
        # Real namespacing (nav2_bringup's own `navigation_launch.py` accepts
        # a `namespace` arg, but it's vestigial here — only affects
        # RewrittenYaml's root_key, irrelevant for this flat-keyed params
        # file, and the composable-container name) requires wrapping the
        # include the way nav2_bringup's own bringup_launch.py wraps
        # navigation_launch.py.
        return [GroupAction([PushRosNamespace(ns)] + nav_actions)]

    return nav_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulated clock (true when running under Gazebo)",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="ROS namespace for the whole Nav2 stack (per-robot, e.g. 'amiga2')",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

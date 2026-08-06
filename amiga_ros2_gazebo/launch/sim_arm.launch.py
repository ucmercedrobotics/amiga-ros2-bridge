"""Kinova arm stack for simulation — mirrors kortex_move/launch/robot.launch.py.

Identical to the hardware launch except:
  * NO ros2_control_node — the ign_ros2_control plugin inside Gazebo already
    provides a per-robot /<robot_name>/controller_manager, and the
    controllers (joint_trajectory_controller, robotiq_gripper_controller)
    are spawned by gazebo.launch.py with the same names as on the real robot.
    * NO fault_controller / twist_controller (kortex hardware only). The
        KinovaCommandVelocity action is therefore unavailable in simulation;
        MoveTo (move_group / pilz) and the gripper action work unchanged.
    * A sim-only joint-state filter republishes /<robot_name>/kinova/joint_states
        from the Gazebo joint-state stream, so MoveIt never sees the Amiga
        wheel joints.

Everything in this file (move_group, kinova robot_state_publisher,
joint_state_filter, rviz, moveto) is namespaced under the `robot_name`
launch argument (empty = unnamespaced, e.g. the single-robot layout when
robot_count==1), so it can be instantiated once per robot (see
sim_bringup.launch.py). The `moveto` convenience action server is the one
exception: kortex_move (a separate git submodule, out of scope for this
phase) hardcodes its action/service names (`/move_to`, `/gripper_control`)
as absolute, so two instances would collide regardless of node namespace —
it is only launched when `primary:=true` (robot1, whether or not it happens
to be namespaced). moveto still needs its OWN namespace set to match
move_group's (see the comment above its Node() below) so its
MoveGroupInterface and robot_description_semantic subscription find the
right instance; the two absolute action servers above are unaffected by
that namespace, so moveto's own public API (/move_to, /gripper_control)
never moves. One residual gap: moveto's downstream gripper action client
(`/robotiq_gripper_controller/gripper_cmd`, kortex_move/src/moveto.cpp) is
ALSO hardcoded absolute, so once robot1 is namespaced (robot_count>1) it
can no longer reach robot1's own (now namespaced) gripper controller —
`/move_to` arm motion still works, but moveto's `/gripper_control` action
will fail to actuate the gripper in that case. Fixing that requires a
kortex_move code change, out of scope here. A non-primary robot's arm is
still fully controllable via its own move_group (MoveGroupInterface / RViz
MotionPlanning panel), just not through this repo's `moveto` wrapper.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def qualify_ros(ns, path):
    """Absolute ROS name, namespaced under `ns` (ns="" leaves it unchanged)."""
    path = path.lstrip("/")
    return f"/{ns}/{path}" if ns else f"/{path}"


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveto = LaunchConfiguration("launch_moveto")
    ns = LaunchConfiguration("robot_name").perform(context)
    primary = LaunchConfiguration("primary").perform(context).lower() == "true"
    use_sim_time = {"use_sim_time": True}

    # Relative "from" patterns (not "/joint_states") so the remap matches
    # regardless of this node's own namespace: move_group/RSP subscribe to
    # the RELATIVE topic "joint_states", which resolves to /joint_states
    # only when ns="" (robot1) — for a namespaced robot it resolves to
    # /<ns>/joint_states BEFORE remapping, so an absolute "from" pattern
    # would silently never match and these nodes would read the raw,
    # unfiltered base+arm stream instead of the kinova-filtered one.
    kinova_remappings = [
        ("joint_states", qualify_ros(ns, "kinova/joint_states")),
        ("robot_description", qualify_ros(ns, "kinova/robot_description")),
    ]

    # Same description/mappings as kortex_move robot.launch.py, but with fake
    # hardware: the ros2_control block in this URDF is never instantiated
    # (physics lives in Gazebo); the description only feeds RSP + MoveIt.
    launch_arguments = {
        "robot_ip": "192.168.1.10",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "6",
        "gripper_max_velocity": "100.0",
        "gripper_max_force": "100.0",
        "use_internal_bus_gripper_comm": "true",
        "vision": "true",
    }

    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    moveit_config.moveit_cpp.update({"use_sim_time": True})

    kinematics_path = os.path.join(
        get_package_share_directory("kortex_move"), "config", "kinematics.yaml"
    )
    with open(kinematics_path, "r") as file:
        kinematics_yaml = yaml.safe_load(file)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=ns,
        output="screen",
        parameters=[moveit_config.to_dict(), kinematics_yaml, use_sim_time],
        remappings=kinova_remappings,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="kinova_robot_state_publisher",
        namespace=ns,
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
        remappings=kinova_remappings,
    )

    # Filter the sim-wide joint states down to the Kinova arm/gripper joints.
    joint_state_filter = Node(
        package="amiga_ros2_gazebo",
        executable="sim_joint_state_filter.py",
        name="kinova_joint_state_filter",
        namespace=ns,
        parameters=[
            use_sim_time,
            {"input_topic": qualify_ros(ns, "joint_states"), "mode": "kinova"},
        ],
        remappings=[
            ("/kinova/joint_states", qualify_ros(ns, "kinova/joint_states")),
        ],
        output="screen",
    )

    rviz_config_file = (
        get_package_share_directory("kinova_gen3_6dof_robotiq_2f_85_moveit_config")
        + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        namespace=ns,
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            use_sim_time,
        ],
        remappings=kinova_remappings,
    )

    actions = [robot_state_publisher, joint_state_filter, move_group_node, rviz_node]

    # kortex_move's moveto hardcodes /move_to, /gripper_control absolute —
    # only safe to run once, on the primary (robot1) instance. It still
    # needs namespace=ns (even though those two action *servers* stay
    # absolute regardless): moveto's MoveGroupInterface and its
    # robot_description_semantic subscription use RELATIVE names, which
    # must resolve under the same namespace as move_group_node's or moveto
    # times out waiting for the SRDF and crashes (only ever visible once
    # robot1 itself is namespaced, i.e. robot_count>1).
    if primary:
        actions.append(
            Node(
                package="kortex_move",
                executable="moveto",
                name="moveto",
                namespace=ns,
                output="screen",
                parameters=[use_sim_time],
                remappings=kinova_remappings,
                condition=IfCondition(launch_moveto),
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "launch_moveto",
                default_value="true",
                description="Start the kortex_move moveto node (as in production tmux)",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="",
                description="Namespace for this robot's arm stack (move_group, "
                "kinova RSP, joint_state_filter). Empty = unnamespaced.",
            ),
            DeclareLaunchArgument(
                "primary",
                default_value="true",
                description="Whether this is robot1 — the only instance that "
                "gets the kortex_move `moveto` node (see module docstring). "
                "Independent of `robot_name`: robot1 may or may not be "
                "namespaced depending on robot_count.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

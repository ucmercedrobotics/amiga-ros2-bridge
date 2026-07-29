"""Kinova arm stack for simulation — mirrors kortex_move/launch/robot.launch.py.

Identical to the hardware launch except:
  * NO ros2_control_node — the ign_ros2_control plugin inside Gazebo already
    provides the root-namespace /controller_manager, and the controllers
    (joint_trajectory_controller, robotiq_gripper_controller) are spawned by
    gazebo.launch.py with the same names as on the real robot.
    * NO fault_controller / twist_controller (kortex hardware only). The
        KinovaCommandVelocity action is therefore unavailable in simulation;
        MoveTo (move_group / pilz) and the gripper action work unchanged.
    * A sim-only joint-state filter republishes /kinova/joint_states from the
        Gazebo joint-state stream, so MoveIt never sees the Amiga wheel joints.

move_group and the kinova robot_state_publisher follow the hardware launch
semantics (same /kinova/* remappings), so kortex_move nodes (moveto,
kinova_motion_server, BT actions) run unchanged.
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


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveto = LaunchConfiguration("launch_moveto")
    use_sim_time = {"use_sim_time": True}

    kinova_remappings = [
        ("/joint_states", "/kinova/joint_states"),
        ("/robot_description", "/kinova/robot_description"),
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
        output="screen",
        parameters=[moveit_config.to_dict(), kinematics_yaml, use_sim_time],
        remappings=kinova_remappings,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="kinova_robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, use_sim_time],
        remappings=kinova_remappings,
    )

    # Filter the sim-wide joint states down to the Kinova arm/gripper joints.
    joint_state_filter = Node(
        package="amiga_ros2_gazebo",
        executable="sim_joint_state_filter.py",
        name="kinova_joint_state_filter",
        parameters=[use_sim_time, {"input_topic": "/joint_states", "mode": "kinova"}],
        output="screen",
    )

    moveto_node = Node(
        package="kortex_move",
        executable="moveto",
        name="moveto",
        output="screen",
        parameters=[use_sim_time],
        remappings=kinova_remappings,
        condition=IfCondition(launch_moveto),
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

    return [
        robot_state_publisher,
        joint_state_filter,
        move_group_node,
        moveto_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "launch_moveto",
                default_value="true",
                description="Start the kortex_move moveto node (as in production tmux)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
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
    launch_segment_leaves_sim = LaunchConfiguration("launch_segment_leaves_sim")
    ns = LaunchConfiguration("robot_name").perform(context)
    use_sim_time = {"use_sim_time": True}

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

    # Resolved here rather than passed as substitutions because an EMPTY list
    # cannot survive the trip: `fail_goals: []` reaches the node as "no
    # parameter value set" and rclcpp throws InvalidParameterValueException
    # against the declared vector<int64_t>, killing the server on every robot
    # that was meant to be healthy. So the healthy case passes no failure
    # parameters at all and lets FailurePolicy's own defaults disable it.
    sampler_fail_goals = [
        int(v)
        for v in LaunchConfiguration("sampler_fail_goals")
        .perform(context)
        .strip("[] ")
        .split(",")
        if v.strip()
    ]
    sampler_failure_params = (
        [
            {
                "fail_goals": sampler_fail_goals,
                "fail_after_n": int(
                    LaunchConfiguration("sampler_fail_after_n").perform(context)
                ),
                "failure_mode": LaunchConfiguration("sampler_failure_mode").perform(
                    context
                ),
            }
        ]
        if sampler_fail_goals
        else []
    )

    actions.append(
        Node(
            package="amiga_ros2_behavior_tree",
            executable="sim_segment_leaves_server",
            name="segment_leaves_sim",
            namespace=ns,
            output="screen",
            # sampler_failure_mode only bites when sampler_fail_goals is
            # non-empty, which is not the default -- see FailurePolicy in
            # mocks/failure_modes.hpp. Passed through so a fleet scenario can
            # break ONE robot's arm while the orchard stays intact for
            # everybody, which is the difference between a task worth offering
            # to a peer and one worth dropping.
            parameters=[use_sim_time, *sampler_failure_params],
            condition=IfCondition(launch_segment_leaves_sim),
        )
    )

    home_topic = qualify_ros(ns, "joint_trajectory_controller/joint_trajectory")
    home_goal = (
        "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], "
        "points: [{positions: [0.0, -0.785398, -2.0, 0.0, -0.436332, 1.5708], "
        "time_from_start: {sec: 3}}]}"
    )
    actions.append(
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "topic",
                        "pub",
                        "--once",
                        home_topic,
                        "trajectory_msgs/msg/JointTrajectory",
                        home_goal,
                    ],
                    output="screen",
                )
            ],
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
                "launch_segment_leaves_sim",
                default_value="true",
                description="Start the simulated `segment_leaves` action server "
                "(steps the arm forward, closes the gripper, returns home) in "
                "place of the real depth-camera/YOLO leaf-segmentation node, "
                "which has nothing to see in Gazebo. Requires launch_moveto.",
            ),
            DeclareLaunchArgument(
                "sampler_fail_goals",
                default_value="[]",
                description="Which segment_leaves goals fail. Empty (default) "
                "disables the failure path entirely. SegmentLeaves takes no "
                "arguments, so there is nothing to key on and the only "
                "meaningful value is [0], meaning this server's goals.",
            ),
            DeclareLaunchArgument(
                "sampler_fail_after_n",
                default_value="0",
                description="Succeed this many goals before failing. 0 fails "
                "from the first, which is what a broken camera looks like.",
            ),
            DeclareLaunchArgument(
                "sampler_failure_mode",
                default_value="no_point_cloud",
                description="Which real failure of the arm to reproduce, in "
                "the real node's own words: no_point_cloud (the depth camera "
                "produced nothing -- a fault in THIS robot, so a peer with a "
                "working camera is the right answer) or no_leaves (the camera "
                "worked and there was nothing there -- nobody else would find "
                "leaves either, so that one is permanent).",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="",
                description="Namespace for this robot's arm stack (move_group, "
                "kinova RSP, joint_state_filter, moveto). Empty = unnamespaced.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

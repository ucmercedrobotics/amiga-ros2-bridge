import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _include(package, launch_file, **launch_args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        launch_arguments={k: v for k, v in launch_args.items()}.items(),
    )


def qualify_ros(ns, topic):
    """Absolute topic name, namespaced under `ns` (ns="" leaves it unchanged)."""
    topic = topic.lstrip("/")
    return f"/{ns}/{topic}" if ns else f"/{topic}"


# Everything the behaviour-tree schema permits. Passed explicitly rather than
# letting each coordinator read the schema itself, because in a simulated fleet
# the interesting scenarios are the asymmetric ones -- one robot with no arm
# stays silent on a sampling task instead of bidding and losing. Override per
# robot by editing this list; the coordinator refuses an unknown name.
ALL_CAPABILITIES = [
    "MoveToTreeID",
    "MoveToAisleHead",
    "MoveToGPSLocation",
    "ApproachGPSWaypoint",
    "MoveToRelativeLocation",
    "OrientRobotHeading",
    "FollowPerson",
    "SampleLeaf",
    "MoveArmToPosition",
    "Wait",
]


def coordination_nodes(
    ns, node_id, device, spreading_factor, battery_percent=100, use_agents=False
):
    """This robot's radio bridge, its mission bridge and its coordinator.

    All namespaced, which is the whole of what multi-robot needs from them:
    ReliabilityNode already publishes and subscribes the *relative* `lora/tx`
    and `lora/rx`, so a namespaced instance lands on its own bridge with no
    remapping. The coordinator executable runs three nodes in one process and
    launch's `namespace=` covers all of them.

    What has to be qualified by hand is every parameter whose default is
    absolute -- `/coordination/infeasible`, the two service names, the orchard
    topic. Those defaults are right for one robot, where the agent stack sits
    outside any namespace; on one machine they mean a single escalation makes
    the whole fleet shed the same task, and every robot resolving trees against
    whichever orchard was published last.
    """
    return [
        Node(
            package="amiga_ros2_comms",
            executable="lora_bridge",
            name="lora_bridge",
            namespace=ns,
            output="screen",
            parameters=[{"use_sim_time": True, "serial_port": device}],
        ),
        # Answers the coordinator's mission questions off /mission/xml, and
        # turns a won task into a candidate for the arbiter. Without it the
        # coordinator's mission port never hears anything and this robot never
        # bids -- which is the deliberate safe default, not a failure.
        Node(
            package="amiga_ros2_agents",
            executable="mission_bridge",
            name="mission_bridge",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    # Both relative, so this robot's namespace places them: the
                    # plan it reads is the one its own bt_runner executes and
                    # its own arbiter writes (see robot_agents.launch.py).
                    "mission_topic": "mission/xml",
                    "state_topic": qualify_ros(ns, "mission/coordination_state"),
                    "battery_percent": battery_percent,
                }
            ],
        ),
        # Deliberately no `name=`: this executable runs the coordinator, its
        # in-process reliability layer and the node holding its port
        # subscriptions, and a name remapping would rename all of them to the
        # same thing. Same reasoning as coordinator.launch.py.
        #
        # coordinator_sim rather than coordinator: the plain executable wires
        # ports that decline every question, so a fleet of those would announce
        # and never bid. See sim_node.py.
        Node(
            package="amiga_ros2_coordinator",
            executable="coordinator_sim",
            namespace=ns,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "node_id": node_id,
                    "spreading_factor": spreading_factor,
                    "capabilities": ALL_CAPABILITIES,
                    "infeasible_topic": qualify_ros(ns, "coordination/infeasible"),
                    "triage_service": qualify_ros(ns, "coordination/interpret_anomaly"),
                    "note_service": qualify_ros(ns, "coordination/interpret_note"),
                    "mission_state_topic": qualify_ros(
                        ns, "mission/coordination_state"
                    ),
                    "gps_topic": qualify_ros(ns, "gps/pvt"),
                    # Published by this robot's own tcp_demux, so per-robot even
                    # though every robot is sent the same orchard.
                    "orchard_topic": qualify_ros(ns, "orchard/tree_info_json"),
                    "verify_replan_service": qualify_ros(ns, "mission/verify_replan"),
                    # Both reasoning points follow the agent stack. Left true
                    # with no agent running, every escalation waits out
                    # triage_timeout_sec (45 s) and then changes nothing, which
                    # looks exactly like a coordinator that ignored it.
                    "use_triage_agent": use_agents,
                    "use_note_agent": use_agents,
                }
            ],
        ),
    ]


# tf2_ros hardcodes /tf, /tf_static as absolute regardless of node namespace
# — every node here that might touch TF gets this remap, same as everywhere
# else in this repo's sim launch files.
def tf_remaps(ns):
    return [
        ("/tf", qualify_ros(ns, "tf")),
        ("/tf_static", qualify_ros(ns, "tf_static")),
    ]


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    name_prefix = LaunchConfiguration("robot_name_prefix").perform(context)
    use_lidar = LaunchConfiguration("use_lidar")
    use_gps = LaunchConfiguration("use_gps")
    launch_rviz = LaunchConfiguration("launch_rviz")
    yaw_offset = LaunchConfiguration("yaw_offset")
    mission_port_base = int(LaunchConfiguration("mission_port_base").perform(context))

    launch_nav = LaunchConfiguration("launch_nav").perform(context).lower() == "true"
    launch_arm = LaunchConfiguration("launch_arm").perform(context).lower() == "true"
    launch_helpers = (
        LaunchConfiguration("launch_helpers").perform(context).lower() == "true"
    )
    launch_bt = LaunchConfiguration("launch_bt").perform(context).lower() == "true"
    launch_coordination = (
        LaunchConfiguration("launch_coordination").perform(context).lower() == "true"
    )
    ltl_verification = LaunchConfiguration("ltl_verification").perform(context).lower()
    objective_gating = LaunchConfiguration("objective_gating").perform(context).lower()
    launch_agents = (
        LaunchConfiguration("launch_agents").perform(context).lower() == "true"
    )
    launch_vlm = LaunchConfiguration("launch_vlm").perform(context).lower()
    vlm_url = LaunchConfiguration("vlm_url").perform(context)
    # 0 (default) means nobody: an ordinary sim run has no broken arm. Set it
    # to a robot index to give exactly that robot a camera fault, which is the
    # kind of failure a peer can take over -- unlike a missing tree, which no
    # robot could reach and which triage therefore drops instead of auctioning.
    broken_sampler_robot = int(
        LaunchConfiguration("broken_sampler_robot").perform(context)
    )
    # 0 (default) means an empty orchard. A tree index puts a standing person
    # on that tree's row waypoint -- the pose Nav2 aims at before handing over
    # to the lidar approach, and the only spot in the aisle where one body is
    # enough to abort a goal rather than be driven around.
    spawn_person = int(LaunchConfiguration("spawn_person").perform(context))
    planner_host = LaunchConfiguration("planner_host").perform(context)
    broken_sampler_mode = LaunchConfiguration("broken_sampler_mode").perform(context)
    symlink_dir = LaunchConfiguration("lora_symlink_dir").perform(context)
    spreading_factor = int(
        LaunchConfiguration("lora_spreading_factor").perform(context)
    )
    # One entry per robot, short list padded with 100. Batteries are the
    # cheapest way to make a fleet asymmetric: they enter default_fitness as a
    # penalty below 50%, so `batteries:=100,20,100` makes robot 2 bid badly on
    # everything without changing where any robot is.
    batteries = [
        int(value)
        for value in LaunchConfiguration("batteries").perform(context).split(",")
        if value.strip()
    ]
    batteries += [100] * max(0, robot_count - len(batteries))

    actions = [
        # Force sim clock on every node started below (includes too).
        SetParameter(name="use_sim_time", value=True),
        # ── Hardware layer replacement for all robot_count robots — see
        # gazebo.launch.py.
        _include(
            "amiga_ros2_gazebo",
            "gazebo.launch.py",
            world=world,
            headless=headless,
            robot_count=str(robot_count),
            robot_name_prefix=name_prefix,
        ),
        _include(
            "amiga_ros2_gazebo",
            "sim_hardware_shims.launch.py",
            robot_count=str(robot_count),
            robot_name_prefix=name_prefix,
        ),
    ]

    # ── One virtual radio medium for the whole fleet ──────────────────────
    # Started once, not per robot: it *is* the shared channel, and modelling
    # contention between robots is most of its point. bridges:=false because
    # each robot's bridge is started below, in that robot's own namespace.
    if launch_coordination:
        actions.append(
            _include(
                "amiga_ros2_comms",
                "lora_sim.launch.py",
                robots=",".join(f"{name_prefix}{i}" for i in range(1, robot_count + 1)),
                symlink_dir=symlink_dir,
                spreading_factor=str(spreading_factor),
                bridges="false",
            )
        )

    for i in range(1, robot_count + 1):
        # robot1 is unnamespaced only when it's the sole robot — see module
        # docstring. Namespace only, not spawn name (that's gazebo.launch.py's
        # own robot1_name arg, not threaded through here, same as before this
        # file generalized to N robots).
        ns = "" if (i == 1 and robot_count == 1) else f"{name_prefix}{i}"
        # Matches the frame_prefix given to robot_state_publisher in
        # urdf.launch.py — needed anywhere a node's frame-name parameter
        # would otherwise default to a bare "base_link"/"lidar_link" that no
        # longer exists once this robot's own TF is prefixed.
        frame_prefix = f"{ns}/" if ns else ""

        # ── Base description/TF (identical-to-hardware stack) ─────────────
        # publish_joints:=false because the joint_state_broadcaster is the
        # source, filtered to base joints.
        actions += [
            Node(
                package="amiga_ros2_gazebo",
                executable="sim_joint_state_filter.py",
                name="amiga_base_joint_state_filter",
                namespace=ns,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": qualify_ros(ns, "joint_states"),
                        "mode": "amiga",
                    }
                ],
                # /amiga/joint_states is hardcoded absolute in
                # sim_joint_state_filter.py's own output (not a param);
                # no-op remap when ns="" (qualify_ros returns the same string).
                remappings=[
                    ("/amiga/joint_states", qualify_ros(ns, "amiga/joint_states")),
                ],
            ),
            _include(
                "amiga_ros2_description",
                "urdf.launch.py",
                publish_joints="false",
                namespace=ns,
                joint_states_topic=qualify_ros(ns, "amiga/joint_states"),
                use_lidar=use_lidar,
                gps_link_name="gps_antenna",
                use_vectornav="false",
            ),
        ]

        # ── Localization + Nav2 ────────────────────────────────────────────
        if launch_nav:
            actions.append(
                _include(
                    "amiga_localization",
                    "bringup.launch.py",
                    use_vectornav="false",
                    use_gps=use_gps,
                    gps_topic=qualify_ros(ns, "gps/pvt"),
                    # Explicit, not omitted: LaunchConfiguration("namespace")
                    # is global across the whole launch tree, not scoped per
                    # include — an omitted arg picks up whatever an earlier
                    # include last set it to, rather than this file's own
                    # default. Always pass it explicitly at every call site
                    # that reaches a file declaring a "namespace" arg.
                    namespace=ns,
                )
            )
            actions.append(
                _include(
                    "amiga_navigation",
                    "navigation.launch.py",
                    use_sim_time="True",
                    namespace=ns,
                    # The Gazebo model is slightly larger than the real Amiga,
                    # so the real robot's footprint/collision polygons are too
                    # tight here and Nav2 racks up phantom-looking collisions
                    # that don't happen on hardware. This sim-only copy (see
                    # amiga_navigation/config/nav2_params_sim.yaml) enlarges
                    # just those bounding boxes; the real bringup path
                    # (amiga_navigation/navigation.launch.py's own default)
                    # is untouched.
                    params_file=os.path.join(
                        get_package_share_directory("amiga_navigation"),
                        "config",
                        "nav2_params_sim.yaml",
                    ),
                )
            )

        if launch_arm:
            actions.append(
                _include(
                    "amiga_ros2_gazebo",
                    "sim_arm.launch.py",
                    launch_rviz=launch_rviz,
                    robot_name=ns,
                    # Empty for every robot except broken_sampler_robot, so at
                    # most one robot's arm is faulty and the rest of the fleet
                    # can still take the work it sheds.
                    sampler_fail_goals=("[0]" if i == broken_sampler_robot else "[]"),
                    sampler_failure_mode=broken_sampler_mode,
                )
            )

        # ── Helper nodes mirroring the production tmux session ────────────
        if launch_helpers:
            actions += [
                Node(
                    package="amiga_navigation",
                    executable="waypoint_follower.py",
                    name="amiga_waypoint_follower",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "yaw_offset": yaw_offset,
                            "orchard_tree_service": qualify_ros(
                                ns, "orchard/get_tree_info"
                            ),
                        }
                    ],
                    # /gps/filtered, /odometry/filtered/local: hardcoded
                    # absolute subscriptions, no param. /navigate_via_lidar:
                    # hardcoded absolute ActionClient (not a param, unlike
                    # its own action servers) — without this remap it would
                    # always dial the unnamespaced top-level action instead
                    # of this robot's own lidar_object_navigator below.
                    remappings=tf_remaps(ns)
                    + [
                        ("/gps/filtered", qualify_ros(ns, "gps/filtered")),
                        (
                            "/odometry/filtered/local",
                            qualify_ros(ns, "odometry/filtered/local"),
                        ),
                        ("/navigate_via_lidar", qualify_ros(ns, "navigate_via_lidar")),
                    ],
                ),
                Node(
                    package="amiga_navigation",
                    executable="linear_velo",
                    name="linear_velo",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "min_linear_velocity": 0.3,
                            "max_linear_velocity": 1.0,
                            "yaw_slowdown": 0.8,
                            "max_angular_velocity": 0.7,
                            "yaw_offset": yaw_offset,
                            "odom_topic": qualify_ros(ns, "odometry/filtered/local"),
                        }
                    ],
                    # /cmd_vel: hardcoded absolute publisher, no param.
                    remappings=tf_remaps(ns)
                    + [("/cmd_vel", qualify_ros(ns, "cmd_vel"))],
                ),
                Node(
                    package="amiga_navigation",
                    executable="lidar_object_navigator",
                    name="lidar_object_navigator",
                    namespace=ns,
                    output="screen",
                    parameters=[
                        {
                            "safety_distance": 2.5,
                            "lidar_topic": qualify_ros(ns, "ouster/points"),
                            "azimuth_tolerance": 0.8,
                            "min_object_height": 0.1,
                            "max_object_height": 1.5,
                            "min_object_distance": 1.0,
                            "max_object_distance": 5.0,
                            # base_frame/lidar_link default to bare
                            # "base_link"/"lidar_link" in the node itself,
                            # which stopped existing on this robot's own
                            # /<ns>/tf the moment robot_state_publisher got a
                            # frame_prefix (see urdf.launch.py) — every TF
                            # lookup here would fail without this.
                            "base_frame": f"{frame_prefix}base_link",
                            "lidar_link": f"{frame_prefix}lidar_link",
                        }
                    ],
                    remappings=tf_remaps(ns),
                ),
            ]

        # ── Mission/behavior-tree layer ────────────────────────────────────
        if launch_bt:
            actions.append(
                _include(
                    "amiga_ros2_behavior_tree",
                    "bt.launch.py",
                    namespace=ns,
                    port=str(mission_port_base + i - 1),
                    planner_host=planner_host,
                )
            )

        # ── Robot-to-robot coordination ────────────────────────────────────
        # The device name is always amiga<i>, independent of ns -- robot1's
        # namespace is "" and cannot name a pty. node_id is i, and it is the
        # one value here with no safe default.
        if launch_coordination:
            actions += coordination_nodes(
                ns=ns,
                node_id=i,
                device=f"{symlink_dir}/{name_prefix}{i}",
                spreading_factor=spreading_factor,
                battery_percent=batteries[i - 1],
                use_agents=launch_agents,
            )

        # ── This robot's own LLM agents ────────────────────────────────────
        # A stack per robot, not one for the fleet: the whole point of the
        # scenario is that robot A's diagnosis and robot B's decision to take
        # the work on are different judgements made by different agents from
        # different context. Sharing one stack would make them the same call.
        #
        # mission_bridge is excluded because coordination_nodes above starts
        # one -- it has no model and the coordinator is inert without it, so it
        # belongs to the coordination layer here rather than to the agents.
        if launch_agents:
            actions.append(
                _include(
                    "amiga_ros2_agents",
                    "robot_agents.launch.py",
                    namespace=ns,
                    use_sim_time="true",
                    launch_mission_bridge="false",
                    battery_percent=str(batteries[i - 1]),
                    ltl_verification=ltl_verification,
                    objective_gating=objective_gating,
                    launch_vlm=launch_vlm,
                    vlm_url=vlm_url,
                    # Relative, and the same for every robot: the namespace is
                    # what makes robot 2 look through robot 2's camera. This is
                    # the topic sim_hardware_shims republishes the Gazebo front
                    # camera on, so it is the same name the real Oak-D driver
                    # publishes and nothing below the shim layer changes.
                    vlm_image_topic="oak0/rgb/image_raw",
                )
            )

    if spawn_person:
        actions.append(
            ExecuteProcess(
                cmd=[
                    os.path.join(
                        get_package_prefix("amiga_ros2_gazebo"),
                        "lib", "amiga_ros2_gazebo", "spawn_person.py",
                    ),
                    "--tree", str(spawn_person),
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(
                    get_package_share_directory("amiga_ros2_gazebo"),
                    "worlds",
                    "orchard_nbv.sdf",
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "robot_count",
                default_value="1",
                description="How many identical robots to bring up (see "
                "module docstring). make sim-dual sets this to 2, "
                "make sim-multi to $(ROBOT_COUNT).",
            ),
            DeclareLaunchArgument(
                "robot_name_prefix",
                default_value="amiga",
                description="Name/namespace prefix for robots 2..N, and for "
                "robot1 too once robot_count>1",
            ),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_gps", default_value="true"),
            DeclareLaunchArgument("launch_nav", default_value="true"),
            DeclareLaunchArgument("launch_arm", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument(
                "launch_helpers",
                default_value="true",
                description="Start waypoint_follower + linear_velo (as in tmux bringup)",
            ),
            DeclareLaunchArgument("launch_bt", default_value="true"),
            DeclareLaunchArgument(
                "planner_host",
                default_value="",
                description="Fleet planner to register and heartbeat with. "
                "Empty (the default here) disables discovery, because a sim is "
                "driven by netcat straight into each robot's mission port and "
                "has no planner to answer -- bt.launch.py's own default points "
                "at the real fleet host, which on a dev box just refuses every "
                "5s per robot. Set it to that host to opt back in.",
            ),
            DeclareLaunchArgument(
                "broken_sampler_robot",
                default_value="0",
                description="Give this robot (1-based) a failing leaf sampler; "
                "0, the default, breaks nobody. The orchard stays intact for "
                "everyone, so the tree it cannot sample is one its peers can "
                "still reach -- which is what makes the shed task worth "
                "auctioning rather than dropping.",
            ),
            DeclareLaunchArgument(
                "broken_sampler_mode",
                default_value="no_point_cloud",
                description="How that robot's sampler fails. no_point_cloud is "
                "a fault in that robot (a peer with a working camera is the "
                "right answer); no_leaves is permanent for everyone.",
            ),
            DeclareLaunchArgument(
                "launch_coordination",
                default_value="true",
                description="Start the robot-to-robot layer: one virtual LoRa "
                "medium for the fleet, plus a bridge and a coordinator per "
                "robot. With robot_count:=1 this is a fleet of one -- it comes "
                "up, heartbeats into an empty channel and has nobody to trade "
                "with, which is harmless and shows the stack is wired.",
            ),
            DeclareLaunchArgument(
                "launch_agents",
                default_value="false",
                description="Start a full LLM agent stack per robot (world "
                "state, arbiter, mission planner, triage). Off by default "
                "because every one of them needs a model endpoint -- set "
                "AGENT_MODEL/AGENT_API_BASE first, see amiga_ros2_agents. It "
                "also switches the coordinators' use_triage_agent, so with this "
                "false they fall back to the local stub interpreter rather than "
                "waiting 45 s on a service nobody serves.",
            ),
            DeclareLaunchArgument(
                "spawn_person",
                default_value="0",
                description="Tree index to put a standing person in front of, "
                "or 0 for none. The person lands on that tree's row waypoint, "
                "which makes MoveToTreeID abort for real instead of routing "
                "around -- the fault the triage agent and the VLM read.",
            ),
            DeclareLaunchArgument(
                "launch_vlm",
                default_value="false",
                description="Give each robot a vlm_server, so every failure "
                "carries a description of what that robot's camera saw into "
                "its triage decisions. Needs launch_agents too -- triage is "
                "what asks -- and a vision model on vlm_url: a separate model "
                "and endpoint from the one the agents reason with. Off by "
                "default.",
            ),
            DeclareLaunchArgument(
                "vlm_url",
                default_value="http://localhost:8001/v1/chat/completions",
                description="OpenAI-compatible endpoint accepting image content "
                "parts. One endpoint serves the whole simulated fleet.",
            ),
            DeclareLaunchArgument(
                "lora_symlink_dir",
                default_value="/tmp/amiga_lora_sim",
                description="Where the virtual radio's per-robot ptys are "
                "symlinked. Robot i's bridge opens <dir>/<prefix><i>.",
            ),
            DeclareLaunchArgument(
                "batteries",
                default_value="",
                description="Comma-separated battery percent per robot, e.g. "
                "'100,20,100'. Short lists are padded with 100. Below 50% "
                "enters the bid as a cost penalty, so this is the cheapest way "
                "to make a fleet bid asymmetrically without moving anyone.",
            ),
            DeclareLaunchArgument(
                "ltl_verification",
                default_value="true",
                description="False drops the arbiter's formal gate: no formula "
                "is generated and SPIN never runs. Plans are still checked for "
                "whether they RUN (XSD + the ontology's required "
                "preconditions) and for whether they still contain the "
                "mission's work (see objective_gating). For bringing the "
                "coordination loop up end to end; every accept is then "
                "reported unverified.",
            ),
            DeclareLaunchArgument(
                "objective_gating",
                default_value="true",
                description="The arbiter's objective-preservation and "
                "viability checks, and with them its ability to ABORT. "
                "/mission/abort is what ends the local repair loop and hands "
                "the fault to the coordinator, so a fleet run needs this on "
                "regardless of ltl_verification. False makes the local loop "
                "endless and nothing is ever auctioned.",
            ),
            DeclareLaunchArgument(
                "lora_spreading_factor",
                default_value="7",
                description="LoRa spreading factor, 6..12. Time on air doubles "
                "per step, so this is the main dial on how much coordination "
                "traffic the simulated fleet can actually sustain.",
            ),
            DeclareLaunchArgument(
                "mission_port_base",
                default_value="12346",
                description="TCP mission port for robot1; robot i gets "
                "mission_port_base + (i-1) (see module docstring).",
            ),
            DeclareLaunchArgument(
                "yaw_offset",
                default_value="0.0",
                description="Heading offset for waypoint helpers; sim GPS/IMU "
                "are ENU-consistent so 0.0 (production uses 1.57 for the real "
                "compass mounting)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

"""One robot's complete agent stack, under that robot's namespace.

    ros2 launch amiga_ros2_agents robot_agents.launch.py
    ros2 launch amiga_ros2_agents robot_agents.launch.py namespace:=amiga2

`agents.launch.py` starts one stack for one robot and is what the real machine
runs. This is the same set placed under a namespace, so a simulated fleet can
run a stack per robot: each robot then diagnoses its own faults, gates its own
plan edits, and escalates to its own coordinator.

Why a remap table and not relative names
----------------------------------------
Every node in this package names its topics and services absolutely --
`/mission/xml`, `/bt/status_change`, `/world_state`. That is right for one
robot, where the agent stack sits outside any namespace, and it is the reason
the package README says those names have to change before a fleet can run one
stack per robot. `ROBOT_INTERFACES` below is the launch-side answer to that: it
remaps each absolute name to its *relative* form, and the node's own namespace
places it. Cost of being wrong is low and visible -- a missing entry means one
subscription still listening to the fleet-wide name, which shows up immediately
as every robot reacting to one robot's fault.

Making the source names relative is still the real fix. This is what lets the
fleet run before that lands, without editing five node files.

`/rosout` is deliberately *not* remapped. There is one log stream per machine,
not one per robot, and the planner and triage both read it for the explanation
behind a fault -- remapping it to a topic nobody publishes would quietly empty
their most useful context. In a simulated fleet they therefore see each other's
log lines, which is noise in the prompt and nothing worse.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = "amiga_ros2_agents"

# world_state first, so the planner's /world_state window has frames in it by
# the time the first BT failure arrives. mission_bridge last: it is not an agent
# and has no model, but it belongs to the same per-robot set.
AGENTS = ["world_state", "arbiter", "mission_planner", "triage", "note"]

#: Every absolute name any node in this package publishes, subscribes, serves or
#: calls, except /rosout (see the module docstring). Grouped by who owns it so a
#: node added later has an obvious place to declare itself.
ROBOT_INTERFACES = [
    # The plan and the loop that edits it.
    "/mission/xml",
    "/mission/candidate_xml",
    "/mission/rejection",
    "/mission/abort",
    "/mission/planner_status",
    "/mission/viability_budget",
    "/mission/verify_replan",
    "/mission/replan_request",
    "/mission/fault_route",
    "/mission_status",
    # The tree.
    "/bt/status_change",
    # What the robot is doing, aggregated.
    "/world_state",
    # The orchard the robot was fed with its mission. Missing from this table
    # through a live fleet run, and the symptom was quiet: the arbiter and the
    # planner both subscribed to the fleet-wide name, which nothing publishes,
    # so `orchard` stayed None in every agent. mission_tasks.synthesize then
    # could not place the aisle move for a task won at auction -- it reported
    # MoveToAisleHead as `dropped` and left the winner's planner to reinvent it
    # from the rows already in its own plan, which it did, correctly, at the
    # cost of a model call and a step nobody could see was missing.
    "/orchard/tree_info_json",
    "/navigate_to_pose/_action/feedback",
    "/navigate_to_pose/_action/status",
    "/follow_tree_id_waypoint/_action/feedback",
    "/follow_tree_id_waypoint/_action/status",
    "/segment_leaves/_action/feedback",
    "/segment_leaves/_action/status",
    # The coordinator's three doors.
    "/coordination/infeasible",
    "/coordination/interpret_anomaly",
    "/coordination/interpret_note",
    # The camera, for triage. A client service name is remapped like any other
    # interface, so triage's absolute default lands on this robot's own VLM and
    # not on whichever one in the fleet happens to answer first -- which would
    # be worse than no camera at all, since a description of another robot's
    # view is evidence that reads as true and is not.
    "/vlm/ask",
]

#: StatusPublisher builds /agents/<node name>/status from the node's *name*,
#: which a namespace does not change -- so three arbiters would publish one
#: robot's readiness three times over.
STATUS_TOPICS = [f"/agents/{agent}/status" for agent in AGENTS + ["mission_bridge"]]


def robot_remaps():
    """Absolute name -> the same name relative, for the node's namespace to place.

    A rule a node has no matching interface for is inert, so every node gets the
    whole table rather than a hand-maintained subset per node.
    """
    return [(name, name.lstrip("/")) for name in ROBOT_INTERFACES + STATUS_TOPICS]


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = {
        "namespace": namespace,
        "output": "screen",
        "remappings": robot_remaps(),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace. Empty gives exactly the layout "
                "agents.launch.py produces.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "ltl_verification",
                default_value="true",
                description="False makes the arbiter gate on whether a plan "
                "will RUN -- well-formed XML, the XSD, and the ontology's "
                "required preconditions -- and skip the checks that decide "
                "whether it is still the mission that was asked for: no "
                "formula, no SPIN, no viability budget, no edit-size or rate "
                "limit. For bringing the coordination loop up end to end. "
                "Every accept is then reported unverified.",
            ),
            DeclareLaunchArgument(
                "objective_gating",
                default_value="true",
                description="Whether the arbiter checks that a candidate still "
                "contains the mission's objectives, and aborts when too few "
                "survive. This is the only check that can publish "
                "/mission/abort, which is what ends the local repair loop and "
                "escalates to the coordinator -- so a fleet run wants it on "
                "even when ltl_verification is off. False only to study the "
                "formal gate on its own.",
            ),
            DeclareLaunchArgument(
                "launch_mission_bridge",
                default_value="true",
                description="Start the mission bridge alongside the agents. "
                "False when something else already runs one for this robot -- "
                "sim_bringup.launch.py starts it with the coordination layer, "
                "since it has no model and the coordinator is inert without it.",
            ),
            DeclareLaunchArgument(
                "launch_vlm",
                default_value="false",
                description="Start this robot's vlm_server. Every failure "
                "then carries a description of what its camera saw at that "
                "moment into triage's decisions. Needs a vision model on "
                "vlm_url -- a separate model and endpoint from the one the "
                "agents reason with. Off by default; with it off triage "
                "decides from logs exactly as before.",
            ),
            DeclareLaunchArgument(
                "vlm_url",
                default_value="http://localhost:8001/v1/chat/completions",
                description="OpenAI-compatible chat-completions endpoint that "
                "accepts image content parts.",
            ),
            DeclareLaunchArgument(
                "vlm_image_topic",
                default_value="oak0/rgb/image_raw",
                description="Camera this robot's VLM looks through. Relative, "
                "so the namespace places it -- absolute here would point every "
                "robot in a simulated fleet at robot 1's camera.",
            ),
            DeclareLaunchArgument(
                "describe_frame",
                default_value="false",
                description="Have the camera describe the image itself -- "
                "glare, washout -- as well as its contents. Off by default: it "
                "also makes the robot's own arm much more likely to be "
                "described, which reads as an obstruction.",
            ),
            DeclareLaunchArgument(
                "camera_description",
                default_value="the front camera",
                description="How the routing prompt names the camera the "
                "description came from. Change it with vlm_image_topic or the "
                "prompt says the wrong device.",
            ),
            DeclareLaunchArgument(
                "vlm_static_image",
                default_value="",
                description="Answer camera questions from this file rather "
                "than the live topic. For a fault the simulator cannot stage; "
                "empty (default) uses the camera.",
            ),
            DeclareLaunchArgument(
                "battery_percent",
                default_value="100",
                description="What the mission bridge reports to the coordinator. "
                "A parameter rather than a measurement -- there is no battery in "
                "simulation, and it is the cheapest way to make a fleet bid "
                "asymmetrically without moving anyone.",
            ),
            # ltl_gen is deliberately absent. The arbiter runs the same gate
            # in-process (ltl_gate.py, sharing ltl.py), because a formula is
            # needed *inside* the decision that publishes /mission/xml and a
            # service call from there would make the gate depend on another node
            # being up. ltl_gen exposes that translation to outside callers, and
            # a fleet does not need one per robot.
            *(
                Node(
                    package=PACKAGE,
                    executable=agent,
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            # Only the arbiter declares these; a parameter a node
                            # never declares is ignored, so passing them to all
                            # of them keeps this a one-line table entry rather
                            # than a special case in the loop.
                            "ltl_verification": ParameterValue(
                                LaunchConfiguration("ltl_verification"),
                                value_type=bool,
                            ),
                            "objective_gating": ParameterValue(
                                LaunchConfiguration("objective_gating"),
                                value_type=bool,
                            ),
                            # Only triage declares this one. Tied to the same
                            # flag that starts the server, so the two cannot
                            # disagree: a triage told to look with nothing to
                            # look through spends its discovery timeout on
                            # every fault and learns nothing.
                            "use_vlm": ParameterValue(
                                LaunchConfiguration("launch_vlm"), value_type=bool
                            ),
                            "camera_description": LaunchConfiguration(
                                "camera_description"
                            ),
                            "describe_frame": ParameterValue(
                                LaunchConfiguration("describe_frame"),
                                value_type=bool,
                            ),
                        }
                    ],
                    **common,
                )
                for agent in AGENTS
            ),
            # This robot's eyes. Not an agent -- it holds no state, decides
            # nothing and has no prompt; it turns the latest camera frame into a
            # sentence when triage asks. It lives in this launch file anyway
            # because it is useless to anything else here and triage is the only
            # caller, the same reasoning that puts mission_bridge beside the
            # coordinator rather than in this package's agent list.
            #
            # Both names relative, so this robot's namespace places them: the
            # camera it looks through is its own, and the service it serves is
            # the one its own triage resolves /vlm/ask to.
            Node(
                package="amiga_vlm_bridge",
                executable="vlm_server",
                name="vlm_server",
                namespace=namespace,
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_vlm")),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "service_name": "vlm/ask",
                        "image_topic": LaunchConfiguration("vlm_image_topic"),
                        "static_image_path": LaunchConfiguration("vlm_static_image"),
                        "vlm_url": LaunchConfiguration("vlm_url"),
                        # Deliberately says "a camera", not "the front camera":
                        # which one it is depends on vlm_image_topic, and a
                        # system prompt insisting on the front one while the
                        # node subscribes to the wrist would put a falsehood in
                        # front of the model on every single call.
                        "system_prompt": "You are looking through a camera on "
                        "a robot working in a pistachio orchard. "
                        "Describe only what is visible. Never guess at causes.",
                        # Well under triage's own 8 s deadline, so the far end
                        # gives up before the caller does and the log says the
                        # model was slow rather than that the service vanished.
                        "http_timeout_sec": 6.0,
                    }
                ],
            ),
            # Not an agent and no model: it summarises the plan for the
            # coordinator's mission port so the coordinator never parses XML.
            # Its names are already relative, so it needs the namespace and
            # nothing from the remap table.
            Node(
                package=PACKAGE,
                executable="mission_bridge",
                name="mission_bridge",
                condition=IfCondition(LaunchConfiguration("launch_mission_bridge")),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "battery_percent": ParameterValue(
                            LaunchConfiguration("battery_percent"), value_type=int
                        ),
                    }
                ],
                **common,
            ),
        ]
    )

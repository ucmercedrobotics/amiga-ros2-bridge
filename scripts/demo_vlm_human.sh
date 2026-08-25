#!/bin/bash
#
# One command, one working demo: three robots, three different aisles, and a
# person standing in one of them.
#
# Each robot is sent to its own aisle to sample two trees there -- robot 1 to
# aisle 6, robot 2 to aisle 4, robot 3 to aisle 2, from the checked-in
# examples/sample_aisle{2,4,6}.bin two-frame payloads. Every robot gets the
# full 144-tree orchard and an unmodified plan. Nothing is broken in software.
#
# The fault is physical, and the sim places it for you: a standing person is
# spawned on the row waypoint of the first tree in the blocked robot's aisle.
# That spot matters. It is the pose Nav2 drives to before handing the last few
# metres to the lidar approach, and a body standing on it sits inside the
# footprint's inscribed radius, so the planner cannot place the goal at all and
# NavigateToPose aborts. Anywhere else in the aisle and the robot simply drives
# around -- the lane is 9 m wide against a 2.5 m robot. What reaches triage is
# a real /bt/status_change, with real Nav2 log lines behind it that say a plan
# could not be made and never say why.
#
# That last part is why this demo exists and why it differs from
# demo_llm_auction.sh. There, the fault names itself -- "No point cloud
# available." is unambiguous evidence about the robot's own hardware, and the
# logs alone are enough to route it. Here they are not: an obstacle reads the
# same in the logs whether it is a person, an animal, a fallen branch or a
# parked vehicle, and those do not call for the same response. Triage captures
# the camera's description at the moment of failure and the reasoning model
# decides with it in hand.
#
#   person stands on the row waypoint -> planner cannot plan to it
#                                -> NavigateToPose aborts, MoveToTreeID FAILS
#                                -> /bt/status_change -> triage
#                                -> triage captures the fault: log slice,
#                                   world frame, and one call to the camera
#                                -> the reasoning model decides repair or
#                                   escalate WITH "a person is visible" in
#                                   front of it
#                                -> if it escalates, the fleet auctions the
#                                   aisle and the note tells peers what is
#                                   standing there
#
# The two other robots work their own aisles throughout, which is the point of
# giving each one a different one: whatever happens to the blocked robot has to
# happen while a real fleet is busy, not to a fleet of one.
#
# Needs BOTH models. The reasoning model on AGENT_API_BASE decides; a separate
# vision model on VLM_URL describes camera frames and decides nothing. See
# amiga_ros2_agents/README.md.
#
# Usage:
#   export AGENT_MODEL=hosted_vllm/openai/gpt-oss-120b
#   export AGENT_API_BASE=http://100.88.70.65:8000/v1
#   export VLM_URL=http://100.88.70.65:8001/v1/chat/completions
#   ./scripts/demo_vlm_human.sh                   # a person in the aisle
#   OBSTRUCTION=truck ./scripts/demo_vlm_human.sh  # a lane blocked by vehicles
#
#   ./scripts/demo_vlm_human.sh stop     # tear down, gz orphans included
#
# The two obstructions are two different faults, not one fault dressed twice.
# A person stands on the row waypoint: that goal pose cannot be placed, the
# approach aborts, and the lane either side of them is still open -- so the
# work is reachable by a peer, or by this robot once they move. Vehicles across
# the mouth of the lane leave nothing reachable down that lane at all, and the
# answer is the other aisle: a row of trees has one on each side.
#
# Which it is, is a thing only the camera can say. The navigation logs read the
# same either way.
#
# Everything runs in one tmux session ("vlm-demo"): a `sim` window, one `bt<i>`
# per robot, an `agents` window showing only the decision story off /rosout
# (what the camera saw, what triage decided), plus `infeasible` and
# `mission-xml`, and a `feed` window that seeds the missions once each robot's
# Nav2 is actually up and then exits. Ctrl-B then a window number to switch.
# Every window is teed to $LOG_DIR.

set -uo pipefail

SESSION="vlm-demo"
ROBOT_COUNT="${ROBOT_COUNT:-3}"
BASE_PORT=12346
LOG_DIR="/tmp/vlm-demo-logs"

# One aisle per robot: each enters one aisle, samples two trees in it, and
# leaves, which keeps the robots physically apart and makes "whose aisle is
# blocked" a question with an unambiguous answer.
#
# Descending: robots spawn at increasing y while aisle centrelines run the
# other way, so index order would send them across the field past each other.
EXAMPLES_DIR="amiga_ros2_behavior_tree/examples"
MISSION_BINS=(
    "${EXAMPLES_DIR}/sample_aisle6.bin"
    "${EXAMPLES_DIR}/sample_aisle4.bin"
    "${EXAMPLES_DIR}/sample_aisle2.bin"
)
MISSION_AISLES=(6 4 2)
# The first tree each mission visits, and so the one whose row waypoint the
# person is spawned on -- the robot has to get past them to reach it. These are
# the `id` attributes of the first MoveToTreeID in each sample_aisleN.xml.
MISSION_FIRST_TREES=(96 58 20)

# Which robot gets the human standing in its way. Drives the spawn: the person
# is placed on the row waypoint of MISSION_FIRST_TREES[BLOCKED_ROBOT-1].
BLOCKED_ROBOT="${BLOCKED_ROBOT:-3}"

# What is in the way: a person, or a pickup parked across the lane. Two
# different faults, not two dressings of one. A person blocks the goal pose and
# leaves the lane open either side; a truck blocks the lane, so nothing further
# down the aisle is reachable at all. Which one it is is a thing only the camera
# can say, and what a plan should do about it differs.
OBSTRUCTION="${OBSTRUCTION:-person}"
case "$OBSTRUCTION" in
    person|truck) ;;
    *) echo "OBSTRUCTION must be 'person' or 'truck', got '$OBSTRUCTION'" >&2; exit 1 ;;
esac

# Must match sim_bringup.launch.py's robot_name_prefix, since that is what
# names both the namespaces and the virtual radio's per-robot ptys.
ROBOT_PREFIX="${ROBOT_PREFIX:-amiga}"

# Unlike demo_llm_auction.sh, this one is watched in the Gazebo window: you
# need to see the robot reach the person. Headless is still available for a
# machine with no display, but then the physical half of the demo is invisible.
HEADLESS="${HEADLESS:-false}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_PATH" || exit 1

# Both patterns are anchored, and that is not cosmetic: `pkill -f` matches the
# WHOLE command line of every process, so an unanchored "ign gazebo" also
# matches any shell whose own arguments happen to mention it -- including the
# one that called this function, which it then kills.
GZ_PATTERN='^ign gazebo'
LAUNCH_PATTERN='bin/ros2 launch amiga_ros2_(gazebo|behavior_tree)'

teardown() {
    tmux kill-session -t "$SESSION" 2>/dev/null
    # Launch first so it stops respawning, then the server it orphaned.
    pkill -f "$LAUNCH_PATTERN" 2>/dev/null
    pkill -f "$GZ_PATTERN" 2>/dev/null
    for _ in $(seq 1 10); do
        pgrep -f "$GZ_PATTERN" >/dev/null 2>&1 || return 0
        sleep 1
    done
    pkill -9 -f "$GZ_PATTERN" 2>/dev/null
    pkill -9 -f "$LAUNCH_PATTERN" 2>/dev/null
}

# Before the environment checks on purpose: tearing a run down is exactly what
# you need to do when the environment is wrong.
if [ "${1:-}" = "stop" ]; then
    teardown
    echo "torn down: tmux session, launches, and any orphaned gz servers."
    exit 0
fi

if [ -z "${AGENT_MODEL:-}" ] || [ -z "${AGENT_API_BASE:-}" ]; then
    echo "AGENT_MODEL / AGENT_API_BASE are not both set." >&2
    echo "That is the REASONING model -- it makes every decision." >&2
    echo "See amiga_ros2_agents/README.md." >&2
    exit 1
fi

if [ -z "${VLM_URL:-}" ]; then
    echo "VLM_URL is not set." >&2
    echo "That is the VISION model -- a separate model on a separate endpoint," >&2
    echo "which describes camera frames and decides nothing." >&2
    echo "Without it this is demo_llm_auction.sh with a different fault: the" >&2
    echo "robot still stops, but nothing can say a PERSON is what stopped it." >&2
    echo >&2
    echo "  export VLM_URL=http://<host>:8001/v1/chat/completions" >&2
    exit 1
fi

if [ "$AGENT_API_BASE" = "${VLM_URL%/chat/completions}" ]; then
    echo "AGENT_API_BASE and VLM_URL point at the same endpoint." >&2
    echo "These are meant to be two models: a large one that reasons and a" >&2
    echo "small one that describes pictures. Serve them separately." >&2
    exit 1
fi

if [ "$ROBOT_COUNT" -ne 3 ]; then
    echo "ROBOT_COUNT=$ROBOT_COUNT: this demo ships three single-aisle missions" >&2
    echo "(aisles ${MISSION_AISLES[*]}), one per robot. Add missions to" >&2
    echo "MISSION_BINS to run more." >&2
    exit 1
fi

for bin in "${MISSION_BINS[@]}"; do
    [ -f "$bin" ] || { echo "missing mission payload: $bin" >&2; exit 1; }
done

namespace_for() { # robot index (1-based) -> that robot's namespace
    # MUST match sim_bringup.launch.py: robot 1 is unnamespaced ONLY when it is
    # the only robot, and this demo requires three.
    echo "${ROBOT_PREFIX}$1"
}

# Killing the tmux session is NOT enough to end a run: ros_gz_sim execs
# `ign gazebo -s` as a process that outlives the launch service, and the
# orphans accumulate until they starve the next run's Nav2 into never
# activating -- which looks exactly like a broken demo and is really a starved
# one.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists -- attaching."
    echo "($0 stop to tear it down, orphaned gz servers included)"
    tmux attach -t "$SESSION"
    exit 0
fi

if pgrep -f "$GZ_PATTERN" >/dev/null 2>&1; then
    echo "orphaned gz server(s) from an earlier run are still up -- clearing them:"
    pgrep -af "$GZ_PATTERN" | sed 's/^/  /'
    teardown
fi

blocked_aisle="${MISSION_AISLES[$((BLOCKED_ROBOT - 1))]}"
blocked_tree="${MISSION_FIRST_TREES[$((BLOCKED_ROBOT - 1))]}"
person_tree=0; truck_tree=0
[ "$OBSTRUCTION" = "person" ] && person_tree="$blocked_tree" || truck_tree="$blocked_tree"
blocked_ns="$(namespace_for "$BLOCKED_ROBOT")"
cat <<BANNER

  Three robots, three aisles:
BANNER
for i in $(seq 1 "$ROBOT_COUNT"); do
    marker=""
    [ "$i" -eq "$BLOCKED_ROBOT" ] && marker="   <-- the ${OBSTRUCTION} is in THIS aisle"
    printf '    %s%-8s aisle %-3s %s%s\n' "" "$(namespace_for "$i")" \
        "${MISSION_AISLES[$((i - 1))]}" "$(basename "${MISSION_BINS[$((i - 1))]}")" "$marker"
done
cat <<BANNER

  The ${OBSTRUCTION} is spawned for you at tree ${blocked_tree}'s row waypoint -- the
  spot ${blocked_ns} must reach before it can approach that tree. A person blocks
  that pose and leaves the lane open; a truck lies across the lane, so nothing
  further down the aisle is reachable either. OBSTRUCTION=person|truck picks.
  To move it by hand: amiga_ros2_gazebo/scripts/spawn_${OBSTRUCTION}.py

  reasoning: ${AGENT_MODEL}
  vision:    ${VLM_URL}

BANNER

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

# `tmux set -g` needs a running server and silently changes nothing without
# one, while both options below are read when a pane is CREATED -- so they have
# to be set before any real window exists. A throwaway session window is the
# only thing that holds a server open long enough to do it.
tmux new-session -d -s "$SESSION" -n bootstrap
tmux set -g mouse on
# Three robots' worth of Nav2 chatter scrolls the agent story out of a default
# 2000-line pane in seconds, and `ros2 launch` does not copy node stdout into
# ~/.ros/log, so without both of these the only record of a run is gone by the
# time you go looking for it.
tmux set -g history-limit 500000

tmux new-window -t "$SESSION" -n sim
tmux send-keys -t "$SESSION:sim" \
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} robot_name_prefix:=${ROBOT_PREFIX} mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true launch_vlm:=true vlm_url:=${VLM_URL} spawn_person:=${person_tree} spawn_truck:=${truck_tree} ltl_verification:=false objective_gating:=true 2>&1 | tee ${LOG_DIR}/sim.log" C-m

for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"
    port=$((BASE_PORT + i - 1))
    tmux new-window -t "$SESSION" -n "bt${i}"
    tmux send-keys -t "$SESSION:bt${i}" \
        "ros2 launch amiga_ros2_behavior_tree bt.launch.py namespace:=${ns} port:=${port} 2>&1 | tee ${LOG_DIR}/bt${i}.log" C-m
done

# The window this demo is actually about. Everything the agents log lands in
# the sim window mixed with Gazebo and three Nav2 stacks; this is the same
# lines off /rosout with everything else dropped, so "camera: a person is
# visible" and the verdict that follows are readable while they happen.
tmux new-window -t "$SESSION" -n agents
tmux send-keys -t "$SESSION:agents" \
    "python3 ${PROJECT_PATH}/scripts/watch_agents.py 2>&1 | tee ${LOG_DIR}/agents.log" C-m

# Two tiled windows, one pane per robot:
#   infeasible   fires only if a robot's local recovery gives up -- then the
#                fleet auctions the aisle
#   mission-xml  the accepted, currently-running plan. Started before any
#                mission is fed, since the topic is not latched.
tmux new-window -t "$SESSION" -n infeasible
tmux new-window -t "$SESSION" -n mission-xml
for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"

    # --full-length: `ros2 topic echo` truncates long strings to ~100 chars by
    # default, which turns every plan into the same unusable stub.
    #
    # The wait loop: `ros2 topic echo` on a topic whose type it cannot resolve
    # does NOT wait -- it prints "does not appear to be published yet" and
    # EXITS. These panes open before the agents exist, precisely so they are
    # already listening when the run starts, so without this every one of them
    # dies in its first second and a message published exactly once
    # (/coordination/infeasible) leaves no trace anywhere.
    wait_topic() { echo "until ros2 topic list 2>/dev/null | grep -qx '$1'; do sleep 2; done"; }

    infeasible_cmd="echo '--- ${ns}: coordination/infeasible ---'; $(wait_topic "/${ns}/coordination/infeasible"); ros2 topic echo --full-length /${ns}/coordination/infeasible | tee -a '$LOG_DIR/${ns}_infeasible.log'"
    mission_cmd="echo '--- ${ns}: mission/xml (accepted) ---'; $(wait_topic "/${ns}/mission/xml"); ros2 topic echo --full-length /${ns}/mission/xml | tee -a '$LOG_DIR/${ns}_mission_xml.log'"

    if [ "$i" -eq 1 ]; then
        tmux send-keys -t "$SESSION:infeasible" "$infeasible_cmd" C-m
        tmux send-keys -t "$SESSION:mission-xml" "$mission_cmd" C-m
    else
        tmux split-window -t "$SESSION:infeasible" "$infeasible_cmd"
        tmux select-layout -t "$SESSION:infeasible" tiled
        tmux split-window -t "$SESSION:mission-xml" "$mission_cmd"
        tmux select-layout -t "$SESSION:mission-xml" tiled
    fi
done

# Feed each robot its aisle mission once that robot's Nav2 is actually up.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
bins=($(printf '%q ' "${MISSION_BINS[@]}"))
aisles=(${MISSION_AISLES[*]})
echo
echo 'Waiting for each robot to finish bringing Nav2 up before feeding it.'
echo
for i in \$(seq 1 ${ROBOT_COUNT}); do
    port=\$(( ${BASE_PORT} + i - 1 ))
    mission=\"\${bins[\$(( i - 1 ))]}\"
    aisle=\"\${aisles[\$(( i - 1 ))]}\"
    ns=\"${ROBOT_PREFIX}\$i\"
    echo \"waiting for robot \$i's mission port (\$port)...\"
    until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
    exec 3<&- 3>&-
    # The open port only means tcp_demux is up, and tcp_demux is up long before
    # Gazebo, Nav2 and waypoint_follower are. Feeding on the port alone hands
    # the tree a mission whose action servers do not exist yet, and every leaf
    # fails instantly with 'Action server is not reachable' -- a real fault,
    # but the wrong one, firing before the robot could even try.
    #
    # Counted off the action's own status topic, not \`ros2 action info\`, which
    # aggregates per node name and goes wrong when the graph reports a node
    # twice -- which it does with a DDS profile whose interface allowlist does
    # not match the machine.
    #
    # Bounded, because the loop is serial: an unbounded wait on a robot whose
    # Nav2 never activates silently prevents every LATER robot from being fed.
    ready=1
    for act in move_to_aisle_head follow_tree_id_waypoint segment_leaves; do
        echo \"  waiting for /\$ns/\$act ...\"
        waited=0
        until timeout 10 ros2 topic info /\$ns/\$act/_action/status 2>/dev/null \\
            | grep -qE 'Publisher count: [1-9]'; do
            sleep 3
            waited=\$(( waited + 3 ))
            if [ \"\$waited\" -ge 300 ]; then
                echo \"  robot \$i never brought up \$act -- its Nav2 did not finish\"
                echo \"  activating (check the sim window for lifecycle_manager\"
                echo \"  bonds). NOT feeding this robot.\"
                ready=0
                break
            fi
        done
        [ \"\$ready\" -eq 1 ] || break
    done
    [ \"\$ready\" -eq 1 ] || continue
    echo \"feeding robot \$i on port \$port: aisle \$aisle (\$mission)\"
    nc -q 1 127.0.0.1 \$port < \"\$mission\"
done
echo
echo 'all missions fed. Robot ${BLOCKED_ROBOT} is the one heading for aisle ${blocked_aisle}.'
echo
echo 'watch:  agents      -- what the camera saw and what triage decided'
echo '        bt<i>       -- each robot executing its tree'
echo '        infeasible  -- fires only if a robot gives up and the fleet bids'
echo 'logs:   ${LOG_DIR}/'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

# Every real window exists now, so the session no longer needs the one that was
# only there to hold the server open long enough to configure it.
tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

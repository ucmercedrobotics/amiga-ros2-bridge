#!/bin/bash
#
# One robot, the same aisle-2 mission every other demo uses (sample_aisle2.bin
# -- trees 20 and 26), fed over the real TCP mission port like every other
# demo. Nothing in the world is broken this time: the mission runs exactly as
# normal. Partway through, a tree in aisle 4 (58, from sample_aisle4.xml -- a
# real tree, already in the orchard map, just not on this mission) is
# reported spotted with possible blight.
#
# That report is a real /bt/status_change message with status DETECTION --
# see demo_opportunistic_tree_feed.py -- landing on mission_planner_node's
# real _on_detection path (not a mock: the LLM call, XSD validation, and
# arbiter accept/reject that follow are the same code every fault repair
# uses). What's scripted is only the sighting itself, not anything
# downstream of it -- there is no camera/vision step in this demo, on
# purpose: this tests one question -- given a real opportunistic sighting,
# does the real planning/arbiter pipeline actually add it to a mission that
# is otherwise running fine? -- not whether a camera can find a sick tree.
#
#   aisle-2 mission running normally (trees 20, 26) ->
#   sighting reported for tree 58 / aisle 4, not on this mission ->
#   /bt/status_change (status DETECTION) -> mission_planner_node:
#   _on_detection -> real LLM call weighing the addition against the mission
#   still in progress -> candidate to /mission/candidate_xml -> arbiter
#   accepts or rejects like any other candidate -> if accepted, published to
#   /mission/xml and adopted once the current mission ends (bt_runner cannot
#   adopt a plan mid-mission -- same as every other replan)
#
# No auction to watch -- one robot, no peers -- this is purely the
# repair-vs-add decision inside a single robot's own planner.
#
# Needs AGENT_MODEL / AGENT_API_BASE, same as demo_missing_tree.sh. No VLM
# needed -- there is no camera step in this demo.
#
# Usage:
#   export AGENT_MODEL=hosted_vllm/openai/gpt-oss-120b
#   export AGENT_API_BASE=http://100.88.70.65:8000/v1
#   ./scripts/demo_opportunistic_tree.sh
#   ./scripts/demo_opportunistic_tree.sh stop     # tear down, gz orphans included
#
# Everything runs in one tmux session ("opportunistic-tree-demo"): a `sim`
# window, `bt1`, an `agents` window showing only the decision story off
# /rosout, plus `watch` and `mission-xml`, and a `feed` window that seeds the
# mission once Nav2 is up, waits for the robot to be underway, then reports
# the sighting and waits to see if it comes back accepted. Ctrl-B then a
# window number to switch. Every window is teed to $LOG_DIR.

set -uo pipefail

SESSION="opportunistic-tree-demo"
ROBOT_COUNT=1
BASE_PORT=12348
LOG_DIR="/tmp/opportunistic-tree-demo-logs"

EXAMPLES_DIR="amiga_ros2_behavior_tree/examples"
MISSION_BIN="${EXAMPLES_DIR}/sample_aisle2.bin"
MISSION_AISLE=2

FEED_SCRIPT="amiga_ros2_agents/scripts/scenarios/demo_opportunistic_tree_feed.py"

# Must match sim_bringup.launch.py's robot_name_prefix; irrelevant here since
# ROBOT_COUNT=1 means robot 1 is unnamespaced (see that file's module
# docstring), but the launch argument still wants a value.
ROBOT_PREFIX="${ROBOT_PREFIX:-amiga}"

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

if [ ! -f "$MISSION_BIN" ]; then
    echo "missing mission payload: $MISSION_BIN" >&2
    exit 1
fi

if [ ! -f "$FEED_SCRIPT" ]; then
    echo "missing feed script: $FEED_SCRIPT" >&2
    exit 1
fi

# Killing the tmux session is NOT enough to end a run: ros_gz_sim execs
# `ign gazebo -s` as a process that outlives the launch service, and the
# orphans accumulate until they starve the next run's Nav2 into never
# activating -- which looks exactly like a broken demo and is really a
# starved one.
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

cat <<BANNER

  One robot, aisle 2, the usual two trees (20, 26) -- mission runs normally.

  Partway through: a tree in aisle 4 (58) is reported spotted, sick, and not
  on this mission. Does the real planner add it?

  reasoning: ${AGENT_MODEL}

  No peers this run, no fault, no camera -- just the repair-vs-add decision
  inside one robot's own planner, on a real opportunistic detection event.

BANNER

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

# `tmux set -g` needs a running server and silently changes nothing without
# one, while both options below are read when a pane is CREATED -- so they
# have to be set before any real window exists. A throwaway session window is
# the only thing that holds a server open long enough to do it.
tmux new-session -d -s "$SESSION" -n bootstrap
tmux set -g mouse on
tmux set -g history-limit 500000

tmux new-window -t "$SESSION" -n sim
tmux send-keys -t "$SESSION:sim" \
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} robot_name_prefix:=${ROBOT_PREFIX} mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true launch_vlm:=false ltl_verification:=false objective_gating:=true 2>&1 | tee ${LOG_DIR}/sim.log" C-m

tmux new-window -t "$SESSION" -n bt1
tmux send-keys -t "$SESSION:bt1" \
    "ros2 launch amiga_ros2_behavior_tree bt.launch.py port:=${BASE_PORT} 2>&1 | tee ${LOG_DIR}/bt1.log" C-m

# The window this demo is actually about: the same /rosout decision story the
# other demos read, off an un-namespaced fleet of one -- see watch_agents.py,
# which does not require a robot prefix to match a line.
tmux new-window -t "$SESSION" -n agents
tmux send-keys -t "$SESSION:agents" \
    "python3 ${PROJECT_PATH}/scripts/watch_agents.py 2>&1 | tee ${LOG_DIR}/agents.log" C-m

# --full-length: `ros2 topic echo` truncates long strings to ~100 chars by
# default, which turns every plan into the same unusable stub.
#
# The wait loop: `ros2 topic echo` on a topic whose type it cannot resolve
# does NOT wait -- it prints "does not appear to be published yet" and EXITS.
# These panes open before the agents exist, precisely so they are already
# listening when the run starts.
wait_topic() { echo "until ros2 topic list 2>/dev/null | grep -qx '$1'; do sleep 2; done"; }

tmux new-window -t "$SESSION" -n watch
tmux send-keys -t "$SESSION:watch" \
    "echo '--- mission_planner candidates ---'; $(wait_topic "/mission/candidate_xml"); ros2 topic echo --full-length /mission/candidate_xml | tee -a '$LOG_DIR/candidates.log'" C-m

tmux new-window -t "$SESSION" -n mission-xml
tmux send-keys -t "$SESSION:mission-xml" \
    "echo '--- mission/xml (accepted) ---'; $(wait_topic "/mission/xml"); ros2 topic echo --full-length /mission/xml | tee -a '$LOG_DIR/mission_xml.log'" C-m

# Feed the mission over the real TCP port once Nav2 is actually up -- same
# payload, same mechanism as demo_missing_tree.sh, so tcp_demux_node
# populates the orchard map (/orchard/tree_info_json) as well as /mission/xml.
# Publishing the XML directly and skipping the TCP port was tried and skips
# that: GetTreeInfo comes back empty and the very first MoveToAisleHead fails
# for real. Once the mission is confirmed live, the feed script reports the
# scripted sighting.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
echo
echo 'Waiting for Nav2 action servers before feeding the mission.'
echo
ready=1
for act in move_to_aisle_head follow_tree_id_waypoint segment_leaves; do
    echo \"  waiting for /\$act ...\"
    waited=0
    until timeout 10 ros2 topic info /\$act/_action/status 2>/dev/null \\
        | grep -qE 'Publisher count: [1-9]'; do
        sleep 3
        waited=\$(( waited + 3 ))
        if [ \"\$waited\" -ge 300 ]; then
            echo \"  Nav2 never brought up \$act -- it did not finish activating\"
            echo \"  (check the sim window for lifecycle_manager bonds). NOT feeding.\"
            ready=0
            break
        fi
    done
    [ \"\$ready\" -eq 1 ] || break
done
if [ \"\$ready\" -eq 1 ]; then
    port=${BASE_PORT}
    echo \"waiting for the mission port (\$port)...\"
    until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
    exec 3<&- 3>&-
    # The action servers being up (checked above) does not mean bt_runner's
    # OWN participant has finished discovering them yet -- DDS discovery is
    # per participant-pair, not global. Ticking the tree the instant the
    # mission arrives can catch that gap: every action node reports 'not
    # reachable' on the first tick, a real failure, confirmed live. A few
    # seconds' buffer here gives bt_runner's discovery time to finish before
    # the mission ever reaches it.
    echo \"letting bt_runner finish discovering the action servers...\"
    sleep 10
    echo \"feeding mission on port \$port: aisle ${MISSION_AISLE} ($(basename "$MISSION_BIN"))\"
    python3 ${PROJECT_PATH}/${FEED_SCRIPT} --port \$port --mission-bin ${PROJECT_PATH}/${MISSION_BIN}
fi
echo
echo 'watch:  agents      -- what triage/planner decided'
echo '        bt1         -- the robot executing its tree'
echo '        watch       -- every LLM planning attempt, accepted or not'
echo '        mission-xml -- the accepted plan, before and after'
echo 'logs:   ${LOG_DIR}/'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

# Every real window exists now, so the session no longer needs the one that
# was only there to hold the server open long enough to configure it.
tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

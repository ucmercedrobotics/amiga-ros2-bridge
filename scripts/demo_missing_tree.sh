#!/bin/bash
#
# One robot, the same aisle-2 mission every other demo uses, and the first
# of its two trees is not there.
#
# The robot is sent to sample trees 20 and 26 (aisle 2, sample_aisle2.bin --
# the exact mission demo_vlm_human.sh feeds robot 3), except tree 20 is
# deleted from the running world before the mission starts. GetTreeInfo --
# the orchard map -- is not touched, so the robot's own plan still says tree
# 20 exists: it drives to the recorded position and finds bare soil. That is
# a stale map, the real shape of a missing tree, not a scripted fault -- see
# remove_tree.py.
#
# The physics matters here in a way it did not for the person/truck demos.
# The lidar approach's own "nothing in the wedge" branch used to report
# success regardless -- so a missing tree was silently marked sampled. Fixed
# in lidar_object_navigator.cpp: that branch now aborts the goal instead.
# This demo is what that fix is for. It is a real behavior change for every
# demo, not just this one -- a single empty scan happens on real trees too,
# caught mid-turn, and now reports a failure where it used to report
# success. Accepted here because this demo is the priority right now.
#
#   tree 20 removed at launch -> robot approaches 20 first ->
#                                 lidar finds nothing -> NavigateViaLidar
#                                 aborts -> MoveToTreeID FAILS ->
#                                 /bt/status_change
#                                 -> triage: log slice, world frame, one call
#                                    to the camera (which sees bare ground,
#                                    not a tree)
#                                 -> repair: another approach angle will not
#                                    conjure a tree, so a repair loop just
#                                    burns the retry budget
#                                 -> escalate: with zero peers, re_delegate
#                                    has nowhere to send the work; the right
#                                    answer is drop_task, then move on to 26
#
# No auction to watch -- one robot, no peers -- which is the point: this
# tests the single-robot decision (repair, then give up locally and drop,
# not endlessly retry or offer a peer that does not exist) in isolation from
# the fleet-coordination layer the multi-robot demos already cover.
#
# Needs AGENT_MODEL / AGENT_API_BASE and VLM_URL, same as demo_vlm_human.sh.
# See amiga_ros2_agents/README.md.
#
# Usage:
#   export AGENT_MODEL=hosted_vllm/openai/gpt-oss-120b
#   export AGENT_API_BASE=http://100.88.70.65:8000/v1
#   export VLM_URL=http://100.88.70.65:8001/v1/chat/completions
#   ./scripts/demo_missing_tree.sh
#   ./scripts/demo_missing_tree.sh stop     # tear down, gz orphans included
#
# Everything runs in one tmux session ("missing-tree-demo"): a `sim` window,
# `bt1`, an `agents` window showing only the decision story off /rosout,
# plus `watch`, `infeasible` and `mission-xml`, and a `feed` window that
# seeds the mission once Nav2 is up and then exits. Ctrl-B then a window
# number to switch. Every window is teed to $LOG_DIR.

set -uo pipefail

SESSION="missing-tree-demo"
ROBOT_COUNT=1
BASE_PORT=12346
LOG_DIR="/tmp/missing-tree-demo-logs"

EXAMPLES_DIR="amiga_ros2_behavior_tree/examples"
MISSION_BIN="${EXAMPLES_DIR}/sample_aisle2.bin"
MISSION_AISLE=2

# The first tree in sample_aisle2.bin.
MISSING_TREE="${MISSING_TREE:-20}"

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

if [ -z "${VLM_URL:-}" ]; then
    echo "VLM_URL is not set." >&2
    echo "That is the VISION model -- a separate model on a separate endpoint," >&2
    echo "which describes camera frames and decides nothing." >&2
    echo "  export VLM_URL=http://<host>:8001/v1/chat/completions" >&2
    exit 1
fi

if [ ! -f "$MISSION_BIN" ]; then
    echo "missing mission payload: $MISSION_BIN" >&2
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

  One robot, aisle ${MISSION_AISLE}, the usual two trees -- and tree ${MISSING_TREE} is not there.

  mission:   $(basename "$MISSION_BIN")  (trees 20, 26)
  removed:   tree ${MISSING_TREE} -- gone from the world, still in the orchard map
  reasoning: ${AGENT_MODEL}
  vision:    ${VLM_URL}

  No peers this run: a fault that cannot be repaired and cannot be handed
  off has to be dropped locally, or the mission never ends cleanly.

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
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} robot_name_prefix:=${ROBOT_PREFIX} mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true launch_vlm:=true vlm_url:=${VLM_URL} remove_tree:=${MISSING_TREE} ltl_verification:=false objective_gating:=true 2>&1 | tee ${LOG_DIR}/sim.log" C-m

tmux new-window -t "$SESSION" -n bt1
tmux send-keys -t "$SESSION:bt1" \
    "ros2 launch amiga_ros2_behavior_tree bt.launch.py port:=${BASE_PORT} 2>&1 | tee ${LOG_DIR}/bt1.log" C-m

# The window this demo is actually about: the same /rosout decision story the
# multi-robot demos read, off an un-namespaced fleet of one -- see
# watch_agents.py, which does not require a robot prefix to match a line.
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

tmux new-window -t "$SESSION" -n infeasible
tmux send-keys -t "$SESSION:infeasible" \
    "echo '--- coordination/infeasible ---'; $(wait_topic "/coordination/infeasible"); ros2 topic echo --full-length /coordination/infeasible | tee -a '$LOG_DIR/infeasible.log'" C-m

tmux new-window -t "$SESSION" -n mission-xml
tmux send-keys -t "$SESSION:mission-xml" \
    "echo '--- mission/xml (accepted) ---'; $(wait_topic "/mission/xml"); ros2 topic echo --full-length /mission/xml | tee -a '$LOG_DIR/mission_xml.log'" C-m

# Feed the mission once Nav2 is actually up. See demo_vlm_human.sh's feed
# window for why this waits on the action servers and not just the port.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
port=${BASE_PORT}
echo
echo 'Waiting for the mission port before feeding it.'
echo
echo \"waiting for the mission port (\$port)...\"
until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
exec 3<&- 3>&-
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
    echo \"feeding mission on port \$port: aisle ${MISSION_AISLE} ($(basename "$MISSION_BIN"))\"
    nc -q 1 127.0.0.1 \$port < ${MISSION_BIN}
fi
echo
echo 'watch:  agents      -- what the camera saw and what triage decided'
echo '        bt1         -- the robot executing its tree'
echo '        watch       -- every LLM repair attempt, accepted or not'
echo '        infeasible  -- fires only if local recovery gives up'
echo 'logs:   ${LOG_DIR}/'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

# Every real window exists now, so the session no longer needs the one that
# was only there to hold the server open long enough to configure it.
tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

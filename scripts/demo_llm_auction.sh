#!/bin/bash
#
# One command, one working demo: bring up a simulated fleet with a real LLM
# behind both reasoning points, feed every robot a real mission with real
# GPS/orchard data (the checked-in examples/*.bin two-frame payloads), and
# let the real pipeline run.
#
# All GPS data is real on purpose. Two earlier versions of this fault are
# worth knowing about, because each one fails in a way that looks like a
# broken pipeline rather than a badly chosen scenario:
#
#   * Starving EVERY robot of orchard data. Every robot fails its first goal
#     at once and every robot tries to shed a task simultaneously -- there is
#     no healthy fleet left to bid, so the auction never chooses a winner.
#   * Pointing one MoveToTreeID at a tree id that does not exist. That fault
#     is real, but the task it produces is one nobody can ever take: a peer
#     resolves tree 9999 against its own orchard, finds nothing, and bids
#     infeasible. So does every other peer. The auction runs, collects a full
#     set of "I cannot", and hands the task straight back -- everything works
#     except the part the demo exists to show.
#
# The fault has to be local to one robot while the work stays real. So
# exactly one robot ($FAIL_ROBOT) gets its normal mission, with its normal
# plan asking for its normal trees, and ONE tree removed from its own copy of
# the orchard JSON -- see scripts/build_broken_mission.py.
# /orchard/get_tree_info filters its cached tree list server-side
# (orchard_management.cpp) and legitimately finds no match, so
# amiga_navigation's waypoint_follower.py aborts that one goal for real
# ("GetTreeInfo returned empty result"), on the real /bt/status_change topic
# bt_runner's own FaultReporter publishes. Every other robot has the untouched
# orchard, so $HIDDEN_TREE is somewhere they can all actually go, and the task
# is worth bidding on. Nothing here injects a topic or service call by hand.
# From there the existing pipeline runs itself:
#
#   leaf fails -> mission_planner (LLM) tries to repair its own XML
#              -> arbiter reviews the candidate; dropping the tree is an
#                 unjustified drop, so the candidate is rejected
#              -> this robot cannot reach that tree however the plan is
#                 written, so repair keeps failing until a budget runs out
#              -> /mission/abort (viability) or a terminal give-up on
#                 /mission/planner_status (rejection retries) -- either one
#                 escalates
#              -> triage -> /coordination/infeasible
#              -> the fleet (still running real, valid missions, and all of
#                 them able to reach $HIDDEN_TREE) auctions the shed task
#              -> the winner's mission_planner (LLM) splices the absorbed
#                 task into ITS OWN XML
#
# Two real LLM replans, on two different robots' XML, off one real,
# single-goal fault. Nothing here is fabricated on the wire -- one robot's
# orchard frame is short by one tree, and that is the whole injection.
#
# NOTE the arbiter must be able to ABORT for the loop above to terminate,
# which is `objective_gating` (on by default) and NOT `ltl_verification`.
# This runs with ltl_verification:=false -- no formula, no SPIN -- and the
# objective/viability checks still on, which is what ends local recovery.
#
# Usage:
#   export AGENT_MODEL=hosted_vllm/openai/gpt-oss-120b
#   export AGENT_API_BASE=http://100.88.70.65:8000/v1
#   make llm-demo
#
# Everything runs in one tmux session ("llm-demo"): a `sim` window, one `bt<i>`
# window per robot, a `feed` window that seeds the missions once the ports are
# listening and then exits, and `watch`/`infeasible`/`mission-xml` windows
# tiled with the topics that tell the whole story. Ctrl-B then a window
# number to switch; `tmux kill-session -t llm-demo` to tear it all down.
#
# `/mission/xml` (the arbiter's sole-write topic -- the accepted, currently
# running plan) is NOT latched, so a subscriber that starts late sees nothing
# until the next change. The `mission-xml` window starts echoing it before
# the missions are even fed, and tees every message into
# $LOG_DIR/<robot>_mission_xml.log, so the finalized plan and every edit that
# produced it are on disk for diffing after the fact even if you weren't
# watching live.

set -uo pipefail

SESSION="llm-demo"
ROBOT_COUNT="${ROBOT_COUNT:-3}"
BASE_PORT=12346
LOG_DIR="/tmp/llm-demo-logs"

# Real, checked-in two-frame (XML + orchard JSON) mission payloads, cycled
# round-robin across the healthy robots so not everyone runs the identical
# plan.
EXAMPLES_DIR="amiga_ros2_behavior_tree/examples"
MISSION_BINS=(
    "${EXAMPLES_DIR}/sample_20_64.bin"
    "${EXAMPLES_DIR}/sample_22_66.bin"
    "${EXAMPLES_DIR}/sample_24_68.bin"
)

# Which robot (1-based) gets the crippled orchard, and which of its two real
# tree targets is removed from that robot's copy of it. The tree stays real
# and stays in everybody else's orchard -- that is what makes the shed task
# something a peer can actually win.
FAIL_ROBOT="${FAIL_ROBOT:-1}"
BROKEN_TREE_SOURCE="${EXAMPLES_DIR}/sample_20_64.bin"
HIDDEN_TREE="${HIDDEN_TREE:-64}"
BROKEN_MISSION_BIN="${LOG_DIR}/broken_mission.bin"

# Must match sim_bringup.launch.py's robot_name_prefix, since that is what
# names both the namespaces and the virtual radio's per-robot ptys.
ROBOT_PREFIX="${ROBOT_PREFIX:-amiga}"

# HEADLESS=true runs gz with no GUI. Nothing in this demo is watched in the
# Gazebo window -- the story is told by the topic panes -- so on a machine with
# no X display (a server, or a container without /tmp/.X11-unix bind-mounted)
# this is the difference between the demo running and gz failing to open a
# window. Default false, which is the behaviour when there is a screen.
HEADLESS="${HEADLESS:-false}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_PATH" || exit 1

if [ -z "${AGENT_MODEL:-}" ]; then
    echo "AGENT_MODEL is not set." >&2
    echo "export AGENT_MODEL=... and AGENT_API_BASE=... first (amiga_ros2_agents/README.md)" >&2
    echo "-- the whole point of this demo is watching a real model repair and auction a plan." >&2
    exit 1
fi

if [ "$ROBOT_COUNT" -lt 3 ]; then
    echo "ROBOT_COUNT=$ROBOT_COUNT: fewer than 3 robots means at most one" >&2
    echo "possible bidder, which is enough for the handshake but not enough to" >&2
    echo "show a winner being chosen. Use 3+." >&2
    exit 1
fi

namespace_for() { # robot index (1-based) -> that robot's namespace
    # MUST match sim_bringup.launch.py, which reads:
    #     ns = "" if (i == 1 and robot_count == 1) else f"{prefix}{i}"
    # i.e. robot 1 is unnamespaced ONLY when it is the only robot. This script
    # requires 3+, so every robot here is namespaced, robot 1 included.
    #
    # This used to return "" for robot 1 unconditionally, which started robot
    # 1's bt_runner at the root while its nav, agents, mission_bridge and
    # coordinator all lived under /amiga1. Its tree could not reach its own
    # action servers, its faults reached no planner, and the panes below
    # echoed topics nobody published -- and FAIL_ROBOT defaults to 1, so the
    # one robot meant to drive the whole demo was the one disconnected.
    if [ "$ROBOT_COUNT" -eq 1 ] && [ "$1" -eq 1 ]; then
        echo ""
    else
        echo "${ROBOT_PREFIX}$1"
    fi
}

topic_prefix_for() { # namespace -> leading path segment for its topics
    if [ -z "$1" ]; then echo ""; else echo "/$1"; fi
}

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists -- attaching."
    echo "(tmux kill-session -t $SESSION to tear down and start clean)"
    tmux attach -t "$SESSION"
    exit 0
fi

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*_mission_xml.log

tmux set -g mouse on
tmux new-session -d -s "$SESSION" -n sim
tmux send-keys -t "$SESSION:sim" \
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} robot_name_prefix:=${ROBOT_PREFIX} mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true ltl_verification:=false objective_gating:=true" C-m

for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"
    port=$((BASE_PORT + i - 1))
    ns_arg=""
    [ -n "$ns" ] && ns_arg="namespace:=${ns}"
    tmux new-window -t "$SESSION" -n "bt${i}"
    tmux send-keys -t "$SESSION:bt${i}" \
        "ros2 launch amiga_ros2_behavior_tree bt.launch.py ${ns_arg} port:=${port}" C-m
done

# Three tiled windows, one pane per robot each:
#   watch        every LLM repair attempt (the loser's local loop, then the
#                winner's absorption edit) -- /mission/candidate_xml
#   infeasible   fires exactly once, on whichever robot's local recovery
#                actually gives up -- /coordination/infeasible
#   mission-xml  the accepted, currently-running plan -- /mission/xml. Started
#                now, before any mission is fed, since the topic isn't
#                latched; also teed to $LOG_DIR for post-hoc inspection.
tmux new-window -t "$SESSION" -n watch
tmux new-window -t "$SESSION" -n infeasible
tmux new-window -t "$SESSION" -n mission-xml
for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"
    prefix="$(topic_prefix_for "$ns")"
    label="${ns:-robot1}"
    log_file="$LOG_DIR/${ns:-robot1}_mission_xml.log"

    watch_cmd="echo '--- ${label}: mission_planner candidates ---'; ros2 topic echo ${prefix}/mission/candidate_xml"
    infeasible_cmd="echo '--- ${label}: coordination/infeasible ---'; ros2 topic echo ${prefix}/coordination/infeasible"
    mission_cmd="echo '--- ${label}: mission/xml (accepted) -- also logging to ${log_file} ---'; ros2 topic echo ${prefix}/mission/xml | tee -a '${log_file}'"

    if [ "$i" -eq 1 ]; then
        tmux send-keys -t "$SESSION:watch" "$watch_cmd" C-m
        tmux send-keys -t "$SESSION:infeasible" "$infeasible_cmd" C-m
        tmux send-keys -t "$SESSION:mission-xml" "$mission_cmd" C-m
    else
        tmux split-window -t "$SESSION:watch" "$watch_cmd"
        tmux select-layout -t "$SESSION:watch" tiled
        tmux split-window -t "$SESSION:infeasible" "$infeasible_cmd"
        tmux select-layout -t "$SESSION:infeasible" tiled
        tmux split-window -t "$SESSION:mission-xml" "$mission_cmd"
        tmux select-layout -t "$SESSION:mission-xml" tiled
    fi
done

python3 "$SCRIPT_DIR/build_broken_mission.py" \
    "$BROKEN_TREE_SOURCE" "$HIDDEN_TREE" "$BROKEN_MISSION_BIN" \
    || { echo "failed to build the broken mission payload" >&2; exit 1; }

# Feed each robot a real, correctly-framed mission once its port is actually
# accepting connections -- the broken one to $FAIL_ROBOT, an unmodified real
# one (round-robin over MISSION_BINS) to everyone else.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
bins=($(printf '%q ' "${MISSION_BINS[@]}"))
n_bins=\${#bins[@]}
for i in \$(seq 1 ${ROBOT_COUNT}); do
    port=\$(( ${BASE_PORT} + i - 1 ))
    if [ \"\$i\" -eq ${FAIL_ROBOT} ]; then
        mission='${BROKEN_MISSION_BIN}'
        echo \"robot \$i is the fail case: tree ${HIDDEN_TREE} removed from its orchard data (its plan still asks for it; every other robot can still reach it)\"
    else
        mission=\"\${bins[\$(( (i - 1) % n_bins ))]}\"
    fi
    echo \"waiting for robot \$i's mission port (\$port)...\"
    until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
    exec 3<&- 3>&-
    echo \"feeding robot \$i on port \$port: \$mission\"
    nc -q 1 127.0.0.1 \$port < \"\$mission\"
done
echo
echo 'all missions fed.'
echo 'watch: bt<i> for BT execution, watch/infeasible/mission-xml for the LLM+auction pipeline'
echo 'logs: $LOG_DIR/*_mission_xml.log'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

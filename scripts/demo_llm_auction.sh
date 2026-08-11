#!/bin/bash
#
# One command, one working demo: bring up a simulated fleet with a real LLM
# behind both reasoning points, feed every robot the same mission with no
# GPS/orchard data behind it, and let the real pipeline run.
#
# Why no orchard data is the trigger, not a bug: MoveToTreeID's action
# (follow_tree_id_waypoint) asks /orchard/get_tree_info for the tree's
# location; nothing here ever populates it (orchard JSON only ever arrives as
# the TCP mission's second frame -- see amiga_ros2_behavior_tree's
# tcp_demux_node), so amiga_navigation's waypoint_follower.py aborts the goal
# ("GetTreeInfo returned empty result") the moment the tree ticks its first
# MoveToTreeID. That is a real BT leaf failure, on the real /bt/status_change
# topic bt_runner's own FaultReporter publishes -- nothing here injects
# anything by hand. From there the existing pipeline runs itself:
#
#   leaf fails -> mission_planner (LLM) tries to repair its own XML
#              -> arbiter reviews the candidate
#              -> repeat until the arbiter's viability budget is exhausted
#                 (every tree in this mission fails the same way, so it will
#                 be) -> /mission/abort -> triage escalates
#              -> the fleet auctions the shed task
#              -> the winner's mission_planner (LLM) splices the absorbed
#                 task into ITS OWN XML
#
# Two real LLM replans, on two different robots' XML, off one real fault.
# Supersedes `make fleet-fault` / `make fleet-scenario`: both hand-inject a
# message: fleet-fault fabricates a /bt/status_change, fleet-scenario
# fabricates a /coordination/infeasible outright (see escalate.py). Nothing
# here is fabricated -- every mission this robot fails to run is one it was
# actually loaded, and every escalation off that.
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
MISSION_XML="amiga_ros2_behavior_tree/examples/sample_leafs.xml"
BASE_PORT=12346
LOG_DIR="/tmp/llm-demo-logs"

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

namespace_for() { # robot index (1-based) -> namespace, "" for robot 1
    if [ "$1" -eq 1 ]; then echo ""; else echo "amiga$1"; fi
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
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} launch_bt:=false launch_coordination:=true launch_agents:=true ltl_verification:=false" C-m

for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"
    port=$((BASE_PORT + i - 1))
    ns_arg=""
    [ -n "$ns" ] && ns_arg="namespace:=${ns}"
    tmux new-window -t "$SESSION" -n "bt${i}"
    tmux send-keys -t "$SESSION:bt${i}" \
        "ros2 launch amiga_ros2_behavior_tree bt.launch.py ${ns_arg} port:=${port} expect_json:=false payload_length_included:=false" C-m
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
    label="${ns:-robot1 (unnamespaced)}"
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

# Feed every robot the same mission once its port is actually accepting
# connections. No orchard/GPS frame follows it -- see the header comment.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
for i in \$(seq 1 ${ROBOT_COUNT}); do
    port=\$(( ${BASE_PORT} + i - 1 ))
    echo \"waiting for robot \$i's mission port (\$port)...\"
    until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
    exec 3<&- 3>&-
    echo \"feeding robot \$i on port \$port\"
    nc -q 1 127.0.0.1 \$port < ${MISSION_XML}
done
echo
echo 'all missions fed.'
echo 'watch: bt<i> for BT execution, watch/infeasible/mission-xml for the LLM+auction pipeline'
echo 'logs: $LOG_DIR/*_mission_xml.log'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

#!/bin/bash
#
# One command, one working demo: bring up a simulated fleet with a real LLM
# behind both reasoning points, feed every robot a real mission with real
# GPS/orchard data (the checked-in examples/*.bin two-frame payloads), and
# let the real pipeline run.
#
# All GPS data is real on purpose, and so is every tree: all three robots get
# the full 144-tree orchard and an unmodified mission. Exactly one robot
# ($FAIL_ROBOT) has a broken depth camera, so it drives to its trees perfectly
# well and fails only when it tries to SAMPLE one -- the real
# "No point cloud available." that pistachio_leaf_segmentation.py logs, from
# the shared mock failure vocabulary in
# amiga_ros2_behavior_tree/include/.../mocks/failure_modes.hpp. The fault
# reaches the pipeline on the real /bt/status_change topic bt_runner's own
# FaultReporter publishes. Nothing here injects a topic or service by hand.
#
# Why an arm fault and not a missing tree, which is what this demo used to do:
# triage's decision is "is this beyond THIS robot, or beyond ANY robot?", and
# it only sees this robot's own evidence -- never what peers know. A robot
# whose orchard copy is missing tree 20 logs "GetTreeInfo returned empty
# result", which from inside that robot is indistinguishable from the tree not
# existing at all, so triage drops the task instead of offering it and no
# auction ever runs. A camera that produced no point cloud is unambiguously
# this robot's problem, and a peer with a working camera is the right answer.
#
#   leaf fails -> mission_planner (LLM) tries to repair its own XML
#              -> arbiter reviews each candidate
#              -> no rewrite makes this robot's camera work, so repair runs out
#                 of budget (MAX_RETRIES planning sessions)
#              -> the planner reports a terminal give-up on
#                 /mission/planner_status
#              -> triage -> /coordination/infeasible, carrying the whole failed
#                 subtree as a task descriptor
#              -> triage interprets it as re_delegate, and the fleet auctions
#                 the shed task over the simulated LoRa radio
#              -> the winner's mission_planner (LLM) splices the absorbed task
#                 into ITS OWN XML
#
# Two real LLM replans, on two different robots' XML, off one real fault.
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

# Which robot (1-based) gets the broken arm, and how it breaks. Everything
# else about that robot is ordinary: it gets a real, unmodified mission and the
# full 144-tree orchard, drives to its trees successfully, and fails only when
# it tries to SAMPLE one.
#
# The mode matters more than it looks. Triage's whole decision is "is this
# beyond THIS robot, or beyond ANY robot?", and it only ever sees this robot's
# own evidence -- the fault, the log lines around it, its battery, and which
# peers are alive. Never what those peers know. So the fault has to be one
# whose own log line says "me", not "the world":
#
#   no_point_cloud  "No point cloud available."  -- this robot's depth camera
#                   produced nothing. A peer with a working camera is exactly
#                   the right answer, so triage re-delegates and the fleet
#                   auctions it. This is the one this demo wants.
#   no_leaves       "No leaves detected in the point cloud." -- the camera
#                   worked and there was nothing there. Nobody else would find
#                   leaves either, so triage correctly DROPS it and no auction
#                   happens.
#
# An earlier version of this demo hid one tree from one robot's orchard copy
# instead. That produced a real fault and a real escalation, but the robot's
# evidence read "GetTreeInfo returned empty result for tree 20" -- which is
# indistinguishable, from inside that robot, from the tree not existing at all.
# Triage dropped the task, exactly as amiga_ros2_coordinator/docs/coordinator.md
# says it should ("a spray task on a felled tree" is not worth a peer's time),
# and the auction never ran. The lesson is in mocks/failure_modes.hpp, which
# labels each mode permanent or not for precisely this reason.
FAIL_ROBOT="${FAIL_ROBOT:-1}"
SAMPLER_FAILURE_MODE="${SAMPLER_FAILURE_MODE:-no_point_cloud}"

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

# Both patterns are anchored, and that is not cosmetic: `pkill -f` matches the
# WHOLE command line of every process, so an unanchored "ign gazebo" also
# matches any shell whose own arguments happen to mention it -- including the
# one that called this function, which it then kills. Anchor on how the process
# actually starts: the gz server's argv[0] is literally `ign`, and a launch is
# always exec'd through .../bin/ros2.
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

# Before the AGENT_MODEL check on purpose: tearing a run down is exactly what
# you need to do when the environment is wrong, so it must not itself require
# the environment to be right.
if [ "${1:-}" = "stop" ]; then
    teardown
    echo "torn down: tmux session, launches, and any orphaned gz servers."
    exit 0
fi

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

# Killing the tmux session is NOT enough to end a run. `ros2 launch` starts the
# Ignition server through ros_gz_sim, which execs `ign gazebo -s` as a process
# that does not die with the launch service that started it, so every teardown
# so far has left a full physics server behind. They accumulate silently: by the
# fifth run this machine had four orphans from previous runs still integrating
# an empty world, load average ~19, and only ONE of the three robots managed to
# bring Nav2 up -- the other two spent the run logging "Timed out waiting for
# transform from amiga<i>/base_link to map". That failure looks exactly like a
# broken demo and is really just a starved one, so teardown is the script's job,
# not the reader's.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists -- attaching."
    echo "($0 stop to tear it down, orphaned gz servers included, and start clean)"
    tmux attach -t "$SESSION"
    exit 0
fi

# No session, but a previous run's gz server may still be alive and will starve
# this one if it is. Sweep before launching rather than after the demo confuses
# someone.
if pgrep -f "$GZ_PATTERN" >/dev/null 2>&1; then
    echo "orphaned gz server(s) from an earlier run are still up -- clearing them:"
    pgrep -af "$GZ_PATTERN" | sed 's/^/  /'
    teardown
fi

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

# `tmux set -g` needs a running server, and prints "no server running" and
# changes nothing without one -- while both options below are read when a pane
# is CREATED, so setting them after the sim window exists is already too late
# for the one window whose scrollback matters most.
#
# `tmux start-server` does not solve it: a server with no sessions exits again
# immediately, so the `set` calls still find nothing. The only thing that holds
# a server open is a session, hence this throwaway window -- created first, so
# the options are in place before any real window is, and killed at the end.
#
# This was invisible for as long as teardown left a tmux server behind: the
# settings landed on the leftover server from the previous run and looked like
# they worked. They only started failing once teardown began actually killing
# it, which is also when it started to matter.
tmux new-session -d -s "$SESSION" -n bootstrap
tmux set -g mouse on
# Three robots' worth of Nav2 chatter scrolls the agent story out of a default
# 2000-line pane in seconds, and ros2 launch does NOT copy node stdout into
# ~/.ros/log (output="screen" goes to the terminal and nowhere else), so
# without both of these the only record of the run is already gone by the time
# you go looking for it. Every window is also teed to $LOG_DIR.
tmux set -g history-limit 500000
tmux new-window -t "$SESSION" -n sim
tmux send-keys -t "$SESSION:sim" \
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=${ROBOT_COUNT} robot_name_prefix:=${ROBOT_PREFIX} mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true ltl_verification:=false objective_gating:=true broken_sampler_robot:=${FAIL_ROBOT} broken_sampler_mode:=${SAMPLER_FAILURE_MODE} 2>&1 | tee ${LOG_DIR}/sim.log" C-m

for i in $(seq 1 "$ROBOT_COUNT"); do
    ns="$(namespace_for "$i")"
    port=$((BASE_PORT + i - 1))
    ns_arg=""
    [ -n "$ns" ] && ns_arg="namespace:=${ns}"
    tmux new-window -t "$SESSION" -n "bt${i}"
    tmux send-keys -t "$SESSION:bt${i}" \
        "ros2 launch amiga_ros2_behavior_tree bt.launch.py ${ns_arg} port:=${port} 2>&1 | tee ${LOG_DIR}/bt${i}.log" C-m
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

    # Two things this line has to get right, both learned the hard way:
    #
    #  --full-length: ros2 topic echo truncates long string fields to ~100
    #  chars by default, which turns every plan in these panes into the same
    #  unusable '<root BTCPP_format=\"4\"...' stub with the actual edit cut off.
    #
    #  the wait loop: `ros2 topic echo` on a topic whose type it cannot resolve
    #  yet does NOT wait -- it prints "does not appear to be published yet /
    #  Could not determine the type for the passed topic" and EXITS. These
    #  windows are opened before the agents exist, precisely so they are
    #  already listening when the run starts, so every one of them used to die
    #  in its first second. That cost a whole run: /coordination/infeasible is
    #  published exactly once, is not latched, and its pane had exited long
    #  before, so the escalation left no trace anywhere.
    #
    # Everything is teed, for the same reason: a message that arrives once
    # should not be recoverable only from scrollback.
    wait_topic() { echo "until ros2 topic list 2>/dev/null | grep -qx '$1'; do sleep 2; done"; }

    watch_cmd="echo '--- ${label}: mission_planner candidates ---'; $(wait_topic "${prefix}/mission/candidate_xml"); ros2 topic echo --full-length ${prefix}/mission/candidate_xml | tee -a '$LOG_DIR/${ns}_candidates.log'"
    infeasible_cmd="echo '--- ${label}: coordination/infeasible ---'; $(wait_topic "${prefix}/coordination/infeasible"); ros2 topic echo --full-length ${prefix}/coordination/infeasible | tee -a '$LOG_DIR/${ns}_infeasible.log'"
    mission_cmd="echo '--- ${label}: mission/xml (accepted) -- also logging to ${log_file} ---'; $(wait_topic "${prefix}/mission/xml"); ros2 topic echo --full-length ${prefix}/mission/xml | tee -a '${log_file}'"

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

# Feed each robot a real, correctly-framed mission once its port is actually
# accepting connections -- the broken one to $FAIL_ROBOT, an unmodified real
# one (round-robin over MISSION_BINS) to everyone else.
tmux new-window -t "$SESSION" -n feed
feed_cmd="
bins=($(printf '%q ' "${MISSION_BINS[@]}"))
n_bins=\${#bins[@]}
for i in \$(seq 1 ${ROBOT_COUNT}); do
    port=\$(( ${BASE_PORT} + i - 1 ))
    mission=\"\${bins[\$(( (i - 1) % n_bins ))]}\"
    if [ \"\$i\" -eq ${FAIL_ROBOT} ]; then
        echo \"robot \$i is the fail case: real mission, full orchard, but its leaf sampler fails (${SAMPLER_FAILURE_MODE})\"
    fi
    echo \"waiting for robot \$i's mission port (\$port)...\"
    until (exec 3<>/dev/tcp/127.0.0.1/\$port) 2>/dev/null; do sleep 1; done
    exec 3<&- 3>&-
    # The open port only means tcp_demux is up, and tcp_demux is up long
    # before Gazebo, Nav2 and waypoint_follower are. Feeding on the port
    # alone hands the tree a mission whose action servers do not exist yet,
    # and every leaf fails instantly with 'Action server ... is not
    # reachable' -- a real fault, but the wrong one, firing before the robot
    # could even try. waypoint_follower.py creates its action servers only
    # after waitUntilNav2Active() returns, so their presence is also the
    # signal that Nav2 finished coming up.
    #
    # Counted off the action's own status topic, NOT \`ros2 action info\`.
    # An action server is exactly the publisher of <action>/_action/status, so
    # this is the same fact, but read the one way that survives this setup:
    # \`ros2 action info\` aggregates per node name and goes wrong when the graph
    # reports a node more than once, which it does here -- with a DDS profile
    # whose interface allowlist does not match the machine, every node is
    # announced twice, and the server counts come back doubled, or 0 for a
    # server that accepts goals perfectly well.
    #
    # Bounded, because the wait is per robot and the loop is serial: an
    # unbounded wait on a robot whose Nav2 never activates does not just fail
    # that robot, it silently prevents every LATER robot from ever being fed,
    # and the window sits there looking busy. A robot that has not made it in
    # five minutes is not going to, so say so and move on -- a two-robot demo
    # still shows an auction.
    ns=\"${ROBOT_PREFIX}\$i\"
    ready=1
    for act in move_to_aisle_head follow_tree_id_waypoint segment_leaves; do
        echo \"  waiting for /\$ns/\$act ...\"
        waited=0
        until timeout 10 ros2 topic info /\$ns/\$act/_action/status 2>/dev/null \\
            | grep -qE 'Publisher count: [1-9]'; do
            sleep 3
            waited=\$(( waited + 3 ))
            if [ \"\$waited\" -ge 300 ]; then
                echo \"  robot \$i never brought up \$act -- its Nav2 did not\"
                echo \"  finish activating (check the sim window for the\"
                echo \"  lifecycle_manager bonds). NOT feeding this robot.\"
                ready=0
                break
            fi
        done
        [ \"\$ready\" -eq 1 ] || break
    done
    if [ \"\$ready\" -eq 0 ]; then
        if [ \"\$i\" -eq ${FAIL_ROBOT} ]; then
            echo \"  robot \$i is FAIL_ROBOT -- without it there is no fault to\"
            echo \"  shed and no auction. Tear down and start clean.\"
        fi
        continue
    fi
    echo \"feeding robot \$i on port \$port: \$mission\"
    nc -q 1 127.0.0.1 \$port < \"\$mission\"
done
echo
echo 'all missions fed.'
echo 'watch: bt<i> for BT execution, watch/infeasible/mission-xml for the LLM+auction pipeline'
echo 'logs: $LOG_DIR/*_mission_xml.log'
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

# Every real window exists now, so the session no longer needs the window that
# was only there to hold the server open long enough to configure it.
tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

#!/bin/bash
#
# One robot's half of the fleet, run on its own machine with its own real
# LoRa radio. Companion to demo_llm_auction.sh, which runs the whole fleet in
# one tmux session on one machine over a *simulated* radio (a virtual medium
# backed by local ptys) -- this instead brings up exactly one simulated
# robot, its own Gazebo world, its own BT, its own coordinator, and reaches
# the other machine's copy of this script only over actual LoRa hardware.
# Run it once per physical machine, each with its own robot id:
#
#   machine 1: ./demo_llm_auction_robot.sh 1
#   machine 2: ./demo_llm_auction_robot.sh 2
#
# Robot 1 is the one with the broken depth camera (FAIL_ROBOT below) -- it
# drives to its trees fine and fails only when it tries to SAMPLE one, the
# same real "No point cloud available." fault demo_llm_auction.sh injects.
# Robot 2 is clean. See that script's own header for the full story of what
# happens next (repair -> triage -> auction -> absorption): it is identical
# here, just carried over real airtime instead of a virtual medium.
#
# <robot_id> becomes this robot's LoRa/coordinator node_id -- the identity
# stamped on every frame it sends. It MUST be unique across machines: two
# robots sharing a node_id have each other's real transmissions silently
# discarded as self-echoes by the reliability layer (rx_self in
# amiga_ros2_comms/amiga_ros2_comms/reliability/session.py), which looks
# exactly like the radios not working when they actually are.
#
# Usage:
#   export AGENT_MODEL=hosted_vllm/openai/gpt-oss-120b
#   export AGENT_API_BASE=http://100.88.70.65:8000/v1
#   ./demo_llm_auction_robot.sh 1          # on machine 1
#   ./demo_llm_auction_robot.sh 2          # on machine 2
#   ./demo_llm_auction_robot.sh 1 stop     # tear that machine's run down
#
# For everyone on one machine over a simulated radio instead, use
# demo_llm_auction.sh -- node ids there are still assigned automatically
# (1..ROBOT_COUNT), unrelated to and unaffected by this script.

set -uo pipefail

ROBOT_ID="${1:-}"
ACTION="${2:-}"

if ! [[ "$ROBOT_ID" =~ ^[0-9]+$ ]] || [ "$ROBOT_ID" -lt 1 ]; then
    echo "Usage: $0 <robot_id> [stop]" >&2
    echo "  <robot_id>: this machine's robot. 1 has the broken camera" >&2
    echo "  (FAIL_ROBOT below); 2+ are clean. Must be unique across machines" >&2
    echo "  -- it becomes this robot's LoRa node_id." >&2
    exit 1
fi

SESSION="llm-demo-r${ROBOT_ID}"
BASE_PORT="${BASE_PORT:-12346}"
LOG_DIR="/tmp/llm-demo-logs/robot${ROBOT_ID}"

# Real, checked-in two-frame (XML + orchard JSON) mission payloads -- the same
# set demo_llm_auction.sh and demo_vlm_human.sh use, picked round-robin by
# robot id so two robots don't open on the same aisle (see that script's own
# comment on why that was a bottleneck worth avoiding).
EXAMPLES_DIR="amiga_ros2_behavior_tree/examples"
MISSION_BINS=(
    "${EXAMPLES_DIR}/sample_20_64.bin"
    "${EXAMPLES_DIR}/sample_22_66.bin"
    "${EXAMPLES_DIR}/sample_24_68.bin"
)
MISSION="${MISSION_BINS[$(( (ROBOT_ID - 1) % ${#MISSION_BINS[@]} ))]}"

# Robot 1 has the broken depth camera; see demo_llm_auction.sh's header for
# why this fault (not a missing tree) is the one that gets auctioned.
FAIL_ROBOT="${FAIL_ROBOT:-1}"
SAMPLER_FAILURE_MODE="${SAMPLER_FAILURE_MODE:-no_point_cloud}"

HEADLESS="${HEADLESS:-false}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_PATH" || exit 1

# Same anchoring reasoning as demo_llm_auction.sh: match how the processes
# actually start, not a loose substring, so teardown never kills its own
# caller and never misses an orphan.
GZ_PATTERN='^ign gazebo'
LAUNCH_PATTERN='bin/ros2 launch amiga_ros2_(gazebo|behavior_tree)'

teardown() {
    tmux kill-session -t "$SESSION" 2>/dev/null
    pkill -f "$LAUNCH_PATTERN" 2>/dev/null
    pkill -f "$GZ_PATTERN" 2>/dev/null
    for _ in $(seq 1 10); do
        pgrep -f "$GZ_PATTERN" >/dev/null 2>&1 || return 0
        sleep 1
    done
    pkill -9 -f "$GZ_PATTERN" 2>/dev/null
    pkill -9 -f "$LAUNCH_PATTERN" 2>/dev/null
}

# Before the AGENT_MODEL check on purpose, same as demo_llm_auction.sh: tear
# down must not itself require the environment to be right.
if [ "$ACTION" = "stop" ]; then
    teardown
    echo "robot ${ROBOT_ID}: torn down (tmux session, launch, any orphaned gz server on this machine)."
    exit 0
fi

if [ -z "${AGENT_MODEL:-}" ]; then
    echo "AGENT_MODEL is not set." >&2
    echo "export AGENT_MODEL=... and AGENT_API_BASE=... first (amiga_ros2_agents/README.md)" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists -- attaching."
    echo "($0 $ROBOT_ID stop to tear it down and start clean)"
    tmux attach -t "$SESSION"
    exit 0
fi

# This machine's own leftover gz server, same reasoning as
# demo_llm_auction.sh: sweep before launching, not after the demo confuses
# someone. Only ever touches this machine -- the other robot's machine has
# its own copy of this script and its own teardown.
if pgrep -f "$GZ_PATTERN" >/dev/null 2>&1; then
    echo "orphaned gz server(s) from an earlier run are still up on this machine -- clearing them:"
    pgrep -af "$GZ_PATTERN" | sed 's/^/  /'
    teardown
fi

mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

# Radio medium for THIS robot. Same Gazebo sim either way; this only chooses
# what carries its coordination traffic. Preset LORA_MODE to skip the prompt
# -- required for a non-interactive run, since there is no terminal to prompt
# on. Real hardware is the point of running one robot per machine at all;
# sim is left in only as a one-machine sanity check of this script by itself
# (it will never see a peer, since nothing else joins its virtual medium).
LORA_MODE="${LORA_MODE:-}"
if [ -z "$LORA_MODE" ]; then
    if [ -t 0 ]; then
        echo "Radio medium for robot ${ROBOT_ID}:"
        echo "  1) Simulated LoRa (virtual medium -- talks to no one else) [default]"
        echo "  2) Real LoRa hardware over serial"
        read -r -p "Choose [1/2]: " lora_choice
        case "$lora_choice" in
            2) LORA_MODE="hardware" ;;
            *) LORA_MODE="sim" ;;
        esac
    else
        LORA_MODE="sim"
    fi
fi

LORA_LAUNCH_ARGS="lora_hardware:=false"
if [ "$LORA_MODE" = "hardware" ]; then
    LORA_SERIAL_PORT="${LORA_SERIAL_PORT:-}"
    if [ -z "$LORA_SERIAL_PORT" ]; then
        if [ -t 0 ]; then
            read -r -p "Serial port for this robot's LoRa radio (e.g. /dev/ttyUSB0): " LORA_SERIAL_PORT
        fi
        if [ -z "$LORA_SERIAL_PORT" ]; then
            echo "LORA_MODE=hardware needs LORA_SERIAL_PORT (e.g. /dev/ttyUSB0)." >&2
            exit 1
        fi
    fi
    LORA_LAUNCH_ARGS="lora_hardware:=true lora_serial_ports:=${LORA_SERIAL_PORT}"
    echo "robot ${ROBOT_ID}: LoRa medium: REAL hardware over serial (${LORA_SERIAL_PORT})"
else
    echo "robot ${ROBOT_ID}: LoRa medium: simulated (talks to no one else)"
fi

# Same throwaway-window trick as demo_llm_auction.sh: `tmux set -g` needs a
# running server, and both options below are read only when a pane is
# created, so this has to exist before the real windows do.
tmux new-session -d -s "$SESSION" -n bootstrap
tmux set -g mouse on
tmux set -g history-limit 500000

tmux new-window -t "$SESSION" -n sim
tmux send-keys -t "$SESSION:sim" \
    "ros2 launch amiga_ros2_gazebo sim_bringup.launch.py robot_count:=1 mission_port_base:=${BASE_PORT} headless:=${HEADLESS} launch_bt:=false launch_coordination:=true launch_agents:=true objective_gating:=true node_id:=${ROBOT_ID} broken_sampler_robot:=${FAIL_ROBOT} broken_sampler_mode:=${SAMPLER_FAILURE_MODE} ${LORA_LAUNCH_ARGS} 2>&1 | tee ${LOG_DIR}/sim.log" C-m

tmux new-window -t "$SESSION" -n bt
tmux send-keys -t "$SESSION:bt" \
    "ros2 launch amiga_ros2_behavior_tree bt.launch.py port:=${BASE_PORT} 2>&1 | tee ${LOG_DIR}/bt.log" C-m

tmux new-window -t "$SESSION" -n agents
tmux send-keys -t "$SESSION:agents" \
    "python3 ${PROJECT_PATH}/scripts/watch_agents.py 2>&1 | tee ${LOG_DIR}/agents.log" C-m

# Same two lessons as demo_llm_auction.sh's per-robot panes: --full-length so
# ros2 topic echo doesn't truncate the XML, and the wait loop so a pane
# opened before the agents exist doesn't just print "not published yet" and
# exit before there is anything to show.
wait_topic() { echo "until ros2 topic list 2>/dev/null | grep -qx '$1'; do sleep 2; done"; }

tmux new-window -t "$SESSION" -n watch
tmux send-keys -t "$SESSION:watch" \
    "echo '--- robot ${ROBOT_ID}: mission_planner candidates ---'; $(wait_topic "/mission/candidate_xml"); ros2 topic echo --full-length /mission/candidate_xml | tee -a '${LOG_DIR}/candidates.log'" C-m

tmux new-window -t "$SESSION" -n infeasible
tmux send-keys -t "$SESSION:infeasible" \
    "echo '--- robot ${ROBOT_ID}: coordination/infeasible ---'; $(wait_topic "/coordination/infeasible"); ros2 topic echo --full-length /coordination/infeasible | tee -a '${LOG_DIR}/infeasible.log'" C-m

tmux new-window -t "$SESSION" -n mission-xml
tmux send-keys -t "$SESSION:mission-xml" \
    "echo '--- robot ${ROBOT_ID}: mission/xml (accepted) ---'; $(wait_topic "/mission/xml"); ros2 topic echo --full-length /mission/xml | tee -a '${LOG_DIR}/mission_xml.log'" C-m

# Feed this one robot once its port is actually accepting connections. Same
# two lessons as demo_llm_auction.sh's feed loop: the open mission port only
# means tcp_demux is up, not that Nav2's action servers exist yet, and that
# has to be counted off the actions' own status topics (see that script's
# comment on why `ros2 action info` is the wrong tool under this DDS setup).
tmux new-window -t "$SESSION" -n feed
feed_cmd="
port=${BASE_PORT}
if [ ${ROBOT_ID} -eq ${FAIL_ROBOT} ]; then
    echo 'robot ${ROBOT_ID} is the fail case: real mission, full orchard, but its leaf sampler fails (${SAMPLER_FAILURE_MODE})'
fi
echo \"waiting for this robot's mission port (\$port)...\"
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
            echo \"  this robot never brought up \$act -- its Nav2 did not\"
            echo \"  finish activating (check the sim window for the\"
            echo \"  lifecycle_manager bonds). NOT feeding it.\"
            ready=0
            break
        fi
    done
    [ \"\$ready\" -eq 1 ] || break
done
if [ \"\$ready\" -eq 0 ]; then
    echo 'robot ${ROBOT_ID} never came up -- nothing fed. Tear down and start clean.'
else
    echo \"feeding robot ${ROBOT_ID} on port \$port: ${MISSION}\"
    nc -q 1 127.0.0.1 \$port < \"${MISSION}\"
    echo
    echo 'mission fed.'
    echo 'watch: bt for BT execution, agents for the decision story, watch/infeasible/mission-xml for the LLM+auction pipeline'
    echo 'logs: ${LOG_DIR}/*.log'
fi
"
tmux send-keys -t "$SESSION:feed" "$feed_cmd" C-m

tmux kill-window -t "$SESSION:bootstrap" 2>/dev/null

tmux select-window -t "$SESSION:sim"
tmux attach -t "$SESSION"

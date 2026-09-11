#!/bin/bash
#
# One command, walk away: runs the deterministic (no-LLM) baseline arm over
# every pre-registered scenario in both ablation harnesses --
#   single-robot (research_harness.py): missing_tree, stuck_robot, sick_tree
#   fleet        (fleet_harness.py):    peer_capable, no_peers, no_capability,
#                                       transient, fleet_wide, new_work
# 5 trials per scenario by default, run sequentially. AGENT_POLICY=deterministic
# calls no model, so this needs no vLLM/OpenAI endpoint reachable -- only
# ROS2 sourced.
#
# Sequential on purpose: each harness launches and kills its own ROS2
# processes (planner+arbiter, or triage_node) by fixed node name, so two
# harnesses running at once would fight over them. Everything is teed to a
# log file as it runs, so `nohup scripts/run_deterministic_baseline.sh &`
# (or just leaving this in its own terminal) gives you a complete record to
# come back to even if the session is gone.
#
# Usage (run INSIDE the container, ROS2 sourced):
#   scripts/run_deterministic_baseline.sh          # 5 trials/scenario
#   scripts/run_deterministic_baseline.sh 10       # 10 trials/scenario
#   TRIALS=20 scripts/run_deterministic_baseline.sh

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"
AGENTS_DIR="$PROJECT_PATH/amiga_ros2_agents"

TRIALS="${1:-${TRIALS:-5}}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="$AGENTS_DIR/runs/baseline_${STAMP}"
mkdir -p "$LOG_DIR"

SINGLE_LOG="$LOG_DIR/single.log"
FLEET_LOG="$LOG_DIR/fleet.log"
SUMMARY_LOG="$LOG_DIR/summary.log"

cd "$AGENTS_DIR" || exit 1
START_TS=$SECONDS

echo "=== single-robot: research_harness.py --arms deterministic --scenarios all --trials $TRIALS ==="
python3 scripts/scenarios/research_harness.py \
    --arms deterministic --scenarios all --trials "$TRIALS" \
    2>&1 | tee "$SINGLE_LOG"
SINGLE_STATUS=${PIPESTATUS[0]}

echo
echo "=== fleet: fleet_harness.py --arms deterministic --scenarios all --trials $TRIALS ==="
python3 scripts/scenarios/fleet_harness.py \
    --arms deterministic --scenarios all --trials "$TRIALS" \
    2>&1 | tee "$FLEET_LOG"
FLEET_STATUS=${PIPESTATUS[0]}

# Both harnesses print their own results path as "... -> <path>" on their
# last line; pull it back out rather than reconstructing run_id ourselves.
SINGLE_RESULTS="$(grep -m1 -- '-> ' "$SINGLE_LOG" | sed -E 's/.*-> //')"
FLEET_RESULTS="$(grep -m1 -- '-> ' "$FLEET_LOG" | sed -E 's/.*-> //')"

{
    echo "single-robot harness: exit=$SINGLE_STATUS results=${SINGLE_RESULTS:-NONE}"
    echo "fleet harness:        exit=$FLEET_STATUS results=${FLEET_RESULTS:-NONE}"
    echo

    if [ -n "$SINGLE_RESULTS" ] && [ -f "$SINGLE_RESULTS" ]; then
        echo "--- single-robot accuracy table ---"
        python3 scripts/scenarios/analyze.py "$SINGLE_RESULTS"
        echo
    else
        echo "single-robot: no results.jsonl found -- see $SINGLE_LOG"
        echo
    fi

    if [ -n "$FLEET_RESULTS" ] && [ -f "$FLEET_RESULTS" ]; then
        echo "--- fleet accuracy table ---"
        python3 scripts/scenarios/analyze.py "$FLEET_RESULTS"
    else
        echo "fleet: no results.jsonl found -- see $FLEET_LOG"
    fi
} 2>&1 | tee "$SUMMARY_LOG"

ELAPSED=$((SECONDS - START_TS))
echo
echo "done in ${ELAPSED}s. logs and tables under: $LOG_DIR"
echo "  $SINGLE_LOG"
echo "  $FLEET_LOG"
echo "  $SUMMARY_LOG"

if [ "$SINGLE_STATUS" -ne 0 ] || [ "$FLEET_STATUS" -ne 0 ]; then
    echo "WARNING: at least one harness exited non-zero -- check the logs above." >&2
    exit 1
fi

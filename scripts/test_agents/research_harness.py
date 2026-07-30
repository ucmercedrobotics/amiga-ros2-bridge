#!/usr/bin/env python3
"""
Research harness for the mission-planner / arbiter replanning study.

For every (model, scenario, rep) cell it:
  1. launches a FRESH planner + arbiter process (clean memory => independent trial)
  2. waits until both A2A ports accept connections (readiness, not a blind sleep)
  3. publishes the scenario mission XML, then the vague failure event
  4. captures viability budget, candidate/final XML, abort, gave_up, rejections, latency
  5. kills both processes and appends ONE json line to results.jsonl

Raw node stdout/stderr for each trial is saved under runs/<run_id>/logs/ so
prompts / token logs can be recovered later without re-running.

Run INSIDE the container with ROS2 sourced (and vLLM already serving):
  python3 scripts/research_harness.py --models local --scenarios all --reps 5
"""

import argparse
import json
import os
import signal
import socket
import subprocess
import time
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ======================================================================
# CONFIG — edit to match your machine, then leave alone per-run.
# ======================================================================

PYBIN = sys.executable

PLANNER_CMD = [PYBIN, "-m", "amiga_ros2_mission_planner.mission_planner_node"]
ARBITER_CMD = [PYBIN, "-m", "amiga_ros2_arbiter.arbiter_node"]

PLANNER_PORT = 10001   # A2A readiness ports (TCP-connectable == node up)
ARBITER_PORT = 10003

COMMON_ENV = {
    "WORLD_STATE_URL": os.environ.get("WORLD_STATE_URL", "http://localhost:10004/"),
    "ENV_FILE_PATH": os.environ.get("ENV_FILE_PATH", "/amiga-ros2-bridge/.env"),
}

# The independent variable. Each entry = env vars merged in to select that model.
# Your nodes derive ACTIVE_MODEL from LOCAL_MODEL, so LOCAL_MODEL/LOCAL_API_BASE
# is the lever. Add a row per model you benchmark.
MODELS = {
    "local": {
        "LOCAL_MODEL": "hosted_vllm/openai/gpt-oss-20b",
        "LOCAL_API_BASE": "http://localhost:8000/v1",
        "MODEL_TEMPERATURE": "0.2",
        "MODEL_MAX_TOKENS": "10000",
    },
    "cloud": {
        "LOCAL_MODEL": "openai/gpt-5.6-sol",   # litellm routes plain "gpt-5.6-sol" to OpenAI
        "LOCAL_API_BASE": "",      # empty => `"" or None` => official OpenAI endpoint
        "MODEL_TEMPERATURE": "1.0",
        "MODEL_MAX_TOKENS": "10000",
    },
}

TRIAL_TIMEOUT_SEC = 300     # max wait for a terminal outcome per trial
POST_READY_WAIT_SEC = 3.0   # grace after ports open so subs are wired
LAUNCH_TIMEOUT_SEC = 90     # max wait for both ports to open
BUDGET_WAIT_SEC = 180       # max wait for the arbiter's viability budget (~2min LLM call)

def _tree_steps(ids):
    rows = []
    for i in ids:
        rows.append(
            f'      <MoveToTreeID name="Visit_Tree_{i}" '
            f'action_name="follow_tree_id_waypoint" id="{i}" approach_tree="true"/>'
        )
        rows.append(
            f'      <SampleLeaf name="Sample_Leaves_Tree_{i}" '
            f'action_name="segment_leaves"/>'
        )
    return "\n".join(rows)


def _row_mission(mission_text, tree_ids):
    return (
        '<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">\n'
        f"  <Mission>{mission_text}</Mission>\n"
        '  <BehaviorTree ID="Sample_Leaves_Row">\n'
        "    <Sequence>\n"
        f"{_tree_steps(tree_ids)}\n"
        "    </Sequence>\n"
        "  </BehaviorTree>\n"
        "</root>"
    )


# Three scenarios. Reasons are deliberately vague (observation only, no menu of
# actions) so the model must reason. `expected` documents the pre-registered
# hypothesis for grading; the model never sees it.
SCENARIOS = {
    "missing_tree": {
        "xml": _row_mission(
            "sample leaves from every tree in the row, trees 1 through 10", range(1, 11)
        ),
        "failure": {
            "node": "Visit_Tree_5",
            "status": "FAILURE",
            "reason": "no tree at the mapped location for tree 5; the spot is bare soil.",
        },
        "expected": "continue remaining trees; do NOT abort whole mission",
    },
    "stuck_robot": {
        "xml": _row_mission("sample leaves from trees 1 through 10", range(1, 11)),
        "failure": {
            "node": "Visit_Tree_3",
            "status": "FAILURE",
            "reason": "the base is not moving; wheels are sunk in a dirt pit and no "
            "forward or backward motion is happening.",
        },
        "expected": "recognize futility and ABORT",
    },
    "sick_tree": {
        "xml": _row_mission(
            "sample leaves from trees 1 through 10 in the current row", range(1, 11)
        ),
        "failure": {
            "node": "Visit_Tree_4",
            "status": "DETECTION",
            "reason": "a tree in the adjacent row (id 54) shows heavy leaf necrosis and "
            "possible blight; it is not in the current mission.",
        },
        "expected": "characterize decision: ignore / append / re-prioritize",
    },
}

# ======================================================================
# Harness internals — no need to edit below.
# ======================================================================


def port_open(port, host="127.0.0.1"):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host, port)) == 0


def wait_for_ports(ports, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if all(port_open(p) for p in ports):
            return True
        time.sleep(0.5)
    return False


def launch_node(cmd, env_extra, log_path):
    env = os.environ.copy()
    env.update(COMMON_ENV)
    env.update(env_extra)
    logf = open(log_path, "w")
    proc = subprocess.Popen(
        cmd,
        env=env,
        stdout=logf,
        stderr=subprocess.STDOUT,
        start_new_session=True,  # own process group => clean kill
    )
    return proc, logf


def kill_node(proc, logf):
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    logf.close()


class Capture(Node):
    """One reusable ROS node that publishes the trial and records all outputs."""

    def __init__(self):
        super().__init__("research_harness")
        self.xml_pub = self.create_publisher(String, "/mission/xml", 10)
        self.bt_pub = self.create_publisher(String, "/bt/status_change", 10)
        self.create_subscription(String, "/mission/xml", self._on_final, 10)
        self.create_subscription(String, "/mission/candidate_xml", self._on_candidate, 10)
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)
        self.create_subscription(String, "/mission/rejection", self._on_rejection, 10)
        self.create_subscription(String, "/mission/planner_status", self._on_status, 10)
        self.create_subscription(String, "/mission/viability_budget", self._on_budget, 10)
        self.reset(None)

    def reset(self, original_xml):
        self._original = original_xml
        self.candidate_xml = None
        self.final_xml = None
        self.abort = None
        self.gave_up = None
        self.rejections = []
        self.viability_budget = None

    def _on_final(self, msg):
        if msg.data == self._original:
            return  # echo of what we published
        if self.final_xml is None:
            self.final_xml = msg.data

    def _on_candidate(self, msg):
        if self.candidate_xml is None:
            self.candidate_xml = msg.data

    def _on_abort(self, msg):
        if self.abort is None:
            self.abort = msg.data

    def _on_status(self, msg):
        if self.gave_up is None:
            self.gave_up = msg.data

    def _on_rejection(self, msg):
        self.rejections.append(msg.data)

    def _on_budget(self, msg):
        try:
            self.viability_budget = json.loads(msg.data).get("viability_budget")
        except json.JSONDecodeError:
            pass

    def run_trial(self, xml, failure):
        self.reset(xml)
        m = String(); m.data = xml
        self.xml_pub.publish(m)
        time.sleep(2.0)  # let planner + arbiter register the mission

        f = String()
        fail = dict(failure)
        fail["timestamp_ms"] = int(time.time() * 1000)
        f.data = json.dumps(fail)
        t0 = time.time()
        self.bt_pub.publish(f)
       

        deadline = t0 + TRIAL_TIMEOUT_SEC
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if (self.final_xml is not None
                    or self.abort is not None
                    or self.gave_up is not None):
                # settle so a trailing message (and the budget) also lands
                settle = time.time() + 2.0
                while time.time() < settle:
                    rclpy.spin_once(self, timeout_sec=0.2)
                break
        latency = time.time() - t0

        if self.abort is not None:
            decision = "abort"
        elif self.gave_up is not None:
            decision = "gave_up"
        elif self.final_xml is not None:
            decision = "accept"
        elif self.candidate_xml is not None:
            decision = "candidate_only"  # planner produced, arbiter never accepted
        else:
            decision = "timeout"

        return {
            "decision": decision,
            "candidate_xml": self.candidate_xml,
            "final_xml": self.final_xml,
            "abort_reason": self.abort,
            "gave_up": self.gave_up,
            "rejection_count": len(self.rejections),
            "rejections": self.rejections,
            "viability_budget": self.viability_budget,
            "latency_sec": round(latency, 2),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--models", default="local", help="comma list of MODELS keys, or 'all'")
    ap.add_argument("--scenarios", default="all", help="comma list of SCENARIOS keys, or 'all'")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--outdir", default="runs")
    args = ap.parse_args()

    models = list(MODELS) if args.models == "all" else args.models.split(",")
    scenarios = list(SCENARIOS) if args.scenarios == "all" else args.scenarios.split(",")
    for k in models:
        assert k in MODELS, f"unknown model {k}"
    for k in scenarios:
        assert k in SCENARIOS, f"unknown scenario {k}"

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.outdir) / run_id
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    results_path = run_dir / "results.jsonl"
    print(f"[harness] run_id={run_id}  -> {results_path}")

    rclpy.init()
    cap = Capture()

    total = len(models) * len(scenarios) * args.reps
    done = 0
    try:
        for model in models:
            for scenario in scenarios:
                sc = SCENARIOS[scenario]
                for rep in range(args.reps):
                    done += 1
                    tag = f"{model}__{scenario}__rep{rep}"
                    print(f"[harness] ({done}/{total}) {tag} launching…", flush=True)

                    plog = run_dir / "logs" / f"{tag}.planner.log"
                    alog = run_dir / "logs" / f"{tag}.arbiter.log"
                    pproc, pf = launch_node(PLANNER_CMD, MODELS[model], plog)
                    aproc, af = launch_node(ARBITER_CMD, MODELS[model], alog)

                    record = {
                        "run_id": run_id,
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "model": model,
                        "model_env": MODELS[model],
                        "scenario": scenario,
                        "rep": rep,
                        "expected": sc["expected"],
                        "failure_reason": sc["failure"]["reason"],
                    }
                    try:
                        if not wait_for_ports([PLANNER_PORT, ARBITER_PORT], LAUNCH_TIMEOUT_SEC):
                            record["decision"] = "launch_failed"
                            record["error"] = "ports did not open"
                        else:
                            time.sleep(POST_READY_WAIT_SEC)
                            record.update(cap.run_trial(sc["xml"], sc["failure"]))
                    finally:
                        kill_node(aproc, af)
                        kill_node(pproc, pf)
                        time.sleep(8.0)  # let ports release before next launch

                    with open(results_path, "a") as fh:
                        fh.write(json.dumps(record) + "\n")
                    print(f"[harness]      -> {record.get('decision')} "
                          f"(budget={record.get('viability_budget')}, "
                          f"{record.get('latency_sec','-')}s)", flush=True)
    finally:
        cap.destroy_node()
        rclpy.shutdown()

    print(f"[harness] done. {results_path}")


if __name__ == "__main__":
    main()
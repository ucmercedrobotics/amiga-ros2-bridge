#!/usr/bin/env python3
"""
Research harness for the mission-planner / arbiter replanning study.

For every (arm, scenario, trial) cell it:
  1. launches a FRESH planner + arbiter process (clean memory => independent trial)
  2. waits until both agents latch a status snapshot (readiness, not a blind sleep)
  3. publishes the scenario mission XML, then the vague failure event
  4. captures viability budget, candidate/final XML, abort, gave_up, rejections, latency
  5. kills both processes and appends ONE json line to results.jsonl

Raw node stdout/stderr for each trial is saved under runs/<run_id>/logs/ so
prompts / token logs can be recovered later without re-running.

The independent variable used to be called `--models`, because every arm was
a model. It no longer is: `deterministic` is a third arm alongside `llm-local`
and `llm-cloud`, and it sets no model variable at all -- see ARMS below.

`--reps` later became `--seeds` on the theory that a shared seed would let
both arms be paired trial-for-trial for McNemar's test (see
`ablation-experiment-spec.md`). That pairing never happened: no call site in
this harness reads the seed value at all, and R4 in the spec's Revisions
section retires the idea. The `seed` field in each record (and the loop
variable below) is nothing more than a trial index in this harness -- it
selects which iteration of the outer loop a trial belongs to and has zero
effect on execution, so it carries no variance-reducing or pairing promise
and nobody should read one into it. It is kept, unrenamed, only because the
record schema is shared verbatim with `fleet_harness.py` (see that file's
docstring), where a same-named field genuinely does drive `Fleet(seed=...)`.

The CLI flag is `--trials` -- what it actually controls is how many
repetitions run per (arm, scenario) cell, which the LLM arms need because
they are stochastic and the deterministic arm needs only for the
nondeterminism check in `analyze.py` to have more than one data point.
`--seeds` is kept as an accepted synonym so an existing invocation still
works.

Scenarios live in `registry.py`, not here, and are pre-registered: the
`expected` hypothesis and the deterministic `scorer` that turns a decision
into `correct` are committed before a single trial runs, so scoring cannot
drift toward a result once results exist. See registry.py's module docstring.

Run INSIDE the container with ROS2 sourced (and vLLM already serving, for the
llm arms -- `deterministic` needs no model endpoint reachable at all):
  python3 scripts/scenarios/research_harness.py --arms llm-local --scenarios all --trials 5

The pre-ablation invocation still works exactly as it always did -- `--arms`
defaults to `llm-local` (what used to be called `local`), so this bare form
never runs the deterministic arm unless you ask for it with `--arms` or
`--arms all`:
  python3 scripts/research_harness.py --models local --scenarios all --reps 5

`--models`/`--reps` are deprecated synonyms for `--arms`/`--trials`, kept
fully functional (see their `--help` text) so old invocations and scripts
never broke when this axis was renamed.
"""

import argparse
import json
import os
import signal
import subprocess
import time
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from amiga_ros2_agents.runtime.status import STATUS_QOS
from rclpy.node import Node
from std_msgs.msg import String

from registry import SINGLE_SCENARIOS, git_sha

# ======================================================================
# CONFIG — edit to match your machine, then leave alone per-run.
# ======================================================================

PYBIN = sys.executable

PLANNER_CMD = [PYBIN, "-m", "amiga_ros2_agents.replanning.mission_planner_node"]
ARBITER_CMD = [PYBIN, "-m", "amiga_ros2_agents.replanning.arbiter_node"]

# Readiness: each agent latches a startup snapshot on /agents/<name>/status
# (TRANSIENT_LOCAL), so receiving one means the node is up with its callbacks wired.
READY_AGENTS = ["mission_planner", "arbiter"]

COMMON_ENV = {
    "ENV_FILE_PATH": os.environ.get("ENV_FILE_PATH", "/amiga-ros2-bridge/.env"),
}

# The independent variable. Each entry = env vars merged in to select that
# arm. `AGENT_POLICY` is the lever `amiga_ros2_agents.runtime.policy` reads
# once at import (see that module's docstring): "deterministic" routes every
# one of the five reasoning seams through a closed-form rule over structured
# state, and "llm" (the default if unset, but every row here sets it
# explicitly so a results record is never ambiguous about which arm produced
# it) leaves the model call in place, selected by LOCAL_MODEL/LOCAL_API_BASE
# exactly as before this axis had a name change. `deterministic` sets no
# model variable at all -- that absence is the point of the arm, not an
# oversight, and `AGENT_POLICY=deterministic` end-to-end with no model
# endpoint reachable is one of this study's own verification steps.
ARMS = {
    "deterministic": {
        "AGENT_POLICY": "deterministic",
    },
    "llm-local": {
        "AGENT_POLICY": "llm",
        "LOCAL_MODEL": "hosted_vllm/openai/gpt-oss-20b",
        "LOCAL_API_BASE": "http://localhost:8000/v1",
        "MODEL_TEMPERATURE": "0.2",
        "MODEL_MAX_TOKENS": "8192",
    },
    "llm-cloud": {
        "AGENT_POLICY": "llm",
        "LOCAL_MODEL": "openai/gpt-5.6-sol",  # litellm routes plain "gpt-5.6-sol" to OpenAI
        "LOCAL_API_BASE": "",  # empty => `"" or None` => official OpenAI endpoint
        "MODEL_TEMPERATURE": "0.2",
        "MODEL_MAX_TOKENS": "10000",
    },
}

TRIAL_TIMEOUT_SEC = 300  # max wait for a terminal outcome per trial
POST_READY_WAIT_SEC = 3.0  # grace after readiness so pub/sub matching settles
LAUNCH_TIMEOUT_SEC = 90  # max wait for both agents to report ready
BUDGET_WAIT_SEC = 180  # max wait for the arbiter's viability budget (~2min LLM call)


# ======================================================================
# Harness internals — no need to edit below.
# ======================================================================


def wait_for_agents(agent_names, timeout):
    """True once every agent has published its latched status snapshot."""
    probe = Node("research_harness_probe")
    seen = set()

    for name in agent_names:
        probe.create_subscription(
            String,
            f"/agents/{name}/status",
            lambda _msg, n=name: seen.add(n),
            STATUS_QOS,
        )

    deadline = time.time() + timeout
    while len(seen) < len(agent_names) and time.time() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.5)

    probe.destroy_node()
    return len(seen) == len(agent_names)


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
        self.create_subscription(
            String, "/mission/candidate_xml", self._on_candidate, 10
        )
        self.create_subscription(String, "/mission/abort", self._on_abort, 10)
        self.create_subscription(String, "/mission/rejection", self._on_rejection, 10)
        self.create_subscription(String, "/mission/planner_status", self._on_status, 10)
        self.create_subscription(
            String, "/mission/viability_budget", self._on_budget, 10
        )
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
        m = String()
        m.data = xml
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
            if (
                self.final_xml is not None
                or self.abort is not None
                or self.gave_up is not None
            ):
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


# Pre-ablation `--models` names, mapped onto today's ARMS keys. `local` and
# `cloud` never collide with a current ARMS key, so this mapping is safe to
# apply unconditionally to whatever `--models` was given.
OLD_MODEL_ALIASES = {"local": "llm-local", "cloud": "llm-cloud"}

# What `--models all` meant before `deterministic` existed as a third arm:
# just the two model arms, never the deterministic one. `--arms all` (the
# current flag) deliberately means all three -- that's a new, opt-in
# capability this study added, not something the deprecated spelling should
# suddenly start doing.
OLD_MODELS_ALL = "llm-local,llm-cloud"


def main():
    ap = argparse.ArgumentParser()
    arms_group = ap.add_mutually_exclusive_group()
    arms_group.add_argument(
        "--arms",
        default=None,
        help="comma list of ARMS keys, or 'all' for every arm including "
        "'deterministic' (default: llm-local, i.e. the pre-ablation default "
        "arm, unchanged)",
    )
    arms_group.add_argument(
        "--models",
        default=None,
        help="[deprecated synonym for --arms] comma list of the old model "
        "names: 'local' (-> llm-local), 'cloud' (-> llm-cloud), or 'all' "
        "(-> llm-local,llm-cloud -- matches the historic meaning of "
        "`--models all`, which never included a deterministic arm)",
    )
    ap.add_argument(
        "--scenarios",
        default="all",
        help="comma list of SINGLE_SCENARIOS keys, or 'all'",
    )
    ap.add_argument(
        "--trials",
        "--seeds",
        "--reps",
        dest="trials",
        type=int,
        default=5,
        help="repetitions per (arm, scenario); the LLM arms need these because "
        "they are stochastic. `seed` in each record is a bare trial index "
        "with no effect on execution (see the module docstring) -- "
        "'--seeds' and '--reps' [deprecated synonym] are both accepted for "
        "this flag, neither is a hint that it seeds anything here.",
    )
    ap.add_argument("--outdir", default="runs")
    args = ap.parse_args()

    if args.models is not None:
        arms_spec = (
            OLD_MODELS_ALL
            if args.models == "all"
            else ",".join(
                OLD_MODEL_ALIASES.get(tok, tok) for tok in args.models.split(",")
            )
        )
    elif args.arms is not None:
        arms_spec = args.arms
    else:
        arms_spec = "llm-local"  # the restored, pre-ablation default

    arms = list(ARMS) if arms_spec == "all" else arms_spec.split(",")
    scenarios = (
        list(SINGLE_SCENARIOS) if args.scenarios == "all" else args.scenarios.split(",")
    )
    for k in arms:
        assert k in ARMS, f"unknown arm {k}"
    for k in scenarios:
        assert k in SINGLE_SCENARIOS, f"unknown scenario {k}"

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    sha = git_sha()
    run_dir = Path(args.outdir) / run_id
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    results_path = run_dir / "results.jsonl"
    print(f"[harness] run_id={run_id} git_sha={sha}  -> {results_path}")

    rclpy.init()
    cap = Capture()

    # Trial index outermost, arm innermost: this used to be justified as
    # pairing both arms' trials at the same (scenario, seed) for McNemar's
    # test, but that pairing was never real (see the module docstring / R4
    # in ablation-experiment-spec.md) -- the ordering is kept anyway because
    # it is still a reasonable way to interleave arms, not because anything
    # downstream depends on it.
    total = len(arms) * len(scenarios) * args.trials
    done = 0
    try:
        for seed in range(args.trials):
            for scenario in scenarios:
                sc = SINGLE_SCENARIOS[scenario]
                for arm in arms:
                    done += 1
                    tag = f"{arm}__{scenario}__seed{seed}"
                    print(f"[harness] ({done}/{total}) {tag} launching…", flush=True)

                    plog = run_dir / "logs" / f"{tag}.planner.log"
                    alog = run_dir / "logs" / f"{tag}.arbiter.log"
                    pproc, pf = launch_node(PLANNER_CMD, ARMS[arm], plog)
                    aproc, af = launch_node(ARBITER_CMD, ARMS[arm], alog)

                    policy_label = (
                        "deterministic"
                        if arm == "deterministic"
                        else ARMS[arm].get("LOCAL_MODEL", arm)
                    )

                    record = {
                        "run_id": run_id,
                        "git_sha": sha,
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                        "harness": "single",
                        "arm": arm,
                        "scenario": scenario,
                        # A bare trial index, kept under the name "seed" only
                        # for schema parity with fleet_harness.py's records
                        # (see this harness's own module docstring). Nothing
                        # in this harness reads it back; it does not seed
                        # anything and has no effect on the trial that
                        # follows. Do not read variance into it.
                        "seed": seed,
                        "seed_affects_execution": False,
                        "policy_label": policy_label,
                        "expected": sc["expected"],
                        "error": None,
                    }
                    try:
                        if not wait_for_agents(READY_AGENTS, LAUNCH_TIMEOUT_SEC):
                            record["decision"] = None
                            record["error"] = "agents did not report ready"
                        else:
                            time.sleep(POST_READY_WAIT_SEC)
                            record.update(cap.run_trial(sc["xml"], sc["failure"]))
                    finally:
                        kill_node(aproc, af)
                        kill_node(pproc, pf)
                        time.sleep(8.0)  # let DDS discovery forget the dead nodes

                    decision = record.get("decision")
                    record["correct"] = (
                        sc["scorer"](decision) if decision is not None else False
                    )
                    # Every seam's decision this trial, even though the arms
                    # are all-or-nothing, so post-hoc attribution costs
                    # nothing and needs no re-run. Only the two of the five
                    # reasoning seams this harness's own processes (the
                    # mission planner and the arbiter) touch are observable
                    # here -- route (seam 1) and interpret_note (seam 3) are
                    # triage_node's and note_node's, neither of which this
                    # harness launches.
                    record["seams"] = {
                        "replan": decision,
                        "viability_budget": record.get("viability_budget"),
                    }

                    with open(results_path, "a") as fh:
                        fh.write(json.dumps(record) + "\n")
                    print(
                        f"[harness]      -> {decision} "
                        f"(correct={record['correct']}, "
                        f"budget={record.get('viability_budget')}, "
                        f"{record.get('latency_sec','-')}s)",
                        flush=True,
                    )
    finally:
        cap.destroy_node()
        rclpy.shutdown()

    print(f"[harness] done. {results_path}")


if __name__ == "__main__":
    main()

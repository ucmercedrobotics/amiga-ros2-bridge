#!/usr/bin/env python3
"""
Fleet harness for the reasoning-ablation study -- the coordination seam.

`research_harness.py` ablates the single-robot seams (replan, viability
budget). This one ablates the seam that only exists once there is a fleet to
coordinate with: `interpret_anomaly`, question 2 of the five in
`ports/reasoning.py` -- re-delegate this task, take on new work, or give up
on it locally? Six scenarios (`registry.FLEET_SCENARIOS`), two arms.

For every (arm, scenario, trial) cell it:
  1. builds a FRESH three-robot `fleet_rig.Fleet` from the scenario's spec
     (clean coordinator state per trial, same as a fresh planner+arbiter
     process buys the single-robot harness independence)
  2. waits for the shedder's peer registry to actually populate from the
     other two robots' heartbeats -- a registry read before the first
     heartbeat lands would see zero peers regardless of the scenario, which
     is not what "no_peers" is supposed to mean
  3. builds the one `AnomalyContext` the scenario's `Task` and the fleet's
     live peer registry produce, via `CoordinatorSession.anomaly_context` --
     the same method the real coordinator calls under its own lock before
     handing an anomaly to an interpreter off it
  4. obtains one action for that context from whichever arm is under test:
     `CapabilityAwareInterpreter.interpret_anomaly`, a pure function, for
     `deterministic`; a live `triage_node` process reached through
     `TriageClient`, for the llm arms
  5. injects the action with `fleet.shed(task=..., detail=..., action=...)`,
     which runs the real contract-net state machine on it exactly as if an
     interpreter inside the coordinator had produced it
  6. waits for the fleet to quiesce, records the action taken, `fleet.
     outcome()`, and `correct` from the registry's own scorer, then tears
     the fleet down

Same `results.jsonl` schema as `research_harness.py` (see that file's and
`registry.py`'s docstrings for why: one committed pre-registration, one
analyzer, both harnesses). `harness` is `"fleet"`; the fleet-specific
addition is `fleet.outcome()` verbatim -- winner, auctions opened, per-
message-type channel counts, elapsed seconds -- logged as a secondary,
clearly-not-correctness metric: `fleet_wide` is expected to show its cost
here (an auction opened and a peer's wasted trip) even on a trial where
nothing about `correct` looks unusual until this dict is read.

Only `interpret_anomaly` is ablated here. `interpret_note` (seam 3) has no
deterministic arm worth measuring -- see `ports.reasoning.IgnoreNotes` and
`runtime.policy.interpret_note`, both `keep`/`KeepBid` by construction -- and
this harness does not attach a note to the shed task, so that seam never
fires in a way that would matter to a score.

`seed` still does real work here, unlike in `research_harness.py`: it is
passed straight through to `fleet_rig.Fleet(seed=...)`, where it drives
bidding backoff jitter, so it can change which peer wins an auction (the
*outcome*) even on a scenario where the deterministic arm's *decision*
never varies. That is why `correct` and `fleet.outcome()` are scored
separately (see the paragraph above), and it is also why `seed` is kept in
every record with the seed-as-unit-of-variation secondary metrics in
`analyze.py`'s fleet table. It is not, however, machinery for pairing
trials across arms any more -- R4 in `ablation-experiment-spec.md` retired
that idea along with the hypothesis test it was feeding -- so the one thing
it is still genuinely for is replay: rerun `--arms <arm> --scenarios <id>
--trials 1` at a specific seed to reproduce one odd trial exactly.

The CLI flag is `--trials` (repetitions per (arm, scenario) cell); `--seeds`
is kept as an accepted synonym.

Run INSIDE the container with ROS2 sourced (llm arms additionally need vLLM
serving or an OpenAI-compatible endpoint reachable; `deterministic` needs
neither):
  python3 scripts/scenarios/fleet_harness.py --arms deterministic --scenarios all --trials 20
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

_HERE = Path(__file__).resolve()
_REPO_ROOT = _HERE.parents[3]

# registry.py and research_harness.py (for ARMS) live next to this file.
sys.path.insert(0, str(_HERE.parent))
# fleet_rig.py and fakes.py live in amiga_ros2_coordinator/test/, and import
# `amiga_ros2_coordinator....` themselves, so both the coordinator package's
# own root and its test/ directory need to be on the path -- the same two
# entries fleet_rig.py's own module docstring assumes a caller has made.
sys.path.insert(0, str(_REPO_ROOT / "amiga_ros2_coordinator"))
sys.path.insert(0, str(_REPO_ROOT / "amiga_ros2_coordinator" / "test"))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from amiga_ros2_agents.runtime.status import STATUS_QOS  # noqa: E402

import fleet_rig  # noqa: E402
from registry import FLEET_SCENARIOS, git_sha  # noqa: E402
from research_harness import ARMS  # noqa: E402

from amiga_ros2_comms.codec import (  # noqa: E402
    CAPABILITY_BY_ELEMENT,
    ReasonCode,
    Target,
)
from amiga_ros2_coordinator.ports.reasoning import (
    CapabilityAwareInterpreter,
)  # noqa: E402
from amiga_ros2_coordinator.vocabulary.model import Task  # noqa: E402
from amiga_ros2_coordinator.vocabulary.schema import (  # noqa: E402
    AddTask,
    DropTask,
    ReDelegate,
)

PYBIN = sys.executable
TRIAGE_CMD = [PYBIN, "-m", "amiga_ros2_agents.coordination.triage_node"]

COMMON_ENV = {
    "ENV_FILE_PATH": os.environ.get("ENV_FILE_PATH", "/amiga-ros2-bridge/.env"),
}

#: Same shape as research_harness.READY_AGENTS: triage_node latches its
#: status on /agents/triage/status the moment its callbacks are wired.
TRIAGE_READY_TIMEOUT_SEC = 90
PEER_REGISTRY_TIMEOUT_SEC = 5.0
QUIESCE_TIMEOUT_SEC = 30.0
TRIAGE_CALL_TIMEOUT_SEC = 60.0


def _cap_mask(names) -> int:
    """XML element names (`registry.py`'s vocabulary) to the codec's mask.

    The one translation `registry.py` deliberately does not do itself -- see
    its module docstring -- because doing it here, in the file that already
    has to import `amiga_ros2_comms`/`amiga_ros2_coordinator` to drive a real
    `Fleet`, is what lets `registry.py` stay importable with neither.
    """
    mask = 0
    for name in names:
        capability = CAPABILITY_BY_ELEMENT.get(name)
        if capability is None:
            raise ValueError(
                f"registry capability {name!r} is not in the mission schema"
            )
        mask |= 1 << int(capability)
    return mask


def _build_fleet(scenario: dict, seed: int) -> "fleet_rig.Fleet":
    robots = [
        {
            "node_id": spec["node_id"],
            "eta_sec": spec.get("eta_sec", 60.0),
            "capabilities": [
                CAPABILITY_BY_ELEMENT[name] for name in spec["capabilities"]
            ],
            "battery": spec.get("battery", 88),
            "idle": spec.get("idle", True),
        }
        for spec in scenario["robots"]
    ]
    return fleet_rig.Fleet(robots=robots, shedder_id=scenario["shedder_id"], seed=seed)


def _build_task(scenario: dict) -> Task:
    return Task(
        task_id=fleet_rig.TASK_ID,
        required_capabilities=_cap_mask(scenario["task_capabilities"]),
        location=Target.tree(fleet_rig.TREE),
        priority=100,
    )


def _normalize_action(action) -> str:
    """A `ReDelegate`/`AddTask`/`DropTask` instance to one of `FLEET_ACTIONS`.

    The bridge between what an interpreter returns and what `registry.py`'s
    scorers compare against -- `registry.py` names ground truth as plain
    strings so it can stay off this file's imports (see its docstring), and
    this is the one place that string and this dataclass are put next to
    each other.
    """
    if isinstance(action, ReDelegate):
        return "re_delegate"
    if isinstance(action, AddTask):
        return "add_task"
    if isinstance(action, DropTask):
        return f"drop_task({action.disposition.value})"
    raise TypeError(
        f"interpret_anomaly returned {type(action).__name__}, not an ActionSchema"
    )


def _wait_for_peer_registry(fleet: "fleet_rig.Fleet", scenario: dict) -> bool:
    shedder = fleet.robots[scenario["shedder_id"]]
    expected_peers = len(scenario["robots"]) - 1
    return fleet_rig.wait_until(
        lambda: len(shedder.session.registry.peers) >= expected_peers,
        timeout=PEER_REGISTRY_TIMEOUT_SEC,
    )


def _deterministic_action(context):
    """Seam 2, ablated: `CapabilityAwareInterpreter` is a pure function.

    No subprocess, no service call -- this is the whole point of the
    deterministic arm existing as a plain Python class in `ports/reasoning.py`
    rather than a node: the ablation's baseline costs nothing to run and
    reaches no endpoint, which is exactly what the harness's own smoke test
    verifies about it.
    """
    return CapabilityAwareInterpreter().interpret_anomaly(context)


def _wait_for_agent(name: str, timeout: float) -> bool:
    """True once `name` has latched its startup status snapshot.

    Same readiness signal `research_harness.wait_for_agents` uses for the
    mission planner and the arbiter, narrowed to one agent: `triage_node`
    publishes on `/agents/triage/status` (TRANSIENT_LOCAL) the moment its
    subscriptions and its `interpret_anomaly` service are wired, so seeing
    one message means the process is ready to be called, not merely running.
    """
    probe = Node("fleet_harness_probe")
    seen = {"ready": False}
    probe.create_subscription(
        String,
        f"/agents/{name}/status",
        lambda _msg: seen.__setitem__("ready", True),
        STATUS_QOS,
    )
    deadline = time.time() + timeout
    while not seen["ready"] and time.time() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.5)
    probe.destroy_node()
    return seen["ready"]


def _launch_triage(env_extra: dict, log_path: Path):
    env = os.environ.copy()
    env.update(COMMON_ENV)
    env.update(env_extra)
    logf = open(log_path, "w")
    proc = subprocess.Popen(
        TRIAGE_CMD,
        env=env,
        stdout=logf,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return proc, logf


def _kill(proc, logf):
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


def _llm_action(context, scenario: dict):
    """Seam 2, answered by a live triage agent -- unlike the deterministic
    arm, this is a real subprocess and a real RPC, and it is the one branch
    of this whole harness the scope of this change does not smoke-test: doing
    so would call a model endpoint, which the task building this harness was
    explicitly told not to do.

    Mirrors production as closely as a one-shot harness reasonably can.
    `triage_node` latches evidence off its own `/bt/status_change`
    subscription (see that node's `_on_fault`), not off the
    `interpret_anomaly` request -- `TriageClient._request` leaves
    `fault_json`/`log_context`/`world_state` empty on the wire on purpose,
    because in production the same node that serves the RPC already saw the
    fault first. So this publishes one `/bt/status_change` FAILURE event
    carrying the scenario's free-text `reason` before making the call, which
    is the only path by which that text can reach the model at all: neither
    `AnomalyContext` nor the RPC request itself carries it (see
    `registry.py`'s docstring on why `reason` is deliberately the only place
    a fleet scenario's distinguishing evidence lives).

    One side effect worth naming rather than hiding: publishing that FAILURE
    event also starts `triage_node`'s *routing* seam (`route_fault`, seam 1),
    which spends a second model call this study is not measuring. That call's
    answer is never read here; it costs latency, not correctness.
    """
    from amiga_ros2_coordinator.adapters.triage_client import TriageClient

    node = Node("fleet_harness_triage_client")
    fault_pub = node.create_publisher(String, "/bt/status_change", 10)
    try:
        fault = {
            "node": "Sample_Leaves_Tree_60",
            "status": "FAILURE",
            "reason": scenario["reason"],
            "timestamp_ms": int(time.time() * 1000),
        }
        msg = String()
        msg.data = json.dumps(fault)
        fault_pub.publish(msg)
        # Give triage_node's own subscription callback a moment to latch the
        # evidence before the RPC that reads it arrives.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        client = TriageClient(node, timeout_sec=TRIAGE_CALL_TIMEOUT_SEC)
        deadline = time.time() + TRIAGE_CALL_TIMEOUT_SEC
        action = None
        last_exc = None
        while time.time() < deadline:
            try:
                action = client.interpret_anomaly(context)
                break
            except (
                Exception
            ) as exc:  # noqa: BLE001 - surfaced in the record's error field
                last_exc = exc
                rclpy.spin_once(node, timeout_sec=0.2)
        if action is None:
            raise last_exc or RuntimeError("triage agent never answered")
        return action
    finally:
        node.destroy_node()


def _run_trial(scenario_id: str, scenario: dict, arm: str, seed: int, run_dir: Path):
    tag = f"{arm}__{scenario_id}__seed{seed}"
    t0 = time.time()
    error = None
    decision_obj = None

    fleet = _build_fleet(scenario, seed)
    triage_proc = triage_log = None
    try:
        if not _wait_for_peer_registry(fleet, scenario):
            error = "peer registry never reached the expected peer count"
        else:
            task = _build_task(scenario)
            shedder = fleet.robots[scenario["shedder_id"]]
            context = shedder.session.anomaly_context(
                task, detail=scenario["reason"], reason_code=ReasonCode.UNSPECIFIED
            )

            if arm == "deterministic":
                decision_obj = _deterministic_action(context)
            else:
                triage_log = run_dir / "logs" / f"{tag}.triage.log"
                triage_proc, triage_logf = _launch_triage(ARMS[arm], triage_log)
                if not _wait_for_agent("triage", TRIAGE_READY_TIMEOUT_SEC):
                    error = "triage agent did not report ready"
                else:
                    decision_obj = _llm_action(context, scenario)

            if decision_obj is not None:
                fleet.shed(task=task, detail=scenario["reason"], action=decision_obj)
                fleet_rig.wait_until(fleet.quiesced, timeout=QUIESCE_TIMEOUT_SEC)
    except (
        Exception
    ) as exc:  # noqa: BLE001 - recorded, not raised: one bad trial must not kill a sweep
        error = f"{type(exc).__name__}: {exc}"
    finally:
        outcome = fleet.outcome()
        fleet.close()
        if triage_proc is not None:
            _kill(triage_proc, triage_logf)

    latency = round(time.time() - t0, 2)
    decision = _normalize_action(decision_obj) if decision_obj is not None else None
    expected = scenario["expected"]
    correct = scenario["scorer"](decision) if decision is not None else False
    policy_label = (
        "deterministic" if arm == "deterministic" else ARMS[arm].get("LOCAL_MODEL", arm)
    )

    return {
        "run_id": run_dir.name,
        "git_sha": git_sha(),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "harness": "fleet",
        "arm": arm,
        "scenario": scenario_id,
        "seed": seed,
        "policy_label": policy_label,
        "decision": decision,
        "expected": expected,
        "correct": correct,
        "seams": {"interpret_anomaly": decision},
        "latency_sec": latency,
        "error": error,
        "outcome": outcome,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--arms", default="deterministic", help="comma list of ARMS keys, or 'all'"
    )
    ap.add_argument(
        "--scenarios",
        default="all",
        help="comma list of FLEET_SCENARIOS keys, or 'all'",
    )
    ap.add_argument(
        "--trials",
        "--seeds",
        dest="trials",
        type=int,
        default=5,
        help="repetitions per (arm, scenario); the seed passed to "
        "Fleet(seed=...) for each trial is this repetition's index, kept in "
        "every record as a replay handle and as the unit of variation for "
        "the fleet's outcome-level secondary metrics (see the module "
        "docstring) -- '--seeds' is accepted as a synonym for this flag.",
    )
    ap.add_argument("--outdir", default="runs")
    args = ap.parse_args()

    arms = list(ARMS) if args.arms == "all" else args.arms.split(",")
    scenarios = (
        list(FLEET_SCENARIOS) if args.scenarios == "all" else args.scenarios.split(",")
    )
    for k in arms:
        assert k in ARMS, f"unknown arm {k}"
    for k in scenarios:
        assert k in FLEET_SCENARIOS, f"unknown scenario {k}"

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.outdir) / run_id
    (run_dir / "logs").mkdir(parents=True, exist_ok=True)
    results_path = run_dir / "results.jsonl"
    print(f"[fleet_harness] run_id={run_id} git_sha={git_sha()}  -> {results_path}")

    rclpy.init()
    total = len(arms) * len(scenarios) * args.trials
    done = 0
    try:
        for seed in range(args.trials):
            for scenario_id in scenarios:
                scenario = FLEET_SCENARIOS[scenario_id]
                for arm in arms:
                    done += 1
                    print(
                        f"[fleet_harness] ({done}/{total}) "
                        f"{arm}__{scenario_id}__seed{seed} running…",
                        flush=True,
                    )
                    record = _run_trial(scenario_id, scenario, arm, seed, run_dir)
                    with open(results_path, "a") as fh:
                        fh.write(json.dumps(record) + "\n")
                    print(
                        f"[fleet_harness]      -> {record['decision']} "
                        f"(correct={record['correct']}, {record['latency_sec']}s)"
                        + (f"  error={record['error']}" if record["error"] else ""),
                        flush=True,
                    )
    finally:
        rclpy.shutdown()

    print(f"[fleet_harness] done. {results_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""The pre-registration: scenarios, ground truth, and how to score a trial.

This file exists to answer one objection before it can be raised: "you graded
the runs after you saw them." Everything a scorer needs -- the scenario setup,
the pre-registered correct action, and the function that turns a decision into
a bool -- is committed here, in one module, before either harness produces a
single `results.jsonl` line. `research_harness.py` and `fleet_harness.py` read
their scenarios from here rather than defining them inline, and `analyze.py`
reads ground truth from here rather than re-deriving it from the records
themselves, so scoring cannot drift toward a result after the fact. Every
record either harness writes carries `git_sha()` of the commit this file was
at, so a reader can always check the registry a given run was scored against.

Two harnesses, two dictionaries, one file:

    SINGLE_SCENARIOS   the three single-robot scenarios `research_harness.py`
                       used to define inline (`missing_tree`, `stuck_robot`,
                       `sick_tree`), moved here unchanged -- same XML, same
                       failure event, same `expected` string, same success
                       criterion `analyze.py::_success` used to grade them.

    FLEET_SCENARIOS    the six fleet scenarios from the ablation spec, each a
                       fleet composition (who is out there, idle or not,
                       capable or not) plus a free-text fault and the one
                       action a policy reading only structured state and a
                       policy reading the fault text as well are each supposed
                       to produce.

Deliberately pure data and pure functions, no ROS and no import of
`amiga_ros2_comms` or `amiga_ros2_coordinator`. Both packages are themselves
free of rclpy (`ports/reasoning.py` says so of itself, and this module holds
itself to the same bar for the same reason), but they are ROS-ament packages
addressed through the workspace layout, not pip-installed, and resolving that
import correctly depends on which of several possible working directories a
caller happens to run from -- exactly the kind of incidental failure a
statistics/registry unit test should never be at the mercy of. So capabilities
here are named the way the wire already names them -- XML element strings,
`"MoveToTreeID"`, `"SampleLeaf"` -- the same vocabulary
`triage_client._peers_json` and `runtime/policy.py` already use, and it is
`fleet_harness.py`'s job, not this module's, to turn those strings into the
real `Capability` mask a `Task` or a `Fleet` robot spec needs. That split is
what lets this file, and the tests pinning it, run on bare Python with no
container: a registry that cannot be read without ROS running is a
pre-registration nobody can audit without infrastructure they may not have.
"""

import os
import subprocess

# ==========================================================================
# git_sha — the pre-registration's own fingerprint
# ==========================================================================


def git_sha() -> str:
    """The commit HEAD is at, from wherever this file lives on disk.

    Every result record is stamped with this, not with a version number
    someone has to remember to bump: the whole point of pre-registering
    ground truth in a committed file is that a reader can check out the exact
    commit a run was scored against and see the same `expected` and the same
    `scorer` the run saw. Falls back to `"unknown"` rather than raising --
    a harness mid-trial should not lose an otherwise-good record because the
    working tree was a source tarball with no `.git` directory, though that
    is exactly the situation `git_sha` exists to make visible rather than
    silently paper over.
    """
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


# ==========================================================================
# Single-robot scenarios — mission_planner_node + arbiter_node
# ==========================================================================
#
# Moved out of research_harness.py unchanged: same mission XML, same vague
# failure event (observation only, no menu of actions, so the model must
# reason), same pre-registered `expected` hypothesis, and the same
# deterministic success criterion that used to live in analyze.py::_success.
# The judgement layer for sick_tree stays a human/LLM-judge rubric pass over
# grading_sheet.csv; the closed-form part a machine can grade -- "did it
# accept or abort, and was that the pre-registered right answer" -- is what
# each `scorer` below checks.


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


#: The closed set of `decision` values either single-robot process actually
#: emits (see `research_harness.Capture.run_trial`). Not every scenario's
#: `scorer` accepts every member -- `stuck_robot` only ever wants `"abort"` --
#: but every member is a value some trial could legitimately produce, and
#: nothing outside this set is a decision this harness knows how to score.
SINGLE_DECISIONS = frozenset(
    {"accept", "abort", "candidate_only", "gave_up", "timeout", "launch_failed"}
)

SINGLE_SCENARIOS = {
    "missing_tree": {
        "xml": _row_mission(
            "sample leaves from every tree in the row, trees 1 through 10",
            range(1, 11),
        ),
        "failure": {
            "node": "Visit_Tree_5",
            "status": "FAILURE",
            "reason": "no tree at the mapped location for tree 5; the spot is bare soil.",
        },
        "expected": "continue remaining trees; do NOT abort whole mission",
        # correct = replans around the gap and keeps going
        "scorer": lambda decision: decision == "accept",
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
        # correct = recognizes futility
        "scorer": lambda decision: decision == "abort",
    },
    "sick_tree": {
        "xml": _row_mission(
            "sample leaves from trees 1 through 10 in the current row",
            range(1, 11),
        ),
        "failure": {
            "node": "Visit_Tree_4",
            "status": "DETECTION",
            "reason": "a tree in the adjacent row (id 54) shows heavy leaf necrosis and "
            "possible blight; it is not in the current mission.",
        },
        "expected": "characterize decision: ignore / append / re-prioritize",
        # any coherent action is scored correct here; the judgement itself
        # (ignore / append / re-prioritize) is graded by rubric, off
        # grading_sheet.csv, not by this closed-form check.
        "scorer": lambda decision: decision in ("accept", "abort"),
    },
}


# ==========================================================================
# Fleet scenarios — CapabilityAwareInterpreter vs a live triage agent
# ==========================================================================
#
# Six scenarios, three-robot fleets throughout: one shedder (node_id 1) and
# two peers (node_id 2, 3). The first three vary the *structured* facts a
# deterministic policy is allowed to read -- who is idle, who is capable --
# and the baseline is right on all three, because structured state is
# sufficient to answer them. The last three hold the fleet composition
# *identical* to `peer_capable` -- two idle, capable peers, the easiest case
# there is -- and vary only the free-text fault `reason`. That is the whole
# ablation: the baseline cannot read `reason`, so on these three it either
# takes the same action it took for peer_capable (wrong, for transient and
# fleet_wide) or cannot express the correct one at all (new_work, where the
# right action -- add_task -- names work no typed field carries).
#
# CRITICAL, and mechanically checked in test_registry.py: transient,
# fleet_wide and new_work carry their distinguishing information *only* in
# `reason`. No robot's idle flag, capability list, or the task's required
# capabilities may differ between one of these three and `peer_capable` --
# doing that would hand the deterministic policy a typed signal the study
# says does not exist, and quietly answer the question the study is trying to
# ask.

#: XML element names, the same vocabulary the wire and `runtime/policy.py`
#: already use for a capability. Not a `Capability` enum import -- see the
#: module docstring for why this file stays off that dependency.
SAMPLING_CAPS = ["MoveToTreeID", "SampleLeaf"]

#: Can drive to a tree, cannot do anything once there. Peers with only this
#: are idle but not a candidate for a sampling task -- the `no_capability`
#: scenario's whole point.
NAV_ONLY_CAPS = ["MoveToTreeID"]

#: Two idle, capable peers. Reused verbatim (not merely equal-by-value) by
#: `peer_capable`, `transient`, `fleet_wide` and `new_work`, so a future edit
#: to one cannot silently drift the fleet composition out from under the
#: other three -- the mechanical check in test_registry.py compares against
#: this same object.
_SHEDDER = {
    "node_id": 1,
    "eta_sec": 60.0,
    "capabilities": SAMPLING_CAPS,
    "battery": 82,
    "idle": True,
}
_TWO_IDLE_CAPABLE_PEERS = [
    dict(_SHEDDER),
    {
        "node_id": 2,
        "eta_sec": 90.0,
        "capabilities": SAMPLING_CAPS,
        "battery": 91,
        "idle": True,
    },
    {
        "node_id": 3,
        "eta_sec": 45.0,
        "capabilities": SAMPLING_CAPS,
        "battery": 76,
        "idle": True,
    },
]

#: The fault this robot's own sampler failing produces, reused by the three
#: scenarios (`peer_capable`, `no_peers`, `no_capability`) whose baseline
#: answer is correct -- their structured facts differ, their fault does not.
_SAMPLER_DEAD_REASON = (
    "the depth camera feeding the sampler cut out mid-approach to tree 60; "
    "SampleLeaf has no verified tree center to act on and three restarts of "
    "the camera driver have not brought it back."
)

#: Everything `interpret_anomaly` may normalize to, once `fleet_harness.py`
#: turns a `ReDelegate`/`AddTask`/`DropTask` instance back into the string an
#: `expected` field can name. `drop_task(hold)` and `drop_task(drop)` are
#: distinct members on purpose: `LocalDisposition` distinguishes "try again
#: once the fleet looks different" from "this work is not happening", and a
#: scorer that collapsed the two could not tell `fleet_wide` (never worth
#: revisiting) from `no_peers` (worth revisiting once a peer frees up) apart.
FLEET_ACTIONS = frozenset(
    {"re_delegate", "drop_task(hold)", "drop_task(drop)", "add_task"}
)


def _action_scorer(expected: str):
    """A scorer that is correct exactly when `decision == expected`.

    Every fleet scenario's ground truth is one action out of the closed
    schema, so every fleet scorer is the same one-line comparison; this is
    the factory, not six copies of the lambda, so the comparison itself
    cannot drift scenario to scenario.
    """

    def score(decision: str) -> bool:
        return decision == expected

    return score


FLEET_SCENARIOS = {
    "peer_capable": {
        "robots": _TWO_IDLE_CAPABLE_PEERS,
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": _SAMPLER_DEAD_REASON,
        "expected": "re_delegate",
        "scorer": _action_scorer("re_delegate"),
    },
    "no_peers": {
        "robots": [
            dict(_SHEDDER),
            {
                "node_id": 2,
                "eta_sec": 90.0,
                "capabilities": SAMPLING_CAPS,
                "battery": 91,
                "idle": False,
            },
            {
                "node_id": 3,
                "eta_sec": 45.0,
                "capabilities": SAMPLING_CAPS,
                "battery": 76,
                "idle": False,
            },
        ],
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": _SAMPLER_DEAD_REASON,
        "expected": "drop_task(hold)",
        "scorer": _action_scorer("drop_task(hold)"),
    },
    "no_capability": {
        "robots": [
            dict(_SHEDDER),
            {
                "node_id": 2,
                "eta_sec": 90.0,
                "capabilities": NAV_ONLY_CAPS,
                "battery": 91,
                "idle": True,
            },
            {
                "node_id": 3,
                "eta_sec": 45.0,
                "capabilities": NAV_ONLY_CAPS,
                "battery": 76,
                "idle": True,
            },
        ],
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": _SAMPLER_DEAD_REASON,
        "expected": "drop_task(hold)",
        "scorer": _action_scorer("drop_task(hold)"),
    },
    # -- the three that differ from peer_capable only in `reason` --------
    "transient": {
        "robots": _TWO_IDLE_CAPABLE_PEERS,
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": (
            "a fallen branch blocks the aisle a few meters short of tree 60; "
            "ground crew radioed that it will be cleared within the hour and "
            "the tree itself is undamaged and still unsampled. Nothing about "
            "this robot or this task has changed -- the aisle will be open "
            "again shortly."
        ),
        # correct = hold; the work is still ours, and re-delegating spends an
        # auction on a peer who will hit the same blocked aisle.
        "expected": "drop_task(hold)",
        "scorer": _action_scorer("drop_task(hold)"),
    },
    "fleet_wide": {
        "robots": _TWO_IDLE_CAPABLE_PEERS,
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": (
            "the robot reached tree 60 and the canopy camera confirms it is "
            "bare -- no leaves, no fruit, nothing here for SampleLeaf to act "
            "on. This is a mapping error, not a local fault: whichever robot "
            "visits tree 60 next will see the same bare tree."
        ),
        # correct = drop, permanently; a capable idle peer exists, so the
        # baseline re-delegates and burns an auction plus a peer's trip to
        # relearn the same bare tree.
        "expected": "drop_task(drop)",
        "scorer": _action_scorer("drop_task(drop)"),
    },
    "new_work": {
        "robots": _TWO_IDLE_CAPABLE_PEERS,
        "shedder_id": 1,
        "task_capabilities": SAMPLING_CAPS,
        "reason": (
            "while stopped at tree 60 the front camera caught tree 84, in "
            "the next row over, showing heavy leaf necrosis consistent with "
            "blight. Tree 84 is not part of any current mission and nobody "
            "has been told about it."
        ),
        # correct = add_task, naming tree 84. No typed field in
        # AnomalyContext carries "there is a diseased tree nobody planned
        # for" -- that is the one action CapabilityAwareInterpreter can never
        # return, by construction, and this scenario is why.
        "expected": "add_task",
        "scorer": _action_scorer("add_task"),
    },
}

#: The three scenarios required, by the spec, to differ from `peer_capable`
#: only in `reason`. `test_registry.py` walks exactly this tuple rather than
#: guessing from `FLEET_SCENARIOS.keys()`, so adding a seventh scenario later
#: does not silently start being checked against an invariant it may not
#: share.
TEXT_ONLY_VARIANTS = ("transient", "fleet_wide", "new_work")

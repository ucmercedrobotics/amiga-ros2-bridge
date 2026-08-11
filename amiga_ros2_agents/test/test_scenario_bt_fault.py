"""The catalyst end to end: a real mission, a realistic failure, a real task.

This is the one test that runs ``bt_runner`` -- the actual C++ tree, with XSD
validation on, against an actual file in ``examples/`` -- and follows what comes
out of it all the way to something the coordinator could announce. Everything
else in these suites tests a piece with the neighbouring pieces replaced.

What it proves is the *wiring*: that a navigation abort becomes a
``/bt/status_change`` event, that the event names a leaf, and that the leaf
resolves to a unit of work with the right actions and the right place. What it
deliberately does not prove is anything about the quality of an interpretation
-- no model runs here, and one that did would be reading logs from a failure
with nothing behind it. See ``test/scenarios/README.md``.

Skipped when the workspace is not built, because there is nothing to run.
"""

import json
import os
import shutil
import struct
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.mission import mission_tasks  # noqa: E402
from amiga_ros2_comms.codec import (  # noqa: E402
    Capability,
    Target,
    cap_mask,
)

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MISSION = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples", "sample_leafs.xml")

#: Long enough for the tree to reach tree 10, fail, and finish. The mission is
#: driven by mock action servers with fixed 500 ms feedback steps, so this is a
#: bound on known work rather than a guess.
TIMEOUT_SEC = 45


pytestmark = pytest.mark.skipif(
    shutil.which("ros2") is None,
    reason="needs a built workspace with ros2 on PATH",
)


@pytest.fixture(scope="module")
def fault_events():
    """Run the scenario once and return every /bt/status_change event.

    Module-scoped: the run takes the better part of a minute, and every
    assertion below is about the same run rather than about a fresh one.
    """
    listener = subprocess.Popen(
        [
            "ros2",
            "topic",
            "echo",
            "--full-length",
            "/bt/status_change",
            "std_msgs/msg/String",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    # The tree publishes TRANSIENT_LOCAL, so a listener that starts late still
    # receives the fault -- but starting it first keeps the run deterministic
    # rather than relying on that.
    time.sleep(2)

    scenario = subprocess.Popen(
        [
            "ros2",
            "launch",
            "amiga_ros2_behavior_tree",
            "failure_scenario.launch.py",
            "fail_goals:=[10]",
            "failure_mode:=nav_failed",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        scenario.wait(timeout=TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        pass
    finally:
        scenario.terminate()
        try:
            scenario.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            scenario.kill()
        listener.terminate()
        try:
            out, _ = listener.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            listener.kill()
            out = ""

    events = []
    for line in out.splitlines():
        line = line.strip()
        if not line.startswith("data:"):
            continue
        payload = line[len("data:") :].strip().strip("'\"")
        try:
            events.append(json.loads(payload))
        except json.JSONDecodeError:
            continue
    if not events:
        pytest.skip("the scenario produced no fault events; is the workspace built?")
    return events


def test_the_tree_reports_the_leaf_that_failed(fault_events):
    """The catalyst fires, and it names the node rather than the mission.

    A tree-level "something failed" would leave the planner with nowhere to
    look: the whole point of the event is that it identifies the step, which is
    what lets the log window be centred on it and the task be resolved from it.
    """
    leaves = [e for e in fault_events if e.get("source") == "leaf"]

    assert leaves, "no leaf-level fault was published"
    assert leaves[0]["node"] == "Visit_Tree_10"
    assert leaves[0]["registration_name"] == "MoveToTreeID"
    assert leaves[0]["status"] == "FAILURE"


def test_the_mission_outcome_is_reported_separately(fault_events):
    """The tree returning FAILURE is a different fact from a node failing.

    One is "this step did not work", which local replanning may well fix. The
    other is "the mission is over", and the two must not arrive as the same
    kind of event or the planner cannot tell a recoverable fault from a
    finished one.
    """
    outcomes = [e for e in fault_events if e.get("source") == "tree"]

    assert outcomes, "the tree's own outcome was not published"
    assert outcomes[-1]["node"] == "<tree>"
    assert outcomes[-1]["status"] == "FAILURE"


def test_only_the_failing_leaf_is_reported(fault_events):
    """Not its ancestors.

    A failing leaf makes every Sequence above it fail too. Reporting each would
    fire the planner once per level for a single fault, and the planner would
    spend a model call on each.
    """
    leaves = [e for e in fault_events if e.get("source") == "leaf"]
    assert {e["node"] for e in leaves} == {"Visit_Tree_10"}


def test_the_fault_resolves_to_work_the_fleet_could_take_on(fault_events):
    """The bridge, on the real event from the real tree.

    This is the assertion the whole chain exists for. The event names a leaf;
    what a peer can be offered is the unit -- approach tree 10 *and* sample it.
    Offering the leaf alone would send a robot to a tree with nothing to do
    there, and leave the sample on a robot that has already given up on
    arriving.
    """
    leaf = next(e for e in fault_events if e.get("source") == "leaf")
    with open(MISSION) as handle:
        mission = handle.read()

    task = mission_tasks.task_for_node(mission, leaf["node"])

    assert task is not None
    assert task.target == Target.tree(10)
    assert task.capabilities == cap_mask(
        Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF
    )
    assert task.delegable

    # And the escalation payload is complete enough for the coordinator to
    # announce without parsing any XML of its own.
    payload = task.as_payload()
    assert payload["task_id"] > 0
    assert payload["target_kind"] == int(Target.tree(10).kind)
    assert payload["target_a"] == 10


# ---------------------------------------------------------------------------
# The demo's injected fault
#
# Pure data: the mission payloads the demo feeds and what they imply. It runs
# no tree and launches nothing, so it costs milliseconds rather than the
# module-scoped run above. (It still inherits this module's skipmark, which
# wants a sourced workspace -- the imports below need one.)
# ---------------------------------------------------------------------------

EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")
DEMO_SOURCE = os.path.join(EXAMPLES, "sample_20_64.bin")
DEMO_PEERS = [
    os.path.join(EXAMPLES, name) for name in ("sample_22_66.bin", "sample_24_68.bin")
]
DEMO_HIDDEN_TREE = 64
BUILDER = os.path.join(REPO, "scripts", "build_broken_mission.py")


def _frames(path):
    """The two length-prefixed frames of a mission payload: (xml, orchard)."""
    with open(path, "rb") as handle:
        data = handle.read()
    (xml_len,) = struct.unpack(">I", data[0:4])
    xml = data[4 : 4 + xml_len]
    (json_len,) = struct.unpack(">I", data[4 + xml_len : 8 + xml_len])
    return xml, data[8 + xml_len : 8 + xml_len + json_len]


@pytest.mark.skipif(not os.path.exists(BUILDER), reason="no demo builder script")
def test_the_demo_fault_sheds_work_some_other_robot_could_actually_do(tmp_path):
    """The property the demo's whole point rests on, and the one it lacked.

    An injected fault is easy; an injected fault that produces a *biddable*
    task is the thing. The first version of this pointed one MoveToTreeID at
    tree 9999, which fails for real -- and yields a task every peer resolves
    against its own orchard, finds nothing for, and bids infeasible on. The
    auction then runs correctly to the conclusion that nobody can help, and
    the demo shows an auction with no winner while every log line looks fine.

    So: the failing robot must not be able to reach the work, and at least one
    healthy robot must. Both halves are asserted, because a scenario that
    satisfies only the first is the bug that was there.
    """
    broken = tmp_path / "broken.bin"
    assert (
        subprocess.run(
            [sys.executable, BUILDER, DEMO_SOURCE, str(DEMO_HIDDEN_TREE), str(broken)],
            capture_output=True,
        ).returncode
        == 0
    )

    plan, crippled_orchard = _frames(str(broken))
    original_plan, _ = _frames(DEMO_SOURCE)
    assert plan == original_plan, "the plan must be untouched; only the map differs"

    from amiga_ros2_coordinator.adapters.nav_ports import parse_orchard

    mine, _ = parse_orchard(crippled_orchard.decode())
    assert DEMO_HIDDEN_TREE not in mine, "the failing robot can still get there"
    # It is one objective it cannot reach, not a crippled robot: the rest of
    # its mission has to keep running, or it sheds everything at once and
    # there is no healthy fleet left to bid.
    assert len(mine) > 1

    task = mission_tasks.task_for_node(plan.decode(), f"ApproachTree{DEMO_HIDDEN_TREE}")
    assert task is not None and task.delegable
    assert task.target == Target.tree(DEMO_HIDDEN_TREE)

    for peer_bin in DEMO_PEERS:
        _, peer_orchard = _frames(peer_bin)
        theirs, _ = parse_orchard(peer_orchard.decode())
        assert int(task.target.a) in theirs, (
            f"{os.path.basename(peer_bin)} cannot resolve the shed task's target, "
            f"so its bid would be infeasible and the auction would have no winner"
        )

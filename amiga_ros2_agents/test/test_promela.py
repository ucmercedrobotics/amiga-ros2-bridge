"""Mission XML -> Promela, against the real example missions.

Like ``test_mission_tasks``, these run on the files in
``amiga_ros2_behavior_tree/examples/`` rather than on fixtures shaped to pass.
A model built from a mission nobody flies proves nothing about the missions the
robots actually carry.

The claims worth pinning here are the two that make verification non-vacuous:
propositions are named from the mission's subject matter and not from the
position of an action in the tree, and a plan defines exactly the propositions
it establishes -- no more, so a formula asking for something the plan skips is
caught, and no fewer, so a plan cannot quietly satisfy a formula by declaring
everything.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.verification import promela  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")

#: Bring-up rigs, not missions: they drive one action server directly and do not
#: describe anything a robot is meant to achieve. ``test_sample_leaf.xml`` is a
#: bare ``SampleLeaf``, which the arbiter's ``_check_no_orphan_sample`` refuses
#: for the same reason the emitter does -- see ``test_bringup_rigs_are_refused``,
#: which pins that rather than letting the exclusion hide it.
NOT_MISSIONS = {"test_sample_leaf.xml"}

EXAMPLE_FILES = sorted(
    f for f in os.listdir(EXAMPLES) if f.endswith(".xml") and f not in NOT_MISSIONS
)


def example(name: str) -> str:
    with open(os.path.join(EXAMPLES, name)) as handle:
        return handle.read()


def plan(*body: str, mission: str = "test mission") -> str:
    """A minimal conforming plan wrapped around ``body``."""
    return (
        '<root BTCPP_format="4" schema_location="schemas/amiga_btcpp.xsd">'
        f"<Mission>{mission}</Mission>"
        '<BehaviorTree ID="T"><Sequence>' + "".join(body) + "</Sequence></BehaviorTree>"
        "</root>"
    )


def visit(tree_id, approach="true") -> str:
    return (
        f'<MoveToTreeID name="V_{tree_id}" action_name="follow_tree_id_waypoint" '
        f'id="{tree_id}" approach_tree="{approach}"/>'
    )


SAMPLE = '<SampleLeaf name="S" action_name="segment_leaves"/>'


# ---------------------------------------------------------------------------
# The action vocabulary comes from the schema
# ---------------------------------------------------------------------------


def test_action_pool_read_from_xsd():
    """The vocabulary is derived, so it cannot drift from the schema.

    A hand-maintained copy is what leaves a compiler silently emitting an empty
    model for a tree of perfectly valid actions -- which then verifies against
    anything.
    """
    pool = promela.action_pool(SCHEMA)
    assert "MoveToTreeID" in pool
    assert "SampleLeaf" in pool
    assert "DetectObject" not in pool  # commented out in the XSD


# ---------------------------------------------------------------------------
# Every real mission compiles
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", EXAMPLE_FILES)
def test_every_example_compiles(name):
    model = promela.compile_mission(example(name), SCHEMA)
    assert model.source.startswith("mtype = {")
    assert "init {" in model.source
    assert model.defined_aps, "a plan that defines no propositions verifies vacuously"


def test_sample_leafs_model():
    model = promela.compile_mission(example("sample_leafs.xml"), SCHEMA)
    assert model.defined_aps == {
        "at_tree_10",
        "at_tree_60",
        "sampled_tree_10",
        "sampled_tree_60",
    }
    assert model.visit_order == ["10", "60"]


# ---------------------------------------------------------------------------
# Naming is by content, not position
# ---------------------------------------------------------------------------


def test_propositions_survive_reordering():
    """Swap the two objectives and the propositions are the same set.

    This is the property that positional binding cannot have, and the reason
    this module does not use it: a replan reorders and edits the tree, and a
    proposition that means "the third action" changes meaning when it does.
    """
    forward = promela.compile_mission(
        plan(visit(10), SAMPLE, visit(60), SAMPLE), SCHEMA
    )
    reverse = promela.compile_mission(
        plan(visit(60), SAMPLE, visit(10), SAMPLE), SCHEMA
    )
    assert forward.defined_aps == reverse.defined_aps
    assert forward.visit_order == ["10", "60"]
    assert reverse.visit_order == ["60", "10"]


def test_dropping_a_task_drops_only_its_propositions():
    """Removing tree 60 leaves tree 10's propositions untouched."""
    full = promela.compile_mission(plan(visit(10), SAMPLE, visit(60), SAMPLE), SCHEMA)
    cut = promela.compile_mission(plan(visit(10), SAMPLE), SCHEMA)
    assert full.defined_aps - cut.defined_aps == {"at_tree_60", "sampled_tree_60"}
    assert {"at_tree_10", "sampled_tree_10"} <= cut.defined_aps


def test_sample_binds_to_the_preceding_move():
    """A SampleLeaf samples where the robot last went, not where it appears."""
    model = promela.compile_mission(plan(visit(7), SAMPLE), SCHEMA)
    assert "sampled_tree_7" in model.defined_aps


# ---------------------------------------------------------------------------
# Objectives vs transit
# ---------------------------------------------------------------------------


def test_transit_moves_define_no_position_proposition():
    """approach_tree="false" is a waypoint, not somewhere the mission wanted.

    Same definition of "objective" the arbiter's ``_objective_tree_ids`` uses;
    if these two ever disagree, a plan could satisfy its formula by driving past
    a tree it was supposed to stop at.
    """
    model = promela.compile_mission(
        plan(visit(10), SAMPLE, visit(2, approach="false"), visit(60), SAMPLE), SCHEMA
    )
    assert "at_tree_2" not in model.defined_aps
    assert model.visit_order == ["10", "60"]


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def test_orphan_sample_is_refused():
    """Better to refuse than to model a sample of nowhere."""
    with pytest.raises(promela.PromelaError, match="SampleLeaf"):
        promela.compile_mission(plan(SAMPLE), SCHEMA)


@pytest.mark.parametrize("name", sorted(NOT_MISSIONS))
def test_bringup_rigs_are_refused(name):
    """The excluded files are excluded because they are refused, not skipped.

    ``test_sample_leaf.xml`` samples wherever the robot happens to be standing,
    which names no objective and cannot be checked against a mission. The
    arbiter already refuses it via ``_check_no_orphan_sample``; the two agreeing
    is the point, since a plan the arbiter rejects should never reach a verifier
    that accepts it.
    """
    with pytest.raises(promela.PromelaError):
        promela.compile_mission(example(name), SCHEMA)


def test_empty_tree_is_refused():
    """A plan with no actions must not compile to a model that verifies."""
    with pytest.raises(promela.PromelaError, match="no actions"):
        promela.compile_mission(plan(), SCHEMA)


def test_malformed_xml_is_refused():
    with pytest.raises(promela.PromelaError, match="well-formed"):
        promela.compile_mission("<root><unclosed>", SCHEMA)


def test_unknown_element_is_refused():
    """An element outside the vocabulary is refused, never skipped.

    Skipping is how the upstream compiler ends up emitting an empty model for a
    tree it does not recognise.
    """
    with pytest.raises(promela.PromelaError, match="Bogus"):
        promela.compile_mission(plan("<Bogus/>"), SCHEMA)


# ---------------------------------------------------------------------------
# Coverage — the cross-check between the two agents
# ---------------------------------------------------------------------------


def test_coverage_accepts_a_formula_the_plan_covers():
    model = promela.compile_mission(plan(visit(10), SAMPLE), SCHEMA)
    ok, _ = promela.coverage_gap("<>(at_tree_10 && <>sampled_tree_10)", model)
    assert ok


def test_coverage_names_the_missing_proposition():
    """The rejection has to say which fact is missing, or nobody can fix it."""
    model = promela.compile_mission(plan(visit(10), SAMPLE), SCHEMA)
    ok, reason = promela.coverage_gap("<>sampled_tree_35", model)
    assert not ok
    assert "sampled_tree_35" in reason


def test_coverage_ignores_operators_and_literals():
    """`U`, `X` and `true` are syntax, not propositions to look up."""
    model = promela.compile_mission(plan(visit(10), SAMPLE), SCHEMA)
    ok, reason = promela.coverage_gap(
        "[](true -> (at_tree_10 U sampled_tree_10)) && X true", model
    )
    assert ok, reason


def test_formula_aps_extraction():
    assert promela.formula_aps("<>(at_tree_1 && X sampled_tree_2)") == {
        "at_tree_1",
        "sampled_tree_2",
    }

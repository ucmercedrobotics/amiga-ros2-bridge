"""The verification gate, with the model stubbed and SPIN real.

The formula is injected rather than generated -- an LLM in a test makes the
result a coin flip -- but the model checking is genuine, because "does this
reject a bad plan" is the only question worth asking here.

The case that matters most is ``test_mission_text_may_not_be_rewritten``. The
formula is derived from the mission text, so a planner that can edit that text
authors the specification it is graded against, and every other test in this
file would keep passing while verifying nothing.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.verification import ltl, ltl_gate, verify  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")

needs_spin = pytest.mark.skipif(not verify.spin_available(), reason="spin not on PATH")

MISSION = "sample leaves from trees 10 and 60"
FORMULA = "<>sampled_tree_10 && <>sampled_tree_60"


def plan(*body: str, mission: str = MISSION) -> str:
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

BOTH = plan(visit(10), SAMPLE, visit(60), SAMPLE)
ONLY_10 = plan(visit(10), SAMPLE)


def gate(formula=FORMULA, error=None):
    calls = []

    def generate(text):
        calls.append(text)
        return ltl.Formula(None, error) if error else ltl.Formula(formula)

    made = ltl_gate.LtlGate(SCHEMA, generate=generate)
    made.calls = calls
    return made


# ---------------------------------------------------------------------------
# The immutability rule
# ---------------------------------------------------------------------------


def test_mission_text_may_not_be_rewritten():
    """A replan that edits <Mission> is rejected, however good its tree.

    Without this the planner can weaken the specification to match whatever it
    produced, and the verification below becomes a tautology.
    """
    weakened = plan(visit(10), SAMPLE, mission="sample leaves from tree 10")
    result = gate().evaluate(weakened, active=BOTH)
    assert not result.accepted
    assert "may not be rewritten" in result.reason


def test_unchanged_mission_text_passes_the_rule():
    result = gate().evaluate(BOTH, active=BOTH)
    assert result.accepted, result.reason


def test_no_active_plan_means_no_opinion():
    """The first plan has nothing to have drifted from."""
    assert gate().evaluate(BOTH, active=None).accepted


def test_coordinator_may_extend_the_mission_text():
    """Absorbing a peer's task legitimately changes what the mission is."""
    made = gate(formula="<>sampled_tree_10")
    made.allow_mission_text("sample leaves from tree 35")
    extended = plan(visit(10), SAMPLE, mission=f"{MISSION}; sample leaves from tree 35")
    assert made.evaluate(extended, active=BOTH).accepted


def test_the_extension_must_match_exactly():
    """A different appendix is still a rewrite."""
    made = gate()
    made.allow_mission_text("sample leaves from tree 35")
    forged = plan(visit(10), SAMPLE, mission=f"{MISSION}; and also do whatever")
    assert not made.evaluate(forged, active=BOTH).accepted


# ---------------------------------------------------------------------------
# Coverage
# ---------------------------------------------------------------------------


def test_dropped_objective_is_caught_by_coverage():
    """The plan stops sampling tree 60 while the mission still asks for it."""
    result = gate().evaluate(ONLY_10, active=BOTH)
    assert not result.accepted
    assert "sampled_tree_60" in result.reason
    assert "coverage" in result.reason.lower()


# ---------------------------------------------------------------------------
# Verification proper
# ---------------------------------------------------------------------------


@needs_spin
def test_satisfying_plan_is_accepted_and_marked_verified():
    result = gate().evaluate(BOTH, active=BOTH)
    assert result.accepted
    assert result.verified


@needs_spin
def test_violating_plan_is_rejected_with_a_counterexample():
    """Ordering the mission forbids, with everything else in place."""
    made = gate(formula="<>(at_tree_60 && <>at_tree_10)")
    result = made.evaluate(BOTH, active=BOTH)
    assert not result.accepted
    assert result.verified, "a refutation is a verification result"
    assert "violation" in result.reason.lower()
    assert result.reason.strip()


# ---------------------------------------------------------------------------
# Degrading loudly
# ---------------------------------------------------------------------------


def test_no_formula_accepts_but_does_not_claim_verification():
    """A model outage must not stall the mission, or be mistaken for a pass."""
    result = gate(error="LLM call failed: boom").evaluate(BOTH, active=BOTH)
    assert result.accepted
    assert not result.verified
    assert "unverified" in result.reason


def test_missing_mission_text_is_not_a_verification():
    result = gate().evaluate(plan(visit(10), SAMPLE, mission=""), active=None)
    assert result.accepted
    assert not result.verified


def test_unmodellable_plan_is_rejected():
    """A tree that cannot be compiled is not a tree that passed."""
    orphan = plan(SAMPLE)
    result = gate().evaluate(orphan, active=orphan)
    assert not result.accepted
    assert "cannot verify" in result.reason


# ---------------------------------------------------------------------------
# The cache
# ---------------------------------------------------------------------------


def test_formula_is_generated_once_per_mission_text():
    """The model call is the slow part; a retry loop must not repeat it."""
    made = gate()
    for _ in range(3):
        made.evaluate(BOTH, active=BOTH)
    assert made.calls == [MISSION]


def test_a_failed_generation_is_not_cached():
    """A transient outage must not pin the mission as unverifiable."""
    made = gate(error="timeout")
    made.evaluate(BOTH, active=BOTH)
    made.evaluate(BOTH, active=BOTH)
    assert len(made.calls) == 2


def test_warm_populates_the_cache():
    made = gate()
    made.warm(MISSION)
    made.evaluate(BOTH, active=BOTH)
    assert made.calls == [MISSION]

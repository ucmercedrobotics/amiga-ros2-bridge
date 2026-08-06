"""SPIN, over real models built from the real example missions.

This is the file a reviewer should read to decide whether the verification
claim is worth anything, so it is deliberately the thickest one and it does not
mock SPIN. A mocked model checker tests that we can spell ``subprocess``.

Two things get pinned. First, that violations are actually caught -- a verifier
nobody has watched reject something is indistinguishable from ``return True``.
Second, the failure modes, because ``verify.check`` has three outcomes and the
dangerous one is the middle: SPIN exits 0 when it *refutes* a property just as
it does when it proves one, so anything reading the exit status as the verdict
reports violations as passes.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.verification import promela, verify  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")
SCHEMA = os.path.join(REPO, "amiga_ros2_behavior_tree", "schemas", "amiga_btcpp.xsd")

needs_spin = pytest.mark.skipif(not verify.spin_available(), reason="spin not on PATH")


@pytest.fixture(scope="module")
def sample_leafs():
    """Model of the mission "sample leaves from trees 10 and 60"."""
    with open(os.path.join(EXAMPLES, "sample_leafs.xml")) as handle:
        return promela.compile_mission(handle.read(), SCHEMA)


# ---------------------------------------------------------------------------
# Properties that hold
# ---------------------------------------------------------------------------


@needs_spin
@pytest.mark.parametrize(
    "formula",
    [
        "<>sampled_tree_10",
        "<>sampled_tree_10 && <>sampled_tree_60",
        "<>(at_tree_10 && <>(sampled_tree_10 && <>(at_tree_60 && <>sampled_tree_60)))",
        # Latching: once sampled, it stays sampled.
        "[](sampled_tree_10 -> []sampled_tree_10)",
        # Sampling tree 10 happens only after arriving at it.
        "(!sampled_tree_10) U at_tree_10",
    ],
)
def test_holds(sample_leafs, formula):
    verdict = verify.check(sample_leafs.source, formula)
    assert verdict.conclusive, verdict.error
    assert verdict.holds, f"expected to hold:\n{verdict.counterexample}"


# ---------------------------------------------------------------------------
# Properties that do not — the half that matters
# ---------------------------------------------------------------------------


@needs_spin
@pytest.mark.parametrize(
    "formula",
    [
        # The plan does tree 10 first, so this ordering is refuted.
        "<>(at_tree_60 && <>(sampled_tree_60 && <>at_tree_10))",
        # The plan does sample tree 10.
        "[]!sampled_tree_10",
        # The robot is never at both trees at once.
        "<>(at_tree_10 && at_tree_60)",
    ],
)
def test_violated(sample_leafs, formula):
    verdict = verify.check(sample_leafs.source, formula)
    assert verdict.conclusive, verdict.error
    assert not verdict.holds


@needs_spin
def test_violation_returns_a_counterexample(sample_leafs):
    """A rejection the planner cannot act on is barely better than none."""
    verdict = verify.check(sample_leafs.source, "[]!sampled_tree_10")
    assert not verdict.holds
    assert verdict.counterexample.strip()
    assert "sampled_tree_10" in verdict.counterexample


@needs_spin
def test_ordering_is_actually_observed(sample_leafs):
    """The trace has more than one observable step.

    Guards the ``d_step`` encoding. Collapse the plan into a single atomic
    transition and every ``X``/``U`` formula becomes meaningless while the
    ``<>`` ones keep passing -- the suite above would stay green and the
    ordering guarantees would be fiction.
    """
    before = verify.check(sample_leafs.source, "(!sampled_tree_60) U at_tree_10")
    after = verify.check(sample_leafs.source, "(!sampled_tree_10) U at_tree_60")
    assert before.holds, "tree 10 is reached before tree 60 is sampled"
    assert not after.holds, "tree 10 is sampled before tree 60 is reached"


# ---------------------------------------------------------------------------
# Editing the plan changes the verdict
# ---------------------------------------------------------------------------


@needs_spin
def test_dropping_a_task_refutes_the_mission():
    """The replan case: a plan that quietly drops tree 60 fails the spec.

    This is what the whole hook is for. The formula comes from the mission text
    ("sample trees 10 and 60"), which a replan may not rewrite; the tree is what
    changed.
    """
    from test_promela import SAMPLE, plan, visit

    formula = "<>sampled_tree_10 && <>sampled_tree_60"

    full = promela.compile_mission(plan(visit(10), SAMPLE, visit(60), SAMPLE), SCHEMA)
    assert verify.check(full.source, formula).holds

    cut = promela.compile_mission(plan(visit(10), SAMPLE), SCHEMA)
    ok, reason = promela.coverage_gap(formula, cut)
    assert not ok, "dropping tree 60 must be caught"
    assert "sampled_tree_60" in reason


@needs_spin
def test_absorbing_a_task_satisfies_the_extended_mission():
    """The auction case: the winner's plan covers the clause it took on."""
    from test_promela import SAMPLE, plan, visit

    winner = promela.compile_mission(plan(visit(10), SAMPLE, visit(35), SAMPLE), SCHEMA)
    verdict = verify.check(winner.source, "<>sampled_tree_10 && <>sampled_tree_35")
    assert verdict.conclusive, verdict.error
    assert verdict.holds


# ---------------------------------------------------------------------------
# Failing closed
# ---------------------------------------------------------------------------


@needs_spin
def test_undeclared_proposition_is_inconclusive_not_a_pass(sample_leafs):
    """SPIN reports an unknown proposition as a *parse* error, not a violation.

    So it arrives with no ``errors:`` line at all. If that fell through to
    "holds", any formula mentioning a fact the plan omits would pass -- exactly
    backwards. Callers run ``coverage_gap`` first so this is unreachable in
    production; it is pinned here because the consequence of getting it wrong is
    silent.
    """
    verdict = verify.check(sample_leafs.source, "<>sampled_tree_35")
    assert not verdict.holds
    assert not verdict.conclusive
    assert "sampled_tree_35" in verdict.error


def test_empty_formula_is_inconclusive(sample_leafs):
    verdict = verify.check(sample_leafs.source, "   ")
    assert not verdict.holds
    assert not verdict.conclusive


@needs_spin
def test_malformed_model_is_inconclusive():
    verdict = verify.check("this is not promela", "<>true")
    assert not verdict.holds
    assert not verdict.conclusive


def test_missing_spin_is_inconclusive(sample_leafs, monkeypatch):
    """No checker installed must not read as a proof."""
    monkeypatch.setattr(verify, "SPIN_BINARY", "definitely-not-a-real-binary")
    verdict = verify.check(sample_leafs.source, "<>sampled_tree_10")
    assert not verdict.holds
    assert not verdict.conclusive
    assert "not found" in verdict.error


@needs_spin
def test_runs_are_isolated(sample_leafs):
    """Successive checks must not read each other's leftovers.

    SPIN writes ``pan``, ``pan.c`` and the ``.trail`` beside the model. Sharing
    a working directory is how a stale trail from a previous run turns the next
    run's verdict into a coin flip.
    """
    before = set(os.listdir(os.getcwd()))
    assert not verify.check(sample_leafs.source, "[]!sampled_tree_10").holds
    assert verify.check(sample_leafs.source, "<>sampled_tree_10").holds
    assert set(os.listdir(os.getcwd())) == before, "spin left files in the cwd"

"""The statistics `analyze.py` reports, pinned against known values.

Nothing here touches a `results.jsonl` file or the registry's scenario
dictionaries -- `wilson`, `ci_verdict` and `deterministic_point_outcome` are
pure functions of plain numbers (and, for the last one, small hand-built
`(decision, correct)` tuples), and the point of testing them this way is
that a regression in the arithmetic itself (a flipped Wilson-bound
inequality, a missed nondeterminism case) cannot hide behind a fixture built
to already agree with whatever the code currently does. Every expected
value below is either a textbook example (Wilson) or read directly off
`ci_verdict`'s own documented cases (never merely "what analyze.py produces
today").

There is no test here for a p-value or a multiple-comparisons correction,
because `analyze.py` computes neither anymore: the deterministic arm is a
fixed function with no sampling distribution, so a binomial null against it
has zero variance and degenerates to p = 0.0 or 1.0 exactly -- inference
theatre, not inference. What replaced it, and what this file tests instead,
is a CI-based verdict (`ci_verdict`) read off the LLM arm's Wilson interval
together with a plain fact about the deterministic arm
(`deterministic_point_outcome`): an interval on the arm that varies, and a
fact about the arm that does not. See the module docstring of `analyze.py`
and Revisions R2/R3 in `ablation-experiment-spec.md` for the full account of
why the binomial test was removed.

Runs on bare Python: `math` and the functions under test, nothing else.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from analyze import (  # noqa: E402
    AMBIGUOUS,
    BASELINE_HOLDS,
    LLM_WINS,
    ci_verdict,
    deterministic_point_outcome,
    wilson,
)


# ==========================================================================
# wilson()
# ==========================================================================


def test_wilson_of_zero_trials_is_the_degenerate_interval():
    assert wilson(0, 0) == (0.0, 0.0)


def test_wilson_matches_the_textbook_example_eight_of_ten():
    # The standard worked example (Newcombe 1998 / Wikipedia's "binomial
    # proportion confidence interval"): 8 successes out of 10 trials, 95%,
    # z=1.96, gives a Wilson interval of approximately (0.49, 0.94).
    lo, hi = wilson(8, 10)
    assert math.isclose(lo, 0.49, abs_tol=0.01)
    assert math.isclose(hi, 0.943, abs_tol=0.01)


def test_wilson_of_zero_successes_has_a_nonzero_upper_bound():
    # Zero observed failures of a policy is not proof of a zero failure
    # rate -- the Wilson interval's whole reason for existing over a naive
    # p +/- z*se interval is that it does not collapse to a point at k=0.
    lo, hi = wilson(0, 10)
    assert lo == 0.0
    assert hi > 0.0


def test_wilson_of_all_successes_has_a_nonone_lower_bound():
    lo, hi = wilson(10, 10)
    assert lo > 0.0
    assert hi == 1.0


def test_wilson_interval_widens_as_n_shrinks_at_the_same_proportion():
    # Same observed proportion (0.5), less evidence for it -- the CI must
    # not narrow, or the interval is not doing its job.
    narrow = wilson(50, 100)
    wide = wilson(5, 10)
    assert (wide[1] - wide[0]) > (narrow[1] - narrow[0])


# ==========================================================================
# deterministic_point_outcome() -- the zero-variance / nondeterminism check
# ==========================================================================


def test_deterministic_point_outcome_of_a_constant_arm_has_zero_variance_and_no_warning():
    # Twenty seeds, same decision, same correctness every time -- exactly
    # what the deterministic arm is supposed to look like.
    trials = [("re_delegate", True)] * 20
    outcome = deterministic_point_outcome("fleet/peer_capable", trials)
    assert outcome["constant"] is True
    assert outcome["variance"] == 0.0
    assert outcome["decision"] == "re_delegate"
    assert outcome["correct"] is True
    assert outcome["warning"] is None


def test_deterministic_point_outcome_of_a_constant_but_incorrect_arm_still_has_zero_variance():
    # Constant does not mean correct -- fleet_wide's baseline is constant
    # AND wrong, and that combination must not be mistaken for the warning
    # case just because "correct" is False.
    trials = [("re_delegate", False)] * 20
    outcome = deterministic_point_outcome("fleet/fleet_wide", trials)
    assert outcome["constant"] is True
    assert outcome["variance"] == 0.0
    assert outcome["correct"] is False
    assert outcome["warning"] is None


def test_deterministic_point_outcome_of_a_varying_arm_triggers_the_loud_warning():
    # One seed out of twenty produced a different decision -- nondeterminism
    # leaking into a policy the design assumes is seed-invariant. This must
    # be flagged loudly, not averaged into a quiet variance number.
    trials = [("re_delegate", True)] * 19 + [("drop_task(hold)", False)]
    outcome = deterministic_point_outcome("fleet/peer_capable", trials)
    assert outcome["constant"] is False
    assert outcome["warning"] is not None
    assert "NONDETERMINISM DETECTED" in outcome["warning"]
    assert "fleet/peer_capable" in outcome["warning"]


def test_deterministic_point_outcome_flags_varying_decisions_even_if_correctness_does_not_vary():
    # Two DIFFERENT wrong decisions across seeds would show zero variance in
    # the correctness indicator (every trial is False) while still being a
    # broken seed-invariance assumption -- the check must key off the
    # decision itself, not merely off `correct`.
    trials = [("re_delegate", False)] * 10 + [("drop_task(drop)", False)] * 10
    outcome = deterministic_point_outcome("fleet/fleet_wide", trials)
    assert outcome["constant"] is False
    assert outcome["warning"] is not None
    assert outcome["variance"] == 0.0  # correctness alone never varied
    assert len(outcome["distinct_decisions"]) == 2


# ==========================================================================
# ci_verdict() -- the CI-based comparison that replaced the binomial test
# ==========================================================================


def test_ci_verdict_is_llm_wins_when_baseline_is_wrong_and_the_llm_ci_clears_zero():
    # Deterministic was incorrect on this scenario; the LLM arm's Wilson
    # lower bound is strictly above 0, ruling out "the LLM's true accuracy
    # here is zero" -- the one claim needed to say the LLM did better than a
    # baseline that is wrong on every seed.
    assert ci_verdict(False, 0.01, 0.99) == LLM_WINS


def test_ci_verdict_is_baseline_holds_when_baseline_is_correct_and_the_llm_ci_reaches_one():
    # Deterministic was correct on this scenario; the LLM arm's CI still
    # reaches 1, so nothing in the data contradicts the baseline's own
    # perfect record.
    assert ci_verdict(True, 0.85, 1.0) == BASELINE_HOLDS


def test_ci_verdict_is_ambiguous_when_baseline_is_wrong_but_the_llm_ci_still_includes_zero():
    # Deterministic was incorrect, but the LLM arm's CI has not yet ruled
    # out zero either -- not enough evidence to call it a win for the LLM,
    # so this must not be forced into LLM_WINS.
    assert ci_verdict(False, 0.0, 0.5) == AMBIGUOUS


def test_ci_verdict_is_ambiguous_when_baseline_is_correct_but_the_llm_ci_excludes_one():
    # Deterministic was correct, but the LLM arm's CI is demonstrably below
    # 1 -- a real finding (the LLM arm is imperfect where the baseline was
    # not), but not "the baseline holds" in R3's sense, so this must not be
    # forced into BASELINE_HOLDS.
    assert ci_verdict(True, 0.2, 0.8) == AMBIGUOUS


def test_ci_verdict_treats_a_llm_lower_bound_of_exactly_zero_as_ambiguous_not_a_win():
    # The boundary case: `wilson_lo > 0.0` is a strict inequality, so a
    # lower bound that only just touches zero has not actually ruled zero
    # out and must stay AMBIGUOUS rather than round up to LLM_WINS.
    assert ci_verdict(False, 0.0, 0.9) == AMBIGUOUS


def test_ci_verdict_treats_a_llm_upper_bound_of_exactly_one_as_baseline_holds():
    # The mirror boundary: `wilson_hi >= 1.0` is inclusive, so an interval
    # that reaches exactly 1 counts as not excluding "also perfect."
    assert ci_verdict(True, 0.1, 1.0) == BASELINE_HOLDS

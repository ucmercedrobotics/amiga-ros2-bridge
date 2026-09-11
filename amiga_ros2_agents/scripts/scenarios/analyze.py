#!/usr/bin/env python3
"""

  python3 scripts/scenarios/analyze.py runs/<run_id>/results.jsonl

Reads `results.jsonl` from either harness (`research_harness.py`, single-
robot, or `fleet_harness.py`, fleet) and reports, per `(harness, arm,
scenario)`:
  - accuracy k/n against `registry.py`'s pre-registered `expected`, with a
    Wilson 95% CI
  - decision distribution + modal-decision fraction (consistency)
  - schema-valid rate (single-robot only: deterministic lxml check against
    the XSD)
  - mean viability budget the arbiter set (single-robot only)
  - mean latency
Then, per scenario, the deterministic arm's point outcome -- its decision
and whether that decision is correct, reported as a FACT with no confidence
interval and no p-value, and rendered visibly differently from the LLM
arms' rows so it cannot be mistaken for a sampled quantity -- alongside
each LLM arm's per-scenario accuracy with a Wilson 95% CI, and a verdict
column that reads the comparison directly off that CI (see
`ci_verdict()`).

There is deliberately no hypothesis test here. See Revisions R2 and R3 in
`ablation-experiment-spec.md` for the full account: R2 proposed testing
each LLM arm's accuracy against the deterministic arm's fixed per-scenario
outcome with an exact binomial test. Implemented, it degenerates -- the
deterministic arm sits at p0 = 0 or p0 = 1 exactly, a binomial null with
zero variance, so the p-value collapses to exactly 0.0 or 1.0 and a single
LLM miss out of twenty trials is enough to report p = 0. That is a
deterministic check wearing a p-value, not inference: a fixed function has
no sampling distribution, so "always correct" is not an estimate of a
probability near 1, it is a fact evaluated once, and there is no population
to test it against. R3 drops the test. A reader will look for a p-value
here; its absence is a property of this design worth stating, not an
omission -- one arm of the comparison does not vary, so the honest report
is an interval on the arm that does and a fact about the arm that does not.

And, for the fleet harness only, a secondary table of auctions opened and
messages carried per trial, with the seed kept as the unit of variation
there because it still does real work at that layer: it drives the fleet's
backoff jitter, so the *outcome* of a trial (which peer wins an auction) can
vary seed to seed even when the deterministic arm's *decision* does not --
this is where a wrong `re_delegate` (`fleet_wide`, most saliently) is
expected to show its cost even on a trial where `correct` alone would not
reveal it.

Ground truth -- `expected` and the `scorer` that turns a `decision` into a
bool -- comes from `registry.py`, never from a rule re-derived here: the
whole reason ground truth lives in one pre-registered, committed file is so
scoring cannot drift toward a result once results exist. This file trusts a
record's own `correct` field for nothing; it recomputes it from the
registry every time, so a hand-edited `results.jsonl` cannot silently
disagree with the pre-registration either.

Also writes grading_sheet.csv, single-robot rows only, for the human/LLM-
judge rubric pass the closed-form scorer cannot finish alone (`sick_tree`
scores any coherent action correct; which of ignore/append/re-prioritize it
actually was is a judgement call for the sheet, not this script).
"""

import csv
import json
import math
import os
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from registry import FLEET_SCENARIOS, SINGLE_SCENARIOS  # noqa: E402

# Point this at your schema (same one bt.cpp validates against).
XSD_PATH = "/amiga-ros2-bridge/amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd"


def _scenario(rec):
    """The registry entry `rec` should be scored against, or None.

    None only for a scenario id neither registry dict recognizes -- a
    results file from before a scenario was renamed, or a typo -- which is
    reported rather than silently scored as incorrect.
    """
    table = SINGLE_SCENARIOS if rec.get("harness") == "single" else FLEET_SCENARIOS
    return table.get(rec.get("scenario"))


def _correct(rec) -> bool:
    """Recomputed from the registry's own scorer, ignoring `rec["correct"]`.

    See the module docstring: trusting a record's own `correct` field would
    let a results file assert its own grade. A decision of None (a launch
    failure, an unanswered triage call, a timeout) is never correct, and no
    scenario's scorer is asked to make sense of it.
    """
    scenario = _scenario(rec)
    decision = rec.get("decision")
    if scenario is None or decision is None:
        return False
    return bool(scenario["scorer"](decision))


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (round(center - half, 3), round(center + half, 3))


#: Verdict labels `ci_verdict()` returns. Named constants rather than bare
#: strings so a typo in a comparison (`== "llm_wins"` vs `"llm-wins"`) is a
#: NameError at import time instead of a silently-always-false comparison.
BASELINE_HOLDS = "baseline-holds"
LLM_WINS = "llm-wins"
AMBIGUOUS = "ambiguous"


def ci_verdict(deterministic_correct: bool, wilson_lo: float, wilson_hi: float) -> str:
    """Read the scenario-level comparison directly off the LLM arm's Wilson
    CI, with no hypothesis test involved -- see the module docstring and
    Revision R3 in `ablation-experiment-spec.md` for why there is no p-value
    to derive this from instead.

    There are exactly two claims this design is positioned to make, both
    stated in `ablation-experiment-spec.md`'s R3:

    - `LLM_WINS`: the deterministic arm was INCORRECT on this scenario, and
      the LLM arm's Wilson lower bound clears 0 -- i.e. the interval rules
      out "the LLM's true accuracy here is zero," which is the one claim
      needed to say the LLM arm did better than a baseline that is wrong on
      every seed.
    - `BASELINE_HOLDS`: the deterministic arm was CORRECT on this scenario,
      and the LLM arm's CI still reaches 1 -- i.e. the interval does not
      rule out "the LLM's true accuracy here is also perfect," so nothing in
      the data contradicts the baseline's own perfect record.

    Anything else is `AMBIGUOUS` and reported as such rather than forced
    into one of the two named buckets: a deterministic-incorrect scenario
    where the LLM's CI still includes 0 (not enough evidence yet that the
    LLM clears the floor), or a deterministic-correct scenario where the
    LLM's CI excludes 1 (the LLM arm is demonstrably imperfect where the
    baseline was not, which is a real finding but not "the baseline holds"
    in the sense R3 defines it -- it deserves a reader's own look at the
    table, not a verdict label claiming more than the CI supports).
    """
    if deterministic_correct:
        return BASELINE_HOLDS if wilson_hi >= 1.0 else AMBIGUOUS
    return LLM_WINS if wilson_lo > 0.0 else AMBIGUOUS


def schema_valid(xml):
    if not xml:
        return None
    try:
        from lxml import etree
    except ImportError:
        return None  # lxml not available here; leave blank
    try:
        schema = etree.XMLSchema(etree.parse(XSD_PATH))
        schema.assertValid(etree.fromstring(xml.encode()))
        return True
    except Exception:
        return False


def _mean(values):
    values = [v for v in values if isinstance(v, (int, float))]
    return round(sum(values) / len(values), 1) if values else "-"


def _group_trials(records):
    """(harness, scenario, arm) -> [(decision, correct), ...], one entry per
    seed.

    A duplicate record at the same `(harness, scenario, arm, seed)` -- a
    re-run -- keeps only the first, the same de-duplication policy the old
    McNemar table used to apply locally before this replaced it; centralizing
    it here means both the deterministic fact table and the LLM-arm CI/
    comparison tables see one consistent count of trials per seed, rather
    than each re-deriving its own notion of "how many times did this arm run
    this scenario."
    """
    by_seed = {}
    for r in records:
        key = (r["harness"], r["scenario"], r["arm"], r["seed"])
        by_seed.setdefault(key, (r.get("decision"), r["_correct"]))
    grouped = defaultdict(list)
    for (harness, scenario, arm, _seed), pair in by_seed.items():
        grouped[(harness, scenario, arm)].append(pair)
    return grouped


def deterministic_point_outcome(label: str, trials):
    """Summarize the deterministic arm's trials for one scenario as a single
    POINT outcome rather than a distribution: one decision, whether it is
    correct, and the within-arm variance across seeds -- which should be
    exactly zero, because the deterministic policy reads no seed-dependent
    evidence and nothing about it changes seed to seed. Reporting that zero
    plainly, rather than collapsing it away as "nothing to see here," is
    itself part of the honest comparison: a baseline that consults no model
    has zero variance on this axis by construction, and that is one of its
    real, and reportable, properties.

    `trials` is the list of `(decision, correct)` pairs `_group_trials`
    produced for this `(harness, scenario, "deterministic")` key -- one entry
    per seed. `label` is used only to build the warning message below; it
    carries no computation.

    If the decisions are NOT all identical, that is not a more interesting
    baseline -- it is a sign that something the design assumes to be
    deterministic is not, e.g. nondeterminism leaking in from a shared
    fixture, a scenario that accidentally reads the seed after all, or a
    registry/harness mismatch. This function detects that directly, from the
    decisions themselves rather than from the correctness variance alone
    (two different WRONG decisions would show zero correctness-variance while
    still being a broken invariant), and returns a loud, specific warning
    string rather than silently averaging over the disagreement -- an average
    over "should never vary" is not a summary, it is a bug wearing a summary's
    clothes.
    """
    n = len(trials)
    decisions = [d for d, _ in trials]
    corrects = [c for _, c in trials]
    distinct_decisions = sorted({repr(d) for d in decisions})
    constant = len(distinct_decisions) <= 1

    mean = sum(corrects) / n if n else 0.0
    variance = sum((c - mean) ** 2 for c in corrects) / n if n else 0.0

    warning = None
    if not constant:
        warning = (
            f"NONDETERMINISM DETECTED in the deterministic arm for {label}: "
            f"{n} seeds produced {len(distinct_decisions)} distinct decisions "
            f"{distinct_decisions} instead of one. The deterministic arm is "
            f"supposed to be seed-invariant by construction; treat this "
            f"scenario's baseline-holds/llm-wins verdict as resting on a "
            f"reference that was not actually fixed, and go find where the "
            f"nondeterminism is leaking in before trusting it."
        )

    return {
        "n": n,
        "decision": decisions[0] if decisions else None,
        "correct": corrects[0] if corrects else None,
        "variance": variance,
        "constant": constant,
        "distinct_decisions": distinct_decisions,
        "warning": warning,
    }


def _print_accuracy_table(records):
    """Per `(harness, arm, scenario)` accuracy with a Wilson 95% CI.

    LLM arms only -- the deterministic arm's per-scenario outcome is a FACT
    (a fixed function evaluated once, not a sample), so it is reported
    separately by `_print_deterministic_point_outcomes` with no CI column at
    all, rather than in here where a Wilson interval would misleadingly
    dress a point outcome up as an estimate with sampling uncertainty. See
    the module docstring and R3 in `ablation-experiment-spec.md`.
    """
    cells = defaultdict(list)
    for r in records:
        if r["arm"] == "deterministic":
            continue
        cells[(r["harness"], r["arm"], r["scenario"])].append(r)

    print("\n[LLM arms] accuracy per scenario, with Wilson 95% CI")
    print(
        f"{'harness':7} {'arm':13} {'scenario':14} {'n':>3} {'correct':>9} "
        f"{'wilson95':>15} {'schemaOK':>9} {'consist':>8} "
        f"{'budget':>7} {'lat(s)':>8}"
    )
    print("-" * 110)
    for (harness, arm, scenario), rs in sorted(cells.items()):
        n = len(rs)
        k = sum(r["_correct"] for r in rs)
        lo, hi = wilson(k, n)

        sv = [r["_schema_valid"] for r in rs if r["_schema_valid"] is not None]
        sv_rate = f"{sum(sv)}/{len(sv)}" if sv else "n/a"

        dist = defaultdict(int)
        for r in rs:
            dist[r.get("decision")] += 1
        consistency = round(max(dist.values()) / n, 2) if n else 0.0

        budget = _mean([r.get("viability_budget") for r in rs])
        lat = _mean([r.get("latency_sec") for r in rs])

        print(
            f"{harness:7} {arm:13} {scenario:14} {n:>3} {k}/{n:<7} "
            f"[{lo:.2f},{hi:.2f}]{'':>4} {sv_rate:>9} "
            f"{consistency:>8} {budget:>7} {lat:>8}   dist={dict(dist)}"
        )


def _print_deterministic_point_outcomes(records):
    """Per `(harness, scenario)`: the deterministic arm's decision and
    whether it is correct, reported as a FACT -- no Wilson CI, no p-value --
    with its within-seed variance stated too, because that variance should
    be exactly 0.0 by construction and its being anything else is the one
    thing worth checking before trusting this table at all (see
    `deterministic_point_outcome`).

    Printed with a `FACT` marker on every row and boxed off with `====`
    rule lines, deliberately unlike the `----`-ruled Wilson-CI tables above
    and below, so a reader skimming output cannot mistake a row here for a
    sampled quantity with an associated interval -- there isn't one.
    """
    grouped = _group_trials(records)
    keys = sorted(
        {(h, s) for (h, s, a) in grouped if a == "deterministic"},
    )
    if not keys:
        return

    print(
        "\n[deterministic arm] FACT per scenario -- fixed function, not a "
        "sample: no CI, no p-value (variance across seeds should be 0.0)"
    )
    print(
        f"{'':4} {'harness':7} {'scenario':14} {'n':>3} {'decision':16} "
        f"{'correct':>7} {'variance':>9}"
    )
    print("=" * 66)
    for harness, scenario in keys:
        trials = grouped[(harness, scenario, "deterministic")]
        outcome = deterministic_point_outcome(f"{harness}/{scenario}", trials)
        print(
            f"FACT {harness:7} {scenario:14} {outcome['n']:>3} "
            f"{str(outcome['decision']):16} {str(outcome['correct']):>7} "
            f"{outcome['variance']:>9.4f}"
        )
        if outcome["warning"]:
            print(f"  !!! {outcome['warning']}")
    print("=" * 66)


def _print_comparison_table(records):
    """The study's primary comparison: per harness, per LLM arm, per
    scenario, read `ci_verdict()` off the deterministic arm's FACT and the
    LLM arm's Wilson 95% CI. No hypothesis test, no correction for multiple
    comparisons -- there are no p-values here to correct (R3 in
    `ablation-experiment-spec.md`; see also the module docstring). Holm went
    with the binomial test it used to correct; nothing else in this file
    computes a p-value, so `holm()` has been deleted rather than kept
    unused.

    A scenario whose deterministic arm is not seed-invariant (flagged by
    `_print_deterministic_point_outcomes` above) still gets a verdict here --
    `deterministic_point_outcome`'s first trial stands in as "the" fact, the
    same way any other cell would if it had only one entry to give -- but
    the verdict is worth reading only after checking that warning was not
    raised; a comparison against a reference that was not actually fixed
    does not mean what this table claims it means.
    """
    grouped = _group_trials(records)
    harnesses = sorted({h for (h, _s, _a) in grouped})
    for harness in harnesses:
        scenarios = sorted({s for (h, s, _a) in grouped if h == harness})
        arms = sorted({a for (h, _s, a) in grouped if h == harness})
        if "deterministic" not in arms:
            print(
                f"\n[comparison] {harness}: no deterministic arm in this run -- nothing to compare against"
            )
            continue
        llm_arms = [a for a in arms if a != "deterministic"]

        det_outcome_by_scenario = {
            scenario: deterministic_point_outcome(
                f"{harness}/{scenario}", grouped[(harness, scenario, "deterministic")]
            )
            for scenario in scenarios
            if (harness, scenario, "deterministic") in grouped
        }

        for arm in llm_arms:
            print(
                f"\n[comparison] {harness}: {arm} vs deterministic "
                f"(baseline FACT vs LLM arm's Wilson 95% CI -- no p-value, see docstring)"
            )
            print(
                f"{'scenario':14} {'n':>4} {'k':>4} {'baseline':>10} "
                f"{'wilson95':>15} {'verdict':>15}"
            )
            print("-" * 66)

            for scenario in scenarios:
                det_outcome = det_outcome_by_scenario.get(scenario)
                trials = grouped.get((harness, scenario, arm), [])
                if det_outcome is None or not trials:
                    print(
                        f"{scenario:14}  (no deterministic reference or no {arm} trials)"
                    )
                    continue
                n = len(trials)
                k = sum(1 for _, correct in trials if correct)
                lo, hi = wilson(k, n)
                verdict = ci_verdict(det_outcome["correct"], lo, hi)
                ref = "correct" if det_outcome["correct"] else "incorrect"
                print(
                    f"{scenario:14} {n:>4} {k:>4} {ref:>10} "
                    f"[{lo:.2f},{hi:.2f}]{'':>4} {verdict:>15}"
                )


def _print_fleet_secondary_table(records):
    fleet = [r for r in records if r["harness"] == "fleet"]
    if not fleet:
        return
    print("\n[secondary, fleet only] auctions opened and messages carried per trial")
    print(
        f"{'arm':13} {'scenario':14} {'n':>3} {'auctions/trial':>15} "
        f"{'msgs/trial':>11}"
    )
    print("-" * 62)
    cells = defaultdict(list)
    for r in fleet:
        cells[(r["arm"], r["scenario"])].append(r)
    for (arm, scenario), rs in sorted(cells.items()):
        n = len(rs)
        outcomes = [r.get("outcome") or {} for r in rs]
        auctions = _mean([o.get("auctions_opened") for o in outcomes])
        msg_totals = [sum((o.get("message_counts") or {}).values()) for o in outcomes]
        msgs = _mean(msg_totals)
        print(f"{arm:13} {scenario:14} {n:>3} {auctions:>15} {msgs:>11}")


def _write_grading_sheet(records, path):
    # Single-robot only: the rubric layer (esp. sick_tree) reads XML
    # candidates the fleet harness never produces.
    single = [r for r in records if r["harness"] == "single"]
    sheet = path.parent / "grading_sheet.csv"
    with open(sheet, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(
            [
                "arm",
                "scenario",
                "seed",
                "decision",
                "schema_valid",
                "viability_budget",
                "expected",
                "candidate_xml",
                "grader1_class",
                "grader2_class",
                "grader_notes",
            ]
        )
        for r in single:
            w.writerow(
                [
                    r["arm"],
                    r["scenario"],
                    r["seed"],
                    r.get("decision"),
                    r.get("_schema_valid"),
                    r.get("viability_budget"),
                    r.get("expected"),
                    (r.get("_final_or_candidate") or "")[:4000],
                    "",
                    "",
                    "",
                ]
            )
    print(
        f"\n[analyze] wrote {sheet}  (fill grader1/grader2 columns for the rubric layer)"
    )


def main():
    if len(sys.argv) < 2:
        print("usage: analyze.py runs/<run_id>/results.jsonl")
        sys.exit(1)
    path = Path(sys.argv[1])
    records = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]

    for r in records:
        # Old-schema tolerance: records written before the ablation study
        # renamed `model` -> `arm` and `rep` -> `seed` (and before `harness`
        # existed at all -- every pre-ablation record is single-robot) still
        # need to analyze correctly, so a previously-recorded results.jsonl
        # is never rendered unreadable by this rename.
        r.setdefault("harness", "single")
        if "arm" not in r and "model" in r:
            r["arm"] = r["model"]
        if "seed" not in r and "rep" in r:
            r["seed"] = r["rep"]
        r["_final_or_candidate"] = r.get("final_xml") or r.get("candidate_xml")
        r["_schema_valid"] = schema_valid(r["_final_or_candidate"])
        r["_correct"] = _correct(r)

    _print_accuracy_table(records)
    _print_deterministic_point_outcomes(records)
    _print_comparison_table(records)
    _print_fleet_secondary_table(records)
    _write_grading_sheet(records, path)


if __name__ == "__main__":
    main()

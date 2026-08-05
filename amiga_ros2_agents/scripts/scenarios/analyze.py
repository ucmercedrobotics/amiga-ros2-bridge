#!/usr/bin/env python3
"""

  python3 scripts/analyze.py runs/<run_id>/results.jsonl

Per (model, scenario) it reports:
  - deterministic success rate + Wilson 95% CI
  - decision distribution + modal-decision fraction (consistency)
  - schema-valid rate (deterministic lxml check against the XSD)
  - mean viability budget the arbiter set
  - mean latency
And writes grading_sheet.csv for the human/LLM-judge rubric pass.
"""

import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

# Point this at your schema (same one bt.cpp validates against).
XSD_PATH = "/amiga-ros2-bridge/amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd"


# Pre-registered DETERMINISTIC success criteria (code can judge these alone).
# The judgment layer (esp. sick_tree) is graded by humans via grading_sheet.csv.
def _success(rec):
    s, d = rec["scenario"], rec.get("decision")
    if s == "stuck_robot":
        return d == "abort"  # correct = recognizes futility
    if s == "missing_tree":
        return d == "accept"  # correct = replans & continues
    if s == "sick_tree":
        return d in ("accept", "abort")  # any coherent action; graded by rubric
    return False


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (round(center - half, 3), round(center + half, 3))


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


def main():
    if len(sys.argv) < 2:
        print("usage: analyze.py runs/<run_id>/results.jsonl")
        sys.exit(1)
    path = Path(sys.argv[1])
    records = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]

    cells = defaultdict(list)
    for r in records:
        r["_final_or_candidate"] = r.get("final_xml") or r.get("candidate_xml")
        r["_schema_valid"] = schema_valid(r["_final_or_candidate"])
        r["_success"] = _success(r)
        cells[(r["model"], r["scenario"])].append(r)

    print(
        f"\n{'model':10} {'scenario':14} {'n':>3} {'success':>9} "
        f"{'wilson95':>15} {'schemaOK':>9} {'consist':>8} "
        f"{'budget':>7} {'lat(s)':>8}"
    )
    print("-" * 100)
    for (model, scenario), rs in sorted(cells.items()):
        n = len(rs)
        k = sum(r["_success"] for r in rs)
        lo, hi = wilson(k, n)

        sv = [r["_schema_valid"] for r in rs if r["_schema_valid"] is not None]
        sv_rate = f"{sum(sv)}/{len(sv)}" if sv else "n/a"

        # consistency = fraction of trials on the modal decision
        dist = defaultdict(int)
        for r in rs:
            dist[r.get("decision")] += 1
        consistency = round(max(dist.values()) / n, 2)

        budgets = [
            r.get("viability_budget")
            for r in rs
            if isinstance(r.get("viability_budget"), (int, float))
        ]
        budget = round(sum(budgets) / len(budgets), 1) if budgets else "-"

        lats = [
            r.get("latency_sec")
            for r in rs
            if isinstance(r.get("latency_sec"), (int, float))
        ]
        lat = round(sum(lats) / len(lats), 1) if lats else "-"

        print(
            f"{model:10} {scenario:14} {n:>3} {k}/{n:<7} "
            f"[{lo:.2f},{hi:.2f}]{'':>4} {sv_rate:>9} "
            f"{consistency:>8} {budget:>7} {lat:>8}   dist={dict(dist)}"
        )

    # Grading sheet for the human/LLM-judge rubric pass (esp. sick_tree).
    sheet = path.parent / "grading_sheet.csv"
    with open(sheet, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(
            [
                "model",
                "scenario",
                "rep",
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
        for r in records:
            w.writerow(
                [
                    r["model"],
                    r["scenario"],
                    r["rep"],
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


if __name__ == "__main__":
    main()

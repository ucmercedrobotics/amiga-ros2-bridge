"""The verification gate the arbiter runs over a candidate plan.

Pulls together the three pieces -- ``ltl`` for the specification, ``promela``
for the model, ``verify`` for the check -- and decides. Kept out of
``arbiter_node`` so the decision can be tested without ROS and without a model,
which is where the interesting cases are.

Ordered cheapest-first, and each step exists because the one after it cannot
express the failure properly:

    1. the mission text is unchanged        string compare
    2. the tree compiles to a model         lxml
    3. the plan defines what the formula    set difference
       talks about
    4. the formula holds of the model       SPIN

Step 3 is not redundant with step 4. SPIN reports a proposition the model never
declares as a *parse error*, which arrives with no verdict at all -- so without
step 3 a plan that simply omits an objective produces "spin: Error: undeclared
variable: sampled_tree_60" instead of "the plan never samples tree 60", and the
planner has nothing to act on.

**Step 1 is what makes the rest mean anything.** The formula is derived from the
mission text. If the replanning model may rewrite that text, it authors the
specification it is graded against, and every proof below is a tautology
dressed up as a check. So the text is carried forward from the active plan and
only the coordinator may extend it -- when it absorbs a task from another robot,
which is a fact about ownership rather than a claim about the mission.
"""

from dataclasses import dataclass
from threading import Lock
from typing import Callable, Dict, Optional, Tuple

from lxml import etree

from . import ltl, promela, verify


@dataclass(frozen=True)
class GateResult:
    """Accepted, or rejected with a reason the planner can act on.

    ``verified`` is not the same as ``accepted``: a plan with no formula
    available is accepted and *not* verified. Callers report the two separately
    so a run where the checker never fired cannot be mistaken for a clean one.
    """

    accepted: bool
    reason: str = ""
    verified: bool = False


class LtlGate:
    """Checks a candidate plan against the formula its mission text yields.

    ``generate`` is injected so tests can supply a formula without a model call;
    it defaults to the real LTL agent.
    """

    def __init__(
        self,
        xsd_path: str,
        generate: Optional[Callable[[str], ltl.Formula]] = None,
    ):
        self._xsd_path = xsd_path
        self._generate = generate or (lambda text: ltl.generate(text))
        self._lock = Lock()
        # Mission text -> formula. Replans overwhelmingly keep the text
        # identical, so this makes the LLM call once per mission rather than
        # once per candidate, which matters when the planner is in a retry loop.
        self._cache: Dict[str, ltl.Formula] = {}
        #: Text the coordinator appended when it absorbed a peer's task. The one
        #: sanctioned way the mission text may grow.
        self._appendix: str = ""

    # ------------------------------------------------------------------
    # Coordinator hook
    # ------------------------------------------------------------------

    def allow_mission_text(self, text: str) -> None:
        """Permit ``text`` as the mission text for the next candidate.

        Called by the coordinator when it takes on a task, because absorbing
        work legitimately changes what this robot's mission is. Deliberately not
        reachable from the planner: this is the only door in the immutability
        rule, and it opens from the side that knows about ownership rather than
        the side that writes the plan.
        """
        with self._lock:
            self._appendix = (text or "").strip()

    def warm(self, mission_text: str) -> ltl.Formula:
        """Translate ``mission_text`` ahead of time and cache the result.

        Called when the pristine mission first arrives. The model call is by far
        the slowest thing the gate does, and doing it here means a replan pays
        only for SPIN -- which matters because the coordinator's path into this
        starts inside a lock that the auction and heartbeat timers need.
        """
        return self._formula_for((mission_text or "").strip())

    # ------------------------------------------------------------------
    # The gate
    # ------------------------------------------------------------------

    def evaluate(self, candidate: str, active: Optional[str]) -> GateResult:
        """Check ``candidate`` against the specification its mission text gives.

        ``active`` is the currently committed plan, or None before the first one
        is seen -- in which case there is nothing to have drifted from and the
        immutability check has no opinion.
        """
        try:
            text = _mission_text(candidate)
        except etree.XMLSyntaxError as exc:
            return GateResult(False, f"not well-formed XML: {exc}")

        # 1. The specification may not be rewritten by the thing being checked.
        if active is not None:
            ok, reason = self._text_unchanged(text, active)
            if not ok:
                return GateResult(False, reason)

        if not text:
            return GateResult(True, "no <Mission> text to verify against")

        # 2. The formula. An outage here must not deadlock the mission, but it
        #    also must not be recorded as a verification that passed.
        formula = self._formula_for(text)
        if not formula.ok:
            return GateResult(True, f"unverified: {formula.error}")

        # 3. The model.
        try:
            model = promela.compile_mission(candidate, self._xsd_path)
        except promela.PromelaError as exc:
            return GateResult(False, f"cannot verify plan: {exc}")

        # 4. Does the plan even talk about what the mission asked for?
        covered, gap = promela.coverage_gap(formula.text, model)
        if not covered:
            return GateResult(False, f"LTL coverage: {gap}")

        # 5. Does it satisfy it?
        if not verify.spin_available():
            return GateResult(True, "unverified: spin not installed")

        verdict = verify.check(model.source, formula.text)
        if not verdict.conclusive:
            # A checker that did not answer has not cleared the plan. Accepting
            # keeps the robot moving; recording it as verified would not.
            return GateResult(True, f"unverified: {verdict.error}")
        if not verdict.holds:
            return GateResult(
                False,
                f"LTL violation of `{formula.text}`:\n{verdict.counterexample}",
                verified=True,
            )
        return GateResult(True, f"verified against `{formula.text}`", verified=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _text_unchanged(self, text: str, active: str) -> Tuple[bool, str]:
        try:
            previous = _mission_text(active)
        except etree.XMLSyntaxError:
            return True, ""  # nothing trustworthy to compare against
        if text == previous:
            return True, ""
        with self._lock:
            appendix = self._appendix
        if appendix and text == _extend(previous, appendix):
            return True, ""
        return False, (
            "<Mission> may not be rewritten by a replan: expected "
            f"{previous!r}, got {text!r}"
        )

    def _formula_for(self, text: str) -> ltl.Formula:
        with self._lock:
            hit = self._cache.get(text)
        if hit is not None:
            return hit
        # Outside the lock: this is a model call taking seconds, and the lock is
        # the one a concurrent candidate needs.
        result = self._generate(text)
        with self._lock:
            # Only successes are cached. A transient model outage must not pin
            # the mission as unverifiable for the rest of the run.
            if result.ok:
                self._cache[text] = result
        return result


def _mission_text(mission_xml: str) -> str:
    doc = etree.fromstring(mission_xml.encode("utf-8"))
    return (doc.findtext("Mission") or "").strip()


def _extend(text: str, appendix: str) -> str:
    """How an absorbed task is added to the mission text.

    One place, because the coordinator builds the candidate and the arbiter
    checks it, and if the two disagree by so much as a separator every transfer
    is rejected as a rewrite.
    """
    return f"{text}; {appendix}" if text else appendix

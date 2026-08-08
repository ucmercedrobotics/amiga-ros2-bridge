"""Mission text -> LTL formula. The specification half of the verification pair.

Extracted from ``ltl_gen_node`` so the arbiter can reach it without a service
call. The arbiter needs a formula *inside* the gate that decides whether a
candidate plan is publishable; going out over a service to get one would make
the gate depend on another node being up, and would run a blocking call from a
subscription callback. Both nodes now share this, so there is still exactly one
place that knows how to turn English into a formula.

**This never sees the behaviour tree.** It is given the mission's ``<Mission>``
text and nothing else. ``promela.py`` compiles the tree without ever seeing the
formula. Neither half can be bent to fit the other, which is what makes their
agreement worth something -- and it is a property that has to be maintained
deliberately, because passing the plan in "for context" would look helpful and
would quietly reduce the check to a tautology.

No ROS. The LLM and the prompt template, so a caller can be tested with the
model stubbed out.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from ..runtime import llm, prompts

# A reply has to contain at least one of these to count as a formula rather than
# prose. Bare "!" is deliberately excluded — it matches ordinary exclamations
# ("Sure! Here is...") and a pure negation isn't a temporal property anyway.
LTL_TOKENS = ("[]", "<>", " U ", "X ", "&&", "||", "->", "<->")

# Punctuation that means the model answered in sentences. "." alone is allowed
# mid-token so Promela struct access (robot.at_tree_1) keeps working once the
# typedefs land; only sentence-shaped usage is rejected.
PROSE_MARKERS = (". ", ".\n", ", ", "?", ";")


@dataclass(frozen=True)
class Formula:
    """A formula, or the reason there isn't one.

    Never both, and the caller must not treat a missing formula as permission to
    proceed unchecked -- see the arbiter, which records it as ``unverified``
    rather than folding it into a pass.
    """

    text: Optional[str]
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.text is not None


def system_prompt(ap_vocabulary: Sequence[str] = ()) -> str:
    """Render the LTL agent's system prompt.

    ``ap_vocabulary`` pins the propositions the model may use. Left empty -- the
    default, and what both callers pass -- the model derives them from the
    mission text using the naming scheme the template mandates. Populating it
    from the *plan* would hand the model the answer and collapse the check;
    it exists for a future where the vocabulary comes from somewhere the plan
    does not.
    """
    return prompts.render("ltl_gen/system.j2", ap_vocabulary=list(ap_vocabulary))


def generate(mission_text: str, prompt: Optional[str] = None) -> Formula:
    """Translate ``mission_text`` into an LTL formula. Never raises."""
    mission_text = (mission_text or "").strip()
    if not mission_text:
        return Formula(None, "empty mission")

    try:
        reply = llm.complete(prompt or system_prompt(), mission_text)
    except Exception as exc:  # noqa: BLE001 - a model outage is not our crash
        return Formula(None, f"LLM call failed: {exc}")

    formula = clean_formula(reply)
    ok, reason = looks_like_ltl(formula)
    if not ok:
        return Formula(None, f"{reason}; model returned: {reply[:200]}")
    return Formula(formula)


def clean_formula(reply: str) -> str:
    """Reduce a model reply to a single-line candidate formula."""
    text = llm.strip_code_fence(reply)
    # Some models still wrap the answer as `ltl name { ... }` despite the prompt
    if text.startswith("ltl") and "{" in text and text.rstrip().endswith("}"):
        text = text[text.index("{") + 1 : text.rstrip().rindex("}")]
    # Collapse to one line — SPIN accepts multi-line, but a single line keeps the
    # /mission/ltl payload and the logs readable.
    return " ".join(text.split()).strip()


def looks_like_ltl(formula: str) -> Tuple[bool, str]:
    """Cheap smoke test — is this a formula or did the model answer in prose?

    Returns (ok, reason). Not a substitute for SPIN, which is the actual parser;
    it only exists to keep obvious garbage off /mission/ltl and out of a model
    file, where it would surface as a confusing syntax error instead of "the
    model answered in English".
    """
    if not formula:
        return False, "empty formula"
    if formula.count("(") != formula.count(")"):
        return False, "unbalanced parentheses"
    if not any(token in f" {formula} " for token in LTL_TOKENS):
        return False, "no LTL or boolean operator found"
    if formula.endswith(".") or any(m in formula for m in PROSE_MARKERS):
        return False, "reply reads as prose, not a bare formula"
    return True, ""

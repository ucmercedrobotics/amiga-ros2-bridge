#!/usr/bin/env python3
"""The places where open-ended reasoning belongs, and the stubs holding them.

Everything else in this package is a decision procedure: given these bids, this
one wins; given this heartbeat gap, that peer is gone. Three questions are not
like that, and pretending otherwise is how you get a coordinator full of
heuristics nobody can justify:

    interpret_anomaly(context) -> ActionSchema
        the mission node says it cannot do something. Re-delegate it? Take on
        different work? Give up? That is a judgement over unstructured context.
        Answered in production by the triage agent in ``amiga_ros2_agents``,
        which reads the behaviour-tree fault, the /rosout window around it and
        the world state, and returns one of the typed actions.

    interpret_note(context) -> BidRevision
        another robot attached a sentence to a task it is offering. Does it
        change what we should bid? The only inbound path where somebody else's
        text steers a decision, which is why its union is the narrowest one
        here -- it can make us bid worse or not bid, and nothing else.

    replan_and_verify(delta) -> ReplanResult
        our mission just changed. Is the new one still coherent? That is the
        arbiter's gate: well-formed XML, schema validity, the ontology's
        preconditions, and objective/viability -- the same checks a local
        replan goes through.

The stubs below are what the acceptance tests run against. The point of the
split is that the contract-net state machine is pinned against trivial stubs
and the real implementations swap in without touching one line of it: a stub
that accepts everything exercises the same code path a verifier that accepts
this particular mission does.

What the stubs must not do is soften the interface. ``interpret_anomaly``
returns a closed union of four typed actions (see schema.py) and never free
text, because a state machine that parses sentences has no enumerable set of
behaviours to test. That constraint is load-bearing now, while it is still easy
to keep.

No ROS, no radio, no I/O and no model in this file. The adapter that reaches
the real triage agent lives in node.py, where the ROS dependency belongs.
"""

from dataclasses import dataclass
from typing import Optional, Protocol, Sequence, runtime_checkable

from amiga_ros2_comms.codec import has_capabilities

from ..vocabulary.model import MissionDelta
from ..vocabulary.schema import (
    ActionSchema,
    AnomalyContext,
    BidRevision,
    DropTask,
    KeepBid,
    LocalDisposition,
    NoteContext,
    ReDelegate,
    validate_action,
    validate_revision,
)


# ==========================================================================
# Anomaly interpretation
# ==========================================================================


@runtime_checkable
class AnomalyInterpreter(Protocol):
    """Turns "the mission node is stuck" into one of four typed decisions."""

    def interpret_anomaly(self, context: AnomalyContext) -> ActionSchema:
        """Decide what to do about the anomaly described by ``context``.

        Must return a member of the ActionSchema union. Raising is permitted
        and is handled by the coordinator as "no interpretation available" --
        the task stays ours, which is the safe direction to fail in.
        """


class AlwaysReDelegate:
    """The default stub: every anomaly means shed the task to the fleet.

    The right default for a stub because re-delegation is the *longest* path
    through this layer -- announce, collect, arbitrate, grant, confirm, replan
    -- so a system wired up with this one exercises the most machinery per
    anomaly. It is not a sensible policy and is not meant to be.
    """

    def __init__(self, fallback: LocalDisposition = LocalDisposition.HOLD):
        self.fallback = fallback
        self.calls: "list[AnomalyContext]" = []

    def interpret_anomaly(self, context: AnomalyContext) -> ActionSchema:
        self.calls.append(context)
        if context.task is None:
            raise ValueError(
                "AlwaysReDelegate cannot interpret an anomaly with no task "
                "attached; script a different action for this scenario"
            )
        return ReDelegate(
            task=context.task,
            reason_code=context.reason_code,
            fallback=self.fallback,
        )


class CapabilityAwareInterpreter:
    """The deterministic arm of the reasoning ablation: no model, one policy.

    ``interpret_anomaly`` has exactly one production implementation that is
    not a test stub -- ``TriageClient``, which asks a language model to read
    the fault, the log window and the world state. Measuring what that model
    buys means comparing it against the best decision procedure reachable
    *without* one, on the same anomalies, and ``AlwaysReDelegate`` cannot be
    that baseline: its own docstring says plainly that it is not a sensible
    policy, chosen instead because re-delegation is the longest path through
    this layer and therefore the stub that exercises the most machinery. This
    class is what you would actually ship if there were no model on the other
    end of the call -- not a strawman built to lose the comparison.

    The policy reads exactly the structured fields ``AnomalyContext`` carries
    and nothing else: is there a peer in ``context.peers`` that is idle right
    now, and does its ``cap_mask`` cover every bit in
    ``context.task.required_capabilities``? If such a peer exists, the task is
    at least plausibly somebody else's to finish, so the answer is
    ``ReDelegate`` -- let the auction find out for certain, with ``HOLD`` as
    the fallback if the announce window closes with no bid. If no live peer is
    both idle and fully capable, re-delegating would spend an announce window
    to learn what this policy can already see from the registry, so the answer
    is ``DropTask`` with disposition ``HOLD``: keep the task, do not announce
    it, try again once the fleet looks different. Either way ``reason_code``
    passes through from the context unchanged, because this policy is not
    diagnosing the fault, only routing around it. A ``context.task`` of
    ``None`` is refused the same way ``AlwaysReDelegate`` refuses it: raising
    is "no interpretation available," which the coordinator treats as leaving
    the task exactly where it was, the safe direction to fail in.

    What this policy can never do, by construction, is return ``AddTask``. Not
    because the case is rare, but because nothing it is allowed to read could
    ever justify it. Synthesizing new work means having observed something
    about the world that no field in ``AnomalyContext`` carries: a task
    carries a capability mask and a location, a peer record carries a
    capability mask, a battery level and whether it is idle, and none of that
    is the sentence "there is a diseased tree in the next row that is in
    nobody's mission." That sentence lives in the /rosout window, the
    behaviour-tree fault and the world state -- exactly the evidence fields
    ``TriageClient`` leaves empty on the wire and the triage agent reads for
    itself. A policy that only sees typed fields has no representation for it
    to synthesize an ``AddTask`` from, so it cannot express one, at any
    confidence, ever.

    That gap has two concrete failure modes worth naming rather than leaving
    implicit, because an ablation is supposed to report them rather than paper
    over them with a policy tuned to dodge its own weak spot:

    (a) A fault that is fleet-wide rather than local -- the row itself is
    impassable, the target no longer exists, whatever it is -- looks, from
    this policy's vantage point, identical to a fault that is merely local. If
    a capable peer happens to be idle, this policy re-delegates regardless,
    because idleness and capability are all it is allowed to check. The
    auction runs, the peer drives over, and the peer discovers the same
    fleet-wide fact this robot already hit. That is an announce window and a
    peer's trip spent to relearn something structured state already implied
    and no typed field carried.

    (b) A fault that reveals new work -- the obstacle blocking this task is
    itself a task -- cannot become an ``AddTask`` no matter how obviously a
    model reading the same fault would produce one, because, again, nothing
    in ``AnomalyContext`` names the new work.

    Both failures are failures of *information*, not of engineering: they are
    not cases this policy reasons about incorrectly, they are cases where the
    only evidence that would produce the right answer never reached it. That
    is the honest way to say what a deterministic baseline costs, and it is
    the reason this class exists in the ablation at all -- not to prove a
    model is unnecessary, and not to prove it is, but to make what it actually
    contributes measurable against the strongest policy reachable without one.
    """

    def __init__(self):
        self.calls: "list[AnomalyContext]" = []

    def interpret_anomaly(self, context: AnomalyContext) -> ActionSchema:
        self.calls.append(context)
        if context.task is None:
            raise ValueError(
                "CapabilityAwareInterpreter cannot interpret an anomaly with "
                "no task attached; script a different action for this scenario"
            )
        task = context.task
        for peer in context.peers:
            if peer.idle and has_capabilities(
                peer.cap_mask, task.required_capabilities
            ):
                return ReDelegate(
                    task=task,
                    reason_code=context.reason_code,
                    fallback=LocalDisposition.HOLD,
                    note="",
                )
        return DropTask(
            task=task,
            disposition=LocalDisposition.HOLD,
            reason_code=context.reason_code,
        )


class ScriptedInterpreter:
    """Returns a prepared sequence of actions, one per anomaly.

    How the acceptance tests choose which branch of the state machine to
    exercise. The last action repeats once the script runs out, so a test that
    only cares about the first decision does not have to count anomalies.
    """

    def __init__(self, actions: Sequence[ActionSchema]):
        if not actions:
            raise ValueError("ScriptedInterpreter needs at least one action")
        self._actions = list(actions)
        self.calls: "list[AnomalyContext]" = []

    def interpret_anomaly(self, context: AnomalyContext) -> ActionSchema:
        index = min(len(self.calls), len(self._actions) - 1)
        self.calls.append(context)
        return validate_action(self._actions[index])


# ==========================================================================
# Note interpretation
# ==========================================================================


@runtime_checkable
class NoteInterpreter(Protocol):
    """Turns another robot's sentence into one of three bid revisions."""

    def interpret_note(self, context: NoteContext) -> BidRevision:
        """Decide what ``context.text`` means for a bid we were going to make.

        Must return a member of the BidRevision union. Raising is permitted and
        is handled by the coordinator as "no interpretation available" -- the
        bid goes out exactly as fitness decided, which is the safe direction:
        an uninterpretable note leaves the auction working the way it worked
        before notes existed.

        Called *off* the coordinator's lock, like interpret_anomaly and for the
        same reason: this is a model call taking seconds, and the lock it would
        otherwise hold is the one ``tick`` and ``on_message`` need.
        """


class IgnoreNotes:
    """The default: notes are recorded and change nothing.

    The right default because it is the behaviour of the system as it was
    before notes existed. A robot with no interpreter wired should still hear
    notes, still count them, and still bid on mechanics alone -- rather than
    silently ignore a whole message type or, worse, act on text nobody
    interpreted.
    """

    def __init__(self):
        self.calls: "list[NoteContext]" = []

    def interpret_note(self, context: NoteContext) -> BidRevision:
        self.calls.append(context)
        return KeepBid(reason="no note interpreter configured")


class ScriptedNoteInterpreter:
    """Returns a prepared sequence of revisions, one per note.

    The note counterpart of ScriptedInterpreter, with the same last-one-repeats
    behaviour, so a test that only cares about the first revision does not have
    to count notes.
    """

    def __init__(self, revisions: Sequence[BidRevision]):
        if not revisions:
            raise ValueError("ScriptedNoteInterpreter needs at least one revision")
        self._revisions = list(revisions)
        self.calls: "list[NoteContext]" = []

    def interpret_note(self, context: NoteContext) -> BidRevision:
        index = min(len(self.calls), len(self._revisions) - 1)
        self.calls.append(context)
        return validate_revision(self._revisions[index])


# ==========================================================================
# Replan and verify
# ==========================================================================


@dataclass(frozen=True)
class ReplanResult:
    """Accepted, with the verified mission, or rejected, with a reason.

    Binary for the same reason ``Outcome`` in the reliability layer is binary:
    the caller has exactly two things it can do about it. Why a verifier
    rejected a mission is invaluable in a log and is not a branch the state
    machine should be taking.
    """

    accepted: bool
    #: Whatever the replanner considers the mission. Opaque to this layer.
    mission: Optional[object] = None
    reason: str = ""

    @property
    def rejected(self) -> bool:
        return not self.accepted


@runtime_checkable
class MissionReplanner(Protocol):
    """Re-plans and verifies our own mission after a committed change."""

    def replan_and_verify(self, delta: MissionDelta) -> ReplanResult:
        """Fold ``delta`` into the mission and verify what comes out.

        Called *after* the change is committed, never before: the deltas this
        layer produces are facts (a peer acknowledged owning that task) rather
        than proposals, and a verifier cannot un-transfer a task.
        """


class AcceptEverything:
    """The pass-through stub. Records what it was asked, accepts all of it.

    Deliberately not a no-op: it keeps the deltas, so the acceptance tests can
    assert that replan-and-verify was called once, with the right change, at
    the right point in the sequence -- which is the whole of what this layer
    owes the real verifier.
    """

    def __init__(self):
        self.deltas: "list[MissionDelta]" = []

    def replan_and_verify(self, delta: MissionDelta) -> ReplanResult:
        self.deltas.append(delta)
        return ReplanResult(accepted=True, mission=None, reason="stub: accepted")

    @property
    def calls(self) -> int:
        return len(self.deltas)


class RejectEverything:
    """The pass-through stub's evil twin, for the rejection path.

    Not an acceptance-test requirement, but the rejection branch exists in the
    coordinator and untested branches rot. Rejecting a delta that absorbed a
    task turns that task back into an anomaly, which is the behaviour worth
    pinning before the real arbiter starts producing rejections for reasons
    nobody anticipated.
    """

    def __init__(self, reason: str = "stub: rejected"):
        self.reason = reason
        self.deltas: "list[MissionDelta]" = []

    def replan_and_verify(self, delta: MissionDelta) -> ReplanResult:
        self.deltas.append(delta)
        return ReplanResult(accepted=False, mission=None, reason=self.reason)

    @property
    def calls(self) -> int:
        return len(self.deltas)

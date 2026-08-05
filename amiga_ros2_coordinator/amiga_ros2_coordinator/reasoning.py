#!/usr/bin/env python3
"""The two places where open-ended reasoning belongs, and the stubs holding them.

Everything else in this package is a decision procedure: given these bids, this
one wins; given this heartbeat gap, that peer is gone. Two questions are not
like that, and pretending otherwise is how you get a coordinator full of
heuristics nobody can justify:

    interpret_anomaly(context) -> ActionSchema
        the mission node says it cannot do something. Re-delegate it? Take on
        different work? Give up? That is a judgement over unstructured context.
        Answered in production by the triage agent in ``amiga_ros2_agents``,
        which reads the behaviour-tree fault, the /rosout window around it and
        the world state, and returns one of the typed actions.

    replan_and_verify(delta) -> ReplanResult
        our mission just changed. Is the new one still coherent, and does it
        still satisfy the temporal constraints it is supposed to? That is the
        existing replan generation plus LTL verification (SPIN/Spot, Z3 for the
        behaviour tree).

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

from .model import MissionDelta
from .schema import (
    ActionSchema,
    AnomalyContext,
    LocalDisposition,
    ReDelegate,
    validate_action,
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
    pinning before a real LTL backend starts producing rejections for reasons
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

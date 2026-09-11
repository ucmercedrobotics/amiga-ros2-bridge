#!/usr/bin/env python3
"""Tests for the deterministic arm of the reasoning ablation.

``CapabilityAwareInterpreter`` is the strongest ``interpret_anomaly`` policy
reachable without a model: re-delegate if a live peer is both idle and fully
capable, otherwise hold the task locally. No ROS, no radio, no model -- it
reads a plain ``AnomalyContext`` and returns a plain ``ActionSchema`` member,
so these tests exercise the class directly rather than through a node.

What is worth pinning here is not just "the happy path returns the right
type." It is the shape of the policy's blind spot: it can never return
``AddTask``, by construction, because nothing in ``AnomalyContext`` carries
the kind of evidence that would justify one. The "never AddTask" test below is
that claim, checked across a spread of peer lists rather than trusted from the
docstring alone. The detail-insensitivity test pins the other half of the same
point: this policy reads structured fields only, so the free-text explanation
of the fault -- exactly the material a model would read -- may say anything at
all without moving the decision by one bit.
"""

import itertools
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    TASK_NONE,
    Capability,
    ReasonCode,
    Target,
    cap_mask,
)

from amiga_ros2_coordinator import (  # noqa: E402
    AddTask,
    AnomalyContext,
    CapabilityAwareInterpreter,
    DropTask,
    LocalDisposition,
    PeerRecord,
    ReDelegate,
    Task,
    validate_action,
)

#: What sampling one tree needs: drive to it, then operate the arm. Matches
#: the pairing used throughout test_coordinator.py, so a reader who knows that
#: suite already knows why this is the realistic default rather than a bare
#: single bit.
SAMPLING = cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF)
#: A robot that can drive but cannot operate the arm -- capable of part of
#: SAMPLING and not the whole of it.
DRIVING = cap_mask(Capability.MOVE_TO_TREE_ID)

TASK_ID = 7


def make_task(**overrides) -> Task:
    fields = dict(
        task_id=TASK_ID,
        required_capabilities=SAMPLING,
        location=Target.tree(60),
        priority=50,
    )
    fields.update(overrides)
    return Task(**fields)


def make_peer(
    robot_id: int, *, idle: bool, capabilities: int, **overrides
) -> PeerRecord:
    fields = dict(
        robot_id=robot_id,
        cap_mask=capabilities,
        current_task=TASK_NONE if idle else 999,
        battery=80,
        last_seen=0.0,
    )
    fields.update(overrides)
    return PeerRecord(**fields)


def make_context(task, peers=(), detail: str = "") -> AnomalyContext:
    return AnomalyContext(
        task=task,
        detail=detail,
        reason_code=ReasonCode.TASK_FAILED,
        peers=tuple(peers),
        battery=100,
        at=100.0,
    )


# ==========================================================================
# Re-delegates when the fleet can actually take the task
# ==========================================================================


def test_re_delegates_when_a_capable_idle_peer_exists():
    task = make_task()
    peer = make_peer(2, idle=True, capabilities=SAMPLING)
    action = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=[peer])
    )

    assert action == ReDelegate(
        task=task,
        reason_code=ReasonCode.TASK_FAILED,
        fallback=LocalDisposition.HOLD,
        note="",
    )


def test_re_delegates_to_the_first_matching_peer_when_several_qualify():
    task = make_task()
    busy_but_capable = make_peer(2, idle=False, capabilities=SAMPLING)
    idle_and_capable = make_peer(3, idle=True, capabilities=SAMPLING)
    action = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=[busy_but_capable, idle_and_capable])
    )

    assert isinstance(action, ReDelegate)
    assert action.task == task


# ==========================================================================
# Drops the task, locally, when re-delegating could not possibly help
# ==========================================================================


def test_drops_the_task_when_peers_exist_but_none_are_idle():
    task = make_task()
    peers = [
        make_peer(2, idle=False, capabilities=SAMPLING),
        make_peer(3, idle=False, capabilities=SAMPLING),
    ]
    action = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=peers)
    )

    assert action == DropTask(
        task=task,
        disposition=LocalDisposition.HOLD,
        reason_code=ReasonCode.TASK_FAILED,
    )


def test_drops_the_task_when_idle_peers_exist_but_none_have_the_capability():
    task = make_task()
    peers = [
        make_peer(2, idle=True, capabilities=DRIVING),
        make_peer(3, idle=True, capabilities=0),
    ]
    action = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=peers)
    )

    assert action == DropTask(
        task=task,
        disposition=LocalDisposition.HOLD,
        reason_code=ReasonCode.TASK_FAILED,
    )


def test_having_part_of_a_task_is_not_having_the_task():
    """A peer capable of driving but not sampling does not qualify, idle or not.

    Same point test_coordinator.py makes about the bidder side: a task is a
    behaviour-tree subtree, not a single action, and this policy has to check
    every bit of the mask rather than any one of them.
    """
    task = make_task()
    peer = make_peer(2, idle=True, capabilities=DRIVING)
    action = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=[peer])
    )

    assert isinstance(action, DropTask)


def test_drops_the_task_when_there_are_no_peers_at_all():
    task = make_task()
    action = CapabilityAwareInterpreter().interpret_anomaly(make_context(task))

    assert isinstance(action, DropTask)


# ==========================================================================
# The one action it cannot express, checked rather than assumed
# ==========================================================================


def test_never_returns_add_task_across_a_range_of_peer_lists():
    """No combination of peers -- idle or not, capable or not -- ever
    produces AddTask.

    Not a property this policy could get right by accident: AddTask requires
    a Task to attach to new work, and nothing this policy is given -- a task,
    a reason code, a peer list -- describes work that is not the task already
    in hand. Checked across every combination of the peer states below rather
    than asserted from the docstring, because a policy that read a stray field
    and returned AddTask for it would only show up here.
    """
    task = make_task()
    peer_states = [
        make_peer(2, idle=True, capabilities=SAMPLING),
        make_peer(3, idle=False, capabilities=SAMPLING),
        make_peer(4, idle=True, capabilities=DRIVING),
        make_peer(5, idle=False, capabilities=0),
        make_peer(6, idle=True, capabilities=0),
    ]
    for size in range(0, len(peer_states) + 1):
        for combo in itertools.combinations(peer_states, size):
            for peers in itertools.permutations(combo):
                action = CapabilityAwareInterpreter().interpret_anomaly(
                    make_context(task, peers=peers)
                )
                assert not isinstance(action, AddTask)


# ==========================================================================
# The safe direction to fail in
# ==========================================================================


def test_raises_when_the_anomaly_has_no_task_attached():
    context = make_context(task=None)
    with pytest.raises(ValueError):
        CapabilityAwareInterpreter().interpret_anomaly(context)


# ==========================================================================
# Whatever it returns is a member of the closed schema
# ==========================================================================


def test_output_always_passes_validate_action():
    task = make_task()
    scenarios = [
        [],
        [make_peer(2, idle=True, capabilities=SAMPLING)],
        [make_peer(2, idle=False, capabilities=SAMPLING)],
        [make_peer(2, idle=True, capabilities=DRIVING)],
    ]
    for peers in scenarios:
        action = CapabilityAwareInterpreter().interpret_anomaly(
            make_context(task, peers=peers)
        )
        assert validate_action(action) is action


# ==========================================================================
# Structured state only -- the free-text explanation never moves the needle
# ==========================================================================


def test_insensitive_to_detail_text():
    """The same structured state produces the same action regardless of what
    the mission node wrote in ``detail``.

    ``detail`` is exactly the kind of unstructured evidence a model would
    read and this policy is defined not to: reading STRUCTURED state only,
    never unstructured evidence, per the class's own contract. A long,
    dramatic fault description must be worth precisely as much as an empty
    string here, because reading it at all would be the deterministic arm
    quietly becoming a worse version of the model it is being compared
    against.
    """
    task = make_task()
    peers = [make_peer(2, idle=True, capabilities=SAMPLING)]
    absurd_detail = (
        "CRITICAL: sensor mast sheared off, robot on fire, tree 60 is "
        "actually a lamppost, mission control has been notified twice, "
        "please advise immediately " * 20
    )

    plain = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=peers, detail="")
    )
    dramatic = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=peers, detail=absurd_detail)
    )

    assert plain == dramatic

    # Same check on the drop branch, so the claim holds on both outcomes and
    # not only the one the first assertion happened to exercise.
    no_peers_plain = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=(), detail="")
    )
    no_peers_dramatic = CapabilityAwareInterpreter().interpret_anomaly(
        make_context(task, peers=(), detail=absurd_detail)
    )

    assert no_peers_plain == no_peers_dramatic

"""Tests for the coordinator's half of the *other* reasoning seam.

``test_triage_client.py`` guards the same boundary for our own faults. This one
matters more, and the difference is where the input comes from: a note is text
another robot broadcast, over a radio whose source address is self-asserted and
carries no MAC. Anyone in range can put a sentence in front of the model behind
this client.

The whole defence is that the answer is closed at three values, all of which
only adjust a bid this robot was already going to make. So what is tested here
is exactly that: every valid revision maps to the right member of the union,
and everything else raises rather than being coerced into something plausible.
A fourth behaviour reachable from a remote sentence is the failure this file
exists to prevent.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    COST_MAX,
    Capability,
    Target,
    TargetKind,
    cap_mask,
)
from amiga_ros2_coordinator.adapters.note_client import (  # noqa: E402
    NoteClient,
    NoteRefused,
)
from amiga_ros2_coordinator.vocabulary.model import Task  # noqa: E402
from amiga_ros2_coordinator.vocabulary.schema import (  # noqa: E402
    KeepBid,
    NoteContext,
    ReviseBid,
    WithdrawBid,
)

TASK = Task(
    task_id=42,
    required_capabilities=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
    location=Target.tree(60),
    priority=100,
)

NOTE = "north end of row 7 is flooded; approach from the south, ~4 min extra"


class Response:
    """Stands in for InterpretNote.Response, which needs a colcon build.

    Same reasoning as the triage tests' stand-in: the decoder reads attributes
    and never the ROS type, so building the interface package to test string
    handling would only slow the suite down.
    """

    def __init__(self, **fields):
        self.ok = True
        self.error = ""
        self.revision = ""
        self.cost_delta = 0
        self.reason = ""
        self.rationale = ""
        self.model = "test-model"
        for name, value in fields.items():
            setattr(self, name, value)


class Request:
    """Stands in for InterpretNote.Request. Attributes are assigned, not read."""

    def __init__(self):
        self.note_text = ""
        self.announcer_id = 0
        self.task_id = 0
        self.required_capabilities = 0
        self.target_kind = 0
        self.target_a = 0
        self.target_b = 0
        self.priority = 0
        self.our_cost = 0
        self.our_eta_s = 0
        self.our_feasible = False
        self.our_bid_unknown = False


class SrvType:
    Request = Request


def _bare_client() -> NoteClient:
    client = NoteClient.__new__(NoteClient)
    client._srv_type = SrvType
    return client


def decode(response) -> object:
    return NoteClient._decode(_bare_client(), response)


def context(**overrides) -> NoteContext:
    fields = {
        "text": NOTE,
        "task_id": 42,
        "src": 3,
        "task": TASK,
        "cost": 80,
        "eta_s": 120.0,
        "feasible": True,
        "at": 1000.0,
    }
    fields.update(overrides)
    return NoteContext(**fields)


# ==========================================================================
# Valid revisions
# ==========================================================================


def test_keep_becomes_a_keep_carrying_its_reason():
    revision = decode(Response(revision="keep", reason="nothing new in it"))
    assert isinstance(revision, KeepBid)
    assert revision.reason == "nothing new in it"


def test_revise_carries_a_signed_delta():
    revision = decode(Response(revision="revise", cost_delta=45, reason="detour"))
    assert isinstance(revision, ReviseBid)
    assert revision.cost_delta == 45


def test_a_negative_delta_survives_because_a_note_can_say_it_is_easier():
    revision = decode(Response(revision="revise", cost_delta=-30))
    assert isinstance(revision, ReviseBid)
    assert revision.cost_delta == -30


def test_withdraw_becomes_a_withdrawal_and_not_a_large_revision():
    revision = decode(Response(revision="withdraw", reason="needs a tool we lack"))
    assert isinstance(revision, WithdrawBid)


def test_the_revision_name_is_matched_case_insensitively_and_trimmed():
    assert isinstance(decode(Response(revision="  KEEP ")), KeepBid)


def test_the_rationale_stands_in_when_no_short_reason_was_given():
    revision = decode(Response(revision="keep", rationale="the row is already dry"))
    assert revision.reason == "the row is already dry"


# ==========================================================================
# Everything else
# ==========================================================================


def test_an_unknown_revision_is_refused_rather_than_guessed_at():
    with pytest.raises(NoteRefused):
        decode(Response(revision="reconsider"))


def test_an_empty_revision_is_refused():
    with pytest.raises(NoteRefused):
        decode(Response(revision=""))


def test_a_revision_that_looks_like_an_action_is_still_refused():
    # The two schemas are separate on purpose: a note may not do what an
    # anomaly interpretation may do, and a model that confuses the two must
    # not be met halfway.
    for word in ("re_delegate", "drop_task", "add_task"):
        with pytest.raises(NoteRefused):
            decode(Response(revision=word))


def test_a_non_numeric_delta_is_refused_rather_than_defaulted_to_zero():
    # Defaulting would turn "the work is harder, by this much" into "keep",
    # silently, and the counters would record a revision that did nothing.
    with pytest.raises(NoteRefused):
        decode(Response(revision="revise", cost_delta="a lot"))


def test_an_enormous_delta_is_clamped_rather_than_refused():
    # A number outside the range is still a claim the coordinator can act on;
    # unlike a missing one, it says which direction the work moved.
    revision = decode(Response(revision="revise", cost_delta=10_000))
    assert revision.cost_delta == COST_MAX
    assert (
        decode(Response(revision="revise", cost_delta=-10_000)).cost_delta == -COST_MAX
    )


# ==========================================================================
# What the agent is told
# ==========================================================================


def test_the_request_carries_the_offer_and_what_we_were_going_to_bid():
    request = NoteClient._request(_bare_client(), context())
    assert request.note_text == NOTE
    assert request.announcer_id == 3
    assert request.task_id == 42
    assert request.target_kind == int(TargetKind.TREE)
    assert request.target_a == 60
    assert request.our_cost == 80
    assert request.our_feasible is True
    assert request.our_bid_unknown is False


def test_an_unassessed_note_says_so_instead_of_claiming_a_zero_cost():
    # The intended ordering is that a note arrives *before* the announcement it
    # annotates. A model told "cost: 0, feasible: false" would read that as a
    # free job we declined, which is the opposite of what it means.
    request = NoteClient._request(
        _bare_client(), context(task=None, cost=None, eta_s=None, feasible=None)
    )
    assert request.our_bid_unknown is True
    assert request.our_cost == 0
    assert request.required_capabilities == 0
    assert request.target_kind == int(TargetKind.NONE)

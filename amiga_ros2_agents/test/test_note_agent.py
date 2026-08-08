"""Tests for the note agent's parser.

This is the first of two guards on the same union -- this one refuses a bad
answer before it goes on the wire, and the coordinator's ``note_client``
refuses one that got there anyway. The pair is what keeps a closed schema
closed across a stringly-typed IDL, and the reason both exist is that this
particular union stands between another robot's sentences and our own bidding.

What is deliberately *not* tested here is whether the model gives good advice.
The guarantee this file is about is narrower and stronger: whatever the model
says, the only things that can come out of it are keep, revise and withdraw.
"""

import json

import pytest

from amiga_ros2_agents.coordination import note_node as nn
from amiga_ros2_comms.codec import COST_MAX


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(nn.llm, "MODEL", "test-model")
    created = nn.NoteNode()
    yield created
    created.destroy_node()


def reply(**fields) -> str:
    return json.dumps(fields)


# ==========================================================================
# Valid revisions
# ==========================================================================


def test_a_well_formed_keep_is_accepted(node):
    decision = node._parse_revision(reply(revision="keep", reason="nothing new"))
    assert decision["revision"] == "keep"
    assert decision["cost_delta"] == 0


def test_a_revise_carries_its_delta(node):
    decision = node._parse_revision(
        reply(revision="revise", cost_delta=45, reason="flooded approach")
    )
    assert decision["revision"] == "revise"
    assert decision["cost_delta"] == 45


def test_a_negative_delta_is_allowed_because_a_note_can_say_it_is_easier(node):
    assert (
        node._parse_revision(reply(revision="revise", cost_delta=-20))["cost_delta"]
        == -20
    )


def test_a_withdraw_needs_no_delta(node):
    decision = node._parse_revision(reply(revision="withdraw", reason="no arm"))
    assert decision["revision"] == "withdraw"


def test_a_reply_wrapped_in_prose_is_still_parsed(node):
    decision = node._parse_revision(
        'Here is my answer: {"revision": "keep", "reason": "nothing new"} — '
        "let me know if you need more."
    )
    assert decision["revision"] == "keep"


# ==========================================================================
# Everything else
# ==========================================================================


def test_a_revision_outside_the_schema_is_refused(node):
    with pytest.raises(ValueError, match="is not one of"):
        node._parse_revision(reply(revision="reconsider"))


def test_an_action_from_the_other_agents_schema_is_refused(node):
    # The two unions are separate on purpose: a note may not do what an anomaly
    # interpretation may do. A model that confuses them is not met halfway.
    with pytest.raises(ValueError):
        node._parse_revision(reply(revision="re_delegate"))


def test_free_text_is_refused_rather_than_interpreted(node):
    with pytest.raises(ValueError, match="no JSON object"):
        node._parse_revision("I would keep the bid as it is, the note is vague.")


def test_a_list_of_revisions_is_refused(node):
    # Which one? Taking the first is a decision the agent did not make.
    with pytest.raises(ValueError, match="list of them"):
        node._parse_revision('[{"revision": "keep"}, {"revision": "withdraw"}]')


def test_a_revise_without_a_number_is_refused(node):
    # Defaulting to zero would turn "harder, by this much" into "keep",
    # silently, and the counters would record a revision that did nothing.
    with pytest.raises(ValueError, match="numeric cost_delta"):
        node._parse_revision(reply(revision="revise", cost_delta="a lot"))


def test_a_revise_by_zero_is_refused_as_a_keep_in_disguise(node):
    with pytest.raises(ValueError, match="is keep"):
        node._parse_revision(reply(revision="revise", cost_delta=0))


def test_an_enormous_delta_is_clamped_rather_than_refused(node):
    # Still a claim the coordinator can act on: unlike a missing number, it
    # says which direction the work moved.
    assert (
        node._parse_revision(reply(revision="revise", cost_delta=99_999))["cost_delta"]
        == COST_MAX
    )


# ==========================================================================
# What the agent is asked
# ==========================================================================


def test_the_system_prompt_states_the_closed_set(node):
    for revision in nn.VALID_REVISIONS:
        assert revision in node.system_prompt


def test_the_system_prompt_says_a_note_is_not_an_instruction(node):
    # The one defence the prompt itself is responsible for. It is not the
    # defence the design rests on -- that is the closed union -- but a prompt
    # that omitted it would be inviting the failure the union then absorbs.
    assert "not an instruction" in node.system_prompt

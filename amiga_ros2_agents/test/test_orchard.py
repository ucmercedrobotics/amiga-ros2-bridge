"""Tree -> aisle, against the orchard the robots are actually given.

The map comes out of ``examples/sample_20_64.bin`` -- the second frame of
the mission binary, which is the same document ``tcp_demux_node`` republishes on
``/orchard/tree_info_json``. Testing against a fixture shaped to pass would
prove nothing: the claim is that ``aisle_of`` and the mission planner agree
about which aisle a tree is in, and the planner has already committed its
answer in the plan sitting next to this file.
"""

import json
import os
import struct
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_agents.mission import orchard  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
EXAMPLES = os.path.join(REPO, "amiga_ros2_behavior_tree", "examples")

#: What the three planner-written plans say, read off their MoveToAisleHead ids.
PLANNER_ANSWER = {20: 2, 64: 4, 22: 2, 66: 4, 24: 2, 68: 4}


def frames(name: str):
    with open(os.path.join(EXAMPLES, name), "rb") as handle:
        data = handle.read()
    offset, out = 0, []
    while offset + 4 <= len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        offset += 4
        out.append(data[offset : offset + length])
        offset += length
    return out


@pytest.fixture(scope="module")
def payload():
    return frames("sample_20_64.bin")[1].decode()


@pytest.fixture(scope="module")
def orchard_map(payload):
    return orchard.parse(payload)


def test_the_map_agrees_with_the_mission_planner(orchard_map):
    """The agreement that makes the ontology's prerequisite real.

    Two independent parties: this function reads ``traversal_axis`` and each
    tree's ``col``; the planner is a language model that read the same document
    and wrote ``<MoveToAisleHead id="6"/>`` ahead of tree 60. If they disagreed,
    the ontology would compute a prerequisite the plans never satisfy, and every
    objective would look like it was missing its aisle move.
    """
    for tree, aisle in PLANNER_ANSWER.items():
        assert orchard_map.aisle_of(tree) == aisle, f"tree {tree}"


def test_an_attribute_string_answers_the_same_as_an_int(orchard_map):
    """Every caller is holding an XML attribute, which is a string."""
    assert orchard_map.aisle_of("20") == orchard_map.aisle_of(20) == 2


def test_the_outer_row_is_reached_from_one_side_only(payload, orchard_map):
    """One more row/column than there are aisles, so the last one has a lane on
    a single side and open field on the other.

    It used to answer ``None`` here, on the reasoning that the outer index is
    not itself an aisle -- true, and the wrong conclusion drawn from it. A row
    is not unreachable because it is on the edge; it is reachable from the one
    lane it still has. ``None`` cost the planner the only route there and cost
    ``synthesize`` the aisle move for any task at one of those trees.

    The field that carries the position is whichever one ``traversal_axis``
    names -- this fixture travels by row, so it is ``row``, not ``col``.
    """
    data = json.loads(payload)
    field = orchard.AXIS_FIELD[data["traversal_axis"]]
    positions = {tree[field] for tree in data["trees"]}
    aisles = {int(key) for key in data["aisle_to_entrance_indices"]}
    outer = max(positions)
    assert outer not in aisles, "the fixture no longer has an outer row/column"

    stranded = [t["tree_index"] for t in data["trees"] if t[field] == outer]
    assert stranded
    for tree in stranded:
        sides = orchard_map.aisles_of(tree)
        assert len(sides) == 1, f"tree {tree} is on the edge, so one lane only"
        # And whatever that lane is, it has to be one a plan may name.
        assert sides[0] in orchard_map.aisles()
        assert orchard_map.aisle_of(tree) == sides[0]


def test_the_two_accessors_cannot_disagree(orchard_map):
    """``aisle_of`` is the first of ``aisles_of``, everywhere, by construction.

    Regression: these were two stored maps built by different rules. They
    matched for the 108 trees in the middle rows and diverged for the 36 in the
    outermost two -- row 1 answering aisle 1, which is off the block, and row 8
    answering None. Deriving one from the other is what makes that class of
    disagreement unrepresentable rather than merely absent.
    """
    for tree in range(1, 145):
        sides = orchard_map.aisles_of(tree)
        assert orchard_map.aisle_of(tree) == (sides[0] if sides else None)


def test_a_row_traversal_reads_the_other_index():
    """``traversal_axis`` is what picks the field, not a convention here."""
    document = {
        "traversal_axis": "row",
        "trees": [{"tree_index": 7, "row": 3, "col": 9}],
        "aisle_to_entrance_indices": {"3": [5, 6]},
    }
    assert orchard.parse(json.dumps(document)).aisle_of(7) == 3


@pytest.mark.parametrize(
    "payload_text",
    ["", "not json", "[]", '{"trees": []}', '{"traversal_axis": "diagonal"}'],
)
def test_an_unreadable_orchard_is_empty_rather_than_an_exception(payload_text):
    """It arrives in a subscription callback; raising there takes the node down.

    Empty means every aisle is unknown, which every caller already handles --
    an expectation it cannot check, never a plan it refuses.
    """
    result = orchard.parse(payload_text)
    assert not result
    assert result.aisle_of(60) is None


def test_an_unplaceable_tree_is_left_out_rather_than_defaulted(orchard_map):
    assert orchard_map.aisle_of(99999) is None
    assert orchard_map.aisle_of(None) is None
    assert orchard_map.aisle_of("not a tree") is None


def test_aisles_are_the_ones_with_a_tree_in_them(orchard_map):
    """Every aisle a real objective could name, for reciting the valid set."""
    assert {2, 4} <= orchard_map.aisles()


def test_a_row_is_reachable_from_the_lane_on_either_side(orchard_map):
    """The fact a blocked lane needs answering with.

    A row of trees sits between two lanes and carries a waypoint on each side,
    so row r is worked from aisle r or aisle r+1. Without this the orchard can
    only name one lane, and a robot whose lane is blocked by something that
    will not move has nothing left to say but "drop it" -- while the trees are
    perfectly reachable from the other side.
    """
    assert orchard_map.aisles_of(20) == (2, 3)
    assert orchard_map.aisles_of(58) == (4, 5)
    assert orchard_map.aisles_of("20") == orchard_map.aisles_of(20)


def test_the_outer_rows_have_a_lane_on_one_side_only(orchard_map):
    """Open field on the other, so the choice this adds genuinely is not there
    for them -- and saying so beats offering a lane that is off the block."""
    assert orchard_map.aisles_of(1) == (2,)
    assert orchard_map.aisles_of(144) == (8,)


def test_the_recited_aisles_are_the_ones_that_reach_something(orchard_map):
    """The prompt states both this set and each tree's lanes, and tells the
    model an aisle outside it reaches nothing. The two have to agree, and they
    did not: reading it off `aisle_of` listed the lowest row's own index, which
    is a lane off the edge of the block, and omitted the lane the highest row
    is worked from."""
    reachable = {a for t in range(1, 145) for a in orchard_map.aisles_of(t)}
    assert orchard_map.aisles() == reachable
    assert 1 not in orchard_map.aisles()
    assert 8 in orchard_map.aisles()


def test_sides_falls_back_to_the_single_aisle_without_a_layout():
    """A document with no rows to read leaves aisles_of empty, and a prompt
    built on it is no worse off than one built on facts_for_trees."""
    document = {
        "traversal_axis": "row",
        "aisle_to_entrance_indices": {"3": [1, 2]},
        "trees": [{"tree_index": 7, "row": 3, "col": 1}],
    }
    parsed = orchard.parse(json.dumps(document))
    assert orchard.sides_for_trees(parsed, [7])[7] == (3, 4)


def test_facts_for_trees_is_scoped_to_what_was_asked(orchard_map):
    """A replan prompt should pay for the trees it has, not the whole orchard."""
    facts = orchard.facts_for_trees(orchard_map, ["20", "64"])
    assert facts == {20: 2, 64: 4}


def test_facts_for_trees_reports_unknown_rather_than_dropping_it(orchard_map):
    """A tree the orchard has no entry for is still worth telling the model
    about -- as an honest unknown, not a silent omission it can't tell apart
    from "this tree doesn't exist"."""
    facts = orchard.facts_for_trees(orchard_map, ["99999"])
    assert facts == {99999: None}

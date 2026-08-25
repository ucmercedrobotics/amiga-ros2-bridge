"""Which aisle a tree is in. The one geometric fact the ontology needs.

``ontology`` knows that reaching a tree expects being in that tree's aisle. It
cannot know *which* aisle without looking at the orchard, and the orchard is a
JSON document published on ``/orchard/tree_info_json`` by ``tcp_demux_node`` --
the same document the mission planner is handed, so both sides answer from the
same map.

**The rule.** ``traversal_axis`` says which of a tree's two indices names the
lane the robot drives down: ``"column"`` means aisles run between columns, so
tree 60 at ``col: 6`` is reached from aisle 6. That is not an invention here --
the mission planner independently writes ``<MoveToAisleHead id="6"/>`` ahead of
``<MoveToTreeID id="60"/>`` in the plans this fleet is being tested with, and
the same holds for every tree in them (10 -> 10, 12 -> 12, 62 -> 8, 14 -> 14,
64 -> 10). Reading it from the document rather than restating it is what keeps
the two from drifting apart.

**Where it stops.** A real orchard has one fewer aisle than it has columns --
144 trees in 18 columns, 17 aisles, 34 entrances -- so the last column's index
is not an aisle at all. ``aisle_of`` returns ``None`` there rather than a
plausible-looking number. Every caller treats an unknown aisle as an
*expectation it cannot check*, never as a violation, so an honest ``None``
costs a suggestion; a wrong integer would send a robot down the wrong lane.

``nav_ports.parse_orchard`` in the coordinator reads the same document for a
different question -- where things are, in metres, for bidding. The two are
deliberately not shared: the packages do not depend on each other, and merging
them would create a dependency to save fifteen lines.

No ROS. json and nothing else, so this is testable against a real dump.
"""

import json
from typing import Dict, Optional

#: ``traversal_axis`` -> the per-tree field that names its aisle. A value this
#: table does not have means the document is describing a layout this code has
#: never seen, and every aisle is then unknown rather than guessed.
AXIS_FIELD = {"column": "col", "row": "row"}


class Orchard:
    """A tree -> aisle map, and nothing else.

    Deliberately not a general orchard model. The ontology asks exactly one
    question of the world, and a class that answered five would have to be kept
    in agreement with ``nav_ports`` about the other four.
    """

    def __init__(
        self,
        aisle_by_tree: Optional[Dict[int, int]] = None,
        aisles_by_tree: Optional[Dict[int, tuple]] = None,
    ):
        self._aisle_by_tree = dict(aisle_by_tree or {})
        self._aisles_by_tree = {k: tuple(v) for k, v in (aisles_by_tree or {}).items()}

    def __bool__(self) -> bool:
        """False when nothing is known, so ``orchard or None`` reads naturally."""
        return bool(self._aisle_by_tree)

    def __len__(self) -> int:
        return len(self._aisle_by_tree)

    def aisle_of(self, tree) -> Optional[int]:
        """The aisle tree ``tree`` is reached from, or None if unknown.

        Accepts the string form too, because that is what an XML attribute is
        and every caller here is holding one.
        """
        try:
            return self._aisle_by_tree.get(int(tree))
        except (TypeError, ValueError):
            return None

    def aisles_of(self, tree) -> tuple:
        """Every aisle tree ``tree`` can be worked from, lowest first.

        A row sits *between* two lanes, so a tree has a row waypoint on each
        side and either lane will reach it. Row r is worked from aisle r on one
        side and aisle r+1 on the other -- checked against the waypoint
        geometry for all 144 trees of the demo orchard, not assumed: each
        waypoint lies on the entrance-to-entrance line of exactly one lane, and
        those are the two.

        The outermost rows have a lane on one side and open field on the other,
        so they come back with one.

        ``aisle_of`` still answers with a single aisle, because a prerequisite
        chain only needs to *suggest* a route. This is for the case where the
        suggestion is unusable -- a lane blocked by something that will not
        move -- and the tree is still perfectly reachable from the other side.
        """
        try:
            return self._aisles_by_tree.get(int(tree), ())
        except (TypeError, ValueError):
            return ()

    def aisles(self) -> set:
        """Every aisle a plan may name, i.e. every one that reaches some tree.

        For reciting the valid set to a planner, not for answering "is this
        aisle real" -- a real orchard's outermost column has no aisle at all,
        and that absence is exactly what makes this set worth stating rather
        than assumed.

        Taken from the two-sided map rather than from ``aisle_of``, because the
        two disagree at the edges and only one of them is about what a plan can
        say: the lowest row's own index names a lane that is off the block, and
        the highest row is reached from a lane no row is numbered after.
        """
        if self._aisles_by_tree:
            return {a for sides in self._aisles_by_tree.values() for a in sides}
        return set(self._aisle_by_tree.values())


def parse(payload) -> Orchard:
    """An ``Orchard`` from the orchard JSON, empty if it cannot be read.

    Empty rather than raising: an unreadable orchard makes prerequisites
    unknown, which every caller already handles, whereas an exception raised in
    a subscription callback takes the node down with it.
    """
    try:
        data = json.loads(payload) if isinstance(payload, (str, bytes)) else payload
    except (TypeError, ValueError):
        return Orchard()
    if not isinstance(data, dict):
        return Orchard()

    field = AXIS_FIELD.get(str(data.get("traversal_axis") or "").strip().lower())
    if field is None:
        return Orchard()

    declared = _declared_aisles(data)
    aisle_by_tree: Dict[int, int] = {}
    for tree in data.get("trees") or []:
        if not isinstance(tree, dict):
            continue
        index, aisle = tree.get("tree_index"), tree.get(field)
        if not isinstance(index, int) or not isinstance(aisle, int):
            continue
        # An index the orchard does not list as an aisle is the outer column,
        # which no MoveToAisleHead can name; see the module docstring.
        if declared and aisle not in declared:
            continue
        aisle_by_tree[index] = aisle

    return Orchard(aisle_by_tree, _aisles_by_row(data, field, declared))


def _aisles_by_row(data: dict, field: str, declared: set) -> Dict[int, tuple]:
    """``tree_index -> (aisle, ...)``: both lanes that reach each tree.

    ``MoveToAisleHead id=N`` reaches the lane the document indexes as N-1, so
    the highest aisle a plan can name is one past the last lane the document
    declares, and the lowest is 2. Row r's two sides are lanes r-1 and r, which
    is aisles r and r+1 once that offset is applied; clipping to the range is
    what leaves the field-edge rows with a single side.
    """
    if not declared:
        return {}
    highest = max(declared) + 1

    out: Dict[int, tuple] = {}
    for tree in data.get("trees") or []:
        if not isinstance(tree, dict):
            continue
        index, row = tree.get("tree_index"), tree.get(field)
        if not isinstance(index, int) or not isinstance(row, int):
            continue
        sides = tuple(a for a in (row, row + 1) if 2 <= a <= highest)
        if sides:
            out[index] = sides
    return out


def sides_for_trees(orchard: Orchard, tree_ids) -> Dict[int, tuple]:
    """``tree_id -> every aisle it can be worked from``, for the trees asked.

    The companion to ``facts_for_trees`` for a planner allowed to choose a
    side. Falls back to the single default aisle where the document carried no
    layout, so a prompt built on this is never emptier than one built on that.
    """
    out: Dict[int, tuple] = {}
    for t in tree_ids:
        try:
            key = int(t)
        except (TypeError, ValueError):
            continue
        sides = orchard.aisles_of(key)
        if not sides:
            only = orchard.aisle_of(key)
            sides = () if only is None else (only,)
        out[key] = sides
    return out


def facts_for_trees(orchard: Orchard, tree_ids) -> Dict[int, Optional[int]]:
    """``tree_id -> aisle_of(tree_id)`` for exactly the trees asked about.

    Scoped rather than the whole map so a prompt pays only for the trees an
    active mission actually references -- a replanner and, later, the LTL
    arbiter both want this same slice, which is why it lives here rather than
    inlined in either caller.
    """
    out: Dict[int, Optional[int]] = {}
    for t in tree_ids:
        try:
            out[int(t)] = orchard.aisle_of(t)
        except (TypeError, ValueError):
            continue
    return out


def _declared_aisles(data: dict) -> set:
    """The aisles the document says exist, as ints. Empty if it does not say."""
    mapping = data.get("aisle_to_entrance_indices")
    if not isinstance(mapping, dict):
        return set()
    out = set()
    for raw in mapping:
        try:
            out.add(int(raw))
        except (TypeError, ValueError):
            continue
    return out

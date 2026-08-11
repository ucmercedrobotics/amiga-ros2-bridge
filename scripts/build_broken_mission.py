#!/usr/bin/env python3
"""Build a real, correctly-framed TCP mission payload that one robot cannot finish.

Takes an existing example .bin (real XML + real orchard JSON, two
4-byte-length-prefixed frames) and removes ONE tree from the orchard frame,
leaving the XML frame byte-identical. The plan still asks for tree 64; this
robot's copy of the orchard no longer knows where tree 64 is.

Why the orchard and not the plan
--------------------------------

The obvious way to break a mission is to point a ``MoveToTreeID`` at a tree id
that does not exist. That produces a real fault -- and a task nobody can ever
take. The whole point of shedding work is that a peer picks it up, and a peer
asked to go to tree 9999 resolves it against its own orchard, finds nothing,
and bids infeasible. Every robot does. The auction runs, collects a full set of
"I cannot", and hands the task straight back. The pipeline looks like it is
working right up until the moment it is supposed to produce a winner.

So the fault has to be *local to one robot* while the work stays real:

    the plan            asks for tree 64, which is a real tree
    this robot          has no entry for tree 64, so it cannot go there
    every other robot   has the untouched orchard, so it can

``/orchard/get_tree_info`` filters its cached tree list server-side
(orchard_management.cpp), so it legitimately finds no match, and
amiga_navigation's waypoint_follower.py aborts that goal for real
("GetTreeInfo returned empty result") on the real ``/bt/status_change`` topic
bt_runner's own FaultReporter publishes. Nothing is injected on a topic or a
service. Only one robot's orchard frame differs from what shipped, by one tree.

Aisles are untouched: ``aisle_entrances`` and ``aisle_to_entrance_indices`` are
separate from ``trees``, so this robot can still drive its rows and run the rest
of its mission. It is one objective it cannot reach, not a crippled robot.

Usage:
    build_broken_mission.py <source.bin> <tree_index> <out.bin>
"""

import json
import struct
import sys


def frames(data: bytes):
    """The two length-prefixed frames of a mission payload."""
    (xml_len,) = struct.unpack(">I", data[0:4])
    xml_bytes = data[4 : 4 + xml_len]
    json_start = 4 + xml_len
    (json_len,) = struct.unpack(">I", data[json_start : json_start + 4])
    json_bytes = data[json_start + 4 : json_start + 4 + json_len]
    return xml_bytes, json_bytes


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 1
    src_path, raw_index, out_path = sys.argv[1:4]

    try:
        tree_index = int(raw_index)
    except ValueError:
        print(f"tree_index must be an integer, got {raw_index!r}", file=sys.stderr)
        return 1

    with open(src_path, "rb") as handle:
        xml_bytes, json_bytes = frames(handle.read())

    # The plan has to actually want this tree, and want it as an objective
    # rather than as a transit waypoint. Checked rather than assumed: a demo
    # whose injected fault silently does not fire is worse than one that
    # refuses to start, because the first looks like a broken pipeline.
    needle = f'id="{tree_index}"'.encode()
    if needle not in xml_bytes:
        print(
            f"{src_path}'s plan never asks for tree {tree_index}, so hiding it "
            f"would produce no fault at all",
            file=sys.stderr,
        )
        return 1

    orchard = json.loads(json_bytes)
    trees = orchard.get("trees") or []
    kept = [tree for tree in trees if tree.get("tree_index") != tree_index]
    if len(kept) == len(trees):
        print(
            f"tree {tree_index} is not in {src_path}'s orchard data, so this "
            f"robot could not have reached it anyway",
            file=sys.stderr,
        )
        return 1
    orchard["trees"] = kept

    new_json = json.dumps(orchard).encode()
    with open(out_path, "wb") as handle:
        handle.write(struct.pack(">I", len(xml_bytes)))
        handle.write(xml_bytes)
        handle.write(struct.pack(">I", len(new_json)))
        handle.write(new_json)

    print(
        f"hid tree {tree_index} from {out_path}: {len(kept)} of {len(trees)} trees "
        f"left, plan unchanged"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

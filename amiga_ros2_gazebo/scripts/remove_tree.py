#!/usr/bin/env python3
"""Delete one tree from the running world -- the map still says it is there.

For the missing-tree scenario: a robot is sent to sample a tree that used to
be there. Nothing in software lies about it. `GetTreeInfo` (orchard_management)
answers from its own map, not from Gazebo, and this script does not touch that
map -- it only removes the model entity, so the service keeps reporting the
tree exactly where it always did. The robot drives to a real position, the
lidar sees real bare ground, and the fault is the same one a stale orchard map
would produce on hardware: the record and the world disagree.

Deliberately not a world-file edit. Removing `tree_20` from orchard_nbv.sdf
would make `GetTreeInfo` blind too -- if it is generated from the same
source -- and would also change the world for every other demo that boots off
it. This removes an already-spawned entity from the live simulation instead,
the way `spawn_truck.py --remove` cleans up a truck: gone for this run, back
the moment the world restarts from its file.

    ros2 run amiga_ros2_gazebo remove_tree.py --tree 20
    ros2 run amiga_ros2_gazebo remove_tree.py --tree 20 --dry-run

`--tree` takes a mission tree id -- the same number a `MoveToTreeID id="..."`
uses -- not a Gazebo model name, and the two are NOT the same number.
`generate_orchard_world.py` names models `tree_{n:02d}` by raw row-major
position (row 0 first); the mission side numbers rows the opposite way
(row 0 = the far row, working inward -- see spawn_truck.py's `tree_pose`,
which every mission's tree ids are already generated to agree with). Row r
in one scheme is row `ROWS-1-r` in the other, so mission tree 26 is model
`tree_116`, not `tree_26` -- confirmed against the live world's actual
`<pose>` entries, not assumed from either file's own comments alone.
Column is the one axis that IS shared as-is between both schemes.

    ros2 run amiga_ros2_gazebo remove_tree.py --tree 20
    ros2 run amiga_ros2_gazebo remove_tree.py --tree 20 --dry-run
"""

import argparse
import subprocess
import sys

ROWS = 8
TREES_PER_ROW = 18
WORLD = "orchard_nbv"


def model_name(mission_tree_id: int) -> str:
    n = ROWS * TREES_PER_ROW
    if not 1 <= mission_tree_id <= n:
        raise SystemExit(f"tree {mission_tree_id} is outside 1..{n}")
    row, col = divmod(mission_tree_id - 1, TREES_PER_ROW)
    model_row = (ROWS - 1) - row
    model_index = model_row * TREES_PER_ROW + col + 1
    return f"tree_{model_index:02d}"


def remove(world: str, name: str) -> int:
    req = f'name: "{name}", type: MODEL'
    return subprocess.run(
        [
            "ign",
            "service",
            "-s",
            f"/world/{world}/remove",
            "--reqtype",
            "ignition.msgs.Entity",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            req,
        ]
    ).returncode


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--tree", type=int, required=True, help="index the mission's MoveToTreeID uses"
    )
    ap.add_argument("--world", default=WORLD)
    ap.add_argument(
        "--dry-run", action="store_true", help="print the model name, remove nothing"
    )
    args = ap.parse_args()

    name = model_name(args.tree)
    print(
        f"removing '{name}' from world '{args.world}' -- tree {args.tree} is now bare soil"
    )
    if args.dry_run:
        return 0

    import shutil

    if not shutil.which("ign"):
        raise SystemExit("no `ign` on PATH -- run this inside the sim container")

    return remove(args.world, name)


if __name__ == "__main__":
    sys.exit(main())

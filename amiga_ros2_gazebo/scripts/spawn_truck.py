#!/usr/bin/env python3
"""Park a pickup across an aisle, so the lane itself is impassable.

The companion to `spawn_person.py`, and deliberately a different kind of
obstruction. A person stands on the row *waypoint*: the goal pose cannot be
placed, `NavigateToPose` aborts, and the lane on either side of them is still
wide open. A truck laid across the lane blocks the lane -- there is no route
past it to anywhere further down the aisle, whatever the goal.

That difference is the point of having both. One is an obstruction a robot
could reasonably wait out or step around; the other is not, and the camera is
the only thing that can tell a planner which it is looking at.

The geometry that makes it a block rather than a nuisance: the Fuel mesh is
2.47 x 5.66 x 1.87 m, aisles are 9 m wide, and rows run along x -- a row is 18
trees at one y, spread along x -- so the robot drives down x. At yaw 0 the
truck lies lengthwise in the lane and leaves 6.5 m to pass. Turned across it
spans 5.66 m and leaves ~1.7 m either side, which is inside the robot's own
width plus its 2.0 m inflation radius. Hence the default yaw.

    ros2 run amiga_ros2_gazebo spawn_truck.py --tree 20
    ros2 run amiga_ros2_gazebo spawn_truck.py --tree 20 --dry-run
    ros2 run amiga_ros2_gazebo spawn_truck.py --x 9.0 --y 27.0
    ros2 run amiga_ros2_gazebo spawn_truck.py --remove

The mesh is Open Robotics' "Pickup" from Fuel, fetched into the local cache on
first use. Unlike the person it already declares itself static and its mesh
URIs are *relative*, so it is spawned from where it was cached rather than from
a staged copy -- a copy in /tmp cannot resolve `meshes/pickup.dae`.
"""

import argparse
import math
import shutil
import subprocess
import sys
from pathlib import Path

#: Must match `generate_orchard_world.py`, and `spawn_person.py`, which carries
#: the same block with the warning about which end the rows are counted from.
#: Payload row 1 is the *northern* one, y = +31.5; read a placement off the SDF
#: model names instead and you get it mirrored about y = 0.
ROWS = 8
TREES_PER_ROW = 18
DX = 4.0  # tree spacing along a row
DY = 9.0  # row spacing, i.e. aisle width
X0 = 5.0  # x of the first tree column

#: Where a lane starts, for `--entrance`. Robots come in from the low-x end --
#: every aisle head resolves to the world datum longitude, x ~ 0 -- so half a
#: tree-spacing ahead of the first trunk is inside the lane mouth and past the
#: aisle head. Blocking here stops the robot entering the lane at all, rather
#: than stopping it partway along.
ENTRANCE_X = X0 - DX / 2

WORLD = "orchard_nbv"
MODEL_NAME = "blocking_truck"

#: Across the lane rather than along it. See the module docstring.
YAW_ACROSS = math.pi / 2

FUEL_URL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pickup"
#: Fortress keeps its cache under ~/.ignition, Garden and later under ~/.gz.
CACHE_GLOBS = (
    ".ignition/fuel/*/openrobotics/models/pickup/*/model.sdf",
    ".gz/fuel/*/openrobotics/models/pickup/*/model.sdf",
)


def tree_pose(index: int) -> tuple[float, float, int]:
    """World (x, y) of a tree and the row it sits in, from its 1-based index."""
    if not 1 <= index <= ROWS * TREES_PER_ROW:
        raise SystemExit(f"tree {index} is outside 1..{ROWS * TREES_PER_ROW}")
    row, col = divmod(index - 1, TREES_PER_ROW)
    x = X0 + col * DX
    y = ((ROWS - 1) / 2 - row) * DY  # row 1 is the northernmost, y = +31.5
    return x, y, row + 1


def lane_of(index: int, entrance: bool = False) -> tuple[float, float]:
    """The point in the lane a robot working this tree has to drive through.

    The lane is the one `spawn_person.py` picks, and for the same reason: it is
    the lane the robot actually travels, which is not always the one the tree's
    aisle id would suggest.

    Alongside the tree by default, which blocks that tree and everything past
    it. With ``entrance``, at the mouth of the same lane instead, which blocks
    the lane outright -- the robot reaches the aisle head and can go no further,
    so nothing in the row is reachable rather than merely most of it.
    """
    x, y, row = tree_pose(index)
    if row == 1:
        raise SystemExit(
            f"tree {index} is in row 1, whose approach side is the field edge; "
            f"pick a tree in rows 2-{ROWS}"
        )
    return (ENTRANCE_X if entrance else x), y + DY / 2


def find_model() -> Path:
    """Path to the cached Fuel model, downloading it if it is not there yet."""
    for _ in range(2):
        for pattern in CACHE_GLOBS:
            found = sorted(Path.home().glob(pattern))
            if found:
                return found[-1]  # highest version
        print(f"fetching {FUEL_URL} into the Fuel cache ...", file=sys.stderr)
        if subprocess.run(["ign", "fuel", "download", "-u", FUEL_URL]).returncode:
            raise SystemExit("could not download the model; is there a network?")
    raise SystemExit("downloaded the model but cannot find it in the cache")


def remove(world: str, name: str) -> int:
    req = f'name: "{name}", type: MODEL'
    return subprocess.run(
        ["ign", "service", "-s", f"/world/{world}/remove",
         "--reqtype", "ignition.msgs.Entity",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", "2000", "--req", req]
    ).returncode


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group()
    where.add_argument("--tree", type=int, help="block the lane this tree is worked from")
    where.add_argument("--x", type=float, help="explicit world x (needs --y)")
    ap.add_argument("--y", type=float, help="explicit world y")
    ap.add_argument("--entrance", action="store_true",
                    help="park it at the mouth of the lane rather than beside "
                         "the tree, so the row cannot be entered at all")
    ap.add_argument("--yaw", type=float, default=YAW_ACROSS,
                    help="facing, radians. Defaults to across the lane, which "
                         "is what makes it a block; 0.0 lies it lengthwise and "
                         "leaves room to pass.")
    ap.add_argument("--name", default=MODEL_NAME, help="model name in the world")
    ap.add_argument("--world", default=WORLD)
    ap.add_argument("--remove", action="store_true", help="delete it again")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the pose and the geometry behind it, spawn nothing")
    args = ap.parse_args()

    if args.remove:
        return remove(args.world, args.name)

    if args.tree is not None:
        x, y = lane_of(args.tree, args.entrance)
        tx, ty, row = tree_pose(args.tree)
        where = "the mouth of the lane" if args.entrance else "the lane beside it"
        print(f"tree {args.tree} (row {row}) is at ({tx:.1f}, {ty:.1f}); blocking "
              f"{where} at ({x:.1f}, {y:.1f}).")
    elif args.x is not None and args.y is not None:
        x, y = args.x, args.y
    else:
        ap.error("give --tree N, or both --x and --y")

    across = abs(math.sin(args.yaw))
    print(f"placing '{args.name}' at x={x:.2f} y={y:.2f} yaw={args.yaw:.2f} "
          f"in world '{args.world}' — spanning {5.66 * across:.1f} m of the "
          f"{DY:.0f} m lane")
    if args.dry_run:
        return 0

    if not shutil.which("ign"):
        raise SystemExit("no `ign` on PATH -- run this inside the sim container")

    # Straight from the cache: the model is already static, and its mesh URIs
    # are relative to its own directory.
    return subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", args.world, "-file", str(find_model()), "-name", args.name,
         "-x", str(x), "-y", str(y), "-z", "0.0", "-Y", str(args.yaw)]
    ).returncode


if __name__ == "__main__":
    sys.exit(main())

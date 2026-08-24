#!/usr/bin/env python3
"""Put a standing person in the sim, on the spot where a robot means to stop.

`MoveToTreeID approach_tree="true"` is two phases: Nav2 drives to a *row
waypoint* on the aisle centreline, half a row-spacing out from the trunk, and
then `NavigateViaLidar` covers that last stretch. Only the first phase can fail
on an obstacle. The second picks the closest lidar return near the tree's
bearing in the 0.5-1.5 m height band and drives to it, so a person standing
between the waypoint and the trunk is read *as* the tree: the robot stops short
and reports success. A person standing on the waypoint itself is inside the
footprint's inscribed radius, the planner cannot place the goal, and
NavigateToPose aborts for real -- which is the fault the triage agent and the
VLM exist to explain.

So the default placement is the row waypoint, not the aisle head and not the
approach corridor:

    ros2 run amiga_ros2_gazebo spawn_person.py --tree 20
    ros2 run amiga_ros2_gazebo spawn_person.py --tree 20 --dry-run
    ros2 run amiga_ros2_gazebo spawn_person.py --x 9.0 --y -18.0
    ros2 run amiga_ros2_gazebo spawn_person.py --remove

The mesh is Open Robotics' "Standing person" from Fuel (~28 MB, too big to
vendor). It is fetched into the local Fuel cache on first use and reused after
that; the copy spawned here is forced static so the robot cannot shoulder it
aside mid-demo.
"""

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

#: Must match `generate_orchard_world.py`'s defaults.
#:
#: Careful with the row axis. The generator lays SDF `tree_01` at the *south*
#: end (y = -31.5) and counts north, but the mission payloads number their rows
#: from the *north* end: payload row 1 converts to y = +31.5, and payload row r
#: is the model named tree_((8-r)*18 + col). Everything below is in payload
#: numbering, because that is what a mission's `id` attribute means and so what
#: the robot actually drives to. Read a placement off the SDF model names and
#: you get it mirrored about y = 0.
ROWS = 8
TREES_PER_ROW = 18
DX = 4.0  # tree spacing along a row
DY = 9.0  # row spacing, i.e. aisle width
X0 = 5.0  # x of the first tree column

WORLD = "orchard_nbv"
MODEL_NAME = "standing_person"

FUEL_URL = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"
#: Fortress keeps its cache under ~/.ignition, Garden and later under ~/.gz.
CACHE_GLOBS = (
    ".ignition/fuel/*/openrobotics/models/standing person/*/model.sdf",
    ".gz/fuel/*/openrobotics/models/standing person/*/model.sdf",
)


def tree_pose(index: int) -> tuple[float, float, int]:
    """World (x, y) of a tree and the row it sits in, from its 1-based index."""
    if not 1 <= index <= ROWS * TREES_PER_ROW:
        raise SystemExit(f"tree {index} is outside 1..{ROWS * TREES_PER_ROW}")
    row, col = divmod(index - 1, TREES_PER_ROW)
    x = X0 + col * DX
    y = ((ROWS - 1) / 2 - row) * DY  # row 1 is the northernmost, y = +31.5
    return x, y, row + 1


def waypoint_of(index: int) -> tuple[float, float]:
    """The row waypoint Nav2 aims at before handing over to the lidar approach.

    Interior trees have one of these on each side, and `MoveToTreeID` does not
    consult the aisle id when choosing between them -- it calls
    `select_closest_point(row_waypoints)` and takes whichever is nearer the
    robot at that moment. Observed on these missions, that is the lane on the
    row-1 side of the tree: tree 20 in aisle 2 is approached from y = +27.1,
    not the +18.1 lane its aisle id would suggest.

    So this follows the robot rather than the aisle numbering. Aisle numbering
    would put the person one lane over, where nothing ever drives.
    """
    x, y, row = tree_pose(index)
    if row == 1:
        # Row 1 is the northern edge, so the side the robot would approach
        # from is open ground rather than a lane. Say so instead of placing a
        # person where no robot will pass.
        raise SystemExit(
            f"tree {index} is in row 1, whose approach side is the field edge; "
            f"pick a tree in rows 2-{ROWS}"
        )
    return x, y + DY / 2


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


def staged_sdf(source: Path, name: str) -> str:
    """A copy of the Fuel model.sdf, renamed and pinned static.

    The mesh URIs inside are absolute Fuel URLs that Ignition resolves against
    the same cache, so the copy does not need to live beside the meshes.
    """
    sdf = source.read_text()
    sdf = re.sub(r'<model name="[^"]*">', f'<model name="{name}">\n    <static>true</static>', sdf, count=1)
    tmp = tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False)
    tmp.write(sdf)
    tmp.close()
    return tmp.name


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
    where.add_argument("--tree", type=int, help="block the row waypoint of this tree")
    where.add_argument("--x", type=float, help="explicit world x (needs --y)")
    ap.add_argument("--y", type=float, help="explicit world y")
    ap.add_argument("--yaw", type=float, default=3.14159,
                    help="facing, radians. Robots enter from low x, so the "
                         "default turns the person back down the aisle to face "
                         "one. Flip to 0.0 to face away.")
    ap.add_argument("--name", default=MODEL_NAME, help="model name in the world")
    ap.add_argument("--world", default=WORLD)
    ap.add_argument("--remove", action="store_true", help="delete it again")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the pose and the geometry behind it, spawn nothing")
    args = ap.parse_args()

    if args.remove:
        return remove(args.world, args.name)

    if args.tree is not None:
        x, y = waypoint_of(args.tree)
        tx, ty, row = tree_pose(args.tree)
        print(f"tree {args.tree} (row {row}) is at ({tx:.1f}, {ty:.1f}); "
              f"the robot approaches it from aisle {row - 1}, stopping at "
              f"({x:.1f}, {y:.1f}), {DY / 2:.2f} m out.")
    elif args.x is not None and args.y is not None:
        x, y = args.x, args.y
    else:
        ap.error("give --tree N, or both --x and --y")

    print(f"placing '{args.name}' at x={x:.2f} y={y:.2f} yaw={args.yaw:.2f} "
          f"in world '{args.world}'")
    if args.dry_run:
        return 0

    if not shutil.which("ign"):
        raise SystemExit("no `ign` on PATH -- run this inside the sim container")

    path = staged_sdf(find_model(), args.name)
    return subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", args.world, "-file", path, "-name", args.name,
         "-x", str(x), "-y", str(y), "-z", "0.0", "-Y", str(args.yaw)]
    ).returncode


if __name__ == "__main__":
    sys.exit(main())

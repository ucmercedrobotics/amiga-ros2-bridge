"""The directory layout, enforced.

Splitting this package into ``runtime`` / ``mission`` / ``verification`` /
``replanning`` / ``coordination`` only means something if the boundaries hold.
They are the kind of thing that decays silently: one convenient import from an
agent into another agent, and the next reader has no way to tell the structure
is a lie. Nothing else in the suite would notice, so this file does.

Three rules, and each is a claim made in ``amiga_ros2_agents/__init__.py``:

1. Imports run *down* the layer list, never back up. This is what keeps the
   libraries testable on their own -- the moment ``mission`` reaches into
   ``replanning`` for something, verifying a plan needs a ROS node.
2. No module imports another subpackage's agent. Agents talk over ROS topics
   and services; an agent that imports another agent has quietly become one
   process with two names.
3. ``mission`` and the verification libraries hold no rclpy. That is the whole
   reason ``test_promela.py`` can compile a plan with no robot in the loop.
"""

import ast
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PACKAGE = "amiga_ros2_agents"
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(os.path.dirname(HERE), PACKAGE)

#: Ordered. A subpackage may import from those before it and no others.
LAYERS = ["runtime", "mission", "verification", "replanning", "coordination"]

#: rclpy is legitimate in ``runtime`` (it *is* the ROS wiring) and in the
#: agents. Everywhere else it would mean a library grew a node.
ROS_FREE = ["mission", "verification"]


def _modules():
    """Every module in the package, as (layer, dotted name, path)."""
    for layer in LAYERS:
        directory = os.path.join(ROOT, layer)
        for name in sorted(os.listdir(directory)):
            if name.endswith(".py"):
                stem = name[:-3]
                yield layer, f"{PACKAGE}.{layer}.{stem}", os.path.join(directory, name)


def _is_agent(dotted: str) -> bool:
    return dotted.rsplit(".", 1)[-1].endswith("_node")


def _imports(path: str, dotted: str):
    """Package-internal modules this file imports, as dotted names."""
    parts = dotted.split(".")[:-1]  # the module's own package
    tree = ast.parse(open(path).read())

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith(PACKAGE + "."):
                    yield alias.name
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                base = parts[: len(parts) - (node.level - 1)]
            elif node.module and node.module.startswith(PACKAGE):
                base = []
            else:
                continue
            target = base + (node.module.split(".") if node.module else [])
            if target[:1] != [PACKAGE]:
                continue
            yield ".".join(target)
            for alias in node.names:
                yield ".".join(target + [alias.name])


def _layer_of(dotted: str):
    parts = dotted.split(".")
    return parts[1] if len(parts) > 1 and parts[1] in LAYERS else None


ALL = list(_modules())


def test_every_module_lives_in_a_layer():
    """No stragglers at the package root -- the map would stop being one."""
    loose = [n for n in os.listdir(ROOT) if n.endswith(".py") and n != "__init__.py"]
    assert not loose, f"file the module away or extend LAYERS: {loose}"


@pytest.mark.parametrize("layer,dotted,path", ALL, ids=[m[1] for m in ALL])
def test_imports_run_downward(layer, dotted, path):
    """A subpackage sees only the ones before it in LAYERS."""
    allowed = set(LAYERS[: LAYERS.index(layer) + 1])
    for imported in _imports(path, dotted):
        other = _layer_of(imported)
        if other is None or other in allowed:
            continue
        pytest.fail(
            f"{dotted} imports {imported}: '{layer}' may not depend on "
            f"'{other}', which sits below it in LAYERS"
        )


@pytest.mark.parametrize("layer,dotted,path", ALL, ids=[m[1] for m in ALL])
def test_no_agent_imports_another_packages_agent(layer, dotted, path):
    """Agents coordinate over ROS. An import here is a hidden dependency.

    Within a layer it is still discouraged but not caught -- the planner and
    the arbiter are one loop and share a directory. Across layers there is no
    such excuse.
    """
    for imported in _imports(path, dotted):
        if not _is_agent(imported) or _layer_of(imported) == layer:
            continue
        pytest.fail(f"{dotted} imports the agent {imported}; use a topic or a service")


@pytest.mark.parametrize(
    "layer,dotted,path",
    [m for m in ALL if m[0] in ROS_FREE and not _is_agent(m[1])],
    ids=[m[1] for m in ALL if m[0] in ROS_FREE and not _is_agent(m[1])],
)
def test_libraries_hold_no_ros(layer, dotted, path):
    """``verification`` earns its claim by being runnable without a robot."""
    tree = ast.parse(open(path).read())
    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            names = [node.module]
        for name in names:
            assert not name.split(".")[0] == "rclpy", f"{dotted} imports {name}"

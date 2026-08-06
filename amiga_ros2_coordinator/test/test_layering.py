"""The directory layout, enforced.

This package's central claim about itself is that the contract-net logic has no
ROS in it -- that is what lets ``test_coordinator.py`` run whole auctions,
backoff windows and peer timeouts in microseconds against fakes, with nothing
sleeping and no middleware. Every other test here is downstream of that claim
being true, and not one of them would notice the day it stops being.

So it is checked directly. Three rules:

1. Imports run *down* the layer list, never back up. ``engine`` may use a port;
   a port may not reach into the engine.
2. ``vocabulary``, ``ports`` and ``engine`` import no rclpy, no ROS message
   package and nothing from ``adapters``. An adapter is where blocking and
   middleware are allowed to live, and the value of that boundary is exactly
   its unbrokenness.
3. Nothing imports an entry point. ``nodes`` is where processes are assembled;
   importing one from underneath inverts the assembly.
"""

import ast
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PACKAGE = "amiga_ros2_coordinator"
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(os.path.dirname(HERE), PACKAGE)

#: Ordered. A subpackage may import from those before it and no others.
LAYERS = ["vocabulary", "ports", "engine", "adapters", "nodes"]

#: The half of the package that must run with no robot and no middleware.
PURE = ["vocabulary", "ports", "engine"]

#: Middleware, and the message packages that only exist to cross it. Any of
#: these in PURE means the boundary has moved.
#:
#: ``ament_index_python`` is deliberately *not* here. It resolves an installed
#: package's share directory -- a filesystem lookup with no node, no executor
#: and nothing to block on -- and ``capabilities.default_schema_path`` imports
#: it inside a try/except that returns None when it is absent. That is the
#: property this test exists to protect, not a breach of it.
ROS_ROOTS = {
    "rclpy",
    "rcl_interfaces",
    "std_msgs",
    "sensor_msgs",
    "geometry_msgs",
    "action_msgs",
    "nav2_msgs",
    "amiga_interfaces",
    "amiga_navigation_interfaces",
    "kortex_interfaces",
}


def _modules():
    for layer in LAYERS:
        directory = os.path.join(ROOT, layer)
        for name in sorted(os.listdir(directory)):
            if name.endswith(".py"):
                dotted = f"{PACKAGE}.{layer}.{name[:-3]}"
                yield layer, dotted, os.path.join(directory, name)


ALL = list(_modules())
IDS = [m[1] for m in ALL]


def _internal_imports(path: str, dotted: str):
    """Package-internal modules this file imports, as dotted names."""
    parts = dotted.split(".")[:-1]
    for node in ast.walk(ast.parse(open(path).read())):
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


def _external_roots(path: str):
    """Top-level names this file imports from outside the package."""
    for node in ast.walk(ast.parse(open(path).read())):
        if isinstance(node, ast.Import):
            for alias in node.names:
                yield alias.name.split(".")[0]
        elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
            yield node.module.split(".")[0]


def _layer_of(dotted: str):
    parts = dotted.split(".")
    return parts[1] if len(parts) > 1 and parts[1] in LAYERS else None


def test_every_module_lives_in_a_layer():
    """No stragglers at the package root, or the map stops being one."""
    loose = [n for n in os.listdir(ROOT) if n.endswith(".py") and n != "__init__.py"]
    assert not loose, f"file the module away or extend LAYERS: {loose}"


@pytest.mark.parametrize("layer,dotted,path", ALL, ids=IDS)
def test_imports_run_downward(layer, dotted, path):
    allowed = set(LAYERS[: LAYERS.index(layer) + 1])
    for imported in _internal_imports(path, dotted):
        other = _layer_of(imported)
        if other is None or other in allowed:
            continue
        pytest.fail(
            f"{dotted} imports {imported}: '{layer}' may not depend on "
            f"'{other}', which sits below it in LAYERS"
        )


@pytest.mark.parametrize(
    "layer,dotted,path",
    [m for m in ALL if m[0] in PURE],
    ids=[m[1] for m in ALL if m[0] in PURE],
)
def test_the_engine_has_no_ros_in_it(layer, dotted, path):
    """The claim ``docs/coordinator.md`` opens with, checked.

    A ROS import here does not break anything on the day it lands -- it makes
    the acceptance suite quietly stop being a statement about the shipped code,
    which is a much worse way to find out.
    """
    found = sorted(ROS_ROOTS & set(_external_roots(path)))
    assert not found, f"{dotted} imports {found}; that belongs in an adapter"


@pytest.mark.parametrize("layer,dotted,path", ALL, ids=IDS)
def test_nothing_imports_an_entry_point(layer, dotted, path):
    """``nodes`` assembles the others; being imported back inverts that."""
    for imported in _internal_imports(path, dotted):
        if _layer_of(imported) == "nodes" and layer != "nodes":
            pytest.fail(f"{dotted} imports the entry point {imported}")

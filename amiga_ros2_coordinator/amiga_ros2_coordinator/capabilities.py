#!/usr/bin/env python3
"""What this robot can do, read off the schema that governs what it may be asked.

A robot's capabilities are not a preference. They are the ``ActionGroup`` of the
mission schema its behaviour tree validates against: the mission planner writes
XML against that file, the arbiter rejects a candidate that violates it, and
``bt_runner`` refuses to build a tree from one. An action the schema does not
permit is an action no mission can ever contain, so advertising it to the fleet
is a claim that can never be tested and can only ever be wrong.

That is why this reads the file rather than taking a list. A launch parameter is
a second source of truth about the same fact, and the failure it produces is the
quiet kind -- a robot that never bids on sampling because ``SAMPLE_LEAF`` was
left out of a YAML file, with nothing anywhere reporting an error. The parameter
still exists as an override, for a robot whose hardware is a subset of what its
schema allows (an arm removed for maintenance), but the default is the file.

stdlib ``xml.etree`` on purpose: this runs at node startup, before anything
else, and adding a parse dependency to that path buys nothing.
"""

from typing import Optional
from xml.etree import ElementTree

from amiga_ros2_comms.codec import CAPABILITY_BY_ELEMENT, Capability, cap_mask

#: The XML Schema namespace, which every element in an .xsd carries.
XS = "{http://www.w3.org/2001/XMLSchema}"

#: The group naming the actions a mission may contain. Named here because it is
#: a fact about the schema's structure rather than about XSD in general -- a
#: different platform's schema (Husky, Spot) organises its actions differently
#: and would need its own reader, not a tweak to this one.
ACTION_GROUP = "ActionGroup"


class SchemaError(ValueError):
    """The schema could not be read, or contains nothing this fleet knows.

    Raised rather than defaulted around. A robot that silently advertises no
    capabilities never bids on anything and reports no error, which is the
    hardest kind of misconfiguration to find -- exactly the failure the whole
    point of reading the file was to avoid.
    """


def action_elements(schema_path: str) -> "set[str]":
    """The XML element names inside ``<xs:group name="ActionGroup">``.

    Commented-out elements are absent from the parse, which is the behaviour
    that matters: ``DetectObject`` is registered in bt.cpp but commented out of
    the Amiga schema, so a mission containing one is rejected before it runs and
    this robot must not claim it.
    """
    try:
        root = ElementTree.parse(schema_path).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise SchemaError(
            f"could not read mission schema {schema_path}: {exc}"
        ) from exc

    for group in root.iter(f"{XS}group"):
        if group.get("name") != ACTION_GROUP:
            continue
        return {
            element.get("name")
            for element in group.iter(f"{XS}element")
            if element.get("name")
        }
    raise SchemaError(f'{schema_path} has no <xs:group name="{ACTION_GROUP}">')


def capabilities_from_xsd(schema_path: str) -> "tuple[Capability, ...]":
    """Every Capability this schema permits, in bit-index order.

    Elements the codec has no bit for are ignored rather than fatal: a schema
    from a newer submodule pointer than this build can legitimately name an
    action we have not allocated a bit for yet, and the right response is to
    advertise what we do understand. Understanding *nothing* is the error, and
    that one raises.
    """
    permitted = action_elements(schema_path)
    known = sorted(
        (
            CAPABILITY_BY_ELEMENT[name]
            for name in permitted
            if name in CAPABILITY_BY_ELEMENT
        ),
        key=int,
    )
    if not known:
        raise SchemaError(
            f"{schema_path} permits {sorted(permitted)}, none of which this "
            f"build has a capability bit for"
        )
    return tuple(known)


def unknown_actions(schema_path: str) -> "tuple[str, ...]":
    """Schema actions with no capability bit. Worth a warning at startup.

    Not an error -- see above -- but not silent either: it is the fleet's
    notice that this robot can be asked to do something it cannot advertise,
    and therefore that such work will never be delegated to or from it.
    """
    return tuple(
        sorted(
            name
            for name in action_elements(schema_path)
            if name not in CAPABILITY_BY_ELEMENT
        )
    )


def mask_from_xsd(schema_path: str) -> int:
    """Convenience: the advertised ``cap_mask`` for this schema."""
    return cap_mask(*capabilities_from_xsd(schema_path))


def default_schema_path() -> Optional[str]:
    """The installed ``amiga_btcpp.xsd``, or None outside a built workspace.

    The same file ``bt.cpp`` resolves through ``ament_index``, found the same
    way, so the coordinator and the behaviour tree cannot end up reading
    different schemas on one robot.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return None
    try:
        share = get_package_share_directory("amiga_ros2_behavior_tree")
    except Exception:  # noqa: BLE001 - PackageNotFoundError, by any name
        return None
    import os

    path = os.path.join(share, "schemas", "amiga_btcpp.xsd")
    return path if os.path.isfile(path) else None

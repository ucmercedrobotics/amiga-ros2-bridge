"""
xsd.py

BT.CPP mission schema loading, shared by the arbiter and the mission planner.

Both agents validate mission XML against the *same* schema the BT executor uses —
`amiga_ros2_behavior_tree`'s installed `schemas/amiga_btcpp.xsd` — so the schema
is a single source of truth rather than a copy that can drift.

Env vars:
    AMIGA_XSD_PATH      optional absolute path override
"""

import os
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from lxml import etree

AMIGA_XSD_PATH = os.environ.get("AMIGA_XSD_PATH", "")


def resolve_path(logger) -> str:
    """Absolute path to amiga_btcpp.xsd, or "" if it can't be located."""
    if AMIGA_XSD_PATH:
        return AMIGA_XSD_PATH
    try:
        share_dir = get_package_share_directory("amiga_ros2_behavior_tree")
    except Exception as exc:
        logger.error(f"Could not resolve amiga_ros2_behavior_tree share dir: {exc}")
        return ""
    return os.path.join(share_dir, "schemas", "amiga_btcpp.xsd")


def load_schema(logger) -> Optional["etree.XMLSchema"]:
    """Parsed schema for validation, or None if it couldn't be loaded."""
    path = resolve_path(logger)
    if not path:
        return None
    try:
        with open(path, "rb") as f:
            return etree.XMLSchema(etree.parse(f))
    except (OSError, etree.XMLSyntaxError, etree.XMLSchemaParseError) as exc:
        logger.error(f"Failed to load mission XSD from {path}: {exc}")
        return None


def read_text(logger) -> str:
    """Raw schema text — the planner pastes this into its prompt."""
    path = resolve_path(logger)
    if not path:
        return ""
    try:
        with open(path, "r") as f:
            return f.read()
    except OSError as exc:
        logger.error(f"Failed to read mission XSD text from {path}: {exc}")
        return ""


def validate(schema, xml_str: str) -> Tuple[bool, str]:
    """Returns (is_valid, error_message). A missing schema passes."""
    if schema is None:
        return True, "schema unavailable — skipped validation"
    try:
        doc = etree.fromstring(xml_str.encode("utf-8"))
    except etree.XMLSyntaxError as exc:
        return False, f"not well-formed XML: {exc}"
    if schema.validate(doc):
        return True, ""
    return False, str(schema.error_log)

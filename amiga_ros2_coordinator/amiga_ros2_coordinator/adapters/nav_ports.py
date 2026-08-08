#!/usr/bin/env python3
"""A NavInterface backed by this robot's GPS fix and the orchard model.

Three questions -- ``can_reach``, ``eta``, ``current_location`` -- answered by
resolving the target to a latitude and longitude and taking the straight-line
distance from the last fix, over a nominal ground speed. Two design constraints
shape all of it, and either alone would be enough.

**Nothing here may block.** ``_assess`` calls all three from ``_on_announce``,
which runs under the coordinator's lock -- the lock ``tick`` and ``on_message``
need. Anything that waits on a service, an action or a network round trip stops
this robot's heartbeats, auctions and retransmit deadlines for that long, and
the fleet reads a robot that stopped answering as a robot that died. So every
question below is a dictionary lookup and some arithmetic, and the two things
that would otherwise be calls are subscriptions instead.

**There is no map to plan against.** The fleet works unknown, changing
environments; the global costmap builds as the robot drives rather than being
surveyed in advance. Asking a planner for a route to a place the robot has never
been near does not answer "can I get there" -- it answers "is there a path
through the part of the world I have already seen", which for a task somewhere
new is a refusal or a straight line with extra steps. Distance is the honest
feasibility signal, and it is the one that does not claim knowledge the robot
does not have.

An optimistic ETA is not a defect. ``NavInterface.eta`` already says a bid "may
be optimistic -- every bidder's is, and a bid is an offer rather than a promise",
and that it only has to be *comparable*. Distance over speed is comparable
between robots, which is all the auction needs: the closest capable robot bids
best and wins. Something far enough to be implausible clamps to ``ETA_MAX_S`` on
the wire and becomes a maximally unattractive bid rather than an impossible one
-- the existing design's answer, and better than a hard cut-off.

**Where the places come from.** ``TargetKind.GPS`` carries its own coordinates.
``TREE`` and ``AISLE`` carry an index into the orchard model, and this resolves
them from ``/orchard/tree_info_json`` -- the same JSON
``orchard_management_node`` consumes to serve ``orchard/get_tree_info``. The
topic rather than the service precisely because the service would be a call, and
a call cannot happen here. Reading the same source keeps this from holding a
second opinion about where tree 60 is.

Subscriptions:

    gps/pvt                    sensor_msgs/NavSatFix   where we are
    /orchard/tree_info_json    std_msgs/String         where everything else is
"""

import json
import math
from threading import Lock
from typing import Optional, Tuple

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from amiga_ros2_comms.codec import Target, TargetKind

#: Mean Earth radius, metres. Over an orchard the spherical approximation is
#: exact to well under the GPS noise, and an ETA is quantized to 4 s on the wire
#: regardless.
EARTH_RADIUS_M = 6_371_008.8

#: Ground speed used to turn a distance into an ETA, m/s. The Amiga's cruise,
#: not its top speed: a bid that assumes best-case travel wins auctions it
#: should have lost.
DEFAULT_SPEED_MPS = 0.8

#: Seconds a GPS fix stays usable. Past it ``current_location`` reports
#: ``Target.none()`` -- asked, and no fix -- rather than the last known
#: position, which would have this robot bidding from where it used to be.
DEFAULT_FIX_TTL_SEC = 10.0

#: Absolute on purpose. There is one orchard and every robot is in it; the JSON
#: arrives over whichever robot's mission port was fed and is published under
#: this name by ``tcp_demux_node``, whose own default is absolute too.
DEFAULT_ORCHARD_TOPIC = "/orchard/tree_info_json"


def great_circle_m(lat_a: float, lon_a: float, lat_b: float, lon_b: float) -> float:
    """Metres between two lat/lon pairs in degrees. Pure function, no ROS."""
    phi_a, phi_b = math.radians(lat_a), math.radians(lat_b)
    d_phi = phi_b - phi_a
    d_lambda = math.radians(lon_b - lon_a)
    h = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi_a) * math.cos(phi_b) * math.sin(d_lambda / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def parse_orchard(payload: str) -> Tuple[dict, dict]:
    """(tree index -> point, aisle index -> [point]) from the orchard JSON.

    The wrapped-object format ``orchard_management.cpp`` documents: ``trees``,
    ``aisle_entrances`` keyed by ``entrance_index``, and
    ``aisle_to_entrance_indices`` mapping an aisle to its ends. Anything
    malformed is skipped rather than raising -- a partial orchard makes some
    tasks unbiddable, and a raised exception in a subscription callback makes
    all of them unbiddable.

    Pure function, so the shape can be tested against a real orchard dump with
    no node running.

    The ``x_offset``/``y_offset`` the service applies are deliberately not
    applied here. They shift every point by the same vector, and every question
    asked of this class is a *comparison* between distances, which that shift
    leaves unchanged.
    """
    trees: dict = {}
    aisles: dict = {}
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return trees, aisles
    if not isinstance(data, dict):
        return trees, aisles

    for tree in data.get("trees") or []:
        point = _point_of(tree)
        index = tree.get("tree_index") if isinstance(tree, dict) else None
        if point is not None and isinstance(index, int):
            trees[index] = point

    entrances = {}
    for entrance in data.get("aisle_entrances") or []:
        point = _point_of(entrance)
        index = entrance.get("entrance_index") if isinstance(entrance, dict) else None
        if point is not None and isinstance(index, int):
            entrances[index] = point

    mapping = data.get("aisle_to_entrance_indices")
    if isinstance(mapping, dict):
        for raw_aisle, indices in mapping.items():
            try:
                aisle = int(raw_aisle)
            except (TypeError, ValueError):
                continue
            ends = [entrances[i] for i in indices or [] if i in entrances]
            if ends:
                aisles[aisle] = ends
    return trees, aisles


def _point_of(record) -> Optional[Tuple[float, float]]:
    if not isinstance(record, dict):
        return None
    lat, lon = record.get("lat"), record.get("lon")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        return None
    return float(lat), float(lon)


class GpsNav:
    """Navigation answers from a cached GPS fix and a cached orchard. No planner."""

    def __init__(
        self,
        node: Node,
        speed_mps: float = DEFAULT_SPEED_MPS,
        fix_ttl_sec: float = DEFAULT_FIX_TTL_SEC,
        gps_topic: str = "gps/pvt",
        orchard_topic: str = DEFAULT_ORCHARD_TOPIC,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._speed = max(float(speed_mps), 1e-3)
        self._fix_ttl = float(fix_ttl_sec)

        self._lock = Lock()
        self._fix: Optional[NavSatFix] = None
        self._fix_at: float = 0.0
        self._trees: dict = {}
        self._aisles: dict = {}
        self._warned_unresolved = False

        self._gps_sub = node.create_subscription(NavSatFix, gps_topic, self._on_fix, 10)
        self._orchard_sub = node.create_subscription(
            String, orchard_topic, self._on_orchard, 10
        )

    # ------------------------------------------------------------------
    # NavInterface
    # ------------------------------------------------------------------

    def current_location(self) -> Optional[Target]:
        """Where we are. None if never asked, Target.none() if the fix is stale.

        The two really are different and the protocol says so: None means
        navigation has not answered yet, ``Target.none()`` means it has and
        there is no fix. Collapsing them advertises a position this robot never
        claimed.
        """
        fix = self._fresh_fix()
        if fix is None:
            return None if self._fix is None else Target.none()
        return Target.gps(fix.latitude, fix.longitude)

    def can_reach(self, target: Target) -> bool:
        """Whether we could go there at all: do we know where it is, and where we are."""
        if TargetKind(int(target.kind)) is TargetKind.NONE:
            # Wherever we are. Trivially reachable; we are already there.
            return True
        fix = self._fresh_fix()
        if fix is None:
            return False
        return self._resolve(target, fix) is not None

    def eta(self, target: Target) -> float:
        """Seconds to reach ``target``: straight line over the nominal speed.

        Raises rather than returning a large number when there is no answer at
        all. ``_assess`` reads a raising nav as a nav that cannot get there,
        which is right -- an unanswerable ETA is not a slow one.
        """
        if TargetKind(int(target.kind)) is TargetKind.NONE:
            return 0.0
        fix = self._fresh_fix()
        point = self._resolve(target, fix) if fix is not None else None
        if point is None:
            raise RuntimeError(f"no ETA available for {target}")
        return great_circle_m(fix.latitude, fix.longitude, *point) / self._speed

    # ------------------------------------------------------------------
    # Resolution
    # ------------------------------------------------------------------

    def _resolve(self, target: Target, fix) -> Optional[Tuple[float, float]]:
        """Where a target actually is, or None if we cannot say."""
        kind = TargetKind(int(target.kind))
        if kind is TargetKind.GPS:
            return target.lat_deg, target.lon_deg

        with self._lock:
            if kind is TargetKind.TREE:
                point = self._trees.get(int(target.a))
            elif kind is TargetKind.AISLE:
                ends = self._aisles.get(int(target.a)) or []
                # An aisle has two ends and only one of them is the near one.
                # Costing the far end would have this robot bid as though it had
                # to drive the length of the row before starting.
                point = min(
                    ends,
                    key=lambda end: great_circle_m(fix.latitude, fix.longitude, *end),
                    default=None,
                )
            else:
                point = None

        if point is None:
            self._warn_once_unresolved(target)
        return point

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        # STATUS_NO_FIX is a message saying there is no position in it. Filtered
        # here rather than stored and checked later, so exactly one place
        # decides what counts as a fix.
        if msg.status.status < 0:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._fix = msg
            self._fix_at = self._now()

    def _on_orchard(self, msg: String) -> None:
        trees, aisles = parse_orchard(msg.data)
        if not trees and not aisles:
            self._logger.warn(
                "the orchard JSON named no places this robot can resolve; "
                "tree- and aisle-targeted work cannot be bid on"
            )
            return
        with self._lock:
            self._trees, self._aisles = trees, aisles
            self._warned_unresolved = False
        self._logger.info(f"orchard model: {len(trees)} trees, {len(aisles)} aisles")

    def _fresh_fix(self) -> Optional[NavSatFix]:
        with self._lock:
            if self._fix is None:
                return None
            if self._fix_ttl and self._now() - self._fix_at > self._fix_ttl:
                return None
            return self._fix

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _warn_once_unresolved(self, target: Target) -> None:
        # Once per orchard update, not once per announcement: a fleet announcing
        # into an orchard this robot has not been given would otherwise fill the
        # log at the auction rate.
        if self._warned_unresolved:
            return
        self._warned_unresolved = True
        self._logger.warn(
            f"cannot resolve {target} against the orchard model "
            f"({len(self._trees)} trees, {len(self._aisles)} aisles known), so "
            f"work there will not be bid on"
        )

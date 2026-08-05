#!/usr/bin/env python3
"""Contract-net coordination for the Amiga fleet. Deterministic, both roles.

The layer above reliability and below nothing:

    firmware -> serial bridge -> codec -> reliability -> coordinator

Reliability makes bytes arrive once. This decides what they should have said.
Every robot runs one, and every robot runs it in both roles at the same time --
owner of tasks it may have to shed, bidder on tasks its peers announce.

    owner   infeasible -> interpret -> ANNOUNCE -> collect -> arbitrate
                       -> GRANT -> *delivered* -> transferred -> replan
    bidder  ANNOUNCE -> capable? -> fitness -> backoff -> (suppressed?)
                     -> BID -> GRANT for us -> absorb -> replan

Three things are worth knowing before reading any of it:

**Unassigned-until-ACKed.** Sending a GRANT does not transfer a task; a
delivery report from the reliability layer does. Announced is still ours,
granted-but-unconfirmed is still ours. Any other rule opens a window in which a
task belongs to nobody.

**Fitness-proportional backoff with suppression.** The best-suited bidder waits
the shortest time, and a bidder that overhears a better bid during its own wait
never transmits at all. Ordering alone would still put one packet per bidder on
a shared half-duplex radio; suppression is what makes an auction cost about one
bid.

**The reasoning is injected.** Exactly two questions here need judgement --
what to do about an anomaly, and whether the re-planned mission still verifies
-- and both are ports (``reasoning.py``). In production the first is answered
by the triage agent in ``amiga_ros2_agents``; the tests answer both with
trivial stubs. Everything else is a decision procedure that can be predicted
from its inputs, which is what makes the acceptance suite a meaningful claim
about the whole layer.

``CoordinatorSession`` is the engine and has no ROS in it: the clock is
injected, transport is a port, and nav and the mission stack are protocols. A
whole auction, backoff window and peer timeout run in microseconds
with nothing sleeping. ``CoordinatorNode`` is the thin wiring.

Specified in ``docs/coordinator.md``.
"""

from amiga_ros2_comms.codec import Capability, Target, TargetKind

from .auction import Auction, ReceivedBid
from .bidding import PendingBid, backoff_for, default_fitness, quantized_eta
from .coordinator import CoordinatorParams, CoordinatorSession, OwnedTask
from .interfaces import (
    MissionInterface,
    NavInterface,
    NullPreemption,
    PreemptionSignal,
)
from .capabilities import (
    SchemaError,
    capabilities_from_xsd,
    default_schema_path,
    mask_from_xsd,
)
from .model import (
    Fitness,
    MissionDelta,
    PeerRecord,
    Task,
    TaskState,
    capability_name,
    capability_names,
)
from .reasoning import (
    AcceptEverything,
    AlwaysReDelegate,
    AnomalyInterpreter,
    MissionReplanner,
    RejectEverything,
    ReplanResult,
    ScriptedInterpreter,
)
from .registry import PeerRegistry
from .schema import (
    ACTION_TYPES,
    ActionSchema,
    AddTask,
    AnomalyContext,
    DropTask,
    LocalDisposition,
    ReDelegate,
    validate_action,
)

__all__ = [
    # Engine
    "CoordinatorSession",
    "CoordinatorParams",
    "OwnedTask",
    # Contract-net pieces
    "Auction",
    "ReceivedBid",
    "PendingBid",
    "backoff_for",
    "default_fitness",
    "quantized_eta",
    "PeerRegistry",
    # Model. Target and Capability are the codec's, re-exported rather than
    # redefined: both are statements about the behaviour tree's schema, and a
    # second copy here is how the two would come to disagree.
    "Task",
    "TaskState",
    "Target",
    "TargetKind",
    "Capability",
    "Fitness",
    "PeerRecord",
    "MissionDelta",
    "capability_name",
    "capability_names",
    # What this robot can do, read off the mission schema it validates against
    "capabilities_from_xsd",
    "mask_from_xsd",
    "default_schema_path",
    "SchemaError",
    # Action schema
    "ActionSchema",
    "ACTION_TYPES",
    "ReDelegate",
    "AddTask",
    "DropTask",
    "LocalDisposition",
    "AnomalyContext",
    "validate_action",
    # Ports
    "NavInterface",
    "MissionInterface",
    "PreemptionSignal",
    "NullPreemption",
    # The two reasoning points, and the stubs standing in for them
    "AnomalyInterpreter",
    "MissionReplanner",
    "ReplanResult",
    "AlwaysReDelegate",
    "ScriptedInterpreter",
    "AcceptEverything",
    "RejectEverything",
]

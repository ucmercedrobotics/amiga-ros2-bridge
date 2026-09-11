#!/usr/bin/env python3
"""The three-robot headless fleet rig: real nodes, faked nav/mission, a wire.

Extracted out of ``test_fleet_auction.py``, whose module docstring explains
*why* three robots on one broadcast channel is the smallest interesting
fleet: with one possible bidder there is nothing to order, nothing to
suppress, and no way to tell a bid that won from a bid that was the only one.

This module owns the rig -- ``Channel``, ``Robot``, ``Spinner``, ``wait_until``
and ``Fleet`` -- so anything that wants to drive a real contract-net auction
end to end can build one without re-deriving it: the acceptance suite here,
and an experiment harness that wants to run scenarios beyond what any one test
asserts on. ``Fleet`` therefore takes a few more knobs than the suite itself
ever passes (``robots``, ``shedder_id``, ``seed``) -- their defaults reproduce
exactly what the suite already relied on, so nothing here changes test
behaviour, only what a caller is allowed to ask for.

Real state machines on every robot, real reliability layers, real codec, real
ROS. Faked: nav and the mission stack (fakes.py), and the radio, which is a
node that copies frames. Nothing about the auction is faked.
"""

import os
import random
import sys
import threading
import time
from typing import Dict, Optional

import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.msg import LoRaFrame  # noqa: E402
from amiga_ros2_comms.codec import (  # noqa: E402
    XML_ELEMENT,
    Capability,
    Target,
    cap_mask,
    decode,
)
from amiga_ros2_comms.reliability.node import ReliabilityNode  # noqa: E402

from amiga_ros2_coordinator.vocabulary.model import Task  # noqa: E402
from amiga_ros2_coordinator.nodes.coordinator_node import CoordinatorNode  # noqa: E402
from amiga_ros2_coordinator.ports.reasoning import (  # noqa: E402
    AcceptEverything,
    ScriptedInterpreter,
    ScriptedNoteInterpreter,
)
from amiga_ros2_coordinator.vocabulary.schema import (  # noqa: E402
    KeepBid,
    LocalDisposition,
    ReDelegate,
)
from fakes import FakeMission, FakeNav  # noqa: E402

TASK_ID = 42
TREE = 60

#: Sampling one tree: drive to it, then operate the arm. The pairing
#: examples/sample_leafs.xml uses, and the one shape mission_tasks.synthesize
#: can rebuild from an announcement -- so it is the shape a fleet can trade.
SAMPLING = (Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF)

NOTE = "north end of row 7 is flooded; approach from the south, expect 4 min extra"

#: Short, because these run against the wall clock. Every relationship the
#: engine checks is preserved -- the backoff fits inside the window, and
#: bid_memory outlasts the *deliberative* window, which is announce_window x9
#: (4.5 s here) and is what a note-bearing auction runs on.
FAST_PARAMS = {
    "announce_window_sec": 0.5,
    "announce_repeat_sec": 0.2,
    "bid_max_backoff_sec": 0.1,
    "bid_memory_sec": 20.0,
    "heartbeat_period_sec": 0.3,
    "tick_period_sec": 0.02,
    "stats_period_sec": 0.0,
    # Both reasoning points are stubs here, injected by Robot below. Left
    # enabled, the node builds service clients instead and each one spends its
    # wait_for_service timeout discovering that no agent is running -- which
    # not only skips the scripted interpreters but perturbs the very timing
    # these tests measure, since that wait happens on the blocking group.
    "use_triage_agent": False,
    "use_note_agent": False,
}

#: The fleet's default scenario, reproduced exactly: robot 1 sheds, is closest
#: to nothing in particular, robot 3 is closest and should win, robot 2 is
#: four times further away and should suppress itself.
DEFAULT_ROBOTS = [
    {
        "node_id": 1,
        "eta_sec": 60.0,
        "capabilities": SAMPLING,
        "battery": 88,
        "idle": True,
    },
    {
        "node_id": 2,
        "eta_sec": 400.0,
        "capabilities": SAMPLING,
        "battery": 88,
        "idle": True,
    },
    {
        "node_id": 3,
        "eta_sec": 40.0,
        "capabilities": SAMPLING,
        "battery": 88,
        "idle": True,
    },
]


def a_task():
    return Task(
        task_id=TASK_ID,
        required_capabilities=cap_mask(*SAMPLING),
        location=Target.tree(TREE),
        priority=100,
    )


def _params(**overrides):
    return [Parameter(k, value=v) for k, v in overrides.items()]


class Channel(Node):
    """One shared broadcast medium: every robot's tx reaches every other's rx.

    A node that copies frames, not a radio model -- ``amiga_ros2_comms``' own
    ``lora_sim`` is the radio model, with airtime and collisions, and it belongs
    in that package's tests. What this adds and that cannot easily give is a
    **filter**: ``drop`` is handed every decoded message and can refuse it,
    which is how the control experiment removes exactly the note fragments and
    nothing else.
    """

    def __init__(self, names, drop=None):
        super().__init__("channel")
        self._drop = drop
        self.carried = []
        self.dropped = []
        self._lock = threading.Lock()
        self._rx = {
            name: self.create_publisher(LoRaFrame, f"/{name}/lora/rx", 64)
            for name in names
        }
        for name in names:
            self.create_subscription(
                LoRaFrame,
                f"/{name}/lora/tx",
                lambda msg, sender=name: self._forward(sender, msg),
                64,
            )

    def _forward(self, sender, msg):
        message = self._decoded(bytes(msg.data))
        if self._drop is not None and message is not None and self._drop(message):
            with self._lock:
                self.dropped.append(message)
            return
        with self._lock:
            self.carried.append((sender, message))
        for name, publisher in self._rx.items():
            # Not back to the sender: a real half-duplex radio does not hear
            # itself, and the reliability layer's dedup would hide the mistake
            # rather than fail on it.
            if name != sender:
                publisher.publish(msg)

    @staticmethod
    def _decoded(payload):
        try:
            return decode(payload)
        except Exception:  # noqa: BLE001 - the test asserts on shape, not bytes
            return None

    def sent(self, message_type):
        with self._lock:
            return [m for _, m in self.carried if isinstance(m, message_type)]

    def order_of(self, *message_types):
        """Which of ``message_types`` appeared, in the order they went out."""
        with self._lock:
            return [
                type(m).__name__
                for _, m in self.carried
                if isinstance(m, message_types)
            ]

    def counts(self):
        """How many of each message type this channel has carried, by class name.

        What ``Fleet.outcome()`` reports -- computed from ``carried`` rather
        than tracked separately, so it can never disagree with what ``sent``
        and ``order_of`` already answer from the same list. Undecodable frames
        (``_decoded`` returned None) are never appended to ``carried`` at all,
        so they cannot appear here either.
        """
        with self._lock:
            tally: Dict[str, int] = {}
            for _, message in self.carried:
                name = type(message).__name__
                tally[name] = tally.get(name, 0) + 1
            return tally


class Robot:
    """One robot: two real nodes, and fakes where its own stack would be."""

    def __init__(
        self,
        namespace,
        node_id,
        eta_sec,
        interpreter=None,
        note_interpreter=None,
        capabilities=SAMPLING,
        battery=88,
        current_task=0,
        rng=None,
        **overrides,
    ):
        # The one number that differs between robots, and the one the auction is
        # supposed to order on. Everything else is deliberately identical, so a
        # winner can only have been chosen by distance.
        self.nav = FakeNav(eta_sec=eta_sec, location=Target.tree(TREE))
        self.mission = FakeMission(battery=battery, current_task=current_task)
        self.replanner = AcceptEverything()
        self.reliability = ReliabilityNode(
            namespace=namespace,
            parameter_overrides=_params(
                node_id=node_id, retransmit_timeout_sec=0.3, tick_period_sec=0.02
            ),
        )
        settings = dict(FAST_PARAMS)
        settings.update(overrides)
        self.coordinator = CoordinatorNode(
            reliability=self.reliability,
            nav=self.nav,
            mission=self.mission,
            interpreter=interpreter,
            replanner=self.replanner,
            note_interpreter=note_interpreter,
            rng=rng,
            namespace=namespace,
            parameter_overrides=_params(
                capabilities=[XML_ELEMENT[c] for c in capabilities], **settings
            ),
        )

    @property
    def session(self):
        return self.coordinator.session

    def stats(self):
        return self.session.stats()

    def nodes(self):
        return (self.reliability, self.coordinator)

    def destroy(self):
        self.coordinator.destroy_node()
        self.reliability.destroy_node()


class Spinner:
    """A background executor, so a test can wait on a condition, not a sleep.

    Single-threaded on purpose -- see the note in test_coordinator_node.py:
    ``spin_once`` on a MultiThreadedExecutor hands the callback to a pool and
    returns, so stopping the thread does not stop the callbacks and teardown
    races the timers on nodes the test is destroying.
    """

    def __init__(self, nodes):
        self.executor = SingleThreadedExecutor()
        for node in nodes:
            self.executor.add_node(node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.02)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)


def wait_until(predicate, timeout=12.0, interval=0.02):
    """Poll ``predicate`` until it holds. Returns whether it did."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def _normalize_robots(robots):
    """Accept a list or a dict of per-robot specs; always hand back a list.

    A dict is keyed by ``node_id`` and its values need not repeat it; a list
    is a sequence of dicts that must each carry their own ``node_id``. Either
    way what comes out is a list of plain dicts, so the rest of ``Fleet``
    only has one shape to deal with.
    """
    if isinstance(robots, dict):
        specs = []
        for node_id, spec in robots.items():
            spec = dict(spec)
            spec.setdefault("node_id", node_id)
            specs.append(spec)
        return specs
    return [dict(spec) for spec in robots]


class Fleet:
    """Three robots and the channel between them, torn down together.

    The default construction reproduces the original acceptance-test scenario
    exactly: robot 1 sheds task 42 and has a note about it, robot 3 is closest
    and should win, robot 2 is four times further and should suppress itself.
    Every parameter beyond ``drop``/``note``/``revisions`` is new surface for a
    scenario-driving harness and defaults to reproducing that same fleet, so
    existing callers see no change in behaviour.
    """

    def __init__(
        self,
        drop=None,
        note=NOTE,
        revisions=None,
        robots=None,
        shedder_id=1,
        interpreter=None,
        seed=None,
    ):
        self.shedder_id = int(shedder_id)
        self._last_task_id: Optional[int] = None
        self._started_at = time.time()

        # Robot 1 sheds the task and has something to say about it. The note
        # rides on the ReDelegate, which is where the decision to shed it was
        # made -- one call, already holding the whole context. Only built when
        # the caller has not supplied its own interpreter for the shedder.
        self.shedder = interpreter or ScriptedInterpreter(
            [
                ReDelegate(
                    task=a_task(),
                    fallback=LocalDisposition.HOLD,
                    note=note,
                )
            ]
        )

        specs = _normalize_robots(robots if robots is not None else DEFAULT_ROBOTS)

        self.robots: Dict[int, Robot] = {}
        namespaces: Dict[int, str] = {}
        for spec in specs:
            node_id = int(spec["node_id"])
            namespace = spec.get("namespace", f"r{node_id}")
            namespaces[node_id] = namespace

            idle = spec.get("idle", True)
            current_task = spec.get(
                "current_task", 0 if idle else spec.get("busy_task_id", 1)
            )
            robot_rng = random.Random(seed + node_id) if seed is not None else None

            if node_id == self.shedder_id:
                robot_interpreter = self.shedder
                robot_note_interpreter = spec.get("note_interpreter")
            else:
                robot_interpreter = spec.get("interpreter")
                # Robot 3 is closer, so its backoff is shorter and it should
                # both transmit first and win.
                robot_note_interpreter = spec.get(
                    "note_interpreter",
                    ScriptedNoteInterpreter(revisions or [KeepBid()]),
                )

            self.robots[node_id] = Robot(
                namespace,
                node_id,
                eta_sec=spec.get("eta_sec", 60.0),
                interpreter=robot_interpreter,
                note_interpreter=robot_note_interpreter,
                capabilities=spec.get("capabilities", SAMPLING),
                battery=spec.get("battery", 88),
                current_task=current_task,
                rng=robot_rng,
                **spec.get("overrides", {}),
            )

        self.channel = Channel(list(namespaces.values()), drop=drop)
        nodes = [self.channel]
        for robot in self.robots.values():
            nodes.extend(robot.nodes())
        self.spinner = Spinner(nodes)

    def shed(self, task=None, detail="", action=None):
        """The shedding robot reports a task infeasible, which starts the auction.

        ``task`` defaults to the fleet's own scenario task, so existing callers
        that pass nothing get exactly the original behaviour. ``action`` is for
        a harness that has already decided what to do -- from a deterministic
        policy, or a round trip through a live triage agent -- and only needs
        it applied; it goes straight through to
        ``CoordinatorSession.report_infeasible(action=...)``, which validates
        it exactly as an interpreter's own answer would be. Returns whatever
        ``report_infeasible`` returns: the action taken, or None.
        """
        task = a_task() if task is None else task
        self._last_task_id = task.task_id if task is not None else None
        return self.robots[self.shedder_id].session.report_infeasible(
            task, detail=detail, action=action
        )

    def owner_of(self, task_id):
        """Which robot's session currently owns ``task_id``, or None.

        Reads it off each session's own ``OwnedTask.ours`` -- true for a task
        that is still ours to execute (OURS/ANNOUNCED/GRANTED) and false once
        it is TRANSFERRED away -- rather than keeping a separate record here,
        so this can never disagree with what the engine itself believes.
        """
        for node_id, robot in self.robots.items():
            record = robot.session.task(task_id)
            if record is not None and record.ours:
                return node_id
        return None

    def quiesced(self):
        """True when nothing is actively resolving on the fleet right now.

        Two honest, engine-owned signals, on every robot: ``auctions_open`` off
        ``CoordinatorSession.stats()`` (nobody is still collecting bids or
        waiting to arbitrate) and ``pending`` off the reliability layer's own
        ``stats()`` (no reliable send -- a GRANT, most likely -- is still
        awaiting an ACK or a retransmit).

        What this does *not* prove: that the fleet agrees on an outcome, that
        a task now sits where it will stay, or that nothing new is about to go
        out -- a bare broadcast (a bid, a heartbeat, another announcement)
        leaves no pending entry at all, so this can be true for an instant
        between two unrelated events. Treat it as "nothing is in flight right
        now", suitable for ``wait_until``, not as "the trial is decided".
        """
        for robot in self.robots.values():
            if robot.stats().get("auctions_open", 0):
                return False
            if robot.reliability.stats().get("pending", 0):
                return False
        return True

    def outcome(self):
        """A plain, JSON-serializable summary of the trial so far.

        Meant to be called once the fleet has gone quiet (see ``quiesced``) --
        call it while an auction is still open and ``winner_id`` will read
        whatever is true in that instant, which is not yet a result.

        ``winner_id`` is ``owner_of`` the last task handed to ``shed()`` -- the
        task owner at quiescence. ``auctions_opened`` sums ``announced`` and
        ``re_announced`` across every robot, so a grant that failed and
        re-announced counts as a second auction, not a continuation of the
        first. ``message_counts`` is ``Channel.counts()`` verbatim.
        """
        return {
            "task_id": self._last_task_id,
            "winner_id": (
                self.owner_of(self._last_task_id)
                if self._last_task_id is not None
                else None
            ),
            "auctions_opened": sum(
                robot.stats().get("announced", 0) + robot.stats().get("re_announced", 0)
                for robot in self.robots.values()
            ),
            "message_counts": self.channel.counts(),
            "elapsed_sec": time.time() - self._started_at,
        }

    def close(self):
        self.spinner.stop()
        for robot in self.robots.values():
            robot.destroy()
        self.channel.destroy_node()


@pytest.fixture
def fleet_factory():
    made = []

    def build(**kwargs):
        fleet = Fleet(**kwargs)
        made.append(fleet)
        return fleet

    yield build
    for fleet in made:
        fleet.close()

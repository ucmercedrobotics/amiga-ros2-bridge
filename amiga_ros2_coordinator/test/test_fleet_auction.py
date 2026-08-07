#!/usr/bin/env python3
"""A task crossing a fleet of three, over a shared channel, with a note on it.

``test_coordinator_node.py`` proves a task crossing *two* robots over a relay.
Two is enough for the handshake and not enough for the mechanism: with one
possible bidder there is nothing to order, nothing to suppress, and no way to
tell a bid that won from a bid that was the only one. Three robots on one
broadcast channel is the smallest fleet where fitness-proportional backoff and
overhear-and-suppress are observable at all, and where "the closest robot won"
is a claim rather than a tautology.

The scenario is the one ``make fleet-scenario`` runs by hand:

    robot 1 cannot do task 42, and has something to say about it
      -> note fragments, then TASK_ANNOUNCE, on the shared channel
      -> robot 1 leaves the task: released, replanned, preemption raised
      -> robots 2 and 3 assess it; 3 is closer, so 3's backoff is shorter
      -> 3 transmits; 2 overhears a better bid and suppresses its own
      -> GRANT to 3, ACKed; the task is 3's and 1 has marked it transferred

and then the control, which is the point of the whole note design:

    the same scenario with **every note fragment dropped** reaches the same
    winner. A note improves a decision; it is never load-bearing for one.

Real state machines on all three, real reliability layers, real codec, real ROS.
Faked: nav and the mission stack (fakes.py), and the radio, which is a node that
copies frames. Nothing about the auction is faked.
"""

import os
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_interfaces.msg import LoRaFrame  # noqa: E402
from amiga_ros2_comms.codec import (  # noqa: E402
    XML_ELEMENT,
    Capability,
    Freeform,
    Target,
    TaskAnnounce,
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


class Robot:
    """One robot: two real nodes, and fakes where its own stack would be."""

    def __init__(
        self,
        namespace,
        node_id,
        eta_sec,
        interpreter=None,
        note_interpreter=None,
        **overrides,
    ):
        # The one number that differs between robots, and the one the auction is
        # supposed to order on. Everything else is deliberately identical, so a
        # winner can only have been chosen by distance.
        self.nav = FakeNav(eta_sec=eta_sec, location=Target.tree(TREE))
        self.mission = FakeMission(battery=88)
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
            namespace=namespace,
            parameter_overrides=_params(
                capabilities=[XML_ELEMENT[c] for c in SAMPLING], **settings
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


class Fleet:
    """Three robots and the channel between them, torn down together."""

    def __init__(self, drop=None, note=NOTE, revisions=None):
        # Robot 1 sheds the task and has something to say about it. The note
        # rides on the ReDelegate, which is where the decision to shed it was
        # made -- one call, already holding the whole context.
        self.shedder = ScriptedInterpreter(
            [
                ReDelegate(
                    task=a_task(),
                    fallback=LocalDisposition.HOLD,
                    note=note,
                )
            ]
        )
        self.robots = {
            1: Robot("r1", 1, eta_sec=60.0, interpreter=self.shedder),
            # Robot 3 is closer, so its backoff is shorter and it should both
            # transmit first and win.
            2: Robot(
                "r2",
                2,
                eta_sec=400.0,
                note_interpreter=ScriptedNoteInterpreter(revisions or [KeepBid()]),
            ),
            3: Robot(
                "r3",
                3,
                eta_sec=40.0,
                note_interpreter=ScriptedNoteInterpreter(revisions or [KeepBid()]),
            ),
        }
        self.channel = Channel([f"r{i}" for i in self.robots], drop=drop)
        nodes = [self.channel]
        for robot in self.robots.values():
            nodes.extend(robot.nodes())
        self.spinner = Spinner(nodes)

    def shed(self):
        """Robot 1 reports the task infeasible, which starts the auction."""
        self.robots[1].session.report_infeasible(
            a_task(), detail="local recovery exhausted"
        )

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


# ==========================================================================
# The scenario
# ==========================================================================


def test_a_task_crosses_the_fleet_to_the_closest_robot(ros, fleet_factory):
    fleet = fleet_factory()
    fleet.shed()

    won = wait_until(lambda: fleet.robots[3].session.task(TASK_ID) is not None)
    assert won, (
        "robot 3 never took the task on. "
        f"channel carried {fleet.channel.order_of(object)}"
    )

    record = fleet.robots[3].session.task(TASK_ID)
    assert record.ours, "the winner has to own what it acknowledged"
    assert fleet.robots[3].mission.absorbed, "the winner's mission absorbed it"

    # The announcer only lets go once someone else has acknowledged owning it:
    # unassigned-until-ACKed, seen from the losing end.
    assert wait_until(lambda: fleet.robots[1].mission.transferred)
    assert [t.task_id for t in fleet.robots[1].mission.transferred] == [TASK_ID]


def test_the_announcer_moves_on_instead_of_waiting_out_its_own_auction(
    ros, fleet_factory
):
    """The mission holds a task iff we still intend to execute it ourselves."""
    fleet = fleet_factory()
    fleet.shed()

    assert wait_until(lambda: fleet.robots[1].replanner.calls > 0), (
        "announcing a task has to take it out of our own mission straight away; "
        "a deliberative auction runs for seconds and the robot has other work"
    )
    removed = [
        task.task_id
        for delta in fleet.robots[1].replanner.deltas
        for task in delta.removed
    ]
    assert TASK_ID in removed
    assert fleet.robots[1].coordinator._preemption.requested, (
        "the behaviour tree has to be asked to yield, since what it is holding "
        "is no longer what the coordinator intends to execute"
    )


def test_the_note_goes_out_before_the_announcement_it_annotates(ros, fleet_factory):
    """Ordering is the whole reason a note is useful.

    ``_on_announce`` is the single synchronous moment a bid is decided, so the
    only question it can ask about a note is a cache lookup. A note arriving
    afterwards is counted and changes nothing.
    """
    fleet = fleet_factory()
    fleet.shed()

    assert wait_until(lambda: fleet.channel.sent(TaskAnnounce))
    order = fleet.channel.order_of(Freeform, TaskAnnounce)
    assert "Freeform" in order, f"no note reached the channel: {order}"
    assert order.index("Freeform") < order.index(
        "TaskAnnounce"
    ), f"the note has to precede its announcement, got {order}"


def test_a_bidder_reads_the_note_before_deciding_what_to_bid(ros, fleet_factory):
    fleet = fleet_factory()
    fleet.shed()

    assert wait_until(
        lambda: fleet.robots[3].stats().get("notes_before_announce", 0) > 0
    ), (
        "robot 3 should have found the note already cached when the "
        f"announcement arrived: {fleet.robots[3].stats()}"
    )
    interpreter = fleet.robots[3].coordinator.session.note_interpreter()
    assert wait_until(lambda: interpreter.calls), "the note was never interpreted"
    assert interpreter.calls[0].text == NOTE
    assert interpreter.calls[0].task_id == TASK_ID


def test_the_note_reaches_the_winners_replanner(ros, fleet_factory):
    """The end of the note's journey, and the reason it has one.

    A note used to be consumed at the bid: it revised a cost and was then
    thrown away. But the sentence is not about *whether* to take the work -- it
    is about how the work has to be done, which is a question only the winner's
    planner ever asks, and it asks it after the auction is already over.

    Carried on the delta rather than fetched at GRANT time. A note expires
    (``session.note_ttl_sec``) well inside the window a note-bearing
    announcement stays open, so by the time the GRANT lands, looking it up
    again would usually find nothing.
    """
    fleet = fleet_factory()
    fleet.shed()

    winner = fleet.robots[3]
    assert wait_until(lambda: winner.session.task(TASK_ID) is not None)
    assert wait_until(lambda: winner.replanner.calls > 0), "the winner never replanned"

    absorbed = [d for d in winner.replanner.deltas if d.added]
    assert absorbed, f"no absorption delta: {winner.replanner.deltas}"
    assert absorbed[0].note == NOTE


def test_an_auction_with_no_note_carries_no_note(ros, fleet_factory):
    """The other half: the field is empty when nobody said anything.

    Without this the test above would pass just as well against a delta that
    always carried the last note anybody had heard, about any task.
    """
    fleet = fleet_factory(drop=lambda message: isinstance(message, Freeform))
    fleet.shed()

    winner = fleet.robots[3]
    assert wait_until(lambda: winner.replanner.calls > 0)
    assert all(delta.note == "" for delta in winner.replanner.deltas)


def test_a_weaker_bidder_suppresses_itself_on_overhearing_a_better_one(
    ros, fleet_factory
):
    """The half of the mechanism that saves the airtime."""
    fleet = fleet_factory()
    fleet.shed()

    assert wait_until(lambda: fleet.robots[3].session.task(TASK_ID) is not None)
    assert wait_until(lambda: fleet.robots[2].stats().get("bids_suppressed", 0) > 0), (
        "robot 2 is four times further away, so it should have heard robot 3's "
        f"bid during its own backoff and never transmitted: {fleet.robots[2].stats()}"
    )


# ==========================================================================
# The control
# ==========================================================================


def test_the_auction_reaches_the_same_winner_when_every_note_fragment_is_lost(
    ros, fleet_factory
):
    """The property the whole note design rests on.

    A note makes a decision better informed. It is never what makes the decision
    possible -- the announcement carries the machine-readable requirement, so
    losing every fragment costs a quality number and never a correctness one.
    Asserted over a real channel here rather than a fake link.
    """
    fleet = fleet_factory(drop=lambda message: isinstance(message, Freeform))
    fleet.shed()

    won = wait_until(lambda: fleet.robots[3].session.task(TASK_ID) is not None)
    assert won, "the auction has to complete with no note at all"
    assert fleet.robots[3].session.task(TASK_ID).ours
    assert wait_until(lambda: fleet.robots[1].mission.transferred)

    assert fleet.channel.dropped, "the test dropped nothing; it proved nothing"
    assert not fleet.channel.sent(Freeform), "a fragment survived the filter"
    for robot in fleet.robots.values():
        assert robot.stats().get("notes_before_announce", 0) == 0
        assert robot.stats().get("notes_received", 0) == 0

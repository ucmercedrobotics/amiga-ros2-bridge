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

The rig itself -- ``Channel``, ``Robot``, ``Spinner``, ``wait_until``, ``Fleet``
and the ``fleet_factory`` fixture -- lives in ``fleet_rig.py``, generalized
enough for a scenario-driving harness to reuse; this file only imports it and
adds the scenario-specific assertions.
"""

from amiga_ros2_comms.codec import Freeform, TaskAnnounce
from fleet_rig import NOTE, TASK_ID, fleet_factory, wait_until  # noqa: F401


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


# ==========================================================================
# The seed
# ==========================================================================


def test_the_seed_makes_bid_backoff_reproducible(ros, fleet_factory):
    """Same seed -> identical backoff draws; a different seed -> can differ.

    Read off the per-robot ``random.Random`` the seed plumbs into
    ``CoordinatorSession`` directly, rather than racing real timers -- the
    wire-level tests above already prove distance orders the auction; this
    isolates what the seed alone controls, without depending on scheduling.
    """
    fleet_a = fleet_factory(seed=7)
    fleet_b = fleet_factory(seed=7)
    fleet_c = fleet_factory(seed=8)

    cost = 400  # an arbitrary mid-range cost; only the jitter varies with seed
    draws_a = [fleet_a.robots[i].coordinator.session._backoff(cost) for i in (1, 2, 3)]
    draws_b = [fleet_b.robots[i].coordinator.session._backoff(cost) for i in (1, 2, 3)]
    draws_c = [fleet_c.robots[i].coordinator.session._backoff(cost) for i in (1, 2, 3)]

    assert draws_a == draws_b, "the same seed has to reproduce the same backoff draws"
    assert draws_a != draws_c, "a different seed has to be free to draw differently"

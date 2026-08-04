#!/usr/bin/env python3
"""Acceptance tests for the coordinator. No ROS, no radio, no model.

The nine groups from the brief -- owner happy path, GRANT failure, no-bid
timeout, bidder bids, bidder suppression, bidder wins, registry liveness,
and safe preemption -- driven against the fakes in fakes.py with both
reasoning points stubbed.

Everything is deterministic. The clock is a number the test moves, so an
auction window, a bid backoff and a peer timeout are exact rather
than waited out; bid jitter is switched off, so a cost maps to one backoff and
one only. Nothing sleeps and nothing depends on chance.

The recurring shape of an ownership test is worth naming once: a GRANT is
*sent*, and then the test decides whether it was delivered. That is the whole
of unassigned-until-ACKed -- the state between those two lines is the one the
rule is about.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    Bid,
    Capability,
    Grant,
    Heartbeat,
    ReasonCode,
    TaskAnnounce,
    cap_mask,
)

from amiga_ros2_coordinator import (  # noqa: E402
    AcceptEverything,
    CoordinatorParams,
    CoordinatorSession,
    DropTask,
    Location,
    LocalDisposition,
    ReDelegate,
    RejectEverything,
    ScriptedInterpreter,
    Task,
    TaskState,
)
from fakes import (  # noqa: E402
    Clock,
    EventLog,
    FakeMission,
    FakeNav,
    FakePreemption,
    FakeReliability,
)

US = 1
PEER_A = 2
PEER_B = 3

TASK_ID = 7


def a_task(task_id=TASK_ID, capability=Capability.SPRAY, row=3, col=4, priority=200):
    return Task(
        task_id=task_id,
        required_capability=capability,
        location=Location(row=row, col=col),
        priority=priority,
    )


class Rig:
    """A coordinator and everything it talks to, all reachable from the test."""

    def __init__(
        self,
        session,
        reliability,
        nav,
        mission,
        preemption,
        clock,
        interpreter,
        replanner,
        events,
    ):
        self.session = session
        self.rel = reliability
        self.nav = nav
        self.mission = mission
        self.preempt = preemption
        self.clock = clock
        self.interpreter = interpreter
        self.replanner = replanner
        self.events = events

    def advance(self, seconds):
        """Move time on and let the coordinator notice."""
        self.clock.advance(seconds)
        self.session.tick()

    def hear(self, msg):
        """Deliver one inbound message, as the reliability layer would."""
        self.session.on_message(msg)

    def heartbeat_from(
        self,
        robot_id,
        capabilities=(Capability.DRIVE, Capability.SPRAY),
        row=0,
        col=0,
        battery=90,
        current_task=0,
    ):
        self.hear(
            Heartbeat(
                src=robot_id,
                seq=1,
                cap_mask=cap_mask(*capabilities),
                grid_row=row,
                grid_col=col,
                battery=battery,
                cur_task=current_task,
            )
        )

    def bid_from(self, robot_id, cost, eta_s=120, task_id=TASK_ID, feasible=True):
        self.hear(
            Bid(
                src=robot_id,
                seq=1,
                task_id=task_id,
                eta_s=eta_s,
                feasible=feasible,
                cost=cost,
            )
        )

    def announce_from(self, robot_id, task, reason_code=ReasonCode.TASK_FAILED):
        self.hear(
            TaskAnnounce(
                src=robot_id,
                seq=1,
                task_id=task.task_id,
                req_capability=int(task.required_capability),
                grid_row=task.location.row,
                grid_col=task.location.col,
                priority=int(task.priority),
                reason_code=reason_code,
            )
        )


def make_rig(
    node_id=US,
    capabilities=(Capability.DRIVE, Capability.SPRAY),
    interpreter=None,
    replanner=None,
    nav=None,
    mission=None,
    **param_overrides,
):
    """Build a coordinator wired to fakes, with jitter and heartbeats off.

    Jitter off so a cost maps to exactly one backoff; heartbeats off so the
    broadcast log contains only what a test put there. Both are switched back
    on by the groups that are about them.
    """
    params = CoordinatorParams(
        **{
            "bid_jitter_fraction": 0.0,
            "heartbeat_period_sec": 0.0,
            **param_overrides,
        }
    )
    clock = Clock()
    reliability = FakeReliability(node_id=node_id)
    nav = nav if nav is not None else FakeNav()
    mission = mission if mission is not None else FakeMission()
    preemption = FakePreemption()
    replanner = replanner if replanner is not None else AcceptEverything()
    events = EventLog()

    session = CoordinatorSession(
        node_id=node_id,
        reliability=reliability,
        nav=nav,
        mission=mission,
        interpreter=interpreter or ScriptedInterpreter([ReDelegate(task=a_task())]),
        replanner=replanner,
        capabilities=capabilities,
        preemption=preemption,
        params=params,
        clock=clock,
        on_event=events,
    )
    reliability.set_on_deliver(session.on_message)
    return Rig(
        session,
        reliability,
        nav,
        mission,
        preemption,
        clock,
        session._interpreter,
        replanner,
        events,
    )


# ==========================================================================
# Group 1: owner happy path
# ==========================================================================


def test_infeasible_task_is_announced_granted_to_the_best_bid_and_transferred():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)

    # Two capable peers, so the auction knows who it is waiting for.
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)

    rig.session.report_infeasible(task, detail="spray tank empty")

    announces = rig.rel.announces
    assert len(announces) == 1
    assert announces[0].task_id == TASK_ID
    assert announces[0].req_capability == Capability.SPRAY
    assert (announces[0].grid_row, announces[0].grid_col) == (3, 4)
    assert rig.session.state_of(TASK_ID) is TaskState.ANNOUNCED

    # Two bids; the second is the better one. Having heard from every expected
    # bidder, the auction closes without waiting out its window.
    rig.bid_from(PEER_A, cost=80, eta_s=200)
    assert rig.rel.grants == []
    rig.bid_from(PEER_B, cost=40, eta_s=120)

    grant = rig.rel.last_grant
    assert grant is not None, "the auction should have closed once everyone answered"
    assert grant.dst == PEER_B
    assert grant.msg.winner_id == PEER_B
    assert grant.msg.task_id == TASK_ID

    # Sent, not delivered. The task is still ours.
    assert rig.session.state_of(TASK_ID) is TaskState.GRANTED
    assert rig.mission.transferred == []
    assert rig.replanner.calls == 0

    grant.deliver()

    assert rig.session.state_of(TASK_ID) is TaskState.TRANSFERRED
    assert rig.mission.transferred == [task]
    assert rig.replanner.calls == 1
    assert rig.replanner.deltas[0].removed == [task]
    assert rig.session.owned_tasks == []


def test_the_auction_waits_out_its_window_when_a_capable_peer_stays_silent():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    rig.bid_from(PEER_A, cost=80)
    assert rig.rel.grants == [], "one of two expected bidders has not answered"

    rig.advance(rig.session.params.announce_window_sec + 0.1)
    assert rig.rel.last_grant.dst == PEER_A


def test_an_infeasible_bid_closes_the_auction_without_winning_it():
    """`feasible=False` is an answer, which is the point of carrying the flag."""
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    rig.bid_from(PEER_A, cost=10, feasible=False)
    rig.bid_from(PEER_B, cost=90, feasible=True)

    grant = rig.rel.last_grant
    assert grant.dst == PEER_B, "a cheap infeasible bid must not win"


def test_bids_are_ranked_by_cost_then_eta_then_robot_id():
    """Every robot must predict the same winner from the same bids."""
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    # Same cost; PEER_B is sooner.
    rig.bid_from(PEER_A, cost=50, eta_s=400)
    rig.bid_from(PEER_B, cost=50, eta_s=100)

    assert rig.rel.last_grant.dst == PEER_B


# ==========================================================================
# Group 2: the GRANT is not delivered
# ==========================================================================


def test_a_failed_grant_leaves_the_task_ours_and_goes_to_the_runner_up():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    rig.bid_from(PEER_A, cost=80)
    rig.bid_from(PEER_B, cost=40)
    first = rig.rel.last_grant
    assert first.dst == PEER_B

    first.fail()

    # Unassigned-until-ACKed: nothing was transferred.
    assert rig.mission.transferred == []
    assert rig.session.state_of(TASK_ID) is not TaskState.TRANSFERRED
    assert task in [record.task for record in rig.session.owned_tasks]

    # Fallback: the next best bid we already hold.
    second = rig.rel.last_grant
    assert second is not first
    assert second.dst == PEER_A
    assert rig.session.state_of(TASK_ID) is TaskState.GRANTED

    second.deliver()
    assert rig.mission.transferred == [task]


def test_a_failed_grant_with_no_runner_up_re_announces():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)

    rig.bid_from(PEER_A, cost=80)
    rig.rel.last_grant.fail()

    assert rig.mission.transferred == []
    assert len(rig.rel.announces) == 2, "a second announce-and-grant cycle"
    assert len(rig.rel.grants) == 1, "nobody has bid on the new announcement yet"
    assert rig.session.state_of(TASK_ID) is TaskState.ANNOUNCED


def test_the_excluded_bidder_is_not_offered_the_task_again():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)
    rig.bid_from(PEER_A, cost=80)
    rig.rel.last_grant.fail()

    # PEER_A bids again on the re-announcement; it is the robot whose GRANT we
    # could not deliver, so its offer is not considered.
    rig.bid_from(PEER_A, cost=10)
    rig.advance(rig.session.params.announce_window_sec + 0.1)

    assert len(rig.rel.grants) == 1
    assert rig.session.stats()["auctions_no_bid"] == 1


def test_delegation_gives_up_locally_once_the_attempt_budget_is_spent():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.DROP)]
        ),
        max_delegation_attempts=1,
    )
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)
    rig.bid_from(PEER_A, cost=80)

    rig.rel.last_grant.fail()

    assert len(rig.rel.announces) == 1, "the budget allowed only one cycle"
    assert rig.session.state_of(TASK_ID) is TaskState.RELINQUISHED
    assert rig.mission.released == [task]


def test_a_refused_send_is_not_a_transfer():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
        max_delegation_attempts=1,
    )
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)

    rig.rel.refuse_reliable = True
    rig.bid_from(PEER_A, cost=80)

    assert rig.rel.grants == []
    assert rig.mission.transferred == []
    assert rig.session.state_of(TASK_ID) is TaskState.OURS


# ==========================================================================
# Group 3: nobody bids
# ==========================================================================


def test_an_announce_window_that_closes_with_no_bid_sends_no_grant():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.HOLD)]
        )
    )
    rig.session.own(task)
    rig.session.report_infeasible(task)
    assert len(rig.rel.announces) == 1

    rig.advance(rig.session.params.announce_window_sec + 0.1)

    assert rig.rel.grants == [], "no viable bid means no GRANT"
    assert rig.session.stats()["auctions_no_bid"] == 1
    # HOLD: still ours, unexecuted, and nothing was committed to verify.
    assert rig.session.state_of(TASK_ID) is TaskState.OURS
    assert rig.mission.released == []
    assert rig.replanner.calls == 0


def test_a_no_bid_window_with_a_drop_fallback_releases_the_task():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.DROP)]
        )
    )
    rig.session.own(task)
    rig.session.report_infeasible(task)
    rig.advance(rig.session.params.announce_window_sec + 0.1)

    assert rig.rel.grants == []
    assert rig.session.state_of(TASK_ID) is TaskState.RELINQUISHED
    assert rig.mission.released == [task]
    assert rig.replanner.calls == 1


def test_a_no_bid_window_can_escalate_to_a_human():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.REQUEST_HUMAN)]
        )
    )
    rig.session.own(task)
    rig.session.report_infeasible(task)
    rig.advance(rig.session.params.announce_window_sec + 0.1)

    assert rig.session.stats()["escalated_to_human"] == 1
    assert rig.session.state_of(TASK_ID) is TaskState.OURS
    assert any("operator" in w for w in rig.events.warnings)


def test_an_announcement_is_repeated_while_its_window_is_open():
    """Announcements are best-effort broadcasts; repetition is their delivery."""
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
        announce_window_sec=5.0,
        announce_repeat_sec=2.0,
    )
    rig.session.own(task)
    rig.session.report_infeasible(task)

    rig.advance(2.1)
    assert len(rig.rel.announces) == 2
    rig.advance(2.1)
    assert len(rig.rel.announces) == 3

    rig.advance(2.0)  # past the window; the auction closes instead
    assert len(rig.rel.announces) == 3


# ==========================================================================
# Group 4: the bidder bids
# ==========================================================================


def test_an_announcement_we_are_capable_of_produces_a_bid_after_a_backoff():
    task = a_task()
    rig = make_rig()

    rig.announce_from(PEER_A, task)

    # The local nodes were asked before any bid was formed.
    assert "can_reach" in rig.nav.calls
    assert "eta" in rig.nav.calls
    assert rig.mission.asked and rig.mission.asked[0].task_id == TASK_ID

    # And nothing has gone on the air yet: the backoff is still running.
    assert rig.rel.bids == []
    assert len(rig.session.pending_bids) == 1

    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)

    bids = rig.rel.bids
    assert len(bids) == 1
    assert bids[0].task_id == TASK_ID
    assert bids[0].feasible is True
    assert bids[0].eta_s == 60
    assert rig.session.pending_bids == []


def test_a_better_fit_waits_less_than_a_worse_one():
    """The ordering on the air has to be the ordering of the bids."""
    near = make_rig(nav=FakeNav(eta_sec=10.0))
    far = make_rig(nav=FakeNav(eta_sec=900.0))
    task = a_task()

    near.announce_from(PEER_A, task)
    far.announce_from(PEER_A, task)

    near_bid = near.session.pending_bids[0]
    far_bid = far.session.pending_bids[0]
    assert near_bid.fitness.cost < far_bid.fitness.cost
    assert near_bid.send_at < far_bid.send_at


def test_we_stay_silent_about_a_capability_we_do_not_have():
    rig = make_rig(capabilities=(Capability.DRIVE,))
    rig.announce_from(PEER_A, a_task(capability=Capability.SPRAY))

    rig.advance(5.0)
    assert rig.rel.bids == []
    assert rig.session.stats()["bids_incapable"] == 1


def test_a_task_the_mission_cannot_absorb_gets_an_infeasible_bid():
    """Silence and "I cannot" are different facts, and the wire carries both."""
    rig = make_rig(mission=FakeMission(can_absorb=False))
    rig.announce_from(PEER_A, a_task())

    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)

    bids = rig.rel.bids
    assert len(bids) == 1
    assert bids[0].feasible is False


def test_an_unreachable_task_gets_an_infeasible_bid():
    rig = make_rig(nav=FakeNav(reachable=False))
    rig.announce_from(PEER_A, a_task())
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)

    assert rig.rel.bids[0].feasible is False


def test_a_repeated_announcement_does_not_produce_a_second_bid():
    task = a_task()
    rig = make_rig()
    rig.announce_from(PEER_A, task)
    rig.announce_from(PEER_A, task)

    assert len(rig.session.pending_bids) == 1
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1


# ==========================================================================
# Group 5: suppression
# ==========================================================================


def test_a_better_bid_overheard_during_the_backoff_suppresses_our_own():
    rig = make_rig(nav=FakeNav(eta_sec=600.0))
    rig.announce_from(PEER_A, a_task())
    ours = rig.session.pending_bids[0]
    assert ours.fitness.cost > 0

    rig.bid_from(PEER_B, cost=ours.fitness.cost - 1, eta_s=60)

    assert rig.session.pending_bids == []
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert rig.rel.bids == [], "the suppressed bid must never reach the air"
    assert rig.session.stats()["bids_suppressed"] == 1


def test_a_worse_bid_overheard_during_the_backoff_does_not_suppress_ours():
    rig = make_rig(nav=FakeNav(eta_sec=10.0))
    rig.announce_from(PEER_A, a_task())
    ours = rig.session.pending_bids[0]

    rig.bid_from(PEER_B, cost=ours.fitness.cost + 50, eta_s=900)

    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1
    assert rig.session.stats()["bids_suppressed"] == 0


def test_suppression_uses_the_same_ordering_the_announcer_arbitrates_with():
    """A tie on cost and ETA is broken by robot ID, on both sides.

    If a bidder suppressed itself by a different rule than the announcer
    arbitrates by, the fleet could silence the bid that would have won.
    """
    rig = make_rig(nav=FakeNav(eta_sec=60.0))
    rig.announce_from(PEER_A, a_task())
    ours = rig.session.pending_bids[0]

    # PEER_B (id 3) ties with us (id 1) on cost and ETA. We have the lower ID,
    # so we win the tiebreak and must not suppress.
    rig.bid_from(PEER_B, cost=ours.fitness.cost, eta_s=ours.fitness.eta_s)

    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1


def test_a_bid_already_on_the_air_is_not_retracted():
    rig = make_rig(nav=FakeNav(eta_sec=600.0))
    rig.announce_from(PEER_A, a_task())
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1

    rig.bid_from(PEER_B, cost=1)

    assert len(rig.rel.bids) == 1
    assert rig.session.stats()["bids_suppressed"] == 0


def test_an_infeasible_bid_is_never_suppressed():
    """Its whole purpose is to be an answer the announcer can close on."""
    rig = make_rig(mission=FakeMission(can_absorb=False))
    rig.announce_from(PEER_A, a_task())

    rig.bid_from(PEER_B, cost=1)

    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1
    assert rig.rel.bids[0].feasible is False


def test_a_grant_to_someone_else_ends_our_pending_bid():
    rig = make_rig(nav=FakeNav(eta_sec=600.0))
    rig.announce_from(PEER_A, a_task())

    rig.hear(Grant(src=PEER_A, seq=2, task_id=TASK_ID, winner_id=PEER_B))

    assert rig.session.pending_bids == []
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert rig.rel.bids == []
    assert rig.session.stats()["bids_lost"] == 1


# ==========================================================================
# Group 6: the bidder wins
# ==========================================================================


def test_a_grant_addressed_to_us_absorbs_the_task_and_replans():
    task = a_task()
    rig = make_rig()
    rig.announce_from(PEER_A, task)
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    assert len(rig.rel.bids) == 1

    rig.hear(Grant(src=PEER_A, seq=2, task_id=TASK_ID, winner_id=US))

    assert [t.task_id for t in rig.mission.absorbed] == [TASK_ID]
    assert rig.replanner.calls == 1
    assert [t.task_id for t in rig.replanner.deltas[0].added] == [TASK_ID]
    assert rig.session.state_of(TASK_ID) is TaskState.OURS
    assert TASK_ID in [r.task.task_id for r in rig.session.owned_tasks]


def test_a_grant_for_a_task_we_never_bid_on_is_reported_not_silently_dropped():
    rig = make_rig()
    rig.hear(Grant(src=PEER_A, seq=2, task_id=999, winner_id=US))

    assert rig.mission.absorbed == []
    assert rig.session.stats()["grants_unmatched"] == 1
    assert any("orphaned" in w for w in rig.events.warnings)


def test_a_rejected_replan_after_absorbing_hands_the_task_back_as_an_anomaly():
    """The one branch where the verifier's answer changes what happens."""
    task = a_task()
    rig = make_rig(
        replanner=RejectEverything("violates the visit-order constraint"),
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.DROP)]
        ),
    )
    rig.announce_from(PEER_A, task)
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)

    rig.hear(Grant(src=PEER_A, seq=2, task_id=TASK_ID, winner_id=US))

    # Absorbed, rejected, released, and re-offered to the fleet.
    assert [t.task_id for t in rig.mission.absorbed] == [TASK_ID]
    assert [t.task_id for t in rig.mission.released] == [TASK_ID]
    assert len(rig.rel.announces) == 1
    assert rig.session.stats()["replans_rejected"] >= 1


# ==========================================================================
# Group 7: the peer registry
# ==========================================================================


def test_heartbeats_populate_the_registry():
    rig = make_rig()
    rig.heartbeat_from(
        PEER_A,
        capabilities=(Capability.DRIVE, Capability.SPRAY),
        row=5,
        col=6,
        battery=77,
        current_task=42,
    )
    rig.heartbeat_from(PEER_B, capabilities=(Capability.DRIVE,))

    assert rig.session.registry.ids() == (PEER_A, PEER_B)
    peer = rig.session.registry.get(PEER_A)
    assert peer.battery == 77
    assert peer.location == Location(5, 6)
    assert peer.current_task == 42
    assert peer.idle is False

    # Capability lookup is what an auction uses to know who it waits for.
    assert {p.robot_id for p in rig.session.registry.capable(Capability.SPRAY)} == {
        PEER_A
    }
    assert {p.robot_id for p in rig.session.registry.capable(Capability.DRIVE)} == {
        PEER_A,
        PEER_B,
    }


def test_a_silent_peer_ages_out():
    rig = make_rig(peer_timeout_sec=30.0)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)

    rig.advance(20.0)
    assert len(rig.session.registry) == 2

    # PEER_A keeps talking; PEER_B has gone quiet.
    rig.heartbeat_from(PEER_A)
    rig.advance(15.0)

    assert rig.session.registry.ids() == (PEER_A,)
    assert rig.session.stats()["aged_out"] == 1


def test_a_peer_that_ages_out_mid_auction_stops_holding_it_open():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
        peer_timeout_sec=1.0,
        announce_window_sec=10.0,
        bid_max_backoff_sec=2.0,
    )
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    rig.bid_from(PEER_A, cost=80)
    assert rig.rel.grants == [], "still waiting on PEER_B"

    # PEER_A keeps heartbeating; PEER_B goes quiet and drops out of the
    # registry. Its silence stops being something the auction waits for, and
    # the auction closes on the bid it has rather than on its whole window.
    rig.advance(0.8)
    rig.heartbeat_from(PEER_A)
    rig.advance(0.4)

    assert rig.session.registry.ids() == (PEER_A,)
    assert rig.rel.last_grant.dst == PEER_A


def test_a_bid_from_a_peer_that_has_gone_is_not_granted_to():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
        peer_timeout_sec=1.0,
        announce_window_sec=10.0,
    )
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.heartbeat_from(PEER_B)
    rig.session.report_infeasible(task)

    # Only PEER_A bids, and its bid is a good one. PEER_B stays alive but
    # silent, which is what keeps the auction open long enough for PEER_A to
    # disappear -- and PEER_A's offer goes with it.
    rig.bid_from(PEER_A, cost=10)
    rig.advance(0.5)
    rig.heartbeat_from(PEER_B)
    rig.advance(0.6)

    assert rig.session.registry.ids() == (PEER_B,)
    assert rig.rel.grants == [], "the only bid was from a robot that has gone"

    rig.advance(9.5)
    assert rig.rel.grants == []
    assert rig.session.stats()["auctions_no_bid"] == 1


# ==========================================================================
# Group 8: safe preemption
# ==========================================================================


def _assert_no_hard_interrupt(rig):
    assert rig.nav.hard_interrupts == [], "coordination must never stop navigation"
    assert rig.mission.hard_interrupts == [], "coordination must never abort a mission"


def test_a_transfer_raises_the_flag_and_interrupts_nothing():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([ReDelegate(task=task)]))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)
    rig.bid_from(PEER_A, cost=40)

    assert rig.preempt.requested is False, "announcing changes nothing we execute"

    rig.rel.last_grant.deliver()

    assert rig.preempt.requested is True
    assert str(TASK_ID) in rig.preempt.reason
    _assert_no_hard_interrupt(rig)


def test_absorbing_a_task_raises_the_flag_and_interrupts_nothing():
    rig = make_rig()
    rig.announce_from(PEER_A, a_task())
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    rig.hear(Grant(src=PEER_A, seq=2, task_id=TASK_ID, winner_id=US))

    assert rig.preempt.requested is True
    _assert_no_hard_interrupt(rig)


def test_the_flag_is_lowered_only_when_the_tree_says_so():
    rig = make_rig()
    rig.announce_from(PEER_A, a_task())
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)
    rig.hear(Grant(src=PEER_A, seq=2, task_id=TASK_ID, winner_id=US))
    assert rig.preempt.requested is True

    rig.advance(60.0)
    assert rig.preempt.requested is True, "time alone must not clear it"

    rig.session.clear_preemption()
    assert rig.preempt.requested is False
    assert rig.preempt.clears == 1


# ==========================================================================
# Anomaly interpretation: the stubbed reasoning point
# ==========================================================================


def test_the_interpretation_gets_a_snapshot_of_the_fleet_and_ourselves():
    task = a_task()
    interpreter = ScriptedInterpreter([ReDelegate(task=task)])
    rig = make_rig(interpreter=interpreter, mission=FakeMission(battery=42))
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)

    rig.session.report_infeasible(
        task, detail="nozzle blocked", reason_code=ReasonCode.TASK_FAILED
    )

    assert len(interpreter.calls) == 1
    context = interpreter.calls[0]
    assert context.task is task
    assert context.detail == "nozzle blocked"
    assert context.reason_code == ReasonCode.TASK_FAILED
    assert context.battery == 42
    assert {p.robot_id for p in context.peers} == {PEER_A}


def test_an_interpretation_that_fails_leaves_the_task_exactly_as_it_was():
    class Exploding:
        def interpret_anomaly(self, context):
            raise RuntimeError("the model is down")

    task = a_task()
    rig = make_rig(interpreter=Exploding())
    rig.session.own(task)

    assert rig.session.report_infeasible(task) is None
    assert rig.session.state_of(TASK_ID) is TaskState.OURS
    assert rig.rel.broadcasts == []
    assert rig.session.stats()["interpret_failed"] == 1


def test_an_interpretation_that_is_not_one_of_the_four_actions_is_refused():
    class Chatty:
        def interpret_anomaly(self, context):
            return "I think you should ask robot 2 about this"

    task = a_task()
    rig = make_rig(interpreter=Chatty())
    rig.session.own(task)

    assert rig.session.report_infeasible(task) is None
    assert rig.rel.broadcasts == [], "free text must never reach the wire"
    assert rig.session.stats()["interpret_failed"] == 1


def test_dropping_a_task_never_announces_it():
    task = a_task()
    rig = make_rig(interpreter=ScriptedInterpreter([DropTask(task=task)]))
    rig.session.own(task)

    rig.session.report_infeasible(task)

    assert rig.rel.announces == []
    assert rig.session.state_of(TASK_ID) is TaskState.RELINQUISHED
    assert rig.mission.released == [task]


def test_the_same_task_cannot_be_re_announced_inside_the_cooldown():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter(
            [ReDelegate(task=task, fallback=LocalDisposition.HOLD)]
        ),
        redelegation_cooldown_sec=30.0,
    )
    rig.session.own(task)

    rig.session.report_infeasible(task)
    rig.advance(rig.session.params.announce_window_sec + 0.1)
    assert len(rig.rel.announces) == 1

    rig.session.report_infeasible(task)
    assert len(rig.rel.announces) == 1, "inside the cooldown"
    assert rig.session.stats()["delegation_refused"] == 1

    rig.clock.advance(30.0)
    rig.session.report_infeasible(task)
    assert len(rig.rel.announces) == 2


# ==========================================================================
# Parameters and robustness
# ==========================================================================


def test_a_backoff_that_can_outlast_the_announce_window_is_refused():
    """The mechanism would order bids into an auction that had already closed."""
    with pytest.raises(ValueError, match="announce_window_sec"):
        CoordinatorParams(announce_window_sec=2.0, bid_max_backoff_sec=2.0)


def test_bid_memory_must_outlast_the_announce_window():
    with pytest.raises(ValueError, match="bid_memory_sec"):
        CoordinatorParams(announce_window_sec=5.0, bid_memory_sec=5.0)


def test_a_node_id_outside_the_fleet_range_is_refused():
    with pytest.raises(ValueError, match="node_id"):
        make_rig(node_id=0)


def test_a_message_handler_that_raises_does_not_take_coordination_down():
    rig = make_rig()

    class Wrecked:
        src = PEER_A
        seq = 1

    rig.session.on_message(Wrecked())  # not a known type; must not raise
    rig.heartbeat_from(PEER_A)
    assert len(rig.session.registry) == 1


def test_a_nav_that_is_failing_produces_an_infeasible_bid_not_an_exception():
    class BrokenNav(FakeNav):
        def eta(self, location):
            raise RuntimeError("nav2 is restarting")

    rig = make_rig(nav=BrokenNav())
    rig.announce_from(PEER_A, a_task())
    rig.advance(rig.session.params.bid_max_backoff_sec + 0.01)

    assert rig.rel.bids[0].feasible is False


def test_heartbeats_are_emitted_on_their_own_period():
    rig = make_rig(
        heartbeat_period_sec=10.0,
        mission=FakeMission(battery=64, current_task=12),
        nav=FakeNav(location=Location(7, 8)),
    )

    rig.advance(0.1)
    beats = rig.rel.heartbeats
    assert len(beats) == 1
    assert beats[0].battery == 64
    assert beats[0].cur_task == 12
    assert (beats[0].grid_row, beats[0].grid_col) == (7, 8)
    assert beats[0].cap_mask == cap_mask(Capability.DRIVE, Capability.SPRAY)

    rig.advance(5.0)
    assert len(rig.rel.heartbeats) == 1
    rig.advance(5.5)
    assert len(rig.rel.heartbeats) == 2


def test_settled_tasks_are_eventually_forgotten():
    task = a_task()
    rig = make_rig(
        interpreter=ScriptedInterpreter([ReDelegate(task=task)]),
        settled_retention_sec=100.0,
    )
    rig.session.own(task)
    rig.heartbeat_from(PEER_A)
    rig.session.report_infeasible(task)
    rig.bid_from(PEER_A, cost=40)
    rig.rel.last_grant.deliver()
    assert rig.session.state_of(TASK_ID) is TaskState.TRANSFERRED

    rig.advance(101.0)
    assert rig.session.state_of(TASK_ID) is None

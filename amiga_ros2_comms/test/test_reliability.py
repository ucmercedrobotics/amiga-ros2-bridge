#!/usr/bin/env python3
"""Acceptance tests for the reliability layer. No ROS, no serial port, no radio.

The six groups from the brief -- dedup, delivery under loss, give-up, ACK
correctness, broadcast-is-best-effort, sequence numbers -- plus the inbound
hostility cases, because this layer is the only thing between the radio and the
coordinator and "never raises" is a claim that has to be tested.

Everything runs over the deterministic lossy link in lossy_link.py: loss is a
predicate, time is a float, and nothing sleeps.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amiga_ros2_comms.codec import (  # noqa: E402
    SEQ_MAX,
    Ack,
    Bid,
    Capability,
    Freeform,
    Grant,
    Heartbeat,
    MessageType,
    ReasonCode,
    Target,
    TaskAnnounce,
    cap_mask,
    encode,
    target_fields,
)
from amiga_ros2_comms.reliability import (  # noqa: E402
    BROADCAST,
    BULK,
    UNICAST,
    URGENT,
    DedupCache,
    NoteTooLong,
    Outcome,
    ReliabilityError,
    ReliabilityParams,
    ReliabilitySession,
    addressee_of,
    addressing_of,
    is_reliable,
    priority_of,
    urgent_types,
)
from lossy_link import Fleet, Medium  # noqa: E402

# Two robots for most tests: 1 grants, 2 works.
GRANTOR = 1
WINNER = 2
BYSTANDER = 3


def a_grant(task_id=7, winner=WINNER):
    # src/seq are placeholders; the layer overwrites both, which is asserted in
    # test_send_overwrites_caller_supplied_identity.
    return Grant(src=0, seq=0, task_id=task_id, winner_id=winner)


def a_heartbeat(battery=88):
    return Heartbeat(
        src=0,
        seq=0,
        cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
        **target_fields(Target.gps(37.366449, -120.423065)),
        battery=battery,
        cur_task=0,
    )


def an_announce(task_id=7):
    return TaskAnnounce(
        src=0,
        seq=0,
        task_id=task_id,
        req_cap_mask=cap_mask(Capability.MOVE_TO_TREE_ID, Capability.SAMPLE_LEAF),
        **target_fields(Target.tree(60)),
        priority=200,
        reason_code=ReasonCode.OPERATOR_REQUEST,
    )


BROADCAST_FACTORIES = (a_heartbeat, an_announce)


# ==========================================================================
# Group 1: dedup
# ==========================================================================


def test_the_same_message_received_n_times_is_delivered_once():
    fleet = Fleet(GRANTOR, WINNER)
    announce = an_announce()
    fleet[GRANTOR].send_broadcast(announce)
    payload = fleet.medium.frames[0].payload

    copies = 10
    for _ in range(copies):
        fleet[WINNER].on_frame(payload)

    delivered = fleet.inbox[WINNER].messages
    assert len(delivered) == 1
    assert delivered[0].task_id == announce.task_id
    assert fleet[WINNER].stats()["rx_duplicates"] == copies - 1


def test_distinct_messages_are_all_delivered():
    fleet = Fleet(GRANTOR, WINNER)
    for task_id in range(1, 6):
        fleet[GRANTOR].send_broadcast(an_announce(task_id))
    fleet.medium.pump()

    delivered = fleet.inbox[WINNER].messages
    assert [m.task_id for m in delivered] == [1, 2, 3, 4, 5]
    assert len({m.seq for m in delivered}) == 5


def test_a_duplicate_older_than_the_dedup_ttl_is_delivered_again():
    # Not a defect, a documented bound: the cache is finite in time, so the TTL
    # has to outlast anything that could still produce a copy. Pinning the
    # behaviour here is what makes the parameter's meaning testable.
    fleet = Fleet(GRANTOR, WINNER, params=ReliabilityParams(dedup_ttl_sec=30.0))
    fleet[GRANTOR].send_broadcast(an_announce())
    payload = fleet.medium.frames[0].payload

    fleet[WINNER].on_frame(payload)
    assert len(fleet.inbox[WINNER].messages) == 1

    fleet.clock.advance(29.0)
    fleet[WINNER].on_frame(payload)
    assert len(fleet.inbox[WINNER].messages) == 1, "still inside the TTL"

    # The refresh from that second copy moves the expiry, so clear both.
    fleet.clock.advance(31.0)
    fleet[WINNER].on_frame(payload)
    assert len(fleet.inbox[WINNER].messages) == 2


def test_the_dedup_cache_is_bounded_and_reports_eviction():
    cache = DedupCache(ttl_sec=1000.0, max_entries=8)
    for seq in range(50):
        assert cache.first_sight((1, seq), now=0.0)

    assert len(cache) == 8
    assert cache.evicted == 42
    assert (1, 49) in cache
    assert (1, 0) not in cache


def test_dedup_refreshes_the_ttl_so_an_ongoing_campaign_stays_suppressed():
    cache = DedupCache(ttl_sec=10.0, max_entries=64)
    assert cache.first_sight((1, 5), now=0.0)
    for now in range(1, 100):
        # A sender retransmitting every second for far longer than the TTL.
        assert not cache.first_sight((1, 5), now=float(now))
    assert cache.duplicates == 99


# ==========================================================================
# Group 2: reliable delivery under loss
# ==========================================================================


def test_a_lost_grant_is_retransmitted_and_eventually_delivered():
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_kind("Grant", count=1)

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()

    assert not future.done(), "the only copy was dropped"
    assert fleet.inbox[WINNER].messages == []

    fleet.advance(fleet.params.retransmit_timeout_sec + 0.5)

    assert future.done()
    assert future.result() is Outcome.DELIVERED
    assert len(fleet.medium.sent("Grant")) == 2, "original plus one retransmit"
    assert len(fleet.inbox[WINNER].messages) == 1
    assert fleet[GRANTOR].pending == 0


def test_a_grant_that_survives_needs_no_retransmit():
    fleet = Fleet(GRANTOR, WINNER)
    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()

    assert future.result() is Outcome.DELIVERED
    assert fleet[GRANTOR].stats()["tx_retransmits"] == 0

    # And the timer is gone, so waiting out several timeouts adds nothing.
    fleet.advance(30.0)
    assert len(fleet.medium.sent("Grant")) == 1


def test_a_lost_ack_makes_the_sender_retransmit_and_the_receiver_re_ack():
    # The subtle one. The GRANT arrived and was delivered; only the ACK was
    # lost. The receiver must answer the retransmit even though it recognises
    # the message, or the sender declares a delivered task failed and the task
    # is orphaned.
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_kind("Ack", count=1)

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    assert len(fleet.inbox[WINNER].messages) == 1
    assert not future.done()

    fleet.advance(fleet.params.retransmit_timeout_sec + 0.5)

    assert future.result() is Outcome.DELIVERED
    assert len(fleet.medium.sent("Ack")) == 2, "the duplicate was ACKed again"
    assert len(fleet.inbox[WINNER].messages) == 1, "but delivered only once"


def test_delivery_survives_heavy_random_loss():
    # A soak, not an assertion about a particular frame: with retries to spare,
    # a 40%-loss channel still lands the grant.
    fleet = Fleet(
        GRANTOR,
        WINNER,
        params=ReliabilityParams(
            retransmit_timeout_sec=1.0, max_retries=10, retransmit_backoff=1.0
        ),
    )
    fleet.medium.drop_rule = Medium.drop_random(rate=0.4, seed=7)

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    fleet.advance(20.0)

    assert future.result() is Outcome.DELIVERED
    assert len(fleet.inbox[WINNER].messages) == 1, "exactly once, despite the churn"


# ==========================================================================
# Group 3: give up rather than hang
# ==========================================================================


def test_a_grant_nobody_answers_is_reported_failed_not_left_hanging():
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_all

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    fleet.advance(60.0)

    assert future.done(), "must resolve, not hang"
    assert future.result() is Outcome.FAILED
    assert fleet[GRANTOR].pending == 0
    assert fleet[GRANTOR].stats()["tx_failed"] == 1


def test_exactly_max_retries_retransmissions_are_sent_before_giving_up():
    params = ReliabilityParams(
        retransmit_timeout_sec=1.0, max_retries=3, retransmit_backoff=1.0
    )
    fleet = Fleet(GRANTOR, WINNER, params=params)
    fleet.medium.drop_rule = Medium.drop_all

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    fleet.advance(60.0)

    assert future.result() is Outcome.FAILED
    # max_retries counts retransmits, so the original makes four copies.
    assert len(fleet.medium.sent("Grant")) == params.max_retries + 1
    assert fleet[GRANTOR].stats()["tx_retransmits"] == params.max_retries


def test_max_retries_zero_gives_up_after_the_first_transmission():
    params = ReliabilityParams(retransmit_timeout_sec=1.0, max_retries=0)
    fleet = Fleet(GRANTOR, WINNER, params=params)
    fleet.medium.drop_rule = Medium.drop_all

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.advance(10.0)

    assert future.result() is Outcome.FAILED
    assert len(fleet.medium.sent("Grant")) == 1


def test_backoff_stretches_the_gap_between_retransmissions():
    params = ReliabilityParams(
        retransmit_timeout_sec=1.0,
        max_retries=3,
        retransmit_backoff=2.0,
        max_retransmit_timeout_sec=100.0,
    )
    fleet = Fleet(GRANTOR, WINNER, params=params)
    fleet.medium.drop_rule = Medium.drop_all
    sent_at = []

    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    seen = 0
    for _ in range(2000):
        fleet.clock.advance(0.05)
        fleet[GRANTOR].tick()
        if len(fleet.medium.sent("Grant")) > seen:
            seen = len(fleet.medium.sent("Grant"))
            sent_at.append(fleet.clock.now)

    gaps = [round(b - a, 2) for a, b in zip(sent_at, sent_at[1:])]
    assert len(gaps) == 3
    # 1s, then 2s, then 4s, within one tick of granularity.
    for gap, want in zip(gaps, (1.0, 2.0, 4.0)):
        assert want <= gap <= want + 0.1, f"gap {gap} is not ~{want}"


def test_the_backoff_ceiling_bounds_the_interval():
    params = ReliabilityParams(
        retransmit_timeout_sec=1.0,
        max_retries=8,
        retransmit_backoff=3.0,
        max_retransmit_timeout_sec=5.0,
    )
    session = ReliabilitySession(
        node_id=GRANTOR, send_frame=lambda payload, priority: None, params=params
    )
    intervals = [session._interval(n) for n in range(1, 9)]
    assert intervals[0] == 3.0
    assert max(intervals) == 5.0


def test_a_full_pending_table_rejects_rather_than_grows():
    # A timeout long enough that nothing resolves or expires during the test:
    # the table fills and stays full.
    params = ReliabilityParams(
        max_pending=4,
        retransmit_timeout_sec=100.0,
        max_retransmit_timeout_sec=100.0,
    )
    fleet = Fleet(GRANTOR, WINNER, params=params)
    fleet.medium.drop_rule = Medium.drop_all

    futures = [
        fleet[GRANTOR].send_reliable(WINNER, a_grant(task_id=n)) for n in range(1, 9)
    ]

    assert fleet[GRANTOR].pending == 4
    assert [f.done() for f in futures] == [False] * 4 + [True] * 4
    assert all(f.result() is Outcome.FAILED for f in futures[4:])
    assert fleet[GRANTOR].stats()["tx_rejected"] == 4


# ==========================================================================
# Group 4: ACK correctness
# ==========================================================================


def test_a_reliable_unicast_produces_exactly_one_ack_for_the_right_id():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    grant = fleet.medium.sent("Grant")[0].msg
    fleet.medium.pump()

    acks = fleet.medium.sent("Ack")
    assert len(acks) == 1
    ack = acks[0].msg
    assert ack.src == WINNER, "the ACK comes from the addressee"
    assert (ack.ack_src, ack.ack_seq) == (grant.src, grant.seq)


def test_acks_are_never_acked():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    fleet.advance(30.0)

    # One ACK on the air, ever. If ACKs were ACKed this would not terminate,
    # and Medium.pump would have raised long before the assertion.
    assert len(fleet.medium.sent("Ack")) == 1


def test_acks_are_never_retransmitted_and_never_go_on_a_timer():
    fleet = Fleet(GRANTOR, WINNER)
    # The ACK is lost, so if this layer treated it as reliable it would retry.
    fleet.medium.drop_rule = Medium.drop_kind("Ack")

    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    assert fleet[WINNER].pending == 0, "the ACKer has nothing outstanding"

    fleet.advance(30.0)
    # Every ACK on the air is a fresh answer to a GRANT retransmit, never a
    # retransmit of an ACK: there are as many ACKs as GRANTs received.
    assert fleet[WINNER].stats()["tx_retransmits"] == 0
    assert len(fleet.medium.sent("Ack")) == len(fleet.medium.sent("Grant"))


def test_acks_are_not_delivered_up_to_the_coordinator():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()

    # The grantor's outcome is the Future, not an inbound ACK message.
    assert fleet.inbox[GRANTOR].messages == []
    assert fleet[GRANTOR].stats()["rx_acks"] == 1


def test_a_broadcast_is_never_acked():
    fleet = Fleet(GRANTOR, WINNER, BYSTANDER)
    for factory in BROADCAST_FACTORIES:
        fleet[GRANTOR].send_broadcast(factory())
    fleet.medium.pump()

    assert fleet.medium.sent("Ack") == []
    assert len(fleet.inbox[WINNER].messages) == len(BROADCAST_FACTORIES)
    assert len(fleet.inbox[BYSTANDER].messages) == len(BROADCAST_FACTORIES)


def test_a_grant_for_someone_else_is_overheard_but_not_acked():
    # GRANT is addressed to its winner and reliable to that robot alone, but
    # everyone hears it -- which is how losing bidders learn the auction closed.
    # Addressing decides who owes an ACK, not who may listen.
    fleet = Fleet(GRANTOR, WINNER, BYSTANDER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant(winner=WINNER))
    fleet.medium.pump()

    assert len(fleet.inbox[BYSTANDER].messages) == 1, "the bystander overhears it"
    assert len(fleet.medium.sent("Ack")) == 1
    assert fleet.medium.sent("Ack")[0].msg.src == WINNER, "only the winner ACKs"


def test_an_ack_from_the_wrong_robot_does_not_confirm_delivery():
    # "Delivered" has to mean the intended owner has it, or the ownership
    # guarantee built on top of it means nothing.
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_kind("Grant")

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    grant = fleet.medium.sent("Grant")[0].msg

    forged = encode(Ack(src=BYSTANDER, seq=1, ack_src=grant.src, ack_seq=grant.seq))
    fleet[GRANTOR].on_frame(forged)

    assert not future.done()
    assert fleet[GRANTOR].stats()["ack_wrong_src"] == 1


def test_an_ack_for_a_message_we_never_sent_is_counted_and_ignored():
    fleet = Fleet(GRANTOR, WINNER)
    stray = encode(Ack(src=WINNER, seq=1, ack_src=GRANTOR, ack_seq=999))
    fleet[GRANTOR].on_frame(stray)

    assert fleet[GRANTOR].stats()["ack_unmatched"] == 1
    assert fleet.inbox[GRANTOR].messages == []


def test_another_robots_ack_overheard_is_ignored_silently():
    fleet = Fleet(GRANTOR, WINNER, BYSTANDER)
    overheard = encode(Ack(src=WINNER, seq=1, ack_src=BYSTANDER, ack_seq=4))
    fleet[GRANTOR].on_frame(overheard)

    stats = fleet[GRANTOR].stats()
    assert stats["rx_acks"] == 1
    assert stats["ack_unmatched"] == 0, "not ours to match"
    assert stats["ack_wrong_src"] == 0


# ==========================================================================
# Group 5: broadcast is best-effort
# ==========================================================================


@pytest.mark.parametrize("factory", BROADCAST_FACTORIES, ids=lambda f: f.__name__)
def test_a_broadcast_is_sent_once_and_never_put_on_a_timer(factory):
    fleet = Fleet(GRANTOR, WINNER)
    assert fleet[GRANTOR].send_broadcast(factory())
    fleet.medium.pump()

    assert fleet[GRANTOR].pending == 0
    before = len(fleet.medium.frames)
    fleet.advance(60.0)
    assert len(fleet.medium.frames) == before, "no retransmit, ever"
    assert fleet[GRANTOR].stats()["tx_retransmits"] == 0


@pytest.mark.parametrize("factory", BROADCAST_FACTORIES, ids=lambda f: f.__name__)
def test_a_lost_broadcast_is_simply_lost(factory):
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_all

    assert fleet[GRANTOR].send_broadcast(factory())
    fleet.advance(60.0)

    assert fleet.inbox[WINNER].messages == []
    assert fleet[GRANTOR].stats()["tx_retransmits"] == 0


def test_an_app_level_rebroadcast_is_still_deduped_to_one_delivery():
    # The coordinator re-emitting an announce is the *intended* reliability
    # mechanism for broadcasts. Receivers must collapse the copies.
    fleet = Fleet(GRANTOR, WINNER)
    announce = an_announce()
    fleet[GRANTOR].send_broadcast(announce)
    payload = fleet.medium.frames[0].payload

    for _ in range(5):
        fleet[WINNER].on_frame(payload)

    assert len(fleet.inbox[WINNER].messages) == 1


def test_broadcast_types_refuse_to_be_sent_reliably():
    fleet = Fleet(GRANTOR, WINNER)
    for factory in BROADCAST_FACTORIES:
        with pytest.raises(ReliabilityError, match="broadcast type"):
            fleet[GRANTOR].send_reliable(WINNER, factory())


def test_unicast_types_refuse_to_be_sent_best_effort():
    fleet = Fleet(GRANTOR, WINNER)
    with pytest.raises(ReliabilityError, match="single robot"):
        fleet[GRANTOR].send_broadcast(a_grant())


def test_the_caller_cannot_send_acks():
    fleet = Fleet(GRANTOR, WINNER)
    ack = Ack(src=0, seq=0, ack_src=1, ack_seq=1)
    with pytest.raises(ReliabilityError, match="emitted by this layer"):
        fleet[GRANTOR].send_broadcast(ack)
    with pytest.raises(ReliabilityError, match="emitted by this layer"):
        fleet[GRANTOR].send_reliable(WINNER, ack)


def test_addressing_table_matches_the_message_vocabulary():
    # The table is the design rule in code form, so it is asserted directly:
    # GRANT is the only built unicast type, and it addresses its winner.
    assert addressing_of(a_grant(winner=9)) == UNICAST
    assert addressee_of(a_grant(winner=9)) == 9
    assert is_reliable(a_grant())

    for factory in BROADCAST_FACTORIES:
        msg = factory()
        assert addressing_of(msg) == BROADCAST
        assert addressee_of(msg) is None
        assert not is_reliable(msg)

    # An ACK is unicast in spirit but must never be treated as reliable.
    assert not is_reliable(Ack(src=1, seq=1, ack_src=1, ack_seq=1))


def test_a_grant_whose_winner_disagrees_with_dst_is_refused():
    # Two ways of saying where a message is going, and they must agree: a GRANT
    # one robot acts on and another acknowledges is the double-assignment this
    # layer exists to prevent.
    fleet = Fleet(GRANTOR, WINNER)
    with pytest.raises(ReliabilityError, match="addressed to"):
        fleet[GRANTOR].send_reliable(WINNER, a_grant(winner=BYSTANDER))


# ==========================================================================
# Group 6: sequence numbers
# ==========================================================================


def test_outgoing_seq_is_monotonic_per_sender():
    fleet = Fleet(GRANTOR, WINNER)
    for n in range(50):
        fleet[GRANTOR].send_broadcast(an_announce(task_id=n + 1))

    seqs = [f.msg.seq for f in fleet.medium.sent(sender=GRANTOR)]
    assert seqs == sorted(seqs)
    assert len(set(seqs)) == len(seqs)
    assert seqs == list(range(50))


def test_every_sender_numbers_independently():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_broadcast(an_announce(1))
    fleet[WINNER].send_broadcast(an_announce(2))
    fleet[GRANTOR].send_broadcast(an_announce(3))

    by_sender = {}
    for frame in fleet.medium.frames:
        by_sender.setdefault(frame.sender, []).append(frame.msg.seq)
    assert by_sender[GRANTOR] == [0, 1]
    assert by_sender[WINNER] == [0]


def test_seq_wraps_at_sixteen_bits_without_repeating_early():
    session = ReliabilitySession(
        node_id=GRANTOR, send_frame=lambda payload, priority: None
    )
    session._next_seq = SEQ_MAX - 1

    assert session.next_seq() == SEQ_MAX - 1
    assert session.next_seq() == SEQ_MAX
    assert session.next_seq() == 0, "wraps, rather than overflowing its field"
    assert session.next_seq() == 1


def test_acks_consume_sequence_numbers_from_the_same_space():
    # ACKs carry a (src, seq) of their own like every other message, so the
    # counter must be shared -- an ACK reusing a GRANT's seq would make the ID
    # ambiguous for anyone deduping our traffic.
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()
    fleet[WINNER].send_broadcast(a_heartbeat())

    seqs = [f.msg.seq for f in fleet.medium.sent(sender=WINNER)]
    assert seqs == [0, 1], "the ACK took seq 0, the heartbeat took 1"


def test_send_overwrites_caller_supplied_identity():
    fleet = Fleet(GRANTOR, WINNER)
    msg = Grant(src=99, seq=12345, task_id=7, winner_id=WINNER)
    fleet[GRANTOR].send_reliable(WINNER, msg)

    on_air = fleet.medium.sent("Grant")[0].msg
    assert on_air.src == GRANTOR
    assert on_air.seq == 0
    assert msg.src == GRANTOR and msg.seq == 0, "the caller's object is updated too"


# ==========================================================================
# Inbound robustness: this layer is the last thing before the coordinator
# ==========================================================================


@pytest.mark.parametrize(
    "payload, counter",
    [
        (b"", "rx_malformed"),
        (b"\x01", "rx_malformed"),
        (bytes([MessageType.HEARTBEAT, 1, 0, 1]), "rx_malformed"),
        (bytes([MessageType.HEARTBEAT, 1, 0, 1]) + b"\x00" * 20, "rx_malformed"),
        (bytes([0x7F, 1, 0, 1, 2, 3]), "rx_unknown_type"),
        # A FREEFORM shorter than its own fixed fields. Now that the tag is
        # built this is a truncated message rather than a reserved one, and it
        # is still counted and dropped rather than raised.
        (bytes([MessageType.FREEFORM, 1, 0, 1, 2, 3]), "rx_malformed"),
    ],
)
def test_garbage_on_the_air_is_counted_and_dropped_not_raised(payload, counter):
    fleet = Fleet(GRANTOR, WINNER)
    fleet[WINNER].on_frame(payload)  # must not raise

    assert fleet.inbox[WINNER].messages == []
    assert fleet[WINNER].stats()[counter] == 1


def test_a_field_that_cannot_mean_what_it_says_is_rejected():
    # A structurally valid HEARTBEAT claiming 200% battery. The codec catches
    # it; this layer must count it rather than hand it up.
    fleet = Fleet(GRANTOR, WINNER)
    payload = bytes([MessageType.HEARTBEAT, 1, 0, 1]) + bytes(
        [0, 3, 0, 12, 0, 47, 200, 0, 0]
    )
    fleet[WINNER].on_frame(payload)

    assert fleet.inbox[WINNER].messages == []
    assert fleet[WINNER].stats()["rx_malformed"] == 1


def test_our_own_transmission_echoed_back_is_ignored():
    # A repeater or a loopback misconfiguration. Self-delivery would have us
    # ACK ourselves and dedup our own traffic.
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    own = fleet.medium.frames[0].payload

    fleet[GRANTOR].on_frame(own)

    assert fleet.inbox[GRANTOR].messages == []
    assert fleet[GRANTOR].stats()["rx_self"] == 1
    assert fleet.medium.sent("Ack") == []


def test_a_raising_deliver_callback_does_not_break_the_session():
    def explode(msg):
        raise RuntimeError("the coordinator had a bad day")

    fleet = Fleet(GRANTOR, WINNER)
    fleet[WINNER].set_on_deliver(explode)

    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()  # must not raise

    # The ACK still went out: transport obligations do not depend on the layer
    # above being well behaved.
    assert len(fleet.medium.sent("Ack")) == 1


def test_a_link_that_throws_is_reported_not_propagated():
    def broken(payload, priority):
        raise OSError("radio unplugged")

    session = ReliabilitySession(node_id=GRANTOR, send_frame=broken)
    assert session.send_broadcast(a_heartbeat()) is False
    assert session.stats()["tx_link_errors"] == 1


def test_sending_from_inside_a_deliver_callback_does_not_deadlock():
    # The coordinator's natural shape: hear an announce, bid on it. If the
    # session held its lock across on_deliver this would hang.
    fleet = Fleet(GRANTOR, WINNER)

    def bid_on_it(msg):
        if isinstance(msg, TaskAnnounce):
            fleet[WINNER].send_broadcast(
                Bid(src=0, seq=0, task_id=msg.task_id, eta_s=40, feasible=True, cost=3)
            )

    fleet[WINNER].set_on_deliver(bid_on_it)
    fleet[GRANTOR].send_broadcast(an_announce(task_id=7))
    fleet.medium.pump()

    bids = fleet.inbox[GRANTOR].of_kind(Bid)
    assert len(bids) == 1
    assert bids[0].task_id == 7


def test_a_done_callback_on_the_future_runs_when_delivery_resolves():
    fleet = Fleet(GRANTOR, WINNER)
    seen = []

    future = fleet[GRANTOR].send_reliable(WINNER, a_grant())
    future.add_done_callback(lambda f: seen.append(f.result()))
    fleet.medium.pump()

    assert seen == [Outcome.DELIVERED]


# ==========================================================================
# Construction
# ==========================================================================


@pytest.mark.parametrize("node_id", [0, -1, 256])
def test_a_node_id_outside_the_fleet_address_space_is_refused(node_id):
    with pytest.raises(ValueError, match="node_id"):
        ReliabilitySession(node_id=node_id, send_frame=lambda payload, priority: None)


@pytest.mark.parametrize("dst", [0, 256])
def test_an_unaddressable_destination_is_refused(dst):
    session = ReliabilitySession(
        node_id=GRANTOR, send_frame=lambda payload, priority: None
    )
    with pytest.raises(ReliabilityError, match="dst"):
        session.send_reliable(dst, a_grant(winner=dst if 0 < dst < 256 else WINNER))


@pytest.mark.parametrize(
    "kwargs",
    [
        {"retransmit_timeout_sec": 0},
        {"retransmit_timeout_sec": -1.0},
        {"max_retries": -1},
        {"retransmit_backoff": 0.5},
        {"max_pending": 0},
        {"retransmit_timeout_sec": 20.0, "max_retransmit_timeout_sec": 5.0},
    ],
)
def test_incoherent_parameters_are_refused_at_construction(kwargs):
    with pytest.raises(ValueError):
        ReliabilityParams(**kwargs)


@pytest.mark.parametrize("kwargs", [{"ttl_sec": 0}, {"max_entries": 0}])
def test_incoherent_dedup_bounds_are_refused(kwargs):
    settings = {"ttl_sec": 10.0, "max_entries": 10}
    settings.update(kwargs)
    with pytest.raises(ValueError):
        DedupCache(**settings)


def test_stats_are_flat_scalars_for_the_log_line():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()

    stats = fleet[GRANTOR].stats()
    assert all(isinstance(v, (int, float)) for v in stats.values())
    assert stats["tx_reliable"] == 1
    assert stats["tx_delivered"] == 1
    assert "dedup_size" in stats


# ==========================================================================
# Transmit priority
#
# The layer does not act on the class it assigns -- it labels a frame and hands
# it down, and the bridge's outbound queue is what orders them. So what is
# testable here is exactly the labelling, and the property worth pinning is that
# the two messages holding task ownership together are the two that get to jump
# the queue.
# ==========================================================================


def test_ack_and_grant_are_the_urgent_types():
    assert set(urgent_types()) == {Ack, Grant}


@pytest.mark.parametrize(
    "factory, expected",
    [
        (a_grant, URGENT),
        (a_heartbeat, BULK),
        (an_announce, BULK),
    ],
    ids=lambda value: getattr(value, "__name__", value),
)
def test_priority_of_each_built_type(factory, expected):
    assert priority_of(factory()) == expected


def test_a_grant_goes_out_urgent():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())

    grants = fleet.medium.sent("Grant")
    assert [frame.priority for frame in grants] == [URGENT]


@pytest.mark.parametrize("factory", BROADCAST_FACTORIES, ids=lambda f: f.__name__)
def test_broadcasts_go_out_bulk(factory):
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_broadcast(factory())

    sent = fleet.medium.sent(type(factory()).__name__)
    assert [frame.priority for frame in sent] == [BULK]


def test_an_ack_goes_out_urgent():
    """The frame whose whole value is its timing.

    The sender is sitting on a retransmit timer while this is queued; an ACK
    that arrives after the timer has given up is a duplicate of a send already
    recorded as failed.
    """
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.medium.pump()

    acks = fleet.medium.sent("Ack")
    assert acks, "the winner should have acknowledged"
    assert all(frame.priority == URGENT for frame in acks)


def test_a_retransmit_keeps_the_class_of_the_message_it_repeats():
    """A GRANT that already fell behind is if anything more urgent, not less."""
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = Medium.drop_kind("Grant", count=2)
    fleet[GRANTOR].send_reliable(WINNER, a_grant())
    fleet.advance(6.0)

    grants = fleet.medium.sent("Grant")
    assert len(grants) > 1, "expected at least one retransmit"
    assert all(frame.priority == URGENT for frame in grants)


# ==========================================================================
# Notes: the one thing here that spans several packets
#
# Fragments are broadcast, therefore unACKed, therefore a note with a hole in
# it is dropped rather than repaired. That is only acceptable because the
# announcement a note accompanies carries the requirement by itself -- so the
# tests that matter most here are the ones about loss.
# ==========================================================================

CAVEAT = "soft ground past the gate, the row is muddy and the arm needs the long reach"


def test_a_note_goes_out_as_several_broadcast_fragments():
    fleet = Fleet(GRANTOR, WINNER)
    count = fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)

    assert count > 1
    fragments = [f for f in fleet.medium.frames if isinstance(f.msg, Freeform)]
    assert len(fragments) == count
    # Distinct sequence numbers, so dedup collapses repeats of a fragment
    # rather than collapsing the fragments into one another.
    assert len({f.msg.seq for f in fragments}) == count
    assert {f.msg.task_id for f in fragments} == {77}


def test_every_note_fragment_is_bulk():
    # ACKs and GRANTs must overtake a note train, or a robot part-way through
    # one cannot confirm a handoff.
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)

    fragments = [f for f in fleet.medium.frames if isinstance(f.msg, Freeform)]
    assert {f.priority for f in fragments} == {BULK}


def test_a_note_arrives_whole_and_never_as_a_message():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()

    (note,) = fleet.notes[WINNER]
    assert note.text == CAVEAT
    assert (note.src, note.task_id) == (GRANTOR, 77)
    # A fragment is not traffic for the coordinator.
    assert fleet.inbox[WINNER].messages == []
    assert fleet[WINNER].stats()["rx_delivered"] == 0


def test_a_note_missing_a_fragment_is_never_delivered():
    fleet = Fleet(GRANTOR, WINNER)
    fleet.medium.drop_rule = lambda frame: (
        isinstance(frame.msg, Freeform) and frame.msg.frag_index == 1
    )
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()

    assert fleet.notes[WINNER] == []
    assert fleet[WINNER].completed_note(GRANTOR, 77) is None


def test_an_incomplete_note_is_eventually_abandoned_and_counted():
    fleet = Fleet(GRANTOR, WINNER, params=ReliabilityParams(note_ttl_sec=10.0))
    fleet.medium.drop_rule = lambda frame: (
        isinstance(frame.msg, Freeform) and frame.msg.frag_index == 0
    )
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()

    fleet.advance(20.0)
    assert fleet[WINNER].stats()["notes_abandoned"] == 1
    assert fleet[WINNER].stats()["notes_complete"] == 0


def test_a_completed_note_is_there_for_the_announcement_that_follows():
    # The lookup that justifies sending notes before their announcement.
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()

    assert fleet[WINNER].completed_note(GRANTOR, 77).text == CAVEAT
    assert fleet[WINNER].completed_note(GRANTOR, 4242) is None
    assert fleet[WINNER].completed_note(99, 77) is None


def test_a_note_is_delivered_once_however_often_its_fragments_repeat():
    fleet = Fleet(GRANTOR, WINNER)
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()
    # Replay every fragment, as a repeater would.
    for frame in list(fleet.medium.frames):
        if isinstance(frame.msg, Freeform):
            fleet[WINNER].on_frame(frame.payload)

    assert len(fleet.notes[WINNER]) == 1
    assert fleet[WINNER].stats()["rx_duplicates"] > 0


def test_a_note_too_long_for_the_cap_is_refused_by_the_session():
    fleet = Fleet(GRANTOR, WINNER)
    with pytest.raises(NoteTooLong):
        fleet[GRANTOR].send_note(task_id=77, text="x" * 4000)
    assert [f for f in fleet.medium.frames if isinstance(f.msg, Freeform)] == []


def test_a_callback_that_raises_on_a_note_does_not_take_the_session_down():
    fleet = Fleet(GRANTOR, WINNER)

    def explode(note):
        raise RuntimeError("the coordinator's problem, not ours")

    fleet[WINNER].set_on_note(explode)
    fleet[GRANTOR].send_note(task_id=77, text=CAVEAT)
    fleet.medium.pump()  # must not raise

    # And the note is still in the lookup, so the announce path still finds it.
    assert fleet[WINNER].completed_note(GRANTOR, 77).text == CAVEAT

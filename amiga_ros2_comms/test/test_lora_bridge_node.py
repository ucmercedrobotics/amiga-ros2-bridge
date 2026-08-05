#!/usr/bin/env python3
"""Acceptance tests for the LoRa bridge node. All run without a radio.

1. Loopback   - two bridge instances over a virtual pty pair; blobs in one end
                come out the other byte-identical.
2. Corruption - a bit flipped on the wire is dropped on CRC, never published.
3. Backpressure - flooding /lora/tx faster than the writer drains never blocks
                the publisher, and the overflow policy behaves as configured.
"""

import random
import time

import serial

from amiga_interfaces.msg import LoRaFrame

from amiga_ros2_comms.lora.framing import DELIMITER, cobs_encode, crc8, encode_frame
from amiga_ros2_comms.ring_queue import DROP_NEWEST, DROP_OLDEST
from ros_harness import Collector, Harness, make_bridge, wait_until


# ----------------------------------------------------------------------
# 1. Loopback
# ----------------------------------------------------------------------


def test_loopback_delivers_random_blobs_byte_identical(serial_pair):
    left = make_bridge(serial_pair.port_a, "left")
    right = make_bridge(serial_pair.port_b, "right")
    collector = Collector(tx_namespace="left", rx_namespace="right")
    harness = Harness(left, right, collector)
    try:
        assert wait_until(
            lambda: left.stats()["port_open"] and right.stats()["port_open"]
        )

        rng = random.Random(20260803)
        sent = []
        for _ in range(60):
            # Random lengths including the empty payload and the configured max,
            # since those are the interesting COBS and length-byte edges.
            length = rng.choice([0, 1, 2, rng.randrange(1, 200), 199, 200])
            sent.append(bytes(rng.randrange(256) for _ in range(length)))

        for payload in sent:
            collector.send(payload)
            # Paced so the flood path is not what is under test here; the
            # backpressure test covers that deliberately.
            time.sleep(0.005)

        assert wait_until(lambda: len(collector.payloads()) >= len(sent), timeout=15.0)
        assert collector.payloads() == sent

        stats = right.stats()
        assert stats["frames"] == len(sent)
        assert stats["crc_errors"] == 0
        assert stats["cobs_errors"] == 0
        assert stats["length_errors"] == 0
        assert stats["rx_dropped_queue"] == 0
    finally:
        harness.close()


def test_loopback_is_bidirectional(serial_pair):
    left = make_bridge(serial_pair.port_a, "left")
    right = make_bridge(serial_pair.port_b, "right")
    forward = Collector(tx_namespace="left", rx_namespace="right")
    backward = Collector(tx_namespace="right", rx_namespace="left")
    harness = Harness(left, right, forward, backward)
    try:
        assert wait_until(
            lambda: left.stats()["port_open"] and right.stats()["port_open"]
        )
        time.sleep(0.5)  # let subscriptions match before the first publish

        forward.send(b"left says hello")
        backward.send(b"right says hello back")

        assert wait_until(lambda: forward.payloads() and backward.payloads())
        assert forward.payloads() == [b"left says hello"]
        assert backward.payloads() == [b"right says hello back"]
    finally:
        harness.close()


def test_oversize_tx_payload_is_rejected_not_fragmented(serial_pair):
    left = make_bridge(serial_pair.port_a, "left", max_payload_bytes=64)
    right = make_bridge(serial_pair.port_b, "right", max_payload_bytes=64)
    collector = Collector(tx_namespace="left", rx_namespace="right")
    harness = Harness(left, right, collector)
    try:
        assert wait_until(
            lambda: left.stats()["port_open"] and right.stats()["port_open"]
        )
        time.sleep(0.5)

        collector.send(bytes(200))
        collector.send(b"this one fits")

        assert wait_until(lambda: collector.payloads() == [b"this one fits"])
        assert left.stats()["tx_oversize"] == 1
        # Nothing was split across frames: exactly one frame crossed the link.
        assert right.stats()["frames"] == 1
    finally:
        harness.close()


# ----------------------------------------------------------------------
# 2. Corruption
# ----------------------------------------------------------------------


def _frame_with_flipped_payload_bit(payload, byte_index, bit):
    """A frame whose CRC is valid for the original payload, then corrupted.

    Flipping after the CRC is computed and before COBS encoding aims the fault
    at the CRC check specifically, rather than at COBS structure validation.
    """
    body = bytearray(payload)
    logical = bytearray([len(body)]) + body + bytearray([crc8(bytes(body))])
    logical[1 + byte_index] ^= 1 << bit
    return cobs_encode(bytes(logical)) + bytes([DELIMITER])


def test_bit_flip_on_the_wire_is_dropped_and_never_published(serial_pair):
    bridge = make_bridge(serial_pair.port_a, "radio")
    collector = Collector(tx_namespace="radio", rx_namespace="radio")
    harness = Harness(bridge, collector)
    peer = serial.Serial(serial_pair.port_b, 115200, timeout=0.5)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])
        time.sleep(0.5)

        good_before = b"before the corruption"
        good_after = b"after the corruption"

        peer.write(encode_frame(good_before))
        peer.write(_frame_with_flipped_payload_bit(b"i have been corrupted", 5, 3))
        peer.write(encode_frame(good_after))
        peer.flush()

        assert wait_until(lambda: len(collector.payloads()) >= 2)
        time.sleep(0.5)  # give a bad frame every chance to show up late

        # The corrupt frame is absent, and the frames either side are intact:
        # a dropped frame does not desynchronise the stream.
        assert collector.payloads() == [good_before, good_after]
        stats = bridge.stats()
        assert stats["crc_errors"] == 1
        assert stats["frames"] == 2
        assert stats["rx_published"] == 2
    finally:
        peer.close()
        harness.close()


def test_random_wire_noise_is_never_published_as_a_frame(serial_pair):
    bridge = make_bridge(serial_pair.port_a, "radio")
    collector = Collector(tx_namespace="radio", rx_namespace="radio")
    harness = Harness(bridge, collector)
    peer = serial.Serial(serial_pair.port_b, 115200, timeout=0.5)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])
        time.sleep(0.5)

        rng = random.Random(7)
        peer.write(bytes(rng.randrange(256) for _ in range(4096)))
        peer.flush()
        time.sleep(1.5)

        # Random bytes can occasionally form a structurally valid frame whose
        # CRC also happens to match, so "publishes nothing" is not a sound
        # assertion. What must hold is that everything published went through
        # validation, and that the overwhelming majority of noise was rejected.
        stats = bridge.stats()
        published = collector.payloads()
        assert len(published) == stats["frames"]
        assert stats["drops"] > 0, "4 KiB of noise should have been rejected"
        assert stats["frames"] < stats["drops"] / 10, (
            f"{stats['frames']} frames accepted from pure noise against "
            f"{stats['drops']} rejected; validation is too weak"
        )

        # And the node recovers. The frame immediately after a garbage run is
        # expected to be lost: the trailing noise sits in the parser's buffer
        # until the next delimiter, which is that frame's own. Resync costs one
        # frame and no more, so the second one must arrive.
        peer.write(encode_frame(b"sacrificial") + encode_frame(b"still alive"))
        peer.flush()
        assert wait_until(lambda: b"still alive" in collector.payloads())
    finally:
        peer.close()
        harness.close()


# ----------------------------------------------------------------------
# 3. Backpressure
# ----------------------------------------------------------------------


def test_tx_callback_never_blocks_while_the_writer_is_wedged(serial_sink):
    bridge = make_bridge(
        serial_sink.port,
        "radio",
        tx_queue_depth=8,
        tx_overflow_policy=DROP_OLDEST,
        write_timeout_sec=30.0,  # keep the writer wedged for the whole test
    )
    harness = Harness(bridge)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])

        # 4-byte index + 196 bytes keeps each message exactly at the 200-byte
        # limit; anything over it is rejected before it reaches the queue, which
        # would make this test vacuous.
        payload = bytes(196)
        latencies = []
        for index in range(3000):
            msg = LoRaFrame()
            msg.data = list(index.to_bytes(4, "big") + payload)
            start = time.monotonic()
            # Called directly, which is exactly what a behaviour-tree tick does
            # through the subscription: this is the call that must return now.
            bridge._on_tx(msg)
            latencies.append(time.monotonic() - start)

        worst = max(latencies)
        assert worst < 0.05, f"a /lora/tx callback blocked for {worst * 1000:.1f} ms"

        stats = bridge.stats()
        assert stats["tx_oversize"] == 0, "payloads were rejected before queueing"
        assert stats["tx_dropped_queue"] > 0, "the queue should have overflowed"
        assert len(bridge._tx_queue) <= 8, "the bound was exceeded"
        # Every message is either written, still queued, or explicitly dropped —
        # nothing disappears silently. The one permitted shortfall is the frame
        # the writer thread has already dequeued and is blocked inside write()
        # with, which belongs to no bucket until it completes.
        accounted = (
            stats["tx_dropped_queue"] + stats["tx_frames"] + len(bridge._tx_queue)
        )
        assert accounted in (2999, 3000), f"{3000 - accounted} messages unaccounted for"
        assert stats["tx_frames"] > 0, "the link should have accepted some bytes first"
    finally:
        harness.close()


def test_drop_oldest_keeps_the_freshest_frames_under_flood(serial_sink):
    bridge = make_bridge(
        serial_sink.port,
        "radio",
        tx_queue_depth=8,
        tx_overflow_policy=DROP_OLDEST,
        write_timeout_sec=30.0,
    )
    harness = Harness(bridge)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])
        for index in range(2000):
            msg = LoRaFrame()
            msg.data = list(index.to_bytes(4, "big") + bytes(196))
            bridge._on_tx(msg)

        queued = [
            int.from_bytes(item[:4], "big") for item in list(bridge._tx_queue._items)
        ]
        # Whatever survives is from the tail of the flood, not the head: a
        # stale coordination message is the one worth throwing away.
        assert queued, "expected a full queue at the end of the flood"
        assert min(queued) > 1000, f"drop_oldest kept stale frames: {queued}"
    finally:
        harness.close()


def test_drop_newest_keeps_the_earliest_frames_under_flood(serial_sink):
    bridge = make_bridge(
        serial_sink.port,
        "radio",
        tx_queue_depth=8,
        tx_overflow_policy=DROP_NEWEST,
        write_timeout_sec=30.0,
    )
    harness = Harness(bridge)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])
        for index in range(2000):
            msg = LoRaFrame()
            msg.data = list(index.to_bytes(4, "big") + bytes(196))
            bridge._on_tx(msg)

        queued = [
            int.from_bytes(item[:4], "big") for item in list(bridge._tx_queue._items)
        ]
        assert queued, "expected a full queue at the end of the flood"
        # The mirror image of the previous test: the policy is a real choice.
        assert max(queued) < 1000, f"drop_newest discarded early frames: {queued}"
    finally:
        harness.close()


def test_flood_over_the_real_topic_does_not_wedge_the_node(serial_sink):
    bridge = make_bridge(
        serial_sink.port,
        "radio",
        tx_queue_depth=8,
        tx_overflow_policy=DROP_OLDEST,
        write_timeout_sec=2.0,
    )
    collector = Collector(tx_namespace="radio", rx_namespace="radio")
    harness = Harness(bridge, collector)
    try:
        assert wait_until(lambda: bridge.stats()["port_open"])
        time.sleep(0.5)

        start = time.monotonic()
        for index in range(1500):
            collector.send(index.to_bytes(4, "big") + bytes(180))
        publish_elapsed = time.monotonic() - start
        assert publish_elapsed < 5.0, (
            f"publishing 1500 messages took {publish_elapsed:.2f}s; "
            "the publisher was throttled by the radio"
        )

        # Unblock the link and confirm the node resumes rather than staying wedged.
        serial_sink.drain()
        before = bridge.stats()["tx_frames"]
        collector.send(b"after the flood")
        assert wait_until(lambda: bridge.stats()["tx_frames"] > before, timeout=15.0)
        assert bridge.stats()["port_open"], "the port should have survived congestion"
    finally:
        harness.close()


def test_node_stays_idle_when_the_radio_is_unplugged():
    """A missing serial port must not cost CPU.

    The reader loop has nothing to block on when the port will not open, so it
    is the one place an accidental busy-wait can hide. On a battery-powered
    robot a spinning core is a real cost, and an unplugged radio is a normal
    state, not an exceptional one.
    """
    bridge = make_bridge("/dev/does-not-exist", "unplugged", reconnect_period_sec=0.5)
    harness = Harness(bridge)
    try:
        assert not bridge.stats()["port_open"]
        time.sleep(0.5)  # let startup settle before sampling

        before = time.process_time()
        time.sleep(2.0)
        cpu_seconds = time.process_time() - before

        # A spin burns ~2.0 CPU-seconds over this window; idling burns ~0.02.
        # The threshold sits far from both, so it flags the bug without being
        # sensitive to how busy the machine running the suite is.
        assert cpu_seconds < 0.5, (
            f"the node burned {cpu_seconds:.2f} CPU-seconds over 2s wall time "
            "with no port; a loop is spinning"
        )
    finally:
        harness.close()

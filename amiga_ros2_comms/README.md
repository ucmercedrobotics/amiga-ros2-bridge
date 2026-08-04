# amiga_ros2_comms

Robot-to-robot comms transports for the Amiga. Each transport is a self-contained
module under `amiga_ros2_comms/`, so adding another protocol means adding a
directory and an entry point, not reworking this one. The shared pieces
(`ring_queue.py`, `serial_link.py`) sit at the package root because any transport
needs them.

Currently implemented: **LoRa** (`amiga_ros2_comms/lora/`), with a simulated
medium so the same node runs in Gazebo without a radio; the **coordination
codec** (`amiga_ros2_comms/codec/`) that gives those opaque bytes a meaning; and
the **reliability layer** (`amiga_ros2_comms/reliability/`) that makes them
arrive, and arrive once.

```
firmware -> serial bridge -> codec -> reliability -> coordinator
            (lora/)          (codec/)  (reliability/)  (amiga_ros2_coordinator)
```

That is the transport spine complete. The coordinator above it lives in its own
package, [`amiga_ros2_coordinator`](../amiga_ros2_coordinator), and consumes
this one in-process.

## LoRa bridge

A ROS2 node that owns the serial port to the LoRa Arduino and exposes the radio
as two topics of opaque framed bytes. An identical instance runs on every robot.

```
ros2 launch amiga_ros2_comms lora_bridge.launch.py serial_port:=/dev/ttyUSB0
```

### Interface

| topic | type | direction |
| --- | --- | --- |
| `/lora/tx` | `amiga_interfaces/LoRaFrame` | in - a payload to transmit |
| `/lora/rx` | `amiga_interfaces/LoRaFrame` | out - one validated payload received |

`LoRaFrame.data` is an opaque byte array. `rssi`/`snr` are only meaningful when
`has_link_stats` is true, which current firmware never sets - subscribers must
tolerate their absence.

### Scope

This node knows about **frames, CRC and queues**, and nothing about what the
bytes mean. No message types, no task IDs, no ACKs, no dedup, no retries, no
sequence numbers. Those belong to the [codec](#coordination-codec), reliability
and coordinator layers above it. The test for whether a change belongs here: it
would work unchanged if
we shipped weather data instead of coordination messages.

Fragmentation is deliberately absent too - an oversized payload is rejected, not
split, because reassembly is a higher layer's problem.

### Concurrency

The radio is half-duplex and slow, and nothing upstream may ever block on a
serial write. Three threads around two bounded queues:

```
/lora/tx callback --> [tx ring] --> writer thread --> serial
serial --> reader thread --> [rx ring] --> publisher thread --> /lora/rx
```

The subscription callback only enqueues and returns, so a behaviour-tree tick
that publishes to `/lora/tx` returns instantly whether or not the radio is
mid-transmit. This is asserted directly in
`test_tx_callback_never_blocks_while_the_writer_is_wedged`, which wedges the
writer on a serial port nobody drains and then measures 3000 callbacks.

Overflow is a named policy, not an accident. `tx_overflow_policy` defaults to
`drop_oldest`, because a stale coordination message is worthless; `drop_newest`
is available where order beats freshness.

### Parameters

| parameter | default | notes |
| --- | --- | --- |
| `serial_port` | `/dev/ttyUSB0` | device path of the LoRa Arduino |
| `baud` | `115200` | **assumed** - must match firmware `Serial.begin()` |
| `max_payload_bytes` | `200` | **assumed** - see the dwell-time discussion in the contract |
| `tx_queue_depth` | `32` | outbound ring capacity, in frames |
| `rx_queue_depth` | `64` | inbound queue capacity, in frames |
| `tx_overflow_policy` | `drop_oldest` | or `drop_newest` |
| `rx_overflow_policy` | `drop_oldest` | or `drop_newest` |
| `rx_link_stats` | `none` | or `header`, once firmware reports RSSI/SNR |
| `read_timeout_sec` | `0.1` | |
| `write_timeout_sec` | `1.0` | on expiry the frame is dropped; the port stays open |
| `reconnect_period_sec` | `2.0` | retry interval for a missing port |
| `stats_period_sec` | `30.0` | counters log line; `0` disables |

`baud` and `max_payload_bytes` are host-side assumptions pending firmware repo
access. See [docs/lora_frame_contract.md](docs/lora_frame_contract.md).

### Wire format

Fully specified in [docs/lora_frame_contract.md](docs/lora_frame_contract.md).
In brief: `COBS(length || body || crc8(body)) || 0x00`, with CRC-8/ATM. The
contract is symmetric, which is what lets two bridge instances talk to each
other over a pty pair with no radio in the loop.

The frame CRC exists because the LoRa PHY protects the *air* link but nothing
protects the USB/UART hop. Its only job is to turn a flipped bit into a dropped
frame rather than a published bad one.

### Tests

All run without a radio:

```
colcon build --packages-select amiga_interfaces amiga_ros2_comms
source install/setup.bash
python3 -m pytest amiga_ros2_comms/test/ -q
```

`test_framing.py`, `test_ring_queue.py`, `test_airtime.py`, `test_codec.py` and
`test_reliability.py`
need no ROS at all. The node tests build their own virtual serial ports in-process
(`test/conftest.py`), which is the pure-Python equivalent of
`socat -d -d pty,raw,echo=0 pty,raw,echo=0` - same topology, no external
dependency.

## Coordination codec

`amiga_ros2_comms/codec/` turns a typed coordination message into one packet of
bytes and back. It is a **pure library, not a node** — no ROS, no serial, no
radio — so it imports and unit-tests anywhere.

```python
from amiga_ros2_comms.codec import Heartbeat, Capability, cap_mask, encode, decode

packet = encode(Heartbeat(
    src=3, seq=1024,
    cap_mask=cap_mask(Capability.DRIVE, Capability.SPRAY),
    grid_row=12, grid_col=47, battery=88, cur_task=0,
))                                  # 13 bytes
msg = decode(packet)                # Heartbeat(...)
```

Six built message types — HEARTBEAT, TASK_ANNOUNCE, BID, GRANT, ACK, HAZARD —
each 7 to 13 bytes, every one of which fits in a single LoRa payload with room
to spare even at SF10. Coordinates are local orchard grid indices, never
lat/lon. Tag `0x07` (FREEFORM) is reserved and cleanly refused in both
directions.

Full byte layouts, enums, quantization and error taxonomy are in
[docs/codec_message_vocabulary.md](docs/codec_message_vocabulary.md).

### Scope

Field layout, quantization and the type tag. **Nothing about delivery.** No
ACKs, no retransmit, no dedup, no fragmentation, no coordination policy;
`encode` and `decode` are pure functions that remember nothing between calls.

The header carries `(src, seq)` so the reliability layer can dedup and ACK on
it later, but this layer never interprets those bytes — it packs and unpacks
them and stops. An oversized message is rejected, never split, for the same
reason the bridge rejects an oversized payload: a fragmentation scheme without
sequence recovery corrupts messages instead of dropping them.

## Reliability layer

`amiga_ros2_comms/reliability/` turns the fire-and-forget radio into something
task ownership can stand on. It wires the codec to the bridge's topics and gives
the coordinator a three-item contract: send reliably, send best-effort, receive
exactly once.

```
ros2 launch amiga_ros2_comms lora_reliability.launch.py node_id:=3
```

### The core rule: reliability follows addressing

| addressing | types | delivery |
| --- | --- | --- |
| **unicast** | GRANT | **reliable** — ACK + bounded retransmit |
| **broadcast** | HEARTBEAT, TASK_ANNOUNCE, BID, HAZARD | **best-effort** — sent once |

You cannot cleanly ACK a broadcast from a set of receivers nobody has
enumerated, so reliability for those is repetition by the coordinator plus
receiver-side dedup here. Losing a GRANT, by contrast, double-assigns or orphans
a task, so the winner must confirm it. GRANT carries no `dst` — its `winner_id`
*is* the address.

Addressed is not private: a GRANT is still overheard by non-winners and
delivered up to them (deduped, unACKed), which is how losing bidders learn the
auction closed.

### Interface

```python
future = reliability.send_reliable(dst, msg)   # -> Future[DELIVERED | FAILED]
reliability.send_broadcast(msg)                # returns immediately
reliability.set_on_deliver(callback)           # deduplicated inbound, once each
```

`DELIVERED` means a specific robot acknowledged a specific `(src, seq)`. That is
the signal the coordinator uses to enforce *unassigned-until-ACKed* — this layer
provides it and does **not** make the ownership decision.

The coordinator consumes this in-process (add the node to its executor) rather
than over a ROS service: the interface carries typed codec messages, and
re-encoding those into `.msg` files to cross a process boundary that does not
exist would buy a serialization hop and a second failure mode.

### Scope

**Message IDs, ACKs, retransmit timers, a dedup cache.** No bidding, no
arbitration, no ownership decisions, no LLM — if a change involves *deciding*
something it belongs in the coordinator. No fragmentation either: every built
message is one packet by construction, and splitting arrives with FREEFORM or
not at all.

### Retransmit timing

`retransmit_timeout_sec` must exceed the round trip **on air** — the message's
time on air plus the ACK's — or retransmits go out while the first ACK is still
in flight and congest a half-duplex channel. Time on air doubles per spreading
factor (82 ms round trip at SF7, 2.15 s at SF12), so no one default spans the
range: the shipped **3.0 s** clears SF7–SF10, and the node checks the configured
value against `spreading_factor` at startup and warns rather than letting it be
discovered as a retransmit storm.

Full parameter table, the inbound path, and the bounds worth knowing (`seq`
wrap, dedup TTL and capacity) are in
[docs/reliability_layer.md](docs/reliability_layer.md).

### Tests

The engine has no ROS in it — the clock is injected and the link is a callable —
so whole retransmit campaigns and minutes of TTL behaviour run in microseconds
with no sleeping. Loss is a predicate the test writes, not a probability, so
every run drops exactly the frame the test meant.

```
python3 -m pytest amiga_ros2_comms/test/test_reliability.py -q
```

`test_reliability_node.py` adds what only a running node can show, including a
GRANT crossing the **whole transport spine**: two reliability nodes, two
bridges, one virtual serial pair, nothing mocked but the radio.

## Simulated LoRa

For Gazebo, CI, or any run without radios. `lora_sim` hands every robot a serial
device that behaves like the host side of its own LoRa modem and carries frames
between them. It models exactly two things a socat pty pair does not: broadcast
minus the sender, and real time on air (including the modem going deaf to its
serial port while transmitting, which is what makes the bridge's backpressure
path testable at all).

```
ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=robot1,robot2,robot3
```

The bridge is unchanged and unaware - same node, same parameters, same topics.
Only `serial_port` differs, and each robot's is `/tmp/amiga_lora_sim/<robot>`.
Launch order does not matter: a bridge whose port does not exist yet retries
until the sim creates it.

Full details, the parameter table, and what is deliberately *not* modelled are in
[docs/lora_sim.md](docs/lora_sim.md). Nothing on a real robot should launch it.

## Not done yet (blocked on firmware repo access)

- Aligning the Arduino firmware to this frame contract.
- Confirming `baud` and `max_payload_bytes` against the firmware.
- Making the firmware prepend RSSI/SNR, then switching `rx_link_stats` to
  `header`.
- The real-radio test: two machines, two radios, echo blobs across, log RSSI/SNR.

## Next layer up

The **coordinator** is built, in
[`amiga_ros2_coordinator`](../amiga_ros2_coordinator): the contract-net state
machine (announce, bid, grant, confirm) with both roles, and the two LLM
invocation points stubbed behind interfaces. It uses exactly what this package
offers — `send_reliable` to hold task ownership consistent, `send_broadcast`
for announcements, bids and hazards, and once-only delivery inbound — and owes
the transport nothing but a decision about what to send.

Its `coordinator` executable runs a `ReliabilityNode` in its own process, so do
not launch `lora_reliability.launch.py` alongside it.

Still deferred below it, deliberately: **fragmentation and reassembly**, built
together with the reserved FREEFORM type or not at all.

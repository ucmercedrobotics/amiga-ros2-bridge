# Coordination message vocabulary (v1)

The wire format for robot-to-robot coordination messages. One typed message is
one packet of bytes; one packet is one LoRa payload.

Implemented in
[`amiga_ros2_comms/codec/`](../amiga_ros2_comms/codec/), which is pure Python
with no ROS, no serial and no radio imports, so either end of this contract can
be implemented and tested anywhere. The tag table, enums and quantization
constants are normative and live in exactly one file,
[`codec/definitions.py`](../amiga_ros2_comms/codec/definitions.py) — encoder and
decoder both import from it, which is the only reason they cannot drift.

```
firmware -> serial bridge -> codec -> reliability -> coordinator
                             ^^^^^
```

## Scope of this layer

Field layout, quantization, and the type tag. **Nothing about delivery.** No
ACK logic, no retransmit, no dedup, no fragmentation, no reassembly, no
coordination policy. `encode` and `decode` are pure functions that remember
nothing between calls.

The layer below (`lora/framing.py`) carries opaque bytes and does not know these
messages exist. The layer above — the
[reliability layer](reliability_layer.md) — is where `(src, seq)` finally gets
used for something.

The test for whether a change belongs here: it is about what a message *means*
or how it is *laid out*. If it is about getting a message *delivered*, or about
a message that does not fit in one packet, it belongs above.

## Byte order

**Big-endian (network order), everywhere.** Note this deliberately differs from
the little-endian link-stats header in
[lora_frame_contract.md](lora_frame_contract.md): that header's layout is
dictated by modem firmware, while this vocabulary is ours to define, and network
order is the portable default for a protocol other implementations have to read.

There is no padding or alignment anywhere. Every layout below is exact.

## Common header

Every message begins with the same 4 bytes.

| offset | field | width | notes |
| --- | --- | --- | --- |
| 0 | `tag` | 1 | message type, table below |
| 1 | `src` | 1 | sender robot ID, 1..255 (0 means "no robot") |
| 2–3 | `seq` | 2 | sender's monotonic sequence number, wraps at 65535 |

`(src, seq)` is the globally unique message ID. It is carried **now** so the
reliability layer can dedup and ACK on it **later**. The codec packs and unpacks
these two fields and does nothing else with them: it does not generate `seq`,
does not check that it advances, and does not remember one.

## Message types

| tag | name | total bytes | status |
| --- | --- | --- | --- |
| `0x01` | HEARTBEAT | 13 | built |
| `0x02` | TASK_ANNOUNCE | 13 | built |
| `0x03` | BID | 9 | built |
| `0x04` | GRANT | 7 | built |
| `0x05` | ACK | 7 | built |
| `0x06` | HAZARD | 13 | built |
| `0x07` | FREEFORM | — | **reserved, not implemented** |

Tag values are frozen once shipped. Renumbering one silently breaks every peer,
so `test_message_type_values_match_the_wire_contract` pins them.

Every type is fixed-size, so the encoded length is a constant per tag rather
than a range — which is why `MAX_MESSAGE_BYTES` (13) is an exact bound and not
an estimate.

### `0x01` HEARTBEAT — 13 bytes

I exist, here is what I can do, where I am, and what I am doing. Periodic. This
is what populates the fleet's view of who is available to bid.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `cap_mask` | 2 | bitmask over Capability bit indices |
| 6–7 | `grid_row` | 2 | unsigned grid index |
| 8–9 | `grid_col` | 2 | unsigned grid index |
| 10 | `battery` | 1 | whole percent, 0..100 |
| 11–12 | `cur_task` | 2 | task ID, or 0 when idle |

### `0x02` TASK_ANNOUNCE — 13 bytes

There is work at this cell; whoever can do it, bid.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `task_id` | 2 | 0 means "none" |
| 6 | `req_capability` | 1 | a single Capability **index**, 0..15 |
| 7–8 | `grid_row` | 2 | unsigned grid index |
| 9–10 | `grid_col` | 2 | unsigned grid index |
| 11 | `priority` | 1 | unitless, 0..255, higher is more urgent |
| 12 | `reason_code` | 1 | ReasonCode |

### `0x03` BID — 9 bytes

My offer on an announced task.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `task_id` | 2 | |
| 6 | `eta_s` | 1 | seconds, **4 s/LSB**, 0..1020 |
| 7 | `feasible` | 1 | 0 or 1; any other value is rejected |
| 8 | `cost` | 1 | unitless, 0..255, **lower is better** |

`feasible = false` is a meaningful answer, not a non-answer: it tells the
announcer this robot heard the announcement and is ruling itself out, which lets
an auction close early instead of waiting out a timeout.

`cost` is whatever scalar the bidder minimises (time, energy, detour). It is
only comparable between bids on the same task.

### `0x04` GRANT — 7 bytes

Task awarded to `winner_id`. The only **addressed** message in the vocabulary,
and therefore the only one the
[reliability layer](reliability_layer.md) sends reliably: a lost GRANT either
double-assigns a task or orphans it.

There is no `dst` in the header — `winner_id` *is* the address. Addressed is not
private, though: it goes out over the same broadcast radio as everything else,
so losing bidders hear it and learn the auction closed. They simply do not ACK
it; only the winner does.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `task_id` | 2 | |
| 6 | `winner_id` | 1 | robot ID |

### `0x05` ACK — 7 bytes

I received message `(ack_src, ack_seq)`.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4 | `ack_src` | 1 | `src` of the message being acknowledged |
| 5–6 | `ack_seq` | 2 | `seq` of the message being acknowledged |

The codec packs this type and nothing more. **When** to send one, what to do
when one fails to arrive, and how long to hold a message for retransmit are all
the reliability layer's problems. The type exists now only so the vocabulary is
complete and the tag is allocated.

### `0x06` HAZARD — 13 bytes

Something is in the way at this cell, for about this long.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4 | `hazard_class` | 1 | HazardClass |
| 5–6 | `grid_row` | 2 | unsigned grid index |
| 7–8 | `grid_col` | 2 | unsigned grid index |
| 9 | `radius` | 1 | **grid cells**, 0..255 |
| 10 | `confidence` | 1 | whole percent, 0..100 |
| 11–12 | `ttl_s` | 2 | seconds, 1 s/LSB, 0..65535 |

## Coordinates

`grid_row` / `grid_col` are **local orchard grid indices, never lat/lon.** They
are unsigned, to match the `ROW_INDEX` / `COL_INDEX` convention already used by
`amiga_interfaces/GetTreeInfo`, which indexes from a corner origin. That service
uses `uint8`; the 16 bits here are deliberate headroom, not a different
coordinate system.

`radius` is in grid cells for the same reason — a hazard is a disc in the grid
its own coordinates already live in, with no unit conversion in between.

## Capabilities

`Capability` values are **bit indices**, not mask values. That is what lets
HEARTBEAT advertise a whole set in 2 bytes while TASK_ANNOUNCE names a single
requirement in 1, with a one-line test between them:

```python
has_capability(heartbeat.cap_mask, announce.req_capability)
```

The alternative — making `req_capability` an 8-bit mask over a 16-bit capability
space — cannot represent half the capabilities and truncates silently, so the
index encoding is the one that stays honest as the enum grows.

`cap_mask` is 16 bits, so a capability index of 16 or more can never be
advertised by anyone. `req_capability` is therefore range-checked to 0..15 and a
larger value is rejected rather than treated as merely unknown.

Adding a capability means **appending** an index. Never renumber one.

| index | name |
| --- | --- |
| 0 | `DRIVE` |
| 1 | `INSPECT` |
| 2 | `SPRAY` |
| 3 | `HARVEST` |
| 4 | `MANIPULATE` |
| 5 | `TRANSPORT` |
| 6 | `SURVEY` |
| 7 | `CHARGE_HOST` |
| 8 | `RELAY` |

## Quantization and ranges

| field | wire | resolution | range | exact? |
| --- | --- | --- | --- | --- |
| `battery`, `confidence` | 1 byte | 1 % | 0..100 | yes |
| `priority`, `cost`, `radius` | 1 byte | 1 | 0..255 | yes |
| `eta_s` | 1 byte | **4 s** | 0..1020 (17 min) | no, ±2 s |
| `ttl_s` | 2 bytes | 1 s | 0..65535 (18 h) | yes |
| `grid_row`, `grid_col` | 2 bytes | 1 cell | 0..65535 | yes |
| `task_id`, `cur_task` | 2 bytes | 1 | 0..65535 | yes |
| `cap_mask` | 2 bytes | — | 16 bits | yes |

Percentages are **0..100, not 0..255 scaled**. A percent field that can legally
read 200 is a percent nobody downstream can sanity-check.

### Why `eta_s` is 4 s/LSB and `ttl_s` is not

At 1 s/LSB a one-byte ETA caps at 255 s — under five minutes, and shorter than
a great many orchard traverses, so a robot would have to either lie or overflow.
4 s/LSB buys 17 minutes for the same byte, and 4 s of slop on an ETA that size
is not a number anyone was going to act on. Encoding rounds half-up, so the
worst-case error is 2 s; `test_eta_quantization_error_is_bounded_by_half_an_lsb`
pins that.

`ttl_s` has two bytes, so it needs no coarsening at all: 1 s/LSB already reaches
18 hours. The asymmetry is a consequence of field width, not of the two fields
meaning different kinds of thing.

Out-of-range values are **rejected, never wrapped or clamped**. Silently turning
a 20-minute ETA into a 3-minute one is the failure this codec most wants to
avoid, because nothing downstream can detect it.

## Sentinels

- `task_id` / `cur_task` of **0** means "none" — idle, or a task nobody owns.
- Robot ID **0** means "no robot"; real IDs are 1..255.

Both are fleet conventions. The codec packs whatever it is given and does not
enforce them.

## The reserved FREEFORM tag

`0x07` is **allocated but not implemented**, and encoding one is refused.

Free text cannot be bounded to a single packet, so FREEFORM needs
fragmentation — which belongs to the reliability layer, which does not exist
yet. Claiming the tag now stops anything else taking `0x07` in the meantime.

Decoding a `0x07` frame raises `ReservedMessageType`, which is deliberately
**not** the same as `UnknownMessageType` and is not a subclass of it in either
direction. "We know exactly what this is and cannot handle it yet" and "we have
never heard of this" call for different operator responses, so a caller must be
able to tell them apart with `isinstance`. The reserved check happens before any
length validation, so a truncated FREEFORM frame still reports as reserved.

## Errors

All are subclasses of `CodecError(ValueError)`, and each carries a `kind` string
so the layer above can count failure classes without matching on message text —
the same arrangement `lora.framing.FrameError` uses.

| exception | `kind` | raised when |
| --- | --- | --- |
| `UnknownMessageType` | `unknown_type` | tag is not allocated |
| `ReservedMessageType` | `reserved_type` | tag is `0x07` FREEFORM |
| `TruncatedMessage` | `truncated` | buffer ends before the message does (includes empty) |
| `TrailingBytes` | `trailing` | buffer holds a whole message and then more |
| `PayloadTooLarge` | `oversize` | encoded size exceeds `max_payload_bytes` |
| `FieldRangeError` | `range` | a value does not fit, or cannot mean what it says |
| `CodecError` | `value` | not a message object / not a bytes-like object |

`decode` never raises anything else and never reads past the end of the buffer.
Lengths are checked *before* any unpack, which is what
`test_every_short_prefix_of_a_valid_message_is_rejected` proves: an unchecked
`struct.unpack_from` would raise `struct.error`, which is not a `CodecError`,
and the test catches only `CodecError`.

### Two deliberate strictnesses

**Trailing bytes are rejected, not ignored.** One message is one packet, so
extra bytes mean the sender and receiver disagree about a field width, or
someone concatenated packets. Both are worth failing loudly over, and neither is
repaired by dropping the tail.

**Impossible values are rejected on decode, not just on encode.** A battery byte
of 200 is not a bit flip — the frame passed CRC to get here — it is a protocol
bug, and handing it up as an object the rest of the stack trusts is strictly
worse than dropping it.

### One deliberate leniency

An **unrecognised enum value** in `reason_code` or `hazard_class` passes through
as a plain `int` rather than failing the message. A newer peer naming a hazard
class we lack is still telling us where the hazard is. Since `IntEnum` compares
equal to its integer value, this costs callers nothing.

This does not apply to `tag` — an unknown tag means the payload cannot be parsed
at all — nor to `req_capability`, where an out-of-range index is genuinely
impossible rather than merely unfamiliar.

## Size budget

`max_payload_bytes` is a parameter to `encode`, defaulting to 50. The largest
built message is 13 bytes, so there is a great deal of headroom; the tight
default is the design rule that keeps the vocabulary single-packet even at a
high spreading factor, where the legal payload collapses to 24 bytes (SF10) or
lower — see the dwell-time table in
[lora_frame_contract.md](lora_frame_contract.md).

The bridge's own `max_payload_bytes` (200 by default) is the *transport*
ceiling. This one is a *design* ceiling and is meant to stay much smaller.

An oversized message is **rejected, never split**. A codec that quietly returned
two packets would be inventing a reliability protocol with no sequence recovery
and no retransmit, which is the same mistake R1 of the frame contract forbids
the modem from making.

## Tests

```
python3 -m pytest amiga_ros2_comms/test/test_codec.py -q
```

No ROS, no serial, no radio, and nothing to build first. The five acceptance
groups from the brief — round-trip, size, reserved tag, unknown tag, malformed
input — plus quantization bounds and capability-mask behaviour.

Test instances are generated **from the codec's own field table**, so every
field's minimum and maximum are exercised for every type, and a new message type
or a widened field is covered the moment it is declared. Boundaries are also
swept one field at a time, which is what catches a field packed at the wrong
offset: an all-maximum instance still round-trips correctly even if two
same-width fields are transposed.

`test_decode_of_arbitrary_garbage_never_raises_anything_but_codec_error` fuzzes
20 000 random buffers; anything other than a `CodecError` escaping is a failure.

## Not done yet

Deferred out of this layer, deliberately. The first three are now **built** in
the [reliability layer](reliability_layer.md):

- ~~Any *use* of `(src, seq)` — dedup, ACK tracking, retransmit.~~
- ~~Wiring the codec to `/lora/tx` and `/lora/rx`. Nothing here imports ROS.~~
- ~~Deciding **when** to send an ACK.~~
- Fragmentation and reassembly, and therefore FREEFORM. Still deferred, and now
  the only thing standing between the vocabulary and completeness.

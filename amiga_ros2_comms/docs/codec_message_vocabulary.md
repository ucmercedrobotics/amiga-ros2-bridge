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
| `0x01` | HEARTBEAT | 18 | built |
| `0x02` | TASK_ANNOUNCE | 19 | built |
| `0x03` | BID | 9 | built |
| `0x04` | GRANT | 7 | built |
| `0x05` | ACK | 7 | built |
| `0x06` | — | — | **retired, do not reuse** (was HAZARD) |
| `0x07` | FREEFORM | — | **reserved, not implemented** |

Tag values are frozen once shipped. Renumbering one silently breaks every peer,
so `test_message_type_values_match_the_wire_contract` pins them.

Every type is fixed-size, so the encoded length is a constant per tag rather
than a range — which is why `MAX_MESSAGE_BYTES` (19) is an exact bound and not
an estimate.

`0x06` carried HAZARD. It was removed because nothing in this system ever
detected or published a hazard: the type described a capability the robot does
not have, and a wire vocabulary that names things nobody can produce is a
vocabulary an implementer has to guess about. The gap stays a gap — decoding
`0x06` reports an unallocated tag, and giving the number to a new type would
make any peer still holding the old code silently misread it.

### `0x01` HEARTBEAT — 18 bytes

I exist, here is what I can do, where I am, and what I am doing. Periodic. This
is what populates the fleet's view of who is available to bid.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `cap_mask` | 2 | bitmask over Capability bit indices |
| 6 | `target_kind` | 1 | TargetKind, 0..3 |
| 7–10 | `target_a` | 4 | signed; see [Targets](#targets) |
| 11–14 | `target_b` | 4 | signed; see [Targets](#targets) |
| 15 | `battery` | 1 | whole percent, 0..100 |
| 16–17 | `cur_task` | 2 | task ID, or 0 when idle |

A robot with no position fix sends `TargetKind.NONE`, which is a different fact
from being at the origin. The grid this replaced could not say it: zeros meant
both, so a robot with no fix advertised itself in the Gulf of Guinea and every
ETA computed against it was fiction.

### `0x02` TASK_ANNOUNCE — 19 bytes

There is work here; whoever can do **all** of it, bid.

| offset | field | width | encoding |
| --- | --- | --- | --- |
| 0–3 | header | 4 | |
| 4–5 | `task_id` | 2 | 0 means "none" |
| 6–7 | `req_cap_mask` | 2 | bitmask over Capability bit indices |
| 8 | `target_kind` | 1 | TargetKind, 0..3 |
| 9–12 | `target_a` | 4 | signed; see [Targets](#targets) |
| 13–16 | `target_b` | 4 | signed; see [Targets](#targets) |
| 17 | `priority` | 1 | unitless, 0..255, higher is more urgent |
| 18 | `reason_code` | 1 | ReasonCode |

`req_cap_mask` is a **set**, not one action, because a task is a behaviour-tree
subtree and a subtree uses several. Sampling a tree is `MoveToTreeID` *and*
`SampleLeaf` — the pairing the arbiter's orphaned-`SampleLeaf` check already
enforces — and a robot with the arm but no tree navigation would pass a
single-action test and then be unable to place the work it just won.

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

## Targets

A place is three fields — `target_kind` plus two signed 32-bit words — and what
the words mean depends on the kind.

| kind | value | `target_a` | `target_b` |
| --- | --- | --- | --- |
| `NONE` | 0 | must be 0 | must be 0 |
| `TREE` | 1 | tree index, 0..65535 | must be 0 |
| `AISLE` | 2 | aisle index, 0..65535 | must be 0 |
| `GPS` | 3 | latitude × 10⁷ | longitude × 10⁷ |

There is no single coordinate system here **because the behaviour tree does not
have one.** `MoveToTreeID` names a tree index, `MoveToAisleHead` an aisle index,
`MoveToGPSLocation` and `ApproachGPSWaypoint` a latitude and longitude — and
`SampleLeaf` and `FollowPerson` name nothing at all, because they happen wherever
the robot already is. `amiga_interfaces/GetTreeInfo` converts between the first
three on demand (`TREE_INDEX`, `ROW_INDEX`, `COL_INDEX`, `AISLE_INDEX` → `lat`,
`lon`), so normalising to one of them at announce time would throw away exactly
what the receiving robot needs to rebuild the action node. A peer that wins
"tree 60" can emit `<MoveToTreeID id="60" approach_tree="true"/>`; a peer that
wins a bare latitude and longitude cannot.

`NONE` is undelegable by construction: work with no place of its own is only
meaningful next to work that has one. `MoveToRelativeLocation` also yields
`NONE`, because an offset from *this* robot's pose is not a place another robot
can be sent to.

GPS resolution is **1e-7 degrees**, about 1.1 cm at the equator — finer than any
orchard waypoint in `examples/`, and `int32` still covers the full ±180° range
with an order of magnitude to spare. Degrees are converted at the boundary, by
`Target.gps()` in and `lat_deg` / `lon_deg` out, so everything below is integers
and the codec needs no fractional scaling.

The three fields are validated **together** on decode. Each word is in range on
its own, but a `TREE` target carrying a longitude, or a `NONE` target carrying
coordinates, is a sender that disagrees with us about the layout — the same
class of problem as trailing bytes, and refused just as loudly.

## Capabilities

`Capability` values are **bit indices**, not mask values, and each one is a
**behaviour-tree action type**: exactly the elements of
`<xs:group name="ActionGroup">` in
`amiga_ros2_behavior_tree/schemas/amiga_btcpp.xsd`, in schema order.

That is what makes a capability claim checkable rather than asserted. The schema
is the one the mission planner writes against, the arbiter validates against,
and `bt_runner` refuses a mission for violating — so a robot advertising these
bits is advertising what its own tree can actually be asked to do.
`test_the_capability_vocabulary_is_the_behaviour_trees_action_group` asserts the
two agree in both directions: a capability with no XSD element would be a robot
claiming a skill no mission could invoke, and an XSD element with no bit would be
work the fleet can never delegate.

Both HEARTBEAT and TASK_ANNOUNCE carry a 16-bit mask, so the test is one line:

```python
has_capabilities(heartbeat.cap_mask, announce.req_cap_mask)
```

Adding a capability means **appending** an index. Never renumber one.

| index | name | XML element |
| --- | --- | --- |
| 0 | `MOVE_TO_TREE_ID` | `<MoveToTreeID>` |
| 1 | `MOVE_TO_AISLE_HEAD` | `<MoveToAisleHead>` |
| 2 | `MOVE_TO_GPS_LOCATION` | `<MoveToGPSLocation>` |
| 3 | `APPROACH_GPS_WAYPOINT` | `<ApproachGPSWaypoint>` |
| 4 | `MOVE_TO_RELATIVE_LOCATION` | `<MoveToRelativeLocation>` |
| 5 | `ORIENT_ROBOT_HEADING` | `<OrientRobotHeading>` |
| 6 | `FOLLOW_PERSON` | `<FollowPerson>` |
| 7 | `SAMPLE_LEAF` | `<SampleLeaf>` |
| 8 | `MOVE_ARM_TO_POSITION` | `<MoveArmToPosition>` |

`DetectObject`, `AssertTrue` and `CheckValue` are registered in `bt.cpp` but
commented out of the schema, so they have no bit: a mission containing one is
rejected before it ever runs, which makes advertising it a lie.

Other platforms in the `schemas/` submodule (Husky, Kinova, Spot) name different
actions. Extending the fleet to one of them means appending its actions as new
indices, or mapping them onto these where they mean the same thing — not
reinterpreting the existing bits.

## Quantization and ranges

| field | wire | resolution | range | exact? |
| --- | --- | --- | --- | --- |
| `battery` | 1 byte | 1 % | 0..100 | yes |
| `priority`, `cost` | 1 byte | 1 | 0..255 | yes |
| `eta_s` | 1 byte | **4 s** | 0..1020 (17 min) | no, ±2 s |
| `target_kind` | 1 byte | — | 0..3 | yes |
| `target_a`/`_b` as tree/aisle | 4 bytes | 1 index | 0..65535 | yes |
| `target_a`/`_b` as GPS | 4 bytes | **1e-7°** | ±180° | no, ±0.55 cm |
| `task_id`, `cur_task` | 2 bytes | 1 | 0..65535 | yes |
| `cap_mask`, `req_cap_mask` | 2 bytes | — | 16 bits | yes |

Percentages are **0..100, not 0..255 scaled**. A percent field that can legally
read 200 is a percent nobody downstream can sanity-check.

### Why `eta_s` is 4 s/LSB and the target words are not

At 1 s/LSB a one-byte ETA caps at 255 s — under five minutes, and shorter than
a great many orchard traverses, so a robot would have to either lie or overflow.
4 s/LSB buys 17 minutes for the same byte, and 4 s of slop on an ETA that size
is not a number anyone was going to act on. Encoding rounds half-up, so the
worst-case error is 2 s; `test_eta_quantization_error_is_bounded_by_half_an_lsb`
pins that.

The target words have four bytes each, so they need no coarsening: a tree index
round-trips exactly, and GPS at 1e-7° is finer than the fix that produced it.
The asymmetry is a consequence of field width, not of the fields meaning
different kinds of thing.

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

An **unrecognised `reason_code`** passes through as a plain `int` rather than
failing the message. A newer peer naming a reason we have no word for is still
telling us there is work. Since `IntEnum` compares equal to its integer value,
this costs callers nothing.

This does not apply to `tag` — an unknown tag means the payload cannot be parsed
at all — nor to `target_kind`, and that contrast is the point. An unnamed reason
is informational and can be logged as a number; an unnamed target kind means the
two words after it are in a coordinate system we cannot read, and a robot that
drove to them anyway would be acting on a misreading rather than on a gap in its
vocabulary.

## Size budget

`max_payload_bytes` is a parameter to `encode`, defaulting to 50. The largest
built message is 19 bytes, so there is a great deal of headroom; the tight
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

# Reliability layer

Makes bytes arrive, and arrive once.

This is the layer that turns a fire-and-forget radio into something task
ownership can stand on. It wires the codec to the serial bridge and gives the
coordinator above it a contract with three items in it: send this reliably, send
this best-effort, and here is an inbound message you have not already been
given.

```
firmware -> serial bridge -> codec -> reliability -> coordinator
            (lora/)          (codec/)  (reliability/)  (amiga_ros2_coordinator)
```

Implemented in
[`amiga_ros2_comms/reliability/`](../amiga_ros2_comms/reliability/).

## Scope

**Message IDs, ACKs, retransmit timers, a dedup cache, a transmit-ordering label
on the way out, and the splitting and reassembly of notes.** That is the whole
list.

It knows nothing about what a message *means* or what to do about it: no
bidding, no arbitration, no task-ownership decisions, no LLM. It reports
delivered-or-failed and hands the coordinator the facts; the coordinator draws
the conclusions. The one thing it does read out of a message body is *who it is
addressed to*, and that is confined to
[`addressing.py`](../amiga_ros2_comms/reliability/addressing.py) — "who is this
for" is addressing, not meaning.

The one other thing it says about a message is *which of these goes next*, in
[`priority.py`](../amiga_ros2_comms/reliability/priority.py): ACK and GRANT are
`urgent`, everything else is `bulk`. That is a label and nothing more — this
layer never queues, reorders, delays or paces a frame. The bridge's outbound
ring is where the backlog forms and where the ordering is applied, which is why
the class travels on `LoRaFrame.priority` rather than being inferred down there.

It fragments **exactly one thing**: a *note*, free text bound to a task id
(`notes.py`). Every other built message fits in a single packet by construction
(19 bytes at the largest, against a 50-byte budget), so one of those being too
big to send is still a design error to be raised rather than a stream to split.

Notes are broadcast, therefore unACKed, therefore a note missing a fragment is
**dropped rather than repaired** — no NACK, no retransmit, no erasure coding.
That is only acceptable because the TASK_ANNOUNCE a note accompanies carries the
whole machine-readable requirement by itself: lose every fragment and the
auction is bit-for-bit what it would have been. Text that could invalidate an
auction by going missing would be a worse design than not carrying text at all.
The rate at which notes fail to reassemble is a number to report, not a bug to
engineer around.

The test for whether a change belongs here: it is about *getting bytes there
once*. If it involves **deciding** something, it belongs in the coordinator.

## The core rule: reliability follows addressing

| addressing | types | delivery | why |
| --- | --- | --- | --- |
| **unicast** | GRANT (later COMPLETE, RELEASE) | **reliable** — ACK + bounded retransmit | losing a GRANT double-assigns or orphans a task |
| **broadcast** | HEARTBEAT, TASK_ANNOUNCE, BID, HAZARD | **best-effort** — sent once | you cannot cleanly ACK a broadcast from a set of receivers nobody has enumerated |

The asymmetry is forced, not chosen. A broadcast goes to a set of receivers
whose membership nobody knows, so there is no set of ACKs whose absence means
anything: waiting for them stalls on a robot that was switched off, and
retransmitting until they arrive never terminates.

Reliability for broadcasts is therefore **repetition at the application level**
— the coordinator re-emits announces on timeout and re-broadcasts hazards until
their TTL — **plus receiver-side dedup here**. This layer does not retransmit
broadcasts. It deduplicates them on receipt, which is the half of the job that
has to live at this level.

Sending the wrong type the wrong way is refused rather than quietly downgraded:
`send_reliable(HEARTBEAT)` and `send_broadcast(GRANT)` both raise
`ReliabilityError`. Those are caller bugs, and a `FAILED` future would bury them.

### Addressing without a `dst` field

The wire header carries `src` and `seq`, and no destination. The addressee is a
property of the body: **GRANT names its `winner_id`**, and that is the address.
Adding a unicast type is one line in `_ADDRESSEE_FIELD`; forgetting the line
makes the new type best-effort broadcast rather than silently half-reliable.

Two consequences worth being explicit about:

- **`send_reliable(dst, msg)` cross-checks `dst` against the body.** A GRANT
  addressed to robot 5 sent with `dst=7` is refused. That mismatch is exactly
  how you get a task one robot acts on and a different robot acknowledges.
- **Addressed is not private.** A GRANT goes out over the same broadcast radio
  as everything else, so non-winners hear it and it *is* delivered up to their
  coordinators (deduped, unACKed) — that is how losing bidders learn the auction
  closed. Addressing decides who owes an ACK, not who may listen.

## Interface to the coordinator

```python
future = reliability.send_reliable(dst, msg)   # -> Future[Outcome]
reliability.send_broadcast(msg)                # -> bool, returns immediately
reliability.set_on_deliver(callback)           # deduplicated inbound, once each
```

`Outcome` is `DELIVERED` or `FAILED`, and nothing else. `DELIVERED` means a
specific robot acknowledged a specific `(src, seq)` — the fact the coordinator
needs in order to hold task ownership consistent. Every other ending is a
failure; the distinctions between them (timed out, rejected, never encoded) are
diagnostics in the counters, not decisions to branch on.

This is what lets the coordinator enforce **unassigned-until-ACKed**. This layer
provides the signal. It does not make the ownership decision.

`send_reliable` overwrites `msg.src` and `msg.seq`: message identity belongs to
this layer, and a caller-supplied `seq` is how two different messages come to
share an ID and one of them gets silently deduped away.

### Why in-process and not a ROS service

The interface carries typed codec messages and a delivery outcome. Re-encoding
those into `.msg` files to move them between two nodes on the same machine would
add a serialization hop, a second failure mode, and an interface to keep in step
with the codec — to connect two layers that always ship together. The
coordinator adds a `ReliabilityNode` to its own executor and calls it directly.

## Inbound path

For each payload on `/lora/rx`:

1. **Decode.** Unknown tags, tags allocated but unbuilt, truncated buffers,
   trailing bytes and impossible field values are each counted and dropped.
   `on_frame` never raises — this layer is the only thing between the radio and
   the coordinator.
2. **ACK?** Resolve the matching pending send. ACKs are never deduped, never
   delivered upward, and never themselves ACKed. The coordinator gets the
   `Outcome`, which is the fact it actually wanted.
3. **Ours?** A frame with our own `src` is an echo from a repeater or a loopback
   misconfiguration. Dropped, or we would ACK ourselves.
4. **Addressed to us and reliable?** Emit an ACK — **before the dedup check and
   independent of it.**
5. **Dedup on `(src, seq)`.** First sight is delivered up; a repeat is counted
   and dropped.

Step 4 is the subtle one. A duplicate arriving means our previous ACK was lost
or is still in the air. Staying silent because we recognise the message would
let the sender exhaust its retries and report a **delivered** task as failed —
orphaning it. So: **re-ACK every copy, deliver only the first.**

## Retransmit timing

Retransmit timing must respect the link, not the CPU. The floor under
`retransmit_timeout_sec` is the message's time on air *plus the ACK's*; set it
below that and retransmits go out while the first ACK is still being
transmitted, congesting a half-duplex channel faster than ACKs can clear it.

Computed from the existing airtime model
([`lora/airtime.py`](../amiga_ros2_comms/lora/airtime.py)), for a 13-byte
message and its 7-byte ACK:

| | SF7 | SF8 | SF9 | SF10 | SF11 | SF12 |
| --- | --- | --- | --- | --- | --- | --- |
| round-trip floor | 82 ms | 155 ms | 289 ms | 537 ms | 1.07 s | 2.15 s |
| want at least (4×) | 0.33 s | 0.62 s | 1.16 s | 2.15 s | 4.29 s | 8.59 s |

The 4× factor covers what the pure airtime does not: queueing in the bridge,
scheduling delay, half-duplex turnaround, and other robots' traffic.

Time on air **doubles per spreading factor**, so no single default spans the
range. The shipped default of **3.0 s clears SF7 through SF10** — the practical
orchard range — and the node checks the configured value against its
`spreading_factor` at startup and warns if it is short. SF11 and SF12 must be
tuned explicitly; the warning is what tells the operator, rather than leaving it
to be discovered as a retransmit storm.

The bridge already paces serial and airtime. This layer only decides *when* to
hand it a retransmit.

## Parameters

| parameter | default | notes |
| --- | --- | --- |
| `node_id` | `1` | this robot's fleet-unique ID, 1..255. **Has no safe default** — the node refuses to start outside the range |
| `retransmit_timeout_sec` | `3.0` | must exceed the round trip on air; see above |
| `max_retries` | `3` | retransmits **after** the original, so up to 4 copies |
| `retransmit_backoff` | `1.5` | multiplier per retransmit; `1.0` is a flat timer |
| `max_retransmit_timeout_sec` | `10.0` | ceiling on the backed-off interval |
| `dedup_ttl_sec` | `120.0` | must outlast the longest retransmit campaign and the slowest app-level re-broadcast |
| `dedup_max_entries` | `512` | hard cap, so a fast or hostile peer costs bounded memory |
| `max_pending` | `32` | reliable sends in flight; beyond it sends **fail fast** |
| `max_payload_bytes` | `50` | per-message budget handed to the codec |
| `tick_period_sec` | `0.1` | retransmit-deadline granularity; keep well under the timeout |
| `spreading_factor` | `7` | **only** used for the timeout sanity check; match the radio |
| `stats_period_sec` | `30.0` | counters log line; `0` disables |

`node_id` must be unique across the fleet. Two robots sharing one make
`(src, seq)` ambiguous, which silently dedups one robot's traffic away as
duplicates of the other's — a failure that looks like radio loss and is not.

`max_pending` fails fast rather than queueing because a caller can retry a
rejection but cannot retry a hang: an unbounded pending table on a link that has
gone quiet turns a dead radio into a dead node.

## Bounds worth knowing

- **`seq` wraps at 16 bits.** Safe only because `dedup_ttl_sec` is vastly
  shorter than the time it takes to emit 65 536 messages on a LoRa link.
- **The dedup cache is finite in time and size.** A duplicate older than
  `dedup_ttl_sec` *will* be delivered again — that bound is pinned by a test, so
  the parameter has a testable meaning. Capacity eviction is counted separately
  from TTL expiry: expiry is the cache working, eviction means it is too small
  for the traffic.
- **An ACK from the wrong robot does not confirm delivery.** "Delivered" has to
  mean the intended owner has it, or the ownership guarantee built on top of it
  means nothing.

## Testing

The engine (`ReliabilitySession`) has **no ROS in it**. The clock is injected
and the link is a callable, so the acceptance tests drive whole retransmit
campaigns and minutes of TTL behaviour in microseconds, with no executor, no
sleeping and no flakiness:

```
python3 -m pytest amiga_ros2_comms/test/test_reliability.py -q
```

Loss is a **predicate the test writes**, not a probability, so "the first GRANT
is lost" is exactly what happens on every run. There is a seeded random rule for
soak-style tests, but no acceptance test depends on chance.

The six acceptance groups from the brief, all in
[`test_reliability.py`](../test/test_reliability.py):

| group | what it pins |
| --- | --- |
| dedup | the same `(src, seq)` received *N* times is delivered exactly once |
| delivery under loss | the first GRANT is dropped; it retransmits and is reported `DELIVERED` |
| give-up | every copy dropped; `FAILED` after `max_retries`, and **not a hang** |
| ACK correctness | one ACK, right `(src, seq)`, right sender; ACKs neither ACKed nor retransmitted |
| broadcast | never put on a timer; a duplicate broadcast still collapses to one delivery |
| sequence | outgoing `seq` monotonic per sender, wrapping cleanly at 16 bits |

Plus inbound hostility (garbage, unknown tags, reserved tags, impossible field
values, self-echo), re-entrancy (sending from inside `on_deliver` must not
deadlock), and a callback that raises.

[`test_reliability_node.py`](../test/test_reliability_node.py) covers what only
a running node can show: that parameters reach the session, that the retransmit
timer fires under a real executor, and that a GRANT survives the **whole
transport spine** — two reliability nodes, two bridges, one virtual serial pair,
nothing mocked but the radio.

## Not done yet

- **Forward error correction for notes.** A note that loses a fragment is
  dropped. Fountain or erasure coding would let `k` of `k+r` fragments
  reconstruct it with no feedback, which is the shape that fits a broadcast —
  and is deliberately not built, because the loss curve is what the experiment
  is measuring.
- **COMPLETE and RELEASE** — the other unicast types. One line each in
  `_ADDRESSEE_FIELD` once the codec has them.

The coordinator is built: [`amiga_ros2_coordinator`](../../amiga_ros2_coordinator)
holds the contract-net state machine, and the two LLM invocation points are
stubbed behind interfaces there. It consumes this layer in-process and uses
exactly the three-item contract above — `send_reliable` to enforce
unassigned-until-ACKed, `send_broadcast` for announcements, bids and hazards
(repeating them itself, on its own schedule), and once-only delivery inbound.

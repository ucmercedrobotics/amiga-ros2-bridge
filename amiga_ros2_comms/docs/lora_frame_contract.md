# LoRa host <-> modem frame contract (v2)

This is the wire contract between a host and a LoRa modem, over the serial hop
between them. It is deliberately **not specific to this robot, this fleet, or
ROS** — it describes framed opaque bytes over a UART and nothing else, so any
project that needs a LoRa link can implement either end of it.

The host side is implemented in
[`amiga_ros2_comms/lora/framing.py`](../amiga_ros2_comms/lora/framing.py), which
has no ROS or platform imports for that reason. **Modem firmware lives in its
own repo and is not managed here.** This document is the requirement it
implements; everything under "Requirements on the modem" is normative.

The contract is **near-symmetric**: host->modem and modem->host frames use the
same layout, differing only in the mandatory inbound link-stats header (below).
The symmetry is what lets two host-side implementations talk to each other over
a plain pseudo-terminal pair with no radio and no firmware in the loop, which is
how the node is tested; the simulator
([lora_sim.md](lora_sim.md)) supplies the inbound header that real firmware
would, so the sim path exercises the asymmetric case too.

## Scope of this layer

Frames, CRC, queues. Nothing else. The payload is **opaque bytes**. This layer
never inspects, types, acknowledges, deduplicates, retries, or reorders it.

## Byte layout

### Frame body (before COBS)

```
+--------+---------------------------+--------+
| length |          body             | crc8   |
| 1 byte |        `length` bytes     | 1 byte |
+--------+---------------------------+--------+
```

- `length` — unsigned count of `body` bytes that follow. One byte, so the
  absolute ceiling is 255 — but see **252** below, which is the real limit.
- `body` — the opaque payload, preceded by a 3-byte link-stats header on the
  inbound direction (see below).
- `crc8` — CRC-8 over `body` only (not over `length`).

`length` is not covered by the CRC, per the brief. It does not need to be: COBS
decoding yields an exact byte count, and the receiver checks
`length == len(decoded) - 2`. A corrupted `length` byte therefore fails that
consistency check and the frame is dropped. Corruption inside `body` is what the
CRC catches.

### On the wire (after COBS)

```
COBS(length || body || crc8) || 0x00
```

- **COBS**: Consistent Overhead Byte Stuffing, standard formulation. The encoded
  form contains no `0x00` bytes, so `0x00` is an unambiguous frame delimiter and
  the stream is self-resynchronising: a receiver that joins mid-frame, or that
  drops a frame, recovers at the next `0x00` with no ambiguity.
- Overhead is 1 byte per 254 bytes of body, plus the delimiter. For the largest
  frame this layer will emit (255-byte body): 257 body bytes -> 259 encoded
  -> 260 on the wire.
- A `0x00` delimiter is emitted after every frame. Consecutive `0x00` bytes
  (i.e. an empty inter-delimiter run) are skipped, not treated as an error, so
  firmware may send a leading `0x00` to flush a partial frame after reset.

### CRC-8 definition

Standard CRC-8/ATM, i.e. the one usually just called "CRC-8":

| parameter | value |
| --- | --- |
| polynomial | `0x07` (x^8 + x^2 + x + 1) |
| initial value | `0x00` |
| input reflected | no |
| output reflected | no |
| final XOR | `0x00` |
| check value (`"123456789"`) | `0xF4` |

The check value is asserted in the host test suite. Firmware must reproduce it.
Note this is *not* the Dallas/Maxim CRC-8 that ships in many Arduino OneWire
libraries (poly `0x31`, reflected) — those are not interchangeable.

Reference implementation (bitwise, no table, suitable for AVR):

```c
uint8_t crc8(const uint8_t *d, uint8_t n) {
    uint8_t c = 0x00;
    while (n--) {
        c ^= *d++;
        for (uint8_t i = 0; i < 8; i++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}
```

### Why a CRC at all

The LoRa PHY already CRCs the *air* link, so a frame that reaches the Arduino
intact was validated there. The USB/UART hop between Arduino and host is
unprotected, and that is what this CRC covers. Its only job is to turn a flipped
bit into a **dropped frame** rather than a published bad one. It does not
correct, and it does not trigger retransmission — there is no retransmission at
this layer.

## Link stats (modem -> host, mandatory)

Every inbound frame carries rssi and snr at the front of the body. In v1 this
was optional and configured out-of-band; making it mandatory removes the only
silent-corruption failure the contract had, where a host configured for bare
payloads would hand three bytes of radio telemetry to its subscribers as though
they were data, with nothing on either side able to notice.

```
+---------------+----------+--------------------+
| rssi (int16)  | snr(int8)|      payload       |
|  little-end.  | qtr-dB   |                    |
+---------------+----------+--------------------+
```

- `rssi` — signed 16-bit, little-endian, dBm. Matches the SX127x packet-RSSI
  register semantics after the datasheet's offset correction.
- `snr` — signed 8-bit in quarter-dB steps (SX127x `PktSnrValue` raw), so the
  representable range is -32.00 .. +31.75 dB. The host divides by 4.

### Why this caps payloads at 252, not 255

An inbound body is `3 + payload`, and the length byte caps the body at 255. So a
253-byte payload could be *transmitted* but could never be *delivered* — it would
need a 256-byte body on the far side. The ceiling is therefore **252 bytes in
both directions**, and a transmitter that accepts 255 is building frames that are
structurally undeliverable.

There is no flags or type byte. A bare frame and a stats-bearing frame would be
indistinguishable on the wire — the payload is opaque, so no byte pattern can
discriminate them — which is exactly why presence is fixed by the contract
rather than sniffed or negotiated. The host's `rx_link_stats` parameter survives
only to talk to a v1 modem and for host-to-host loopback tests; against a v2
modem it must be `header`.

Subscribers still see `has_link_stats` on the published message, because the
message type is shared with transports that have no such notion.

## Requirements on the modem

Normative, and numbered so firmware can cite them. Nothing here is
project-specific: a modem satisfying R1-R8 works for any host that speaks this
frame format.

**R1 — one host frame is one radio packet.** No chunking, no reassembly, no
accumulation across frames. A modem that splits a frame into indexed chunks has
invented an unprotected reliability protocol: with no sequence recovery and no
retransmit, a single lost chunk silently *corrupts* a message instead of
dropping it, which is strictly worse than not sending it. Fragmentation belongs
to a layer above this one, which is why the host rejects an oversized payload
rather than splitting it.

**R2 — payload ceiling is 252 bytes, and oversize is dropped, not truncated.**
The trap here is real: `LoRa.write()` in arduino-LoRa silently clamps to
`MAX_PKT_LENGTH`, so an oversized packet transmits *successfully* as a
truncated one and the CRC failure surfaces at the far end as if the air link
were bad.

**R3 — every inbound frame carries the link-stats header**, per the section
above. Not conditional, not configurable.

**R4 — radio parameters are set explicitly, never inherited.** Spreading factor,
bandwidth, coding rate, preamble length, sync word and TX power all assigned in
code at init. `LoRa.begin()` sets none of them, so a sketch that omits these
runs on SX127x reset defaults — a configuration nobody chose, nobody wrote down,
and which silently decides the legal payload size.

**R5 — the PHY CRC is enabled.** The SX127x powers up with `RxPayloadCrcOn = 0`
and `LoRa.begin()` does not change it. With it off, a packet corrupted over the
air is handed to the host, and the only thing between it and a published frame
is this contract's 8-bit CRC — which misses roughly 1 corruption in 256. The
PHY CRC is 16 bits and costs 5.1 ms on a 255-byte packet at SF7. Our CRC-8
exists to protect the *serial* hop and was never sized to be the air link's only
defence.

**R6 — nothing but frames on the framed port.** Debug and status text goes to a
different port. One stray `println` on the link is a guaranteed CRC error, and a
plausible-looking one, since ASCII decodes as perfectly valid COBS often enough
to waste an afternoon.

**R7 — transmitting must not stall the host link.** At 115200 baud a 318 ms
blocking transmit is ~3.6 kB of arriving host bytes against a typical 64-byte
UART buffer. Drain the host port while the radio is busy, or the host loses
frames it was never told about.

**R8 — the sync word is explicit and shared fleet-wide.** On the library default
every LoRa device in earshot shares the channel.

### Serial settings

| setting | value |
| --- | --- |
| baud | `115200` |
| framing | 8N1, no flow control |
| debug output | a different port, never this one (R6) |

`115200` is roughly two orders of magnitude faster than the LoRa air rate, so the
UART is never the bottleneck and the host's backpressure comes from the radio
rather than from the serial line. Some stock LoRa sketches still use `9600`.

### The spreading-factor decision

This is the one number that has to be chosen rather than defaulted, because it
sets the legal payload size and nothing in the code will tell you when you have
broken it.

1. **The length byte** — hard ceiling of 255, and 252 after R3.
2. **The SX127x FIFO** — 256 bytes, so one packet cannot exceed 255 regardless.
3. **FCC Part 15.247 dwell time (US 915 MHz ISM)** — this is the binding one.
   Narrowband LoRa (BW 125 kHz) in the US operates under the hybrid FHSS rules,
   which cap channel dwell at 400 ms. **So the real ceiling depends on the
   spreading factor the firmware is configured for**, and it collapses fast:

   | config (CR 4/5, 8-symbol preamble, explicit header, CRC on) | largest legal payload | airtime of a 200 B payload |
   | --- | --- | --- |
   | SF7 / BW125  | 255 B (the length byte runs out first) | 318 ms |
   | SF8 / BW125  | 138 B | 564 ms |
   | SF9 / BW125  | 66 B  | 1005 ms |
   | SF10 / BW125 | 24 B  | 1845 ms |
   | SF11 / BW125 | 0 B — even an empty frame is 578 ms | 4018 ms |
   | SF12 / BW125 | 0 B — even an empty frame is 991 ms | 7217 ms |

   These come from `amiga_ros2_comms.lora.airtime`, which implements Semtech's
   time-on-air formula; `max_payload_for_dwell(radio)` recomputes the middle
   column for any configuration. Note the last two rows: at SF11 or SF12 on a
   125 kHz channel there is *no* legal single-transmission payload in this band,
   so a firmware configured that way needs a wider bandwidth or true frequency
   hopping, not a smaller `max_payload_bytes`.

#### BW500 removes the dwell limit entirely

The 400 ms cap applies because a 125 kHz channel puts the radio under the hybrid
FHSS rules. At **BW500** the occupied bandwidth clears the 500 kHz threshold and
the link falls under the digital-modulation rules instead, where there is no
dwell limit at all:

| | 252 B airtime | dwell-limited? |
| --- | --- | --- |
| SF7 / BW125 | 391 ms | yes — 9 ms of margin |
| SF7 / BW500 | 98 ms | no |
| SF9 / BW500 | 310 ms | no |

Four times the throughput and the legal ceiling disappears, for about 8 dB of
link budget. If robot-to-robot range is not the binding constraint, BW500 is a
far more comfortable place to sit than 9 ms under a hard regulatory limit.

#### Why the host still defaults to 200

`200` is below every ceiling above and leaves 52 bytes of headroom, which is
what absorbs enabling the PHY CRC (R5) without going illegal. It is a policy
choice, not a limit: set it to the dwell budget for whatever SF/BW the modem is
actually configured for. Oversized payloads are dropped with a warning, never
fragmented — fragmentation is a higher layer's problem, not this node's.

The simulator (`docs/lora_sim.md`) enforces the same budget: it holds each frame
in the air for its real airtime and warns once if a frame exceeds the dwell
limit, so an illegal payload size shows up in Gazebo rather than on a radio.

## Failure modes and what the node does

| condition | action |
| --- | --- |
| CRC mismatch | drop frame, count `crc_errors` |
| `length` disagrees with decoded size | drop frame, count `length_errors` |
| malformed COBS (zero code byte, truncated block) | drop frame, count `cobs_errors` |
| body larger than `max_payload_bytes` | drop frame, count `oversize_errors` |
| no delimiter within max frame size | discard buffer, resync at next `0x00` |
| TX payload larger than `max_payload_bytes` | reject in the subscriber callback, warn |
| serial port disappears | close, retry open every `reconnect_period_sec` |

Every one of these is a drop. Nothing at this layer ever repairs or re-requests
a frame.

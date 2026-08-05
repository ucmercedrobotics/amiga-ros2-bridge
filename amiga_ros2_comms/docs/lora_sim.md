# Simulated LoRa

Gazebo has no radios. This gives every simulated robot a serial device that
behaves like the host side of its own LoRa modem, and carries frames between
them.

**The bridge does not know it is in simulation.** Same node, same parameters,
same `/lora/tx` and `/lora/rx` topics, same frames and CRC. The only thing that
changes is which path `serial_port` points at. That is the point: whatever the
coordination layer is tested against in Gazebo is the code that ships.

```
                     ┌──────────────── lora_sim ────────────────┐
/r1/lora/tx ──▶ bridge ──▶ /tmp/amiga_lora_sim/r1 ──▶ ┐         │
                     │                                 │        │
                     │              time on air                 │
                     │                                 │        │
/r2/lora/rx ◀── bridge ◀── /tmp/amiga_lora_sim/r2 ◀── ┘         │
/r3/lora/rx ◀── bridge ◀── /tmp/amiga_lora_sim/r3 ◀── ┘         │
                     └──────────────────────────────────────────┘
```

## Running it

```sh
ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=robot1,robot2,robot3
```

That starts the medium and one bridge per robot, each namespaced by robot name,
so the fleet talks over `/robot1/lora/tx`, `/robot2/lora/rx` and so on.

If the robots' own bringup already launches their bridges, pass
`bridges:=false` and point each bridge at its port:

```sh
ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=robot1,robot2 bridges:=false
ros2 launch amiga_ros2_comms lora_bridge.launch.py serial_port:=/tmp/amiga_lora_sim/robot1
```

Launch order does not matter. A bridge whose port does not exist yet retries
until it appears, which is the normal case since nothing orders the sim against
the robots' bringup.

## What is modelled, and why

Only two things, because only two things would otherwise make a test lie.

**Broadcast, minus the sender.** One robot transmits, every *other* robot hears
it. Nothing in the bridge addresses anyone, so coordination logic above has to
cope with hearing every message on the network — and a half-duplex radio never
hears itself, so dedup logic that was only tested against a two-ended pipe looks
correct until it meets a third robot.

**Time on air.** A frame occupies the modem for its real airtime: 318 ms for a
200-byte payload at SF7/BW125. While the modem is busy it stops draining its
serial port, exactly as real firmware does, so the host's write buffer fills and
the bridge's writes block. That path is the entire reason the bridge has bounded
queues and a non-blocking `/lora/tx` callback, and against a plain pty it is
unreachable, because a pty accepts everything instantly.

**Nothing else is modelled.** No collisions, no packet loss, no range, no
capture effect, no clock skew. Those all belong to the layer that can *react* to
them, and this bridge is explicitly forbidden to — no ACKs, no retries, no
dedup. Simulating them here would be scenery: nothing under test could behave
differently because of them. When the reliability layer above exists, that is
the moment to add them, and to add them against something that can fail.

## Parameters

| parameter | default | meaning |
| --- | --- | --- |
| `robots` | `[robot1, robot2]` | Names on the air. One device each. |
| `symlink_dir` | `/tmp/amiga_lora_sim` | Where the per-robot device paths live. |
| `spreading_factor` | `7` | 6..12. Drives airtime, and with it the fleet's whole traffic budget. |
| `bandwidth_hz` | `125000` | Channel bandwidth. |
| `coding_rate` | `5` | Denominator of the 4/N rate, 5..8. |
| `preamble_symbols` | `8` | Preamble length. |
| `dwell_limit_sec` | `0.4` | Warn once if a frame exceeds this airtime. FCC Part 15.247. |
| `stats_period_sec` | `30.0` | Counters log line. 0 disables. |

## Counters

Logged periodically and available from `LoRaSim.stats()`: `frames_sent`,
`frames_delivered`, `tx_frame_errors`, `in_flight`.

`tx_frame_errors` is the one worth watching: there is no line noise on a pty, so
a frame this side cannot parse means the host framing disagrees with
[the contract](lora_frame_contract.md) — a bug, not a loss.

## Sizing traffic

`amiga_ros2_comms.lora.airtime` will tell you what a configuration can carry
before you design against it:

```python
>>> from amiga_ros2_comms.lora.airtime import RadioConfig, airtime_sec, max_payload_for_dwell
>>> airtime_sec(200, RadioConfig(spreading_factor=7))
0.31770...
>>> max_payload_for_dwell(RadioConfig(spreading_factor=10))
24
```

At SF7/BW125 a 200-byte frame is 318 ms of air. Three robots broadcasting one of
those every two seconds is already ~48% channel occupancy — worth knowing before
the coordination layer assumes it can chat.

## Limits worth knowing

* **This is not part of the robot stack.** Nothing on a real robot should
  launch `lora_sim`, and nothing should depend on it existing.
* **The ports are ptys, so `baud` is ignored.** The pacing comes from the
  airtime model, not from the serial line. On hardware the UART is roughly two
  orders of magnitude faster than the radio, so the radio is the bottleneck in
  both cases, but they are not the same mechanism.
* **A pty absorbs about 16 kB before it blocks a writer.** Real firmware has a
  UART buffer of tens of bytes, so the sim is far more forgiving than hardware.
  It will show you that backpressure exists; it will not show you the right
  threshold.
* **Airtime assumes the modem transmits the payload**, not the COBS frame — the
  framing is the host-to-modem contract, and what goes over the air is the
  firmware's business.

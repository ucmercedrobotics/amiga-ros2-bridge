# amiga_ros2_coordinator

Deciding what the fleet does. One node per robot, running both contract-net
roles at once.

```
firmware -> serial bridge -> codec -> reliability -> coordinator
            (amiga_ros2_comms)                       (this package)
```

[`amiga_ros2_comms`](../amiga_ros2_comms) makes bytes arrive, and arrive once.
This decides what they should have said: who bids on a task, who wins it, when
to give up, and when to ask the robot to yield.

```
ros2 launch amiga_ros2_coordinator coordinator.launch.py node_id:=3
```

That one executable runs the coordinator **and** its reliability layer in the
same process. Do not launch `lora_reliability.launch.py` alongside it.

No capability list: the robot reads its own `amiga_btcpp.xsd` — the schema its
behaviour tree validates against — and advertises the actions that schema
permits. An action the schema forbids is an action no mission can contain, so
advertising it would be a claim that could never be tested. Pass
`capabilities:=MoveToTreeID,SampleLeaf` only to advertise *less* than the
schema allows, for hardware that is temporarily absent.

## Two roles, one node

Every robot always has tasks it might have to shed, and is always a candidate
for its peers' tasks.

```
owner   infeasible -> interpret_anomaly -> ANNOUNCE -> collect -> arbitrate
                   -> GRANT -> *delivered* -> transferred -> replan_and_verify
bidder  ANNOUNCE -> capable? -> nav/mission -> fitness -> backoff
                 -> (suppressed?) -> BID -> GRANT for us -> absorb -> replan
```

### Unassigned-until-ACKed

A task is ours until the reliability layer confirms *another robot
acknowledged* the GRANT. Announced is still ours. Granted-but-unconfirmed is
still ours. Sending a GRANT is not a transfer; a delivery report is — any other
rule opens a window in which a task belongs to nobody.

### Fitness-proportional backoff, with suppression

A bid waits, and how long it waits encodes how good it is: the best fit waits
least and transmits into a quiet channel. A bidder that overhears a better bid
during its own backoff never transmits at all. Ordering alone would still put
one packet per bidder on a shared half-duplex radio; suppression is what makes
an auction cost about one bid.

`bid_max_backoff_sec` must be shorter than `announce_window_sec` across the
fleet, or the best bid arrives after the auction closed. The node refuses to
start on an incoherent pair.

### Safe preemption

Coordination events never hard-interrupt anything. This layer publishes a
latched flag on `~/preempt_requested`; the behaviour tree mirrors it onto its
blackboard and yields at a tick point it considers safe. There is no
`abort()`, `cancel()` or `stop()` anywhere in the ports — the capability is
absent by construction, and the tests assert it from the other side.

## The two reasoning points

Exactly two questions here need judgement rather than a decision procedure, and
a third port sits beside them without being one. All three are wired by
default now.

| point | answered by | in tests |
| --- | --- | --- |
| `interpret_anomaly(context)` | the **triage agent** in `amiga_ros2_agents` — an LLM over the BT fault, the `/rosout` window around it, the world state and the live peers | `ScriptedInterpreter` |
| `interpret_note(context)` | the **note agent** in `amiga_ros2_agents` — reads a peer's sentence about work it is offering and says whether it changes our bid | `ScriptedNoteInterpreter`, `IgnoreNotes` |
| `replan_and_verify(delta)` | `VerifyingReplanner`, which hands the delta to the arbiter's `/mission/verify_replan` and runs it through the same LTL gate a local replan gets — **not a reasoning point itself** | `AcceptEverything` |

`interpret_anomaly` returns one of exactly three typed actions — `ReDelegate`,
`AddTask`, `DropTask` — and **never free text**. `interpret_note` is closed the
same way, to `keep` / `revise` / `withdraw`. A state machine that parses
sentences has no enumerable set of behaviours to test. Constraining the output
means the reasoning step chooses among decisions this layer already knows how to
execute, and everything below it stays deterministic and testable *regardless of
what the model says*.

Both calls run off the state machine's lock, on their own thread: each takes
seconds, and `report_infeasible`/bidding hold the lock `tick` and `on_message`
need. If an agent is unreachable or answers outside its schema, the fallback is
conservative rather than a stub filling in an answer: an anomaly whose triage
call fails leaves the task ours (no fleet-wide decision on no evidence), and a
note whose interpretation fails leaves the bid as computed (a note only ever
adjusts a bid this robot was already going to make).

`replan_and_verify` defaults to `VerifyingReplanner`; set `verify_replans:=false`
to fall back to `AcceptEverything` (the acceptance tests pin the state machine
against this and the other ports as stubs, and must not need an arbiter to
run). See
[docs/coordinator.md](docs/coordinator.md#the-two-reasoning-points) for the
full detail on both agents, including why `note` costs the auction a
deliberative window when it fires.

## Where an anomaly comes from

```
BT fault -> /bt/status_change -> planner -> arbiter    (local loop, most faults end here)
                                    |
                        it gives up: /mission/abort
                                    |
                 triage agent -> /coordination/infeasible -> [ this layer ]
                                    |
                    /coordination/interpret_anomaly (LLM) -> one typed action
```

"Has local recovery run out?" is deterministic — the planner and arbiter already
answered it. "What do we do about it?" is the model's call.

## Interface to the robot's own nodes

Navigation and the mission/behaviour-tree stack are separate existing nodes,
reached through ports defined in
[`interfaces.py`](amiga_ros2_coordinator/ports/interfaces.py) and mocked in the
tests. The `coordinator` executable's `main()` supplies placeholders
(`UnavailableNav`, `UnavailableMission`) that decline everything and say so
once — enough to run on a bench with two radios and watch the peer registry
populate.

`coordinator_sim` (`nodes/sim_node.py`) is the second entry point, and wires
the real thing: `GpsNav` for nav, `BehaviorTreeMission` (backed by
`mission_bridge` in `amiga_ros2_agents`) for mission, both answering from
cache and never blocking, since they're called under the same lock `tick` and
`on_message` need. Not one line of `engine/coordinator.py` or
`nodes/coordinator_node.py` knows which entry point it's running under — see
["The wired ports" in docs/coordinator.md](docs/coordinator.md#the-wired-ports-coordinator_sim)
for the constraints that come with that.

## Layout

Five directories, and the split between them is the ports-and-adapters one this
layer is built on. Imports run down the list and never back up.

| Directory | What lives there |
| --- | --- |
| `vocabulary/` | The nouns, the closed action unions, and what this robot can do. |
| `ports/` | The seams, declared as Protocols. No ROS types. |
| `engine/` | Contract net itself — the state machine, auctions, bidding, the peer registry. |
| `adapters/` | Where the ports meet the real robot. **Every ROS dependency is here.** |
| `nodes/` | The three processes: `coordinator`, `coordinator_sim`, `escalate`. |

The top three import no rclpy and no message package, which is what the Tests
section below is claiming when it says the engine has no ROS in it.
[`test_layering.py`](test/test_layering.py) checks that rather than trusting it:
it is the kind of property that decays on one convenient import, and no other
test in the suite would notice the day it does.

## Scope

**Coordination decisions.** No transport (that is the reliability layer), no
fragmentation, no reasoning in the state machine, and no navigation or mission
implementation. If a change is about getting bytes there it belongs one layer
down; if it needs a language model or the LTL backend it goes behind one of the
two reasoning interfaces.

## Tests

The engine has no ROS in it — the clock is injected, transport is a port, nav
and mission are protocols — so whole auctions, backoff windows and peer
timeouts run in microseconds with nothing sleeping and nothing random.

```
colcon build --packages-select amiga_interfaces amiga_ros2_comms amiga_ros2_coordinator
source install/setup.bash
python3 -m pytest amiga_ros2_coordinator/test/ -q
```

`test_coordinator.py` holds the eight acceptance groups. `test_coordinator_node.py`
adds what only a running node can show, including a task crossing **two
robots** — two coordinators, two reliability layers, real ROS in between,
nothing faked but nav and the mission stack.

Full design notes, the parameter table and the bounds worth knowing are in
[docs/coordinator.md](docs/coordinator.md).

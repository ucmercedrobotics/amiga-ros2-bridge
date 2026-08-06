# Coordinator

Deciding what the fleet does. Deterministically.

The layer above reliability. Reliability makes bytes arrive once; this decides
what they should have said. One node per robot, running both contract-net roles
at the same time.

```
firmware -> serial bridge -> codec -> reliability -> coordinator -> triage agent
            (lora/)          (codec/)  (reliability/)  (this)      (amiga_ros2_agents)
```

Implemented in
[`amiga_ros2_coordinator/`](../amiga_ros2_coordinator/).

## Scope

**Coordination decisions.** Who bids, who wins, when to give up, when to
interrupt. That is the whole list.

It does not do transport (that is the reliability layer), does not fragment
(nothing here does), and does not *reason*: the two questions that need
judgement are calls to injected interfaces answered elsewhere. It does not
implement navigation or the mission stack either — those are separate existing
nodes reached through the ports in
[`interfaces.py`](../amiga_ros2_coordinator/ports/interfaces.py).

The test for whether a change belongs here: it is about *deciding* something.
If it is about getting bytes there, it belongs one layer down. If it needs a
language model or the LTL backend, it goes behind one of the two reasoning
interfaces, not into the state machine.

## Watching a fleet do this

```bash
make sim ROBOT_COUNT=3          # three robots, one virtual radio, three coordinators
make fleet-scenario             # tell robot 2 it cannot finish task 42
```

`sim_bringup.launch.py` brings up the coordination layer alongside each robot
when `launch_coordination` is true, which is the default: one `lora_sim` medium
for the fleet, and per robot a `lora_bridge`, a `mission_bridge` and a
`coordinator_sim`. Robot *i* gets `node_id:=i`; robot 1 is unnamespaced and the
rest live under `amiga2`, `amiga3`, …

`fleet-scenario` wraps `ros2 run amiga_ros2_coordinator escalate`, which
publishes one escalation on a chosen robot's `coordination/infeasible` — the
same message the triage agent publishes when the planner and the arbiter have
both given up. Everything after that is the real stack. Useful well beyond the
demo: it is the manual injection point for any bench run.

```bash
ros2 run amiga_ros2_coordinator escalate --robot amiga2 --task 42 --tree 60 \
    --note "north end of row 7 is flooded; approach from the south"
```

Two knobs make the fleet asymmetric, which is what makes an auction worth
watching. `batteries:=100,20,100` gives robot 2 a flat battery, which enters the
bid as a cost penalty below 50%. Distance does the rest on its own, since the
nav port costs travel from each robot's actual GPS fix.

Worth knowing before it puzzles you: `COORDINATION=false` turns the whole layer
off, and with `ROBOT_COUNT=1` you get a fleet of one — it comes up, heartbeats
into an empty channel and has nobody to trade with.

## Where an anomaly comes from

The behaviour tree is the catalyst, and nothing here starts without it.

```
BT node fails -> /bt/status_change -> mission_planner -> arbiter -> /mission/xml
                       (the local self-correction loop; most faults end here)
                                          |
                     it gives up: /mission/abort or a planner give-up
                                          |
                       triage agent -> /coordination/infeasible
                                          |
                                  [ this layer ]
                                          |
                    /coordination/interpret_anomaly (service, LLM)
                                          |
                      re_delegate | add_task | drop_task
```

Two decisions, deliberately different in kind. **"Has local recovery run out?"**
is deterministic — the planner and the arbiter already answered it, and this
layer is only told the answer. **"What should be done about it?"** is not, and
that is the service call.

The escalation arrives as JSON on `infeasible_topic`
(`/coordination/infeasible` by default). It carries the whole task descriptor —
`task_id`, `capabilities` (a mask of behaviour-tree action types), `target_kind`
/ `target_a` / `target_b`, `priority` — plus a `detail` and the originating
fault. The full descriptor rather than an id, because the triage agent is the
only thing in the system that reads `/mission/xml`, and therefore the only thing
that can say what the failed work *is*. A coordinator sent an id alone would
have to invent both the action set and the place, and it would announce them.

This layer never parses XML. That is a boundary, not an omission: coordination
decides who does what, and the mission's structure is the agents' business.

## What a task is

A **behaviour-tree subtree**, not a leaf, and that is the whole shape of this
layer's vocabulary. In `examples/sample_leafs.xml`:

```xml
<MoveToTreeID name="Visit_Tree_60" id="60" approach_tree="true"/>
<SampleLeaf   name="Sample_Leaves_Tree_60"/>
```

`SampleLeaf` alone cannot be delegated — it samples wherever the robot is
standing. The unit is the two together, which is not an invention here: the
arbiter's `_check_no_orphan_sample` already refuses a plan where they come
apart.

Three consequences run through everything below:

- **`required_capabilities` is a mask, not one action.** A robot with the arm
  but no tree navigation would pass a single-action test and then be unable to
  place the work it just won. `has_capabilities(ours, required)` is an all-of
  test.
- **A capability *is* a behaviour-tree action type** — the elements of
  `ActionGroup` in `amiga_btcpp.xsd`, which is the schema the planner writes
  against and `bt_runner` refuses a mission for violating. The robot reads its
  own `mission_schema` at startup rather than being told what it can do.
- **A place is a `Target`, not a coordinate pair**: `TREE(index)`,
  `AISLE(index)`, `GPS(lat, lon)`, or `NONE`. There is no single coordinate
  system because the tree does not have one, and `GetTreeInfo` converts between
  the first three on demand. `NONE` work is **undelegable by construction** —
  the coordinator refuses to announce it rather than naming a place every
  listener resolves differently.

## Two roles, one node

A robot always has tasks it might have to shed and is always a candidate for
its peers' tasks. Both run in the same session, on the same tick.

```
owner   infeasible -> interpret_anomaly -> ANNOUNCE -> collect -> arbitrate
                   -> GRANT -> *delivered* -> transferred -> replan_and_verify
bidder  ANNOUNCE -> capable? -> nav/mission -> fitness -> backoff
                 -> (suppressed?) -> BID -> GRANT for us -> absorb -> replan
```

## The core rule: unassigned-until-ACKed

A task is **ours** from the moment the mission node hands it to us until the
reliability layer confirms another robot acknowledged the GRANT.

| state | ours? | meaning |
| --- | --- | --- |
| `OURS` | yes | executing or waiting; no coordination in flight |
| `ANNOUNCED` | **yes** | offered to the fleet, collecting bids |
| `GRANTED` | **yes** | GRANT sent, delivery unconfirmed |
| `TRANSFERRED` | no | a specific robot ACKed a specific GRANT |
| `RELINQUISHED` | no | dropped locally |

Sending a GRANT is not a transfer. A delivery report is. Any other rule opens a
window in which a task belongs to nobody, and a task belonging to nobody is a
row that never gets sprayed and nobody notices.

When a GRANT is *not* delivered, the task is still ours and there are three
fallbacks, cheapest first: offer it to the next-best bid we already hold;
failing that, run another announce cycle if `max_delegation_attempts` allows;
failing that, hand it to local handling.

## Fitness-proportional backoff, and suppression

The naive bidder answers an announcement immediately. On a shared half-duplex
radio that is the worst possible behaviour: every capable robot answers in the
same instant, the packets collide, and the announcer hears nothing.

So a bid waits, and **how long it waits encodes how good it is** — the best fit
waits the shortest time and transmits into a quiet channel. Meanwhile every
bidder is listening: one that overhears a *better* bid during its own backoff
**suppresses its bid entirely and never transmits**.

Both halves matter. Ordering alone would still put N packets on the air for N
bidders; suppression means a strong bid silences the weak ones before they cost
anything. It is CSMA collision avoidance with fitness in place of randomness.

The rules that make it work:

- **Suppression uses the announcer's own ordering** — `(cost, eta_s, robot_id)`,
  robot-ID tiebreak included. A bidder suppressing itself by a different rule
  than the announcer arbitrates by could silence the bid that would have won.
- **`bid_max_backoff_sec` must be shorter than `announce_window_sec`**, or the
  best-fitting bidder transmits after the auction closed and the whole
  mechanism achieves the opposite of its purpose. The node refuses to start on
  an incoherent pair.
- **Jitter is small and exists for one case**: two robots of identical fitness
  would otherwise wait identical times and collide.
- **Infeasible bids are never suppressed.** Their purpose is to be an answer.

## Arbitration

Lowest `cost` wins; `eta_s` breaks a tie; `robot_id` breaks that. Dull on
purpose: every robot that heard the same bids must predict the same winner, and
an arbitrary-but-fixed tiebreak is the only thing that makes the ordering
total.

An auction closes when its window expires **or** when every peer it was waiting
for has answered. That second ending is why BID carries a `feasible` flag
instead of a declining bidder staying silent: silence and "I cannot" are
different facts, and distinguishing them lets an auction close early instead of
waiting out a timeout for an answer that already arrived.

A peer that ages out of the registry is dropped from both the bids *and* the
set the auction is waiting for. Leaving it in the second is what would make an
auction wait out its whole window for a robot that has been switched off.

## Peer registry

HEARTBEATs go in; a table of `peer -> capabilities, pose, load, last seen`
comes out. It answers the two questions the rest of the layer keeps asking:
which peers could do this task, and is this peer still there.

Entries expire by **timeout, not by counting missed heartbeats**: a count needs
a known period, and the period is the sender's parameter, not ours.
`peer_timeout_sec` wants to be a small multiple of the fleet's heartbeat
period, so a peer survives losing one or two to the radio.

## Safe preemption

Coordination events arrive whenever the radio says so, and a robot halfway
through an arm move must not be stopped by one.

So this layer **raises a flag and never stops anything**. It publishes a
latched `std_msgs/Bool` on `~/preempt_requested`; the behaviour tree mirrors it
onto its blackboard and a reactive condition node yields at a tick point it has
chosen to be safe. The tree acknowledges on `~/preempt_ack`, which lowers the
flag — an acknowledgement, not a handshake: nothing here waits for it, and a
tree that never sends one leaves the flag raised, which is the safe direction.

The capability to hard-interrupt is **absent by construction**: there is no
`abort()`, `cancel()` or `stop()` anywhere in the ports in `ports/interfaces.py`. The
acceptance suite pins this from the other side — the nav and mission fakes
*do* carry those methods, and the preemption tests assert none was ever called.

## The two reasoning points

Exactly two questions here need judgement rather than a decision procedure.
Both are ports; one is now answered by a real agent, the other is still stubbed.

| point | answered by | in tests |
| --- | --- | --- |
| `interpret_anomaly(context) -> ActionSchema` | the **triage agent** (`amiga_ros2_agents/coordination/triage_node.py`) over the BT fault, the `/rosout` window around it, the world state and the live peer list | `ScriptedInterpreter` |
| `replan_and_verify(delta) -> ReplanResult` | **not built** — the existing replan generation + LTL verification (SPIN/Spot, Z3 for the BT) | `AcceptEverything` |

The load-bearing constraint is the **closed union**: `interpret_anomaly`
returns one of exactly three typed actions and never free text.

```
ReDelegate(task, reason_code, fallback)   shed it to the fleet
AddTask(task, reason_code)                take on work we discovered
DropTask(task, disposition)               give it up locally
```

A state machine that parses sentences has no enumerable set of behaviours to
test. Constraining the output to three means the reasoning step chooses *among
decisions this layer already knows how to execute*, and everything below it
stays deterministic — which is what makes the acceptance suite a meaningful
claim about the whole layer regardless of what the model says. The guard is
enforced twice: the agent refuses a malformed answer before it goes on the
wire, and `validate_action` refuses one that gets here anyway.

Choosing between `ReDelegate` and `DropTask` is the decision that matters, and
it turns on one question: is this beyond *this* robot, or beyond *any* robot? A
spray task with an empty tank is worth offering to a peer; a spray task on a
felled tree is not, and announcing it costs the fleet airtime and another
robot's time.

**The service call does not run under the state machine's lock.** It is a model
and takes seconds; `report_infeasible` holds the lock that `tick` and
`on_message` need, so blocking there would stop this robot's heartbeats,
auctions and bids for the length of the call — which the fleet reads as it
dying. The node resolves the decision on its own thread and hands the finished
action back through `report_infeasible(action=...)`, where it is validated
exactly as an interpreter's answer would be.

If the agent is unreachable or answers with something outside the schema, the
task **stays ours**. There is deliberately no fallback to the local stub: a
robot that sheds work because an agent timed out has made a fleet-wide decision
on no evidence.

The give-up path has three named endings (`DROP`, `HOLD`, `REQUEST_HUMAN`)
because "nobody took it" still has to end somewhere, and dropping the work,
blocking the robot and spending a human's attention are different costs. The
choice is made by the interpretation when it decides to delegate, not defaulted
to by whichever branch was written first.

## Interface to the local nodes

Ports, not ROS types — defined here, mocked in the tests, implemented later.

```python
nav.eta(target) -> float            nav.can_reach(target) -> bool
nav.current_location() -> Target | None

mission.can_absorb(task) -> bool    mission.absorb(task)
mission.release(task)               mission.mark_transferred(task)
mission.current_task_id() -> int    mission.battery_percent() -> int

preemption.request_yield(reason)    preemption.clear()
```

The infeasibility signal arrives on `/coordination/infeasible` from the triage
agent; `session.report_infeasible(task, detail, reason_code)` is the same entry
point for anything in-process.

`CoordinatorNode` takes all of these as constructor arguments. `main()` passes
placeholders (`UnavailableNav`, `UnavailableMission`) that decline everything
and say so once in the log — enough to run on a bench with two radios and watch
the peer registry populate. Wiring the real stacks means passing two objects
and changing nothing else.

### The wired ports: `coordinator_sim`

`nodes/sim_node.py` is the second entry point, and it does exactly that — three
objects to the same constructor, and not one line of `engine/coordinator.py`
or `nodes/coordinator_node.py` knows it exists.

| port | implementation | reads |
| --- | --- | --- |
| nav | `nav_ports.GpsNav` | `gps/pvt`, `/orchard/tree_info_json` |
| mission | `mission_ports.BehaviorTreeMission` | `mission/coordination_state` |
| replanner | `replanner_client.VerifyingReplanner` | `/mission/verify_replan` |

**Every one of them answers from cache, and none of them blocks.** That is not
a shortcut, it is the constraint: `_assess` calls `can_reach`, `eta`,
`can_absorb`, `current_task_id` and `battery_percent` from `_on_announce`,
under the lock that `tick` and `on_message` need, and `_replan` is called under
it too. A service call, an action, or a model round trip in any of them stops
this robot's heartbeats and auctions for its duration, and a robot that stops
answering is one the fleet writes off as dead.

Two consequences worth stating plainly:

* **Travel is costed as a straight line**, not a planned route. The fleet works
  unknown, changing environments — the global costmap builds as the robot
  drives and is not a surveyed map — so a planner asked about a place the robot
  has never been near answers "is there a path through what I have already
  seen", not "can I get there". `NavInterface.eta` only ever promised an
  answer that is *comparable* between robots, and distance over a nominal speed
  is exactly that.
* **`absorb`, `release` and `mark_transferred` do not edit the plan.** The
  arbiter does, from the same ownership change sent over `VerifyReplan`. Two
  writers for one event is how a subtree gets grafted twice.

`can_absorb` and `battery_percent` come from `mission_bridge` in
`amiga_ros2_agents`, which watches `/mission/xml` and republishes a latched
summary. The coordinator still never parses XML.

## Reliability, consumed in-process

`main()` constructs a `ReliabilityNode` and adds both to one executor:

```python
reliability = ReliabilityNode()
coordinator = CoordinatorNode(reliability=reliability, nav=..., mission=...)
executor.add_node(reliability); executor.add_node(coordinator)
```

In-process rather than over a ROS service for the same reason the layer below
says so: the interface carries typed codec messages and a delivery outcome, and
re-encoding those into `.msg` files to cross a process boundary that does not
exist would buy a serialization hop and a second failure mode.

The executor is multi-threaded because the GRANT outcome callback runs on the
reliability layer's retransmit tick.

**Do not launch `lora_reliability.launch.py` alongside `coordinator.launch.py`.**
Two reliability layers on one radio would each ACK the other's inbound traffic
and duplicate every send.

## Parameters

| parameter | default | notes |
| --- | --- | --- |
| `mission_schema` | installed `amiga_btcpp.xsd` | The schema this robot's tree validates against. Its `ActionGroup` is what the robot advertises it can do |
| `capabilities` | `[]` | Override, by XML element name (`SampleLeaf`, `MoveToTreeID`, …). For hardware temporarily absent from a robot whose schema still permits the action. An unknown name is **refused at startup** — a typo that silently drops `SampleLeaf` is the hardest kind of misconfiguration to find. Empty means read the schema |
| `announce_window_sec` | `5.0` | how long an announcement collects bids |
| `announce_repeat_sec` | `2.0` | re-broadcast interval while a window is open; `0` disables |
| `bid_max_backoff_sec` | `2.0` | longest a bidder waits. **Must be < `announce_window_sec`** |
| `bid_jitter_fraction` | `0.05` | separates identically-fit bidders |
| `bid_memory_sec` | `60.0` | how long a sent bid is remembered so a GRANT can be matched to it. Must outlast the announcer's window plus its whole GRANT retransmit campaign |
| `max_delegation_attempts` | `2` | announce-and-grant cycles per task before local handling |
| `redelegation_cooldown_sec` | `30.0` | floor between re-announcements of one task, so a mission node looping on one failure cannot become an announce storm |
| `peer_timeout_sec` | `30.0` | silence before a peer is presumed gone |
| `heartbeat_period_sec` | `10.0` | our own HEARTBEAT rate; `0` disables emission |
| `settled_retention_sec` | `300.0` | how long a settled task stays introspectable |
| `max_open_auctions` | `8` | beyond it, re-delegation is refused and the task is handled locally |
| `tick_period_sec` | `0.1` | granularity of every deadline here; keep well under `bid_max_backoff_sec` |
| `stats_period_sec` | `30.0` | counters log line; `0` disables |
| `infeasible_topic` | `/coordination/infeasible` | where "this robot cannot recover on its own" arrives |
| `use_triage_agent` | `true` | ask the triage agent. `false` falls back to the local stub — a bench setting, not a policy |
| `triage_service` | `/coordination/interpret_anomaly` | the agent's service |
| `triage_timeout_sec` | `45.0` | on a timeout the anomaly goes unanswered and the task stays put |
| `default_task_capabilities` | `[MoveToTreeID]` | assumed for a task the escalation names without describing. Only reached when the triage agent could not resolve the failing node to a subtree — an ordinary escalation carries the real action set |

`node_id` and the retransmit settings belong to the in-process reliability node
and are documented in
[`../../amiga_ros2_comms/docs/reliability_layer.md`](../../amiga_ros2_comms/docs/reliability_layer.md).

## Bounds worth knowing

- **The backoff/window relationship is fleet-wide.** The node checks its own
  pair, but the bidder that matters is on another robot. A fleet where the two
  disagree is a fleet where nobody's auction works.
- **An announcement is a broadcast**, so it is lost silently. `announce_repeat_sec`
  is the only thing that gets it to a robot that was transmitting.
- **A GRANT matching no bid of ours is an orphan.** It has already been ACKed by
  the reliability layer, so the announcer believes the task is ours, and a GRANT
  carries only a task ID — there is nothing to absorb even in principle. Counted
  as `grants_unmatched` and logged loudly rather than swallowed.
- **A rejected replan after absorbing a task re-enters the anomaly path.** "Work
  we hold and cannot complete" is exactly what that path is for, and it is the
  one route that can offer the task back to the fleet.
- **Ports are allowed to fail.** Nav and the mission stack are separate
  processes; every call through them is guarded, and a nav that cannot answer
  produces an infeasible bid rather than an exception. A fleet that stops
  arbitrating because one robot's navigation stack is reloading is worse than
  one that bids badly.

## Testing

The engine (`CoordinatorSession`) has **no ROS in it**. The clock is injected,
transport is a port and nav/mission are protocols, so the acceptance tests
drive whole auctions, backoff windows and peer timeouts in
microseconds with no executor, no sleeping and no flakiness. Bid jitter is
switched off in tests, so a cost maps to exactly one backoff.

```
python3 -m pytest amiga_ros2_coordinator/test/ -q
```

The eight acceptance groups, all in
[`test_coordinator.py`](../test/test_coordinator.py):

| group | what it pins |
| --- | --- |
| owner happy path | infeasible -> ANNOUNCE -> two BIDs -> best GRANTed -> delivered -> transferred, `replan_and_verify` called once |
| owner GRANT fails | reported failed -> **not** transferred -> falls back to the runner-up |
| owner no-bid timeout | window closes with no viable bid -> local handling, **no GRANT sent** |
| bidder bids | capable announcement -> nav/mission queried -> BID after the backoff |
| bidder suppression | a better bid overheard mid-backoff -> ours never reaches the air |
| bidder wins | GRANT addressed to us -> absorbed -> `replan_and_verify` called |
| registry | HEARTBEATs populate the table; a silent peer ages out |
| safe preemption | a coordination event raises the flag, and **no hard interrupt is issued** |

Plus the interpretation guard (free text refused, a failing interpreter leaves
the task untouched), the rejected-replan branch, the parameter cross-checks,
and port failures.

[`test_coordinator_node.py`](../test/test_coordinator_node.py) covers what only
a running node can show: that parameters reach the session, that the tick timer
drives the deadlines, that the preemption flag is latched — and **a task
crossing two robots**: two coordinators, two reliability layers, a relay for
the radio, real ROS in between, and nothing faked but nav and the mission
stack.

[`test_fleet_auction.py`](../test/test_fleet_auction.py) does the same with
**three robots on one shared channel**, which is the smallest fleet where the
mechanism is observable rather than tautological: with a single possible bidder
there is nothing to order and nothing to suppress. It runs the scenario `make
fleet-scenario` runs by hand — robot 1 sheds a task with a note on it, the note
fragments precede the announcement, robot 1 leaves the task immediately, the
closer of the two bidders wins and the further one suppresses itself on
overhearing it.

The last test there is the control, and it is the one the note design rests on:
**the same auction reaches the same winner when every note fragment is
dropped**. The channel is a node that copies frames, so it can filter by
decoded message type and remove exactly the `Freeform` frames and nothing else.
A note improves a decision; it is never what makes one possible.

## Not done yet

- **`replan_and_verify` — asynchrony.** Now `VerifyingReplanner`, which reaches
  the arbiter's `/mission/verify_replan`; the arbiter applies the edit to the
  plan it holds and runs it through the same `_evaluate` a planner-authored
  candidate goes through. What is *not* resolved is that this is called from
  inside the lock `tick` and `on_message` need, and one round trip costs a C
  compile (`spin -search` regenerates and builds `pan`, ~1.4 s measured on the
  Jetson) before any model call. So the request is dispatched on a thread and
  the verdict arrives late, routed to the anomaly path — the same place the
  inline rejection in `_take_on` goes, just later. Nothing unverified reaches
  the robot regardless, because the arbiter is the sole writer of
  `/mission/xml` and publishes only after the gate passes. Making the *coordinator*
  wait for a verdict would need the lock split, which is a larger change than
  the hook itself.
- ~~**Real nav and mission adapters**~~ — written; see "The wired ports" above.
  `GpsNav`, `BehaviorTreeMission` and `mission_bridge` are behind
  `coordinator_sim`. What the decision came out as, since it was called out here
  as unmade: **bidders do not pre-verify.** `can_absorb` answers from a cached
  capacity number, so it stays fast and certain and the announce window stays
  short; the winner's arbiter gates the edit *after* the ACK, and a rejection
  comes back around as an anomaly. So bidders do commit before they know they
  can — the other horn of that dilemma — and the cost is bounded by the fact
  that a rejected absorb re-offers the task rather than losing it. Dragging an
  LLM call into the auction was the alternative, and the note path already shows
  what that costs: a 5 s window becomes 45 s.
- ~~**The winner's side of the task ↔ mission mapping**~~ — wired, but only for
  the shape `mission_tasks.synthesize` supports: a **tree**-targeted task using
  `MoveToTreeID` and optionally `SampleLeaf`. GPS- and aisle-targeted tasks can
  be announced, bid on and won, and then the winner cannot rebuild a subtree for
  them. `GpsNav` resolves all three kinds, so the auction is wider than the
  execution; `escalate --gps` warns about exactly this.
- **COMPLETE and RELEASE.** The other unicast types, once the codec has them —
  a task finishing and a task being handed back are currently invisible to the
  fleet.
- ~~**FREEFORM and fragmentation**~~ — built. A `ReDelegate` may carry a `note`,
  which is broadcast before its announcement and interpreted by the
  `NoteInterpreter` port into one of `KeepBid | ReviseBid | WithdrawBid`. An
  auction carrying a note runs on the deliberative clock
  (`note_window_multiplier`), because a model call is orders of magnitude
  slower than a bid backoff.

  Still deferred: a real `NoteInterpreter` behind the port (the default,
  `IgnoreNotes`, records notes and changes nothing), and forward error
  correction for the fragments — a note that loses one is dropped.
- **Cross-vendor / wire-level A2A** (Paper B).

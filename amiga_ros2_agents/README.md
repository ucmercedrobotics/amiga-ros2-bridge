# amiga_ros2_agents

Home for every LLM agent on the Amiga. All five agents live in this one package
and talk to each other over ROS topics and services — there is no HTTP layer, no
agent discovery and no A2A. Everyone on this network is known ahead of time, so
a DDS participant is the whole story.

## Layout

Four directories, and what separates them is what each part is allowed to know.
The dependency direction runs down this list and never back up.

| Directory | What lives there |
| --- | --- |
| `runtime/` | How an agent is wired, with nothing about what it decides. |
| `mission/` | The plan document: its schema, and the tasks inside it. |
| `replanning/` | The self-correction loop — the agents that repair a plan in flight. |
| `coordination/` | Where this stack meets the fleet. |

`runtime/` knows nothing about missions. `mission/` is a library with no
rclpy in it, so it unit-tests with no ROS and no network. Only `replanning/`
and `coordination/` hold a ROS node. When an agent needs something a sibling
agent also needs, it imports the library, never the other agent.

| File | What it is |
| --- | --- |
| `runtime/llm.py` | The only LiteLLM entry point. Model/endpoint selection lives here. |
| `runtime/prompts.py` | The only Jinja2 entry point. `render("<agent>/<file>.j2", **vars)`. |
| `runtime/status.py` | `StatusPublisher` — every agent's status snapshot on `/agents/<name>/status`. |
| `runtime/spin.py` | `run()` — the init/spin/shutdown every agent's `main()` delegates to. |
| `mission/xsd.py` | BT.CPP mission schema loading, shared by the arbiter and the planner. |
| `mission/mission_tasks.py` | The only module that turns plan XML into task records and back. |
| `replanning/mission_planner_node.py` | **Mission planner** — minimally edits the BT.CPP XML plan on failure. |
| `replanning/arbiter_node.py` | **Arbiter** — gates candidate edits; sole writer of `/mission/xml`. |
| `replanning/world_state_node.py` | **World state** — aggregates action feedback into `/world_state`. |
| `coordination/triage_node.py` | **Triage** — decides what happens to work the local loop could not recover. |
| `coordination/note_node.py` | **Note** — reads a peer's note about work it is offering and says what it means for our bid. |
| `coordination/mission_bridge_node.py` | **Mission bridge** — answers the coordinator's questions about the plan, read-only. |
| `coordination/vlm_client.py` | How triage asks what the camera sees when a fault is captured. |

`world_state` sits under `replanning/` because the planner's context window is
what it feeds; `mission_bridge` sits under `coordination/` because the only
reason it exists is that the coordinator never parses XML.

## The replanning loop

```
                       /mission/xml
   tcp_demux_node ──────────────────▶ bt_runner
                            ▲             │
                            │             ▼ (BT node fails)
                        arbiter ◀── /bt/status_change ──▶ triage
                            ▲                               │
                            │                    /mission/fault_route
                            │                               │ repair
                            │                               ▼
                            │                        mission_planner
   /mission/rejection ──────┴─────── /mission/candidate_xml ◀────┘
   /mission/abort │                                             ▲
                  │     world_state ──── /world_state (1 Hz) ───┘
                  ▼
               triage ──── /coordination/infeasible ───▶ coordinator
                  ▲                                          │
                  └──── /coordination/interpret_anomaly ◀─────┘
```

`bt_runner` publishes `/bt/status_change` itself — see `FaultReporter` in
`amiga_ros2_behavior_tree`. Leaf failures only, one per node per interval, plus
a mission-level event when the whole tree returns. The topic is
`TRANSIENT_LOCAL`, so an agent that starts *because of* a fault still sees it.

The arbiter is the only node that publishes `/mission/xml`. The planner proposes;
the arbiter accepts (republish), rejects (planner retries with the reason) or
aborts (planner stops replanning for the rest of the mission).

`triage` sits at *both* ends of that chain, and does three things that are
deliberately different in kind.

**It routes the fault first.** The first FAILURE on a leaf reaches triage
before the planner acts on it: one model call over the fault and the `/rosout`
lines behind it answers "would a different plan fix this?", and the verdict —
`repair` or `escalate` — goes out on `/mission/fault_route`. The planner blocks
on that verdict (`ROUTE_TIMEOUT_SEC`, fail-open) and opens a session only for
`repair`. One call per failing node per mission; repeats of the same leaf get
the cached verdict re-published. This exists because the alternative was using
`MAX_RETRIES` as a proxy for incapacity, and a counter cannot tell a plan that
can be fixed from a camera that is broken — it spent three sessions on both, and
on the broken camera each session wrapped the dead leaf in another retry
decorator and replaced the plan the escalation needed to name its failing node
in.

**It looks at the scene, every time.** Everything else triage reads is the
robot's own account of itself — log lines, world state, the plan. None of it
says what is in front of the robot, and that is the fact both decisions turn on:
a sampler that fails because its camera is dead and one that fails because the
robot is parked a row over produce the same repeated error.

So with `use_vlm:=true`, a fault is captured the moment it arrives — the
fault-centred log slice, the world-state frame current at that instant, and one
call to `/vlm/ask` (the `vlm_server` in [`amiga_vlm`](../amiga_vlm)) asking the
camera to describe the scene. `_capture` does this on the routing thread, and
both decisions read that snapshot.

Two properties fall out of latching it at the fault rather than pulling it when
a prompt is built:

* **The evidence describes one moment.** The logs were always fault-centred; the
  world frame and the camera were not. For routing that gap was milliseconds.
  For interpretation it was minutes — the coordinator only asks once local
  recovery has run out — so the prompt was pairing a fault with a world frame
  and a picture from long afterwards and presenting them as one picture.
* **One camera call per fault**, not one per decision. Measured: routing and
  interpretation chained six times, zero extra calls for the second.

### The camera describes, the model decides

The question put to the camera is fixed and asks for a description only. That
division is the easiest thing here to get wrong:

* **No judgements.** "Is anything in the way", "is the row passable", "is the
  image usable" all sound like camera questions and are all decisions belonging
  to the model holding the logs, the plan, the battery and the fleet. A vision
  model asked them hands back a conclusion, and a conclusion is hard to argue
  with downstream — *"the row appears passable"* came back once and was repeated
  verbatim as the reason for a verdict.
* **No geometry.** The robot measures range and bearing. Asked for position
  anyway, a 4B describer put a person *"to the right"* while `collision_monitor`
  had an obstacle 0.62 m directly ahead, and the reasoner concluded the robot
  was misaligned.

What is left is the one thing only the camera knows: what the things in front of
the robot *are*.

### What it changes

Measured against a Gazebo run with a person standing in the robot's path, the
same navigation failure put through the pipeline twelve times with the camera
and twelve times without:

| | with the camera | without |
| --- | --- | --- |
| verdict | `repair` 12/12 | `repair` 12/12 |
| names a person | **11/12** | **0/12** |
| suggests waiting for them | 9/12 | 2/12 |
| median latency | 10.9 s | 7.0 s |

The verdict never changes; the reasoning behind it does. Without the camera the
model gets no further than *"an obstacle blocked the current path"* and plans a
way around it every time. With it: *"a man standing in the aisle; this is a
transient blockage"* — and the advice becomes *wait for the person to move*,
which is the right manoeuvre near a human in a narrow row and one the logs alone
give no reason to consider.

The vision model is a **separate model on a separate endpoint** from
`AGENT_MODEL` / `AGENT_API_BASE`, which are untouched by any of this. A small
vision model is enough precisely because it decides nothing.

Never load-bearing. A camera that is absent, slow or broken leaves the section
out or says it could not answer, and the verdict comes out as it always did.

**Escalation stays deterministic.** A routed `escalate`, an `/mission/abort`,
or a planner give-up on `/mission/planner_status` all publish to
`/coordination/infeasible` unchanged. **No model is involved in that step**: the
judgement was already made by whichever path got there. The give-up paths remain
as backstops — routing says "don't bother", the budget says "we bothered and it
didn't work", and both have to reach the fleet.

What *does* need a model again is
the coordinator's follow-up question, served on `/coordination/interpret_anomaly`:
given the fault, the logs around it, the world state and who else is on the
radio, should this task be offered to the fleet, dropped, or replaced with
different work? The answer is constrained to those three typed actions by the
service definition, so a coordinator built and tested against a stub does not
change when a real model sits behind it.

### The other side of the trade: `note`

Triage answers "what should happen to work *we* cannot finish". `note` answers
the question the robots on the receiving end have: another robot has offered
work and sent a sentence with it — does that sentence change what we bid?

They are separate agents because they are separate judgements from separate
evidence. Triage reasons from a fault it watched happen, over logs and world
state it collected. `note` subscribes to nothing at all: everything the decision
needs arrives in the request, because a note is *about* one specific
announcement and context from any other moment would be context about something
else.

The response is closed at three values — `keep`, `revise` (by a signed cost
delta), `withdraw` — and that closure is doing more work here than anywhere else
in this package. This is the only place where text written by another robot
reaches a decision, and the radio's `src` is self-asserted with no MAC, so the
sentence in front of the model could have been written by anyone in range. The
prompt does not defend against that and is not asked to. What defends against it
is that all three answers only adjust a bid this robot was already going to
make: a note cannot start an auction, take on work, drop work, or move the
robot. The worst outcome from the worst possible sentence is one bad bid, or
silence.

Both ends are refused twice — `note_node._parse_revision` before it goes on the
wire, and the coordinator's `note_client` after it arrives.

The cost of a note falls on the reader, which is why triage's prompt tells it to
send one rarely: reading a note makes the receiving auction deliberative, and a
5 s announce window becomes 45 s while every bidder thinks. `note_timeout_sec`
(15 s) has to stay under the note-stretched bid backoff (20 s at the defaults),
or every answer lands after its own bid and is counted `notes_too_late`; the
coordinator warns at startup when it does not.

### Verification

Every candidate plan — a local repair or a fleet-level task transfer alike —
passes through the same gate before it reaches the executing tree: well-formed
XML, schema validity against the mission grammar, the ontology's preconditions
(e.g. a leaf-sampling action requires the plan first establishes the robot is
at that tree), and objective/viability (the only check that can abort a
mission, bounding how much of it an edit may silently drop before local repair
gives up). All four are `arbiter_node.py:_evaluate`'s job; see its module
docstring for the exact order.

This pipeline used to include a fifth, formal step here — a temporal-logic
specification generated once from the pristine mission text, checked against
each candidate with SPIN — gated behind an `ltl_verification` flag. That check,
and the Promela model compiler it verified the plan against, have both been
removed; this pipeline does not use LTL.

## ROS interfaces

| Agent | Subscribes | Publishes | Services |
| --- | --- | --- | --- |
| `mission_planner` | `/mission/xml`, `/rosout`, `/bt/status_change`, `/mission/fault_route`, `/mission/rejection`, `/mission/abort`, `/world_state`, `/mission/replan_request` | `/mission/candidate_xml`, `/mission/planner_status` | — |
| `arbiter` | `/mission/candidate_xml`, `/mission/xml`, `/bt/status_change` | `/mission/xml`, `/mission/rejection`, `/mission/abort`, `/mission/viability_budget` | `/mission/verify_replan` (`amiga_interfaces/srv/VerifyReplan`) |
| `world_state` | action feedback/status topics, `/mission_status` | `/world_state` (1 Hz JSON) | — |
| `triage` | `/bt/status_change`, `/rosout`, `/world_state`, `/mission/xml`, `/mission/abort`, `/mission/planner_status` | `/mission/fault_route`, `/coordination/infeasible` | `/coordination/interpret_anomaly` (`amiga_interfaces/srv/InterpretAnomaly`) |
| `note` | — | — | `/coordination/interpret_note` (`amiga_interfaces/srv/InterpretNote`) |
| `mission_bridge` | `mission/xml` | `mission/coordination_state` (latched JSON) | — |

`mission_bridge` is the only node here whose topic names are *relative*, so a
namespaced instance is per-robot with no further help. Everything else in this
package hardcodes absolute names, which is right for the real machine — one
agent stack, sitting outside any namespace.

### One stack per robot

A simulated fleet needs several, and `robot_agents.launch.py` is how: it starts
the same set under a namespace and remaps every absolute name above to its
*relative* form, letting the node's own namespace place it.
`ROBOT_INTERFACES` in that file is the list, and a name missing from it is
visible rather than subtle — one subscription still on the fleet-wide topic
shows up immediately as every robot reacting to one robot's fault.

Making the source names relative is still the real fix; the remap table is what
lets the fleet run before that lands, without editing five node files.

**`/rosout` is deliberately not remapped.** There is one log stream per machine,
not one per robot, and both the planner and triage read it for the explanation
behind a fault — the fault event says a node failed, the node itself logged why.
Remapping it to a topic nobody publishes would quietly empty their most useful
context. The cost is that in a simulated fleet they also see each other's log
lines, which is noise in a prompt and nothing worse.

Every agent also publishes `/agents/<node_name>/status` as JSON. That topic is
`TRANSIENT_LOCAL` depth 1, so a late subscriber still gets the last value — which
also makes it a readiness signal, since the startup snapshot latches.

## Launching

Inside the container (`make bash`), from `/amiga-ros2-bridge`:

```bash
colcon build --packages-select amiga_interfaces amiga_ros2_agents
source install/setup.bash

ros2 launch amiga_ros2_agents replanning.launch.py   # planner + arbiter + world state
ros2 launch amiga_ros2_agents agents.launch.py       # every agent, unnamespaced
ros2 run amiga_ros2_agents arbiter                   # one agent

# One robot's stack inside a fleet. `make sim ROBOT_COUNT=3 AGENTS=true`
# includes this per robot; see the remap note above.
ros2 launch amiga_ros2_agents robot_agents.launch.py namespace:=amiga2
```

There is no wrapper script. `setup.cfg` sets the console-script shebang to
`/usr/bin/env python3`, and the image activates `/.venv` from `~/.bashrc`, so an
interactive container shell resolves that to the venv interpreter — which is where
litellm, jinja2 and lxml live. Do not build these with `--symlink-install`: that
generates the console scripts in setuptools `develop` mode, which ignores
`[build_scripts]` and hardcodes system Python, and the agents then die on import.

Point the agents at a model before launching — the defaults assume a local vLLM:

```bash
export AGENT_MODEL=hosted_vllm/openai/gpt-oss-20b   # must match `vllm serve <model>`
export AGENT_API_BASE=http://localhost:8000/v1      # "" for a cloud provider
```

The rest of `llm.py`'s knobs (`AGENT_TEMPERATURE`, `AGENT_MAX_TOKENS`,
`ENV_FILE_PATH`) and the prompt/schema overrides (`AGENT_PROMPT_DIR`,
`AMIGA_XSD_PATH`) are documented at the top of `llm.py`, `prompts.py` and
`xsd.py`.

## Example run

Start vLLM, wait for `curl -s localhost:8000/v1/models` to answer, then bring up
the loop:

```bash
export AGENT_MODEL=hosted_vllm/openai/gpt-oss-20b
ros2 launch amiga_ros2_agents replanning.launch.py
```

In a second shell, watch the loop and feed it a mission:

```bash
source install/setup.bash
ros2 topic echo /mission/candidate_xml &
ros2 topic echo /mission/rejection &

ros2 topic pub --once /mission/xml std_msgs/msg/String \
  "{data: '$(cat amiga_ros2_behavior_tree/examples/sample_leafs.xml)'}"
```

The arbiter captures the mission's objective trees and asks the model how many of
them may be skipped:

```
[arbiter] Captured original mission objectives: trees ['10', '60']
[arbiter] Model viability budget: up to 2 tree(s) may be skipped
```

Now inject a failure. `bt_runner` publishes these itself when a leaf fails, but
driving it by hand is how you exercise the loop without a robot:

```bash
ros2 topic pub --once /bt/status_change std_msgs/msg/String \
  "{data: '{\"node\":\"Visit_Tree_60\",\"reason\":\"navigation failed\",\"timestamp_ms\":0}'}"
```

The planner reads its `/world_state` window, edits the XML and publishes a
candidate; the arbiter accepts or rejects it:

```
[mission_planner]   Calling model (hosted_vllm/openai/gpt-oss-20b) — world_state=3 frames, logs=0 entries
[mission_planner]   Published candidate XML to /mission/candidate_xml
[arbiter] ACCEPTED candidate — published to /mission/xml
```

A rejection comes back on `/mission/rejection` with the reason and the planner
retries; after too many failures the arbiter publishes `/mission/abort` and the
planner stops replanning for the rest of the mission.

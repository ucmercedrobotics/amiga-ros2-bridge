# amiga_ros2_agents

Home for every LLM agent on the Amiga. All five agents live in this one package
and talk to each other over ROS topics and services — there is no HTTP layer, no
agent discovery and no A2A. Everyone on this network is known ahead of time, so
a DDS participant is the whole story.

## Layout

Five directories, and what separates them is what each part is allowed to know.
The dependency direction runs down this list and never back up.

| Directory | What lives there |
| --- | --- |
| `runtime/` | How an agent is wired, with nothing about what it decides. |
| `mission/` | The plan document: its schema, and the tasks inside it. |
| `verification/` | The LTL specification, the Promela model, and SPIN. |
| `replanning/` | The self-correction loop — the agents that repair a plan in flight. |
| `coordination/` | Where this stack meets the fleet. |

`runtime/` knows nothing about missions. `mission/` and `verification/` are
libraries with no rclpy in them, so they unit-test with no ROS and no network —
which is what makes the verification claim below checkable on its own. Only
`replanning/` and `coordination/` hold a ROS node. When an agent needs
something a sibling agent also needs, it imports the library, never the other
agent.

| File | What it is |
| --- | --- |
| `runtime/llm.py` | The only LiteLLM entry point. Model/endpoint selection lives here. |
| `runtime/prompts.py` | The only Jinja2 entry point. `render("<agent>/<file>.j2", **vars)`. |
| `runtime/status.py` | `StatusPublisher` — every agent's status snapshot on `/agents/<name>/status`. |
| `runtime/spin.py` | `run()` — the init/spin/shutdown every agent's `main()` delegates to. |
| `mission/xsd.py` | BT.CPP mission schema loading, shared by the arbiter and the planner. |
| `mission/mission_tasks.py` | The only module that turns plan XML into task records and back. |
| `verification/ltl.py` | Mission text → LTL formula. The specification. |
| `verification/promela.py` | Behaviour tree → Promela model, and the propositions it establishes. |
| `verification/verify.py` | SPIN, and the three-way verdict: holds, violated, or not established. |
| `verification/ltl_gate.py` | The four ordered checks the arbiter runs. |
| `verification/ltl_gen_node.py` | **LTL generator** — natural-language mission → Promela/SPIN LTL. |
| `replanning/mission_planner_node.py` | **Mission planner** — minimally edits the BT.CPP XML plan on failure. |
| `replanning/arbiter_node.py` | **Arbiter** — gates candidate edits; sole writer of `/mission/xml`. |
| `replanning/world_state_node.py` | **World state** — aggregates action feedback into `/world_state`. |
| `coordination/triage_node.py` | **Triage** — decides what happens to work the local loop could not recover. |
| `coordination/mission_bridge_node.py` | **Mission bridge** — answers the coordinator's questions about the plan, read-only. |

`world_state` sits under `replanning/` because the planner's context window is
what it feeds; `mission_bridge` sits under `coordination/` because the only
reason it exists is that the coordinator never parses XML.
| `mission_bridge_node.py` | **Mission bridge** — not an agent, and no model. Summarises `/mission/xml` for the coordinator's mission port, so the coordinator never parses XML. |

## The replanning loop

```
                       /mission/xml
   tcp_demux_node ──────────────────▶ bt_runner
                            ▲             │
                            │             ▼ (BT node fails)
                        arbiter ◀── /bt/status_change ──▶ mission_planner
                            ▲                                   │
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

`triage` sits at the end of that chain and does two things that are deliberately
different in kind. When the loop above gives up — `/mission/abort`, or a planner
give-up on `/mission/planner_status` — it escalates to the coordinator on
`/coordination/infeasible`. **No model is involved in that**: the planner and the
arbiter already decided local recovery was exhausted. What *does* need a model is
the coordinator's follow-up question, served on `/coordination/interpret_anomaly`:
given the fault, the logs around it, the world state and who else is on the
radio, should this task be offered to the fleet, dropped, or replaced with
different work? The answer is constrained to those three typed actions by the
service definition, so a coordinator built and tested against a stub does not
change when a real model sits behind it.

`ltl_gen` exposes the same translation over ROS — `/mission/generate_ltl` or the
`/mission/text` topic — but the arbiter does not go through it. The gate needs a
formula *inside* the decision that publishes `/mission/xml`, and a service call
from there would make the gate depend on another node being up. Both share
`ltl.py`, so there is still one place that turns English into a formula.

### Verification

Every candidate plan is checked against the LTL specification its own mission
text yields, and the two halves are produced independently: `ltl.py` sees the
`<Mission>` text and never the tree, `promela.py` compiles the tree and never
sees the formula. Neither can be bent to fit the other, which is what makes
their agreement evidence rather than construction.

That only holds if the mission text is fixed, so **a replan may not rewrite
`<Mission>`** — a planner that could would be authoring the specification it is
graded against. The one exception is the coordinator absorbing a peer's task,
which genuinely changes what this robot's mission is; the clause is generated
from the announcement's own fields, never by a model.

All of it is in `verification/`, and the four library modules there hold no
rclpy — the model checking is something you can point at a plan and a formula
with no robot in the loop, which is why `test_verify.py` runs SPIN for real
rather than mocking it. `ltl_gen_node` is the one agent in that directory.
Atomic propositions are named by content — `at_tree_10`, `sampled_tree_10` — so
they survive a replan reordering or dropping tasks. The naming scheme is
mandated by `prompts/ltl_gen/system.j2` and emitted by `promela.py`; that shared
convention is the whole contract between the two halves.

Needs `spin` on PATH (`scripts/ci/install_spin.sh`, built from source so it
works on aarch64). Without it, plans are accepted and recorded as
`ltl_unverified` — never silently as verified.

## ROS interfaces

| Agent | Subscribes | Publishes | Services |
| --- | --- | --- | --- |
| `mission_planner` | `/mission/xml`, `/rosout`, `/bt/status_change`, `/mission/rejection`, `/mission/abort`, `/world_state` | `/mission/candidate_xml`, `/mission/planner_status` | — |
| `arbiter` | `/mission/candidate_xml`, `/mission/xml`, `/bt/status_change` | `/mission/xml`, `/mission/rejection`, `/mission/abort`, `/mission/viability_budget` | `/mission/verify_replan` (`amiga_interfaces/srv/VerifyReplan`) |
| `world_state` | action feedback/status topics, `/mission_status` | `/world_state` (1 Hz JSON) | — |
| `ltl_gen` | `/mission/text` | `/mission/ltl` | `/mission/generate_ltl` (`amiga_interfaces/srv/GenerateLTL`) |
| `triage` | `/bt/status_change`, `/rosout`, `/world_state`, `/mission/xml`, `/mission/abort`, `/mission/planner_status` | `/coordination/infeasible` | `/coordination/interpret_anomaly` (`amiga_interfaces/srv/InterpretAnomaly`) |
| `mission_bridge` | `mission/xml` | `mission/coordination_state` (latched JSON) | — |

`mission_bridge` is the only node here whose topic names are *relative*, so a
namespaced instance is per-robot. Everything else in this package hardcodes
absolute names (`/mission/xml`, `/bt/status_change`, `/world_state`), which is
fine for one agent stack and is what has to change before a fleet can run one
stack per robot.

Every agent also publishes `/agents/<node_name>/status` as JSON. That topic is
`TRANSIENT_LOCAL` depth 1, so a late subscriber still gets the last value — which
also makes it a readiness signal, since the startup snapshot latches.

## Launching

Inside the container (`make bash`), from `/amiga-ros2-bridge`:

```bash
colcon build --packages-select amiga_interfaces amiga_ros2_agents
source install/setup.bash

ros2 launch amiga_ros2_agents replanning.launch.py   # planner + arbiter + world state
ros2 launch amiga_ros2_agents agents.launch.py       # all four
ros2 run amiga_ros2_agents arbiter                   # one agent
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

For the LTL agent (needs `agents.launch.py`):

```bash
ros2 service call /mission/generate_ltl amiga_interfaces/srv/GenerateLTL \
  "{mission: 'visit trees 1 through 3 and sample the leaves at each'}"
# ok: true, formula: <>(at_tree_1 && <>sampled_tree_1) && <>(at_tree_2 && ...)
```

To pin the atomic propositions instead of letting the model invent them:

```bash
ros2 launch amiga_ros2_agents agents.launch.py ap_vocabulary:="[at_tree_1, sampled_tree_1]"
```

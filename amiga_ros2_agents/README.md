# amiga_ros2_agents

Home for every LLM agent on the Amiga. All five agents live in this one package
and talk to each other over ROS topics and services — there is no HTTP layer, no
agent discovery and no A2A. Everyone on this network is known ahead of time, so
a DDS participant is the whole story.

| File | What it is |
| --- | --- |
| `llm.py` | The only LiteLLM entry point. Model/endpoint selection lives here. |
| `prompts.py` | The only Jinja2 entry point. `render("<agent>/<file>.j2", **vars)`. |
| `status.py` | `StatusPublisher` — every agent's status snapshot on `/agents/<name>/status`. |
| `xsd.py` | BT.CPP mission schema loading, shared by the arbiter and the planner. |
| `spin.py` | `run()` — the init/spin/shutdown every agent's `main()` delegates to. |
| `mission_planner_node.py` | **Mission planner** — minimally edits the BT.CPP XML plan on failure. |
| `arbiter_node.py` | **Arbiter** — gates candidate edits; sole writer of `/mission/xml`. |
| `world_state_node.py` | **World state** — aggregates action feedback into `/world_state`. |
| `ltl_gen_node.py` | **LTL generator** — natural-language mission → Promela/SPIN LTL. |
| `triage_node.py` | **Triage** — decides what happens to work the local loop could not recover. |

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

`ltl_gen` sits outside this loop: it turns a plain-English mission into an LTL
formula on `/mission/ltl`, via the `/mission/generate_ltl` service or the
`/mission/text` topic.

## ROS interfaces

| Agent | Subscribes | Publishes | Services |
| --- | --- | --- | --- |
| `mission_planner` | `/mission/xml`, `/rosout`, `/bt/status_change`, `/mission/rejection`, `/mission/abort`, `/world_state` | `/mission/candidate_xml`, `/mission/planner_status` | — |
| `arbiter` | `/mission/candidate_xml`, `/mission/xml`, `/bt/status_change` | `/mission/xml`, `/mission/rejection`, `/mission/abort`, `/mission/viability_budget` | — |
| `world_state` | action feedback/status topics, `/mission_status` | `/world_state` (1 Hz JSON) | — |
| `ltl_gen` | `/mission/text` | `/mission/ltl` | `/mission/generate_ltl` (`amiga_interfaces/srv/GenerateLTL`) |
| `triage` | `/bt/status_change`, `/rosout`, `/world_state`, `/mission/xml`, `/mission/abort`, `/mission/planner_status` | `/coordination/infeasible` | `/coordination/interpret_anomaly` (`amiga_interfaces/srv/InterpretAnomaly`) |

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

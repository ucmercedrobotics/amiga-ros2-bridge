# amiga_ros2_agents

Home for the Amiga A2A agents. Each agent is one module at the package root and
shares two helpers:

| File | What it is |
| --- | --- |
| `llm.py` | The only LiteLLM entry point. Model/endpoint selection lives here. |
| `a2a_server.py` | `serve_agent()` (spin thread + uvicorn + A2A app), `agent_message()`, `StatusExecutor`. |
| `agent_card.py` | Every agent card, with its port. |
| `ltl_gen_node.py` | **LTL generator** — natural-language mission → Promela/SPIN LTL. Port 20001. |
| `dummy_node.py` | Placeholder agent, proves multiple agents coexist. Port 20002. |

Ports use the 20000 range to stay clear of the standalone agents
(mission planner 10001, arbiter 10003, world state 10004).

## Adding an agent

1. Add its card to `agent_card.py`.
2. Write `<name>_node.py`: a `Node` subclass holding the state, an `AgentExecutor`
   (or reuse `StatusExecutor` if it's status-only), and a `main()` that calls
   `serve_agent(node, executor, CARD, PORT)`.
3. Add a `console_scripts` entry point in `setup.py` and a `Node` action in
   `launch/agents.launch.py`.

Use `llm.complete(system, user)` for any model call — don't import litellm directly.

## Configuration

`llm.py` reads these; the `LOCAL_*` fallbacks match what `scripts/run_*.sh` already export.

| Env var | Default |
| --- | --- |
| `AGENT_MODEL` / `LOCAL_MODEL` | `hosted_vllm/openai/gpt-oss-20b` |
| `AGENT_API_BASE` / `LOCAL_API_BASE` | `http://localhost:8000/v1` (`""` = provider default) |
| `AGENT_TEMPERATURE` | `0.2` |
| `AGENT_MAX_TOKENS` | `2048` |
| `ENV_FILE_PATH` | `/amiga-ros2-bridge/.env` (provider API keys) |

For a cloud model instead of the local vLLM endpoint:

```bash
export AGENT_MODEL=gpt-5.6-sol
export AGENT_API_BASE=""      # official OpenAI endpoint; key comes from .env
```

## Running

Inside the container (`make bash` / `make shell`):

```bash
colcon build --packages-select amiga_ros2_agents
source install/setup.bash

ros2 run amiga_ros2_agents ltl_gen        # port 20001
ros2 run amiga_ros2_agents dummy_agent    # port 20002
# or both:
ros2 launch amiga_ros2_agents agents.launch.py
```

## LTL agent

Two ways in, both publishing the formula to `/mission/ltl`:

```bash
# A2A (this is the path an LLM/orchestrator uses)
python3 amiga_ros2_agents/tools/ltl_client_demo.py "visit trees 1 through 3 and sample the leaves at each"

# ROS
ros2 topic pub --once /mission/text std_msgs/String "{data: 'patrol trees 1 and 2 forever'}"
ros2 topic echo /mission/ltl
```

Expected shape of the output — SPIN operators, identifier-only propositions:

```
<>(at_tree_1 && <>sampled_tree_1) && <>(at_tree_2 && <>sampled_tree_2) && <>(at_tree_3 && <>sampled_tree_3)
```

The reply carries the formula as a text part and `{mission, formula, model, ok, error}`
as a data part. Failures come back with `ok: false` and a reason rather than an
HTTP error, so a caller always gets an answer.

The agent card is at `http://localhost:20001/.well-known/agent-card.json`.

### Known gaps

- Atomic propositions are invented by the model. Once the Promela typedefs exist,
  inject them into `LTL_SYSTEM_PROMPT` as the fixed AP vocabulary (there's a TODO
  marking the spot).
- Validation is a smoke test only (non-empty, balanced parens, contains a
  temporal/boolean operator, doesn't read as prose) — enough to keep garbage off
  `/mission/ltl`, but real validation means shelling out to `spin -f`.

# amiga_vlm

A vision-language model, exposed to the rest of the robot as one service.

```
/oak0/rgb/image_raw ──▶ vlm_server ──HTTP──▶ your VLM endpoint
                            ▲
                            └── /vlm/ask (amiga_vlm_interfaces/srv/VlmAsk)
```

Two packages: `amiga_vlm_interfaces` holds `VlmAsk.srv` and nothing else, so a
caller depends on the service definition without dragging in OpenCV;
`amiga_vlm_bridge` holds the node.

## Scope

The node keeps **only the latest frame** and nothing else. On a request it
JPEG-encodes that frame, base64s it into an OpenAI-style `image_url` content
part, POSTs it, and returns the text. No history, no state, no decisions, and
nothing published — a question in, a sentence out.

The test for whether a change belongs here: it would work unchanged if the
question came from a person instead of an agent. Anything about *what to ask*
or *what the answer means* belongs to the caller.

## Running it

Needs an OpenAI-compatible endpoint that accepts image content parts — a vision
model, served **separately from the reasoning model the agents use**. Different
model, different endpoint, different job: this one describes a picture, the
agents' model decides what to do about it.

```bash
vllm serve google/gemma-4-E4B-it --port 8001     # vision only
```

A small model is the right choice here rather than a compromise: everything
downstream treats the answer as one more piece of evidence weighed by the
reasoning model, so what is wanted is an accurate description of a frame.
Measured with the above on a Jetson Thor over Tailscale, on a 1280×720 Oak-D
frame: ~1.5 s and ~400 prompt tokens, the image having been downsampled by the
vision encoder long before the KV cache sees it.

Then, with a camera publishing (`make oakd` on the robot, or `make sim`):

```bash
make vlm                                    # or: ros2 run amiga_vlm_bridge vlm_server
make vlm-ask VLM_QUESTION="How many trees are in this row?"
```

Usually you do not run it by hand: both agent launch files start one on
`launch_vlm:=true`, wired to this robot's own camera and its own triage agent.

```bash
ros2 launch amiga_ros2_agents agents.launch.py launch_vlm:=true       # real robot
make sim ROBOT_COUNT=3 AGENTS=true VLM=true                           # simulated fleet
```

## Parameters

| parameter | default | notes |
| --- | --- | --- |
| `image_topic` | `/oak0/rgb/image_raw` | Oak-D front camera; the Gazebo shim republishes under the same name |
| `service_name` | `/vlm/ask` | relative (`vlm/ask`) under a namespace, for a fleet |
| `vlm_url` | `http://localhost:8001/v1/chat/completions` | |
| `system_prompt` | generic | the agent launch files pass an orchard-specific one |
| `max_tokens` | `256` | |
| `min_tokens` | `0` | a **vLLM extension**; sent only when above 0, so the default request is one any OpenAI-compatible server accepts |
| `jpeg_quality` | `85` | |
| `http_timeout_sec` | `180.0` | generous for a hand-driven call against a cold model; every automated caller overrides it downward |

## Who calls it

The **triage agent** in [`amiga_ros2_agents`](../amiga_ros2_agents), when
`use_vlm` is set. It calls once per behaviour-tree failure, at the moment the
fault arrives, and both of its decisions reuse that one description rather than
asking again.

The question is always the same — *describe what you see* — and it asks for a
description only. The service takes a question rather than fixing it internally
so a different caller can ask a different thing, not because triage varies it.

Two consequences for anything else that calls this later:

* **it must not be on a deadline path without its own timeout.** Triage gives up
  at 8 s, and the agent launch files pass `http_timeout_sec:=6.0` so the far end
  gives up first — a reply that lands after the caller stopped waiting is worse
  than an error, because the log then blames the wrong thing.
* **the answer is a description, not a decision.** Asking this service what to
  *do* would move the decision to the one component that can see the least.

## Concurrency

`MultiThreadedExecutor`, with the subscription and the service in separate
callback groups. That is load-bearing rather than tidiness: the service handler
blocks on HTTP for seconds, and on the default single-threaded executor nothing
else in the node runs for that whole time — so the frame buffer stops being
refilled and the next question is answered from a picture taken before the last
inference began. Inference itself is still serialised by `_infer_lock`; a
shared GPU does not go faster for being asked twice at once.

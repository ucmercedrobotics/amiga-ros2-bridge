import requests
import json

WORLD_STATE_URL = "http://localhost:10004/"
OLLAMA_URL      = "http://localhost:11434/api/chat" # Ollama API endpoint
OLLAMA_MODEL    = "gemma4" # can change to different model if desired
STREAM_UPDATES  = 3   # how many telemetry updates to collect before querying Ollama

print("--- Streaming telemetry from World State agent ---")
robot_state = {}
count = 0

# message/stream opens an SSE connection and receives continuous updates
with requests.post(WORLD_STATE_URL, json={
    "jsonrpc": "2.0",
    "id": "stream-1",
    "method": "message/stream",
    "params": {
        "message": {
            "role": "user",
            "messageId": "msg-1",
            "parts": [{"type": "text", "text": "stream context"}]
        }
    }
}, stream=True) as resp:
    for line in resp.iter_lines():
        if not line:
            continue
        # SSE lines come as "data: {...}"
        raw = line.decode("utf-8")
        if raw.startswith("data:"):
            payload = json.loads(raw[5:].strip())
            print(f"Update {count + 1}: {json.dumps(payload, indent=2)}")
            # extract state from the data part
            try:
                parts = payload["result"]["parts"]
                for part in parts:
                    if "data" in part:
                        robot_state = part["data"]
            except (KeyError, TypeError):
                pass
            count += 1
            if count >= STREAM_UPDATES:
                break

print(f"\nLast known robot state:\n{json.dumps(robot_state, indent=2)}")

# Feed the latest state to Ollama
print("\n--- Querying Ollama/Gemma with robot context ---")
prompt = f"""You are a mission planner for an autonomous agricultural robot.
Here is the current robot state:
{json.dumps(robot_state, indent=2)}

Based on this state, suggest one simple mission the robot could perform right now.
Keep your answer to 2-3 sentences."""

ollama_response = requests.post(OLLAMA_URL, json={
    "model": OLLAMA_MODEL,
    "messages": [{"role": "user", "content": prompt}],
    "stream": False
})
ollama_response.raise_for_status()
print(f"\nGemma says:\n{ollama_response.json()['message']['content']}")
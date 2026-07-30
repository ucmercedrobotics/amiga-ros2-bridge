#!/usr/bin/env python3
"""Manual A2A client for the LTL agent — the "LLM talking to the agent" path.

Usage:
    python3 amiga_ros2_agents/tools/ltl_client_demo.py "visit trees 1 through 3 and sample the leaves at each"

Plain JSON-RPC over requests, same style as scripts/test_llm_world_state.py, so
it runs without the a2a client library.
"""

import json
import os
import sys
import uuid

import requests

LTL_AGENT_URL = os.environ.get("LTL_AGENT_URL", "http://localhost:20001/")
DEFAULT_MISSION = "visit trees 1 through 3 and sample the leaves at each"


def generate_ltl(mission: str) -> dict:
    response = requests.post(
        LTL_AGENT_URL,
        json={
            "jsonrpc": "2.0",
            "id": str(uuid.uuid4()),
            "method": "message/send",
            "params": {
                "message": {
                    "role": "user",
                    "messageId": str(uuid.uuid4()),
                    "kind": "message",
                    "parts": [{"kind": "text", "text": mission}],
                }
            },
        },
        timeout=180,
    )
    response.raise_for_status()
    return response.json()


def main():
    mission = " ".join(sys.argv[1:]) or DEFAULT_MISSION
    print(f"Mission: {mission}\nAgent:   {LTL_AGENT_URL}\n")

    payload = generate_ltl(mission)
    print(json.dumps(payload, indent=2))

    # The agent replies with a text part (the formula) and a data part (details)
    formula, details = None, {}
    for part in payload.get("result", {}).get("parts", []):
        if "text" in part:
            formula = part["text"]
        elif "data" in part:
            details = part["data"]

    print(f"\nLTL: {formula}")
    if not details.get("ok", True):
        print(f"FAILED: {details.get('error')}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

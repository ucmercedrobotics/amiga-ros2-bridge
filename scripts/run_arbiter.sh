#!/bin/bash
source /opt/ros/humble/setup.bash
source /amiga-ros2-bridge/install/setup.bash
export PYTHONPATH=/.venv/lib/python3.10/site-packages:$PYTHONPATH
export LOCAL_MODEL="${LOCAL_MODEL:-hosted_vllm/openai/gpt-oss-20b}"
export LOCAL_API_BASE="${LOCAL_API_BASE:-http://localhost:8000/v1}"
export ENV_FILE_PATH="${ENV_FILE_PATH:-/amiga-ros2-bridge/.env}"
/.venv/bin/python3 -m amiga_ros2_arbiter.arbiter_node
#!/bin/bash
source /opt/ros/humble/setup.bash
source /amiga-ros2-bridge/install/setup.bash
export PYTHONPATH=/.venv/lib/python3.10/site-packages:$PYTHONPATH
export CLOUD_MODEL="${CLOUD_MODEL:-gpt-4o}"
export ENV_FILE_PATH="${ENV_FILE_PATH:-/amiga-ros2-bridge/.env}"
/.venv/bin/python3 -m amiga_ros2_arbiter.arbiter_node
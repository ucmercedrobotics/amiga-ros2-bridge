#!/bin/bash
source /opt/ros/humble/setup.bash
source /amiga-ros2-bridge/install/setup.bash
export PYTHONPATH=/.venv/lib/python3.10/site-packages:$PYTHONPATH
export WORLD_STATE_URL="${WORLD_STATE_URL:-http://localhost:10004/}"
export OLLAMA_URL="${OLLAMA_URL:-http://localhost:11434/api/chat}"
export OLLAMA_MODEL="${OLLAMA_MODEL:-gemma4}"
/.venv/bin/python3 -m amiga_ros2_mission_planner.mission_planner_node
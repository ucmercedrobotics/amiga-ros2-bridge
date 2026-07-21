#!/bin/bash
source /opt/ros/humble/setup.bash
source /amiga-ros2-bridge/install/setup.bash
export PYTHONPATH=/.venv/lib/python3.10/site-packages:$PYTHONPATH
/.venv/bin/python3 -m amiga_ros2_arbiter.arbiter_node
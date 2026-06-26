#!/bin/bash
source /opt/ros/humble/setup.bash
source /amiga-ros2-bridge/install/setup.bash
export PYTHONPATH=/amiga-ros2-bridge/.venv_a2a/lib/python3.10/site-packages:$PYTHONPATH
/amiga-ros2-bridge/.venv_a2a/bin/python3 -m amiga_ros2_world_state.world_state_node

#Steps to run world state node

1. Clone the repo:
    `git clone https://github.com/ucmercedrobotics/amiga-ros2-bridge.git`
    `cd amiga-ros2-bridge`

2. Make image
    `make build-image`

3. Enter the container
    `make bash`

    ignore Kortex control warning for now

4. Create a virtual environment
    `python -m venv .venv`
    `source /.venv/bin/activate`

5. Install Requirements
    `/usr/bin/python3 -m pip install -r requirements.txt`

6. Build packages
    `colcon build`
    `source install/setup.bash`

7. Run world state node
    `ros2 run amiga_ros2_world_state world_state`



###In a new terminal

1. `make shell`
2. `source install/setup.bash`

3. check if sourced correctly by seeing world state is running

    `ros2 node list`

    should display /world_state

4. run the test script
    `python3 scripts/test_llm_world_state.py`

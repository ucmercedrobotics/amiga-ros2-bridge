# FarmNG Amiga ROS2 Bridge
[![github](https://img.shields.io/badge/GitHub-ucmercedrobotics-181717.svg?style=flat&logo=github)](https://github.com/ucmercedrobotics)
[![website](https://img.shields.io/badge/Website-UCMRobotics-5087B2.svg?style=flat&logo=telegram)](https://robotics.ucmerced.edu/)
[![python](https://img.shields.io/badge/Python-3.10.12-3776AB.svg?style=flat&logo=python&logoColor=white)](https://www.python.org)
[![pre-commits](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
<!-- TODO: work to enable mypy -->
<!-- [![Checked with mypy](http://www.mypy-lang.org/static/mypy_badge.svg)](http://mypy-lang.org/) -->
<!-- TODO: work to enable pydocstyle -->
<!-- [![pydocstyle](https://img.shields.io/badge/pydocstyle-enabled-AD4CD3)](http://www.pydocstyle.org/en/stable/) -->

<!-- [![arXiv](https://img.shields.io/badge/arXiv-2409.04653-b31b1b.svg)](https://arxiv.org/abs/2409.04653) -->

## Overview

The Amiga-ROS2 bridge, currently supported for ROS2 Humble, interfaces with the
[Amiga gRPC services](https://github.com/farm-ng/farm-ng-amiga) to:

- Stream data from the Amiga brain services
- Control the Amiga using the available vehicle control APIs.

Using this bridge with the amiga requires an Amiga OS `v2.3.x`.

> [!Note]
> * For ROS Noetic bridge on Amiga brains running Amiga OS `v2.3.x`, please refer to
> [github.com/farm-ng/amiga-ros-bridge](https://github.com/farm-ng/amiga-ros-bridge-v1).
>
> * For ROS Noetic bridge on Amiga brains running Amiga OS `v1.x`, please refer to
> [github.com/farm-ng/amiga-ros-bridge-v1](https://github.com/farm-ng/amiga-ros-bridge-v1).

To setup the environment, you'll be working in Docker on your local machine. It will communicate to the Amiga via Tailscale using gRPC, just like any of our [ADK Brain examples](https://amiga.farm-ng.com/docs/examples/examples-index#brain-adk-examples).

In theory, this would also work on the Amiga with minimal changes, but we haven't tested it yet.

Even though this repo is a collaboration between [farm-ng](https://www.github.com/orgs/farm-ng) and [Robotics@UC Merced](https://github.com/ucmercedrobotics), this is **NOT** an official release from farm-ng and, therefore, we can't guarantee support or long term compatibility. That said, we will do everything possible to help you use it. For suggestions, questions, or concerns, raise an issue [here](https://github.com/ucmercedrobotics/amiga-ros2-bridge/issues/new).

Collaboration and PRs are welcome and will be evaluated by UC Merced and Farm-ng teams on regular basis.

## How to set up the node (needed only one time):

1. Clone this repo and open the directory:
```bash
git clone https://github.com/ucmercedrobotics/amiga-ros2-bridge.git
cd amiga-ros2-bridge
```

[Optional, but highly recommended] Create a new virtual environment
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

From now on, we will be using Makefile in this repo to make chain commands, feel free to modify and/or use your own commands.

2. [Optional, for application requiring GUIs] Make your local Docker network to connect your VNC client, local machine, and Amiga together. You'll use this later when remote controlling the Amiga.
```bash
make network
```

3. After, build your ROS2 container:
```bash
make build-image
```

<!-- TODO: options for images based on arm platforms -->

4. [Optional, for application requiring GUIs] Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`. We use `localhost` here assuming the container is running locally.
```bash
make vnc
```
## Execution (every time you start the node):

5. Configure and spin up your docker container:

This step assumes you have installed Tailscale and properly [configured your access](https://amiga.farm-ng.com/docs/ssh/) to the Brain with farm-ng's [support team](mailto:support@farm-ng.com).

Check which [services/streams](https://amiga.farm-ng.com/docs/concepts/system_overview/) you want to see and/or control at `amiga-ros2-bridge/include/service_config.json`. Make sure you update your Brains addresses on each `host`:
```bash
{
    "configs": [
        {
            "name": "canbus",
            "port": 6001,
            "host": "000.000.000.000"
        },
        ...
```

Finally, standup the container and connect to it:

```bash
make bash
```

You should now be inside the container, and the terminal will show something like:
```bash
root@docker-desktop:/amiga_ros2_bridge#
```

> [!Note]
> Feel free to modify Makefile to change your container's startup command based
> on what you need adding port forwarding, displays, different volumes, etc.

6. Expose your Amiga gRPC servers as ROS2 topics:

```bash
make amiga-streams
```

You should see confirmation of the topics exposed:
```bash
...
[amiga_streams-1] [INFO] [1743180979.278455758] [amiga_streams]: Subscribing to farm-ng topic: /oak0/left and publishing on ROS topic: /oak0/left
...
```

7. Do Stuff:

You should now be able to interact with the services from your ROS2 container. An example is to allow twist commands to drive the Amiga:

Open a new terminal, connect to the container with a shell:

```bash
make shell
```

Now run our example twist converter

```bash
make twist
```

You should see two more topics (published by twist_control node):
```bash
ros2 topic list
/amiga/cmd_vel
/amiga/vel
```

Now you should see your commands getting to the robot. You can confirm this on the AUTO page from the dashboard:

![dashboard in auto page](https://github.com/farm-ng/amiga-dev-kit/assets/133177230/9a8dcddf-cb5d-4e3c-95e0-0224f521ae6d)

> [!WARNING]
> If your Amiga has AUTO CONTROL enabled, the robot will move. Make sure it is safe.

8. Control remotely or autonomously:

```bash
make joy
```

This defaults to a PS4 controller, which should be configured via `bluetoothctl`.

Feel free to explore around the code and add functionality. You will see there are a lot of commented sections that are useful to get a better understanding of this repo, ROS2 and the Amiga.

Have fun!

## Nav2
If you want to build autonomy with ROS2 into your ecosystem, follow these steps.

1. Initialize the Nav2 submodule in this repo with `git submodule update`. This will clone the Nav2 packages for the Amiga.

2. Follow the instructions in the README in [`amiga-ros2-nav`](https://github.com/ucmercedrobotics/amiga-ros2-nav).

3. Build and Start the docker container with `make build-prod` and `make bash`.

4. From within the container, install dependencies
```bash
rosdep update; rosdep install --from-paths . --ignore-src -r -y
```

5. Run `colcon build`

### Foxglove [Optional]

1. Inside the docker container, install the ros2 foxglove node with 
```bash
apt install ros-humble-foxglove-bridge
```

2. Run the foxglove node (Farm-ng uses port `8765` for their foxglove bridge, so use `8766`)
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="8766"
```

3. Follow instructions [here](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2) to open foxglove

### Autonomy

1. Bringup the relevant nodes for robot localization (you can `make shell` in new windows as each command will block the shell)
```bash
make foxglove # optional
make amiga-streams
make twist
make description
make oakd
make localization
```

This will spawn the robot localization in preparation for Nav2 control.
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

Even though this repo is a collaboration between [farm-ng](https://www.github.com/orgs/farm-ng) and [Robotics@UC Merced](https://github.com/ucmercedrobotics), this is **NOT** an official release from farm-ng and, therefore, we can't guarantee support or long term compatibility. That said, we will do everything possible to help you use it. For suggestions, questions, or concerns, raise an issue [here](https://github.com/ucmercedrobotics/amiga-ros2-bridge/issues/new).

Collaboration and PRs are welcome and will be evaluated by UC Merced and Farm-ng teams on regular basis.

## How to set up the node (needed only one time):

## Installation

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

2. Clone submodules and other repos
```bash
git submodule update
cd amiga-ros2-nav
vcs import < nav.repos
cd ..
```
Note, this bridge works even if you're not using Nav2! We've added support for Nav2 assuming you are using a UBlox module.
We use a UBlox RTK Zed-F9P module to get RTK corrections from a base station instead of using NTRIP over IP as is provided from FarmNG on the Amiga.

3. After, build your ROS2 container. Note you can run either on your own machine becausce of gRPC or directly on the Amiga:

To build on your own machine,
```bash
make build-dev
```

To build on the Amiga,
```bash
make build-prod
```

4. [Optional, for application requiring GUIs] Next, standup the noVNC container to forward X11 to your web browser.

To run the container,
```bash
make vnc
```
Be sure to run this on the same machine you're running your ROS2 bridge on.

You can view the noVNC page at `http://localhost:8080/vnc.html` if ran locally, or `http://{IP}:8080/vnc.html` if the image is remote.

## Execution (every time you start the node):

5. Configure and spin up your Docker container:

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

6. Finally, standup the container and connect to it:

```bash
make bash
```

You should now be inside the container, and the terminal will show something like:
```bash
root@docker-desktop:/amiga_ros2_bridge#
```

Once inside, build ROS2 packages,
```bash
colcon build
source install/setup.bash
```

> [!Note]
> Feel free to modify Makefile to change your container's startup command based
> on what you need adding port forwarding, displays, different volumes, etc.

7. Expose your Amiga gRPC servers as ROS2 topics:

```bash
make amiga-streams
```

You should see confirmation of the topics exposed:
```bash
...
[amiga_streams-1] [INFO] [1743180979.278455758] [amiga_streams]: Subscribing to farm-ng topic: /oak0/left and publishing on ROS topic: /oak0/left
...
```

## Examples

You should now be able to interact with the services from your ROS2 container. An example is to allow twist commands to drive the Amiga:

1. Open a new terminal, connect to the container with a shell:

```bash
make shell
```
> Note, please use `make shell` after you've used `make bash` to create the container instance.
> This command will attach you to the current environment instead of creating a completely new one.
> See `Makefile` for more details.

2. Now run the twist converter:

```bash
make twist
```

This will listen on topic `/cmd_vel` from standard ROS2 velocity commands and convert them to Amiga specific CANbus velocity commands.

Prior to running anything using ROS2, make sure you activate auto control as seen in the image below.

![dashboard in auto page](https://github.com/farm-ng/amiga-dev-kit/assets/133177230/9a8dcddf-cb5d-4e3c-95e0-0224f521ae6d)

> ⚠️ Make sure it is safe to move the robot without physically controlling it! ⚠️

3. Control remotely:

```bash
make joy
```

This defaults to a controller mapped to `/dev/input/js0`.
This default file descriptor is usually a PS4 controller.
You can connect your own contoller via `bluetoothctl`.

## Current Support

### Kinova Kortex
If you're using the Kinova as an attachment to your Amiga, use `vcs import < kinova.repos`.
From there, make sure you build using the MoveIt enabled image we provided (you must have access since the registry is private).
To do so, use `make build-image USE_KINOVA=1`.

See https://github.com/ucmercedrobotics/ros2-kortex-control.

### Luxonis Oak-D
Using the onboard Oak-D cameras or your own, you can configure them to stream using ROS2 drivers.
The below command will execute the DepthAI provided ROS2 drivers (including 9-axis IMU support).
```bash
make oakd
```

To edit the config to match your cameras, edit `amiga_ros2_oakd/config/amiga_cameras.yaml` prior to running the ROS2 drivers.

### BT.CPP
This repo supports mission command and control via [BT.CPP](https://www.behaviortree.dev/) XML.
We have an example behavior tree in `amiga_ros2_behavior_tree`, which can be adapted to fit your own robot configuration.

See [amiga_ros_behavior_tree](amiga_ros2_behavior_tree/README.md) for examples.

### Foxglove

1. Inside the docker container, run the foxglove node (Farm-ng uses port `8765` for their foxglove bridge, so we use `8766` by default),
```bash
make foxglove
```

2. Follow instructions [here](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2) to open foxglove

### Nav2
While we use our own UBlox RTK module in order to support Nav2 related ROS2 nodes, this can be adapted to model any configuration.
See [our Nav2 repo](https://github.com/ucmercedrobotics/amiga-ros2-nav/tree/main) for more details.

### LLM Planning
From any computer (except the Amiga since it's behind Tailscale) on the same network as the Amiga, run the proxy that will connect our webapp to the Amiga.
`git clone https://github.com/vtomnet/mp-proxy`, make a venv, install `requirements.txt`, `python proxy.py --to-tcp <amiga_ip>:<bt_runner_port>`.
From https://mp.vtom.net/ plan out your mission.

Note, you can configure what port the Amiga talks from through `bt_runner` parameter `mission_port`, which is defaulted to `12346`.

## Stopping Services
Many services, such as the Oak-D cameras, require the exclusive use of the hardware.
You may also see it fit to use this
To disable default Amiga drivers, use the following command:
```bash
bash ./scripts/stop_amiga_services.sh
```

## Nav2 Autonomous Pipeline

We provide two ways of bringing up the autonomy stack:

1. shell script that spawns all nodes inside tmux for debugging:
```bash
bash ./scripts/bringup_amiga_tmux.sh
```

2. Master launch file for deployment:

Start the container,
```bash
make bash
```

Once inside, bring up the nodes,
```bash
make bringup
```

This will spawn the Amiga-ROS2 bridge, Oakd cameras, URDF descriptions, localization stack, and nav2 stack.

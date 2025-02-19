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

To setup the environment, you'll be working in Docker on your local machine. In theory, this should also work on the Amiga, but I haven't tested it yet.
It will communicate to the Amiga via Tailscale using gRPC.
## How to Start
Make sure you initialize the repo with the repo pre-commits:
```bash
make repo-init
```

<!-- To start, make your local Docker network to connect your VNC client, local machine, and Amiga together. You'll use this later when remote controlling the Amiga.
```bash
make network
``` -->

After, build your ROS2 container:
```bash
make build-image
```

<!-- Next, standup the VNC container to forward X11 to your web browser. You can see this at `localhost:8080`.
```bash
make vnc
``` -->
## Execution
Finally, standup your ROS1 container:
```bash
make bash
```

This last step makes the assumption you have installed Tailscale and configured it with FarmNG.
Start by exposing Amiga gRPC servers as ROS2 topics:
```bash
make amiga-streams
```

Topics published by amiga_streams node:
```bash
$ ros2 topic list
/canbus/twist
/filter/state
/gps/pvt
/oak0/imu
/oak0/left
```

Second, publish topics that will handle ROS2 compliant message types to be converted and serialized into protobufs within FarmNG gRPC stack.
To do so, run the following command in a new window:
```bash
make twist
```

If unfamiliar with Docker, to expose a new shell of your current container run the following:
```bash
docker exec -it <container_id> bash
```
You can obtain `<container_id>` from `docker ps`.

Topics published by twist_control node:
```bash
$ ros2 topic list
/amiga/cmd_vel
/amiga/vel
```

# amiga-ros2-nav
This describes the current state of Amiga localization using ROS2/Nav2. This pipeline is used with an RTK module similar to what is in the Brain. However, to configure it properly, we use our own hardware and plug it into the available USB in the back of the Brain. This can work without RTK, but right now the configuration requires it. Future work will allow parameters to remove it as a requirement and only use Amiga provided hardware.

## Installation

1. Install vcs following the instructions [here](https://github.com/dirk-thomas/vcstool?tab=readme-ov-file#how-to-install-vcstool)

2. Run `vcs import < nav.repos` to install the required 3rd party packages

3. The remaining steps require the rest of the Amiga ROS2 packages, so see steps in [`amiga-ros2-bridge`](https://github.com/ucmercedrobotics/amiga-ros2-bridge).

## VectorNav
If using the VectorNav IMU, also make sure to import the ROS2 drivers using `vcs import < vectornav.repos`.

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full

FROM ${BASE_IMAGE} AS base

ARG WORKSPACE_ROOT="/amiga-ros2-bridge"
ARG PACKAGE_NAME="amiga_ros2_bridge"

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-full python3-pip vim net-tools netcat-traditional build-essential cmake \
    ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-behaviortree-cpp ros-${ROS_DISTRO}-generate-parameter-library

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN python3 -m venv .venv && \
    . .venv/bin/activate && \
    pip install -r /requirements.txt

WORKDIR ${WORKSPACE_ROOT}

# Copy everything into the workspace (except what's in .dockerignore)
COPY . ${WORKSPACE_ROOT}

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

# install BT CPP ROS2 wrapper
ARG BTCPP_ROS2_WORKSPACE="/btcpp_ros2_ws"
RUN mkdir -p ${BTCPP_ROS2_WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git ${BTCPP_ROS2_WORKSPACE} && \
    cd ${BTCPP_ROS2_WORKSPACE} && \
    colcon build --symlink-install

RUN cd ${WORKSPACE_ROOT} && \
    rosdep install --from-paths . --ignore-src -r -y

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${BTCPP_ROS2_WORKSPACE}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc
RUN echo "source /.venv/bin/activate" >> /root/.bashrc
RUN echo "export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH" >> /root/.bashrc

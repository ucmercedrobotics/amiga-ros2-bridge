ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full-2025-12-07

FROM ${BASE_IMAGE} AS base

ARG WORKSPACE_ROOT="/amiga-ros2-bridge"
ARG PACKAGE_NAME="amiga_ros2_bridge"
ARG MACHINE_NAME="agx"

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-full python3-pip vim net-tools netcat-traditional build-essential cmake \
    ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-behaviortree-cpp ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-tf-transformations \
    tmux \
    rhash librhash-dev

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN . /.venv/bin/activate && \
    pip install -r /requirements.txt

WORKDIR ${WORKSPACE_ROOT}

COPY manifests/ /manifests/
RUN rosdep install --from-paths /manifests --ignore-src -r -y

RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

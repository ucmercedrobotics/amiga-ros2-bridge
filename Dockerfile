ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full-2025-12-07

FROM ${BASE_IMAGE} AS base

ARG WORKSPACE_ROOT="/amiga-ros2-bridge"
ARG PACKAGE_NAME="amiga_ros2_bridge"
ARG MACHINE_NAME="agx"

ARG INSTALL_ZED_SDK=1
ARG ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.2/l4t36.4/jetsons"

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-full python3-pip vim net-tools netcat-traditional build-essential cmake \
    ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-depthai-ros \
    ros-${ROS_DISTRO}-behaviortree-cpp ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-zed-description \
    tmux \
    rhash librhash-dev

# ZED installer prerequisites (arm64 only)
RUN if [ "$(dpkg --print-architecture)" = "arm64" ]; then \
    set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates curl sudo zstd bc file; \
    rm -rf /var/lib/apt/lists/*; \
    fi

# Install ZED SDK
COPY scripts/install_zed_sdk_if_missing.sh /usr/local/bin/install_zed_sdk_if_missing.sh
RUN chmod +x /usr/local/bin/install_zed_sdk_if_missing.sh
ENV INSTALL_ZED_SDK="${INSTALL_ZED_SDK}" \
    ZED_SDK_URL="${ZED_SDK_URL}"
RUN if [ "${INSTALL_ZED_SDK}" = "1" ] && [ "$(dpkg --print-architecture)" = "arm64" ]; then \
    /usr/local/bin/install_zed_sdk_if_missing.sh; \
    fi

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN . /.venv/bin/activate && \
    pip install -r /requirements.txt

WORKDIR ${WORKSPACE_ROOT}

COPY manifests/ /manifests/
RUN apt-get update && \
    rosdep install --from-paths /manifests --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

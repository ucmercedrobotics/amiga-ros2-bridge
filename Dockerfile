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
    ros-${ROS_DISTRO}-moveit \
    rhash librhash-dev

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN python3 -m venv .venv && \
    . .venv/bin/activate && \
    pip install -r /requirements.txt

# install BT CPP ROS2 wrapper
ARG INSTALL_BTCPP_ROS2=true
ARG BTCPP_ROS2_WORKSPACE="/btcpp_ros2_ws"
RUN if [ "$INSTALL_BTCPP_ROS2" = "true" ]; then \
    mkdir -p ${BTCPP_ROS2_WORKSPACE} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git ${BTCPP_ROS2_WORKSPACE} && \
    cd ${BTCPP_ROS2_WORKSPACE} && \
    colcon build --symlink-install; \
    fi

WORKDIR ${WORKSPACE_ROOT}

COPY manifests/ /manifests/
RUN rosdep install --from-paths /manifests --ignore-src -r -y

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source ${BTCPP_ROS2_WORKSPACE}/install/setup.bash" >> /root/.bashrc
RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc
RUN echo "source /.venv/bin/activate" >> /root/.bashrc
RUN echo "export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH" >> /root/.bashrc

FROM base AS jetson
# This is terrible to do, but they offer me no choice...
# only works on AGX because it's built for libcudnn 9 (cuda 12.6)
RUN curl -O "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl" && \
    curl -O "https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl" && \
    . /.venv/bin/activate && \
    pip install torch-2.8.0-cp310-cp310-linux_aarch64.whl \
    torchvision-0.23.0-cp310-cp310-linux_aarch64.whl

ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib/:/usr/lib/aarch64-linux-gnu-host/openblas-pthread:/usr/lib/aarch64-linux-gnu-host/
ENV PATH=/usr/local/cuda/bin:${PATH}

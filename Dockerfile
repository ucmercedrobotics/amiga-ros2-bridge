FROM ghcr.io/sloretz/ros:humble-desktop-full AS base

ARG WORKSPACE_ROOT="/amiga-ros2-bridge"
ARG PACKAGE_NAME="amiga_ros2_bridge"

# TODO: downgrade this image in production

# copy over all python files from builder stage
# TODO: figure out why Farm-NG libs don't go to /usr/local
# COPY --from=builder /usr/local /usr/local

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat build-essential cmake \
    ros-humble-foxglove-bridge ros-humble-depthai-ros

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

WORKDIR ${WORKSPACE_ROOT}

# Copy everything into the workspace (except what's in .dockerignore)
COPY . ${WORKSPACE_ROOT}

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

# Rosdep get packages
RUN rosdep update && rosdep install --from-paths . --ignore-src -r -y

# build artifacts to run by default
# RUN /bin/bash -c "cd ${WORKSPACE_ROOT} && \
#     colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc

FROM osrf/ros:noetic-desktop-full AS base

RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat

RUN mkdir -p /root/catkin_ws/src
RUN git clone --recursive https://github.com/farm-ng/amiga-ros-bridge.git /root/catkin_ws/src/amiga-ros-bridge

ENV DISPLAY=novnc:0.0

RUN /bin/bash -c "cd /root/catkin_ws && \
                    source /opt/ros/noetic/setup.bash && catkin_make && \
                    ./src/amiga-ros-bridge/setup_venv.sh"

WORKDIR /root/catkin_ws

RUN echo "export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"" >> /root/.bashrc
RUN echo "source \$DIR/src/amiga-ros-bridge/source_venv.sh >> devel/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
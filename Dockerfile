FROM osrf/ros:humble-desktop-full AS base 
# TODO: downgrade this image in production

ARG UID
ARG GID

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat

WORKDIR /amiga_ros2_bridge

COPY amiga_ros2_bridge /amiga_ros2_bridge/amiga_ros2_bridge
COPY requirements.txt /amiga_ros2_bridge/requirements.txt

RUN pip install -r requirements.txt

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

# # # build artifacts to run by default
RUN /bin/bash -c "cd /amiga_ros2_bridge && \
                    colcon build"

RUN adduser -u ${UID} --disabled-password --gecos "" appuser && chown -R appuser /amiga_ros2_bridge
USER appuser

RUN echo "source /opt/ros/humble/setup.bash" >> /home/appuser/.bashrc
RUN echo "source /amiga_ros2_bridge/install/setup.bash" >> /home/appuser/.bashrc
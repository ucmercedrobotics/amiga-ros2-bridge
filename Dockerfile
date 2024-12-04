FROM osrf/ros:humble-desktop-full AS base
# TODO: downgrade this image in production

# copy over all python files from builder stage
# TODO: figure out why Farm-NG libs don't go to /usr/local
# COPY --from=builder /usr/local /usr/local

ARG UID
ARG GID

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat

WORKDIR /amiga_ros2_bridge

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /amiga_ros2_bridge/requirements.txt
RUN pip install -r /amiga_ros2_bridge/requirements.txt

COPY amiga_ros2_bridge /amiga_ros2_bridge/amiga_ros2_bridge

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

# # # build artifacts to run by default
RUN /bin/bash -c "cd /amiga_ros2_bridge && \
                    colcon build"

RUN adduser -u ${UID} --disabled-password --gecos "" appuser && chown -R appuser /amiga_ros2_bridge
USER appuser

RUN echo "source /opt/ros/humble/setup.bash" >> /home/appuser/.bashrc
RUN echo "source /amiga_ros2_bridge/install/setup.bash" >> /home/appuser/.bashrc

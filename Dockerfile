FROM osrf/ros:humble-desktop-full AS base
# TODO: downgrade this image in production

# copy over all python files from builder stage
# TODO: figure out why Farm-NG libs don't go to /usr/local
# COPY --from=builder /usr/local /usr/local

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-pip vim net-tools netcat build-essential cmake

# TODO: remove once you figure out why farm-ng isn't in /usr/local
COPY requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

WORKDIR /amiga_ros2_bridge

COPY amiga_ros2_bridge /amiga_ros2_bridge/amiga_ros2_bridge

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=novnc:0.0

# build artifacts to run by default
RUN /bin/bash -c "cd /amiga_ros2_bridge && \
                    colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /amiga_ros2_bridge/install/setup.bash" >> /root/.bashrc

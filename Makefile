IMAGE:=ghcr.io/ucmercedrobotics/amiga-ros2-bridge
WORKSPACE:=amiga-ros2-bridge
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

multiarch-builder:
	docker buildx create --name multiarch --driver docker-container --use

push:
	docker buildx build --platform linux/arm64/v8,linux/amd64 -t ${IMAGE} --target base . --push

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}) && \
	docker exec -it $${CONTAINER_PS} bash

network:
	docker network create ros

build-dev:
	docker build . -t ${IMAGE} --target base

build-prod:
	docker buildx build --platform linux/arm64/v8 . -t ${IMAGE} --target base

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

bash:
	docker run -it --rm \
	--net=host \
	--privileged \
	--env="DISPLAY=:2" \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	-v /dev/input:/dev/input \
	-v /dev/ttyACM1:/dev/ttyACM1 \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

amiga-streams:
	ros2 launch amiga_ros2_bridge amiga_streams.launch.py

twist:
	ros2 launch amiga_ros2_bridge twist_control.launch.py
	
joy:
	ros2 launch amiga_ros2_teleop joy.launch.py

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766

oakd:
	ros2 launch amiga_ros2_oakd amiga_cameras.launch.py
	
description:
	ros2 launch amiga_ros2_description urdf.launch.py

localization:
	ros2 launch amiga_localization bringup.launch.py
IMAGE:=ghcr.io/ucmercedrobotics/amiga-ros2-bridge
WORKSPACE:=amiga-ros2-bridge
PACKAGE:=amiga_ros2_bridge
NOVNC:=theasp/novnc:latest

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

multiarch-builder:
	docker buildx create --name multiarch --driver docker-container --use

push:
	docker buildx build --platform linux/arm64/v8,linux/amd64 -t ${IMAGE} --target base . --push

network:
	docker network create ros

build-dev:
	docker build . -t ${IMAGE} --target base

build-prod:
	docker buildx build --platform linux/arm64/v8 . -t ${IMAGE} --target base

vnc:
	docker run -d --rm --net=ros \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	${NOVNC}

bash:
	docker run -it --rm \
	--net=host \
	-v ./${PACKAGE}:/${WORKSPACE}/${PACKAGE}:Z \
	-v ./Makefile:/${WORKSPACE}/Makefile:Z \
	-v ~/.ssh:/root/.ssh:ro \
	${IMAGE} bash

clean:
	rm -rf build/ install/ log/

amiga-streams:
	ros2 launch ${PACKAGE} amiga_streams.launch.py

twist:
	ros2 launch ${PACKAGE} twist_control.launch.py

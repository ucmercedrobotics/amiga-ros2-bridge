network:
	docker network create ros

build-image:
	docker build --build-arg UID=$(shell id -u) --build-arg GID=$(shell id -g) . -t humble --target base

vnc:
	docker run -d --rm --net=ros \
	--env="DISPLAY_WIDTH=3000" \
	--env="DISPLAY_HEIGHT=1800" \
	--env="RUN_XTERM=no" \
	--name=novnc -p=8080:8080 \
	theasp/novnc:latest

bash:
	docker run -it --rm \
	--net=ros \
	--user $(shell id -u):$(shell id -g) \
	-v ./amiga_ros2_bridge:/amiga_ros2_bridge/amiga_ros2_bridge:Z \
	-v ./Makefile:/amiga_ros2_bridge/Makefile:Z \
	-v ~/.ssh:/home/appuser/.ssh:ro \
	humble bash

clean: 
	rm -rf build/ install/ log/

amiga-streams:
	ros2 launch amiga_ros2_bridge amiga_streams.launch.py

twist:
	ros2 launch amiga_ros2_bridge twist_control.launch.py
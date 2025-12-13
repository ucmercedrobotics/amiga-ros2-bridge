IMAGE:=ghcr.io/ucmercedrobotics/amiga-ros2-bridge
WORKSPACE:=amiga-ros2-bridge
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
MACHINE_NAME?=agx
ARCH := $(shell uname -m)
PLATFORM := linux/amd64
TARGET:=base
ARCH_TAG:=x86_64
CUDA_MOUNT:=
ifneq (,$(filter $(ARCH),arm64 aarch64))
	PLATFORM := linux/arm64/v8
	ARCH_TAG:=arm64
	TARGET:=jetson
	CUDA_MOUNT:= --runtime=nvidia \
			 -v /usr/local/cuda:/usr/local/cuda:ro \
		     -v /usr/lib/aarch64-linux-gnu/:/usr/lib/aarch64-linux-gnu-host/:ro \
			 --device /dev/nvhost-gpu \
			 --device /dev/nvmap 
endif

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${ARCH_TAG}) && \
	docker exec -it $${CONTAINER_PS} bash

build-image:
	docker build --platform ${PLATFORM} . -t ${IMAGE}:${ARCH_TAG} --target ${TARGET}

vnc:
	docker run -d --rm --net=host \
	--name=novnc \
	${NOVNC}

udev:
	cp udev/99-ucm.rules /etc/udev/rules.d && \
	udevadm control --reload-rules && \
	udevadm trigger

bash: udev
	docker run -it --rm \
	--net=host \
	--privileged \
	${CUDA_MOUNT} \
	--env="DISPLAY=:2" \
	-v .:/${WORKSPACE}:Z \
	-v ~/.ssh:/root/.ssh:ro \
	-v /dev/:/dev/ \
	-e FASTDDS_DEFAULT_PROFILE_FILE=file:///${WORKSPACE}/dds/${MACHINE_NAME}.xml \
	${IMAGE}:${ARCH_TAG} bash

deps:
	rosdep install --from-paths . --ignore-src -r -y

clean:
	rm -rf build/ install/ log/

bringup:
	ros2 launch amiga_bringup brain_bringup.launch.py

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

amiga:
	./scripts/bringup_amiga_tmux.sh

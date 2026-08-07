IMAGE:=ghcr.io/ucmercedrobotics/amiga-ros2-bridge
WORKSPACE:=amiga-ros2-bridge
NOVNC:=ghcr.io/ucmercedrobotics/docker-novnc
BASE_IMAGE=ghcr.io/ucmercedrobotics/ros2-kortex-control:${IMAGE_TAG}
MACHINE_NAME?=agx

PORT:=12346
LORA_PORT?=/dev/ttyUSB0
LORA_ROBOTS?=robot1,robot2,robot3
# This robot's fleet-unique coordination ID, 1..255. Two robots sharing one
# silently dedup each other's traffic away, so it has no safe default and is
# meant to be overridden per robot.
NODE_ID?=1
PAYLOAD:=true
ARCH := $(shell uname -m)
PLATFORM := linux/amd64
ARCH_TAG:=amd64
CUDA_MOUNT:=

# Select the build platform/tag purely from the CPU architecture.
ifneq (,$(filter $(ARCH),arm64 aarch64))
	PLATFORM := linux/arm64/v8
	ARCH_TAG:=arm64
endif

OS := $(shell uname -s)
IS_JETSON := $(shell test -f /etc/nv_tegra_release && echo 1)
HAS_NVIDIA_GPU := $(shell command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 && echo 1)
NVIDIA_INJECTS := $(shell nvidia-container-cli info >/dev/null 2>&1 && echo 1)
IN_CONTAINER := $(shell test -f /.dockerenv && echo 1)

ifeq ($(IN_CONTAINER),1)
else ifeq ($(OS),Darwin)
	# No GPU passthrough on Docker Desktop; nothing to add.
else ifeq ($(IS_JETSON),1)
ifeq ($(NVIDIA_INJECTS),1)
	IMAGE_VARIANT := thor
	CUDA_MOUNT := --runtime=nvidia --gpus all
else
	IMAGE_VARIANT := jetson
	CUDA_HOST_PATH := $(shell readlink -f /usr/local/cuda 2>/dev/null)
ifeq (,$(wildcard $(CUDA_HOST_PATH)/targets/*/lib/libcudart.so))
	CUDA_HOST_PATH := $(shell ls -d /usr/local/cuda-*.* 2>/dev/null | sort -V | tail -1)
endif
ifeq (,$(CUDA_HOST_PATH))
	CUDA_MOUNT := --runtime=nvidia
$(warning No CUDA toolkit found under /usr/local -- CUDA nodes will not run.)
else
	CUDA_MOUNT := --runtime=nvidia \
		-v $(CUDA_HOST_PATH):/usr/local/cuda:ro \
		-v /usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu-host:ro \
		-e LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib/:/usr/lib/aarch64-linux-gnu-host/openblas-pthread:/usr/lib/aarch64-linux-gnu-host/:/usr/lib/aarch64-linux-gnu-host/nvidia
endif
endif
else ifeq ($(HAS_NVIDIA_GPU),1)
	CUDA_MOUNT := --gpus all
endif

# Thor and JetPack 6 boards are both arm64, but an image built for one cannot
# run on the other (CUDA 13/sm_110 vs CUDA 12.6), so the GPU variant is part of
# the tag instead of the architecture alone. This has to match the tag the
# base image was published under.
IMAGE_TAG := $(ARCH_TAG)
ifneq (,$(IMAGE_VARIANT))
IMAGE_TAG := $(ARCH_TAG)-$(IMAGE_VARIANT)
endif

repo-init:
	python3 -m pip install pre-commit && \
	pre-commit install

shell:
	CONTAINER_PS=$(shell docker ps -aq --filter ancestor=${IMAGE}:${IMAGE_TAG}) && \
	docker exec -it $${CONTAINER_PS} bash

manifest:
	mkdir -p manifests
	rsync -av --include '*/' --include 'package.xml' --exclude '*' amiga* manifests/

build-image: manifest
	docker build --platform ${PLATFORM} . -t ${IMAGE}:${IMAGE_TAG} --target base --build-arg BASE_IMAGE=${BASE_IMAGE}

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
	-v /${WORKSPACE}/manifests \
	-v ~/.ssh:/root/.ssh:ro \
	-v /dev/:/dev/ \
	-e FASTDDS_DEFAULT_PROFILE_FILE=file:///${WORKSPACE}/dds/${MACHINE_NAME}.xml \
	${IMAGE}:${IMAGE_TAG} bash

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

lora:
	ros2 launch amiga_ros2_comms lora_bridge.launch.py serial_port:=${LORA_PORT}

lora-sim:
	ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=${LORA_ROBOTS}

coordinator:
	ros2 launch amiga_ros2_coordinator coordinator.launch.py \
		node_id:=${NODE_ID}

# Exactly what .github/workflows/ci.yml runs, in the same image, so a red build
# can be reproduced without pushing. Not the dev image: that one is 14-25 GB
# and no GitHub-hosted runner has the disk for it, so CI uses stock ros-base
# (~1 GB) and rosdep-installs only what scripts/ci/packages.txt declares.
# Output goes to build/ci and install/ci to leave a full workspace build alone.
# The named volume is the local stand-in for the workflow's cache: it keeps the
# BehaviorTree.ROS2 underlay across runs so only the first one pays for it.
CI_IMAGE:=ros:humble-ros-base
CI_RUN=docker run --rm -v $(CURDIR):/${WORKSPACE}:Z -w /${WORKSPACE} \
	-v amiga-ci-underlay:/opt/underlay_ws \
	-e BUILD_BASE=build/ci -e INSTALL_BASE=install/ci

ci-test:
	${CI_RUN} ${CI_IMAGE} ./scripts/ci/run_tests.sh

# The ament copyright/flake8/pep257 checks CI reports but does not gate on.
ci-lint:
	${CI_RUN} -e LINTERS=only ${CI_IMAGE} ./scripts/ci/run_tests.sh

oakd:
	ros2 launch amiga_ros2_oakd amiga_cameras.launch.py

description:
	ros2 launch amiga_ros2_description urdf.launch.py

localization:
	ros2 launch amiga_localization bringup.launch.py

mission-interface:
	ros2 run amiga_ros2_behavior_tree bt_runner --ros-args -p mission_port:=${PORT} -p mission_payload_length_included:=${PAYLOAD}

amiga:
	./scripts/bringup_amiga_tmux.sh

ROBOT_COUNT ?= 5
# Spreading factor of the simulated radio, 6..12. Time on air doubles per step,
# so this is the dial on how much coordination traffic the fleet can sustain.
LORA_SF ?= 7
# Set COORDINATION=false to bring up the robots without the radio layer.
COORDINATION ?= true
# AGENTS=true adds a full LLM stack per robot (world state, arbiter, mission
# planner, triage, note). Needs AGENT_MODEL/AGENT_API_BASE set -- see
# amiga_ros2_agents/README.md. It also switches the coordinators over to asking
# those agents instead of their local stubs.
AGENTS ?= false
# LTL=false drops the arbiter's formal gate. Plans are still checked for whether
# they will RUN -- well-formed XML, the XSD, and the ontology's required
# preconditions -- but not for whether they still satisfy the mission: no
# formula, no SPIN, no viability budget, no edit-size or rate limit. For
# bringing the coordination loop up end to end, where the question is whether a
# task crosses robots and comes back as executable XML. Every accept is then
# reported unverified, in the service response and in the arbiter's status.
LTL ?= true
sim:
	ros2 launch amiga_ros2_gazebo sim_bringup.launch.py \
		robot_count:=$(ROBOT_COUNT) \
		launch_coordination:=$(COORDINATION) \
		launch_agents:=$(AGENTS) \
		ltl_verification:=$(LTL) \
		lora_spreading_factor:=$(LORA_SF)

# Poke one robot into shedding a task, so the fleet has something to auction.
# Run it against a `make sim ROBOT_COUNT=3` that is already up. SCENARIO_ROBOT
# is a namespace: robot 1 is unnamespaced, so leave it empty to target robot 1.
SCENARIO_ROBOT ?= amiga2
SCENARIO_TASK ?= 42
SCENARIO_TREE ?= 60
SCENARIO_NOTE ?= north end of row 7 is flooded; approach from the south, expect 4 min extra
fleet-scenario:
	ros2 run amiga_ros2_coordinator escalate \
		--robot "$(SCENARIO_ROBOT)" \
		--task $(SCENARIO_TASK) \
		--tree $(SCENARIO_TREE) \
		--note "$(SCENARIO_NOTE)"

# The other injection point: fail a leaf in one robot's tree and let the real
# loop run -- mission planner edits, arbiter reviews, and only if that gives up
# does triage escalate. Slower and less certain than fleet-scenario, and the
# only one of the two where the decision to shed is earned rather than handed
# over. FAULT_NODE must name a leaf in that robot's own mission XML.
FAULT_ROBOT ?= amiga2
FAULT_NODE ?= Visit_Tree_60
FAULT_REASON ?= navigation failed: no progress toward the tree for 90s
# Robot 1 is unnamespaced, so an empty FAULT_ROBOT must not produce "//bt/...".
FAULT_TOPIC = $(if $(FAULT_ROBOT),/$(FAULT_ROBOT)/bt/status_change,/bt/status_change)
fleet-fault:
	ros2 topic pub --once "$(FAULT_TOPIC)" std_msgs/msg/String \
		"{data: '{\"node\":\"$(FAULT_NODE)\",\"reason\":\"$(FAULT_REASON)\",\"timestamp_ms\":0}'}"

kortex-home:
	ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{ \
	joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], \
	points: [ \
	{ positions: [0.0, -0.785398, -2.0, 0.0, -0.436332, 1.5708], time_from_start: { sec: 10 } }, \
	] \
	}" -1

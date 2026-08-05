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

# How a GPU reaches the container differs per host, so detect it rather than
# assume it from the architecture:
#
#   macOS (Intel/Apple Silicon)   Docker Desktop has no GPU passthrough, so
#                                 these always run CPU-only.
#   Linux, no NVIDIA GPU          CPU-only.
#   Linux + discrete NVIDIA GPU   The Container Toolkit injects the driver on
#                                 its own: --gpus all.
#   Jetson Thor (JetPack 7+)      Ships the open NVRM driver with working NVML,
#                                 so the toolkit injects the driver exactly
#                                 like it does for a discrete GPU. It still
#                                 needs an explicit --runtime=nvidia, because
#                                 the OCI hook behind a bare --gpus all refuses
#                                 to run on Tegra.
#   Jetson AGX/Orin (JetPack 6)   NVML is unsupported on the iGPU, so
#                                 nvidia-container-cli cannot enumerate it and
#                                 the toolkit injects nothing. CUDA and the
#                                 driver libraries get bind-mounted from the
#                                 host instead, so CUDA must be installed on
#                                 the host.
OS := $(shell uname -s)
IS_JETSON := $(shell test -f /etc/nv_tegra_release && echo 1)
HAS_NVIDIA_GPU := $(shell command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 && echo 1)
# Succeeds only where NVML works, which is precisely the condition for the
# toolkit being able to inject the driver stack by itself. This is what
# separates Thor from AGX/Orin: nvidia-smi now works on both, so it cannot
# tell them apart.
NVIDIA_INJECTS := $(shell nvidia-container-cli info >/dev/null 2>&1 && echo 1)

ifeq ($(OS),Darwin)
	# No GPU passthrough on Docker Desktop; nothing to add.
else ifeq ($(IS_JETSON),1)
ifeq ($(NVIDIA_INJECTS),1)
	IMAGE_VARIANT := thor
	CUDA_MOUNT := --runtime=nvidia --gpus all
else
	IMAGE_VARIANT := jetson
	# /usr/local/cuda is meant to be a symlink to the versioned toolkit
	# directory, but a partial/re-flashed install can leave it an empty
	# directory, so resolve it and fall back to the newest cuda-<major>.<minor>
	# that actually contains a runtime library.
	CUDA_HOST_PATH := $(shell readlink -f /usr/local/cuda 2>/dev/null)
ifeq (,$(wildcard $(CUDA_HOST_PATH)/targets/*/lib/libcudart.so))
	CUDA_HOST_PATH := $(shell ls -d /usr/local/cuda-*.* 2>/dev/null | sort -V | tail -1)
endif
ifeq (,$(CUDA_HOST_PATH))
	# Fall through with GPU device access but no CUDA: nodes that only need
	# the driver still work, CUDA nodes fail with a clear missing-library
	# error rather than a confusing empty bind mount.
	CUDA_MOUNT := --runtime=nvidia
$(warning No CUDA toolkit found under /usr/local -- CUDA nodes will not run.)
else
	# libcuda.so.1 -- the versioned SONAME the loader actually resolves --
	# only exists under .../aarch64-linux-gnu/nvidia, which is not on the
	# image's default search path, so LD_LIBRARY_PATH is extended here.
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

# Virtual radios for Gazebo and anywhere else without hardware. Starts one
# simulated LoRa modem per robot plus its bridge; the bridge is unchanged and
# unaware. See amiga_ros2_comms/docs/lora_sim.md.
lora-sim:
	ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=${LORA_ROBOTS}

# Every acceptance test runs without a radio: loopback over a virtual serial
# pair, CRC drop on a corrupted frame, and backpressure under flood.
test-comms:
	python3 -m pytest amiga_ros2_comms/test/ -q

# The contract-net state machine, both roles. No radio and no model either:
# the clock is injected and the two reasoning points are stubbed, so whole
# auctions and backoff windows run in microseconds. See
# amiga_ros2_coordinator/docs/coordinator.md.
test-coordinator:
	python3 -m pytest amiga_ros2_coordinator/test/ -q

# The agent stack: the triage decision parser, the escalation trigger, and the
# mission-XML -> task extraction, which runs against the real files in
# amiga_ros2_behavior_tree/examples/ rather than fixtures.
test-agents:
	python3 -m pytest amiga_ros2_agents/test/ -q

# One real mission with one step failing the way the real node fails, followed
# from the behaviour tree to a task the fleet could be offered. Needs a built
# workspace; takes about a minute. See
# amiga_ros2_behavior_tree/test/scenarios/README.md for what this does and does
# not prove -- in particular, it tests the wiring and says nothing about the
# quality of any interpretation.
test-scenario:
	python3 -m pytest amiga_ros2_agents/test/test_scenario_bt_fault.py -q

# One failure, watched live rather than asserted. FAIL_GOALS is a tree id;
# FAILURE_MODE picks which real failure path to take.
FAIL_GOALS?=[60]
FAILURE_MODE?=nav_failed
scenario:
	ros2 launch amiga_ros2_behavior_tree failure_scenario.launch.py \
		fail_goals:="${FAIL_GOALS}" failure_mode:=${FAILURE_MODE}

# One robot's full coordination stack: the coordinator plus its in-process
# reliability layer. Do not run this alongside `make lora-reliability` -- two
# reliability layers on one radio would ACK each other's inbound traffic.
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

# Full-stack Gazebo simulation (run inside the container).
# Append headless:=true on machines without a display.
sim:
	ros2 launch amiga_ros2_gazebo sim_bringup.launch.py

kortex-home:
	ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{ \
	joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], \
	points: [ \
	{ positions: [0.0, -0.785398, -2.0, 0.0, -0.436332, 1.5708], time_from_start: { sec: 10 } }, \
	] \
	}" -1

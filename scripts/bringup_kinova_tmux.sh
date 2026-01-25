#!/bin/bash

# This script will run the Leaf Grasping Pipeline in a tmux session for easy debugging

IMAGE="ghcr.io/ucmercedrobotics/amiga-ros2-bridge"
SESSION="leaf_pipeline"
WINDOW_INDEX=1

# -- Get project path from script
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_PATH="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# -- Utilities
start_tmux_session() {
    echo "Starting tmux session: $SESSION"
    tmux set -g mouse on
    tmux new-session -d -s "$SESSION"
}

start_container() {
    echo "Running 'make bash' to start container"
    tmux send-keys -t "$SESSION" "cd $PROJECT_PATH; make -C $PROJECT_PATH bash MACHINE_NAME=$MACHINE_NAME" C-m
}

attach_tmux() {
    tmux attach -t "$SESSION"
}

wait_for_container() {
    echo "Waiting for container from image ${IMAGE} to start..."

    ARCH_TAG="x86_64"
    uname -m | grep -Eq '^(arm64|aarch64)$' && ARCH_TAG=arm64

    for i in $(seq 30); do
        CONTAINER_ID=$(docker ps -q --filter ancestor="${IMAGE}:${ARCH_TAG}" | head -n 1)
        if [ -n "$CONTAINER_ID" ]; then
            echo "Container started: $CONTAINER_ID"
            export CONTAINER_ID
            return 0
        fi
        sleep 1
    done
    echo "Error: Timed out waiting for container."
    exit 1
}

run() {
    local CMD="$1"
    local WINDOW_NAME="$2"
    
    # Use provided window name or generate one
    if [ -z "$WINDOW_NAME" ]; then
        WINDOW_NAME="exec-${WINDOW_INDEX}"
    fi
    
    echo "Creating new tmux window '$WINDOW_NAME'..."

    tmux new-window -t "$SESSION" -n "$WINDOW_NAME" "docker exec -it $CONTAINER_ID bash"
    tmux send-keys -t "$SESSION:$WINDOW_NAME" "$CMD" C-m

    WINDOW_INDEX=$((WINDOW_INDEX + 1))
}

setup_parameters() {
    # Always run on AGX
    MACHINE_NAME="agx"
    export MACHINE_NAME

    echo "Starting Leaf Grasping Pipeline..."
}

main() {
    # -- Set parameters (always AGX)
    setup_parameters

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Tmux session '$SESSION' already exists. Attaching..."
        attach_tmux
        exit 0
    fi

    start_tmux_session
    start_container
    wait_for_container

    sudo make udev -B

    # MoveIt (main arm control)
    run "ros2 launch kortex_move robot.launch.py robot_ip:=10.55.155.10 vision:=true" "moveit"


    # Vision camera streams
    run "ros2 launch kinova_vision kinova_vision.launch.py depth_registration:=true device:=10.55.155.10" "vision"

    # Leaf segmentation (YOLO)
    run "ros2 launch kortex_vision leaf_segmentation.launch.py" "segmentation"

    # Arm control (arm_commander + target_manager)
    run "ros2 launch leaf_grasping_move arm_control.launch.py" "arm-control"

    # Nanospec sensor service
    run "ros2 run nanospec NSP32_service_node" "nanospec"

    # Trigger window (ready for user to run)
    echo "Creating trigger window..."
    tmux new-window -t "$SESSION" -n "trigger" "docker exec -it $CONTAINER_ID bash"
    sleep 1
    tmux send-keys -t "$SESSION:trigger" "echo '=== Press Enter to trigger the pipeline ==='" C-m
    sleep 0.5
    tmux send-keys -t "$SESSION:trigger" "ros2 action send_goal /segment_leaves kortex_interfaces/action/SegmentLeaves \"{}\""

    attach_tmux
}

main "$@"

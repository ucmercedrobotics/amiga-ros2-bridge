#!/bin/bash

# This script will run all ros2 launch files in a tmux session for easy debugging

IMAGE="ghcr.io/ucmercedrobotics/amiga-ros2-bridge"
SESSION="amiga"
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
    tmux send-keys -t "$SESSION" "cd $PROJECT_PATH; make -C $PROJECT_PATH bash" C-m
}

attach_tmux() {
    tmux attach -t "$SESSION"
}

wait_for_container() {
    echo "Waiting for container from image ${IMAGE} to start..."
    for i in $(seq 30); do
        CONTAINER_ID=$(docker ps -q --filter ancestor="${IMAGE}" | head -n 1)
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
    local WINDOW_NAME="exec-${WINDOW_INDEX}"
    echo "Creating new tmux window '$WINDOW_NAME'..."

    tmux new-window -t "$SESSION" -n "$WINDOW_NAME" "docker exec -it $CONTAINER_ID bash"
    tmux send-keys -t "$SESSION:$WINDOW_NAME" "$CMD" C-m

    WINDOW_INDEX=$((WINDOW_INDEX + 1))
}

# Ask the user for optional parameters before launching.
# Currently asks whether a sensor tower is being used and exports
# USE_SENSOR_TOWER=1 (yes) or 0 (no). The choice is shown and the
# user must confirm before the script proceeds.
ask_parameters() {
    # Prompt: sensor tower
    while true; do
        read -r -p "Do you have the sensor tower connected? [y/N]: " ans
        case "$ans" in
            [Yy]* ) USE_SENSOR_TOWER="use_lidar:=True"; break ;;
            [Nn]* ) USE_SENSOR_TOWER="use_lidar:=False"; break ;;
            ""    ) USE_SENSOR_TOWER="use_lidar:=False"; break ;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    export USE_SENSOR_TOWER

    echo
    echo "Starting bringup..."
}

main() {
    # -- Ask user for optional parameters before proceeding
    ask_parameters

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "Tmux session '$SESSION' already exists. Attaching..."
        attach_tmux
        exit 0
    fi

    start_tmux_session
    start_container
    wait_for_container

    make udev -B

    # -- Commands
    # Amiga bridge
    run "ros2 launch amiga_ros2_bridge amiga_streams.launch.py"
    run "ros2 launch amiga_ros2_bridge twist_control.launch.py"

    # Foxglove
    run "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766"

    # URDF
    run "ros2 launch amiga_ros2_description urdf.launch.py ${USE_SENSOR_TOWER}"

    # Cameras
    run "ros2 launch amiga_ros2_oakd amiga_cameras.launch.py"

    # Localization
    run "ros2 launch amiga_localization bringup.launch.py"

    # Nav2
    run "ros2 launch amiga_navigation navigation.launch.py"
    run "ros2 run amiga_navigation waypoint_follower"
    run "ros2 run amiga_navigation linear_velo"
    # TODO: replace the above with this once we confirm NAV2 working with IMU
    # run "ros2 launch amiga_navigation navigate_to_pose_in_frame"

    # Behavior Tree
    run "ros2 run amiga_ros2_behavior_tree bt_runner"

    attach_tmux
}

main "$@"

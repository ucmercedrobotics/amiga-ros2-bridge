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
    tmux send-keys -t "$SESSION" "cd $PROJECT_PATH; make -C $PROJECT_PATH bash MACHINE_NAME=$MACHINE_NAME" C-m
}

attach_tmux() {
    tmux attach -t "$SESSION"
}

wait_for_container() {
    echo "Waiting for container from image ${IMAGE} to start..."

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
    # Single environment question: Carpin vs Reza
    while true; do
        read -r -p "Is this Carpin Amiga? [y/N]: " ans
        case "$ans" in
            [Yy]* )
                # Carpin preset: BNO085 IMU, gps_link, ublox driver, no sensor tower
                USE_SENSOR_TOWER="use_lidar:=False"
                GPS_LINK="gps_link_name:=gps_link"
                VECTOR_NAV="use_vectornav:=False"
                CAM_CONFIG="camera_config:=amiga_cameras.yaml"
                YAW_OFFSET="-p yaw_offset:=0.0"
                break ;;
            [Nn]* | "" )
                # Reza preset: VectorNav IMU, gps_antenna, sensor tower
                USE_SENSOR_TOWER="use_lidar:=True"
                GPS_LINK="gps_link_name:=gps_antenna"
                VECTOR_NAV="use_vectornav:=True"
                CAM_CONFIG="camera_config:=reza_cameras.yaml"
                YAW_OFFSET="-p yaw_offset:=1.57"
                break ;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    export USE_SENSOR_TOWER
    export GPS_LINK
    export VECTOR_NAV
    export CAM_CONFIG
    export YAW_OFFSET

    # Machine selection remains: AGX or Brain
    while true; do
        read -r -p "Running on AGX (N for Brain)? [y/N]: " ans
        case "$ans" in
            [Yy]* ) MACHINE_NAME="agx"; break ;;
            [Nn]* | "" ) MACHINE_NAME="brain"; break ;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    export MACHINE_NAME

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

    while true; do
        read -r -p "Do you want to run the Kortex MoveIt stack? [y/N]: " ans
        case "$ans" in
            [Yy]* )
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
                break;;
            [Nn]* )
                break;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    while true; do
        read -r -p "Do you want to run CAN bus nodes? [y/N]: " ans
        case "$ans" in
            [Yy]* )
                run "ros2 launch amiga_ros2_bridge amiga_streams.launch.py";
                run "ros2 launch amiga_ros2_bridge twist_control.launch.py";
                break ;;
            [Nn]* )
                break;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    while true; do
        read -r -p "Do you want to run Nav2 stack? [y/N]: " ans
        case "$ans" in
            [Yy]* )
                sudo make udev -B

                if [[ "$VECTOR_NAV" == *"True"* ]]; then
                    run "ros2 launch vectornav vectornav.launch.py";
                else
                    echo "Defaulting to BNO085 IMU.";
                    run "ros2 run amiga_localization bno085_node --ros-args --params-file \
                         install/amiga_localization/share/amiga_localization/config/bno085_params.yaml";
                fi

                GPS_TOPIC="gps_topic:=/gps/pvt"
                if [[ "$GPS_LINK" != *"gps_antenna"* ]]; then
                    echo "Using UBLOX GPS RTK driver.";
                    run "ros2 launch amiga_localization ublox.launch.py";
                    GPS_TOPIC="gps_topic:=/ublox_gps_node/fix"
                fi

                echo "Launching Nav2 components...";

                # Foxglove
                run "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766";
                # run "rviz2";

                # URDF
                run "ros2 launch amiga_ros2_description urdf.launch.py ${USE_SENSOR_TOWER} ${GPS_LINK} ${VECTOR_NAV}";

                # Cameras
                run "ros2 launch amiga_ros2_oakd amiga_cameras.launch.py ${CAM_CONFIG}";

                # Localization
                run "ros2 launch amiga_localization bringup.launch.py ${VECTOR_NAV} ${GPS_TOPIC} use_gps:=True";

                # Nav2
                run "ros2 launch amiga_navigation navigation.launch.py";
                # run "ros2 run nav2_collision_monitor collision_monitor --ros-args --params-file amiga-ros2-nav/amiga_navigation/config/nav2_params.yaml";
                run "ros2 run amiga_navigation lidar_object_navigator --ros-args -p safety_distance:=0.8";
                run "ros2 run amiga_navigation waypoint_follower.py --ros-args ${YAW_OFFSET}";
                # NOTE: the commented node does the same as the below node using only /cmd/vel (no collision avoidance)
                # run "ros2 run amiga_navigation navigate_to_pose_in_frame"
                run "ros2 run amiga_navigation linear_velo --ros-args \
                    -p min_linear_velocity:=0.3 -p max_linear_velocity:=1.0 -p yaw_slowdown:=0.8 -p max_angular_velocity:=0.7 ${YAW_OFFSET}";
                run "ros2 run amiga_navigation yolo_person_follower.py";

                # Behavior Tree
                # run "ros2 launch amiga_ros2_behavior_tree bt.launch.py x_offset:=59.5 y_offset:=-7.5";
		        # run "ros2 launch amiga_ros2_behavior_tree bt.launch.py x_offset:=-3.0 y_offset:=5.0";
                run "ros2 launch amiga_ros2_behavior_tree bt.launch.py";
                break;;
            [Nn]* )
                break ;;
            * ) echo "Please answer y or n." ;;
        esac
    done

    attach_tmux
}

main "$@"

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["bash", "/amiga-ros2-bridge/scripts/run_world_state.sh"],
            output="screen",
        )
    ])

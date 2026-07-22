from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
import launch.actions
import launch.event_handlers
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_params = os.path.join(
        get_package_share_directory("amiga_localization"),
        "config",
        "zed_f9p.yaml",
    )

    ublox_node = Node(package='ublox_gps',
        executable='ublox_gps_node',
        output='both',
        parameters=[config_params])

    ublox_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ublox_node,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())],
        ))

    ld = LaunchDescription()

    ld.add_action(ublox_node)
    ld.add_action(ublox_event_handler)

    return ld

if __name__ == '__main__':
    generate_launch_description()
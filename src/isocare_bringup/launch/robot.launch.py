"""Launch the IsoCare robot nodes. Nodes are added here as they are built."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description: serial_bridge so far."""
    serial_port = LaunchConfiguration('serial_port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/tmp/esp_pi',
            description='Serial port of the ESP32 (the fake ESP32 uses /tmp/esp_pi via socat)'),
        Node(
            package='isocare_serial',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'port': serial_port}],
        ),
    ])

"""Launch the IsoCare robot nodes. Nodes are added here as they are built."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return the launch description: serial, odometry, planner, navigator, mission (+ stubs)."""
    serial_port = LaunchConfiguration('serial_port')
    use_stubs = LaunchConfiguration('use_stubs')
    marker_file = LaunchConfiguration('marker_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/tmp/esp_pi',
            description='Serial port of the ESP32 (the fake ESP32 uses /tmp/esp_pi via socat)'),
        DeclareLaunchArgument(
            'use_stubs', default_value='false',
            description='Also start demo_stubs: throw-away navigator, patient_id, '
                        'decision_engine and app_bridge so a whole visit can run'),
        Node(
            package='isocare_serial',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'port': serial_port}],
        ),
        DeclareLaunchArgument(
            'marker_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('isocare_bringup'), 'config', 'markers.yaml']),
            description='Marker table (positions and connections); placeholders until measured'),
        Node(
            package='isocare_navigation',
            executable='odometry',
            name='odometry',
            output='screen',
            # Placeholders until the chassis exists; floats must be written as floats.
            parameters=[{'ticks_per_mm': 2.0, 'wheelbase_mm': 150.0}],
        ),
        Node(
            package='isocare_navigation',
            executable='route_planner',
            name='route_planner',
            output='screen',
            parameters=[{'marker_file': marker_file}],
        ),
        Node(
            package='isocare_navigation',
            executable='navigator',
            name='navigator',
            output='screen',
            # Placeholders until the real chassis exists; floats must be written as floats.
            parameters=[{'dock_standoff_m': 0.4, 'bed_standoff_m': 0.5, 'steer_gain': 1.5}],
        ),
        Node(
            package='isocare_mission',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{
                'identify_mode': 'nurse',               # 'qr', 'nurse' or 'skip'
                'bed_patients': ['A=P001', 'B=P002'],   # bed id = assigned patient id
                'drawer_hold_s': 20,
            }],
        ),
        Node(
            package='isocare_mission',
            executable='demo_stubs',
            name='demo_stubs',
            output='screen',
            condition=IfCondition(use_stubs),
        ),
    ])

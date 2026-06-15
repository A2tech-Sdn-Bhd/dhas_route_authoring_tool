"""Convenience launch file for the editor.

This package's executables are plain Python (no rclpy node), but a launch
file is still handy so users can ``ros2 launch route_authoring_tool
route_editor.launch.py config:=/path/to/route_authoring.yaml`` without
remembering the underlying entry point.

For per-run overrides of bag/input/output, call ``route_editor`` directly
from the shell — that's less plumbing than passing them through launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='',
            description='Path to route_authoring.yaml. Empty uses installed defaults.',
        ),
        ExecuteProcess(
            cmd=['route_editor', '--config', config],
            output='screen',
        ),
    ])

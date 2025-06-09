from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_tools',
            executable='gps_parser',
            name='gps'
        )
    ])

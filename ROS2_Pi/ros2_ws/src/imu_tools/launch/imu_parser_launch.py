#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_tools',
            executable='imu_parser_node',
            name='imu_parser_node',
            output='screen',
            parameters=[
                # You can add parameters here if needed in the future
            ]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_com',
            executable='serial_node',
            name='comm'
        )
    ])

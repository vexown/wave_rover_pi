from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Convert compressed images to raw
        Node(
            package='visual_odo',
            executable='compressed_to_raw',
            name='compressed_to_raw',
            output='screen'
        ),
        
        # Simple visual odometry
        Node(
            package='visual_odo',
            executable='simple_vo',
            name='simple_visual_odometry',
            output='screen'
        ),
    ])

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # static TF odom -> base_link (replace zeros with real offset if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # static TF base_link -> camera_link (replace zeros with real offset if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            output='screen'
        ),

        # local compressed->raw republisher (decoding on PC). Ensure script is executable.
        ExecuteProcess(
            cmd=['python3',
                 '/home/blankmcu/Repos/wave_rover_pi/PC-side-Software/ROS2_PC/ros2_ws/src/visual_odo/scripts/camera_feed_raw_republisher.py'],
            output='screen'
        ),

        # RTAB-Map Visual Odometry Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'base_link',           # Robot's base frame
                'odom_frame_id': 'odom',           # Odometry frame
                'subscribe_depth': False,          # RGB-only VO (no depth)
                'subscribe_rgb': True,             # Subscribe to RGB images
                'rgb_topic': '/camera/image_raw',
                'camera_info_topic': '/camera/camera_info',    # Camera calibration
                'sync_queue_size': 10,             # Message queue size
                'publish_tf': True,                # Publish TF transforms
                'odom_tf_angular_variance': 0.01,  # Angular variance for odometry
                'odom_tf_linear_variance': 0.01,   # Linear variance for odometry
                'Vis/MaxFeatures': '500',          # Feature detection limit
                'Vis/MinInliers': '15',            # Minimum inliers for motion estimation
                'qos_image': 2,                    # QoS for image (match your publisher)
                'qos_camera_info': 2,              # QoS for camera_info (match your publisher)
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/visual/odom'),
            ],
            output='screen'
        )
    ])
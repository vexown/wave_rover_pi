import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'rgb_topic': '/camera/image_raw/compressed',  # Your camera topic
                'camera_info_topic': '/camera/camera_info',    # Camera calibration
                'sync_queue_size': 10,             # Message queue size
                'publish_tf': True,                # Publish TF transforms
                'odom_tf_angular_variance': 0.01,  # Angular variance for odometry
                'odom_tf_linear_variance': 0.01,   # Linear variance for odometry
                'Vis/MaxFeatures': '500',          # Feature detection limit
                'Vis/MinInliers': '15',            # Minimum inliers for motion estimation
                'qos_image': 2,                    # Set to BEST_EFFORT (matches camera)
                'qos_camera_info': 2,              # Set to BEST_EFFORT (matches camera)
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw/compressed'),  # Remap to your topic
                ('rgb/camera_info', '/camera/camera_info'),     # Remap to your topic
                ('odom', '/visual/odom'),          # Publish to /visual/odom
            ],
            output='screen'
        )
    ])
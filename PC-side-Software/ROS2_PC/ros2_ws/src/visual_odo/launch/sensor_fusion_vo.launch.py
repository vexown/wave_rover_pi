#!/usr/bin/env python3
"""
Sensor Fusion Visual Odometry Launch File

This launch file combines visual odometry with IMU data using robot_localization's
Extended Kalman Filter (EKF) for robust pose estimation. It launches:

1. Image conversion node (compressed to raw)
2. Visual odometry node with debug visualization
3. Robot localization EKF node for sensor fusion

The EKF fuses:
- Visual odometry estimates (position and orientation)  
- IMU data (orientation and angular velocity)

This provides more robust localization with reduced drift compared to 
visual odometry alone.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Convert compressed images to raw for visual odometry
        Node(
            package='visual_odo',
            executable='compressed_to_raw',
            name='compressed_to_raw',
            output='screen'
        ),
        
        # Visual odometry node with all improvements
        Node(
            package='visual_odo',
            executable='simple_vo',
            name='simple_visual_odometry',
            output='screen',
            parameters=[
                {'focal': 800.0},           # Focal length in pixels (tune for your camera)
                {'cx': 400.0},              # Principal point x-coordinate
                {'cy': 300.0},              # Principal point y-coordinate
                {'scale': 0.1},             # Scale factor for translation (placeholder)
                {'orb_nfeatures': 500},     # Number of ORB features to detect
                {'min_matches': 10},        # Minimum feature matches required
                {'ransac_thresh': 1.0},     # RANSAC threshold for essential matrix
                {'max_good_matches': 50},   # Maximum good matches to use
                {'ratio_thresh': 0.75},     # Lowe's ratio test threshold
                {'min_inliers': 5},         # Minimum RANSAC inliers for pose estimation
                {'enable_debug_viz': True}, # Enable debug visualization
            ]
        ),
        
        # Extended Kalman Filter for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_se_odom',
            output='screen',
            parameters=[{
                # EKF Configuration
                # ================
                'frequency': 30.0,          # Filter update frequency (Hz)
                'sensor_timeout': 0.1,      # Timeout for sensor data (seconds)
                'two_d_mode': True,         # Enable 2D mode for ground robots
                'transform_time_offset': 0.0,  # Time offset for transforms
                'transform_timeout': 0.0,   # Transform timeout
                'print_diagnostics': True,  # Print diagnostic information
                'debug': False,             # Disable debug output for performance
                
                # Visual Odometry Input Configuration
                # ===================================
                # Subscribe to visual odometry output and use selected state variables
                'odom0': '/visual_odom',    # Visual odometry topic
                'odom0_config': [
                    True,   True,  False,   # x, y, z position (use x,y for 2D robot)
                    False, False,  True,    # roll, pitch, yaw (use yaw for heading)
                    False, False, False,    # x_dot, y_dot, z_dot (velocities - not reliable from VO)
                    False, False, False,    # roll_dot, pitch_dot, yaw_dot (angular velocities)
                    False, False, False     # x_ddot, y_ddot, z_ddot (accelerations)
                ],
                'odom0_queue_size': 10,     # Queue size for odometry messages
                'odom0_differential': False,# Use absolute measurements (not differential)
                'odom0_relative': False,    # Use global coordinates (not relative)
                
                # IMU Input Configuration  
                # =======================
                # Subscribe to IMU data and use orientation and angular velocity
                'imu0': '/imu/data',        # IMU topic
                'imu0_config': [
                    False, False, False,    # x, y, z position (IMU doesn't provide position)
                    False, False, True,     # roll, pitch, yaw (use yaw orientation from IMU)
                    False, False, False,    # x_dot, y_dot, z_dot (velocities)
                    False, False, True,     # roll_dot, pitch_dot, yaw_dot (use yaw angular velocity)
                    False, False, False     # x_ddot, y_ddot, z_ddot (accelerations - can enable if needed)
                ],
                'imu0_queue_size': 10,      # Queue size for IMU messages
                'imu0_differential': False, # Use absolute measurements
                'imu0_relative': False,     # Use global coordinates
                'imu0_remove_gravitational_acceleration': True,  # Remove gravity from acceleration
                
                # Frame Configuration
                # ==================
                # Define coordinate frames for the robot
                'map_frame': 'map',         # Global reference frame
                'odom_frame': 'odom',       # Odometry frame (continuous, but may drift)
                'base_link_frame': 'base_link',  # Robot body frame
                'world_frame': 'odom',      # World frame (same as odom for this setup)
                
                # Process Noise Covariance
                # ========================
                # Diagonal elements of process noise covariance matrix
                # Higher values allow more rapid changes in state estimates
                'process_noise_covariance': [
                    0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # x
                    0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # y
                    0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # z
                    0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # roll
                    0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # pitch
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # yaw
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vx
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vy
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vz
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,  # vroll
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  # vpitch
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,  # vyaw
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  # ax
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  # ay
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015 # az
                ],
                
                # Initial Estimate Covariance
                # ===========================
                # Uncertainty in initial state estimate (smaller = more confident)
                'initial_estimate_covariance': [
                    1e-9, 1e-9, 1e-9,  # position (x, y, z)
                    1e-9, 1e-9, 1e-9,  # orientation (roll, pitch, yaw)
                    1e-9, 1e-9, 1e-9,  # velocity (vx, vy, vz)
                    1e-9, 1e-9, 1e-9,  # angular velocity (vroll, vpitch, vyaw)
                    1e-9, 1e-9, 1e-9   # acceleration (ax, ay, az)
                ],
            }]
        ),
        
        # Optional: Static transform from base_link to camera_link
        # (Adjust translation/rotation based on your camera mounting)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_transform',
            arguments=['0.1', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
            output='screen'
        ),
        
        # Optional: Static transform from base_link to imu_link  
        # (Adjust based on your IMU mounting position)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            name='imu_transform',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
            output='screen'
        ),
    ])
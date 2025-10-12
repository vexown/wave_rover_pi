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
        
        # Visual odometry node with enhanced stationary drift prevention
        Node(
            package='visual_odo',
            executable='simple_vo',
            name='simple_visual_odometry',
            output='screen',
            parameters=[
                # Camera intrinsics - CALIBRATED VALUES from camera_calibration.yaml
                {'focal': 1204.0},          # fx from camera matrix: 1203.78 ≈ 1204
                {'cx': 365.0},              # cx from camera matrix: 364.80 ≈ 365  
                {'cy': 291.0},              # cy from camera matrix: 290.55 ≈ 291
                
                # Scale factor - reduced to minimize noise amplification
                {'scale': 0.05},            # Reduced from 0.08 - less amplification of noise
                {'orb_nfeatures': 500},     # Balanced feature count: enough for robust matching without excessive CPU load
                
                # Feature matching parameters - stricter for better quality
                {'min_matches': 15},        # Increased from 10 - more matches required for reliability
                {'ransac_thresh': 0.8},     # Reduced from 1.0 - stricter geometric consistency
                {'max_good_matches': 50},   # Process top 50 matches: reduces computation while retaining best correspondences
                {'ratio_thresh': 0.60},     # Lowe's ratio test: stricter than default 0.75 to improve match distinctiveness and reduce ambiguous matches
                {'min_inliers': 9},         # RANSAC inlier requirement: higher than minimal 5 for reliability, lower than strict 15 for motion sensitivity
                
                # ENHANCED MOTION FILTERING - multi-layer drift prevention
                {'motion_threshold': 0.005},      # Increased from 0.002 - 2.5x stricter translation threshold
                {'min_motion_inliers': 12},       # NEW: Require more inliers to accept motion (was implicit at min_inliers)
                {'rotation_threshold': 0.015},    # NEW: Separate rotation threshold (0.86 degrees minimum)
                
                # Debug visualization and diagnostics
                {'enable_debug_viz': False}, # Real-time match visualization for parameter tuning and system monitoring
            ]
        ),
        
        # Extended Kalman Filter for sensor fusion
        #
        # What is EKF?
        # ============
        # Think of EKF as a "smart data combiner" that takes multiple imperfect sensor readings
        # and produces a single, more accurate estimate of where your robot is and how it's moving.
        #
        # Why do we need it here?
        # =======================
        # Our robot has two main sensors telling us about position/orientation:
        # 1. CAMERA (visual odometry) - Good at detecting movement but drifts over time and has no absolute reference
        # 2. IMU (gyroscope/accelerometer) - Good at detecting rotation and short-term changes but also drifts
        #
        # Each sensor has different strengths and weaknesses:
        # - Camera: Accurate for short distances but accumulates errors over time ("drift")
        # - IMU: Great for detecting quick rotations but terrible for long-term position tracking
        #
        # How does EKF help?
        # ==================
        # Instead of just picking one sensor or averaging them, EKF is smart about it:
        # - When the camera says "we moved forward 10cm" and IMU says "we rotated 5 degrees", 
        #   EKF combines these intelligently based on how much it trusts each sensor
        # - If camera data becomes unreliable (poor lighting, motion blur), EKF relies more on IMU
        # - If IMU starts drifting, EKF trusts the camera more for position
        # - The result is smoother, more accurate robot pose estimation than either sensor alone
        #
        # Real-world benefits for our robot:
        # ==================================
        # - Better navigation in challenging conditions (shadows, bright light, fast turns)
        # - Reduced "jumping" or erratic position estimates in visualization
        # - More stable autonomous driving behavior
        # - Backup sensing when one sensor temporarily fails
        #
        Node(
            # robot_localization is a state estimation package that fuses data from multiple sensors (e.g., IMU, GPS, wheel odometry, visual odometry) 
            # to estimate a robot’s pose (position and orientation) and velocity over time.
            package='robot_localization',  
            executable='ekf_node',         # Node binary to run (Extended Kalman Filter) provided by the robot_localization package
            name='ekf_se_odom',            # Custom name for the node instance
            output='screen',               # Print node logs to the terminal
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
                # REDUCED VALUES: Lower process noise makes the filter more resistant to drift
                # when stationary, but still responsive to real motion from sensors
                'process_noise_covariance': [
                    0.001, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # x (was 0.05, now 0.001 - 50x lower)
                    0.0,  0.001, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # y (was 0.05, now 0.001)
                    0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # z (not used in 2D mode)
                    0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # roll (not used in 2D mode)
                    0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # pitch (not used in 2D mode)
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.001, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # yaw (was 0.06, now 0.001 - 60x lower)
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vx
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vy
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  # vz
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,  # vroll
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  # vpitch
                    0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.005, 0.0,  0.0,  0.0,  # vyaw (was 0.02, now 0.005 - 4x lower)
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
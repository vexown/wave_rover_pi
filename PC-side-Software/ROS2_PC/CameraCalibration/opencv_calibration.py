#!/usr/bin/env python3
"""
OpenCV Camera Calibration using your existing ROS camera stream
# First print-out a 250mm x 200m checkerboard with 7 rows 9 columns and 25mm squares.
# https://calib.io/pages/camera-calibration-pattern-generator?srsltid=AfmBOoqFdbxj9zzbAa_V9Pem9-nJ1KIuPBrhBbDZQLspMJFW69y3F3B0
#
# Then run this script while pointing the camera at the checkerboard from different angles and distances.
# - Translation: Move closer/farther from camera
# - Rotation: Tilt in different directions
# - Orientation: Show different angles (portrait/landscape)
# - Coverage: Fill different parts of the frame
# Make sure the checkerboard is well-lit and in focus.
#
# To capture a calibration image, press SPACE when the checkerboard is detected (green corners).
# You need at least 5 images for a good calibration.
# To finish the calibration, press ESC.
#
# Use the results to update your camera intrinsics in the launch file and provide the .yaml calibration file (the standard one) to your camera node.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import glob
import pickle
import os
import yaml  # Added for standard YAML output

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        self.bridge = CvBridge()
        
        # Match QoS with camera publisher (BEST_EFFORT, KEEP_LAST, depth=1)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to compressed images
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            sensor_qos)
        
        # Chessboard parameters
        self.checkerboard_size = (8, 6)  # 8x6 internal corners
        self.square_size = 0.025  # 25mm squares
        
        # Arrays to store object points and image points
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (7,5,0)
        self.objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        self.image_count = 0
        self.calibrating = True
        
        self.get_logger().info('Camera Calibrator started. Green corners = detected. Press SPACE to capture, ESC to finish (need 5+ images)')

    def image_callback(self, msg):
        if not self.calibrating:
            return
            
        try:
            # Convert compressed image to OpenCV
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)
            
            # Always draw corners if detected (for visual feedback)
            if ret:
                # Refine corner detection
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # Draw corners on image for visual feedback
                cv2.drawChessboardCorners(cv_image, self.checkerboard_size, corners, ret)
            
            # Display image
            cv2.imshow('Camera Calibration', cv_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27:  # ESC
                self.calibrating = False
                if self.image_count >= 5:  # Need at least 5 images for calibration
                    self.perform_calibration(cv_image.shape[:2][::-1])  # (width, height)
                else:
                    self.get_logger().error('Need at least 5 calibration images. Exiting.')
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
            elif key == 32:  # SPACE - Manual capture
                if ret:
                    # Store points only when SPACE is pressed
                    self.objpoints.append(self.objp)
                    self.imgpoints.append(corners)
                    self.image_count += 1
                    self.get_logger().info(f'Captured calibration image {self.image_count}')
                else:
                    self.get_logger().warn('No chessboard detected. Adjust position and try again.')
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def perform_calibration(self, image_size):
        if len(self.objpoints) < 5:
            self.get_logger().error(f'Need at least 5 calibration images, got {len(self.objpoints)}. Exiting.')
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
            
        self.get_logger().info(f'Performing calibration with {len(self.objpoints)} images...')
        
        # Calibrate camera
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None)
        
        if ret:
            self.get_logger().info('Calibration successful!')
            
            # Save calibration data
            calibration_data = {
                'camera_matrix': camera_matrix,
                'dist_coeffs': dist_coeffs,
                'image_size': image_size,
                'rms_error': ret
            }
            
            # Save as pickle
            with open('camera_calibration.pkl', 'wb') as f:
                pickle.dump(calibration_data, f)
            
            # Save as YAML for ROS
            fs = cv2.FileStorage('camera_calibration.yaml', cv2.FILE_STORAGE_WRITE)
            fs.write('camera_matrix', camera_matrix)
            fs.write('dist_coeff', dist_coeffs)
            fs.write('image_width', image_size[0])
            fs.write('image_height', image_size[1])
            fs.release()
            
            # NEW: Save in standard YAML format for ROS2 node compatibility
            standard_data = {
                'image_width': image_size[0],
                'image_height': image_size[1],
                'camera_name': 'rpi_camera',
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': camera_matrix.flatten().tolist()
                },
                'distortion_model': 'plumb_bob',
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': 5,
                    'data': dist_coeffs.flatten().tolist()
                },
                'rectification_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity (no rectification)
                },
                'projection_matrix': {
                    'rows': 3,
                    'cols': 4,
                    'data': [float(camera_matrix[0,0]), float(camera_matrix[0,1]), float(camera_matrix[0,2]), 0.0,
                             float(camera_matrix[1,0]), float(camera_matrix[1,1]), float(camera_matrix[1,2]), 0.0,
                             float(camera_matrix[2,0]), float(camera_matrix[2,1]), float(camera_matrix[2,2]), 0.0]
                }
            }
            
            with open('camera_calibration_standard.yaml', 'w') as f:
                yaml.dump(standard_data, f, default_flow_style=False)
            
            # Print results
            print("\n=== CALIBRATION RESULTS ===")
            print(f"RMS re-projection error: {ret:.4f}")
            print(f"Camera matrix:\n{camera_matrix}")
            print(f"Focal length (fx, fy): {camera_matrix[0,0]:.2f}, {camera_matrix[1,1]:.2f}")
            print(f"Principal point (cx, cy): {camera_matrix[0,2]:.2f}, {camera_matrix[1,2]:.2f}")
            print(f"Distortion coefficients: {dist_coeffs.ravel()}")
            
            print("\n=== UPDATE YOUR LAUNCH FILE ===")
            print(f"  'focal': {camera_matrix[0,0]:.1f},")
            print(f"  'cx': {camera_matrix[0,2]:.1f},")
            print(f"  'cy': {camera_matrix[1,2]:.1f},")
            
            print("\n=== FILES SAVED ===")
            print("  camera_calibration.yaml (OpenCV format)")
            print("  camera_calibration_standard.yaml (Standard YAML for ROS2)")
            print("Copy camera_calibration_standard.yaml to your ROS2 workspace (e.g., ~/ros2_ws/camera_ros_config/)")
            
        else:
            self.get_logger().error('Calibration failed!')
        
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    calibrator = CameraCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import tempfile
import os
from threading import Thread
import time

class RPiCamPublisher(Node):
    def __init__(self):
        super().__init__('rpicam_publisher')
        
        # Parameters
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 600)
        self.declare_parameter('fps', 10)
        self.declare_parameter('format', 'rgb')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.format = self.get_parameter('format').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Compressed image publisher (optional)
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera info message
        self.camera_info_msg = self.create_camera_info()
        
        # Start camera capture
        self.running = True
        self.capture_thread = Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info(f'RPiCam Publisher started - {self.width}x{self.height}@{self.fps}fps')
        if self.publish_compressed:
            self.get_logger().info(f'Publishing compressed images with quality {self.jpeg_quality}%')
    
    def create_camera_info(self):
        """Create a basic camera info message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_link"
        camera_info.width = self.width
        camera_info.height = self.height
        
        # Basic camera matrix (you should calibrate for accurate values)
        fx = fy = self.width  # Rough estimate
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return camera_info
    
    def create_compressed_image(self, cv_image, timestamp):
        """Create a compressed image message from OpenCV image"""
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = timestamp
        compressed_msg.header.frame_id = "camera_link"
        compressed_msg.format = "jpeg"
        
        # Encode image as JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        success, encoded_image = cv2.imencode('.jpg', cv_image, encode_params)
        
        if success:
            compressed_msg.data = np.array(encoded_image).tobytes()
        else:
            self.get_logger().warn('Failed to encode image as JPEG')
            
        return compressed_msg
    
    def capture_loop(self):
        """Main capture loop using rpicam-still"""
        frame_time = 1.0 / self.fps
        
        while self.running:
            start_time = time.time()
            
            try:
                # Use rpicam-still to capture image
                with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as temp_file:
                    temp_filename = temp_file.name
                
                # Capture image using rpicam-still
                cmd = [
                    'rpicam-still',
                    '--output', temp_filename,
                    '--width', str(self.width),
                    '--height', str(self.height),
                    '--immediate',
                    '--nopreview',
                    '--timeout', '1000'  # 1 second timeout
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True)
                
                if result.returncode == 0:
                    # Read the captured image
                    cv_image = cv2.imread(temp_filename)
                    
                    if cv_image is not None:
                        # Convert BGR to RGB if needed
                        if self.format == 'rgb':
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                            encoding = 'rgb8'
                        else:
                            encoding = 'bgr8'
                        
                        # Create ROS2 image message
                        now = self.get_clock().now()
                        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
                        image_msg.header.stamp = now.to_msg()
                        image_msg.header.frame_id = "camera_link"
                        
                        # Create camera info message
                        camera_info_msg = self.camera_info_msg
                        camera_info_msg.header.stamp = now.to_msg()
                        
                        # Publish messages
                        self.image_pub.publish(image_msg)
                        self.camera_info_pub.publish(camera_info_msg)
                        
                        # Publish compressed image if enabled
                        if self.publish_compressed:
                            compressed_msg = self.create_compressed_image(cv_image, now.to_msg())
                            self.compressed_pub.publish(compressed_msg)
                        
                        self.get_logger().debug('Published image frame')
                    else:
                        self.get_logger().warn('Failed to read captured image')
                else:
                    self.get_logger().warn(f'rpicam-still failed: {result.stderr}')
                
                # Clean up temp file
                if os.path.exists(temp_filename):
                    os.unlink(temp_filename)
                    
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {str(e)}')
            
            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed)
            time.sleep(sleep_time)
    
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RPiCamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

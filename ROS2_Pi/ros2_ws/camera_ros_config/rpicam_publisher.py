#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import tempfile
import os
import time
from threading import Thread

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
        
        # Calculate frame interval
        self.frame_interval = 1.0 / self.fps
        
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
        
        # Create temporary file for image capture
        self.temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
        self.temp_filename = self.temp_file.name
        self.temp_file.close()
        
        # Start camera capture
        self.running = True
        self.capture_thread = Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info(f'RPiCam Publisher started - {self.width}x{self.height}@{self.fps}fps')
        if self.publish_compressed:
            self.get_logger().info(f'Publishing compressed images at {self.jpeg_quality}% quality')
    
    def create_camera_info(self):
        """Create a basic camera info message"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera_link"
        camera_info.width = self.width
        camera_info.height = self.height
        # Basic camera matrix (you may want to calibrate this properly)
        camera_info.k = [self.width, 0.0, self.width/2, 0.0, self.height, self.height/2, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [self.width, 0.0, self.width/2, 0.0, 0.0, self.height, self.height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        return camera_info
    
    def create_compressed_image(self, cv_image, timestamp):
        """Create compressed image message"""
        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = timestamp
        compressed_msg.header.frame_id = "camera_link"
        compressed_msg.format = "jpeg"
        
        # Encode image as JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        success, encoded_image = cv2.imencode('.jpg', cv_image, encode_params)
        
        if success:
            compressed_msg.data = encoded_image.tobytes()
        
        return compressed_msg
    
    def capture_image(self):
        """Capture a single image using rpicam-still"""
        try:
            # Build rpicam-still command
            cmd = [
                'rpicam-still',
                '--output', self.temp_filename,
                '--width', str(self.width),
                '--height', str(self.height),
                '--timeout', '1',  # Very short timeout for fast capture
                '--nopreview',     # No preview window
                '--immediate',     # Capture immediately
                '--encoding', 'jpg'
            ]
            
            # Execute command with timeout
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            
            if result.returncode == 0 and os.path.exists(self.temp_filename):
                # Read the captured image
                cv_image = cv2.imread(self.temp_filename, cv2.IMREAD_COLOR)
                if cv_image is not None:
                    return cv_image
                else:
                    self.get_logger().warning("Failed to read captured image file")
            else:
                self.get_logger().warning(f"rpicam-still failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().warning("rpicam-still capture timeout")
        except Exception as e:
            self.get_logger().error(f"Capture error: {str(e)}")
        
        return None
    
    def capture_loop(self):
        """Main capture loop"""
        last_capture_time = time.time()
        
        while self.running and rclpy.ok():
            start_time = time.time()
            
            # Capture image
            cv_image = self.capture_image()
            
            if cv_image is not None:
                # Create timestamp
                now = self.get_clock().now()
                
                # Convert BGR to RGB if needed
                if self.format == 'rgb':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                    encoding = 'rgb8'
                else:
                    encoding = 'bgr8'
                
                # Create and publish raw image
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
                    img_msg.header.stamp = now.to_msg()
                    img_msg.header.frame_id = "camera_link"
                    self.image_pub.publish(img_msg)
                    
                    # Publish camera info
                    self.camera_info_msg.header.stamp = now.to_msg()
                    self.camera_info_pub.publish(self.camera_info_msg)
                    
                    # Publish compressed image if enabled
                    if self.publish_compressed:
                        # Convert back to BGR for JPEG encoding
                        if self.format == 'rgb':
                            bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                        else:
                            bgr_image = cv_image
                        
                        compressed_msg = self.create_compressed_image(bgr_image, now.to_msg())
                        self.compressed_pub.publish(compressed_msg)
                    
                    # Log performance occasionally
                    current_time = time.time()
                    if current_time - last_capture_time > 5.0:  # Every 5 seconds
                        actual_fps = 1.0 / (current_time - start_time) if (current_time - start_time) > 0 else 0
                        self.get_logger().info(f"Capture performance: {actual_fps:.1f} fps target, actual loop time: {(current_time - start_time):.3f}s")
                        last_capture_time = current_time
                        
                except Exception as e:
                    self.get_logger().error(f"Publishing error: {str(e)}")
            
            # Sleep to maintain target FPS
            elapsed = time.time() - start_time
            sleep_time = max(0, self.frame_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def destroy_node(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)
        
        # Clean up temporary file
        try:
            if os.path.exists(self.temp_filename):
                os.unlink(self.temp_filename)
        except:
            pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = RPiCamPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("Shutting down RPiCam Publisher...")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
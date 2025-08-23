#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import os
import time
import signal
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
        
        # Initialize camera streaming
        self.running = True
        self.gst_process = None
        self.capture_thread = None
        self.frame_count = 0
        self.last_log_time = time.time()
        self.debug_count = 0
        
        # Start camera streaming
        self.start_camera()
        
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
    
    def start_camera(self):
        """Start GStreamer libcamera pipeline for continuous streaming"""
        try:
            # Check if gstreamer is available
            if not self.check_gstreamer():
                self.get_logger().error("GStreamer or libcamera plugin not available")
                return
            
            # Build GStreamer pipeline for libcamera -> MJPEG streaming
            pipeline = (
                f"libcamerasrc ! "
                f"video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1 ! "
                f"jpegenc quality={self.jpeg_quality} ! "
                f"fdsink fd=1"
            )
            
            cmd = ['gst-launch-1.0', '-q', pipeline]
            
            self.get_logger().info(f"Starting GStreamer pipeline: {' '.join(cmd)}")
            
            # Start the process
            self.gst_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0
            )
            
            # Check if process started successfully
            time.sleep(0.5)  # Give it a moment to start
            if self.gst_process.poll() is not None:
                # Process has already terminated
                _, stderr = self.gst_process.communicate()
                self.get_logger().error(f"GStreamer failed to start: {stderr.decode()}")
                return
            
            # Start capture thread
            self.capture_thread = Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            self.get_logger().info("Started GStreamer libcamera pipeline successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to start camera: {str(e)}")
            self.running = False
    
    def check_gstreamer(self):
        """Check if GStreamer and libcamera plugin are available"""
        try:
            # Check if gst-launch-1.0 is available
            result1 = subprocess.run(['which', 'gst-launch-1.0'], capture_output=True)
            if result1.returncode != 0:
                self.get_logger().error("gst-launch-1.0 not found. Install with: sudo apt install gstreamer1.0-tools")
                return False
            
            # Check if libcamerasrc plugin is available
            result2 = subprocess.run(['gst-inspect-1.0', 'libcamerasrc'], capture_output=True)
            if result2.returncode != 0:
                self.get_logger().error("libcamerasrc plugin not found. Install with: sudo apt install gstreamer1.0-libcamera")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error checking GStreamer: {str(e)}")
            return False
    
    def capture_loop(self):
        """Main capture loop - reads MJPEG stream from GStreamer"""
        buffer = b''
        bytes_read = 0
        
        self.get_logger().info("Starting GStreamer capture loop...")
        
        while self.running and rclpy.ok() and self.gst_process:
            try:
                # Check if process is still running
                if self.gst_process.poll() is not None:
                    _, stderr = self.gst_process.communicate()
                    self.get_logger().error(f"GStreamer process terminated: {stderr.decode()}")
                    break
                
                # Read data from GStreamer stdout
                chunk = self.gst_process.stdout.read(4096)
                if not chunk:
                    self.get_logger().warning("No data received from GStreamer")
                    time.sleep(0.1)
                    continue
                
                bytes_read += len(chunk)
                buffer += chunk
                
                # Debug: Log data reception occasionally
                self.debug_count += 1
                if self.debug_count % 100 == 0:  # Every 100 chunks
                    self.get_logger().info(f"Received {bytes_read} bytes total, buffer size: {len(buffer)}")
                
                # Look for JPEG frame boundaries (0xFFD8 = start, 0xFFD9 = end)
                frames_found = 0
                while True:
                    start = buffer.find(b'\xff\xd8')
                    if start == -1:
                        break
                    
                    end = buffer.find(b'\xff\xd9', start)
                    if end == -1:
                        break
                    
                    # Extract complete JPEG frame
                    jpeg_data = buffer[start:end+2]
                    buffer = buffer[end+2:]
                    frames_found += 1
                    
                    # Process the frame
                    self.process_frame(jpeg_data)
                    
                if frames_found > 0:
                    self.get_logger().debug(f"Found and processed {frames_found} frames")
                    
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Capture loop error: {str(e)}")
                break
        
        self.get_logger().info("Capture loop ended")
    
    def process_frame(self, jpeg_data):
        """Process a single JPEG frame"""
        try:
            self.get_logger().debug(f"Processing JPEG frame of {len(jpeg_data)} bytes")
            
            # Decode JPEG to OpenCV format
            nparr = np.frombuffer(jpeg_data, np.uint8)
            cv_image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warning("Failed to decode JPEG frame")
                return
            
            self.get_logger().debug(f"Decoded image: {cv_image.shape}")
            
            # Create timestamp
            now = self.get_clock().now()
            
            # Convert BGR to RGB if needed
            if self.format == 'rgb':
                display_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                encoding = 'rgb8'
            else:
                display_image = cv_image
                encoding = 'bgr8'
            
            # Create and publish raw image
            img_msg = self.bridge.cv2_to_imgmsg(display_image, encoding=encoding)
            img_msg.header.stamp = now.to_msg()
            img_msg.header.frame_id = "camera_link"
            self.image_pub.publish(img_msg)
            
            # Publish camera info
            self.camera_info_msg.header.stamp = now.to_msg()
            self.camera_info_pub.publish(self.camera_info_msg)
            
            # Publish compressed image if enabled (reuse the JPEG data)
            if self.publish_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = now.to_msg()
                compressed_msg.header.frame_id = "camera_link"
                compressed_msg.format = "jpeg"
                compressed_msg.data = jpeg_data
                self.compressed_pub.publish(compressed_msg)
            
            # Log performance occasionally
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:  # Every 5 seconds
                actual_fps = self.frame_count / (current_time - self.last_log_time)
                self.get_logger().info(f"Publishing at {actual_fps:.1f} fps (target: {self.fps} fps)")
                self.frame_count = 0
                self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {str(e)}")
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info("Shutting down camera publisher...")
        self.running = False
        
        # Stop GStreamer process
        if self.gst_process:
            try:
                self.gst_process.terminate()
                self.gst_process.wait(timeout=2)
            except:
                if self.gst_process.poll() is None:
                    self.gst_process.kill()
        
        # Wait for capture thread to finish
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    publisher = None
    try:
        publisher = RPiCamPublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("Shutting down RPiCam Publisher...")
    finally:
        if publisher:
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
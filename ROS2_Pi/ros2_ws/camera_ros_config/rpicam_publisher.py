#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
import subprocess
import os
import time
import signal
from threading import Thread

# Global shutdown coordination flag - allows clean termination across threads
# Set to True when SIGINT (Ctrl+C) or SIGTERM (kill command) received to signal all components to stop
#       SIGINT = Signal Interrupt (signal 2) - triggered by Ctrl+C, allows graceful cleanup
#       SIGTERM = Signal Terminate (signal 15) - triggered by kill command, also allows graceful shutdown
# Without this flag, abrupt termination would leave camera processes running and ROS2 resources unclean
shutdown_requested = False

class RPiCamPublisher(Node):
    # Constructor that sets up the ROS2 camera publisher node.
    def __init__(self):
        super().__init__('rpicam_publisher')  # Create ROS2 node named 'rpicam_publisher'
        
        # ROS2 parameters - configurable values that can be set when launching the node
        # These allow external configuration without code changes (via launch files, command line, etc.)
        # The values below are default ones and are overriden by the values provided in the ros2 launch command
        self.declare_parameters('', [
            ('width', 800),                      # Camera resolution width in pixels
            ('height', 600),                     # Camera resolution height in pixels  
            ('fps', 10),                         # Target frames per second
            ('jpeg_quality', 80),                # JPEG compression quality (1-100, higher = better quality/larger files)
            ('watchdog_grace_sec', 6.0),         # Extra time allowed between frames before restart
            ('startup_no_frame_timeout', 12.0),  # Max seconds to wait for first frame before restart
            ('enable_stderr_logging', True)      # Whether to log GStreamer error messages
        ])

        # Extract and validate parameter values
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = max(1, int(self.get_parameter('fps').value))  # Ensure FPS >= 1
        self.jpeg_quality = int(min(100, max(1, self.get_parameter('jpeg_quality').value)))  # Clamp 1-100

        # QoS (Quality of Service) Profile - controls how ROS2 handles message delivery
        # BEST_EFFORT: prioritize speed over reliability (OK to drop frames if network is slow)
        # KEEP_LAST + depth=1: only keep the newest frame, discard older ones to prevent lag
        # This prevents video buffering/delay by always showing the most recent camera frame
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Drop frames rather than guarantee delivery
            history=HistoryPolicy.KEEP_LAST,            # Only keep the most recent messages
            depth=1                                     # Buffer size of 1 = drop old frames immediately
        )

        # Create ROS2 publishers - these send data to other nodes in the network
        # CompressedImage: sends JPEG-compressed camera frames to '/camera/image_raw/compressed' topic
        # CameraInfo: sends camera calibration data to '/camera/camera_info' topic (required by many vision nodes)
        self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', sensor_qos)

        # Create camera calibration message - contains intrinsic camera parameters
        # This tells subscribers the camera's focal length, distortion, etc. for 3D vision tasks
        self.camera_info_msg = self.create_camera_info()

        # Initialize camera streaming state variables
        self.running = True                            # Main control flag - set to False to stop everything
        self.gst_process = None                        # Handle to GStreamer subprocess (camera pipeline)
        self.capture_thread = None                     # Background thread that reads camera frames
        self.frame_count = 0                           # Counter for FPS calculation (resets every 5 seconds)
        self.frames_seen = 0                           # Total frames received since startup (never resets)
        self.last_log_time = time.monotonic()          # Timestamp of last FPS log message
        self.last_frame_time = time.monotonic()        # Timestamp when last frame was received

        # Watchdog timing - automatically restarts camera if it stops producing frames
        self.watchdog_interval = 3.0 # Watchdog interval (seconds)
        self.max_jpeg_size = 200_000 # Max expected JPEG size (200KB) for corruption detection

        # Extract remaining parameters for camera monitoring and error handling
        self.watchdog_grace_sec = float(self.get_parameter('watchdog_grace_sec').value)              # Extra time before restart
        self.startup_no_frame_timeout = float(self.get_parameter('startup_no_frame_timeout').value)  # Startup timeout
        self.enable_stderr_logging = bool(self.get_parameter('enable_stderr_logging').value)         # Log GStreamer errors

        # Initialize threading and process tracking variables
        self._stderr_thread = None                     # Background thread for reading GStreamer error messages
        self.pipeline_start_time = None                # When GStreamer pipeline was started (for startup timeout)

        # Start the camera streaming pipeline - this launches GStreamer and begins frame capture
        self.start_camera()

        # Log successful initialization with current settings
        self.get_logger().info(f'RPiCam Publisher started - {self.width}x{self.height}@{self.fps}fps')
        self.get_logger().info(f'Publishing JPEG compressed images only (quality={self.jpeg_quality})')
    
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
    
    # (Raw image path removed to reduce CPU; only compressed is published.)
    
    def start_camera(self):
        """Start GStreamer libcamera pipeline for continuous streaming"""
        try:
            if not self.check_gstreamer():
                self.get_logger().error("GStreamer or libcamera plugin not available")
                return

            # Formulate the GStreamer pipeline command
            # Example command: gst-launch-1.0 -q libcamerasrc ! video/x-raw,width=800,height=600,framerate=30/1 ! videoconvert ! jpegenc quality=75 ! fdsink fd=1
            #
            # GStreamer Pipeline Concept:
            # GStreamer works like a factory assembly line - data flows through connected processing elements
            # The '!' symbol connects elements together, creating a pipeline where each element transforms the data
            # Format: source ! processor1 ! processor2 ! sink
            # 
            # Example pipeline breakdown:
            # libcamerasrc              → Camera hardware (SOURCE: produces raw video frames)
            #     ↓ (via !)
            # video/x-raw,width=800...  → Format specification (tells next element what to expect)
            #     ↓ (via !)  
            # videoconvert              → Color format converter (FILTER: transforms video format)
            #     ↓ (via !)
            # jpegenc quality=75        → JPEG encoder (FILTER: compresses raw video to JPEG)
            #     ↓ (via !)
            # fdsink fd=1               → File descriptor sink (SINK: outputs data to stdout)
            #
            # Data flow: Raw Video → Format Conversion → JPEG Compression → Output Stream
            # Think of '!' as pipes in Unix: camera | convert | compress | output
            cmd = [
                'gst-launch-1.0', '-q',           # Launch GStreamer pipeline quietly
                'libcamerasrc',                   # RPi camera source (uses libcamera)
                '!', f'video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1',  # Set resolution/FPS
                '!', 'videoconvert',              # Convert color formats if needed (to ensure it's compatible with downstream elements, like jpegenc)
                '!', 'jpegenc', f'quality={self.jpeg_quality}',  # Encode to JPEG with specified quality
                '!', 'fdsink', 'fd=1'             # Output to stdout (file descriptor 1) so it can later be read in the capture_loop by the python process
            ]
            
            self.get_logger().info(f"Starting GStreamer pipeline: {' '.join(cmd)}")
            
            # Start the process
            self.gst_process = subprocess.Popen(
                cmd,                               # GStreamer pipeline command
                stdout=subprocess.PIPE,            # Capture JPEG stream
                stderr=subprocess.PIPE if self.enable_stderr_logging else subprocess.DEVNULL,  # Log errors if enabled
                bufsize=0,                         # No buffering (real-time)
                preexec_fn=os.setsid               # Create new process group for clean termination
            )
            self.pipeline_start_time = time.monotonic()

            # Error Monitoring Thread
            # Start stderr reader thread (non-blocking) if enabled
            # This captures and logs GStreamer error messages without blocking the main thread
            if self.enable_stderr_logging and self.gst_process.stderr:
                def _drain_stderr():
                    for raw_line in iter(self.gst_process.stderr.readline, b''):
                        try:
                            line = raw_line.decode('utf-8', 'replace').strip()
                        except Exception:
                            continue
                        if not line:
                            continue
                        lower = line.lower()
                        if 'error' in lower:
                            self.get_logger().error(f"[gst] {line}")
                        elif 'warn' in lower:
                            self.get_logger().warning(f"[gst] {line}")
                        elif 'libcamera' in lower:
                            self.get_logger().info(f"[gst] {line}")
                self._stderr_thread = Thread(target=_drain_stderr, daemon=True)
                self._stderr_thread.start()
            
            # Check if process started successfully
            time.sleep(2.0)  # allow pipeline negotiation
            if self.gst_process.poll() is not None:
                self.get_logger().error("GStreamer failed to start (exited early)")
                return

            # Start capture thread (implemented by the capture_loop method)
            self.capture_thread = Thread(target=self.capture_loop)
            self.capture_thread.daemon = True # daemon thread is a background thread that runs without preventing the main program from exiting
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
    
    def restart_camera(self):
        """Restart the camera pipeline"""
        try:
            # Stop current process
            if self.gst_process:
                try:
                    os.killpg(os.getpgid(self.gst_process.pid), signal.SIGTERM)
                    self.gst_process.wait(timeout=3)
                except:
                    try:
                        os.killpg(os.getpgid(self.gst_process.pid), signal.SIGKILL)
                    except:
                        pass
                self.gst_process = None
            
            # Give camera hardware time to reset
            time.sleep(3.0)
            
            # Restart camera
            self.start_camera()
            
        except Exception as e:
            self.get_logger().error(f"Error restarting camera: {str(e)}")
    
    def capture_loop(self):
        """Main capture loop - reads MJPEG stream from GStreamer and extracts individual JPEG frames"""
        buffer = bytearray()  # Accumulating buffer for incoming binary data from GStreamer
        
        self.get_logger().info("Starting GStreamer capture loop...")
        
        # Main processing loop - continues until shutdown is requested
        while self.running and rclpy.ok() and self.gst_process and not shutdown_requested:
            try:
                # === PROCESS HEALTH CHECK ===
                # Check if GStreamer subprocess is still alive
                if self.gst_process.poll() is not None:  # poll() returns exit code if process died, None if still running
                    _, stderr = self.gst_process.communicate()  # Get any error messages
                    self.get_logger().error(f"GStreamer process terminated: {stderr.decode()}")
                    break
                
                # === READ RAW DATA FROM GSTREAMER ===
                # Read binary data chunks from GStreamer's stdout (where JPEG stream flows)
                chunk = self.gst_process.stdout.read(4096)  # Read up to 4KB at a time (non-blocking)
                if not chunk:  # No data available right now
                    time.sleep(0.05)  # Brief pause to avoid busy-waiting (CPU spinning)
                    continue
                
                buffer.extend(chunk)  # Append new data to our accumulating buffer

                # === BUFFER OVERFLOW PROTECTION ===
                # If buffer grows too large (3x max JPEG size = 600KB), it's likely corrupted
                # Keep only the most recent potential frame start to recover from corruption
                if len(buffer) > self.max_jpeg_size * 3:  # 600KB threshold
                    self.get_logger().warning('Buffer oversized, pruning to last potential frame start')
                    last_start = buffer.rfind(b'\xff\xd8')  # Find last JPEG start marker (backwards search)
                    if last_start != -1:
                        buffer = buffer[last_start:]  # Keep only data from last frame start
                    else:
                        buffer.clear()  # No valid frame start found, clear everything
                
                # === JPEG FRAME EXTRACTION ===
                # JPEG files have specific byte markers: 0xFFD8 (start) and 0xFFD9 (end)
                # We need to find complete frames (start + content + end) in the stream
                while True:  # Process all complete frames in current buffer
                    # Look for JPEG start marker (SOI = Start of Image)
                    start = buffer.find(b'\xff\xd8')  # 0xFFD8 in binary
                    if start == -1:  # No frame start found
                        break  # Wait for more data
                    
                    # Look for JPEG end marker (EOI = End of Image) after the start
                    end = buffer.find(b'\xff\xd9', start)  # 0xFFD9 in binary, search after start position
                    if end == -1:  # Frame not complete yet
                        break  # Wait for more data to complete the frame
                    
                    # === COMPLETE FRAME FOUND ===
                    # Extract the complete JPEG frame (including both markers)
                    jpeg_data = bytes(buffer[start:end+2])  # +2 to include the end marker (2 bytes)
                    del buffer[:end+2]  # Remove processed data from buffer (keep any remaining data)
                    
                    # === BASIC CORRUPTION CHECK ===
                    # Skip frames that are obviously corrupt (too small/large)
                    if 1000 < len(jpeg_data) < self.max_jpeg_size:  # Between 1KB and 200KB
                        self.process_frame(jpeg_data)  # Send valid frame to ROS2 publishing

                # === WATCHDOG SYSTEM ===
                # Automatic restart if camera stops producing frames
                now_mono = time.monotonic()  # Current timestamp (monotonic = doesn't go backwards)
                
                # STARTUP TIMEOUT: No frames received since pipeline started
                if self.frames_seen == 0:  # Never received any frames
                    if self.pipeline_start_time and (now_mono - self.pipeline_start_time) > self.startup_no_frame_timeout:
                        self.get_logger().error('Startup timeout: no frames received, restarting pipeline')
                        self.restart_camera()  # Pipeline failed to produce frames, restart it
                        return
                else:
                    # RUNTIME WATCHDOG: Had frames before, but stopped getting them
                    time_since_last_frame = now_mono - self.last_frame_time
                    timeout_threshold = self.watchdog_interval + self.watchdog_grace_sec  # 3.0 + 6.0 = 9 seconds
                    if time_since_last_frame > timeout_threshold:
                        self.get_logger().error('Watchdog: frame gap exceeded, restarting pipeline')
                        self.restart_camera()  # Camera stopped working, restart it
                        return
                        
            except Exception as e:
                # Any unexpected error in the loop
                if self.running:  # Only log if we're not shutting down
                    self.get_logger().error(f"Capture loop error: {e}; restarting pipeline")
                    self.restart_camera()  # Try to recover by restarting
                return
        
        self.get_logger().info("Capture loop ended")  # Normal shutdown
    
    def process_frame(self, jpeg_data):
        """Publish a JPEG frame as CompressedImage (no decode)."""
        try:
            now = self.get_clock().now().to_msg()

            # Publish camera info (timestamped)
            self.camera_info_msg.header.stamp = now
            self.camera_info_pub.publish(self.camera_info_msg)

            msg = CompressedImage()
            msg.header.stamp = now
            msg.header.frame_id = "camera_link"
            msg.format = "jpeg"
            msg.data = jpeg_data
            self.compressed_pub.publish(msg)

            self.frame_count += 1
            self.frames_seen += 1
            self.last_frame_time = time.monotonic()
            current_time = self.last_frame_time
            if current_time - self.last_log_time > 5.0:
                actual_fps = self.frame_count / (current_time - self.last_log_time)
                self.get_logger().info(f"Compressed stream: {actual_fps:.1f} fps (target {self.fps})")
                self.frame_count = 0
                self.last_log_time = current_time
        except Exception as e:
            self.get_logger().error(f"Frame publish error: {e}")
    
    def destroy_node(self):
        """Clean up resources"""
        try:
            self.get_logger().info("Shutting down camera publisher...")
        except:
            print("Shutting down camera publisher...")
        
        self.running = False
        
        # Stop GStreamer process
        if self.gst_process:
            try:
                # Send SIGTERM to process group
                os.killpg(os.getpgid(self.gst_process.pid), signal.SIGTERM)
                self.gst_process.wait(timeout=3)
            except:
                try:
                    # Force kill if needed
                    os.killpg(os.getpgid(self.gst_process.pid), signal.SIGKILL)
                    self.gst_process.wait(timeout=1)
                except:
                    pass
            self.gst_process = None
        
        # Wait for capture thread to finish
        if self.capture_thread:
            self.capture_thread.join(timeout=3.0)
        
        try:
            super().destroy_node()
        except Exception as e:
            print(f"Warning: Error in parent destroy_node: {e}")

def main(args=None):
    # Set up signal handlers for graceful shutdown
    import signal
    import sys
    
    def signal_handler(signum, frame):
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        global shutdown_requested
        shutdown_requested = True
    
    global shutdown_requested
    shutdown_requested = False
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    publisher = None
    rclpy_initialized = False
    
    try:
        rclpy.init(args=args)
        rclpy_initialized = True
        
        publisher = RPiCamPublisher()
        
        # Custom spin loop with shutdown check
        while rclpy.ok() and not shutdown_requested:
            try:
                rclpy.spin_once(publisher, timeout_sec=0.1)
            except KeyboardInterrupt:
                break
        
    except KeyboardInterrupt:
        print("Keyboard interrupt received...")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        print("Cleaning up...")
        if publisher:
            try:
                publisher.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
        
        if rclpy_initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"Error shutting down rclpy: {e}")
        
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
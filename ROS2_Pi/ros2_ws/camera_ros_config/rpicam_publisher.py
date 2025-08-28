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

# Global shutdown flag
shutdown_requested = False

class RPiCamPublisher(Node):
    def __init__(self):
        super().__init__('rpicam_publisher')
        # Parameters (keep legacy ones for compatibility, even if unused now)
        self.declare_parameters('', [
            ('width', 800),
            ('height', 600),
            ('fps', 10),
            ('format', 'rgb'),              # unused (compressed only)
            ('publish_compressed', True),    # must remain True; raw disabled
            ('jpeg_quality', 80),
            ('watchdog_grace_sec', 6.0),     # post-first-frame grace for gaps
            ('startup_no_frame_timeout', 12.0),  # restart if no frame at all in this many sec
            ('enable_stderr_logging', True)
        ])

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = max(1, int(self.get_parameter('fps').value))  # clamp
        self.jpeg_quality = int(min(100, max(1, self.get_parameter('jpeg_quality').value)))

        # QoS: depth=1 to drop old frames instead of queueing
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers (compressed only + camera info)
        self.compressed_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', sensor_qos)

        # Camera info message
        self.camera_info_msg = self.create_camera_info()

        # Initialize camera streaming state
        self.running = True
        self.gst_process = None
        self.capture_thread = None
        self.frame_count = 0
        self.frames_seen = 0
        self.last_log_time = time.monotonic()
        self.last_frame_time = time.monotonic()
        self.watchdog_interval = max(3.0, 2.5 / self.fps)
        self.max_jpeg_size = 200_000
        self.watchdog_grace_sec = float(self.get_parameter('watchdog_grace_sec').value)
        self.startup_no_frame_timeout = float(self.get_parameter('startup_no_frame_timeout').value)
        self.enable_stderr_logging = bool(self.get_parameter('enable_stderr_logging').value)

        self._stderr_thread = None
        self.pipeline_start_time = None  # set after spawning gst

        # Start camera streaming
        self.start_camera()

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
            # Check if gstreamer is available
            if not self.check_gstreamer():
                self.get_logger().error("GStreamer or libcamera plugin not available")
                return
            
            # Use a simpler, more stable pipeline
            cmd = [
                'gst-launch-1.0', '-q',
                'libcamerasrc',
                '!', f'video/x-raw,width={self.width},height={self.height},framerate={self.fps}/1',
                '!', 'videoconvert',
                '!', 'jpegenc', f'quality={self.jpeg_quality}',
                '!', 'fdsink', 'fd=1'
            ]
            
            self.get_logger().info(f"Starting GStreamer pipeline: {' '.join(cmd)}")
            
            # Start the process
            self.gst_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE if self.enable_stderr_logging else subprocess.DEVNULL,
                bufsize=0,
                preexec_fn=os.setsid,
                text=True
            )
            self.pipeline_start_time = time.monotonic()

            # Start stderr reader thread (non-blocking) if enabled
            if self.enable_stderr_logging and self.gst_process.stderr:
                def _drain_stderr():
                    for line in self.gst_process.stderr:
                        line = line.strip()
                        if not line:
                            continue
                        # Log negotiation / error lines at warn/error levels heuristically
                        if 'ERROR' in line or 'error' in line.lower():
                            self.get_logger().error(f"[gst] {line}")
                        elif 'WARN' in line or 'warning' in line.lower():
                            self.get_logger().warning(f"[gst] {line}")
                        elif 'libcamera' in line.lower():
                            self.get_logger().info(f"[gst] {line}")
                self._stderr_thread = Thread(target=_drain_stderr, daemon=True)
                self._stderr_thread.start()
            
            # Check if process started successfully
            time.sleep(2.0)  # allow pipeline negotiation
            if self.gst_process.poll() is not None:
                self.get_logger().error("GStreamer failed to start (exited early)")
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
        """Main capture loop - reads MJPEG stream from GStreamer"""
        buffer = bytearray()
        
        self.get_logger().info("Starting GStreamer capture loop...")
        
        while self.running and rclpy.ok() and self.gst_process and not shutdown_requested:
            try:
                # Check if process is still running
                if self.gst_process.poll() is not None:
                    _, stderr = self.gst_process.communicate()
                    self.get_logger().error(f"GStreamer process terminated: {stderr.decode()}")
                    break
                
                # Read data from GStreamer stdout
                chunk = self.gst_process.stdout.read(4096)
                if not chunk:
                    time.sleep(0.05)
                    continue
                
                buffer.extend(chunk)

                # Guard against pathological growth (corruption); keep last sync marker region only
                if len(buffer) > self.max_jpeg_size * 3:
                    self.get_logger().warning('Buffer oversized, pruning to last potential frame start')
                    last_start = buffer.rfind(b'\xff\xd8')
                    if last_start != -1:
                        buffer = buffer[last_start:]
                    else:
                        buffer.clear()
                
                # Look for JPEG frame boundaries (0xFFD8 = start, 0xFFD9 = end)
                while True:
                    start = buffer.find(b'\xff\xd8')
                    if start == -1:
                        break
                    
                    end = buffer.find(b'\xff\xd9', start)
                    if end == -1:
                        break
                    
                    # Extract complete JPEG frame
                    jpeg_data = bytes(buffer[start:end+2])  # copy out
                    del buffer[:end+2]
                    
                    # Skip obviously corrupt frames
                    if 1000 < len(jpeg_data) < self.max_jpeg_size:
                        self.process_frame(jpeg_data)

                # Watchdog: restart if no frame recently
                now_mono = time.monotonic()
                # Startup: no frames ever seen
                if self.frames_seen == 0:
                    if self.pipeline_start_time and (now_mono - self.pipeline_start_time) > self.startup_no_frame_timeout:
                        self.get_logger().error('Startup timeout: no frames received, restarting pipeline')
                        self.restart_camera()
                        return
                else:
                    # Post-first-frame watchdog
                    if (now_mono - self.last_frame_time) > (self.watchdog_interval + self.watchdog_grace_sec):
                        self.get_logger().error('Watchdog: frame gap exceeded, restarting pipeline')
                        self.restart_camera()
                        return
                    
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Capture loop error: {str(e)}")
                break
        
        self.get_logger().info("Capture loop ended")
    
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
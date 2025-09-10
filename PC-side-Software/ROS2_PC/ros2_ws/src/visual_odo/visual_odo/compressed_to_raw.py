#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class CompressedToRaw(Node):
    def __init__(self):
        super().__init__('compressed_to_raw')
        self.bridge = CvBridge()
        
        # QoS Profile matching camera publisher (BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.compressed_callback,
            sensor_qos)
        
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            sensor_qos)
        
        self.get_logger().info('Compressed to Raw converter started')
    
    def compressed_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Convert to ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            image_msg.header = msg.header
            # Use base_link frame for now to avoid transform issues in RViz
            image_msg.header.frame_id = "base_link"
            
            self.publisher.publish(image_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedToRaw()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down compressed_to_raw...')
    finally:
        # Clean shutdown
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Already shutdown

if __name__ == '__main__':
    main()

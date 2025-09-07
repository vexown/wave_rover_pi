#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

class CompressedToRawRepublisher(Node):
    def __init__(self):
        super().__init__('compressed_to_raw_republisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._bridge = CvBridge()
        self._sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.cb_compressed,
            qos)
        self._pub = self.create_publisher(Image, '/camera/image_raw', qos)

    def cb_compressed(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR8
            if cv_img is None:
                return
            img_msg = self._bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            img_msg.header = msg.header
            self._pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"republish error: {e}")

def main():
    rclpy.init()
    node = CompressedToRawRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
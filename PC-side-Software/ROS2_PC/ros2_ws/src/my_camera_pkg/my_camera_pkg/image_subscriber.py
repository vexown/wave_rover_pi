import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge  # Important for converting ROS Image messages to OpenCV images

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Or the topic your camera is publishing to
            self.image_callback,
            10
        )
        self.bridge = CvBridge()  # Create a CvBridge object

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  #  encoding
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Display the image using OpenCV
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)  #  needed to update the display.

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()

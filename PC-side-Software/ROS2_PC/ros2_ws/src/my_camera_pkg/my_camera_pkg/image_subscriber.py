import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('CompressedImageSubscriber initialized and subscribed to /camera/image_raw/compressed')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.get_logger().debug('Received and decoded compressed image')
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return

        cv2.imshow('Camera Feed', cv_image)
        # Allow graceful shutdown with 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('\'q\' pressed, shutting down...')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = CompressedImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        image_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

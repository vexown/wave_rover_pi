import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        self.get_logger().info('CompressedImageSubscriber initialized and subscribed to /camera/image_raw/compressed')
        self._running = True

    def image_callback(self, msg):
        if not self._running: # Check if shutdown has been initiated
            return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Rotate image 180 degrees to flip it upside down
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            self.get_logger().debug('Received and decoded compressed image')
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return

        cv2.imshow('Camera Feed', cv_image)
        # Allow graceful shutdown with 'q' key
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('\'q\' pressed, initiating shutdown...')
            self._running = False # Set flag to stop processing
            rclpy.shutdown() # Initiate ROS shutdown

    def is_running(self):
        return self._running

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = CompressedImageSubscriber()
    try:
        while rclpy.ok() and image_subscriber.is_running():
            rclpy.spin_once(image_subscriber, timeout_sec=0.1) # Process callbacks
    except KeyboardInterrupt:
        image_subscriber.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        if image_subscriber.is_running(): # Ensure shutdown is called if not already
            image_subscriber.get_logger().info('Exiting main loop, ensuring shutdown...')
        # rclpy.shutdown() might have already been called by 'q' press or KeyboardInterrupt
        # It's safe to call again, but we can also check rclpy.ok()
        if rclpy.ok(): # If ROS is still ok, means shutdown wasn't called by 'q' or Ctrl+C in a way that stopped it
             rclpy.shutdown()
        image_subscriber.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
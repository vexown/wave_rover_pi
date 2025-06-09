#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import re

class GpsParserNode(Node):
    """
    A ROS2 node that subscribes to raw UART data, parses GPS information,
    and republishes it as a sensor_msgs/msg/NavSatFix message.
    """
    def __init__(self):
        super().__init__('gps_parser_node')
        self.get_logger().info('GPS Parser Node has been started.')

        # Create a subscriber to the /uart_data topic
        self.subscription = self.create_subscription(
            String,
            '/uart_data',
            self.listener_callback,
            10)

        # Create a publisher for the /gps/fix topic
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # A regular expression to find key-value pairs in the data string
        # This is more robust than splitting by comma, as it finds specific keys.
        self.regex = re.compile(r'(\w+):([-\d\.]+)')

    def listener_callback(self, msg):
        """
        Callback function for the /uart_data subscriber.
        """
        raw_data = msg.data

        # Check if the message contains the expected navigation data prefix
        if 'NaviLogging Task: New coordinates received' not in raw_data:
            # self.get_logger().info(f"Ignoring non-GPS data: '{raw_data}'")
            return

        try:
            # Use the regex to find all key-value pairs
            matches = self.regex.findall(raw_data)
            parsed_data = dict(matches)

            # Create a NavSatFix message
            fix_msg = NavSatFix()

            # Populate the message header
            fix_msg.header.stamp = self.get_clock().now().to_msg()
            fix_msg.header.frame_id = 'gps_link' # A common frame ID for GPS sensors

            # Populate the GPS data from the parsed dictionary
            fix_msg.latitude = float(parsed_data.get('lat', 0.0))
            fix_msg.longitude = float(parsed_data.get('lon', 0.0))
            fix_msg.altitude = float(parsed_data.get('alt', 0.0))

            # Populate the status
            fix_status = int(parsed_data.get('fix', 0))
            if fix_status > 0:
                fix_msg.status.status = fix_msg.status.STATUS_FIX
            else:
                fix_msg.status.status = fix_msg.status.STATUS_NO_FIX
            
            # This indicates we are getting a 2D/3D fix
            fix_msg.status.service = fix_msg.status.SERVICE_GPS

            # Position covariance is unknown, so set to a default
            # A large value indicates high uncertainty
            fix_msg.position_covariance[0] = 1.0 # Variance of latitude
            fix_msg.position_covariance[4] = 1.0 # Variance of longitude
            fix_msg.position_covariance[8] = 4.0 # Variance of altitude
            fix_msg.position_covariance_type = fix_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN

            # Publish the message
            self.publisher_.publish(fix_msg)
            self.get_logger().info(f"Published NavSatFix: Lat={fix_msg.latitude}, Lon={fix_msg.longitude}")

        except (ValueError, KeyError) as e:
            self.get_logger().error(f"Failed to parse GPS data: '{raw_data}'. Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    gps_parser_node = GpsParserNode()
    rclpy.spin(gps_parser_node)
    gps_parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

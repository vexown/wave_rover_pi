#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import re
import math

class ImuParserNode(Node):
    """
    A ROS2 node that subscribes to raw UART data, parses IMU information,
    and republishes it as a sensor_msgs/msg/Imu message.
    
    This node specifically looks for IMU data in the format:
    'IMU: Roll: X.XX, Pitch: Y.YY, Yaw: Z.ZZ'
    
    The Roll/Pitch/Yaw (Euler angles) are converted to quaternion format
    as required by the standard ROS2 sensor_msgs/Imu message.
    """
    
    def __init__(self):
        super().__init__('imu_parser_node')
        self.get_logger().info('IMU Parser Node has been started.')

        # Create a subscriber to the /uart_data topic
        self.subscription = self.create_subscription(
            String,
            '/uart_data',
            self.listener_callback,
            10)

        # Create a publisher for the /imu/data topic
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)

        # Regular expression to extract Roll, Pitch, Yaw values from IMU data
        # Matches format: 'IMU: Roll: X.XX, Pitch: Y.YY, Yaw: Z.ZZ'
        self.imu_regex = re.compile(
            r'IMU:\s*Roll:\s*([-\d\.]+),\s*Pitch:\s*([-\d\.]+),\s*Yaw:\s*([-\d\.]+)'
        )

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion.
        
        Args:
            roll (float): Roll angle in degrees
            pitch (float): Pitch angle in degrees  
            yaw (float): Yaw angle in degrees
            
        Returns:
            tuple: (qx, qy, qz, qw) quaternion components
        """
        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Calculate quaternion components
        # Using the standard aerospace sequence: Roll (X), Pitch (Y), Yaw (Z)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def listener_callback(self, msg):
        """
        Callback function for the /uart_data subscriber.
        Parses IMU data and publishes sensor_msgs/Imu messages.
        """
        raw_data = msg.data

        # Check if the message contains IMU data
        if 'IMU:' not in raw_data:
            # Not IMU data, ignore silently
            return

        try:
            # Use regex to extract Roll, Pitch, Yaw values
            match = self.imu_regex.search(raw_data)
            
            if not match:
                self.get_logger().debug(f"IMU data format not recognized: '{raw_data}'")
                return

            # Extract the values
            roll = float(match.group(1))
            pitch = float(match.group(2))
            yaw = float(match.group(3))

            # Convert Euler angles to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            # Create an Imu message
            imu_msg = Imu()

            # Populate the message header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'  # Standard frame ID for IMU sensors

            # Set orientation (quaternion)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            # Set orientation covariance
            # Since we're getting orientation data, we set a reasonable covariance
            # The covariance matrix is 3x3, but stored as a 9-element array (row-major)
            # We set diagonal elements to indicate uncertainty in each axis
            imu_msg.orientation_covariance[0] = 0.01  # Roll variance (rad²)
            imu_msg.orientation_covariance[4] = 0.01  # Pitch variance (rad²)  
            imu_msg.orientation_covariance[8] = 0.01  # Yaw variance (rad²)

            # Angular velocity - not provided by ESP32, so we mark as unknown
            # Set all values to 0 and covariance to -1 to indicate no data
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            imu_msg.angular_velocity_covariance[0] = -1  # -1 indicates no data

            # Linear acceleration - not provided by ESP32, so we mark as unknown
            # Set all values to 0 and covariance to -1 to indicate no data
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            imu_msg.linear_acceleration_covariance[0] = -1  # -1 indicates no data

            # Publish the message
            self.publisher_.publish(imu_msg)
            self.get_logger().info(
                f"Published IMU data: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}° "
                f"(Quaternion: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f})"
            )

        except (ValueError, AttributeError) as e:
            self.get_logger().error(f"Failed to parse IMU data: '{raw_data}'. Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_parser_node = ImuParserNode()
    rclpy.spin(imu_parser_node)
    imu_parser_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

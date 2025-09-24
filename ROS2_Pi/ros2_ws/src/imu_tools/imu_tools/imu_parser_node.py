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

        # Regular expression to extract all IMU data (Roll, Pitch, Yaw, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ)
        # Matches format: 'IMU: Roll: X.XX, Pitch: Y.YY, Yaw: Z.ZZ, GyroX: A.AA, GyroY: B.BB, GyroZ: C.CC, AccelX: D.DD, AccelY: E.EE, AccelZ: F.FF'
        self.imu_regex = re.compile(
            r'IMU:\s*Roll:\s*([-\d\.]+),\s*Pitch:\s*([-\d\.]+),\s*Yaw:\s*([-\d\.]+),'
            r'\s*GyroX:\s*([-\d\.]+),\s*GyroY:\s*([-\d\.]+),\s*GyroZ:\s*([-\d\.]+),'
            r'\s*AccelX:\s*([-\d\.]+),\s*AccelY:\s*([-\d\.]+),\s*AccelZ:\s*([-\d\.]+)'
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
            # Use regex to extract all IMU values (Roll, Pitch, Yaw, GyroX, GyroY, GyroZ, AccelX, AccelY, AccelZ)
            match = self.imu_regex.search(raw_data)
            
            if not match:
                self.get_logger().debug(f"IMU data format not recognized: '{raw_data}'")
                return

            # Extract all the values from the regex groups
            roll = float(match.group(1))    # Roll angle (degrees)
            pitch = float(match.group(2))   # Pitch angle (degrees)
            yaw = float(match.group(3))     # Yaw angle (degrees)
            gyro_x = float(match.group(4))  # Angular velocity around X-axis (degrees/s)
            gyro_y = float(match.group(5))  # Angular velocity around Y-axis (degrees/s)
            gyro_z = float(match.group(6))  # Angular velocity around Z-axis (degrees/s)
            accel_x = float(match.group(7)) # Linear acceleration along X-axis (m/s²)
            accel_y = float(match.group(8)) # Linear acceleration along Y-axis (m/s²)
            accel_z = float(match.group(9)) # Linear acceleration along Z-axis (m/s²)

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

            # Angular velocity - now provided by ESP32 gyroscope data
            # Convert from degrees/second to radians/second (ROS2 standard)
            imu_msg.angular_velocity.x = math.radians(gyro_x)  # Roll rate (rad/s)
            imu_msg.angular_velocity.y = math.radians(gyro_y)  # Pitch rate (rad/s)
            imu_msg.angular_velocity.z = math.radians(gyro_z)  # Yaw rate (rad/s)
            
            # Set angular velocity covariance (reasonable estimates for typical IMU)
            imu_msg.angular_velocity_covariance[0] = 0.01  # GyroX variance (rad²/s²)
            imu_msg.angular_velocity_covariance[4] = 0.01  # GyroY variance (rad²/s²)
            imu_msg.angular_velocity_covariance[8] = 0.01  # GyroZ variance (rad²/s²)

            # Linear acceleration - now provided by ESP32 accelerometer data
            # Assuming the ESP32 provides acceleration in m/s² (if in g-force, multiply by 9.81)
            imu_msg.linear_acceleration.x = accel_x  # Forward/backward acceleration (m/s²)
            imu_msg.linear_acceleration.y = accel_y  # Left/right acceleration (m/s²)
            imu_msg.linear_acceleration.z = accel_z  # Up/down acceleration (m/s²)
            
            # Set linear acceleration covariance (reasonable estimates for typical IMU)
            imu_msg.linear_acceleration_covariance[0] = 0.1  # AccelX variance (m²/s⁴)
            imu_msg.linear_acceleration_covariance[4] = 0.1  # AccelY variance (m²/s⁴)
            imu_msg.linear_acceleration_covariance[8] = 0.1  # AccelZ variance (m²/s⁴)

            # Publish the message
            self.publisher_.publish(imu_msg)
            self.get_logger().info(
                f"Published full IMU data: "
                f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}° | "
                f"GyroX={gyro_x:.2f}°/s, GyroY={gyro_y:.2f}°/s, GyroZ={gyro_z:.2f}°/s | "
                f"AccelX={accel_x:.2f}m/s², AccelY={accel_y:.2f}m/s², AccelZ={accel_z:.2f}m/s²"
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

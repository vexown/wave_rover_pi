import rclpy
from rclpy.node import Node
import serial
import time

class SerialComNode(Node):
    def __init__(self):
        super().__init__('serial_com_node')
        self.serial_port = None
        self.baud_rate = 115200  # Adjust to your ESP32's baud rate
        self.serial_device = '/dev/ttyS0'  # Or /dev/ttyAMA0, check your Pi's configuration

        try:
            self.serial_port = serial.Serial(self.serial_device, self.baud_rate, timeout=1)
            self.get_logger().info(f'Successfully opened serial port {self.serial_device} at {self.baud_rate} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {self.serial_device}: {e}')
            self.destroy_node()
            return

        self.timer = self.create_timer(0.1, self.read_serial)  # Check for incoming data every 100ms

    def read_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    received_data = self.serial_port.readline().decode('utf-8').strip()
                    if received_data == "Hello from ESP32!":
                        self.get_logger().info(f'Received: "{received_data}"')
                        response_message = "Hello ROS 2 Pi received your message!\n"
                        self.serial_port.write(response_message.encode('utf-8'))
                        self.get_logger().info(f'Sent: "{response_message.strip()}"')
                    elif received_data:
                        self.get_logger().info(f'Received unknown data: "{received_data}"')
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading/writing from serial port: {e}')
            except UnicodeDecodeError as e:
                self.get_logger().warn(f'Could not decode received data: {e}')

def main(args=None):
    rclpy.init(args=args)
    serial_com_node = SerialComNode()
    rclpy.spin(serial_com_node)
    serial_com_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

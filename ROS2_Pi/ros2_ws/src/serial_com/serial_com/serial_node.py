# Important:
#
# To ensure proper serial communication with this ROS 2 node,
# the Raspberry Pi on which it is running must be configured correctly.
#
# Configuration Steps:
#
# 1. Edit /boot/firmware/config.txt:
#    - Ensure the line 'enable_uart=1' is present (uncomment it if necessary).
#    - (optional?) Ensure the line 'dtoverlay=disable-bt' is present to disable Bluetooth
#      and make the primary UART (/dev/ttyS0 or /dev/ttyAMA0) available on the GPIO pins.
#
# 2. Edit /boot/firmware/cmdline.txt:
#    - Remove any instances of 'console=serial0,115200' or 'console=ttyS0,115200'
#      to prevent the Raspberry Pi from using the UART for system console output,
#      which will interfere with serial communication.
#
# Failure to configure these files correctly will result in interference
# with the serial communication and the node may not function as expected.

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
                    if received_data: # Log any received data
                        self.get_logger().info(f'Received: "{received_data}"')
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading from serial port: {e}')
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


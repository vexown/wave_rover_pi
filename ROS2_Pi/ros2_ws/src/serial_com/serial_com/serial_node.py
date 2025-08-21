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
from std_msgs.msg import String

class SerialComNode(Node): # Our SerialComNode class inherits from Node (the base class for ROS2 nodes)
    """
    __init__ is Python's constructor method - a special method that automatically runs
    when you create a new instance of a class. It initializes the object's attributes
    and sets up the initial state. In this case, it sets up the ROS2 node and serial communication.
    """
    def __init__(self):
        # Initialize the parent ROS2 Node class with our node name
        super().__init__('serial_com_node') # Super() calls the parent class's __init__ method
        
        # Initialize serial communication attributes
        self.serial_port = None # This will hold our serial port object once opened
        self.baud_rate = 921600  # Communication speed - must match ESP32 configuration
        self.serial_device = '/dev/ttyS0'  # Raspberry Pi serial device path
        
        # Create publisher for broadcasting received UART messages
        # Other nodes can subscribe to this topic to receive the data
        self.uart_publisher = self.create_publisher(
            String,           # Message type
            'uart_data',      # Topic name
            10               # Queue size (buffer 10 messages)
        )
        
        # Attempt to establish serial connection
        self._initialize_serial_connection()
        
        # Set up periodic data reading (every 100ms)
        self.timer = self.create_timer(0.1, self.read_serial)

    def _initialize_serial_connection(self):
        """
        Private method to handle serial port initialization with error handling
        """
        try:
            # Open serial port with specified parameters
            self.serial_port = serial.Serial(
                port=self.serial_device,
                baudrate=self.baud_rate,
                timeout=1  # 1 second timeout for read operations
            )
            
            # Log successful connection
            self.get_logger().info(
                f'Successfully opened serial port {self.serial_device} at {self.baud_rate} bps'
            )
            
        except serial.SerialException as e:
            # Handle serial port errors (device not found, permission issues, etc.)
            self.get_logger().error(
                f'Could not open serial port {self.serial_device}: {e}'
            )
            
            # Clean shutdown if serial connection fails
            self.destroy_node()
            return

    def _publish_uart_data(self, data):
        """
        Private method to publish received UART data to ROS2 topic
        
        Args:
            data (str): The received UART data to publish
        """
        # Create a String message object
        msg = String()
        msg.data = data
        
        # Publish the message to the 'uart_data' topic
        self.uart_publisher.publish(msg)
        
        # Log that we published the data
        self.get_logger().debug(f'Published UART data: "{data}"')

    def read_serial(self):
        """
        Timer callback method - reads data from serial port every 100ms
        This method is called automatically by the ROS2 timer
        """
        # Check if serial port exists and is open before attempting to read
        if self.serial_port and self.serial_port.is_open:
            try:
                # Check if there's data waiting in the serial buffer
                if self.serial_port.in_waiting > 0:
                    # Read a complete line from serial port
                    received_data = self.serial_port.readline().decode('utf-8').strip()
                    
                    # Only process if we actually received some data
                    if received_data:
                        # Log the received data locally
                        self.get_logger().info(f'Received: "{received_data}"')
                        
                        # Publish the data to other ROS2 nodes
                        self._publish_uart_data(received_data)
                        
            except serial.SerialException as e:
                # Handle hardware/connection errors with serial port
                self.get_logger().error(f'Error reading from serial port: {e}')
                
            except UnicodeDecodeError as e:
                # Handle cases where received data isn't valid UTF-8
                self.get_logger().warn(f'Could not decode received data: {e}')


def main(args=None):
    """
    Main entry point for the ROS2 serial communication node
    
    This function:
    1. Initializes the ROS2 system
    2. Creates and runs the SerialComNode
    3. Handles cleanup when the node shuts down
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of our serial communication node
    serial_com_node = SerialComNode()
    
    # Keep the node running and processing callbacks until interrupted
    # This is a blocking call that handles ROS2 message processing
    rclpy.spin(serial_com_node)
    
    # Clean up the node when shutting down
    serial_com_node.destroy_node()
    
    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    """
    Python script entry point
    This block only runs when the script is executed directly,
    not when imported as a module
    """
    main()

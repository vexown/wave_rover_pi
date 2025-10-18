#!/usr/bin/env python3

import os
import sys
import time
from periphery import Serial
import fcntl
import struct
import array

# --- Configuration ---
UART_PORT = "/dev/ttyS0" # Serial Port (UART) connected to the ESP32 when the RPi is inserted onto the waverover top-controller pin header. Allows comms between the two.
BAUD_RATE = 921600
CONTROLLER_DEV = "/dev/input/js0" # Use the joystick device
CONTROLLER_WAIT_TIMEOUT = None  # None = wait forever, or set number of seconds

# Data structure for current controller state
# Axes are -32767 to 32767 (full 16-bit resolution preserved for maximum precision).
controller_state = {
    'LX': 0, 'LY': 0, # Left Stick
    'RX': 0, 'RY': 0, # Right Stick
    'LT': 0, 'RT': 0, # Triggers 
    'DPX': 0, 'DPY': 0, # D-Pad
    'A': 0, 'B': 0, 'X': 0, 'Y': 0, # Face Buttons
    'LB': 0, 'RB': 0, 'BACK': 0, 'START': 0, 'GUIDE': 0, # Other Buttons
}

# Mapping of joystick index to controller_state key
# Axis indices: 0=LX, 1=LY, 2=LT, 3=RX, 4=RY, 5=RT, 6=DPX, 7=DPY (May vary based on xboxdrv config)
AXIS_MAP = {
    0: 'LX', 1: 'LY', 2: 'LT', 3: 'RX', 4: 'RY', 5: 'RT', 6: 'DPX', 7: 'DPY'
}

# Button indices: 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Back, 7=Start, 8=Guide (May vary)
BUTTON_MAP = {
    0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'LB', 5: 'RB', 6: 'BACK', 7: 'START', 8: 'GUIDE'
}

# ioctl constants for joystick device (from linux/joystick.h)
def JSIOCGNAME(length):
    """Get joystick device name - equivalent to JSIOCGNAME(len) from linux/joystick.h"""
    return 0x80006a00 + length


def format_and_send(uart):
    # Format the data into a single string: "S|LX:0|LY:0|RX:0|RY:0|...|E\n"
    data_string = "S|"
    data_string += "|".join(f"{k}:{v}" for k, v in controller_state.items())
    data_string += "|E\n"
    
    try:
        uart.write(data_string.encode('utf-8'))
        print(f"Sent: {data_string.strip()}") # Uncomment for debugging
    except Exception as e:
        print(f"Error writing to serial: {e}") # Uncomment for debugging
        pass
        
# Constants for joystick event reading
JS_EVENT_SIZE = 8
JS_EVENT_BUTTON = 0x01  # button pressed/released
JS_EVENT_AXIS = 0x02    # joystick moved
JS_EVENT_INIT = 0x80    # initial state of device

def is_controller_connected(device_path):
    """
    Check if a controller is actually connected (not just the receiver).
    Returns True if controller is powered on and responsive.
    """
    try:
        # Try to open the device
        test_fd = open(device_path, 'rb')
        
        # Try to get device name - this fails with errno 22 if no controller connected
        buf = array.array('B', [0] * 64)
        fcntl.ioctl(test_fd, JSIOCGNAME(len(buf)), buf)
        name = buf.tobytes().rstrip(b'\x00').decode('utf8', errors='ignore')
        
        test_fd.close()
        
        # If we got a valid name, controller is connected
        return len(name) > 0
        
    except OSError as e:
        if e.errno == 22:  # Device exists but no controller connected
            return False
        # Other errors - assume not connected
        return False
    except Exception:
        return False

def wait_for_controller(device_path, timeout=None):
    """
    Wait for controller to be powered on and responsive.
    Returns True if controller connects, False if timeout expires.
    """
    print(f"Waiting for active controller at {device_path}...")
    start_time = time.time()
    last_message_time = 0
    
    while True:
        # Show periodic status
        current_time = time.time()
        if current_time - last_message_time > 5:
            elapsed = int(current_time - start_time)
            print(f"Still waiting for controller... ({elapsed}s elapsed)")
            last_message_time = current_time
        
        # Check timeout
        if timeout is not None and (current_time - start_time) > timeout:
            print(f"Timeout waiting for controller after {timeout} seconds")
            return False
        
        # Check if device file exists
        if not os.path.exists(device_path):
            print(f"Waiting for receiver/device {device_path}...")
            time.sleep(2)
            continue
        
        # Device exists - check if controller is actually connected
        if is_controller_connected(device_path):
            print(f"Controller connected and ready!")
            time.sleep(0.5)  # Give it a moment to stabilize
            return True
        
        # Receiver present but no controller yet
        time.sleep(1)

def main():
    print("Starting Controller UART Bridge...")

    # 1. Open Serial Port
    try:
        serial_port = Serial(UART_PORT, BAUD_RATE)
        print(f"Successfully opened serial port {UART_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Could not open serial port {UART_PORT}: {e}")
        sys.exit(1)

    # 2. Wait for and Open Controller Device
    jsdev = None
    while True:  # Main retry loop
        try:
            # Wait for controller to be powered on
            if not wait_for_controller(CONTROLLER_DEV, CONTROLLER_WAIT_TIMEOUT):
                print("Controller wait timeout expired")
                serial_port.close()
                sys.exit(1)
            
            # Now try to open it (should succeed since we verified it's connected)
            jsdev = open(CONTROLLER_DEV, 'rb')
            
            # Get the name of the device
            buf = array.array('B', [0] * 64)
            fcntl.ioctl(jsdev, JSIOCGNAME(len(buf)), buf) 
            js_name = buf.tobytes().rstrip(b'\x00').decode('utf8')
            print(f"Listening to controller: {js_name} at {CONTROLLER_DEV}")
            break  # Successfully opened, exit retry loop
            
        except FileNotFoundError:
            print(f"Controller device {CONTROLLER_DEV} disappeared, waiting again...")
            time.sleep(2)
            continue
        except PermissionError as e:
            print(f"Permission denied for {CONTROLLER_DEV}: {e}")
            print("Ensure user is in 'input' group: sudo usermod -a -G input blankmcu")
            time.sleep(5)
            continue
        except OSError as e:
            if e.errno == 22:  # Invalid argument - controller disconnected during open
                print(f"Controller disconnected during open, retrying...")
                time.sleep(1)
                continue
            else:
                print(f"OS error opening controller device: {e} (errno {e.errno})")
                time.sleep(2)
                continue
        except Exception as e:
            print(f"Error opening controller device: {e}")
            time.sleep(2)
            continue

    # 3. Main Event Loop
    try:
        last_send_time = time.time()
        SEND_INTERVAL = 0.05 # Limit updates to 20 times per second (50ms interval)

        # Read the initial state (discarding INIT events)
        while True:
            event_data = jsdev.read(JS_EVENT_SIZE)
            if not event_data:
                break

            # The structure is: time (4 bytes), value (2 bytes), type (1 byte), index (1 byte)
            js_time, js_value, js_type, js_index = struct.unpack('IhBB', event_data)
            
            # Check for initial state event
            if not (js_type & JS_EVENT_INIT):
                break

        print("Controller initialized, starting main loop...")

        while True:
            try:
                # Non-blocking read (read_loop is not used with fcntl open)
                event_data = jsdev.read(JS_EVENT_SIZE)
                if not event_data:
                    # Use select or poll for truly non-blocking, but simple read is often sufficient
                    time.sleep(0.001) 
                    continue

                js_time, js_value, js_type, js_index = struct.unpack('IhBB', event_data)

                # Process Axis Event
                if js_type & JS_EVENT_AXIS:
                    key = AXIS_MAP.get(js_index)
                    if key:
                        controller_state[key] = js_value  # Store raw value for full precision

                # Process Button Event
                elif js_type & JS_EVENT_BUTTON:
                    key = BUTTON_MAP.get(js_index)
                    if key:
                        controller_state[key] = js_value # 1 for press, 0 for release

                # Send the state if enough time has passed
                current_time = time.time()
                if current_time - last_send_time >= SEND_INTERVAL:
                    format_and_send(serial_port)
                    last_send_time = current_time
                    
            except OSError as e:
                # Controller disconnected during operation
                print(f"\nController disconnected: {e}")
                print("Waiting for controller to reconnect...")
                jsdev.close()
                # Return to controller wait loop
                break

    except KeyboardInterrupt:
        print("\nShutting down bridge...")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

    finally:
        try:
            if jsdev:
                jsdev.close()
        except:
            pass
        serial_port.close()
        print("Serial port closed. Script finished.")

if __name__ == "__main__":
    main()
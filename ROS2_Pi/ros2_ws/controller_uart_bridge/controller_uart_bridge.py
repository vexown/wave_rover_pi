#!/usr/bin/env python3

import os
import sys
import time
from periphery import Serial
import select
import fcntl
import struct
import array

# --- Configuration ---
UART_PORT = "/dev/ttyS0"  # Serial Port (UART) connected to the ESP32 when the RPi is inserted onto the waverover top-controller pin header. Allows comms between the two.
BAUD_RATE = 921600
CONTROLLER_DEV = "/dev/input/js0"  # Use the joystick device
CONTROLLER_WAIT_TIMEOUT = None  # None = wait forever, or set number of seconds

# Data structure for current controller state
# Axes are -32767 to 32767 (full 16-bit resolution preserved for maximum precision).
controller_state = {
    'LX': 0, 'LY': 0,  # Left Stick
    'RX': 0, 'RY': 0,  # Right Stick
    'LT': 0, 'RT': 0,  # Triggers 
    'DPX': 0, 'DPY': 0,  # D-Pad
    'A': 0, 'B': 0, 'X': 0, 'Y': 0,  # Face Buttons
    'LB': 0, 'RB': 0, 'BACK': 0, 'START': 0, 'GUIDE': 0,  # Other Buttons
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

# Constants for joystick event reading
JS_EVENT_SIZE = 8
JS_EVENT_BUTTON = 0x01  # button pressed/released
JS_EVENT_AXIS = 0x02    # joystick moved
JS_EVENT_INIT = 0x80    # initial state of device

# Update rate configuration
# IMPORTANT: Increased from 20Hz to 100Hz to catch fast joystick movements
# This prevents missing "return to center" events when joystick is quickly released
SEND_INTERVAL = 0.01  # Send updates at 100 Hz (every 10ms) - was 0.05 (20Hz)
JOYSTICK_IMMEDIATE_SEND = True  # Send immediately when joystick axes change (LX, LY, RX, RY)


def format_and_send(uart):
    """
    Format controller state and send to ESP32 via UART.
    
    Sends data in format: S|LX:0|LY:0|RX:0|RY:0|...|E\n
    """
    data_string = "S|"
    data_string += "|".join(f"{k}:{v}" for k, v in controller_state.items())
    data_string += "|E\n"
    
    try:
        uart.write(data_string.encode('utf-8'))
        # Uncomment for debugging:
        timestamp = int(time.time() * 1000)  # Milliseconds since epoch
        print(f"[{timestamp}] Sent: {data_string.strip()}", flush=True)
    except Exception as e:
        print(f"Error writing to serial: {e}", flush=True)


def is_controller_connected(device_path):
    """
    Check if controller is actually connected and responsive.
    
    This function uses select() to verify the device can be opened and has
    data available. This distinguishes between:
    - Receiver present but no controller paired (device exists but not ready)
    - Controller actively connected and responsive
    
    Args:
        device_path: Path to the joystick device (e.g., /dev/input/js0)
        
    Returns:
        True if controller is connected and responsive, False otherwise
    """
    try:
        # Try to open the device
        fd = open(device_path, 'rb')
        
        # Use select to check if device has data available (with 0.5s timeout)
        # For active controllers, select() will return immediately with the device in ready list
        ready, _, _ = select.select([fd], [], [], 0.5)
        
        fd.close()
        
        # If device is in ready list or we can at least open it successfully, controller is connected
        return True
        
    except Exception:
        return False


def wait_for_controller(device_path, timeout=None):
    """
    Wait for controller device to become available and responsive.
    
    This function polls the device file and tests connectivity until either:
    - Controller is detected as connected
    - Timeout expires (if specified)
    
    Args:
        device_path: Path to the joystick device (e.g., /dev/input/js0)
        timeout: Maximum seconds to wait (None = wait forever)
        
    Returns:
        True if controller is ready, False if timeout expires
    """
    print(f"Waiting for controller at {device_path}...", flush=True)
    start_time = time.time()
    last_message_time = 0
    
    while True:
        current_time = time.time()
        
        # Check timeout
        if timeout is not None and (current_time - start_time) > timeout:
            print(f"Timeout waiting for controller after {timeout} seconds", flush=True)
            return False
        
        # Periodic status message (every 5 seconds)
        if current_time - last_message_time > 5:
            elapsed = int(current_time - start_time)
            print(f"Still waiting for controller... ({elapsed}s elapsed)", flush=True)
            last_message_time = current_time
        
        # Check if device file exists
        if not os.path.exists(device_path):
            time.sleep(1)
            continue
        
        # Device exists - test if controller is actually connected
        if is_controller_connected(device_path):
            print(f"Controller connected at {device_path}", flush=True)
            return True
        
        # Device exists but controller not responding yet (e.g., receiver present but controller off)
        time.sleep(1)


def main():
    """
    Main entry point for the Xbox Controller to UART Bridge.
    
    This bridge:
    1. Opens a serial connection to the ESP32
    2. Waits for an Xbox controller to connect via xboxdrv
    3. Reads controller events and sends them to the ESP32
    4. Handles controller disconnects/reconnects gracefully
    """
    print("Starting Controller UART Bridge...", flush=True)

    # 1. Open Serial Port
    try:
        serial_port = Serial(UART_PORT, BAUD_RATE)
        print(f"Successfully opened serial port {UART_PORT} at {BAUD_RATE} baud.", flush=True)
    except Exception as e:
        print(f"Could not open serial port {UART_PORT}: {e}", flush=True)
        sys.exit(1)

    # 2. Wait for and Open Controller Device
    jsdev = None
    while True:  # Main retry loop for controller connection
        try:
            # Wait for controller to be powered on and connected
            if not wait_for_controller(CONTROLLER_DEV, CONTROLLER_WAIT_TIMEOUT):
                print("Controller wait timeout expired", flush=True)
                serial_port.close()
                sys.exit(1)
            
            # Controller is ready - open the device
            jsdev = open(CONTROLLER_DEV, 'rb')
            print(f"Successfully opened controller device {CONTROLLER_DEV}", flush=True)
            break  # Exit retry loop
            
        except FileNotFoundError:
            print(f"Controller device {CONTROLLER_DEV} disappeared, waiting again...", flush=True)
            time.sleep(2)
            continue
        except PermissionError as e:
            print(f"Permission denied for {CONTROLLER_DEV}: {e}", flush=True)
            print("Ensure user is in 'input' group: sudo usermod -a -G input blankmcu", flush=True)
            time.sleep(5)
            continue
        except OSError as e:
            print(f"OS error opening controller device: {e} (errno {e.errno})", flush=True)
            time.sleep(2)
            continue
        except Exception as e:
            print(f"Error opening controller device: {e}", flush=True)
            time.sleep(2)
            continue

    # 3. Main Event Loop
    try:
        last_send_time = time.time()

        # Discard initial state events
        # The joystick device sends JS_EVENT_INIT events on startup
        # to report the current state of all buttons/axes
        while True:
            event_data = jsdev.read(JS_EVENT_SIZE)
            if not event_data:
                break

            js_time, js_value, js_type, js_index = struct.unpack('IhBB', event_data)
            
            # Break when we get a non-init event (actual controller input)
            if not (js_type & JS_EVENT_INIT):
                break

        print("Controller initialized, starting main loop (100Hz updates)...", flush=True)

        # Main event processing loop
        while True:
            try:
                # Use select() with timeout to efficiently wait for controller events
                # This allows us to batch process multiple events and maintain consistent send rate
                ready, _, _ = select.select([jsdev], [], [], SEND_INTERVAL)
                
                current_time = time.time()
                joystick_moved = False  # Track if critical joystick axes changed
                
                # Process all available events (batch processing)
                # This ensures we don't miss events that happen in quick succession
                if ready:
                    while True:
                        # Try to read event without blocking
                        try:
                            event_data = jsdev.read(JS_EVENT_SIZE)
                            if not event_data:
                                break
                        except:
                            break

                        # Unpack event: time, value, type, index
                        js_time, js_value, js_type, js_index = struct.unpack('IhBB', event_data)

                        # Process Axis Event (joysticks, triggers, d-pad)
                        if js_type & JS_EVENT_AXIS:
                            key = AXIS_MAP.get(js_index)
                            if key:
                                controller_state[key] = js_value  # Store raw 16-bit value
                                # Mark that a critical joystick axis moved (for immediate send)
                                # This is crucial to catch "return to center" events on fast movements
                                if key in ['LX', 'LY', 'RX', 'RY']:
                                    joystick_moved = True

                        # Process Button Event
                        elif js_type & JS_EVENT_BUTTON:
                            key = BUTTON_MAP.get(js_index)
                            if key:
                                controller_state[key] = js_value  # 1 for press, 0 for release
                        
                        # Check if more data available in the buffer
                        # If yes, continue reading; if no, break and send current state
                        ready_check, _, _ = select.select([jsdev], [], [], 0)
                        if not ready_check:
                            break

                # Send state to ESP32 immediately if joystick moved, OR at regular interval
                # Immediate send on joystick movement prevents missing "return to center" events
                # Regular interval ensures continuous updates even when controller is idle
                if (JOYSTICK_IMMEDIATE_SEND and joystick_moved) or \
                   (current_time - last_send_time >= SEND_INTERVAL):
                    format_and_send(serial_port)
                    last_send_time = current_time
                    
            except OSError as e:
                # Controller disconnected during operation
                print(f"\nController disconnected: {e}", flush=True)
                print("Waiting for controller to reconnect...", flush=True)
                jsdev.close()
                # Return to controller wait loop
                break

    except KeyboardInterrupt:
        print("\nShutting down bridge...", flush=True)
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}", flush=True)

    finally:
        # Clean up resources
        try:
            if jsdev:
                jsdev.close()
        except:
            pass
        serial_port.close()
        print("Serial port closed. Script finished.", flush=True)


if __name__ == "__main__":
    main()
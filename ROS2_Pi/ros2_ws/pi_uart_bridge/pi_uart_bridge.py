#!/usr/bin/env python3
"""
Raspberry Pi to ESP32 UART bridge.

The only process that writes to the UART going to the wave_rover_driver board. It sends:
 - Xbox controller frames, while a controller is in use:  S|LX:0|LY:0|...|E\n
 - a status line, once per second, always:                 $PI,<uptime_s>,<flags>*<checksum>\n
"""

import os
import subprocess
import sys
import threading
import time
from periphery import Serial
import select
import struct

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

# ============================================================================
# Xbox 360 Controller Mapping (Experimentally Determined)
# ============================================================================
# This mapping was determined using the test_controller_mapping.py diagnostic tool
# and is specific to the Xbox 360 controller with Microsoft Wireless Receiver.
#
# AXES (Analog inputs, range: -32767 to 32767):
# -----------------------------------------------
# Index 0: Left Joystick X-axis (LX)
# Index 1: Left Joystick Y-axis (LY)
# Index 2: Right Joystick X-axis (RX)
# Index 3: Right Joystick Y-axis (RY)
# Index 4: Right Trigger (RT) - 32767 fully pressed, -32767 fully released
# Index 5: Left Trigger (LT) - 32767 fully pressed, -32767 fully released
# Index 6: D-Pad X-axis (DPX) - -32767 left, 32767 right, 0 centered
# Index 7: D-Pad Y-axis (DPY) - -32767 up, 32767 down, 0 centered
#
# BUTTONS (Digital inputs, 0=released, 1=pressed):
# ------------------------------------------------
# Index 0: A button
# Index 1: B button
# Index 2: X button
# Index 3: Y button
# Index 4: Left Bumper (LB)
# Index 5: Right Bumper (RB)
# Index 6: Back button
# Index 7: Start button
# Index 8: Guide button (Xbox logo) - NOTE: Not included in controller_state
# Index 9: Left Joystick Press (L3) - NOTE: Not included in controller_state
# Index 10: Right Joystick Press (R3) - NOTE: Not included in controller_state
# ============================================================================

# Mapping of joystick axis index to controller_state key
AXIS_MAP = {
    0: 'LX',   # Left Joystick X-axis
    1: 'LY',   # Left Joystick Y-axis
    2: 'RX',   # Right Joystick X-axis
    3: 'RY',   # Right Joystick Y-axis
    4: 'RT',   # Right Trigger (32767 pressed, -32767 released)
    5: 'LT',   # Left Trigger (32767 pressed, -32767 released)
    6: 'DPX',  # D-Pad X-axis (-32767 left, 32767 right)
    7: 'DPY'   # D-Pad Y-axis (-32767 up, 32767 down)
}

# Mapping of joystick button index to controller_state key
BUTTON_MAP = {
    0: 'A',      # A button
    1: 'B',      # B button
    2: 'X',      # X button
    3: 'Y',      # Y button
    4: 'LB',     # Left Bumper
    5: 'RB',     # Right Bumper
    6: 'BACK',   # Back button
    7: 'START',  # Start button
    # Note: Guide button (index 8), L3 (index 9), and R3 (index 10) are available
    # but not currently mapped to controller_state
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

# Disconnect detection timeout
# I haven't found a reliable way to detect controller disconnects because
# the Microsoft Xbox Wireless Receiver keeps the device file present even when
# no controller is paired. So for now I'm using a timeout-based approach to 
# detect lack of controller events as a sign of disconnect.
DISCONNECT_TIMEOUT = 5.0  # seconds

# Status line, shown by the wave_rover_driver board in the Raspberry Pi row of its STATUS panel.
# The driver board shows the Pi as offline after 3 s without one.
#
# Format: $PI,<uptime_s>,<flags>*<checksum>\n
#   uptime_s  seconds since the Pi booted, decimal
#   flags     sum of the STATUS_FLAG_ values below, decimal
#   checksum  XOR of all characters between '$' and '*', two uppercase hex digits
#
# The flag values must match ROBOT_PI_FLAG_ in robot_status.h of the wave_rover_driver repo.
STATUS_INTERVAL = 1.0  # seconds
STATUS_FLAG_CONTROLLER_ACTIVE = 0x01  # Controller frames are being sent (events within DISCONNECT_TIMEOUT)
# ros2_camera_feed.service is not in the list: it is disabled on purpose.
STATUS_SERVICES = [  # (systemd unit, flag set when the unit is active)
    ("xbox360_pad.service",       0x02),
    ("ros2_serial_nodes.service", 0x04),
    ("ros2_imu_parser.service",   0x08),
    ("ros2_gps_parser.service",   0x10),
]

# The main thread sends controller frames and the status thread sends status lines.
# Each one is written while holding this lock, so they can't get mixed on the wire.
uart_write_lock = threading.Lock()

# Set by the main thread, read by the status thread
controller_active = False


def format_and_send(uart):
    """
    Format controller state and send to ESP32 via UART.
    
    Sends data in format: S|LX:0|LY:0|RX:0|RY:0|...|E\n
    """
    data_string = "S|"
    data_string += "|".join(f"{k}:{v}" for k, v in controller_state.items())
    data_string += "|E\n"
    
    try:
        with uart_write_lock:
            uart.write(data_string.encode('utf-8'))
        # Uncomment for debugging:
        # timestamp = int(time.time() * 1000)  # Milliseconds since epoch
        # print(f"[{timestamp}] Sent: {data_string.strip()}", flush=True)
    except Exception as e:
        print(f"Error writing to serial: {e}", flush=True)


def build_status_line():
    """
    Build the status line: Pi uptime, whether a controller is active and which services run.
    """
    with open('/proc/uptime') as uptime_file:
        uptime_s = int(float(uptime_file.read().split()[0]))

    flags = STATUS_FLAG_CONTROLLER_ACTIVE if controller_active else 0

    # systemctl prints one state per unit, in the order the units were given.
    # Its exit code is not checked: it is non-zero whenever any unit is not active.
    units = [unit for unit, _ in STATUS_SERVICES]
    result = subprocess.run(["systemctl", "is-active"] + units,
                            capture_output=True, text=True, timeout=2)
    states = result.stdout.split()
    if len(states) != len(units):
        raise RuntimeError(f"systemctl is-active returned {len(states)} states for {len(units)} units")
    for (_, flag), state in zip(STATUS_SERVICES, states):
        if state == "active":
            flags |= flag

    payload = f"PI,{uptime_s},{flags}"
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return f"${payload}*{checksum:02X}\n"


def status_task(uart):
    """
    Thread: send the status line once per second, whatever the controller is doing.
    """
    last_error = None

    while True:
        try:
            status_line = build_status_line()
            with uart_write_lock:
                uart.write(status_line.encode('utf-8'))
            error = None
        except Exception as e:
            error = str(e)

        # Print only when the result changes, not once per second
        if error != last_error:
            if error is not None:
                print(f"Status line not sent: {error}", flush=True)
            else:
                print("Status line is being sent again", flush=True)
            last_error = error

        time.sleep(STATUS_INTERVAL)


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
    Main entry point for the Raspberry Pi to ESP32 UART bridge.
    
    This bridge:
    1. Opens a serial connection to the ESP32
    2. Starts sending the status line once per second
    3. Waits for an Xbox controller to connect
    4. Reads controller events and sends them to the ESP32
    5. Handles controller disconnects/reconnects gracefully
    """
    global controller_active

    print("Starting Pi UART Bridge...", flush=True)

    # 1. Open Serial Port (once, stays open for entire script lifetime)
    try:
        serial_port = Serial(UART_PORT, BAUD_RATE)
        print(f"Successfully opened serial port {UART_PORT} at {BAUD_RATE} baud.", flush=True)
    except Exception as e:
        print(f"Could not open serial port {UART_PORT}: {e}", flush=True)
        sys.exit(1)

    # Status line thread. A daemon thread, so it ends when the main thread does.
    threading.Thread(target=status_task, args=(serial_port,), daemon=True).start()
    print(f"Status line started (every {STATUS_INTERVAL} s)", flush=True)

    # 2. Main Controller Connection Loop
    # This outer loop handles controller connect/disconnect/reconnect cycles
    # The script stays alive and keeps the serial port open throughout
    try:
        while True:  # Main retry loop for controller connection
            jsdev = None
            
            try:
                # Wait for controller to be powered on and connected
                if not wait_for_controller(CONTROLLER_DEV, CONTROLLER_WAIT_TIMEOUT):
                    print("Controller wait timeout expired", flush=True)
                    break  # Exit main loop, will close serial and exit
                
                # Controller is ready - open the device
                jsdev = open(CONTROLLER_DEV, 'rb')
                print(f"Successfully opened controller device {CONTROLLER_DEV}", flush=True)
                
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
                print(f"OS error opening controller device: {e}", flush=True)
                time.sleep(2)
                continue
            except Exception as e:
                print(f"Error opening controller device: {e}", flush=True)
                time.sleep(2)
                continue

            # 3. Main Event Loop (for this connection session)
            try:
                last_send_time = time.time()
                last_event_time = time.time()

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
                controller_active = True

                # Main event processing loop
                while True:
                    try:
                        # Use select() with timeout to efficiently wait for controller events
                        # This allows us to batch process multiple events and maintain consistent send rate
                        ready, _, _ = select.select([jsdev], [], [], SEND_INTERVAL)
                        
                        current_time = time.time()
                        joystick_moved = False  # Track if critical joystick axes changed
                        
                        # Check for controller disconnect via event timeout
                        if current_time - last_event_time > DISCONNECT_TIMEOUT:
                            print(f"\nNo controller events for {DISCONNECT_TIMEOUT}s - controller disconnected", flush=True)
                            
                            # Reset controller state to neutral
                            for key in controller_state:
                                controller_state[key] = 0
                            
                            # Send neutral state to ESP32
                            format_and_send(serial_port)
                            print("Sent neutral state after disconnect detection", flush=True)
                            
                            break  # Exit inner loop, will close jsdev and restart outer loop
                        
                        # Process all available events (batch processing)
                        # This ensures we don't miss events that happen in quick succession
                        if ready:
                            last_event_time = current_time  # Update timestamp when we receive events
                            
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
                        
                        # Reset controller state to neutral
                        for key in controller_state:
                            controller_state[key] = 0
                        
                        # Send neutral state to ESP32
                        format_and_send(serial_port)
                        print("Sent neutral state after disconnect", flush=True)
                        
                        break  # Exit inner loop
                
            finally:
                # Clean up this controller session
                controller_active = False
                if jsdev:
                    try:
                        jsdev.close()
                        print("Closed controller device", flush=True)
                    except:
                        pass
                
                # Loop back to wait for controller reconnection
                print("Returning to controller wait loop...", flush=True)

    except KeyboardInterrupt:
        print("\nShutting down bridge...", flush=True)
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}", flush=True)

    finally:
        # Clean up serial port on final exit
        serial_port.close()
        print("Serial port closed. Script finished.", flush=True)


if __name__ == "__main__":
    main()
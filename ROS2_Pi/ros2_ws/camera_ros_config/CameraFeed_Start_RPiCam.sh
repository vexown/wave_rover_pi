#!/bin/bash
# This script starts the custom RPi camera node using GStreamer with libcamera
# This avoids the libcamera IPA proxy issues with camera_ros and rpicam-vid symbol issues
#
# REQUIRED DEPENDENCIES - Install before running:
# System packages:
#   sudo apt update && sudo apt install \
#     gstreamer1.0-tools \
#     gstreamer1.0-plugins-base \
#     gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad \
#     gstreamer1.0-libcamera \
#     libcamera-tools \
#     libcamera-ipa \
#     python3-opencv \
#     v4l-utils
#
# ROS2 packages (if using ROS2):
#   sudo apt install \
#     ros-jazzy-cv-bridge \
#     ros-jazzy-image-transport
#
# Python modules (should be available after installing above packages):
#   cv2, cv_bridge, rclpy, sensor_msgs

######################################################################
# --- Script Initialization ---
######################################################################
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

######################################################################
# --- Camera Configuration Parameters ---
######################################################################
TARGET_FPS=15 # Reduced FPS for stability - high FPS causes JPEG corruption
WIDTH=800
HEIGHT=600
FORMAT="rgb"  # rgb or bgr
PUBLISH_COMPRESSED=true  # Set to true to publish compressed images
JPEG_QUALITY=75  # Reduced quality for less CPU load (1-100, higher = better quality, larger size)
######################################################################

######################################################################
# --- Startup Information Display ---
######################################################################
echo "=== RPi Camera Publisher Setup & Start ==="
echo "Camera settings: ${WIDTH}x${HEIGHT} @ ${TARGET_FPS}fps, format: ${FORMAT}"
echo "Compressed images: ${PUBLISH_COMPRESSED}, JPEG quality: ${JPEG_QUALITY}%"
echo

######################################################################
# --- Script Preparation ---
######################################################################
# Make sure the Python script is executable
chmod +x "$SCRIPT_DIR/rpicam_publisher.py"

######################################################################
# --- Hardware Detection & Setup ---
######################################################################
echo "=== Hardware Detection ==="

# Check camera hardware detection
if [ -e "/dev/video0" ]; then
    echo "✓ Video devices found: $(ls /dev/video* | wc -l) devices"
else
    echo "⚠ No video devices found in /dev/"
fi

# Reset camera hardware to clear any stuck states
echo "Resetting camera hardware..."
if [ -d "/sys/class/gpio/gpio5" ]; then
    echo "Toggling camera reset GPIO..."
    echo 0 | sudo tee /sys/class/gpio/gpio5/value > /dev/null 2>&1
    sleep 0.5
    echo 1 | sudo tee /sys/class/gpio/gpio5/value > /dev/null 2>&1
    sleep 1
fi

# Kill any existing libcamera processes that might be stuck
echo "Cleaning up any stuck camera processes..."
sudo pkill -f "libcamerasrc" 2>/dev/null || true
sudo pkill -f "gst-launch" 2>/dev/null || true
sleep 2

# Test camera detection with libcamera
echo "Testing camera detection..."
if timeout 10s cam --list 2>/dev/null | grep -q "Available cameras" && 
   ! timeout 10s cam --list 2>&1 | grep -q "ERROR.*Failed to register camera"; then
    echo "✓ Camera detected by libcamera"
else
    echo "⚠ Camera not detected or has errors"
    echo "Common solutions:"
    echo "  1. Ensure camera cable is properly connected"
    echo "  2. Check if camera is enabled in /boot/firmware/config.txt"
    echo "  3. Reboot after installing libcamera-ipa"
    echo "  4. Try: sudo modprobe bcm2835-v4l2"
    echo "  5. Check dmesg for camera errors: dmesg | grep -i camera"
fi

######################################################################
# --- Camera Publisher Startup ---
######################################################################
echo "=== Starting Camera Publisher ==="

echo "All checks passed! Starting RPi Camera Publisher..."
echo "Press Ctrl+C to stop the camera feed"
echo

# Start the camera publisher
exec python3 "$SCRIPT_DIR/rpicam_publisher.py" \
    --ros-args \
    -p width:=$WIDTH \
    -p height:=$HEIGHT \
    -p fps:=$TARGET_FPS \
    -p format:="$FORMAT" \
    -p publish_compressed:=$PUBLISH_COMPRESSED \
    -p jpeg_quality:=$JPEG_QUALITY
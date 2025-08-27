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
TARGET_FPS=30
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
# --- Camera Publisher Startup ---
######################################################################
echo "=== Starting Camera Publisher ==="
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
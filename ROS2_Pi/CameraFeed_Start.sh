#!/bin/bash
# This script starts the camera_ros node with the specified parameters.
# See details on the github repo: https://github.com/christianrauch/camera_ros
# The package is also available on the official ROS2 package repository: https://index.ros.org/p/camera_ros/

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

######################################################################
# --- Configuring Camera Parameters ---
######################################################################
CALIBRATION_FILE="file://"$SCRIPT_DIR"/dummy_calibration.yaml"
TARGET_FPS=20 # Set your desired FPS (FPS is the same as framerate in Hz)
WIDTH=800
HEIGHT=600
######################################################################

######################################################################
# --- Controlling Camera Framerate with camera_ros ---
######################################################################
# The 'camera_ros' driver, when using libcamera, doesn't directly expose a
# 'framerate' parameter. Instead, you control the frame rate by setting the
# 'FrameDurationLimits' parameter. This parameter expects a duration in
# microseconds (µs) which corresponds to the inverse of the desired frame rate.

# Formula to convert desired Frame Rate (in Hz) to Duration (in µs):
#   duration_us = (1 / framerate_hz) * 1000000

# Formula to convert Duration (in µs) back to Frame Rate (in Hz):
#   framerate_hz = 1000000 / duration_us

# Example: Setting a fixed frame rate of 20 Hz
#   1. Calculate the duration:
#      duration_us = (1 / 20) * 1000000 = 0.05 * 1000000 = 50000 µs
#   2. Set the 'FrameDurationLimits' parameter when running the node:
#      ros2 run camera_ros camera_node --ros-args -p FrameDurationLimits:="[50000,50000]"

# Replace '20' with your desired frame rate to calculate the appropriate
# 'FrameDurationLimits' value. The value should be a string in the format
# "[min_duration_us,max_duration_us]". To set a fixed frame rate, both
# min_duration_us and max_duration_us should be the same.

# Note: The 'FrameDurationLimits' control must be exposed by your camera for
# this method to work. Check the output of 'ros2 param list /camera' (confirm node 
# name with ros2 node list) while the node is running to see if 'FrameDurationLimits' 
# is available.
######################################################################
if (( TARGET_FPS > 0 )); then
  FRAME_DURATION_US=$((1000000 / TARGET_FPS))
else
  echo "Error: TARGET_FPS must be greater than 0."
  exit 1
fi
######################################################################

# Install dependencies if not already done
#sudo apt install ros-jazzy-camera-ros
#sudo apt install ros-jazzy-image-pipeline

ros2 run camera_ros camera_node \
  --ros-args \
  -p camera_info_url:="$CALIBRATION_FILE" \
  -p width:=$WIDTH \
  -p height:=$HEIGHT \
  -p FrameDurationLimits:="[$FRAME_DURATION_US,$FRAME_DURATION_US]" \
  -p camera:="/base/soc/i2c0mux/i2c@1/imx708@1a"

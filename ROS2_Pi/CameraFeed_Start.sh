#!/bin/bash

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
CALIBRATION_FILE="file://"$SCRIPT_DIR"/dummy_calibration.yaml"

# Install dependencies if not already done
#sudo apt install ros-jazzy-camera-ros
#sudo apt install ros-jazzy-image-pipeline

ros2 run camera_ros camera_node \
  --ros-args \
  -p camera_info_url:="$CALIBRATION_FILE" \
  -p width:=800 \
  -p height:=600 \
  -p camera:="/base/soc/i2c0mux/i2c@1/imx708@1a"

#!/bin/bash
# Launch/start all ROS 2 nodes in the workspace.

# Define the ros2 workspace directory
ROS2_WS_DIR="$HOME/ros2_ws"

# Source the ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source the workspace's install setup file
source "${ROS2_WS_DIR}/install/setup.bash"

# Start/launch the selected nodes
# Node #1 - visual_odo
# This command opens a new tab in GNOME Terminal to run the ROS2 visual odometry node in isolation.
# After launching, 'exec bash' keeps the terminal tab open, allowing for interactive commands or monitoring,
# and prevents the tab from closing immediately after the launch completes.
gnome-terminal --tab -- bash -c "ros2 launch visual_odo simple_vo.launch.py; exec bash"

# Node #2 - camera_feed_subscriber
# [disabled for now] gnome-terminal --tab -- bash -c "ros2 run camera_feed_subscriber image_subscriber; exec bash"

# Wait for all background jobs to finish
# The 'wait' command is used here to pause the script execution until all background processes 
# (launched ROS2 nodes) have completed. This ensures the script remains active while the nodes are running, 
# allowing for proper synchronization and subsequent shutdown of the nodes when the script ends, rather than 
# relying on external PID tracking or manual termination.
wait
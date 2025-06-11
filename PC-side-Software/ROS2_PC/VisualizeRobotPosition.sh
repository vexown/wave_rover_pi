#!/bin/bash

# This script sets up the environment for visualizing a robot's position in ROS2 using RViz2.

# This command defines a fixed relationship (transform) between the 'map' frame and the 'gps_link' frame,
# which is necessary for RViz2 to correctly position data associated with the 'gps_link' frame within the 'map'.
# The map frame is typically defined as a fixed, global coordinate frame that represents the environment your robot is operating in.
# gps_link comes from your NavSatFix messages (frame_id) on the /gps/fix topic published by the RPi which gets it from ESP32 running the ublox GNSS module.
#
# The following command runs the 'static_transform_publisher' node from the 'tf2_ros' package (https://wiki.ros.org/tf2_ros)
# 'tf2_ros' is the package that provides tools for working with coordinate transforms - see more at about tf2 in ROS 2:
# https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html
#
# 'static_transform_publisher' is a node that publishes a static transform between two coordinate frames 
# The arguments '0 0 0 0 0 0' represent the translation (x, y, z in meters) and rotation (roll, pitch, yaw in radians)
# of the child frame relative to the parent frame. In this case, it's a zero transform (no offset or rotation).
# No offset = No translation = the origin of the child frame is at the origin of the parent frame
# No rotation = the axes of the child frame are perfectly aligned with the axes of the parent frame
#
# 'map' is the name of the parent coordinate frame. 
# 'gps_link' is the name of the child coordinate frame.
#
# The '&' at the end of the command runs it in the background, to not block the terminal for subsequent commands.
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map gps_link &

# The following command launches RViz2, the 3D visualization tool for ROS 2.
# 'rviz2' is the command to start the RViz2 application.
# The '-d' option (short for --display-config) specifies an RViz2 configuration file to load.
rviz2 -d RVizConfig/rviz_config.rviz
# wave_rover_pi

This repository contains the software for the Raspberry Pi operating as the upper-level computer on the Wave Rover robot. It also includes the PC-side software for monitoring, visualization, and interaction with the robot.

## Overview

The software is built upon the Robot Operating System 2 (ROS2) and is structured into two main parts:

1.  **Raspberry Pi Software (`ROS2_Pi`)**: This runs on the Raspberry Pi aboard the Wave Rover. It is responsible for interfacing with lower-level board, the ESP32, which provides GNSS and other sensor data via serial communication. It also handles the camera data (directly connected to the Pi - Camera Module 3)
2.  **PC-side Software (`PC-side-Software`)**: This runs on a separate computer. It is used to visualize the camera feed, robot position, and potentially run more computationally intensive tasks like object detection using YOLO.

The system is designed to be modular, with different functionalities encapsulated in ROS2 packages.

## Features

-   **ROS2-based:** Utilizes the latest ROS2 framework for robust and modular robotics applications.
-   **GPS Navigation:** A dedicated ROS2 node (`gps_tools`) to parse and publish GPS data.
-   **Serial Communication:** A ROS2 node (`serial_com`) for communication with microcontrollers or other serial devices.
-   **Camera Streaming:** Scripts to stream video from the Raspberry Pi camera.
-   **Object Detection:** Includes a pre-trained YOLO model (`yolo11x.pt`) and tools for camera stream viewing on the PC.
-   **Docker Support:** Dockerfiles and scripts are provided for setting up the ROS2 environment on both the PC and the Raspberry Pi, ensuring easy and consistent deployment.
-   **Systemd Services:** Systemd services are included for automatically starting the ROS2 nodes on the Raspberry Pi at boot.

## Getting Started

1.  **Setup ROS2:** Use the `Ubuntu_24_04_LTS_Install_ROS2_Jazzy.sh` script to install ROS2 Jazzy on your system (both PC and Pi if needed).
2.  **Setup Camera:** On the Raspberry Pi, use `Ubuntu_24_04_LTS_SetupCameraModule3.sh` to configure the camera module.
3.  **Docker Environment:** For a containerized setup, use the scripts in the `DockerROS2` directories inside `ROS2_Pi` and `PC-side-Software` to build and run a Docker container with the ROS2 environment.
4.  **Build ROS2 Workspaces:**
    -   On the Pi, navigate to `ROS2_Pi/ros2_ws` and build the workspace.
    -   On the PC, navigate to `PC-side-Software/ROS2_PC/ros2_ws` and build the workspace.

## Usage

### On the Raspberry Pi

The ROS2 nodes on the Pi can be launched automatically on boot using the provided systemd services. See `ROS2_Pi/systemd_services/SetupServices.sh` to enable them.

The main nodes are:
-   `gps_parser_node`: Publishes GPS data.
-   `serial_node`: Handles serial communication.

### On the PC

-   **View Camera Stream:** Run `camera_stream_viewer.py` in `PC-side-Software`.
-   **Visualize Robot Data:** Use the ROS2 tools and RViz with the provided configuration (`PC-side-Software/ROS2_PC/RVizConfig/`) to visualize data from the robot. The `camera_feed_subscriber` can be used to subscribe to image topics.


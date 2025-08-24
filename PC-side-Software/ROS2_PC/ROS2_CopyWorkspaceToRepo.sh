#!/bin/bash
# Copy the entire ROS 2 workspace from the home directory to the repository

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the source directory
SRC_DIR="${HOME}/ros2_ws"

# Define the destination directory
DEST_DIR="${SCRIPT_DIR}"

# Check if the source directory exists
if [ ! -d "${SRC_DIR}" ]; then
    echo "Error: Source directory ${SRC_DIR} not found."
    exit 1
fi

# Create the destination directory if it doesn't exist
mkdir -p "${DEST_DIR}"

# First delete the existing ros2_ws directory in the destination
if [ -d "${DEST_DIR}/ros2_ws" ]; then
    echo "Deleting existing ros2_ws directory in ${DEST_DIR}..."
    sudo rm -rf "${DEST_DIR}/ros2_ws"
fi

# Then copy the ros2_ws directory to the destination
echo "Copying ${SRC_DIR} to ${DEST_DIR}..."
cp -r "${SRC_DIR}" "${DEST_DIR}/"

if [ $? -eq 0 ]; then
    echo "Successfully copied ros2_ws to ${DEST_DIR}."
else
    echo "Error: Failed to copy ros2_ws to ${DEST_DIR}."
    exit 1
fi


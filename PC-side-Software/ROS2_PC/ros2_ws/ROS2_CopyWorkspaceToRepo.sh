#!/bin/bash
# Copy the src directory from the ROS 2 workspace to the repository

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the source directory
SRC_DIR="${HOME}/ros2_ws/src"

# Define the destination directory
DEST_DIR="${SCRIPT_DIR}"

# Check if the source directory exists
if [ ! -d "${SRC_DIR}" ]; then
    echo "Error: Source directory ${SRC_DIR} not found."
    exit 1
fi

# Create the destination directory if it doesn't exist
mkdir -p "${DEST_DIR}"

# First delete the existing src directory in the destination
if [ -d "${DEST_DIR}/src" ]; then
    echo "Deleting existing src directory in ${DEST_DIR}..."
    sudo rm -rf "${DEST_DIR}/src"
fi

# Then copy the src directory to the destination
echo "Copying ${SRC_DIR} to ${DEST_DIR}..."
cp -r "${SRC_DIR}" "${DEST_DIR}/"

if [ $? -eq 0 ]; then
    echo "Successfully copied src to ${DEST_DIR}."
else
    echo "Error: Failed to copy src to ${DEST_DIR}."
    exit 1
fi


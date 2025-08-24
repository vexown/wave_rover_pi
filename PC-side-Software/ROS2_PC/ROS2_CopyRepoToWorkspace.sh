#!/bin/bash
# Copy the entire ROS 2 workspace from the repository to the home directory.

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Define the source directory
SRC_DIR="${SCRIPT_DIR}/ros2_ws"

# Define the destination directory
DEST_DIR="${HOME}"

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
    
    # Build all ROS2 packages after copying
    echo "Building all ROS2 packages..."
    cd "${DEST_DIR}/ros2_ws"
    
    # Source the ROS2 environment
    source /opt/ros/jazzy/setup.bash
    
    # Build all packages
    colcon build
    
    if [ $? -eq 0 ]; then
        echo "Successfully built all ROS2 packages."
        echo "Run 'source ${DEST_DIR}/ros2_ws/install/setup.bash' to use the packages."
    else
        echo "Error: Failed to build ROS2 packages."
        exit 1
    fi
else
    echo "Error: Failed to copy ros2_ws to ${DEST_DIR}."
    exit 1
fi

exit 0
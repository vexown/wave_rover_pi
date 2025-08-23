#!/bin/bash
# This script starts the custom RPi camera node using GStreamer with libcamera
# This avoids the libcamera IPA proxy issues with camera_ros and rpicam-vid symbol issues

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

######################################################################
# --- Configuring Camera Parameters ---
######################################################################
TARGET_FPS=30 # Set your desired FPS
WIDTH=800
HEIGHT=600
FORMAT="rgb"  # rgb or bgr
PUBLISH_COMPRESSED=true  # Set to true to publish compressed images
JPEG_QUALITY=80  # JPEG quality (1-100, higher = better quality, larger size)
######################################################################

# Make sure the Python script is executable
chmod +x "$SCRIPT_DIR/rpicam_publisher.py"

# Check if GStreamer is available
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo "Error: gst-launch-1.0 not found. Please install it:"
    echo "sudo apt update && sudo apt install -y gstreamer1.0-tools"
    exit 1
fi

# Check if libcamera GStreamer plugin is available
if ! gst-inspect-1.0 libcamerasrc &> /dev/null; then
    echo "Error: libcamerasrc plugin not found. Please install it:"
    echo "sudo apt update && sudo apt install -y gstreamer1.0-libcamera"
    exit 1
fi

# Install required Python packages if needed
echo "Checking Python dependencies..."
python3 -c "import cv2, cv_bridge" 2>/dev/null || {
    echo "Installing required Python packages..."
    sudo apt install -y python3-opencv ros-jazzy-cv-bridge ros-jazzy-image-transport
}

echo "Starting RPi Camera Publisher with GStreamer libcamera..."
echo "Camera settings: ${WIDTH}x${HEIGHT} @ ${TARGET_FPS}fps, format: ${FORMAT}"
echo "Compressed images: ${PUBLISH_COMPRESSED}, JPEG quality: ${JPEG_QUALITY}%"

python3 "$SCRIPT_DIR/rpicam_publisher.py" \
    --ros-args \
    -p width:=$WIDTH \
    -p height:=$HEIGHT \
    -p fps:=$TARGET_FPS \
    -p format:="$FORMAT" \
    -p publish_compressed:=$PUBLISH_COMPRESSED \
    -p jpeg_quality:=$JPEG_QUALITY
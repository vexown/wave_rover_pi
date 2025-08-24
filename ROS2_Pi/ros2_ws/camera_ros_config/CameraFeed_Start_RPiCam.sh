#!/bin/bash
# This script starts the custom RPi camera node using GStreamer with libcamera
# This avoids the libcamera IPA proxy issues with camera_ros and rpicam-vid symbol issues
# Auto-installs all required dependencies for reliable camera streaming

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

######################################################################
# --- Configuring Camera Parameters ---
######################################################################
TARGET_FPS=15 # Reduced FPS for stability - high FPS causes JPEG corruption
WIDTH=800
HEIGHT=600
FORMAT="rgb"  # rgb or bgr
PUBLISH_COMPRESSED=true  # Set to true to publish compressed images
JPEG_QUALITY=75  # Reduced quality for less CPU load (1-100, higher = better quality, larger size)
######################################################################

echo "=== RPi Camera Publisher Setup & Start ==="
echo "Camera settings: ${WIDTH}x${HEIGHT} @ ${TARGET_FPS}fps, format: ${FORMAT}"
echo "Compressed images: ${PUBLISH_COMPRESSED}, JPEG quality: ${JPEG_QUALITY}%"
echo

# Function to check if a package is installed
check_package_installed() {
    dpkg -l | grep -q "^ii  $1 " 2>/dev/null
}

# Function to install packages with error checking
install_packages() {
    local packages=("$@")
    local missing_packages=()
    
    # Check which packages are missing
    for package in "${packages[@]}"; do
        if ! check_package_installed "$package"; then
            missing_packages+=("$package")
        fi
    done
    
    # Install missing packages
    if [ ${#missing_packages[@]} -gt 0 ]; then
        echo "Installing missing packages: ${missing_packages[*]}"
        if sudo apt update && sudo apt install -y "${missing_packages[@]}"; then
            echo "✓ Successfully installed: ${missing_packages[*]}"
        else
            echo "✗ Failed to install packages: ${missing_packages[*]}"
            echo "Please install manually: sudo apt install ${missing_packages[*]}"
            exit 1
        fi
    else
        echo "✓ All required packages already installed"
    fi
}

# Make sure the Python script is executable
chmod +x "$SCRIPT_DIR/rpicam_publisher.py"

echo "=== Checking System Dependencies ==="

# Core system packages required for camera streaming
SYSTEM_PACKAGES=(
    "gstreamer1.0-tools"        # GStreamer command line tools
    "gstreamer1.0-plugins-base" # Basic GStreamer plugins
    "gstreamer1.0-plugins-good" # Good quality GStreamer plugins
    "gstreamer1.0-plugins-bad"  # Additional plugins (may help with stability)
    "gstreamer1.0-libcamera"    # GStreamer libcamera plugin
    "libcamera-tools"           # libcamera utilities (cam, etc.)
    "libcamera-ipa"             # IPA libraries for camera sensors (CRITICAL!)
    "python3-opencv"            # OpenCV for Python
    "v4l-utils"                 # V4L2 utilities for debugging
)

# ROS2 packages (check if ROS2 is installed first)
if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 detected, checking ROS2 camera packages..."
    ROS2_PACKAGES=(
        "ros-jazzy-cv-bridge"       # OpenCV-ROS bridge
        "ros-jazzy-image-transport" # Image transport plugins
    )
    SYSTEM_PACKAGES+=("${ROS2_PACKAGES[@]}")
else
    echo "⚠ ROS2 not detected - skipping ROS2 packages"
fi

# Install all required packages
install_packages "${SYSTEM_PACKAGES[@]}"

echo
echo "=== Verifying Critical Components ==="

# Check GStreamer
if command -v gst-launch-1.0 &> /dev/null; then
    echo "✓ GStreamer found: $(gst-launch-1.0 --version | head -n1)"
else
    echo "✗ gst-launch-1.0 not found"
    exit 1
fi

# Check libcamera GStreamer plugin
if gst-inspect-1.0 libcamerasrc &> /dev/null; then
    echo "✓ libcamerasrc plugin available"
else
    echo "✗ libcamerasrc plugin not found"
    echo "Try: sudo apt install gstreamer1.0-libcamera"
    exit 1
fi

# Check libcamera tools
if command -v cam &> /dev/null; then
    echo "✓ libcamera tools found"
else
    echo "⚠ libcamera tools not found (optional)"
fi

# Check if IPA libraries are available
if [ -d "/usr/lib/*/libcamera/" ] && [ -n "$(find /usr/lib -name "*ipa*.so" 2>/dev/null)" ]; then
    echo "✓ IPA libraries found"
else
    echo "⚠ IPA libraries not found - camera may not work properly"
    echo "Install with: sudo apt install libcamera-ipa"
fi

# Check Python dependencies
echo "=== Checking Python Dependencies ==="
PYTHON_MODULES=("cv2" "cv_bridge" "rclpy" "sensor_msgs")
MISSING_PYTHON_MODULES=()

for module in "${PYTHON_MODULES[@]}"; do
    if python3 -c "import $module" 2>/dev/null; then
        echo "✓ Python module '$module' available"
    else
        echo "✗ Python module '$module' missing"
        MISSING_PYTHON_MODULES+=("$module")
    fi
done

if [ ${#MISSING_PYTHON_MODULES[@]} -gt 0 ]; then
    echo "⚠ Some Python modules are missing: ${MISSING_PYTHON_MODULES[*]}"
    echo "This may cause runtime errors. Install missing ROS2 packages if needed."
fi

echo
echo "=== Hardware Detection ==="

# Check camera hardware detection
if [ -e "/dev/video0" ]; then
    echo "✓ Video devices found: $(ls /dev/video* | wc -l) devices"
else
    echo "⚠ No video devices found in /dev/"
fi

# Reset camera hardware to clear any stuck states
echo "Resetting camera hardware..."
if [ -d "/sys/class/gpio/gpio5" ]; then
    echo "Toggling camera reset GPIO..."
    echo 0 | sudo tee /sys/class/gpio/gpio5/value > /dev/null 2>&1
    sleep 0.5
    echo 1 | sudo tee /sys/class/gpio/gpio5/value > /dev/null 2>&1
    sleep 1
fi

# Kill any existing libcamera processes that might be stuck
echo "Cleaning up any stuck camera processes..."
sudo pkill -f "libcamerasrc" 2>/dev/null || true
sudo pkill -f "gst-launch" 2>/dev/null || true
sleep 2

# Test camera detection with libcamera
echo "Testing camera detection..."
if timeout 10s cam --list 2>/dev/null | grep -q "Available cameras" && 
   ! timeout 10s cam --list 2>&1 | grep -q "ERROR.*Failed to register camera"; then
    echo "✓ Camera detected by libcamera"
else
    echo "⚠ Camera not detected or has errors"
    echo "Common solutions:"
    echo "  1. Ensure camera cable is properly connected"
    echo "  2. Check if camera is enabled in /boot/firmware/config.txt"
    echo "  3. Reboot after installing libcamera-ipa"
    echo "  4. Try: sudo modprobe bcm2835-v4l2"
    echo "  5. Check dmesg for camera errors: dmesg | grep -i camera"
fi

echo
echo "=== Starting Camera Publisher ==="

# Final check before starting
if ! command -v gst-launch-1.0 &> /dev/null || ! gst-inspect-1.0 libcamerasrc &> /dev/null; then
    echo "✗ Critical dependencies missing - cannot start camera"
    exit 1
fi

echo "All checks passed! Starting RPi Camera Publisher..."
echo "Press Ctrl+C to stop the camera feed"
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
#!/bin/bash
# Camera troubleshooting and diagnostic script

echo "=== Camera System Diagnostics ==="
echo "Date: $(date)"
echo

echo "=== System Information ==="
echo "OS: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"
echo

echo "=== Camera Hardware Detection ==="
echo "Video devices:"
ls -la /dev/video* 2>/dev/null || echo "No video devices found"
echo

echo "=== libcamera Status ==="
echo "Testing camera detection..."
timeout 10s cam --list 2>&1 || echo "Camera detection failed or timed out"
echo

echo "=== GStreamer Information ==="
echo "GStreamer version:"
gst-launch-1.0 --version 2>/dev/null || echo "GStreamer not found"
echo

echo "libcamerasrc plugin status:"
gst-inspect-1.0 libcamerasrc >/dev/null 2>&1 && echo "✓ Available" || echo "✗ Not available"
echo

echo "=== Process Information ==="
echo "Camera-related processes:"
ps aux | grep -E "(gst-launch|libcamera|rpicam)" | grep -v grep || echo "No camera processes running"
echo

echo "=== Memory Usage ==="
free -h
echo

echo "=== GPU Memory (if available) ==="
if command -v vcgencmd &> /dev/null; then
    echo "GPU memory split: $(vcgencmd get_mem gpu)"
    echo "GPU temperature: $(vcgencmd measure_temp)"
else
    echo "vcgencmd not available (not on Raspberry Pi)"
fi
echo

echo "=== Recent System Logs ==="
echo "Recent camera-related kernel messages:"
dmesg | tail -20 | grep -i -E "(camera|libcamera|v4l2)" || echo "No recent camera messages"
echo

echo "=== Service Status ==="
echo "Camera service status:"
systemctl status ros2_camera_feed.service --no-pager -l || echo "Service not found"
echo

echo "=== Recent Service Logs ==="
echo "Last 20 lines of camera service logs:"
journalctl -u ros2_camera_feed.service -n 20 --no-pager || echo "No service logs available"
echo

echo "=== Network Interfaces ==="
ip addr show | grep -E "^[0-9]+:|inet " || echo "No network information available"
echo

echo "=== Disk Space ==="
df -h / /tmp || echo "Disk space check failed"
echo

echo "=== Environment Variables ==="
echo "Relevant environment variables:"
env | grep -E "(LIBCAMERA|GST|ROS)" | sort || echo "No relevant environment variables"
echo

echo "=== Configuration Files ==="
echo "Boot config (if available):"
if [ -f "/boot/firmware/config.txt" ]; then
    echo "Camera settings in config.txt:"
    grep -i camera /boot/firmware/config.txt 2>/dev/null || echo "No camera settings found"
elif [ -f "/boot/config.txt" ]; then
    echo "Camera settings in config.txt:"
    grep -i camera /boot/config.txt 2>/dev/null || echo "No camera settings found"
else
    echo "Boot config file not found"
fi
echo

echo "=== Recommendations ==="
echo "If camera is not working:"
echo "1. Check camera cable connection"
echo "2. Ensure camera is enabled in boot config"
echo "3. Try rebooting the system"
echo "4. Check for conflicting processes using the camera"
echo "5. Verify libcamera-ipa package is installed"
echo "6. Check GPU memory split (should be at least 128MB for Pi)"
echo

echo "=== Quick Fixes ==="
echo "To kill stuck camera processes:"
echo "sudo pkill -f 'gst-launch|libcamera|rpicam'"
echo
echo "To restart camera service:"
echo "sudo systemctl restart ros2_camera_feed.service"
echo
echo "To reset camera hardware (Pi only):"
echo "echo 0 | sudo tee /sys/class/gpio/gpio5/value; sleep 1; echo 1 | sudo tee /sys/class/gpio/gpio5/value"
echo

echo "=== End Diagnostics ==="

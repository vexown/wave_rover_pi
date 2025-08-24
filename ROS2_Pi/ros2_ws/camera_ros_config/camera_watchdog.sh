#!/bin/bash
# Camera service watchdog script
# This script monitors the camera service and restarts it if it fails
# Run this as a separate systemd service or cron job

SERVICE_NAME="ros2_camera_feed.service"
MAX_RESTART_ATTEMPTS=3
RESTART_DELAY=30  # seconds between restart attempts
LOG_FILE="/var/log/camera_watchdog.log"

# Function to log messages
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$LOG_FILE"
}

# Function to check if service is running and healthy
check_service_health() {
    # Check if service is active
    if ! systemctl is-active --quiet "$SERVICE_NAME"; then
        return 1
    fi
    
    # Check if there are recent fatal errors in the journal
    if journalctl -u "$SERVICE_NAME" --since "5 minutes ago" | grep -q "FATAL\|assertion.*failed"; then
        return 1
    fi
    
    return 0
}

# Function to restart service with camera hardware reset
restart_service() {
    log_message "Restarting $SERVICE_NAME..."
    
    # Stop the service
    systemctl stop "$SERVICE_NAME"
    
    # Kill any stuck processes
    pkill -f "gst-launch" || true
    pkill -f "libcamerasrc" || true
    pkill -f "rpicam_publisher" || true
    
    # Reset camera GPIO if available (RPi specific)
    if [ -d "/sys/class/gpio/gpio5" ]; then
        echo 0 > /sys/class/gpio/gpio5/value 2>/dev/null || true
        sleep 1
        echo 1 > /sys/class/gpio/gpio5/value 2>/dev/null || true
    fi
    
    # Wait for hardware reset
    sleep 5
    
    # Restart the service
    systemctl start "$SERVICE_NAME"
    
    # Wait and check if it started successfully
    sleep 10
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        log_message "Service restarted successfully"
        return 0
    else
        log_message "Service failed to restart"
        return 1
    fi
}

# Main monitoring loop
restart_count=0
log_message "Camera watchdog started for $SERVICE_NAME"

while true; do
    if ! check_service_health; then
        log_message "Service unhealthy detected"
        
        if [ $restart_count -lt $MAX_RESTART_ATTEMPTS ]; then
            restart_count=$((restart_count + 1))
            log_message "Attempt $restart_count/$MAX_RESTART_ATTEMPTS to restart service"
            
            if restart_service; then
                restart_count=0  # Reset counter on successful restart
                log_message "Service recovered successfully"
            else
                log_message "Restart attempt $restart_count failed"
            fi
            
            sleep $RESTART_DELAY
        else
            log_message "Maximum restart attempts ($MAX_RESTART_ATTEMPTS) reached. Manual intervention required."
            # Send notification or alert here if needed
            sleep 300  # Wait 5 minutes before trying again
            restart_count=0  # Reset counter after longer wait
        fi
    else
        # Service is healthy, reset restart counter
        restart_count=0
    fi
    
    # Check every 30 seconds
    sleep 30
done

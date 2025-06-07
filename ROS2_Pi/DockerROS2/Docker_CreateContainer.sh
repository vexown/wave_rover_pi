
# Check if Docker is installed
if ! command -v docker &> /dev/null
then
    echo "Docker is not installed. Please install Docker first (with Docker_Install.sh)."
    exit 1
fi

# Check if the container already exists
if [ "$(sudo docker ps -a -q -f name=ros2_dev)" ]; then
    echo "Container 'ros2_dev' already exists. Please remove it first (with Docker_RemoveContainer.sh)."
    exit 1
fi

# Pull the latest ROS 2 image (Jazzy)
sudo docker pull ros:jazzy-ros-core

# Create and run a new Docker container named "ros2_dev" from the "ros:jazzy-ros-base" image.
# This container is configured for ROS 2 development, especially on systems like Raspberry Pi,
# by providing access to host hardware and network.
#
# Options explained:
#   -it                    : Allocates a pseudo-TTY and keeps STDIN open, allowing interactive shell access.
#   --name=ros2_dev        : Assigns the name "ros2_dev" to the container for easy management.
#   --privileged           : Gives the container extended privileges, often required for direct hardware access.
#   --network host         : Uses the host's network stack, allowing the container to share the host's IP address
#                            and network interfaces. This simplifies ROS 2 discovery and communication.
#   -v /tmp/.X11-unix:/tmp/.X11-unix : Mounts the host's X11 socket to allow GUI applications from the container
#                                      to display on the host's screen.
#   -v ~/ros2_ws:/root/ros2_ws : Mounts the '~/ros2_ws' directory from the host to '/root/ros2_ws' in the container.
#                                This allows persistent storage and development of ROS 2 workspaces.
#   --device /dev/vchiq:/dev/vchiq : Maps the VideoCore Host Interface Quality of Service device (Raspberry Pi).
#   --device /dev/video0:/dev/video0 : Maps the primary camera device.
#   --device /dev/ttyS0:/dev/ttyS0   : Maps the primary serial port (UART).
#   --device /dev/i2c-1:/dev/i2c-1   : Maps the I2C bus 1.
#   --device /dev/spidev0.0:/dev/spidev0.0 : Maps SPI device 0.0.
#   --device /dev/spidev0.1:/dev/spidev0.1 : Maps SPI device 0.1.
#   --device /dev/gpiochip0:/dev/gpiochip0 : Maps the GPIO controller, allowing GPIO pin access.
#   ros:jazzy-ros-base     : Specifies the Docker image to use for creating the container.
#
# The purpose of this command is to set up an isolated, reproducible ROS 2 development environment
# with necessary hardware access and GUI capabilities, making it suitable for robotics projects
# that interact with the physical world.
sudo docker run -it --name=ros2_dev --privileged \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/ros2_ws:/root/ros2_ws \
    --device /dev/vchiq:/dev/vchiq \
    --device /dev/video0:/dev/video0 \
    --device /dev/ttyS0:/dev/ttyS0 \
    --device /dev/i2c-1:/dev/i2c-1 \
    --device /dev/spidev0.0:/dev/spidev0.0 \
    --device /dev/spidev0.1:/dev/spidev0.1 \
    --device /dev/gpiochip0:/dev/gpiochip0 \
    ros:jazzy-ros-base

# Note: The above command will run the container interactively. To exit the container, use Ctrl + C or type 'exit'.

echo "Remember to run the following command to source the ROS 2 environment in the container:"
echo "source /opt/ros/jazzy/setup.bash >> ~/.bashrc "




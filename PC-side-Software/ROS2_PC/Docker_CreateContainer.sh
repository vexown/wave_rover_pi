
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

# Allow the root user on the local machine (which includes processes within the Docker container,
# especially if they run as root) to connect to the host's X server.
# This is necessary for GUI applications (e.g., RViz, Gazebo) running inside the Docker container
# to display their windows on the host machine's screen.
# TODO: Consider more secure alternatives for X11 forwarding, such as using xauth with specific cookies
# or running the container with a user that matches the host's UID/GID to avoid needing broad root access to X.
xhost +local:root

# Pull the latest ROS 2 image (Jazzy)
sudo docker pull ros:jazzy-ros-core

# Create and run a new Docker container named "ros2_dev" from the "ros:jazzy-ros-base" image.
#
# -it:
#   - '-i' (interactive): Keeps STDIN (standard input) open even if not attached. This allows you to send input to the container.
#   - '-t' (tty): Allocates a pseudo-terminal (TTY), essentially connecting your current terminal to the container's terminal.
#   - Combined, '-it' gives you an interactive shell session inside the container.
#
# --name=ros2_dev:
#   - Assigns a human-readable name "ros2_dev" to the container. This makes it easier to manage the container later
#     (e.g., 'docker start ros2_dev', 'docker stop ros2_dev').
#
# --privileged:
#   - Grants the container extended privileges on the host machine. This can be necessary for tasks like accessing
#     hardware devices directly, which is sometimes required in robotics applications.
#
# --network host:
#   - Configures the container to use the host machine's network stack. This means the container shares the host's
#     IP address and network interfaces, simplifying network communication between processes running inside
#     the container and processes running on the host or other devices on the same network.
#
# -v /tmp/.X11-unix:/tmp/.X11-unix:
#   - '-v' (volume): Mounts a directory or file from the host into the container.
#   - This specific mount shares the host's X11 Unix socket with the container. X11 is the system used by Linux
#     to display graphical user interfaces (GUIs). This allows GUI applications running inside the container
#     (like RViz or Gazebo) to display on your host machine's screen.
#
# -v ~/ros2_ws:/root/ros2_ws:
#   - Another volume mount.
#   - '~/ros2_ws': This is a path on your host machine, typically your ROS 2 workspace directory located in your home directory.
#   - ':/root/ros2_ws': This maps the host's '~/ros2_ws' directory to '/root/ros2_ws' inside the container.
#   - This is crucial for development as it allows you to edit code in your workspace on your host machine using your
#     preferred editor, and those changes are immediately reflected inside the container where you compile and run your ROS 2 nodes.
#
# -e DISPLAY=$DISPLAY:
#   - '-e' (environment): Sets an environment variable inside the container.
#   - 'DISPLAY=$DISPLAY': This passes the value of the host's 'DISPLAY' environment variable to the container.
#   - The 'DISPLAY' variable tells applications where to send their graphical output. Combined with the X11 socket mount,
#     this ensures that GUI applications inside the container know which screen to display on (your host's screen).
#
# ros:jazzy-ros-base:
#   - This is the name of the Docker image to use for creating the container.
#   - 'ros': Refers to the official ROS images available on Docker Hub.
#   - 'jazzy-ros-base': Specifies the tag of the image. 'jazzy' indicates the ROS 2 Jazzy Jalisco distribution,
#     and 'ros-base' is a minimal installation of ROS (core libraries, build tools, communication tools, but no GUI tools pre-installed).
sudo docker run -it --name=ros2_dev --privileged \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/ros2_ws:/root/ros2_ws \
    -e DISPLAY=$DISPLAY \
    ros:jazzy-ros-base

# Note: The above command will run the container interactively. To exit the container, use Ctrl + C or type 'exit'.

echo "Remember to run the following command to source the ROS 2 environment in the container:"
echo "source /opt/ros/jazzy/setup.bash >> ~/.bashrc "

#TODO: Consider adding a command to automatically source the ROS 2 environment in the container's ~/.bashrc file.
# Also add these env variables to the container:
# echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
# echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
# echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
# echo "export ROS_DISCOVERY_SERVER=" >> ~/.bashrc

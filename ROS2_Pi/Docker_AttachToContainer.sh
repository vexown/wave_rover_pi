
# This script is used to attach to already running Docker container with the name "ros2_dev"
# Check if Docker is installed
if ! command -v docker &> /dev/null
then
    echo "Docker is not installed. Please install Docker first (with Docker_Install.sh)."
    exit 1
fi
# Check if the container exists
if [ "$(sudo docker ps -a -q -f name=ros2_dev)" ]; then
    echo "Container 'ros2_dev' exists."
else
    echo "Container 'ros2_dev' does not exist. Please create it first (with Docker_CreateContainer.sh)."
    exit 1
fi

# Start the container and attach to it
echo "Attaching to the container 'ros2_dev'..."

# Connect to the already running Docker container named 'ros2_dev'.
# This command will attach your current terminal's input, output, and error streams
# to the container's main process, allowing you to interact with it.
# If you want to detach without stopping the container, use Ctrl+P then Ctrl+Q.
sudo docker attach ros2_dev
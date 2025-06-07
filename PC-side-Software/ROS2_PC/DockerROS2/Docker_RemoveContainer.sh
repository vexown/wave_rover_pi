
# Check if Docker is installed
if ! command -v docker &> /dev/null
then
    echo "Docker is not installed. Please install Docker first (with Docker_Install.sh)."
    exit 1
fi

# Check if the container is running
if [ "$(sudo docker ps -q -f name=ros2_dev)" ]; then
    # Stop the container if it is running
    echo "Stopping the container..."
    sudo docker stop ros2_dev
else
    echo "Container is not running."
fi
# Remove the container
echo "Removing the container..."
sudo docker rm ros2_dev
echo "Container removed successfully."
# Note: The above command will remove the container named "ros2_dev". If you want to remove all stopped containers, you can use:
# sudo docker container prune
# This command will prompt for confirmation before removing all stopped containers.



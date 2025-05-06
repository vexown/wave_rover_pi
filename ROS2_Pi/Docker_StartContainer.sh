
# This script is used to start a Docker container with the name "ros2_dev"
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
# Check if the container is running
if [ "$(sudo docker ps -q -f name=ros2_dev)" ]; then
    echo "Container 'ros2_dev' is already running."
    exit 0
fi
# Start the container and attach to it
echo "Starting the container 'ros2_dev'..."

# 'start' is a Docker subcommand that instructs the Docker daemon to start one or more stopped containers.
#
# -a or --attach: This is an option for the docker start command. It means "attach STDOUT/STDERR and forward 
# signals". In simpler terms, it connects your terminal's standard output (where regular output is displayed) 
# and standard error (where error messages are displayed) to the running container's output and also forwards 
# any signals (like Ctrl+C) you send from your terminal to the main process running inside the container.
# 
# -i or --interactive: This is another option for the docker start command. It means "attach container's STDIN". 
# STDIN stands for standard input, which is typically your keyboard. This option allows you to send input to 
# the main process running inside the container. This is essential if you want to interact with a shell or 
# any interactive application running within the container.
sudo docker start -ai ros2_dev
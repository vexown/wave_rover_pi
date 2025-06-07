
# This script is used to start a new terminal session inside a running Docker container named 'ros2_dev'.
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

echo "Starting a new terminal in the container 'ros2_dev'..."

# Execute a new command (in this case, 'bash' to start a new shell)
# within the running Docker container named 'ros2_dev'.
#
# sudo:  Execute the docker command with superuser privileges (if required).
# docker exec:  Run a command inside a running container.
# -it:   -i (interactive): Keep STDIN open, allowing you to send input to the shell.
#        -t (tty): Allocate a pseudo-TTY, providing a terminal-like environment.
# ros2_dev: The name of the Docker container where the command will be executed.
# bash:    The command to run inside the container (starts a new Bash shell).
#
# This command is useful for getting a separate, interactive shell session
# within the container, independent of the container's main process,
# allowing you to run additional commands or perform debugging.
sudo docker exec -it ros2_dev bash
# Only run this once to install Docker on your Raspberry Pi 4B running Debian Bookworm.

# Check if the OS is Debian
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" != "debian" ]; then
        echo "This script is intended for Debian only. Exiting."
        exit 1
    fi
else
    echo "Cannot determine operating system. This script is intended for Debian only. Exiting."
    exit 1
fi

# Check if Docker is already installed:
# If you have already installed Docker, this script will not run again.
if command -v docker &> /dev/null
then
    echo "Docker is already installed."
    exit 0
fi

# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update

# Install Docker Engine, CLI, and Containerd:
# Note: The following command will install the latest version of Docker.
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify that Docker Engine is installed correctly by running the hello-world image:
sudo docker run hello-world
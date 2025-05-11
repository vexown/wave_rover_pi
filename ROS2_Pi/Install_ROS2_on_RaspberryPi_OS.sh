# Source: https://github.com/Ar-Ray-code/rpi-bullseye-ros2
# Adapted for Raspberry Pi OS Bookworm (arm64) and ROS2 Jazzy
# To check your Raspberry Pi OS version, you can use the command: cat /etc/os-release

echo "Starting ROS2 Jazzy installation for Raspberry Pi OS Bookworm (arm64)..."

# Update package lists
sudo apt update
sudo apt upgrade -y

# Install dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget

# Download ROS2 Jazzy desktop package
echo "Downloading ROS2 Jazzy desktop package..."
wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb

# Install the downloaded package
echo "Installing ROS2 Jazzy desktop package..."
sudo apt install -y ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb

# Install Python packages using pip
echo "Installing Python packages using pip..."
pip install --break-system-packages empy==3.3.4
sudo pip install --break-system-packages vcstool colcon-common-extensions

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# Source ROS2 setup file
echo "Adding ROS2 setup to .bashrc..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Clean up downloaded package
rm ros-jazzy-desktop-0.3.2_20240525_arm64.deb

echo "ROS2 Jazzy installation complete."
echo "Please close and reopen your terminal or run 'source ~/.bashrc' to apply changes."
echo "Remember: When using Rviz2, switch the display server from Wayland to X11."
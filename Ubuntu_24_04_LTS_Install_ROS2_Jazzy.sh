#
# Make sure you have a locale which supports UTF-8
#
# locale  # check for UTF-8
#
# If you see something like "en_US.UTF-8" in the output, you're good to go.
# If not, you may need to generate the locale.
#
# sudo apt update && sudo apt install locales
# sudo locale-gen en_US en_US.UTF-8
# sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8
#
# locale  # verify settings
#


# Ensure Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools (opptional but recommended)
sudo apt update && sudo apt install ros-dev-tools

# Install ROS2 packages
echo "Select the type of ROS2 Jazzy installation:"
options=("Raspberry Pi (headless)" "PC (desktop)")
PS3="Enter your choice (1 or 2): "
select opt in "${options[@]}"
do
    case $opt in
        "Raspberry Pi (headless)")
            sudo apt update && sudo apt install ros-jazzy-ros-base
            break
            ;;
        "PC (desktop)")
            sudo apt update && sudo apt install ros-jazzy-desktop
            break
            ;;
        *) echo "Invalid option $REPLY. Please enter 1 or 2.";;
    esac
done

# Set up the environment (adding this to ~/.bashrc to make it permanent)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc



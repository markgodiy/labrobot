## Install ROS 2 Jazzy on Ubuntu 24.04

Ros2 Jazzy install guide:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#

```sh
# Ensure UTF-8 locale is configured
sudo apt update
sudo apt install -y locales software-properties-common curl lsb-release gnupg
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable universe repo
sudo add-apt-repository universe -y

# Add ROS 2 APT source
ROS_APT_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Add Gazebo Harmonic APT source and key (for Ubuntu 24.04 "noble")
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Cleanup old/harmonic-specific lists
sudo rm -f /etc/apt/sources.list.d/gazebo-harmonic.list

# Update system
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Jazzy, dev tools, and Gazebo Harmonic
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  gz-harmonic \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Add ROS environment to shell startup
if ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if [ -f ~/.zshrc ] && ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.zshrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.zshrc
fi

# Source immediately for current session
source /opt/ros/jazzy/setup.bash


# Final cleanup
sudo apt autoremove -y
```


## Raspberry Pi 4b Setup

Use the script below to install ROS 2 Jazzy core and Gazebo Harmonic on a Raspberry Pi 4b running Ubuntu 24.04 LTS.

The main difference is we're not installing the desktop version, just the core ROS 2 packages and Gazebo.

```sh
# Ensure UTF-8 locale is configured
sudo apt update
sudo apt install -y locales software-properties-common curl lsb-release gnupg
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable universe repo
sudo add-apt-repository universe -y

# Add ROS 2 APT source
ROS_APT_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VERSION}/ros2-apt-source_${ROS_APT_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Add Gazebo Harmonic APT source and key (for Ubuntu 24.04 "noble")
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Cleanup old/harmonic-specific lists
sudo rm -f /etc/apt/sources.list.d/gazebo-harmonic.list

# Update system
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Jazzy core, dev tools, and Gazebo Harmonic
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  gz-harmonic \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Add ROS environment to shell startup
if ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if [ -f ~/.zshrc ] && ! grep -Fxq "source /opt/ros/jazzy/setup.bash" ~/.zshrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.zshrc
fi

# Source immediately for current session
source /opt/ros/jazzy/setup.bash

# Final cleanup
sudo apt autoremove -y
```

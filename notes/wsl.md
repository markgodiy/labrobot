## Install WSL Ubuntu 24.04 on Windows 11


```bash
wsl --install -d Ubuntu-24.04
```

This command installs the Ubuntu 24.04 distribution on WSL (Windows Subsystem for Linux).

## Optional: Export to another drive

Initial WSL install needed about ~1.6GB. After installing ROS, Gazebo, and dependencies, total size grew to almost 8GB.

Since I dont have enough space on my C:\ drive, I need to export the WSL Ubuntu 24.04 distribution to another drive, in this case, D:\WSL\Ubuntu2404. Here are the steps to do that:

```bash
mkdir D:\WSL\Ubuntu2404
```

```bash
# as Administrator in PowerShell
wsl --shutdown
wsl --list --verbose
wsl --export Ubuntu-24.04 D:\WSL\Ubuntu2404\ext4.vhdx --vhd
wsl --unregister Ubuntu-24.04
wsl --import Ubuntu-24.04 D:\WSL\Ubuntu2404 D:\WSL\Ubuntu2404\ext4.vhdx --version 2
```

I also had to fix the Ubuntu profile icon in Windows Terminal, else I kept getting an error. Likely since I moved the ubuntu distribution to another drive, the icon path was no longer valid. I fixed it by going to Windows Terminal settings, finding the Ubuntu profile, and just copying the icon path from the Ubuntu profile in Windows Terminal settings to the Ubuntu profile that I moved to another drive:

`https://assets.ubuntu.com/v1/49a1a858-favicon-32x32.png`

## Install script:

This is the install script I used to set up ROS2 Jazzy and Gazebo Harmonic on WSL Ubuntu 24.04. It installs the necessary packages, sets up the ROS2 environment.

```bash
# Ensure UTF-8 locale is configured
if ! locale | grep -q "UTF-8"; then
  sudo apt update
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi

# Add Universe repo
sudo add-apt-repository -y universe

# Install base packages
sudo apt install -y software-properties-common curl git python3-pip python3-venv python3-dev build-essential

# Add APT sources: ROS 2 Jazzy + Gazebo Harmonic
ROS_DISTRO=jazzy

# Add ROS 2 apt source
sudo apt update
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Add Gazebo Harmonic repo and GPG key
curl -fsSL https://packages.osrfoundation.org/gazebo.key | sudo tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-harmonic $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-harmonic.list

# Update once after adding both sources
sudo apt update
sudo apt upgrade -y

# Install ROS 2 Jazzy + tools + bridge + Gazebo transport
sudo apt install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-${ROS_DISTRO}-ros-gz \
  libgz-transport15-dev

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Source ROS 2 Jazzy in shell (once)
if ! grep -Fxq "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi
# If you use zsh, also add to ~/.zshrc
test -f ~/.zshrc && (grep -Fxq "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.zshrc || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.zshrc)
source /opt/ros/${ROS_DISTRO}/setup.bash

# Final cleanup
sudo apt autoremove -y
```

# Quick Check of WSL, ROS2, and Gazebo installation

Getting the version of ROS, Gazebo and other info about the system

```bash
lsb_release -a
uname -a
echo $ROS_DISTRO
gz sim --version
```

Example output:

```bash
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.2 LTS
Release:        24.04
Codename:       noble
Linux AURORA 6.6.87.2-microsoft-standard-WSL2 #1 SMP PREEMPT_DYNAMIC Thu Jun  5 18:30:46 UTC 2025 x86_64 x86_64 x86_64 GNU/Linux
jazzy
Gazebo Harmonic, version 11.0.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License.
```

## ROS2 Doctor Report

After install, run the following command to check the ROS2 installation and generate a report:

```bash
ros2 doctor --report
```

This command will generate a report of the ROS2 installation:
- Network configuration
- Package versions
- Platform information
- QOS Compatibility List
- RMW Middleware
- ROS2 Information
- Topic List

## Test Gazebo installation:

```bash
gz sim -v 4
```

if getting this error: QStandardPaths: wrong permissions on runtime directory /run/user/1000/, 0755 instead of 0700

This is caused by the fact that WSL2 runs as root, and the default permissions for the /run/user/1000 directory are too open. Fix it by running the following command:

```bash
chmod 700 /run/user/1000 to fix it
```

# Workspace setup

 I prefer to edit the src from my windows host, 
 I keep my code in the G: drive from my Windows host,
 so I mounted the G: drive to /mnt/g
 but create a symbolic link to the src directory in my ROS workspace

```bash
# mount g:\ to /mnt/g
sudo mkdir -p /mnt/g
sudo mount -t drvfs G: /mnt/g
# Add mount command to fstab for persistence
echo "G: /mnt/g drvfs defaults 0 0" | sudo tee -a /etc/fstab

mkdir -p ~/dev_ws
ln -s /mnt/g/dev_ws/src ~/dev_ws/src
cd ~/dev_ws
colcon build
```

## Sync src directory with Windows host
To sync the src directory with your Windows host, you can use the following command:

```bash
rsync -av /mnt/g/dev_ws/src /home/botmanager/dev_ws/
```

## Notes:

to check hardware interfaces, joints and sensors, you can use the following command:

```bash
 ros2 param get /robot_state_publisher robot_description | grep joint
```


## ROS2 topics:

to list all topics, you can use:

```bash
ros2 topic list
```

to echo the cmd_vel topic for a specific model, you can use:

```bash
ros2 topic echo /model/my_bot/cmd_vel
```


## Gazebo topics

```bash
gz topic -l
```

to echo a specific topic, you can use:

```bash
gz topic -e /model/my_bot/cmd_vel
```

## Worlds

Seems gz sim does not automatically create a ground plane with physics, so my robot is floating in the air and would fall straight down if I run the simulation.

To create a ground plane, I referenced the ros_gz example vehicle.sdf, fed it to chatgpt, then modified it to create an empty world with ground plane and a skybox:

https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim_demos/worlds/vehicle.sdf

then, I modified the launch_sim.launch.py file to use the empty world with ground plane and skybox:

Adding:
```python
from launch.substitutions import LaunchConfiguration
```

```python
    world_file = LaunchConfiguration('world') # <-- World file to load
```

```python
    # Start Gazebo Ionic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_file],
        output='screen'
    )
```

Then I specify the world file with the ros2 launch command:

```bash
ros2 launch my_bot launch_sim.launch.py world:=src/my_bot/worlds/empty.world
```

Alternatively, I could've also hardcoded the world file path in the launch file, but using `LaunchConfiguration` was a new method with potential usefulness in the future, so why not.

## Moving the robot sim in Gazebo

The original guide used teleops keyboard to move the robot in Gazebo, but it wouldnt work in WSL2. I couldnt find any good documentation on why it wasnt working and if there are any workarounds. For now, chatgpt suggested manually pubshing to the cmd_vel topic to move the robot in Gazebo:

```bash
ros2 topic pub /model/my_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

This command publishes a looping Twist message to the cmd_vel topic, which moves the robot forward at 0.2 m/s and rotates it at 0.3 rad/s, essentially moving it in a circle.

To move it in a square-ish route, I can use the following commands:

```bash
for i in {1..4}
do
  echo "Moving forward..."
  ros2 topic pub --once /model/my_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
  sleep 3

  echo "Turning..."
  ros2 topic pub --once /model/my_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.6}}"
  sleep 2
done

# Final stop
echo "Stopping..."
ros2 topic pub --once /model/my_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

```

to move faster, increase the linear velocity in the Twist message, e.g., change `x: 0.5` to `x: 1.0` for faster forward movement, and adjust the angular velocity as needed for turning speed. The unit is meters per second for linear velocity and radians per second for angular velocity.


This script will move the robot forward for 3 seconds, then turn it for 2 seconds, repeating this process 4 times to create a square-ish route. Then it stops the robot by publishing a zero velocity command.

These commands will have to do for now while I get ready for incorporating the robot's sensors and control logic, which will allow for more sophisticated movement and navigation.



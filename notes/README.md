# LabRobot Project


LabRobot Project 

Hardware:

# Labrobot Project Notes


## 2025-07-04 
- Restarting project, targeting ROS2 Jazzy LTS and Gazebo Harmonic LTS. 
- Jazzy Jalisco (May 2024 - May 2029)
- Gazebo Harmonic (September 2023 - September 2028)

```sh
$ ros2 doctor --report
...
   PLATFORM INFORMATION
system           : Linux
platform info    : Linux-6.6.87.2-microsoft-standard-WSL2-x86_64-with-glibc2.39
release          : 6.6.87.2-microsoft-standard-WSL2
processor        : x86_64

   QOS COMPATIBILITY LIST
compatibility status    : No publisher/subscriber pairs found

   RMW MIDDLEWARE
middleware name    : rmw_fastrtps_cpp

   ROS 2 INFORMATION
distribution name      : jazzy
distribution type      : ros2
distribution status    : active
release platforms      : {'debian': ['bookworm'], 'rhel': ['9'], 'ubuntu': ['noble']}

   TOPIC LIST
topic               : none
publisher count     : 0
subscriber count    : 0
```

```sh
$ gz sim --version
Gazebo Sim, version 8.9.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 Licens
```

### `colcon build --symlink-install`

`colcon` is the recommended build tool for ROS 2 projects. It supports building multiple packages in a workspace and handles dependencies automatically.

`colcon build` is the command used to build all packages in the current workspace. It will compile the source code, generate necessary files, and prepare the packages for use. However, this command will need to be run each time you make changes to the source code or add new packages.

**Recommended** `colcon build --symlink-install` instead creates symbolic links to the source files instead of copying them into the build directory. This allows for faster development since changes made in the source files are immediately reflected in the build without needing to rebuild the entire package.

## Common Issues and Resolutions

### 1. RTPS_TRANSPORT_SHM Error in WSL or Container

**Issue:**
```
[robot_state_publisher-1] [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7000: open_and_lock_file failed -> Function open_port_internal
```
**Resolution:**
This is a warning from Fast DDS about shared memory transport not being available (common in WSL or containers). It does not prevent ROS 2 from working. To suppress the warning, disable shared memory transport:
```sh
export RMW_FASTRTPS_USE_SHM=OFF
```

or just close and restart the terminal.

Add this to your `.bashrc` if desired.

### 2. Package 'joint_state_publisher_gui' not found

**Issue:**
```
ros2 run joint_state_publisher_gui joint_state_publisher_gui
Package 'joint_state_publisher_gui' not found
```
**Resolution:**
Install the package for ROS 2 Jazzy:
```sh
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
```
**What it is for:**
`joint_state_publisher_gui` provides a graphical interface to manually set and adjust the positions of robot joints. This is especially useful for visualizing and testing robot models in RViz or Gazebo when no real hardware or controllers are available. It allows you to interactively move joints and see the resulting robot pose in real time.



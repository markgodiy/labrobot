# Project Name: `LabRobot`
# Model Number: TR45H

Primary reference: Josh Newans' ROS2 Robot Build Guide

- https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT
- https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros

## Objective

- Build a diff-drive robot to deliver specimens to multiple locations in a laboratory setting. 
- LabRobot must be capable of mapping the environment, navigating autonomously, and performing tasks such as waypoint following and object detection.
- The robot should be able to be controlled remotely, including from a mobile phone, if desired.

### Applications

- Differential-drive control (using ros2_control)
- Advanced teleoperation (including remote control from phone)
- Mapping and localisation with SLAM (using slam_toolbox)
- Autonomous navigation (using Nav2)
- Detecting an object with the camera and following it (using OpenCV)

### Software

- **Operating System**: Ubuntu 24.04 LTS
- **ROS2 Distribution**: Jazzy Jalisco (May 2024 - May 2029)
- **Gazebo Version**: Harmonic (September 2023 - September 2028)

### Hardware

- **Chassis**: 3D printed chassis with a 3D printed top plate.
- **Motors**: 2 x 12V DC motors with encoders.
- **Motor Driver**: L298N motor driver.
- **Power Supply**: 12V LiPo battery with a BMS (Battery Management System).
- **Sensors**: 
  - LIDAR (SLAMTEC RPLIDAR C1)
  - Depth Camera (Luxonis OAK-D Lite)
  - IMU module (MPU6050)

## Running the Simulation

To launch the robot in Gazebo with the default world (ground plane and sun):

```sh
colcon build --symlink-install
source install/setup.bash
ros2 launch labrobot sim.launch.py
```

- The launch file will:
  - Start Gazebo with the default world
  - Spawn the robot slightly above the ground
  - Bridge common ROS 2 and Gazebo topics

If you encounter issues with missing models or environment variables, try restarting WSL:

```sh
wsl --shutdown
# Then reopen your Ubuntu terminal
```

If you see harmless warnings about QML or /clock, you can ignore them.

# Project Name: LabRobot
# Model/Codename: TR45H


Primary reference: Josh Newans' ROS2 Robot Build Guide

* [https://www.youtube.com/watch?v=OWeLUSzxMsw\&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT](https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)
* [https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros](https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros)

## Objective:
Build a differential-drive mobile robot designed to deliver specimens to multiple locations within a laboratory environment.
The robot must be capable of mapping its environment, navigating autonomously, avoiding obstacles in real time, and supporting manual control via remote or phone interface.

## Updates:
- 27Jul2025: [update](/docs/README.md#2025-07-27-update)


## Core Applications:

* Differential-drive control using ros2\_control
* Mapping and localization using SLAM (slam\_toolbox)
* Autonomous navigation with path planning (Nav2)
* Sensor fusion using wheel encoders and IMU (robot\_localization)
* Local obstacle detection with a front-facing depth camera (OAK-D Lite)
* Object detection and basic visual processing using OpenCV
* Remote and teleoperation support (e.g., via mobile device or joystick)

Software:

* Operating System: Ubuntu 24.04 LTS
* ROS 2 Distribution: Jazzy Jalisco
* Simulator: Gazebo Harmonic
* Visualization: RViz2
* Middleware: ROS 2 lifecycle nodes, tf2, robot\_state\_publisher

Hardware:

* Chassis: Plastic enclosure with drawers (Sterilite), weight 2.56 kg
* Drive system: 2x 12V DC gear motors with encoders (130 RPM)
* Rear caster wheel for balance
* Wheels: 65mm diameter, 25mm width
* Motor Driver: L298N or BTS7960 dual H-bridge motor driver with optocoupler isolation
* Power Supply: 12V 3000mAh Li-ion battery with onboard BMS
* Hot-swappable battery design: 1 active battery, 1 standby onboard, 2 charging
* Wiring: 16 AWG 2-conductor stranded copper with terminal block connectors

Sensors:

* 2D LIDAR: RPLIDAR C1 mounted top-front-center
* Depth Camera: Luxonis OAK-D Lite mounted front-facing
* IMU: MPU6050
* Wheel encoders: Integrated in the motor assemblies

Navigation Considerations:

* LIDAR provides 360-degree scan data for SLAM and global planning
* Depth camera enables detection of close, low-profile, or small obstacles missed by LIDAR
* Costmap parameters include both LIDAR and depth camera in observation\_sources
* Localization uses robot\_localization EKF node with encoder and IMU data
* Conservative velocity and inflation settings help prevent tipping due to the high center of gravity
* All sensor links and frames are carefully placed in the TF tree
* Accurate footprint polygon used in global and local costmaps to support reliable indoor navigation

## USB Device Passthrough in WSL 2 (Windows)

To use hardware like RPLIDAR, OAK-D, or other USB devices in WSL 2, you must forward the USB device from Windows to WSL using usbipd-win:

1. **Install usbipd-win on Windows:**
   - Download and install from: https://github.com/dorssel/usbipd-win

2. **List USB devices in Windows PowerShell (as Administrator):**
   ```powershell
   usbipd list
   ```
   Find the BUSID for your device (e.g., RPLIDAR, OAK-D Lite).

3. **Attach the device to WSL:**
   ```powershell
   usbipd attach --wsl --busid <BUSID>
   ```
   Replace `<BUSID>` with the correct value.

4. **In WSL, check for the device:**
   - For serial devices: `ls /dev/ttyUSB*`
   - For OAK-D: `lsusb`

5. **Now you can use the device with ROS 2 or DepthAI as usual in WSL.**

**Reference:**
- https://github.com/dorssel/usbipd-win

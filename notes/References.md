# References for LabRobot Project

- How to Send Waypoints to the ROS 2 Navigation Stack – Nav 2  
https://automaticaddison.com/how-to-send-waypoints-to-the-ros-2-navigation-stack-nav-2/  
**Summary:** This tutorial explains how to send waypoints to a mobile robot using the ROS 2 Navigation Stack (Nav2) with Python. It covers real-world applications (e.g., delivery, hospitals, restaurants), prerequisites (ROS 2, Python 3.7+, workspace setup), and step-by-step directions. The guide includes creating Python scripts (`waypoint_follower.py`, `robot_navigator.py`), setting up launch and parameter files, and running the robot in simulation. The robot follows a sequence of waypoints, and the tutorial provides code snippets, configuration tips, and troubleshooting advice for successful navigation in Gazebo simulation environments.

- The Ultimate Guide to the ROS 2 Navigation Stack
https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/
    - **Summary:** This comprehensive guide introduces the ROS 2 Navigation Stack (Nav2), detailing its purpose, real-world applications, and prerequisites. It provides a step-by-step learning path through a series of tutorials, covering robot simulation, odometry, sensor fusion, LIDAR setup, and navigation/SLAM. The guide emphasizes simulation before hardware deployment, offers links to example packages and resources, and is suitable for both beginners and experienced roboticists aiming to master mobile robot navigation using ROS 2.

- AutomaticAddison YouTube Channel
https://www.youtube.com/@automaticaddison/featured
    - **Summary:** The AutomaticAddison YouTube channel offers free, practical robotics tutorials focused on ROS 2, mobile robot navigation, and real-world robotics skills. Videos range from beginner to advanced topics, including hands-on demonstrations, simulation, and code walkthroughs. The channel is designed to help viewers become job-ready in robotics without unnecessary theory or paid courses, making it a valuable resource for both students and professionals.

- DepthAI Manual Install – Supported Platforms
https://docs.luxonis.com/software/depthai/manual-install#supported-platforms
    - **Summary:** This official documentation page from Luxonis details the manual installation process for the DepthAI library, including supported platforms (Windows, macOS, Linux, Raspberry Pi, Jetson), prerequisites, and troubleshooting tips. It provides step-by-step instructions for installing DepthAI via pip or from source, as well as guidance for setting up udev rules on Linux for proper USB device access. This resource is essential for anyone integrating OAK-D or other Luxonis devices with Python or ROS 2.

- Luxonis Documentation Portal
https://docs.luxonis.com/
    - **Summary:** The official documentation portal for Luxonis devices, including OAK-D, OAK-D Lite, and DepthAI. It provides comprehensive guides, API references, hardware setup instructions, ROS/ROS 2 integration, and troubleshooting resources for all Luxonis products. Essential for development, testing, and deployment with Luxonis vision hardware.

- Running DepthAI Demo in WSL (Luxonis Forum)
https://discuss.luxonis.com/d/693-i-got-depthai-demo-to-run-in-wsl
    - **Summary:** A community forum thread where users share their experiences and solutions for running the DepthAI demo in Windows Subsystem for Linux (WSL). Includes troubleshooting tips, environment setup, and workarounds for common issues encountered when using OAK-D and DepthAI in WSL environments.

- usbipd-win (USB device passthrough for WSL 2)
https://github.com/dorssel/usbipd-win
    - **Summary:** Official repository for usbipd-win, a Windows service and CLI tool that enables USB device passthrough from Windows to WSL 2. Essential for using hardware like RPLIDAR, OAK-D, and other USB peripherals directly in WSL. Includes installation instructions, usage examples, and troubleshooting tips.

- Microsoft Docs: Connect USB devices to WSL 2
https://learn.microsoft.com/en-us/windows/wsl/connect-usb
    - **Summary:** Official Microsoft documentation for connecting USB devices to Windows Subsystem for Linux (WSL 2). Explains requirements, setup, and usage of usbipd-win, with troubleshooting and security notes. Essential for anyone using USB hardware with WSL 2.
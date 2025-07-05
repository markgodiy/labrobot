# Additional ROS 2 Jazzy + Gazebo Harmonic Integration Packages

To enable ROS <-> Gazebo (gz sim) integration, install the following packages:

```sh
sudo apt update
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

- `ros-jazzy-ros-gz-sim`: Provides the Gazebo Harmonic simulator integration for ROS 2.
- `ros-jazzy-ros-gz-bridge`: Provides the bridge for ROS <-> Gazebo message and service communication.

After installation, source your ROS 2 environment:

```sh
source /opt/ros/jazzy/setup.bash
```

You can now launch your simulation with ROS 2 and Gazebo Harmonic.

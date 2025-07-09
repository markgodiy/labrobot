# RPLIDAR C1 on Raspberry Pi with ROS 2

## 1. Install RPLIDAR ROS 2 Driver

- SSH into your Raspberry Pi.
- Install dependencies:
  ```sh
  sudo apt update
  sudo apt install git build-essential ros-${ROS_DISTRO}-ros2launch ros-${ROS_DISTRO}-sensor-msgs
  ```
- Clone the RPLIDAR ROS 2 driver (example: EAI's official or SLLidar's fork):
  ```sh
  cd ~/ros2_ws/src
  git clone https://github.com/Slamtec/rplidar_ros.git
  # or for SLLidar: git clone https://github.com/SLLIDAR/rplidar_ros2.git
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```

## 2. Connect and Identify the LIDAR

- Plug the RPLIDAR C1 into a USB port.
- Check the device name:
  ```sh
  ls /dev/tty*
  # Usually /dev/ttyUSB0 or /dev/ttyACM0
  ```
- Add your user to the dialout group if needed:
  ```sh
  sudo usermod -a -G dialout $USER
  # Then reboot or log out/in
  ```

## 3. Launch the RPLIDAR Node

- Example launch command (adjust device path if needed):
  ```sh
  ros2 launch rplidar_ros rplidar.launch.py serial_port:=/dev/ttyUSB0
  ```
- The node will publish LaserScan messages on `/scan`.

## 4. Test the LIDAR

- On the Pi or a remote machine:
  ```sh
  ros2 topic echo /scan
  # Or visualize in RViz2
  ```

## 5. Integrate with Your Robot

- Make sure the LIDAR's frame matches your robot's URDF (e.g., `lidar_frame`).
- You may need to use a static transform publisher if the frame names differ.

---

**Troubleshooting:**
- If you see permission errors, check user groups and device path.
- If no data, try swapping USB cables/ports or lowering the baud rate in the launch file.

**References:**
- https://github.com/Slamtec/rplidar_ros
- https://github.com/SLLIDAR/rplidar_ros2

# RPLIDAR C1 on Raspberry Pi with ROS 2 (Updated for sllidar_ros2)

Ref: 

https://github.com/Slamtec/sllidar_ros2

## 1. Install sllidar_ros2 Driver

- SSH into your Raspberry Pi.
- Install dependencies:
  ```sh
  sudo apt update
  sudo apt install git build-essential
  ```
- Clone the sllidar_ros2 driver:
  ```sh
  cd ~/ros2_ws/src
  git clone https://github.com/Slamtec/sllidar_ros2.git
  cd ~/ros2_ws
  source /opt/ros/$ROS_DISTRO/setup.bash
  colcon build --symlink-install
  source install/setup.bash
  ```
  `$ROS_DISTRO` is your ROS 2 distribution (e.g., humble, foxy).

## 2. Connect and Identify the LIDAR

- Plug the RPLIDAR C1 into a USB port.
- Check the device name:
  ```sh
  # See descriptive names for USB serial devices
  ls -l /dev/serial/by-id/
  # See devices by physical USB port connection
  ls -l /dev/serial/by-path/
  ```
  The `by-id` directory gives you device names with manufacturer and serial info, while `by-path` shows which physical USB port each device is connected to. Both are useful for identifying and troubleshooting multiple devices.
- (Optional) Add your user to the dialout group:
  ```sh
  sudo usermod -a -G dialout $USER
  # Then reboot or log out/in
  ```

## 3. Set Up Permissions (Recommended)

- For persistent permissions, create udev rules:
  ```sh
  cd ~/ros2_ws/src/sllidar_ros2/scripts
  sudo bash create_udev_rules.sh
  ```
- Or, for a quick test:
  ```sh
  sudo chmod 777 /dev/ttyUSB0
  ```

## 4. Launch the RPLIDAR Node

- For RPLIDAR C1, use the dedicated launch file:
  ```sh
  ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
  ```
- The node will publish LaserScan messages on `/scan`.

## 5. Test the LIDAR

- On the Pi or a remote machine:
  ```sh
  ros2 topic echo /scan
  ```

- Or visualize in RViz2
  ```sh
  rviz2
  ```

## 6. Integrate with Your Robot

- Ensure the LIDAR's frame matches your robot's URDF (e.g., `lidar_frame`).
- You may need to use a static transform publisher if the frame names differ.

---

**Troubleshooting:**
- If you see permission errors, check user groups, udev rules, and device path.
- If no data, try swapping USB cables/ports or lowering the baud rate in the launch file.
- To check for power issues on your Raspberry Pi:
  ```sh
  vcgencmd get_throttled
  ```
  - `throttled=0x0` means no power issues detected.
  - Any nonzero value (e.g., `throttled=0x50005`) means undervoltage or throttling has occurred. Replace your power supply or cable if needed.

**References:**
- https://github.com/Slamtec/sllidar_ros2


## launch scripts:

This is a basic non-RVIZ launch script for the C1 sllidar_ros2 package.

```python
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen'),
    ])
```

this is the rviz launch script for the C1 sllidar_ros2 package.

```python
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    rviz_config_dir = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'sllidar_ros2.rviz')


    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                           'scan_mode': scan_mode
                         }],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])
``` 
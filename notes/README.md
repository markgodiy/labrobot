# LabRobot Project Notes

A differential drive robot project using ROS 2 Jazzy and Gazebo Harmonic for simulation and real hardware deployment.

## Bill of Materials

TODO: Add Bill of Materials (BOM) for the LabRobot project.

## 2025-07-04: Project Restart and Environment Setup

Restarting project, targeting ROS2 Jazzy LTS and Gazebo Harmonic LTS:

- Jazzy Jalisco (May 2024 - May 2029)
- Gazebo Harmonic (September 2023 - September 2028)

### Environment Verification

```bash
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

```bash
$ gz sim --version
Gazebo Sim, version 8.9.0
Copyright (C) 2018 Open Source Robotics Foundation.
Released under the Apache 2.0 License
```

## Common Issues and Resolutions

### 1. RTPS_TRANSPORT_SHM Error in WSL or Container

**Issue:**

```text
[robot_state_publisher-1] [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7000: open_and_lock_file failed -> Function open_port_internal
```

**Resolution:**

This is a warning from Fast DDS about shared memory transport not being available (common in WSL or containers). It does not prevent ROS 2 from working.

To suppress the warning, disable shared memory transport:

```bash
export RMW_FASTRTPS_USE_SHM=OFF
```

or just close and restart the terminal.

### 2. Package 'joint_state_publisher_gui' not found

**Issue:**

```text
ros2 run joint_state_publisher_gui joint_state_publisher_gui
Package 'joint_state_publisher_gui' not found
```

**Resolution:**

Install the package for ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui
```

**What it is for:**

`joint_state_publisher_gui` provides a graphical interface to manually set and adjust the positions of robot joints. This is especially useful for visualizing and testing robot models in RViz or Gazebo when no real hardware or controllers are available. It allows you to interactively move joints and see the resulting robot pose in real time.

### colcon build --symlink-install

`colcon` is the recommended build tool for ROS 2 projects. It supports building multiple packages in a workspace and handles dependencies automatically.

`colcon build` is the command used to build all packages in the current workspace. It will compile the source code, generate necessary files, and prepare the packages for use. However, this command will need to be run each time you make changes to the source code or add new packages.

**Recommended:** `colcon build --symlink-install` instead creates symbolic links to the source files instead of copying them into the build directory. This allows for faster development since changes made in the source files are immediately reflected in the build without needing to rebuild the entire package.

## 2025-07-05: Simulation Bringup and Debugging Log

### Differential Drive Robot Simulation Setup (ROS 2 Jazzy + Gazebo Harmonic)

Set up robot model using Xacro/URDF, including:

- `robot.urdf.xacro` (main entry point)
- `robot_core.xacro` (links, joints, properties)
- `gazebo_control.xacro` (DiffDrive plugin and Gazebo integration)
- `inertial_macros.xacro` (inertial macros)

Created and updated launch files for simulation:

- `rsp.launch.py`, `rsp_gz.launch.py`, `sim.launch.py`
- Launches robot_state_publisher, joint_state_publisher_gui, gz sim, spawn_entity, and ros_gz_bridge

Added and configured the Gazebo DiffDrive plugin for compatibility with gz sim:

- Plugin block in `gazebo_control.xacro` uses correct joint names and topic names
- Set `<topic>/model/labrobot/cmd_vel</topic>` to match the ROS-Gazebo bridge

Added ROS-Gazebo bridge for:

- `/clock`, `/model/labrobot/cmd_vel`, `/model/labrobot/odometry`, `/tf`

Debugged and fixed:

- Xacro macro inclusion issues (typos, missing includes)
- Wheel and caster joint origins for correct placement
- DiffDrive plugin topic mismatch (was `cmd_vel`, changed to `/model/labrobot/cmd_vel`)
- Wheel joint axes: changed from `<axis xyz="0 0 1" />` (Z axis) to `<axis xyz="0 1 0" />` (Y axis) for correct wheel rotation

Confirmed:

- Robot spawns in simulation
- `/model/labrobot/cmd_vel` messages are bridged and received
- Robot now moves correctly in simulation

#### Key Lessons / Troubleshooting

- **DiffDrive plugin topic must match the bridged topic exactly** (use full `/model/labrobot/cmd_vel` path)
- **Wheel joint axes must be set to Y axis** (`<axis xyz="0 1 0" />`) for differential drive robots
- Use `ros2 topic echo` and `ros2 topic pub` to verify message flow
- Check simulation logs for plugin or physics errors if robot does not move

### Gazebo DiffDrive Plugin Notes

The Gazebo DiffDrive plugin (`gz::sim::systems::DiffDrive`) is responsible for simulating differential drive (two-wheeled) robot motion in Gazebo Sim. It reads velocity commands (Twist messages) from a specified topic and applies them to the wheel joints, simulating realistic robot movement.

**Key configuration points:**

- **Plugin block is included in `gazebo_control.xacro`.**
- **Joint names** must match the URDF exactly (e.g., `left_wheel_to_base_link`, `right_wheel_to_base_link`).
- **Topic** must match the fully qualified name used by the ROS-Gazebo bridge (e.g., `/model/labrobot/cmd_vel`).
- **Wheel separation and radius** should use Xacro properties for accuracy.
- **Odometry and TF**: The plugin can publish odometry and TF frames if configured.

**Example block:**

```xml
<plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diff-drive-system">
    <left_joint>left_wheel_to_base_link</left_joint>
    <right_joint>right_wheel_to_base_link</right_joint>
    <wheel_separation>${2 * wheelbase_y}</wheel_separation>
    <wheel_radius>${wheel_radius}</wheel_radius>
    <topic>/model/labrobot/cmd_vel</topic>
    <odom_topic>/model/labrobot/odometry</odom_topic>
    <odom_publish>true</odom_publish>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <update_rate>50</update_rate>
    <publish_tf>true</publish_tf>
</plugin>
```

**Troubleshooting:**

- If the robot does not move, check that the topic and joint names match exactly.
- If the wheels spin on the wrong axis, verify the `<axis>` tag in the wheel joints (should be `0 1 0` for Y axis).
- Use `ros2 topic echo` to confirm messages are being received on the correct topic.
- Check simulation logs for plugin errors or warnings.

### Caster Wheel Setup (Simulation)

The caster wheel is attached to the robot using a `fixed` joint, so it does not rotate or swivel in simulation. The caster wheel's collision element is configured with zero friction (`mu=0.0`, `mu2=0.0`) to ensure it does not add resistance or interfere with robot movement. This setup is useful for simple simulation where caster dynamics are not needed, and prevents instability or the caster "falling off" in Gazebo.

Example configuration:

```xml
<joint name="caster_wheel_to_base_link" type="fixed">
    <parent link="base_link" />
    <child link="caster_wheel" />
    <origin xyz="..." rpy="..." />
</joint>
<link name="caster_wheel">
    ...
    <collision>
        ...
        <surface>
            <friction>
                <ode>
                    <mu>0.0</mu>
                    <mu2>0.0</mu2>
                </ode>
            </friction>
        </surface>
    </collision>
    ...
</link>
```

For more realistic caster behavior, use a `continuous` joint and tune friction/inertia, but this can introduce simulation instability.

### Running the Simulation

To launch the robot in Gazebo with the default world (ground plane and sun):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch labrobot sim.launch.py
```

The launch file will:

- Start Gazebo with the default world
- Spawn the robot slightly above the ground
- Bridge common ROS 2 and Gazebo topics

If you encounter issues with missing models or environment variables, try restarting WSL:

```bash
wsl --shutdown
# Then reopen your Ubuntu terminal
```

If you see harmless warnings about QML or /clock, you can ignore them.

## 2025-07-27: Hardware Integration and Real Robot Testing

23 days later - major hardware progress:

- Finally 3D printed the base and lidar + camera mount
- Did a lot of learning about basic wiring and electronics; rewired things a few times
- Using a Pico(W) + micropython to control the motors and (read the encoders). I just happened to have a Pico W lying around but a non-W Pico would work just as well
- Got the Slamtec C1 LIDAR working with ROS 2 Jazzy
- Had to rotate the z-axis of the lidar joint by 3.14 radians (180 degrees) to get it to point in the right direction. Apparently this is a common issue (Figure 1)
- Since I'm using a PicoW, I added a web server to the robot movement for testing. The micropython code will change later for use with ROS2. See: [main.py](../upython/main.py)

![Figure 1. LIDAR alignment](images/lidar_plane.png)

![Figure 2. Pico W web interface](images/picow_web_interface.png)

### 3. Installing DepthAI on Raspberry Pi

**Issue:**

```text
python3 -m pip install depthai
error: externally-managed-environment
```

**Resolution:**

Modern Python installations use externally managed environments to prevent conflicts. Follow the official Luxonis installation guide:


#### ROS 2 Integration with depthai-ros

For full ROS 2 integration with OAK cameras, install the official depthai-ros package:

```bash
# With virtual environment and ROS 2 sourced
source ~/robot_env/bin/activate
source /opt/ros/jazzy/setup.bash

# Install depthai-ros from apt (if available for Jazzy)
sudo apt update
sudo apt install ros-jazzy-depthai-ros

# OR build from source if not available in apt
cd ~/lab_ws/src
git clone https://github.com/luxonis/depthai-ros.git
cd ~/lab_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select depthai_ros_msgs depthai_ros_driver depthai_bridge
```

#### Testing OAK Camera with ROS 2

```bash
# Launch basic RGB camera node (recommended for robot integration)
ros2 launch depthai_ros_driver camera_as_part_of_a_robot.launch.py

# OR launch basic camera node
ros2 launch depthai_ros_driver camera.launch.py

# Verify topics are publishing
ros2 topic list | grep oak

# View RGB image (rectified)
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_rect

# Check camera info
ros2 topic echo /oak/rgb/camera_info --once

# Test with RViz2
rviz2
```

#### RViz2 Setup for OAK Camera

In RViz2, add these displays to visualize your camera data:

1. **Fixed Frame**: Set to `oak_camera_frame` or `base_link`
2. **Image Display**: Subscribe to `/oak/rgb/image_rect`
3. **TF Display**: Enable to see camera transforms
4. **Robot Model**: Add to see your robot with camera

#### Current Topics (OAK-D Lite on Robot)

Based on your robot setup, these topics are available:
- `/oak/rgb/image_rect` - Rectified RGB camera image
- `/oak/rgb/image_rect/compressed` - Compressed RGB image
- `/scan` - LIDAR data
- `/joint_states` - Robot joint positions
- `/tf` and `/tf_static` - Transform data

#### Integration with Robot Launch Files

Add OAK camera to your robot's launch file:

```python
# In pi_launch.py, add OAK camera node
oak_camera = Node(
    package='depthai_ros_driver',
    executable='depthai_ros_driver_node',
    name='oak_camera',
    parameters=[{
        'camera_model': 'OAK-D-LITE',  # Adjust for your model
        'tf_prefix': 'oak',
        'mode': 'depth',
        'depth_enabled': True,
        'stereo_enabled': True,
        'rgb_enabled': True,
    }],
    output='screen'
)
```

**Published Topics:**

```
/oak/rgb/image_rect
/oak/rgb/camera_info
```

#### Troubleshooting depthai-ros Build Issues

**Issue:**

```text
/home/botmanager/dai_ws/src/depthai-ros/depthai_filters/src/segmentation_overlay.cpp:3:10: fatal error: cv_bridge/cv_bridge.h: No such file or directory
    3 | #include "cv_bridge/cv_bridge.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```

**Resolution:**

The depthai-ros packages require OpenCV bridge and image transport dependencies that aren't automatically installed. Install them manually:

```bash
# Install required OpenCV and image transport packages
sudo apt update
sudo apt install ros-jazzy-cv-bridge ros-jazzy-vision-opencv ros-jazzy-image-transport

# Also install additional vision dependencies
sudo apt install ros-jazzy-image-geometry ros-jazzy-camera-info-manager

# Then retry the build
cd ~/dai_ws
colcon build --symlink-install
```

**Alternative: Use rosdep to install all dependencies**

```bash
cd ~/dai_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**If the build still fails with cv_bridge errors:**

This might be a CMake path issue. Try these additional steps:

```bash
# Install development headers and CMake files
sudo apt install libopencv-dev ros-jazzy-cv-bridge-dev

# OR try sourcing ROS environment and rebuilding from scratch
source /opt/ros/jazzy/setup.bash
cd ~/dai_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# OR as a workaround, try building without the problematic filters package
colcon build --symlink-install --packages-skip depthai_filters
```

#### Alternative: Use apt package instead of building from source

If building from source continues to fail, try using the pre-built package:

```bash
# Remove the source build
cd ~
rm -rf dai_ws

# Install from apt repositories
sudo apt update
sudo apt install ros-jazzy-depthai-ros

# Test if it works
source /opt/ros/jazzy/setup.bash
ros2 launch depthai_ros_driver camera.launch.py
```

**Published Topics:**

```
/oak/rgb/image_rect
/oak/rgb/camera_info
```

## 2025-07-31: OAK-D Lite Integration and Power Issues

### Hardware Connection Issues (Power/Brownout)

**Issue:**

Camera connects initially but then crashes with power-related errors:

```text
[INFO] [oak]: Camera with MXID: 19443010C1CFE42C00 and Name: 1.1.1 connected!
[INFO] [oak]: USB SPEED: HIGH
[INFO] [oak]: Device type: OAK-D-LITE
[INFO] [oak]: Finished setting up pipeline.
terminate called after throwing an instance of 'std::system_error'
  what(): Device already closed or disconnected: Input/output error
[ERROR] [component_container-1]: process has died [pid 4254, exit code -6]
```

OR continuous communication errors:

```text
[ERROR] [oak]: No data on logger queue!
[ERROR] [oak]: Camera diagnostics error: Communication exception - possible device error/misconfiguration. Original message 'Couldn't read data from stream: 'sys_logger_queue' (X_LINK_ERROR)'
```

**Root Cause:**

This indicates a **USB power brownout issue**. The OAK-D Lite requires significant power (~2.5W) and can cause USB communication failures on Raspberry Pi when:
- Pi power supply is insufficient (<3A)
- USB ports can't provide enough current
- USB cable has high resistance
- Multiple USB devices compete for power

**Immediate Solutions:**

```bash
# 1. Try reduced power mode first
ros2 launch depthai_ros_driver camera.launch.py \
    camera_model:=OAK-D-LITE \
    mode:=rgb \
    rgb_resolution:=720p \
    depth_enabled:=false \
    spatial_detection_enabled:=false

# 2. Check USB power in system logs
dmesg | grep -i usb | tail -20

# 3. Monitor USB events during camera startup
sudo dmesg -w | grep -i usb &
ros2 launch depthai_ros_driver camera.launch.py
```

**Hardware Solutions (in order of effectiveness):**

1. **Use a powered USB 3.0 hub** - Most reliable solution
   - Get a hub with dedicated 2A+ per port
   - Connect OAK-D Lite through the hub, not direct to Pi

2. **Upgrade Pi power supply** - Use official 5V/3A+ adapter
   - Check current supply: `vcgencmd get_throttled`
   - Value should be `0x0` (no throttling)

3. **Use shorter, high-quality USB 3.0 cable**
   - Avoid extension cables or hubs without power
   - USB-C to USB-A cable, <1 meter preferred

4. **Enable USB power boost** in `/boot/firmware/config.txt`:
   ```bash
   sudo nano /boot/firmware/config.txt
   # Add these lines:
   max_usb_current=1
   usb_max_current_enable=1
   ```
   Then reboot: `sudo reboot`

**Testing and Diagnostics:**

```bash
# Check Pi power status
vcgencmd get_throttled
vcgencmd measure_volts core

# Monitor USB device power
lsusb -v | grep -A5 -B5 MaxPower

# Test with minimal launch
ros2 launch depthai_ros_driver camera.launch.py \
    camera_model:=OAK-D-LITE \
    mode:=rgb \
    rgb_resolution:=480p \
    depth_enabled:=false \
    stereo_enabled:=false \
    spatial_detection_enabled:=false \
    publish_tf_from_calibration:=false
```

**Update 2025-07-31:**

Even with reduced power settings (`mode:=rgb`, `depth_enabled:=false`, `spatial_detection_enabled:=false`), the camera still crashes with the same error. This confirms it's a **hardware power delivery issue** that requires physical solutions.

```text
# Even with reduced settings, still crashes:
ros2 launch depthai_ros_driver camera.launch.py camera_model:=OAK-D-LITE mode:=rgb rgb_resolution:=720p depth_enabled:=false spatial_detection_enabled:=false
# Result: terminate called after throwing an instance of 'std::system_error'
```

**Required Hardware Solutions:**

Software workarounds are **not sufficient** for this power issue. You need one of these hardware fixes:

1. **Powered USB 3.0 Hub** (Most Reliable)
   - Anker, UGREEN, or similar with 2A+ per port
   - Connect OAK-D Lite through hub, not directly to Pi

2. **Better Power Supply**
   - Official Raspberry Pi 5V/3A+ adapter
   - Check current power status: `vcgencmd get_throttled`

3. **USB Power Boost Configuration**
   ```bash
   sudo nano /boot/firmware/config.txt
   # Add:
   max_usb_current=1
   usb_max_current_enable=1
   # Then: sudo reboot
   ```

**Immediate Action Required:**
Get a powered USB 3.0 hub - this is the most reliable solution for OAK cameras on Raspberry Pi.

**Next Steps:**



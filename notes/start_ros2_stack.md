# Labrobot — ROS 2 Stack Startup Guide

## Hardware

| Component | Connection | Device path |
|---|---|---|
| Pico W / Pico 2W (motor controller) | USB | `/dev/ttyACM0` |
| RPLIDAR (LIDAR) | USB–UART (CP2102N) | `/dev/ttyUSB0` or `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_...` |
| OAK-D Lite (depth camera) | USB 3 | auto-detected by depthai |

---

## Boot checklist

1. Power on Pi — wait ~30 s for boot
2. Confirm Pico W is plugged in via USB (`/dev/ttyACM0` should appear)
3. Confirm LIDAR USB dongle is plugged in (`/dev/ttyUSB0`)
4. SSH in or open a terminal on the Pi

---

## Start the full stack

```bash
cd ~/lab_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch labrobot pi.autonomous.serial.launch.py
```

To run detached from the terminal (survives SSH disconnect):

```bash
nohup bash -c '
  cd ~/lab_ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch labrobot pi.autonomous.serial.launch.py
' > /tmp/stack.log 2>&1 &
```

Monitor the log:

```bash
tail -f /tmp/stack.log
```

> **Always stop any existing stack before starting a new one** — see the section below.
> Running `ros2 launch` twice causes two stacks to compete for `/dev/ttyACM0` and the OAK-D camera,
> producing `Resource temporarily unavailable` and `X_LINK_DEVICE_ALREADY_IN_USE` errors.

---

## Stop the stack / clean restart

### Normal stop (when the terminal is still attached)

Press `Ctrl+C` in the terminal running the launch. Wait ~2 s for nodes to exit.

### Stop when running detached (most common case)

Directly `pkill`-ing ROS processes from SSH can drop your SSH session (a process in the tree holds the TTY).
Use the `nohup` trick to schedule the kill after SSH returns:

```bash
# Step 1 — schedule the kill (SSH returns immediately)
nohup bash -c '
  sleep 1 &&
  pkill -9 -f "ros2 launch" ;
  pkill -9 -f serial_motor_bridge ;
  pkill -9 -f robot_dashboard ;
  pkill -9 -f autonomous_navigation_node ;
  pkill -9 -f sllidar_node ;
  pkill -9 -f camera_node ;
  pkill -9 -f robot_state_publisher ;
  pkill -9 -f joint_state_publisher ;
  pkill -9 -f static_transform_publisher ;
  pkill -9 -f republish
' &>/dev/null &

# Step 2 — wait a few seconds, then verify everything is gone
sleep 5
ps aux | grep -E 'ros2|serial_motor|camera_node|robot_dash|autonomous|sllidar' | grep -v grep
# should return nothing (or only the grep itself)
```

Check devices are free:

```bash
fuser /dev/ttyACM0   # should return nothing
fuser /dev/ttyUSB0   # should return nothing
```

### Clean restart (stop then start)

```bash
# Kill existing stack (scheduled, SSH-safe)
nohup bash -c '
  sleep 1 &&
  pkill -9 -f "ros2 launch" ;
  pkill -9 -f serial_motor_bridge ;
  pkill -9 -f robot_dashboard ;
  pkill -9 -f autonomous_navigation_node ;
  pkill -9 -f sllidar_node ;
  pkill -9 -f camera_node ;
  pkill -9 -f robot_state_publisher ;
  pkill -9 -f joint_state_publisher ;
  pkill -9 -f static_transform_publisher ;
  pkill -9 -f republish
' &>/dev/null &

# Wait for processes to die
sleep 5

# Start fresh
nohup bash -c '
  cd ~/lab_ws &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch labrobot pi.autonomous.serial.launch.py
' > /tmp/stack.log 2>&1 &

# Verify after ~8 s
sleep 8
pgrep -fa serial_motor_bridge
pgrep -fa robot_dashboard
tail -5 /tmp/stack.log
```

### From the dashboard

Use the **Shutdown Robot** button (Pi System card) for a safe full power-off.
This stops all nodes and halts the OS — no risk of file corruption from a hard power cut.

---

## What the launch file starts

`pi.autonomous.serial.launch.py` includes `pi.basic.sensors.launch.py` and adds the motor bridge, navigation, and dashboard.

| Node | Script | Purpose |
|---|---|---|
| `robot_state_publisher` | ROS built-in | Publishes TF tree from URDF model |
| `joint_state_publisher` | ROS built-in | Publishes joint states for URDF |
| `map_to_odom_publisher` | ROS built-in (`static_transform_publisher`) | Static TF: `map` → `odom` |
| `oak_to_depth_camera_publisher` | ROS built-in (`static_transform_publisher`) | Static TF: `depth_camera_link` → `oak` |
| `sllidar_node` | sllidar_ros2 | Publishes `/scan` from RPLIDAR |
| `camera_node` (oak) | depthai_ros_driver | Publishes RGB + depth images and MobileNet-SSD detections from OAK-D Lite |
| `rgb_republisher` | image_transport | Republishes `/oak/rgb/image_raw` → `/camera/rgb/image_raw` |
| `depth_republisher` | image_transport | Republishes `/oak/stereo/image_raw` → `/camera/depth/image_raw` |
| `serial_motor_bridge` | `scripts/serial_motor_bridge.py` | Translates ROS 2 topics/services ↔ JSON serial commands to Pico W. Publishes odometry and encoder data. |
| `autonomous_navigation_node` | `scripts/autonomous_navigation_node.py` | Fuses LIDAR + depth camera data, makes drive decisions, publishes `/cmd_vel` |
| `robot_dashboard` | `scripts/robot_dashboard.py` | Web dashboard at `http://<pi-ip>:8080` |

---

## All scripts reference

### Nodes (run as part of the stack)

| File | Role |
|---|---|
| `serial_motor_bridge.py` | Primary ROS ↔ Pico W bridge. Sends JSON commands over `/dev/ttyACM0` (115200 baud). Subscribes to `/cmd_vel`; publishes `/odom`, `/motor_controller/status`, `/motor_controller/encoder_data`. Handles e-stop, autonomous mode, ping/status. |
| `autonomous_navigation_node.py` | Obstacle-avoidance navigation. Subscribes to `/scan` (LIDAR) and `/camera/depth/image_raw` (OAK-D). Publishes `/cmd_vel` and `/navigation/debug`. Blocks motion when LIDAR scan is absent. |
| `follow_me_node.py` | Person-following mode. Subscribes to OAK-D MobileNet-SSD detections; tracks nearest person and drives toward them. Activated from the dashboard `/follow` page. |
| `robot_dashboard.py` | HTTP server + ROS node. Serves dashboard UI, remote control, follow-me, and flash-Pico pages. Aggregates metrics from all topics and exposes them as JSON at `/metrics`. |
| `basic_odom_publisher.py` | Minimal odometry publisher (fallback / testing). Publishes `/odom` from wheel encoder data when the full bridge is not used. |

### Utilities (run manually)

| File | How to run | Purpose |
|---|---|---|
| `serial_nav_control.py` | `python3 serial_nav_control.py` | CLI to start/stop autonomous mode, trigger e-stop, and check status without the dashboard. |
| `manual_robot_control.py` | `python3 manual_robot_control.py` | Send raw movement commands directly to the Pico W over serial for hardware testing. |
| `test_micropython_controller.py` | `python3 test_micropython_controller.py` | Full serial comms test — ping, motor commands, encoder read, temperature. Run before first drive. |
| `test_movement.py` | `python3 test_movement.py` | Short movement sequences (forward/back/turn) for quick drive verification. |
| `reset_emergency_stop.py` | `python3 reset_emergency_stop.py` | Calls the ROS 2 `/reset_emergency_stop` service. Use if the robot is stuck in e-stop with no dashboard. |
| `robot_perception_monitor.py` | `python3 robot_perception_monitor.py` | Live terminal view of LIDAR readings, navigation decisions, and motor commands. |
| `monitor_robot.sh` | `bash monitor_robot.sh` | Shell script that tails key ROS topics for quick status monitoring. |
| `nav_control.py` | `python3 nav_control.py` | Low-level navigation command helper. |
| `labrobot_square.py` | `python3 labrobot_square.py` | Drives the robot in a square — geometry_msgs/Twist test. |

### Wrappers (internal — not called directly)

| File | Purpose |
|---|---|
| `serial_motor_bridge_wrapper.py` | Entry-point wrapper for the bridge when installed via colcon. |
| `autonomous_navigation_wrapper.py` | Entry-point wrapper for the navigation node when installed via colcon. |

### MicroPython (Pico W — flashed via `/flashpico`)

| File | Purpose |
|---|---|
| `upython/main.py` | Main firmware. Runs on boot. Handles motor PWM (ENA=GP3, IN1=GP4, IN2=GP5 / IN3=GP11, IN4=GP12, ENB=GP13), encoder counting, serial JSON command loop, e-stop, ping/status, battery monitoring via INA219. |
| `upython/ina219.py` | INA219 current/voltage sensor driver. |
| `upython/wifi_config.py` | WiFi credentials helper (unused in current USB-serial mode). |
| `upython/ros2_autonomous_main.py` | Experimental: autonomous logic running on-Pico (not used in current stack). |
| `upython/ros2_serial_main.py` | Experimental: ROS 2 serial protocol variant (not used in current stack). |

---

## Launch files reference

| File | What it launches | When to use |
|---|---|---|
| `pi.autonomous.serial.launch.py` | Full stack: sensors + bridge + navigation + dashboard | **Normal operation** |
| `pi.basic.sensors.launch.py` | RSP + LIDAR + OAK-D only | Sensor-only testing (no motors) |
| `pi.lidar.launch.py` | LIDAR only | LIDAR-only testing |
| `pi.lidar.depthsensor.launch.py` | LIDAR + OAK-D | Sensor pair testing |
| `pi.cv.launch.py` | Camera + CV pipeline | Vision-only testing |
| `pi.full.launch.py` | Full stack (older, without serial bridge) | Legacy |
| `pi.autonomous.launch.py` | Autonomous without serial bridge | Legacy |
| `pi.lab.mapping.launch.py` | SLAM mapping mode | Building a map |
| `pi.lab.nav.launch.py` | Nav2 navigation on saved map | Navigating a known map |
| `sim.launch.py` / `rsp_gz.launch.py` | Gazebo simulation | Simulation on desktop |

---

## Stopping the stack

```bash
# Kill everything cleanly
pkill -f 'ros2 launch'
# Wait ~2s then kill any stragglers
pkill -9 -f 'serial_motor_bridge|robot_dashboard|autonomous_nav|sllidar|camera_node'
```

From the dashboard, use the **Shutdown Robot** button (Pi System card) for a safe full power-off.

---

## Key topics

| Topic | Type | Published by |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | sllidar_node |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | rgb_republisher |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | depth_republisher |
| `/cmd_vel` | `geometry_msgs/Twist` | autonomous_navigation_node |
| `/odom` | `nav_msgs/Odometry` | serial_motor_bridge |
| `/motor_controller/status` | `std_msgs/String` | serial_motor_bridge |
| `/motor_controller/encoder_data` | `std_msgs/String` | serial_motor_bridge |
| `/navigation/debug` | `std_msgs/String` | autonomous_navigation_node |
| `/follow_me/debug` | `std_msgs/String` | follow_me_node |

---

## Dashboard pages

| URL | Purpose |
|---|---|
| `http://192.168.25.135:8080/` | Main dashboard — system metrics, e-stop, motor status |
| `http://192.168.25.135:8080/remote` | Mobile remote control (d-pad) |
| `http://192.168.25.135:8080/follow` | Follow-me mode control |
| `http://192.168.25.135:8080/mfollow` | Mobile follow mode |
| `http://192.168.25.135:8080/flashpico` | Flash / re-upload Pico W firmware |

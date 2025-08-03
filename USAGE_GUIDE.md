# LapRobot Usage Guide
## Essential Files for Production Use

This guide explains which files you need to use for your autonomous robot system after cleanup.

## 🚀 Production Files (Use These)

### 1. MicroPython Controller
**File**: `upython/ros2_serial_main.py`
- **Purpose**: Main controller code for Pico W
- **Communication**: USB Serial (JSON commands)
- **Features**: Motor control, autonomous mode, emergency stop
- **Usage**: Upload this to your Pico W

**File**: `upython/wifi_config.py`
- **Purpose**: WiFi configuration (optional, for HTTP testing only)
- **Usage**: Configure your WiFi credentials if needed for testing

### 2. ROS 2 Launch Files
**File**: `launch/pi.autonomous.serial.launch.py` ⭐ **MAIN LAUNCH**
- **Purpose**: Complete autonomous navigation system with serial communication
- **Includes**: LIDAR, OAK-D Lite camera, IMU, serial bridge, navigation
- **Usage**: `ros2 launch labrobot pi.autonomous.serial.launch.py`

**File**: `launch/pi.full.launch.py`
- **Purpose**: Full sensor suite (included by main launch)
- **Includes**: LIDAR, cameras, IMU, robot model
- **Usage**: Used automatically by autonomous launch

**File**: `launch/pi.lidar.depthsensor.launch.py`
- **Purpose**: LIDAR + depth camera only
- **Usage**: For sensor testing: `ros2 launch labrobot pi.lidar.depthsensor.launch.py`

### 3. ROS 2 Nodes (Scripts)
**File**: `scripts/serial_motor_bridge.py` ⭐ **CORE BRIDGE**
- **Purpose**: Translates ROS 2 commands to serial JSON for Pico W
- **Services**: `/motor/stop`, `/motor/emergency_stop`, `/motor/set_autonomous_mode`
- **Topics**: Subscribes to `/cmd_vel`, publishes `/motor_controller/status`

**File**: `scripts/autonomous_navigation_node.py` ⭐ **CORE NAVIGATION**
- **Purpose**: LIDAR-based obstacle avoidance and navigation decisions
- **Features**: Autonomous navigation, emergency stop, path planning
- **Communication**: Uses serial bridge (no HTTP)

**File**: `scripts/serial_nav_control.py` ⭐ **CONTROL UTILITY**
- **Purpose**: Command-line control and monitoring
- **Features**: Enable/disable autonomous mode, emergency stop, status monitoring
- **Usage**: `python3 serial_nav_control.py --help`

**File**: `scripts/test_micropython_controller.py`
- **Purpose**: Testing and debugging serial communication
- **Usage**: `python3 test_micropython_controller.py`

## 🗑️ Removed Files (No Longer Needed)

These files were removed during cleanup as they were redundant or obsolete:

- ❌ `launch/autonomous_navigation_node.py` - Was duplicate of script
- ❌ `launch/pi.autonomous.launch.py` - HTTP-based (replaced by serial version)
- ❌ `scripts/nav_control.py` - HTTP-based (replaced by serial version)
- ❌ `upython/ros2_autonomous_main.py` - HTTP-based (replaced by serial version)

## 📋 Quick Start Commands

### 1. Build and Setup
```bash
cd ~/lab_ws
colcon build --packages-select labrobot
source install/setup.bash
```

### 2. Upload to Pico W
Copy `upython/ros2_serial_main.py` to your Pico W as `main.py`

### 3. Start Full System
```bash
ros2 launch labrobot pi.autonomous.serial.launch.py
```

### 4. Control Commands
```bash
# Enable autonomous navigation
python3 serial_nav_control.py --autonomous on

# Emergency stop
python3 serial_nav_control.py --emergency-stop

# Monitor status
python3 serial_nav_control.py --status

# Disable autonomous mode
python3 serial_nav_control.py --autonomous off
```

## 🔧 Launch File Parameters

### Main Launch (`pi.autonomous.serial.launch.py`)
- `serial_port`: USB serial port (default: `/dev/ttyACM0`)
- `baudrate`: Serial baudrate (default: `115200`)
- `autonomous_enabled`: Start with autonomous mode (default: `true`)
- `min_obstacle_distance`: Obstacle detection distance in meters (default: `0.5`)
- `max_speed`: Maximum motor speed percentage (default: `70`)
- `use_rviz`: Start RViz visualization (default: `false`)

### Example with Custom Parameters
```bash
ros2 launch labrobot pi.autonomous.serial.launch.py \
  serial_port:=/dev/ttyUSB0 \
  autonomous_enabled:=false \
  min_obstacle_distance:=0.8 \
  use_rviz:=true
```

## 🛡️ Safety Features

- **Emergency Stop**: Available via service call or control utility
- **Serial Timeout**: Automatic motor stop if communication lost
- **Obstacle Detection**: LIDAR-based collision avoidance
- **Autonomous Mode Toggle**: Can enable/disable navigation remotely
- **Status Monitoring**: Real-time system health monitoring

## 📊 Topics and Services

### Key Topics
- `/scan` - LIDAR data
- `/cmd_vel` - Motor commands
- `/motor_controller/status` - Controller status
- `/navigation/state` - Navigation state

### Key Services
- `/motor/emergency_stop` - Emergency stop
- `/motor/set_autonomous_mode` - Enable/disable autonomous mode
- `/emergency_stop` - System-wide emergency stop
- `/set_autonomous_mode` - Navigation autonomous mode

## 🔍 Debugging

### Check Serial Connection
```bash
python3 test_micropython_controller.py
```

### Monitor Topics
```bash
ros2 topic echo /motor_controller/status
ros2 topic echo /navigation/state
ros2 topic echo /scan
```

### Check Services
```bash
ros2 service list | grep motor
ros2 service call /motor/emergency_stop std_srvs/srv/Trigger
```

This streamlined setup provides a robust, production-ready autonomous navigation system using serial communication as the primary method.

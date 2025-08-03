# ROS 2 Autonomous Navigation System
## MicroPython Motor Controller Integration

This system enables autonomous navigation for a robot using LIDAR and depth camera data, with a MicroPython-based motor controller.

## Architecture Overview

```
┌─────────────────┐    HTTP Commands    ┌──────────────────────┐
│   ROS 2 Pi      │ ────────────────── │ MicroPython          │
│                 │                     │ Motor Controller     │
│ ┌─────────────┐ │                     │ (Pico W)             │
│ │ LIDAR       │ │                     │                      │
│ │ OAK-D Lite  │ │    Navigation       │ ┌──────────────────┐ │
│ │ IMU         │ │    Decisions        │ │ Motor A (Left)   │ │
│ └─────────────┘ │                     │ │ Motor B (Right)  │ │
│                 │                     │ │ Safety Systems   │ │
│ ┌─────────────┐ │                     │ └──────────────────┘ │
│ │ Autonomous  │ │                     └──────────────────────┘
│ │ Navigation  │ │
│ │ Node        │ │
│ └─────────────┘ │
└─────────────────┘
```

## Hardware Requirements

### ROS 2 Pi System
- Raspberry Pi 4B (8GB recommended)
- RPLIDAR A1M8 or compatible LIDAR
- OAK-D Lite depth camera
- IMU (integrated with OAK-D Lite)

### MicroPython Motor Controller
- Raspberry Pi Pico W
- L298N or compatible motor driver
- 2x DC motors (differential drive)
- WiFi connectivity

## Software Components

### 1. MicroPython Controller (`ros2_autonomous_main.py`)
- **Purpose**: Motor control and safety systems
- **Communication**: HTTP server on port 8080
- **Features**:
  - Smooth speed ramping to prevent tipping
  - Emergency stop capabilities
  - Command timeout safety
  - Autonomous mode toggle
  - Real-time status reporting

### 2. ROS 2 Navigation Node (`autonomous_navigation_node.py`)
- **Purpose**: Process sensor data and make navigation decisions
- **Subscriptions**:
  - `/scan` - LIDAR data
  - `/oak/depth/image_raw` - Depth camera
  - `/oak/rgb/image_raw` - RGB camera
  - `/oak/points` - Point cloud data
- **Services**:
  - `/emergency_stop` - Emergency stop trigger
  - `/set_autonomous_mode` - Enable/disable autonomous navigation

### 3. Launch System (`pi.autonomous.launch.py`)
- **Purpose**: Start complete autonomous navigation system
- **Includes**: Full sensor suite + navigation node
- **Parameters**: Configurable navigation behavior

### 4. Control Utility (`nav_control.py`)
- **Purpose**: Easy control interface
- **Features**: Command-line control, status monitoring, manual override

## Installation & Setup

### 1. Deploy MicroPython Code

```bash
# Connect to MicroPython device
mpremote a0 fs cp src/labrobot/upython/ros2_autonomous_main.py :main.py

# Verify deployment
mpremote a0 ls
```

### 2. Build ROS 2 Workspace

```bash
cd ~/lab_ws
colcon build --packages-select labrobot
source install/setup.bash
```

### 3. Make Scripts Executable

```bash
chmod +x src/labrobot/scripts/autonomous_navigation_node.py
chmod +x src/labrobot/scripts/nav_control.py
```

## Usage

### Start Complete Autonomous System

```bash
# Launch full autonomous navigation
ros2 launch labrobot pi.autonomous.launch.py

# With custom parameters
ros2 launch labrobot pi.autonomous.launch.py \
  micropython_ip:=192.168.25.72 \
  autonomous_enabled:=true \
  max_speed:=60 \
  min_obstacle_distance:=0.8
```

### Control Commands

```bash
# Emergency stop
python3 src/labrobot/scripts/nav_control.py --emergency-stop

# Enable autonomous mode
python3 src/labrobot/scripts/nav_control.py --autonomous on

# Disable autonomous mode  
python3 src/labrobot/scripts/nav_control.py --autonomous off

# Monitor status
python3 src/labrobot/scripts/nav_control.py --monitor

# Manual control (when autonomous is off)
python3 src/labrobot/scripts/nav_control.py --move forward --speed 50
python3 src/labrobot/scripts/nav_control.py --rotate left --speed 40
python3 src/labrobot/scripts/nav_control.py --stop
```

### ROS 2 Service Calls

```bash
# Emergency stop
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Toggle autonomous mode
ros2 service call /set_autonomous_mode std_srvs/srv/SetBool "data: true"
ros2 service call /set_autonomous_mode std_srvs/srv/SetBool "data: false"

# Check navigation status
ros2 topic echo /motor_controller/status
ros2 topic echo /navigation/state
```

### Direct MicroPython Communication

```bash
# Test controller directly
curl "http://192.168.25.72:8080/status"
curl "http://192.168.25.72:8080/move?dir=forward&speed=50&duration=2"
curl "http://192.168.25.72:8080/rotate?dir=left&speed=40&duration=1"
curl "http://192.168.25.72:8080/stop"
curl "http://192.168.25.72:8080/estop"
```

## Navigation Algorithm

### Obstacle Avoidance Logic
1. **LIDAR Analysis**: Scan front 90° range for obstacles
2. **Distance Check**: Stop if obstacle < `min_obstacle_distance`
3. **Path Planning**:
   - **Clear path**: Move forward at adaptive speed
   - **Left clear**: Rotate left to avoid obstacle
   - **Right clear**: Rotate right to avoid obstacle
   - **Both clear**: Choose direction with more space
   - **No clear path**: Back up and reassess

### Safety Features
- **Command Timeout**: Stop if no command received within 2 seconds
- **Emergency Stop**: Immediate motor shutdown with override
- **Speed Ramping**: Smooth acceleration/deceleration
- **Dual Communication**: ROS 2 + direct HTTP fallback

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `micropython_ip` | 192.168.25.72 | Controller IP address |
| `micropython_port` | 8080 | Controller port |
| `autonomous_enabled` | false | Start with autonomous mode |
| `min_obstacle_distance` | 0.5 | Obstacle detection distance (m) |
| `max_speed` | 70 | Maximum motor speed (%) |
| `default_speed` | 50 | Default forward speed (%) |
| `rotation_speed` | 40 | Rotation speed (%) |
| `scan_angle_range` | 90 | LIDAR scan range (degrees) |
| `command_timeout` | 2.0 | Safety timeout (seconds) |

## Troubleshooting

### Connection Issues
```bash
# Check MicroPython controller
ping 192.168.25.72
curl "http://192.168.25.72:8080/status"

# Check ROS 2 topics
ros2 topic list
ros2 topic echo /scan --max-count 1
```

### Motor Issues
```bash
# Test manual control
python3 src/labrobot/scripts/nav_control.py --move forward --speed 30
python3 src/labrobot/scripts/nav_control.py --stop

# Check emergency stop status
python3 src/labrobot/scripts/nav_control.py --status
```

### Sensor Issues
```bash
# Verify sensor data
ros2 topic echo /scan --max-count 1
ros2 topic echo /oak/depth/image_raw --max-count 1

# Check launch system
ros2 launch labrobot pi.full.launch.py
```

## Development Notes

### Adding New Navigation Behaviors
1. Modify `analyze_lidar_obstacles()` in `autonomous_navigation_node.py`
2. Update `navigation_loop()` for new decision logic
3. Add corresponding MicroPython commands if needed

### Tuning Parameters
- **Obstacle Distance**: Increase for more cautious navigation
- **Speed Settings**: Adjust for robot weight/power
- **Scan Range**: Modify for different sensor coverage
- **Timeout Values**: Balance safety vs responsiveness

## Safety Considerations

⚠️ **IMPORTANT SAFETY NOTES**:
- Always test in a safe, enclosed area first
- Keep emergency stop accessible at all times
- Monitor initial runs closely
- Verify obstacle detection before autonomous operation
- Check motor direction/wiring before first use

## Status Monitoring

The system provides comprehensive status information:
- **Controller connectivity and health**
- **Autonomous mode state**
- **Current movement and speed**
- **Obstacle detection status**
- **Emergency stop state**
- **Command timing and safety**

Use `nav_control.py --monitor` for real-time status display.

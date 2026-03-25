# 2026-03-21 — Project Discovery

## Overview

Resumed the labrobot project after a hiatus. Connected to the robot Pi and did a full audit of all existing code. This note captures the complete picture of what exists, what works, and why odometry failed.

---

## System Architecture

```
[Windows PC / WSL2 "AURORA"]  <--SSH-->  [Raspberry Pi 4B "robotpi4b8g01"]
     ~/dev_ws/src/robot/                      ~/lab_ws/src/labrobot/
     (empty skeleton, unused)                 (real working codebase)
                                                      |
                                              USB serial (115200 baud)
                                                      |
                                           [Pico W - MicroPython]
                                           upython/ros2_serial_main.py
```

**Key point**: `~/dev_ws/src/robot/` on AURORA is a blank placeholder. All real code lives on the Pi at `~/lab_ws/src/labrobot/`.

---

## Hardware Inventory

| Component | Details |
|-----------|---------|
| SBC | Raspberry Pi 4B 8GB, Ubuntu, ROS 2 Jazzy |
| Microcontroller | Raspberry Pi Pico W (MicroPython firmware) |
| Drive | Differential drive — 2 driven wheels + 2 caster wheels |
| LiDAR | SLAMTEC RPLidar C1 (USB via CP2102N UART bridge) |
| Camera / IMU | Luxonis OAK-D Lite (depth + stereo + IMU) |
| Battery monitor | INA219 (code in `~/labrobot_mpy/battery_management/`) |

### Robot Physical Dimensions (from `notes/measurements.yaml` + URDF)

| Parameter | Value | Source |
|-----------|-------|--------|
| Wheel diameter | 65 mm | measurements.yaml + URDF |
| Wheel radius | 32.5 mm | derived |
| Wheel width (tread) | 26 mm | measurements.yaml |
| Chassis width | 321 mm | measurements.yaml + URDF |
| Chassis depth | 368 mm | measurements.yaml + URDF |
| Chassis height | 620 mm | measurements.yaml + URDF |
| Wheel center-to-center | ~347 mm | derived from URDF: `2 × (chassis_width/2 + wheel_length/2)` = `2 × 173.5` |
| Motor mount offset | 20 mm | URDF |

### Pico Motor Pins

| Motor | Enable | Dir 1 | Dir 2 |
|-------|--------|-------|-------|
| Left (A) | PWM Pin(2) | Pin(3) | Pin(4) |
| Right (B) | PWM Pin(8) | Pin(6) | Pin(7) |

### Pico Encoder Pins

| Encoder | Channel A | Channel B |
|---------|-----------|-----------|
| Left | Pin(9) | Pin(10) |
| Right | Pin(11) | Pin(12) |

### Encoder Parameters (from Pico firmware)

| Parameter | Value | Note |
|-----------|-------|------|
| Pulses per revolution | **3436** | Set in `EncoderState.__init__` |
| Wheel circumference | π × 65 ≈ 204.2 mm | derived |
| mm per pulse | 204.2 / 3436 ≈ **0.0594 mm** | computed at runtime |
| Wheel base (firmware) | **150 mm** | ⚠️ WRONG — hardcoded, does not match physical robot |
| Wheel base (URDF) | ~347 mm | correct physical value |

---

## Software Inventory (Pi: `~/lab_ws/src/labrobot/`)

### Scripts

| File | Purpose | Status |
|------|---------|--------|
| `scripts/serial_motor_bridge.py` | ROS 2 node — JSON serial bridge to Pico; publishes `/odom`, `/motor_controller/*`; handles `/cmd_vel` | Runs, but **odometry is broken** (see below) |
| `scripts/basic_odom_publisher.py` | Publishes static (0,0,0) odometry at 10 Hz | Placeholder — not real odometry |
| `scripts/autonomous_navigation_node.py` | LIDAR-based obstacle avoidance; sends move/rotate commands via serial bridge | Untested with real encoders |
| `scripts/manual_robot_control.py` | CLI for manual movement | Working |
| `scripts/nav_control.py` | Service-call CLI for navigation | Working |
| `scripts/reset_emergency_stop.py` | Resets Pico emergency stop | Working |
| `scripts/robot_perception_monitor.py` | Monitors sensor topics | Working |

### MicroPython (Pico) — `upython/ros2_serial_main.py`

Full JSON serial protocol, quadrature encoder ISRs, odometry math, motor PWM control, safety timeout (2s), speed ramping, emergency stop.

**Serial protocol summary:**

```
Direction: Pi → Pico (commands)
Format: {"cmd": "<command>", ...}\n

Direction: Pico → Pi (responses)
Format: {"status": "ok"|"error", ...}\n  or  {"type": "status"|"log"|"diagnostics", ...}\n
```

**Key commands:**

| Command | Payload | Response |
|---------|---------|----------|
| `get_encoders` | `{"cmd": "get_encoders"}` | `{"status":"ok","encoder_data":{"left_count":N,"right_count":N,"left_distance_mm":F,"right_distance_mm":F,"center_distance_mm":F,"rotation_rad":F}}` |
| `status` | `{"cmd": "status"}` | `{"type":"status","autonomous_mode":bool,"is_moving":bool,...}` — does NOT include encoder data |
| `move` | `{"cmd":"move","dir":"forward"\|"backward","speed":0-100,"duration":0}` | `{"status":"ok"}` |
| `rotate` | `{"cmd":"rotate","dir":"left"\|"right","speed":0-100,"duration":0}` | `{"status":"ok"}` |
| `stop` | `{"cmd":"stop"}` | `{"status":"ok"}` |
| `estop` | `{"cmd":"estop"}` | `{"status":"ok"}` |
| `reset_estop` | `{"cmd":"reset_estop"}` | `{"status":"ok"}` |
| `reset_encoders` | `{"cmd":"reset_encoders"}` | `{"status":"ok"}` |
| `ping` | `{"cmd":"ping"}` | `{"status":"ok","message":"pong"}` |

**Encoder counting**: Quadrature ISR on all 4 encoder pins. Counts are cumulative integers (positive = forward, negative = backward). Pico also computes odometry internally but **this is not used by the Pi**.

### URDF (`description/`)

Complete xacro-based URDF:
- `robot.urdf.xacro` — top-level, includes all sub-files
- `robot_core.xacro` — chassis, wheels, casters with correct dimensions
- `lidar.xacro` — RPLidar C1 frame
- `depth_camera.xacro` — OAK-D Lite frame
- `ros2_control.xacro` — ros2_control plugin (for simulation)
- `gazebo_control.xacro` — Gazebo diff drive plugin (commented out in robot.urdf.xacro)

Frame tree: `base_footprint → base_link → chassis, left_wheel, right_wheel, caster_wheel_f, caster_wheel_b`

### Launch Files

| File | What it launches |
|------|-----------------|
| `pi.full.launch.py` | robot_state_publisher + joint_state_publisher + RPLidar + OAK-D Lite + CV processing nodes + static tf placeholders |
| `pi.autonomous.serial.launch.py` | sensor stack + serial_motor_bridge + autonomous_navigation_node |
| `pi.basic.sensors.launch.py` | robot_state_publisher + RPLidar + OAK-D Lite |
| `pi.lidar.launch.py` | RPLidar only |
| `rsp.launch.py` | robot_state_publisher only |

### Other Workspaces on Pi

| Path | Contents |
|------|---------|
| `~/depthai_ws/` | depthai-ros (OAK-D Lite ROS 2 driver, built) |
| `~/lidar_ws/` | rplidar_ros (older, superseded by sllidar_ros2 in lab_ws) |
| `~/labrobot_mpy/battery_management/` | INA219 battery monitor MicroPython code |
| `~/labrobot_dev/motor_test.py` | Standalone motor test script |

---

## Current Hardware State (as of 2026-03-21)

| Component | Connected | Powered | Notes |
|-----------|-----------|---------|-------|
| Raspberry Pi 4B | ✓ | ✓ | SSH reachable at 192.168.25.135 |
| Pico W | ✓ (USB serial) | ✓ (via USB) | Serial comms working |
| Encoders | ✓ (wired to Pico) | ✓ | Can test by spinning wheels by hand |
| RPLidar C1 | ✓ | ✓ | |
| OAK-D Lite | ✗ | ✗ | Not connected |
| Drive motors | ✓ (wired to Pico) | ✗ | No motor power — robot won't move |

**What this means for development:**
- Serial bridge and encoder reading can be tested now (spin wheels by hand)
- LiDAR can be tested
- Motor commands can be sent without risk of movement (safe for software testing)
- OAK-D Lite / IMU not available — skip those nodes in launch files for now
- Any launch file that starts the OAK-D Lite node will error — use `pi.basic.sensors.launch.py` with lidar only, or a trimmed launch

---

## Root Cause of Odometry Failure

### The core bug

**`serial_motor_bridge.py` never polls the Pico for encoder data.**

The bridge only sends `{"cmd": "status"}` every 1 second (via `request_status()` timer). The `status` response does NOT include encoder counts. The bridge's `process_incoming_message()` only handles `type == "status"` and `type == "log"` — there is no encoder data path.

The odometry variables (`odom_x`, `odom_y`, `odom_theta`, `last_left_distance`, `last_right_distance`) are declared in `__init__` but **never updated**. The `odom_publisher` is created but **never published to**.

### The placeholder workaround

`basic_odom_publisher.py` was added as a stopgap. It publishes `nav_msgs/Odometry` at (0, 0, 0) forever. The launch files use `static_transform_publisher` for `odom → base_footprint` — a fixed identity transform, not a real odometry feed.

### Secondary bug: wrong wheel base in Pico firmware

`EncoderState.wheel_base_mm = 150` but physical measurement from URDF gives ~347mm. Even if the Pi were polling encoder data, the Pico's own odometry computation would be wrong by ~2.3×. The fix is to either correct this value in the firmware OR compute odometry entirely on the Pi side using raw `left_count` / `right_count`.

### Summary of what needs to be fixed

1. **Modify `serial_motor_bridge.py`** to periodically poll `get_encoders`, compute differential drive odometry from cumulative count deltas, and publish real `nav_msgs/Odometry` + broadcast dynamic `odom → base_footprint` tf.
2. **Remove `basic_odom_publisher.py`** from any launch file that uses it alongside the bridge.
3. **Remove static `odom → base_footprint` transform** from launch files — it conflicts with the dynamic tf from the odometry node.
4. **Fix `wheel_base_mm`** in `upython/ros2_serial_main.py` (or just ignore the Pico's computed odometry and compute it on the Pi with the correct value).
5. **Verify encoder CPR** (3436 counts/rev) against actual hardware — this determines odometry scale accuracy.

---

## Development Environment

### SSH access (set up 2026-03-21)
```bash
# From AURORA WSL2:
ssh robot   # configured in ~/.ssh/config → botmanager@192.168.25.135
```

### Building on Pi
```bash
ssh robot
cd ~/lab_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select labrobot
source install/setup.bash
```

### Running the stack
```bash
# Sensors + motor bridge (no autonomous):
ros2 launch labrobot pi.basic.sensors.launch.py

# Full autonomous:
ros2 launch labrobot pi.autonomous.serial.launch.py
```

### Checking serial port
```bash
ls /dev/ttyACM*    # Pico W
ls /dev/serial/by-id/  # RPLidar (CP2102N)
```

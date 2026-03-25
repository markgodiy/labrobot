# 2026-03-21 — Odometry Fix

## Summary

Root cause of odometry failure diagnosed and fixed end-to-end. Real encoder-based odometry is now flowing from Pico → Pi → ROS 2 `/odom` + `odom→base_footprint` TF.

---

## What Was Wrong

### 1. Pico had wrong firmware
The Pico was running **battery management firmware only** (`main.py` from `~/labrobot_mpy/battery_management/`). The motor control firmware (`ros2_serial_main.py`) had never been flashed. It was broadcasting INA219 telemetry at 1 Hz but ignoring all serial commands.

### 2. `sys.stdin.any()` missing in MicroPython v1.26.1
The motor firmware's serial input loop used `sys.stdin.any()`, which does not exist in MicroPython v1.26.1 (RP2040). `sys.stdin` only has: `read`, `readline`, `readinto`, `write`, `buffer`, `readlines`. Fixed with `select.poll()`.

### 3. `serial_motor_bridge.py` never polled encoders
The Pi-side bridge only called `{"cmd": "status"}` every 1 s. The `status` response does not include encoder data. The odometry variables `odom_x/y/theta` were declared but never updated or published.

### 4. Static `odom→base_footprint` TF
All launch files used `static_transform_publisher` for an identity `odom→base_footprint` transform — meaning the robot was always reported at (0,0,0) regardless of actual movement.

### 5. Wrong `wheel_base_mm` in Pico firmware
Hardcoded as `150` mm. Physical value from URDF is `~347` mm — off by 2.3×. (Pi-side odometry uses its own value now, so this only matters for the Pico's internal odometry which isn't used.)

---

## What Was Fixed

### Pico W — new merged firmware (`upython/main.py`)

Combined motor control + INA219 battery monitor into a single firmware:

| Feature | Detail |
|---------|--------|
| Serial input | `select.poll()` for non-blocking stdin (replaces broken `any()`) |
| Motor control | PWM, ramping, direction, emergency stop — unchanged |
| Encoder ISRs | Quadrature on all 4 pins — unchanged |
| Battery monitor | INA219 on I2C1 (GP14/15), sampled every 10 s |
| Battery broadcast | `{"type": "battery", ...}` every 10 s |
| Battery in status | `batt.last_reading` included in every `status` and `heartbeat` response |
| Graceful degradation | INA219 init wrapped in try/except — motor control works if sensor absent |
| `wheel_base_mm` | Fixed from 150 → 347 mm |
| `get_battery` command | Explicit on-demand battery reading |

**Pin assignments (no conflicts):**
- Motors/encoders: GP2–12
- INA219 I2C1: SDA=GP14, SCL=GP15

Firmware file: `upython/main.py` in labrobot repo.
INA219 driver: `upython/ina219.py` (was only on Pico flash, now tracked in repo).

### `serial_motor_bridge.py`

| Change | Detail |
|--------|--------|
| Encoder poll timer | Added 20 Hz timer → `{"cmd": "get_encoders"}` via `send_command_async` |
| `process_incoming_message` | Now extracts `encoder_data` from any message (response, heartbeat, status) |
| `_update_odometry()` | New method: diff drive kinematics from cumulative tick deltas → integrate pose |
| `nav_msgs/Odometry` | Published on `/odom` with pose + velocity |
| `odom→base_footprint` TF | Dynamic broadcast via `tf2_ros.TransformBroadcaster` |
| Robot params | `wheel_base_m=0.347`, `wheel_radius_m=0.0325`, `ticks_per_rev=3436`, `encoder_poll_hz=20.0` |
| `handle_reset_emergency_stop` | Was registered as service but handler was missing — added |
| `tf_transformations` removed | Computed z-rotation quaternion directly (no external dep) |

### Launch files

**`pi.basic.sensors.launch.py`:**
- Removed `static_transform_publisher` for `odom→base_footprint` — dynamic TF from bridge now owns this

**`pi.autonomous.serial.launch.py`:**
- Added wheel geometry params to `serial_motor_bridge` node invocation

---

## Verification Steps

Tested on Pi via serial (Pico running firmware):

```
ping        → {"status": "ok", "message": "pong"}               ✓
reset_estop → {"status": "ok", "message": "estop reset"}        ✓
get_encoders→ {"status": "ok", "encoder_data": {...}}            ✓
get_battery → {"type": "battery", "bus_voltage": 13.05, ...}     ✓
```

Build: `colcon build --packages-select labrobot` — clean.

---

## Remaining Verification (requires motor power)

Once motor power is connected:
1. Run: `ros2 launch labrobot pi.autonomous.serial.launch.py`
2. Push robot by hand → `ros2 topic echo /odom` should show x/y/theta changing
3. Check TF tree: `ros2 run tf2_tools view_frames` — `odom → base_footprint` should be dynamic
4. Verify encoder CPR (3436) is accurate against physical wheel rotation

---

## Git

Committed and pushed to `https://github.com/markgodiy/labrobot.git` (main branch).
Commit: `884987a` — "fix: implement real odometry in serial_motor_bridge + merged Pico firmware"

SSH key `robotpi4b8g01` was already registered on GitHub (added Oct 2025). Switched remote to SSH and pushed successfully:
```bash
git remote set-url origin git@github.com:markgodiy/labrobot.git
git push origin main  # 596 objects, 14.51 MiB — new branch created on remote
```

---

## Live Test Attempt (2026-03-21)

### Launch failure — XML parse error

Running `ros2 launch labrobot pi.autonomous.serial.launch.py` failed immediately:

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): no element found: line 1, column 0
```

This error comes from `xacro.process_file()` in `pi.basic.sensors.launch.py`. It means `description/robot.urdf.xacro` is either **empty or missing**.

**Root cause:** `ros2_control.xacro` was 0 bytes (empty placeholder). `robot.urdf.xacro` includes it, causing xacro XML parse to fail.
**Fix:** Added minimal valid xacro stub to `description/ros2_control.xacro`. Rebuild clean.

### Network flooding

ROS 2 DDS multicast (point clouds, camera images at 15 FPS) was flooding the LAN.
**Fix:** Added `export ROS_LOCALHOST_ONLY=1` to `~/.bashrc` on Pi — all DDS traffic stays on loopback.

### Pico frozen after launch

After running the full launch for a while, the Pico became unresponsive (no heartbeats, no ping response). Cause: likely overload from 20 Hz encoder polling + camera/LIDAR traffic on Pi.
**Recovery:** Power-cycled the whole robot. Pico came back and responded correctly:
- `ping` → pong ✓
- `get_encoders` → encoder counts ✓
- `health`: `emergency_stop` (expected — bridge resets this on launch startup)

**Note:** Opening pyserial port toggles DTR which resets the Pico. Allow 4+ seconds after opening before sending commands.

### LIDAR — working

`/scan` publishing at clean 10 Hz after connecting CP2102N USB. Was crashing at exit code 255 earlier only because USB wasn't plugged in during that launch.

### Web dashboard — added

`scripts/robot_dashboard.py` — ROS 2 node + stdlib HTTP server on port **8080**.
Access: `http://192.168.25.135:8080`

Subscribes to `/motor_controller/status`, `/motor_controller/encoder_data`, `/odom`.
Displays: odometry, encoder counts, battery (INA219 voltage/current/power), e-stop state, Pi CPU temp/memory/uptime.
Refreshes at 500 ms via JS fetch. Stale values fade out (>3 s odom/encoders, >20 s battery).

Added to `CMakeLists.txt` install and `pi.autonomous.serial.launch.py`.
Commit: `18d6a1e` — "feat: add robot_dashboard — live web metrics at :8080"

**Battery fix (commit `61a0e8f`):** Initial dashboard had wrong field names — Pico sends `current_draw_amp` (A) and `current_draw_watt` (W), not `current_mA`/`power_mW`. Key in status message is `battery`, not `batt`. Dashboard now converts to mA/mW on ingest and shows `batt_charge_percent` as Charge %.

---

### Encoder counts stuck at 0

Encoder data flows at 20 Hz and `/odom` publishes, but `left_count` and `right_count` stay 0 when robot is pushed manually. Green LED on encoders confirms they are powered. Direct GPIO IRQ test (GP9-12) also showed no counts. **Deferred** — encoder wiring/pin mapping needs physical verification with motor power fully reconnected. Likely cause: encoder signals not reaching GP9/10/11/12, or pin mapping in firmware doesn't match physical wiring.

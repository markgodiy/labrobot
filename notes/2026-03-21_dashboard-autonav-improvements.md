# 2026-03-21 — Dashboard & Autonomous Navigation Improvements

Commit `f0b22c2` — "fix: LIDAR 180° mount correction in nav node and dashboard"
Pushed to `git@github.com:markgodiy/labrobot.git` (main branch).

Files changed:
- `scripts/autonomous_navigation_node.py` — LIDAR sector fix, stop_movement deadlock fix
- `scripts/robot_dashboard.py` — car-dashboard layout, LIDAR radar, nav state card

---

## Issue 1: LIDAR Mounted 180° Rotated

### Discovery
The LIDAR radar visualization on the dashboard showed the room layout mirrored
front-to-back. The physical mounting places the LIDAR sensor facing the robot's
rear. Confirmed by comparing the live radar with the known room layout.

### Impact on nav node (critical)
`analyze_lidar_obstacles` used `center_index = n // 2` as the "front" sector.
With the sensor mounted backward, index `n//2` corresponds to LIDAR angle 0°
which is the robot's **rear**. The robot was detecting and avoiding obstacles
behind it, while driving blindly forward into actual obstacles.

Left/right sectors were also inverted: LIDAR's first quarter (low angle indices)
is the robot's right side; LIDAR's last quarter is the robot's left.

### Fix — autonomous_navigation_node.py

**Front sector**: wrap around index 0 (LIDAR angle ±π = robot forward):
```python
# Before (WRONG — reads robot rear):
center_index = len(scan_msg.ranges) // 2
start_idx = max(0, center_index - range_indices)
end_idx = min(n, center_index + range_indices)
front_ranges = scan_msg.ranges[start_idx:end_idx]

# After (CORRECT — wraps around index 0 for 180° mount):
half_idx = int(half_range / scan_msg.angle_increment)
front_indices = list(range(n - half_idx, n)) + list(range(0, half_idx))
front_ranges_raw = [scan_msg.ranges[i] for i in front_indices]
```

**Left/right swap**:
```python
# Before (WRONG — swapped for 180° mount):
left_ranges  = scan_msg.ranges[:quarter_point]
right_ranges = scan_msg.ranges[-quarter_point:]

# After (CORRECT):
right_ranges = scan_msg.ranges[:quarter_point]   # LIDAR left = robot right
left_ranges  = scan_msg.ranges[-quarter_point:]  # LIDAR right = robot left
```

### Fix — robot_dashboard.py (_on_scan)
Same front sector wrapping and left/right swap applied to the dashboard's
sector clearance display (Front/Left/Right meters). Uses `CHASSIS_MIN = 0.30 m`
instead of `msg.range_min` (~0.1 m) to suppress chassis frame returns.

### Fix — LIDAR radar canvas (JS)
Display rotation: changed `cx - sin(angle)` to `cx + sin(angle)` (and same
for y) to rotate the polar plot 180°, matching physical robot orientation.

---

## Issue 2: stop_movement() Executor Deadlock

### Symptom
`stop_movement()` called `rclpy.spin_until_future_complete(self, future, ...)`.
This is invoked from `navigation_loop`, which runs inside a 10 Hz timer callback
while the node is already spinning via `rclpy.spin(node)`. Result: "Executor is
already spinning" exception — stop commands silently failed.

### Fix
```python
# Before (DEADLOCK):
def stop_movement(self):
    request = Trigger.Request()
    future = self.motor_stop_client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

# After (safe — cmd_vel publish is non-blocking):
def stop_movement(self):
    return self.send_movement_command(0.0, 0.0)
```

The same pattern was already fixed in `emergency_stop_callback` and
`set_autonomous_mode_callback` in the previous session. `initialize_controller`
still uses `spin_until_future_complete` but runs before `rclpy.spin()` starts,
so it is not affected.

---

## Issue 3: Dashboard Chassis Frame Returns (Red Dots at ~0.1 m)

### Symptom
4 red dots clustered around the robot icon on the LIDAR radar — the 4 vertical
corner posts of the shelving unit chassis surrounding the LIDAR sensor.

### Root Cause
`_on_scan` filtered using `msg.range_min` (~0.1 m hardware minimum), which
passed the chassis posts (~0.1–0.15 m from the LIDAR) through to the display.

### Fix
```python
CHASSIS_MIN = 0.30  # Filter out chassis posts (~10–15 cm from LIDAR)
# Applied to both visualization points and sector clearance calculations
```

---

## Dashboard Layout Redesign — Car Dashboard Style

Replaced `auto-fill` CSS grid with a fixed 3-column car dashboard layout:

```
┌─────────────────┬──────────────────────────┬─────────────────┐
│  NAVIGATION     │   LIDAR TOP-DOWN VIEW    │  ENCODERS       │
│  (mode/action/  │   (520×520 canvas,       │  BRIDGE STATUS  │
│   path/sectors) │    updates at 2.5 Hz)    ├─────────────────┤
├─────────────────┤                          │  PI SYSTEM      │
│  ODOMETRY       ├──────────────────────────┤                 │
│  (x/y/heading/  │  CONTROL                 ├─────────────────┤
│   speed)        │  (d-pad + speed slider)  │  PICO W (MC)    │
├─────────────────┤                          │                 │
│  BATTERY        │                          │                 │
│  (INA219)       │                          │                 │
└─────────────────┴──────────────────────────┴─────────────────┘
                      LOG STREAM (full width)
```

CSS: `grid-template-columns: 260px 1fr 260px` with `@media` fallback to 1-column
on narrow screens (<900 px).

---

## New Dashboard Features Added (this session)

### Navigation card
- Autonomous mode toggle button → POST `/nav` → calls `/set_autonomous_mode` service
- Action label color-coded: green=forward, blue=rotating, red=escape, yellow=backward
- Front/Left/Right sector clearances in meters (color: red<0.5m, yellow<1m, green>1m)
- Subscribes to `/navigation/debug` (1 Hz)

### LIDAR radar canvas
- Top-down polar view, 3 m radius, 2.5 Hz updates
- Points: red <0.5 m, yellow <1 m, green >1 m
- Blue triangle = robot (pointing forward = up)
- Subtle blue cone = ±45° forward scanning arc
- Dashed red ring = 0.5 m obstacle threshold
- Shows "No LIDAR data" text when `/scan` topic is stale
- Subscribes to `/scan` (BEST_EFFORT QoS, throttled to 2.5 Hz)

---

## Current State After This Session

- Dashboard: car-dashboard layout, LIDAR radar working and correctly oriented
- Nav node: obstacle analysis now reads correct robot front and correct left/right
- stop_movement(): no longer deadlocks executor
- Chassis frame dots: suppressed by 0.30 m minimum range filter
- Git: up to date on GitHub main branch (commit f0b22c2)

## Remaining Before Floor Test

- Nav node's `initialize_controller` still calls `spin_until_future_complete` —
  low risk (runs before spin starts) but worth cleaning up eventually
- Heading display occasionally shows >180° — cosmetic, wrap with modulo if needed
- Left motor backward asymmetry (~99 edges/s vs ~1965 forward) will cause
  backup maneuvers to pull right — acceptable for initial testing, may need
  PID or speed compensation later
- Consider lowering default_speed from 90% to 60–70% for first floor test

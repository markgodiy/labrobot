# 2026-03-21 — Full Troubleshooting Session Log

This document captures every issue diagnosed and resolved across multiple
sessions to get the robot to a working autonomous navigation state.

---

## Environment

- **Pi**: Raspberry Pi 4B 8GB, ROS 2 Jazzy, `~/lab_ws/src/labrobot`
- **Pico W**: MicroPython v1.25.0, firmware at `upython/main.py`
- **WSL**: Ubuntu on Windows, `~/dev_ws/src/robot` (synced to Pi via scp + git)
- **Git remote**: `git@github.com:markgodiy/labrobot.git`
- **Dashboard**: `http://192.168.25.135:8080`

---

## Issue 1: Motors Never Running — H-Bridge STBY Pins Floating

### Symptom
Pico firmware sent PWM signals, serial commands returned `ok`, but motors never
moved. INA219 showed no current increase. Encoder counts stayed at 0.

### Root Cause
The TB6612FNG-style H-bridge ICs have a **STBY (standby) pin** that must be
held HIGH to enable motor output. Both H-bridge ICs had their STBY pins
unconnected in `main.py` — the pins were left floating (effectively LOW), so
both H-bridges stayed permanently in standby mode.

There are **two separate H-bridge ICs** on the PCB:
| IC | STBY GPIO | Motor side |
|----|-----------|------------|
| Left | GP5 | Motor A (Left) |
| Right | GP13 | Motor B (Right) |

### Why It Was Hidden During pin_discover.py
`pin_discover.py`'s `_safe_inputs()` applied `PULL_UP` to ALL GPIOs, which
accidentally pulled STBY HIGH and made motors run during discovery. In normal
`main.py` with no pull-ups, STBY was floating LOW.

### Discovery Method
Three diagnostic scripts run via `mpremote exec`:
1. `motor_stby_test.py` — confirmed PULL_UP is needed (4000 edges vs 0 without)
2. `find_stby.py` — iterated all non-motor GPIOs, set each HIGH, counted encoder
   edges → GP5 HIGH gave ~1565 edges/0.8s (left motor)
3. `find_stby2.py` — same with GP5 already HIGH → GP13 gave ~1571 edges/0.8s
   (right motor)

### Fix
```python
# In upython/main.py — add immediately after motor pin definitions
stby_l = Pin(5,  Pin.OUT); stby_l.high()   # Left H-bridge STBY
stby_r = Pin(13, Pin.OUT); stby_r.high()   # Right H-bridge STBY
```

---

## Issue 2: Encoder Counts Always Return 0 — IRQ Timing

### Symptom
After STBY fix, motors spin but left_count/right_count stay 0 in ROS topics.
Green LEDs on encoder boards confirmed encoder power.

### Root Cause
MicroPython soft IRQ handlers are delivered between bytecodes, not
immediately. At 2000 Hz per channel (~500μs period), the quadrature phase
offset between channels A and B is ~250μs. By the time `_update_left` runs,
**both** A and B pins have already transitioned → handler sees `a == b` on
every call → alternating +1 / -1 → net count = 0.

This is specific to `main.py`'s complex loop context creating IRQ delivery
delay. Confirmed: direct GPIO polling detected ~4000 edges/2s correctly.
A-channel-only counting in a tight loop also worked.

### Fix
Replace quadrature ISRs with directional A-channel counting:

```python
class EncoderState:
    __slots__ = ("left_count", "right_count")
    def __init__(self):
        self.left_count  = 0
        self.right_count = 0

def _update_left(_pin):
    enc.left_count += nav.left_dir   # +1 forward, -1 backward, 0 stopped

def _update_right(_pin):
    enc.right_count += nav.right_dir
```

Direction is set in `_move()` / `_rotate()` and cleared to 0 in
`_smooth_stop()` / `_emergency_stop()`. B-channel IRQs removed entirely.
IRQ attachment moved AFTER `nav = NavState()` (handlers reference `nav`).

### Verification
After fix: `left_count=83610, right_count=83633` after ~5m travel (<0.03% error).

---

## Issue 3: INA219 Misleading During Diagnosis

### Finding
INA219 measures **logic/supply rail** current only, not motor drive current.
During early diagnosis, no current increase when commanded to move led to
false conclusion that motors were running (they weren't — STBY was LOW).

Motor drive current would require a shunt resistor in the motor power line
wired to INA219 IN+ / IN−. Currently not wired this way.

**Lesson**: Never use INA219 readings to verify motor activity on this robot.
Use encoder edge counts or direct GPIO polling instead.

---

## Issue 4: Encoder Counts 0 Even With STBY Fixed

### Symptom
After fixing STBY pins, during a backward-move test: `left_count=0` during
backward motion (forward motion worked fine).

### Finding
Left motor backward is very slow mechanically (~99 edges/s vs ~1965 forward).
The directional count fix uses `nav.left_dir = -1` for backward. With 99
pulses/s, the count was non-zero but nearly zero over short test windows.
This appears to be mechanical asymmetry (worn motor/gearbox), not software.

Not a blocking issue for navigation — the robot navigates forward primarily.

---

## Issue 5: Autonomous Navigation Continuously Commanding Forward

### Symptom
Robot wheels spin continuously when ROS stack launches, even with nothing in
the way. Nav node logs showed constant "Moving forward at 90%" messages.

### Root Causes (two separate bugs)

**A) Stale LIDAR data:**
LIDAR publishes one scan before crashing (USB not fully connected, etc.).
`latest_scan` is set from that one message → not None → nav loop continuously
commands forward forever, even with no active LIDAR.

**Fix:** Track `latest_scan_time` and `latest_depth_time`. In `navigation_loop`,
discard data older than 2 seconds:
```python
scan_data = self.latest_scan if (
    self.latest_scan_time and now - self.latest_scan_time < self.sensor_timeout
) else None
```

**B) 20Hz encoder polling resets Pico safety timer:**
`poll_encoders()` sends `get_encoders` every 50ms. On the Pico, any received
command resets `last_cmd_time`. So the Pico's internal safety timer (which
stops motors if no command arrives) never fires, even after nav node stops
publishing cmd_vel.

**Fix:** Added `check_cmd_vel_watchdog()` timer (1Hz) to the bridge. If a
cmd_vel move was in progress and no new cmd_vel arrives for 5 seconds, the
bridge sends `{"cmd": "stop"}` to the Pico directly.

---

## Issue 6: Emergency Stop / Autonomous Mode Service Deadlock

### Symptom
Calling `/emergency_stop` service returned:
`"Emergency stop error: Executor is already spinning"`

### Root Cause
`emergency_stop_callback` and `set_autonomous_mode_callback` called
`rclpy.spin_until_future_complete(self, future, ...)` from inside a service
callback. The node is already being spun by `rclpy.spin(node)` → calling
`spin_until_future_complete` on the same executor causes a deadlock/exception.

### Fix
Replace with `call_async()` fire-and-forget. The response isn't needed
synchronously — just sending the command is sufficient:
```python
# Instead of:
future = self.motor_stop_client.call_async(request)
rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

# Use:
self.motor_stop_client.call_async(request)  # fire and forget
```
Also send immediate `cmd_vel` zero twist as belt-and-suspenders stop.

---

## Issue 7: Dashboard Showing "Connecting..." — JS Script Block Failure

### Symptom
Dashboard at port 8080 loaded but showed "Connecting..." with all "—" values.
`curl http://localhost:8080/metrics` returned complete, correct JSON data.

### Root Cause: Python Triple-Quoted String + JS String Literals

The dashboard HTML is a Python triple-quoted string. Inside it, the JS
`copyAllLogs()` function had **literal newline characters** inside JS
single-quoted strings:

```python
# In Python source (triple-quoted string) — WRONG:
navigator.clipboard.writeText(lines.join('\n'))
#                                          ^^
# Python expands \n → literal newline → JS sees unclosed string → SyntaxError
```

A JavaScript `SyntaxError` in a `<script>` block fails **silently** in the
browser but prevents the **entire script block** from executing. So
`setInterval(poll, 500)` was never registered, the badge never updated, and
the dashboard stayed "Connecting..." forever.

Additionally, `Array.from(.children)` was missing the argument (`$('logbox')`
was dropped), which would also crash the function.

### Fix

```python
# In Python source — CORRECT:
navigator.clipboard.writeText(lines.join('\\n'))
#                                          ^^^
# Python sees \\n → outputs \n → JS sees valid \n escape in string
```

### Diagnosis Trail
1. `curl /metrics` confirmed server-side data correct → client-side JS issue
2. Fetched live HTML via `curl http://192.168.25.135:8080/` to WSL
3. Used Python `re` to extract script blocks and check for odd-quote lines
4. Found lines with `lines.join('` then newline then `')` — unclosed strings
5. Traced back to Python source: `'\n'` in triple-quoted string

### How To Avoid In Future
When embedding JS inside Python triple-quoted HTML strings:
- Use `'\\n'` not `'\n'` for newline literals in JS strings
- Prefer template literals (backticks) in JS where newlines are allowed
- Or use `'\\n'` and document why

### Browser Cache Complication
Even after deploying the fix, browser served old broken JS from cache.
**Always use `Ctrl+Shift+R`** (hard refresh) after any dashboard JS change.

---

## Issue 8: Multiple ROS2 Stacks Running Simultaneously

### Symptom
Two serial_motor_bridge instances both trying to use `/dev/ttyACM0`.
Motors spinning erratically, dashboard getting conflicting data.

### Cause
Background tasks from previous SSH sessions left old stack running.
New launch added a second stack without killing the first.

### Fix / Prevention
Before relaunching, always kill all existing processes:
```bash
pkill -9 -f "ros2 launch"
pkill -9 -f "serial_motor_bridge"
pkill -9 -f "autonomous_navigation"
pkill -9 -f "robot_dashboard"
sleep 2
```
Or check with: `ps aux | grep python3 | grep -v grep`

### Warning Sign
If `ps aux` shows two processes with the same script name but different PIDs
and different start times → duplicate stack. Kill all and relaunch once.

---

## Issue 9: Dashboard Node Not Starting With Launch File

### Symptom
After fresh launch, dashboard not accessible. `pgrep -f robot_dashboard` empty.

### Cause
Dashboard crashed on startup (likely ROS 2 topic subscription error or
port conflict with leftover process).

### Workaround
Start dashboard manually outside the launch file:
```bash
source /opt/ros/jazzy/setup.bash && source ~/lab_ws/install/setup.bash
nohup python3 ~/lab_ws/src/labrobot/scripts/robot_dashboard.py \
  --ros-args -p port:=8080 > /tmp/dashboard.log 2>&1 &
```
Check logs: `tail -f /tmp/dashboard.log`

---

## Issue 10: SSH Commands Returning Exit Code 255

### Finding
`ssh user@host 'pkill -f something; echo ok'` returns exit code 255 if
`pkill` finds nothing to kill (exit code 1) and ssh interprets the session
as failed.

### Workaround
Use `pkill -f something 2>/dev/null; true` or check return separately.
Or use `pgrep` first to check existence before killing.

---

## Confirmed Working Pin Mapping (from pin_discover.py + STBY discovery)

```
Motor A (Left):   ENA=GP2, IN1=GP3, IN2=GP4
Left H-bridge:    STBY=GP5  ← MUST be HIGH
Motor B (Right):  ENB=GP10, IN3=GP11, IN4=GP12
Right H-bridge:   STBY=GP13 ← MUST be HIGH
Encoder Left:     A=GP16, B=GP17
Encoder Right:    A=GP18, B=GP19
INA219 I2C:       SDA=GP14, SCL=GP15
```

---

## Standard Restart Procedure (Working)

```bash
# 1. Kill everything on Pi
ssh botmanager@192.168.25.135
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "serial_motor_bridge" 2>/dev/null
pkill -9 -f "autonomous_navigation" 2>/dev/null
pkill -9 -f "robot_dashboard" 2>/dev/null
sleep 2

# 2. Rebuild (only needed after code changes)
cd ~/lab_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select labrobot --symlink-install

# 3. Launch stack
source install/setup.bash
ros2 launch labrobot pi.autonomous.serial.launch.py enable_camera:=false

# 4. If dashboard doesn't start with launch, start manually:
nohup python3 ~/lab_ws/src/labrobot/scripts/robot_dashboard.py \
  --ros-args -p port:=8080 > /tmp/dashboard.log 2>&1 &
```

---

## Git Commit Reference

Commit `c203243` — "fix: H-bridge STBY pins, encoder IRQ, dashboard JS, nav safety"
Pushed to `git@github.com:markgodiy/labrobot.git` (main branch).

Files changed:
- `upython/main.py` — STBY pins + directional encoder counting
- `scripts/robot_dashboard.py` — JS `\\n` escape fix
- `scripts/autonomous_navigation_node.py` — sensor freshness + executor deadlock fix
- `scripts/serial_motor_bridge.py` — cmd_vel watchdog

---

## Current Status (end of session)

- Dashboard: **fully working** — Bridge connected, battery, encoders, system all live
- Motors: **running** (wheels spinning in autonomous mode with LIDAR active)
- Encoder counts: **accurate** (<0.03% left/right symmetry over 5m)
- Control buttons: **working** from dashboard
- Autonomous nav: **fixed** (sensor freshness + deadlock fixes applied)
- Git: **up to date** on GitHub main branch
- Next step: Physical floor testing with robot reassembled

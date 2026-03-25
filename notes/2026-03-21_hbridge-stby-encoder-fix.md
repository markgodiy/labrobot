# 2026-03-21 — H-Bridge STBY Pin Discovery + Encoder IRQ Fix

## Summary

Three bugs prevented the robot from moving at all during post-odometry testing. All three were
diagnosed and fixed this session. After fixes, encoder counts matched within 0.03% over ~5 m of
travel (left=83610, right=83633).

---

## Bug 1: H-Bridge STBY Pins Floating LOW

### Symptom
Motors never ran. Pico firmware sent PWM signals, INA219 showed no current increase, encoder
counts stayed at 0.

### Root Cause
The TB6612FNG-style H-bridge ICs have a STBY (standby) pin that must be held HIGH to enable the
output stage. Both H-bridge ICs had their STBY pins floating (not wired to anything in main.py),
so the H-bridges stayed in standby mode and no current reached the motors.

There are **two separate H-bridge ICs**:
- Left motor H-bridge: STBY on **GP5**
- Right motor H-bridge: STBY on **GP13**

### How STBY was discovered
`pin_discover.py`'s `_safe_inputs()` applied `PULL_UP` to all GPIOs — this accidentally pulled
STBY HIGH, making the motors run during discovery. In normal `main.py` the pins were left as
inputs without pull-ups, so STBY was effectively LOW.

Diagnosis scripts written (`/tmp/find_stby.py`, `/tmp/find_stby2.py`):
- Iterated non-motor, non-encoder GPIOs, setting each HIGH
- Polled encoder pins for edge count over 0.8 s
- GP5 HIGH → left motor: ~1565 edges / 0.8 s ✓
- GP13 HIGH → right motor: ~1571 edges / 0.8 s ✓

### Fix Applied to `upython/main.py`
```python
stby_l = Pin(5,  Pin.OUT); stby_l.high()   # Left H-bridge STBY
stby_r = Pin(13, Pin.OUT); stby_r.high()   # Right H-bridge STBY
```
Added immediately after motor pin definitions, before IRQ setup.

---

## Bug 2: Encoder IRQ Quadrature Counting Returns 0

### Symptom
Even with motors spinning and STBY fixed, quadrature IRQ handlers returned net 0 count.

### Root Cause
MicroPython soft IRQ handlers are delayed (scheduled between bytecodes). The encoder quadrature
phase offset is ~250 μs at 2000 Hz per channel. By the time `_update_left` runs, both A and B
pins have already transitioned → handler sees `a == b` → alternating +1/-1 → net 0.

Confirmed: simple polling detected ~4000 edges/2 s correctly. A-channel-only IRQ counting with
`cnt += 1` in a tight loop also worked correctly. The issue is specific to the complex main.py
loop context creating IRQ delivery delay.

### Fix Applied to `upython/main.py`
Replaced quadrature ISRs with directional A-channel counting:

```python
class EncoderState:
    __slots__ = ("left_count", "right_count")
    def __init__(self):
        self.left_count  = 0
        self.right_count = 0

def _update_left(_pin):
    enc.left_count += nav.left_dir   # +1 or -1 based on commanded direction

def _update_right(_pin):
    enc.right_count += nav.right_dir
```

- B-channel IRQs removed entirely (A channel alone gives 2000 pulses/s per direction)
- `left_dir` / `right_dir` (0/+1/-1) added to `NavState`
- Set in `_move()` and `_rotate()` before movement starts
- Zeroed in `_smooth_stop()` and `_emergency_stop()`
- IRQ attachment moved AFTER `nav = NavState()` (handlers reference `nav`)

### Confirmed Pin Mapping (from pin_discover.py)
```
Motor A (Left):  ENA=GP2,  IN1=GP3,  IN2=GP4
Motor B (Right): ENB=GP10, IN3=GP11, IN4=GP12
Left H-bridge STBY:  GP5  (must be HIGH)
Right H-bridge STBY: GP13 (must be HIGH)
Encoder Left:  A=GP16, B=GP17
Encoder Right: A=GP18, B=GP19
INA219 I2C:    SDA=GP14, SCL=GP15
```

---

## Bug 3: Dashboard "Connecting..." / All Values Blank

### Symptom
Dashboard at `http://192.168.25.135:8080` showed "Connecting..." with all "—" values, even
though the server was running and `curl http://localhost:8080/metrics` returned correct data.

### Root Cause: JavaScript Syntax Errors
Two syntax errors in the `copyAllLogs()` function inside a Python triple-quoted HTML string
caused the **entire `<script>` block to fail to parse**. The JS errors were:

1. `Array.from(.children)` — missing argument (`$('logbox')` dropped)
2. Literal newlines inside single-quoted JS string literals (should be `'\n'`)

Because the script block failed silently, `setInterval` for `/metrics` polling was never
registered → dashboard never fetched data.

### Fix Applied to `scripts/robot_dashboard.py`
```javascript
function copyAllLogs() {
  const lines = Array.from($('logbox').children).map(el => el.textContent);
  navigator.clipboard.writeText(lines.join('\n')).catch(() => {
    const ta = document.createElement('textarea');
    ta.value = lines.join('\n');
    document.body.appendChild(ta); ta.select();
    document.execCommand('copy'); document.body.removeChild(ta);
  });
}
```

### Browser Cache Note
After deploying the fix, browsers may serve the old broken JS from cache.
**Always hard-refresh with `Ctrl+Shift+R`** after redeploying dashboard changes.

---

## INA219 Note

The INA219 only measures current on the logic/supply rail, **not motor current**. This caused
misleading readings during diagnosis — current didn't increase when motors were supposedly running
because the STBY pins were keeping the H-bridge outputs disabled. Motor current would need a
shunt in the motor power line to be visible to the INA219, which is not currently wired.

---

## Navigation Stack Fixes (this session)

### Nav Node — Stale Sensor Data Bug
**Problem:** LIDAR sends one scan before crashing → `latest_scan` is not None → nav node
continuously commands forward at 90% forever.

**Fix:** Added `latest_scan_time` / `latest_depth_time` timestamps. In `navigation_loop`, data
older than 2 s is treated as None → robot stops if no fresh sensor data.

### Bridge — cmd_vel Watchdog
**Problem:** `poll_encoders` at 20 Hz sends `get_encoders` to Pico every 50 ms, which resets
Pico's `last_cmd_time`. So the Pico's internal safety timer never fires even when nav node stops
publishing cmd_vel — motors run indefinitely.

**Fix:** Added `check_cmd_vel_watchdog()` timer (1 Hz). If `last_cmd_vel_time` is set and no
cmd_vel has arrived for 5 s, bridge sends `{"cmd": "stop"}` to Pico directly.

---

## Git

Changes made in WSL `dev_ws` (not yet synced to Pi `lab_ws` git repo).
Files changed:
- `src/robot/upython/main.py` — STBY pins + directional encoder counting
- `src/robot/scripts/robot_dashboard.py` — JS syntax fix
- `src/robot/scripts/autonomous_navigation_node.py` — sensor freshness timeout
- `src/robot/scripts/serial_motor_bridge.py` — cmd_vel watchdog

# 2026-03-21 — Serial Bridge Race Condition Fix

## Problem

`serial_motor_bridge.py` had two bugs that prevented the robot from initializing:

1. **Race condition in `read_serial_loop`**: background thread read serial data without holding `serial_lock`, consuming responses that `send_command` was waiting for.
2. **Wrong initial ping**: `connect_serial()` used `{"cmd": "status"}` for the connection check. The status response has `"type": "status"` which was in the skip list, causing a timeout.
3. **Incomplete skip list in `send_command`**: periodic push messages from the Pico (`type: "status"`, `type: "heartbeat"`, etc.) were not all skipped, causing `send_command` to return a push message instead of the command response.

## Fixes Applied

**File:** `lab_ws/src/labrobot/scripts/serial_motor_bridge.py`

### 1. Lock in `read_serial_loop`
Added `self.serial_lock.acquire(blocking=False)` around serial reads in the background thread. If the lock is held by `send_command`, the background thread sleeps 10ms and retries. This prevents the background thread from consuming responses mid-command.

```python
if not self.serial_lock.acquire(blocking=False):
    time.sleep(0.01)
    continue
try:
    # ... read and process serial data ...
finally:
    self.serial_lock.release()
```

### 2. Use `ping` instead of `status` for initial check
`connect_serial()` now sends `{"cmd": "ping"}` and expects `{"status": "ok"}` (pong). Ping responses have no `"type"` field so they aren't skipped.

### 3. Skip all push messages in `send_command`
Changed the filter from checking specific types to skipping **any message with a `"type"` field**:
```python
if "type" in response:
    continue  # skip push messages; wait for command response
```
All Pico push messages (heartbeat, status, battery, log) have `"type"`. Command responses don't.

## Result

- `Emergency stop reset successfully` logged on launch ✓
- `Autonomous mode ENABLED` logged on launch ✓
- `MicroPython controller initialized successfully` logged on launch ✓
- Dashboard accessible at `http://192.168.25.135:8080` ✓
- Encoder data published on `/motor_controller/encoder_data` ✓

---

## Verified Working (confirmed by `ros2 topic echo`)

```
data: '{"rotation_rad": 0.0, ..., "left_count": 0, "right_count": 0, ...}'
```

Counts are 0 because encoder counting has a separate issue (see encoder notes below).

---

## Encoder IRQ Issue (unresolved as of 2026-03-21)

### What works
- Motor A (GP2/3/4) and Motor B (GP10/11/12) both spin and generate encoder signals (confirmed by direct GPIO polling via mpremote scripts)
- Encoder channels GP16/17 (left) and GP18/19 (right) confirmed by `pin_discover.py` hand-spin test
- GP16 and GP17 change independently (~4000 edges each per 2s at ~70% speed) — proper quadrature relationship

### What doesn't work
- IRQ-based quadrature counting in `main.py` returns 0 — soft IRQ delay exceeds the ~250μs quadrature phase offset
- By the time the IRQ handler executes, both A and B channels have advanced → handler reads `a == b` → net +0 count

### Recommended fix
Replace the quadrature ISRs with simple edge counting on the A channel only, with motor direction tracked separately:

```python
def _update_left(_pin):
    enc.left_count += nav.left_dir  # +1 forward, -1 backward

def _update_right(_pin):
    enc.right_count += nav.right_dir
```

Set `nav.left_dir` and `nav.right_dir` in `_move()` and `_rotate()` based on which direction functions are passed.

### MicroPython update
Pico W was on v1.25.0 (2025-04-15). Update to v1.27.0 (2025-12-09):
- Download: `https://micropython.org/resources/firmware/RPI_PICO_W-20251209-v1.27.0.uf2`
- Flash via BOOTSEL button + drag-and-drop
- Newer versions may improve soft IRQ delivery timing

---

## Hardware Issue Found (2026-03-21 — hardware check needed)

During testing, the encoder signals on GP16/17/18/19 stopped being generated during motor drive. The GPIO pins became stuck HIGH (encoder output floating), while direction pins (GP3=1, GP4=0) are correct and PWM is active on GP2.

**Possible causes:**
- Encoder Vcc (5V supply to the encoder board) disconnected
- Encoder signal wires loose at terminal block
- Motor itself not spinning (H-bridge power wire loose)

**Diagnostic:** Run `test_all.py` via mpremote — if encoder edges detected, encoders are wired OK. If 0, check encoder Vcc wiring.

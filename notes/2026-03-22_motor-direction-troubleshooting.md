# Motor Direction Troubleshooting — 2026-03-22

## Symptoms

After Nav2/SLAM bringup, robot movement was wrong in 2 of 4 directions:
- FWD ✓, LEFT turn ✓
- BACK and RIGHT turn: only one wheel moved (right motor not engaging)

Initial guess was a wiring fault, but this was wrong.

---

## Diagnostic Method

Direct serial test (`python3 motor_test.py` via SSH) sending Pico commands and reading
encoder counts per-wheel, bypassing ROS entirely.

**Critical lesson:** used LIDAR + `/odom` for LIDAR-guided clearance checks and direction
verification. Raw encoder counts have sign ambiguity; `/odom` displacement is unambiguous.

---

## Root Cause 1 — Left Motor Backward Stiction (Pico firmware)

**Finding:** Left motor encoder shows ~0 counts when commanded backward from cold start,
but ~1000+ counts going forward. Direct serial test at all speeds (85–100%) confirmed.

**Proof:** "Warm start" test — running forward 0.5s then switching to backward mid-ramp
gave L=–888 counts. Cold start from rest always gave ~0.

**Cause:** Left motor has high static friction (stiction) in the backward direction from
rest. The Pico ramp starts from 0% PWM and climbs slowly (ramp_step=3000 = ~1s to full
speed), never reaching enough torque before the motor gives up.

**Fix (upython/main.py):**

1. Anti-stiction pulse: when `current_speed == 0` and left motor is going backward
   (`left_fn is _lb`), fire a 30ms forward pulse at 95% PWM to break the brush stiction,
   then ramp normally. Encoder direction saved/restored around the pulse.

2. Faster ramp: `ramp_step` increased from 3000 → 10000 (~250ms ramp vs old ~1s).
   Ensures the motor reaches effective PWM quickly.

```python
_ANTISTICTION_PWM = int(95 * 65535 // 100)

if nav.current_speed == 0 and new_speed > 0 and left_fn is _lb:
    saved_left_dir = nav.left_dir
    nav.left_dir = 1
    _set_dir(_lf, _rs)
    ena_pwm.duty_u16(_ANTISTICTION_PWM)
    enb_pwm.duty_u16(0)
    sleep_ms(30)
    nav.left_dir = 0
    _ls()
    ena_pwm.duty_u16(0)
    sleep_ms(15)
    nav.left_dir = saved_left_dir
    nav.current_speed = 0
```

---

## Root Cause 2 — Right Motor Physically Inverted (bridge + odom)

**Finding:** `rotate right` (_lf + _rb) = both wheels physical-forward = FORWARD.
For this to give linear motion, right motor firmware-backward must = physical-forward.
Right motor is physically mounted inverted (one motor faces opposite direction).

**Impact on handle_twist:** The Pico's `move` and `rotate` commands were mapped to the
wrong ROS axes. Fixed in a previous commit by swapping the commands:
- `linear_x` → uses `rotate` Pico command (direction: right=fwd, left=bwd)
- `angular_z` → uses `move` Pico command (direction: forward=CCW, backward=CW)

**Impact on odometry:** The bridge odom formula assumed standard convention:
```python
delta_right = (right_count - last) * m_per_tick   # WRONG
angular = (delta_right - delta_left) / wheelbase  # WRONG
```
With right physically inverted, right_count decreases when moving forward. The formula
interpreted this as the right wheel going backward = rotation rather than translation.

**Fix (serial_motor_bridge.py):**
```python
delta_right = -(right_count - last_right_count) * m_per_tick   # negate: inverted motor
angular = (delta_left - delta_right) / wheel_base_m             # flipped vs standard
```

**Verification:** `/odom` displacement confirmed:
- FWD: dx > 0 ✓
- BACK: dx < 0 ✓
- LEFT: dheading > 0 (CCW) ✓
- RIGHT: dheading < 0 (CW) ✓

---

## Follow-up Session (same date) — Odom Verification & Stiction Revisit

### Odom Formula Confirmed Correct

**Finding:** The odom formula (`delta_right = -raw_right_delta`, `angular = (dL - dR)/wb`) IS correct.
Confirmed by `ros2 topic pub /cmd_vel "{linear: {x: 0.15}}" --once` → odom x jumped to +0.359m (forward ✓).

**Why direction_test showed dx=-0.070 for FWD:** The robot had been rotated by prior tests and was
facing ~144° heading (not +x). The odom reported -x movement because the robot physically went
forward in its own frame but the odom frame was rotated. The formula itself is fine.

**Lesson:** `direction_test.py` is only valid when robot faces near +x (odom heading ≈ 0).
Use clearance data from LIDAR to infer orientation before trusting direction test results.

### Left Motor Backward Stiction — Anti-Stiction Pulse Insufficient

**Finding (raw serial test):** For `rotate left` (= robot backward):
- dL = +22 (only the anti-stiction forward pulse contributed)
- dR = +2212 (right motor ran fine)
- Left motor did NOT run backward during the ramp phase

**Root cause:** The anti-stiction pulse frees the brushes momentarily, but the ramp then starts
from 0 PWM (u16 ramp_step = 10000 → first step = 15.3% PWM). After the 15ms brake, the motor
is cold again and 15% is insufficient to start it. The stiction requires a kick to overcome, not
a slow ramp from 0.

**Effect:** When the bridge sends `rotate left` (BACK), only the right wheel drives → robot spins
instead of translating backward.

**Software attempts (all tested, none solved the core problem):**
- Starting ramp from 40%, 70%, or target PWM: left motor still barely moves backward
- 150ms and 300ms forward pulse + no brake: both give exactly dL=-186 (stall-limited)
- Conclusion: motor stalls at a fixed shaft position after ~186 backward encoder counts
  regardless of pulse duration. This is a commutator dead-spot / brush wear issue.

**Hardware root cause:** Worn brush contact at a specific commutator position causes the
left motor to reliably stall ~11mm into backward motion. Not fixable in software.

**Current firmware state (`upython/main.py`):**
- 150ms forward pulse at 95% PWM, no brake, jump to target speed
- This is the best available software mitigation; gives ~10% of expected backward speed

**Practical impact:**
- BACK translation: robot drifts CW (right motor drives, left motor stalls)
- RIGHT turn (move-backward): same left motor problem, turn is slow and inaccurate
- FWD and LEFT turn: unaffected ✓

**Mitigation for Nav2:** Configure forward-only navigation — robot turns around rather
than reversing. This avoids the problematic motion entirely.

### Safety Incident — Raw cmd_vel Without LIDAR

**Incident:** Used `ros2 topic pub /cmd_vel ... --once` without LIDAR clearance check.
The robot ran forward for ~5 seconds (watchdog timeout) and bumped into an object.

**Rule:** NEVER publish raw cmd_vel. Always use LIDAR-gated scripts:
- `direction_test.py` (LIDAR clearance + odom verification)
- `smart_motor_test.py` (LIDAR clearance + stall detection + auto-reposition)

---

## Debugging Tools Used

- `python3 /tmp/motor_test.py` — direct serial, 4-direction encoder count test
- `python3 /tmp/smart_motor_test.py` — ROS node with LIDAR clearance checking
- `python3 /tmp/direction_test.py` — odom-based direction verification
- `mpremote connect /dev/ttyACM0 cp main.py :main.py` — flash Pico firmware
- `mpremote exec 'import machine; machine.reset()'` — hard reset Pico

---

## Files Modified

| File | Change |
|------|--------|
| `upython/main.py` | Anti-stiction pulse + ramp_step 3000→10000 |
| `scripts/serial_motor_bridge.py` | Negate right delta, fix angular formula |

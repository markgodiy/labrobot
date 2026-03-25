# Motor Direction Mapping — Verified 2026-03-25

## Pin Assignments (Pico W → H-Bridge)

| Motor | Enable (PWM) | Dir 1 | Dir 2 |
|-------|-------------|-------|-------|
| A (Left) | ENA = GP3 | IN1 = GP4 | IN2 = GP5 |
| B (Right) | ENB = GP13 | IN3 = GP11 | IN4 = GP12 |

H-Bridge STBY: tied HIGH on PCB (no GPIO control).

## Pico Firmware Direction Functions (main.py)

```
_lf(): IN1=HIGH, IN2=LOW   → left motor "forward"
_lb(): IN1=LOW,  IN2=HIGH  → left motor "backward"
_rf(): IN3=HIGH, IN4=LOW   → right motor "forward"
_rb(): IN3=LOW,  IN4=HIGH  → right motor "backward"
```

## set_speed Command

The bridge sends `{"cmd": "set_speed", "left_dir": ±1, "right_dir": ±1, "speed": 0-100}`.

On the Pico:
- `left_dir=+1` → `_lf()` → IN1=HIGH, IN2=LOW
- `left_dir=-1` → `_lb()` → IN1=LOW, IN2=HIGH
- `right_dir=+1` → `_rf()` → IN3=HIGH, IN4=LOW
- `right_dir=-1` → `_rb()` → IN3=LOW, IN4=HIGH

## Encoder Verification

Tested by sending set_speed commands and measuring encoder count deltas:

| left_dir | right_dir | Left encoder delta | Right encoder delta | Physical motion |
|----------|-----------|-------------------|--------------------|----|
| +1 | +1 | increases | increases | **Forward** |
| -1 | -1 | decreases | decreases | **Backward** |
| -1 | +1 | decreases | increases | **CCW (left turn)** |
| +1 | -1 | increases | decreases | **CW (right turn)** |

## Bridge handle_twist() Mapping (serial_motor_bridge.py)

```python
# Forward/backward (linear_x from /cmd_vel)
left_dir, right_dir = (1, 1) if linear_x > 0 else (-1, -1)

# Rotation (angular_z from /cmd_vel)
# angular_z > 0 = CCW (left turn); < 0 = CW (right turn)
left_dir, right_dir = (-1, 1) if angular_z > 0 else (1, -1)
```

## Dashboard Arrow Key Chain

```
ArrowUp   → 'fwd'   → linear_x=+speed  → (1,1)   → forward
ArrowDown → 'back'  → linear_x=-speed  → (-1,-1) → backward
ArrowLeft → 'left'  → angular_z=+speed → (-1,1)  → CCW (left turn)
ArrowRight→ 'right' → angular_z=-speed → (1,-1)  → CW (right turn)
```

## History

- **Original mapping** (pre H-bridge fix): empirical values `(1,-1)` for forward, `(1,1)` for CCW.
  These were wrong after correcting the GPIO pin assignments.
- **First correction attempt**: swapped linear and angular mappings — still wrong
  (verified by encoder test: CCW produced left fwd + right back = CW motion).
- **Final mapping**: derived from encoder delta signs. Verified correct on 2026-03-25.

# 2026-03-21 — Robot Dashboard

## Summary

Live web metrics dashboard (`scripts/robot_dashboard.py`) accessible at `http://192.168.25.135:8080`.
Also serves a mobile remote control page at `http://192.168.25.135:8080/remote`.

---

## Architecture

Single Python script — ROS 2 node + stdlib `HTTPServer` (no Flask). The HTTP server runs in a daemon thread; ROS callbacks update shared dicts under `_lock`.

**Subscribes to:**

| Topic | Type | Data used |
| --- | --- | --- |
| `/motor_controller/status` | std_msgs/String | battery, emergency_stop, pico metrics |
| `/motor_controller/encoder_data` | std_msgs/String | left_count, right_count |
| `/odom` | nav_msgs/Odometry | x, y, heading, vx, wz |
| `/rosout` | rcl_interfaces/Log | all node logs → ring buffer |

**Publishes:**

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | teleop from dashboard/remote |

**System metrics** (polled every 5 s via `/proc` and `/sys`): CPU temp, memory used/total, uptime.

---

## HTTP Endpoints

| Method | Path | Description |
| --- | --- | --- |
| GET | `/` | Dashboard HTML |
| GET | `/metrics` | JSON snapshot of all metrics |
| GET | `/remote` | Mobile remote control page |
| GET | `/logs?since=N` | Log ring-buffer (incremental, returns new entries since count N) |
| GET | `/logs/download` | Full log buffer as `robot_log.txt` download |
| POST | `/cmd` | Publish Twist to `/cmd_vel` — body: `{"linear": 0.3, "angular": 0.0}` |

---

## Dashboard Cards

| Card | Metrics |
| --- | --- |
| Odometry | x, y, heading, linear vel, angular vel |
| Encoders | left/right tick counts, bridge connected, e-stop state |
| Battery (INA219) | voltage (color-coded), current mA, power mW, charge % |
| Pi System | CPU temp, memory used/total, uptime |
| Pico W (MC) | CPU temp, LED state, uptime, command count, error count, firmware version |
| Control | Speed slider, D-pad buttons, link to /remote |
| Log Stream | Scrollable terminal, level filter, Pause/Clear/Download |

**Stale fading:** values fade to 35% opacity when not updated — 3 s for odom/encoders, 20 s for battery/pico.

---

## Pico W Metrics

Pico firmware sends `cpu_temp_c` and `led_on` in every status/heartbeat message.
`led_on = True` means e-stop is cleared (motors allowed). LED is OFF when e-stop is active.

`_status_dict()` and `_heartbeat_dict()` in `main.py` both include:

```python
"cpu_temp_c": _cpu_temp_c(),   # RP2040 internal ADC(4)
"led_on":     bool(led.value()),
```

---

## Robot Control

**Dashboard D-pad** (Control card):

- Hold button → sends Twist at 10 Hz; release → sends zero
- Speed slider sets linear speed (0.05–0.5 m/s); angular = `max(0.5, speed × 2.5)` rad/s
- Keyboard: arrow keys to drive, Space to stop

**Mobile remote (`/remote`):**

- Full-screen touch UI, portrait-optimised, no scroll
- Large D-pad + speed slider + big E-STOP button
- Live velocity readout
- Also works with keyboard on desktop

---

## Log Stream

- Subscribes to `/rosout` → `deque(maxlen=500)` ring buffer with monotonic global counter
- `/logs?since=N` returns only entries added since count N (incremental — client tracks `total`)
- Level color-coding: DEBUG=muted, INFO=white, WARN=yellow, ERROR/FATAL=red
- Dropdown filter: All / INFO+ / WARN+ / ERROR+
- Pause button freezes auto-scroll (new entries still added to DOM)
- Download serves full buffer as plain-text with timestamps

---

## Quiet Launch

All `pi.*.launch.py` files (except `sim.launch.py`) now use `output='log'` instead of `output='screen'`.
Node stdout/stderr goes to `~/.ros/log/` — terminal stays clean. `/rosout` publishing is unaffected so the dashboard log stream still captures everything.

---

## Bug Fixed During Development

Initial version had wrong battery field names. Pico firmware sends:

| Firmware field | Correct interpretation | Dashboard was looking for |
| --- | --- | --- |
| `current_draw_amp` | current in **Amps** | `current_mA` / `current` |
| `current_draw_watt` | power in **Watts** | `power_mW` / `power` |
| `battery` (dict key in status) | battery sub-dict | `batt` |

Fix: read `current_draw_amp` → multiply × 1000 → store as `current_mA`. Same for `power_mW`.

---

## Files Changed

| File | Change |
| --- | --- |
| `dev_ws/scripts/robot_dashboard.py` | Main dashboard — all features |
| `lab_ws/src/labrobot/scripts/robot_dashboard.py` | Deployed copy on Pi (symlinked via colcon install) |
| `lab_ws/src/labrobot/launch/pi.*.launch.py` | `output='screen'` → `output='log'` on all nodes |

---

## Launch (Manual — Dev Mode)

The systemd service was removed (see systemd note). Launch manually:

```bash
source ~/lab_ws/install/setup.bash
ros2 launch labrobot pi.autonomous.serial.launch.py
```

Terminal will only show WARN/ERROR — all logs visible in dashboard at `:8080`.

---

## Color Thresholds

| Metric | Green | Yellow | Red |
| --- | --- | --- | --- |
| Battery voltage | ≥ 12 V | ≥ 11 V | < 11 V |
| Pi CPU temp | < 60 °C | < 75 °C | ≥ 75 °C |
| Pico CPU temp | < 45 °C | < 60 °C | ≥ 60 °C |

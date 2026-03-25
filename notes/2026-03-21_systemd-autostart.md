# 2026-03-21 — Systemd Autostart (labrobot.service)

## Summary

Configured the full ROS 2 stack to auto-start on boot via a systemd system service.

---

## Files

| File | Purpose |
| ------ | ------- |
| `/etc/systemd/system/labrobot.service` | Systemd unit (system-level, survives reboot) |
| `~/lab_ws/launch_labrobot.sh` | Wrapper script that sources ROS 2 and runs the launch |

---

## Service File

```ini
[Unit]
Description=Labrobot ROS 2 Stack
After=multi-user.target

[Service]
Type=simple
User=botmanager
Environment=HOME=/home/botmanager
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStart=/home/botmanager/lab_ws/launch_labrobot.sh
Restart=on-failure
RestartSec=10
TimeoutStopSec=15
StandardOutput=journal
StandardError=journal
SyslogIdentifier=labrobot

[Install]
WantedBy=multi-user.target
```

**Why system service (not user service):** Linger was not enabled for `botmanager`, so a user service would only start on login. A system service with `User=botmanager` starts at boot without a login session.

**Why `After=multi-user.target`:** USB serial devices (Pico, LIDAR) are typically available by the time multi-user.target is reached. If a device isn't ready, `Restart=on-failure` + `RestartSec=10` retries every 10 seconds until it connects.

---

## Common Commands

```bash
# Start / stop / restart
sudo systemctl start labrobot
sudo systemctl stop labrobot
sudo systemctl restart labrobot

# Enable / disable autostart
sudo systemctl enable labrobot
sudo systemctl disable labrobot

# Check status
systemctl status labrobot

# Live log stream
journalctl -u labrobot -f

# Last 100 lines of logs
journalctl -u labrobot -n 100
```

---

## Gotcha — Stale Processes After Manual Testing

If you ran the bridge or dashboard manually before enabling the service, those processes hold `/dev/ttyACM0` and port 8080. The service's child processes will fail to bind.

**Fix:** Kill stray processes, then restart the service:
```bash
# Find stray PIDs
ps aux | grep -E 'serial_motor_bridge|robot_dashboard' | grep -v grep

# Kill them (replace PIDs)
kill <pid1> <pid2>

# Restart service
sudo systemctl restart labrobot
```

After this the service fully owns all resources and the conflict won't recur on future reboots.

---

## What Starts on Boot

Everything in `pi.autonomous.serial.launch.py`:
- `robot_state_publisher` — URDF / TF tree
- `joint_state_publisher`
- Static TF publishers (map→odom, oak→depth_camera)
- `sllidar_node` — RPLIDAR LIDAR
- `camera_node` (depthai) — OAK-D Lite
- `point_cloud_container` — XYZ + XYZRGB point clouds
- `rgb_republisher` / `depth_republisher`
- `serial_motor_bridge` — Pico serial bridge, odometry, TF
- `autonomous_navigation_node`
- `robot_dashboard` — web dashboard at :8080

# Dashboard Public Access & Security Hardening

**Date:** 2026-03-26

## Overview

Added security features to `robot_dashboard.py` so the dashboard can be safely exposed through a reverse proxy for remote friends to control the robot.

## How It Works

### Modes

- **LAN mode** (default): No auth, no sessions, everything works exactly as before. This is what you get when you just run the stack normally.
- **Public mode**: Set `DASHBOARD_PUBLIC=1` env var to enable. Requires invite codes, enforces single-user sessions with timeouts.

### Invite Code Flow

1. You open the **Admin page** at `http://<pi-ip>:8080/admin` (LAN-only, not accessible from the internet)
2. Enter a label (friend's name), click **Generate Invite**
3. Copy the invite link (e.g., `https://your-proxy.com/join/abc123xyz...`)
4. Send the link to your friend
5. Friend opens the link, enters their display name, clicks **Take Control**
6. The invite code is consumed (single-use), and a session starts
7. After the session timeout (default 5 min), they're kicked off with a cooldown (default 10 min) before they can go again

### Single-Session Model

- Only ONE person can control the robot at a time
- If someone is already driving, the join page shows "Someone is currently driving the robot" with a countdown timer
- After timeout, the user enters cooldown (tracked by IP) so others get a turn

## Env Vars

| Variable | Default | Description |
|----------|---------|-------------|
| `DASHBOARD_PUBLIC` | (unset) | Set to `1` to enable public mode |
| `DASHBOARD_SESSION_MINUTES` | `5` | Max session duration per user |
| `DASHBOARD_COOLDOWN_MINUTES` | `10` | Per-user cooldown after session ends |
| `DASHBOARD_CORS_ORIGIN` | `*` | CORS origin header (set to your proxy domain) |

### Example: Start Stack in Public Mode

```bash
# On the Pi:
nohup bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source ~/lab_ws/install/setup.bash && \
  export DASHBOARD_PUBLIC=1 && \
  export DASHBOARD_SESSION_MINUTES=5 && \
  export DASHBOARD_COOLDOWN_MINUTES=10 && \
  ros2 launch labrobot pi.autonomous.serial.launch.py' \
  > /tmp/stack.log 2>&1 &

# Tail the log:
tail -f /tmp/stack.log
```

Or for systemd (add these to the `[Service]` section alongside the existing ROS env vars):
```ini
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
Environment=DASHBOARD_PUBLIC=1
Environment=DASHBOARD_SESSION_MINUTES=5
Environment=DASHBOARD_COOLDOWN_MINUTES=10
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/botmanager/lab_ws/install/setup.bash && ros2 launch labrobot pi.autonomous.serial.launch.py'
```

## Pages & Routes

### Public-Facing (through proxy)

| Route | Description |
|-------|-------------|
| `GET /join/<code>` | Invite redemption page (enter name, take control) |
| `GET /`, `/remote`, `/follow`, `/mfollow` | Dashboard pages (require valid session in public mode) |
| `GET /metrics` | JSON metrics (requires session) |
| `POST /cmd` | Drive the robot (requires session, rate-limited 20/s) |
| `POST /nav`, `/estop`, `/follow_me` | Robot controls (require session) |
| `POST /join` | Redeem invite (form submit) |
| `POST /logout` | End session early |

### LAN-Only (admin)

| Route | Description |
|-------|-------------|
| `GET /admin` | Admin dashboard |
| `GET /admin/data` | JSON data for admin page polling |
| `POST /admin/invite` | Generate an invite code |
| `POST /admin/revoke_invite` | Revoke an unused invite |
| `POST /admin/kick` | Kick the current user |
| `POST /admin/clear_cooldown` | Clear cooldown for an IP |
| `POST /admin/settings` | Update session/cooldown timeouts on the fly |

### Blocked in Public Mode (403)

These dangerous endpoints are completely blocked when `DASHBOARD_PUBLIC=1`:

- `POST /system/shutdown`
- `POST /pico/release_port`
- `POST /pico/flash`
- `POST /pico/upload`
- `POST /bridge/restart`
- `GET /flashpico`

## Reverse Proxy Setup (Your NAS)

The dashboard itself handles auth, sessions, and endpoint blocking. Your NAS reverse proxy just needs to:

1. **Forward all traffic** to `http://192.168.25.135:8080`
2. **Set `X-Forwarded-For` header** so the dashboard sees real client IPs (needed for cooldown tracking)
3. **Terminate HTTPS** (Let's Encrypt, Cloudflare, etc.)

### Recommended proxy-level blocks (belt-and-suspenders)

Block these paths at the proxy as extra protection:
```
/admin
/flashpico
/system/shutdown
/pico/release_port
/pico/flash
/pico/upload
/bridge/restart
```

### Minimal nginx example

```nginx
server {
    listen 443 ssl;
    server_name robot.yourdomain.com;

    ssl_certificate     /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;

    # Block admin and dangerous endpoints
    location ~ ^/(admin|flashpico) {
        return 403;
    }
    location ~ ^/(system/shutdown|pico/(release_port|flash|upload)|bridge/restart) {
        return 403;
    }

    location / {
        proxy_pass http://192.168.25.135:8080;
        proxy_set_header Host $host;
        proxy_set_header X-Forwarded-For $remote_addr;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

## Security Summary

- **Invite codes**: Single-use, generated from LAN-only admin page
- **Single session**: Only one driver at a time
- **Timeout + cooldown**: Prevents one person from hogging the robot
- **Dangerous endpoints blocked**: Shutdown, firmware flash, bridge kill/restart all return 403 in public mode
- **Admin page LAN-only**: Only accessible from private network IPs (192.168.x.x, 10.x.x.x, 172.16.x.x, 127.x.x.x)
- **Rate limiting**: `/cmd` limited to 20 req/s per IP
- **Audit logging**: All POST requests logged to ROS logger with IP and user
- **CORS configurable**: Set `DASHBOARD_CORS_ORIGIN` to lock down cross-origin requests

## Audit Trail

All POST requests are logged to ROS with format:
```
[AUDIT] 203.0.113.42 POST /cmd Alex
[AUTH] Session started: Alex from 203.0.113.42
[ADMIN] Invite created for Alex: a1b2c3d4...
```

View in the admin page activity log or in `/tmp/stack.log`.

## Troubleshooting

### Changes not taking effect after scp / colcon build

The `--symlink-install` flag means the installed script is a symlink to the source file, so `scp` + `colcon build` updates the file on disk. However, the **running Python process** has the old code loaded in memory. You must **restart the stack** for changes to take effect.

### Stopping the stack — zombie processes

`pkill -9 -f "ros2 launch"` kills the launch process but child nodes (bridge, dashboard, camera, LIDAR, etc.) may survive as orphans. Always kill everything:

```bash
# Nuclear stop — kill launch + all child nodes:
nohup bash -c 'sleep 1 && \
  pkill -9 -f "ros2 launch"; \
  pkill -9 -f serial_motor_bridge; \
  pkill -9 -f robot_dashboard; \
  pkill -9 -f autonomous_navigation_node; \
  pkill -9 -f sllidar_node; \
  pkill -9 -f camera_node; \
  pkill -9 -f robot_state_publisher; \
  pkill -9 -f joint_state_publisher; \
  pkill -9 -f static_transform_publisher; \
  pkill -9 -f republish' &>/dev/null &

# Wait a few seconds, then verify nothing remains:
sleep 3
ps aux | grep -E 'ros2|robot_dashboard|serial_motor|sllidar|camera_node|state_publisher|republish|autonomous' | grep -v grep
```

If processes survive (especially `serial_motor_bridge` or `camera_node`), kill by PID:

```bash
kill -9 <pid1> <pid2> ...
```

### Duplicate stack — two sets of processes

If the stack was accidentally started twice, you'll see duplicate PIDs for every node and serial port conflicts on `/dev/ttyACM0`. Always stop before starting:

```bash
# Stop everything first
# (use the nuclear stop above)

# Verify clean state
ps aux | grep -E 'ros2|robot_dashboard' | grep -v grep
# Should show 0 results

# Then start
nohup bash -c '\
  source /opt/ros/jazzy/setup.bash && \
  source ~/lab_ws/install/setup.bash && \
  ros2 launch labrobot pi.autonomous.serial.launch.py' \
  > /tmp/stack.log 2>&1 &

tail -f /tmp/stack.log
```

### /admin returns 404

The admin page is part of the updated `robot_dashboard.py`. If you get 404:

1. Verify the file on disk has the admin code: `grep '/admin' ~/lab_ws/src/labrobot/scripts/robot_dashboard.py`
2. If the code is there, the running process has old code — restart the stack (see above)
3. If the code is missing, redeploy from WSL: `scp src/labrobot/scripts/robot_dashboard.py botmanager@192.168.25.135:~/lab_ws/src/labrobot/scripts/`

### /admin returns 403

The admin page is restricted to private network IPs (192.168.x.x, 10.x.x.x, 172.16-31.x.x, 127.x.x.x). If you're accessing from a public IP (through the reverse proxy), it will return 403 — this is by design. Access admin only from LAN.

# Nav2 + SLAM Implementation — 2026-03-22

## Goal

Upgrade the robot from reactive obstacle-avoidance to full map-based autonomous
navigation using Nav2 (Jazzy) + SLAM Toolbox, with LIDAR only (OAK-D Lite
connection TBD).

---

## Files Added / Modified

| File | Change |
|------|--------|
| `scripts/serial_motor_bridge.py` | Fix velocity scaling + odom covariance |
| `config/nav2_params.yaml` | New — full Nav2 params for this robot |
| `launch/pi.lab.mapping.launch.py` | New — SLAM mapping mode |
| `launch/pi.lab.nav.launch.py` | New — Nav2 operational mode |
| `maps/.gitkeep` | New — placeholder for map files |
| `package.xml` | Added nav2 + slam_toolbox exec deps |
| `CMakeLists.txt` | Install new launch files, config, maps/ |
| `.gitattributes` | New — eol=lf for scripts/launch |

---

## Key Fixes in serial_motor_bridge.py

### Velocity Scaling (`handle_twist`)

**Problem**: Nav2 DWB controller outputs `linear.x` up to 0.22 m/s.  The
bridge was computing `speed = int(abs(linear_x) * 100)` → 22% motor PWM.
The motors need ≥85% to move.  Result: Nav2 could never actually drive the robot.

**Fix**: Map Nav2 full-scale output (0.22 m/s / 1.0 rad/s) to the motor's
effective range (85–100%):
```python
MAX_LINEAR_MS  = 0.22
MOTOR_MIN, MOTOR_MAX = 85, 100
ratio = min(1.0, abs(linear_x) / MAX_LINEAR_MS)
speed = int(MOTOR_MIN + ratio * (MOTOR_MAX - MOTOR_MIN))
```

### Odometry Covariance

**Problem**: All covariance values were 0.0.  AMCL uses covariance for sensor
fusion weighting — zero covariance means infinite confidence in odometry,
which prevents AMCL from correcting drift.

**Fix**: Set diagonal entries:
```python
odom.pose.covariance[0]  = 0.01   # x
odom.pose.covariance[7]  = 0.01   # y
odom.pose.covariance[35] = 0.05   # yaw
odom.twist.covariance[0]  = 0.01
odom.twist.covariance[35] = 0.05
```

---

## Nav2 Configuration Choices (`config/nav2_params.yaml`)

- **Robot radius**: 0.22 m (≈ 0.44 m diagonal / 2)
- **Inflation radius**: 0.30 m (tight for lab corridors)
- **max_vel_x**: 0.22 m/s, **max_vel_theta**: 1.0 rad/s (conservative for Pi 4B)
- **xy_goal_tolerance**: 0.15 m, **yaw_goal_tolerance**: 0.25 rad
- **obstacle_min_range**: 0.30 m (matches CHASSIS_MIN chassis filter)
- **Controller**: DWB (DynamicWindowApproach-like, good for diff drive)
- **Planner**: NavFn (GridBased, proven for indoor environments)
- **Localiser**: AMCL with DifferentialMotionModel
- **No `collision_monitor`** remap in simple deploy (can add later)

---

## TF Architecture

```
map ──(AMCL, dynamic)──► odom ──(serial_motor_bridge TF broadcaster)──► base_footprint
                                                                               │
                                          robot.urdf.xacro (RSP) ─────────────┤
                                                                        base_link
                                                                        lidar_frame
```

**Critical**: The old `pi.basic.sensors.launch.py` publishes a static identity
`map→odom` TF.  The new lab launch files (`pi.lab.*.launch.py`) do NOT include
this — they let AMCL / SLAM Toolbox provide it dynamically.  Never include
both.

---

## Launch Workflows

### One-time Mapping

```bash
# 1. Install prerequisites (once)
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-nav2-map-server

# 2. Launch mapping mode
source ~/lab_ws/install/setup.bash
ros2 launch labrobot pi.lab.mapping.launch.py

# 3. Teleop robot around entire lab
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. Save map
ros2 run nav2_map_server map_saver_cli -f ~/lab_ws/src/labrobot/maps/lab_map

# 5. Record station poses — drive to each station and note /amcl_pose or /odom
ros2 topic echo /odom --once
# Add poses to config/lab_stations.yaml (future Phase 3)
```

### Normal Navigation Operation

```bash
source ~/lab_ws/install/setup.bash
ros2 launch labrobot pi.lab.nav.launch.py \
  map:=~/lab_ws/src/labrobot/maps/lab_map.yaml

# Verify AMCL converges (may need to manually set initial pose in RViz)
ros2 topic echo /amcl_pose --once

# Test navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Verify TF tree

```bash
ros2 run tf2_tools view_frames
# Expect: map → odom (from AMCL), odom → base_footprint (from bridge)
ros2 topic echo /scan --once | grep frame_id   # expect: lidar_frame
```

---

## Issues Encountered During Bringup

### 1. Package not found on first launch

**Error**: `Package 'labrobot' not found, searching: ['/opt/ros/jazzy']`

**Cause**: Workspace overlay not sourced / `colcon build` not run after adding new files.

**Fix**:

```bash
cd ~/lab_ws
colcon build --symlink-install --packages-select labrobot
source ~/lab_ws/install/setup.bash
```

### 2. Scripts not executable after `git pull`

**Error**: `executable 'serial_motor_bridge.py' not found on the libexec directory`

**Cause**: With `--symlink-install`, ROS 2 symlinks `lib/labrobot/*.py` directly to
`src/labrobot/scripts/*.py`. Git does not preserve the execute bit when cloning or
pulling, so the source files land as `100644` (not executable). The symlink inherits
the source permission, causing ROS 2's node finder to reject the file.

**Fix (one-time)**: Mark all scripts executable in the git index so the bit is stored
in git and applied on every future clone/pull:

```bash
git update-index --chmod=+x scripts/*.py launch/*.py scripts/*.sh
git commit -m "fix: mark all scripts and launch files executable in git index"
git push
```

On the Pi after any pull before this fix was in place:

```bash
chmod +x ~/lab_ws/src/labrobot/scripts/*.py ~/lab_ws/src/labrobot/launch/*.py
```

---

## Known Risks / Watch Points

1. **Motor asymmetry during backup**: Left motor backward is weaker than
   forward (~99 encoder edges/s vs ~1965 forward).  Nav2 `BackUp` behavior
   may pull right.  Tune `backup` behavior distance conservatively (0.15 m).

2. **AMCL startup**: AMCL needs a good initial pose estimate.  If the robot
   starts far from the last known pose, localisation may take time or fail.
   Use RViz "2D Pose Estimate" tool to give AMCL a hint, or enable
   `set_initial_pose: true` with known coordinates.

3. **encoder ticks_per_rev**: Currently set to 3436.  If odometry drifts
   badly, measure by pushing robot exactly 1 m and checking `/odom` x output.

4. **OAK-D Lite**: Not included in these launch files (connection TBD).
   FollowMe mode (Phase 5 in plan) will add it via `enable_camera:=true` arg.

---

## Future Steps (see plan)

- Phase 2: Zebra barcode scanner node
- Phase 3: Lab mission state machine + `lab_stations.yaml` poses
- Phase 4: Dashboard delivery dispatch + job status cards
- Phase 5: FollowMe mode with OAK-D Lite (once camera connection confirmed)

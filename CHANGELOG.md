# Changelog

All notable changes to labrobot (codename TR45H) are documented here, reconstructed from git
history. Format loosely follows [Keep a Changelog](https://keepachangelog.com/).

## [Unreleased]

Open items carried over from the last session (see notes/ and project memory):

- Pi undervoltage/throttle metrics not yet on the dashboard (`vcgencmd get_throttled`,
  `measure_clock`, `measure_volts`).
- Encoder counting uses simple A-channel edge counting (quadrature IRQ abandoned — soft-IRQ
  timing on the Pico W can't resolve the ~250us phase offset reliably).
- Pico W standalone AP + browser control page (no Pi/router dependency) was designed but never
  implemented.

## [0.1.0] - 2026-08-22

Baseline release — `package.xml` had carried the placeholder `0.0.0` since day one, so this tag
retroactively versions the project as of the last working state (`92da41f`, public access mode)
and turns on semver + tagging for future changes. No code changed as part of this bump beyond the
version field itself and this changelog.

## [2026-03-26] — Public access mode

- **feat:** Invite-based public access mode for the dashboard — single-active-session model with
  per-user timeouts, IP-based cooldown, and an admin panel (LAN-only) for generating invite links.
  Enabled via `DASHBOARD_PUBLIC=1`; LAN mode (no auth) remains the default.

## [2026-03-25] — Mobile control, LIDAR safety, motor direction fixes

- **feat:** Responsive mobile layout for the dashboard; rear LIDAR safety gate; arc motion
  (combined linear + angular) support.
- **refactor:** Merged `/remote` and `/mfollow` into a single mobile control page.
- **fix:** Corrected linear direction mapping — forward/backward were swapped after the H-bridge
  pin fix. Verified against encoder data; mapping recorded in
  `notes/motor_direction_mapping.md`.
- **feat:** Startup/shutdown operational tooling — "Shutdown Robot" button on the dashboard Pi
  card, "Restart Bridge" button on the flashpico Verify step, and a full startup guide
  (`notes/start_ros2_stack.md`) with clean stop/restart procedures.

## [2026-03-22 to 2026-03-24] — Motor wiring corrections, Nav2/SLAM, dashboard growth

- **feat:** Nav2 + SLAM Toolbox infrastructure for lab navigation (`pi.lab.mapping.launch.py`,
  `pi.lab.nav.launch.py`), with lifecycle manager wiring and bond-timeout tuning to get
  slam_toolbox activating reliably.
- **feat:** Rear sector clearance added to the dashboard and navigation node; LIDAR 180°-mount
  correction applied to left/right sector mapping.
- **feat:** `flashpico` dashboard page (Pico firmware flash/upload), `follow_me_node.py`
  (follow-me behavior), dashboard navigation between pages.
- **fix:** Motor pin assignments corrected; move/rotate Pico commands swapped to match physical
  wiring; right-motor-inverted odometry fixed.
- **fix:** Anti-stiction firmware added and iteratively tuned (left motor backward direction
  needed extra correction).
- **fix:** H-bridge STBY pin handling and encoder IRQ issues addressed; various dashboard JS and
  nav safety fixes bundled together.

## [2026-03-21] — Dashboard debut, real odometry, serial bridge race fix

- **feat:** `robot_dashboard.py` added — live web metrics UI on port 8080, with hover tooltips on
  metric labels and corrected battery field names/state-of-charge display.
- **fix:** Real odometry implemented in `serial_motor_bridge.py` (previously stubbed); fixed a
  `now` variable bug and added a `ros2_control.xacro` stub.
- **fix:** Serial bridge race condition — the background read loop could consume responses meant
  for `send_command`. Fixed with a non-blocking lock in the reader, broader type-field filtering
  (status/heartbeat/battery/log), and switching the initial connection check from `status` to
  `ping`. (See `notes/2026-03-21_serial-bridge-fix.md`.)

## [2025-08] — Sensor integration, launch file overhaul, serial autonomy

- **feat:** OAK-D Lite (RGB-D camera) integration — computer vision launch file, IMU data,
  TF tree connection fixes, following the depthai-ros driver reference.
- **feat:** Serial-based autonomous navigation system introduced.
- **refactor:** Pi launch files reorganized for full sensor integration; package structure
  cleaned up (`lib` directory, python3-executable invocation, CMakeLists.txt fixes) to satisfy
  ROS 2 packaging conventions.
- A long stretch of "repush last" commits in this window reflects iterative on-Pi debugging
  without internet access (workspace synced via `git fetch` + `git reset --hard`, per the
  project's Pi-as-primary-dev-machine workflow).

## [2025-07] — Initial commit and hardware bring-up

- **feat:** Initial ROS 2 Jazzy workspace scaffolding for labrobot.
- **feat:** RPLIDAR integration (multiple iterations).
- **feat:** Pico motor controller firmware (`upython/main.py`) — camera hookup, speed ramping,
  odometry and joint_state_publisher fixes.
- Project bring-up on real hardware: differential-drive base, LIDAR, Pico W motor controller.

---

*Note: `2025-07` through `2025-08` commit messages were terse ("+lidar", "repush last", etc.) from
early hardware bring-up on the Pi; entries above are grouped and described from the actual diffs
rather than the raw commit subjects.*

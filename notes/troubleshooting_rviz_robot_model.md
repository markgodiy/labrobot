# RViz Robot Model Troubleshooting Guide

This guide covers common issues when robot models don't appear correctly in RViz, along with systematic debugging steps and commands.

## Common Symptoms

- ✅ TF frames are visible in RViz, but robot body/meshes are not
- ✅ LIDAR/sensor data is working, but robot model is missing
- ✅ Topics are publishing, but RViz shows empty robot model
- ✅ "Robot model" display shows warnings or errors

## Systematic Debugging Approach

### 1. Verify Basic ROS2 Setup and Topics

**Check if your launch file is running:**
```bash
ros2 launch labrobot pi_launch.py
# or your specific launch file
```

**List all active topics:**
```bash
ros2 topic list
```

**Expected topics for basic robot:**
- `/robot_description` - URDF content
- `/joint_states` - Joint positions
- `/tf` and `/tf_static` - Transform tree

**Check if robot_description is being published:**
```bash
ros2 topic list | grep robot_description
ros2 topic info /robot_description
ros2 topic echo /robot_description --once
```

**What to look for:**
- Topic should exist and have at least 1 publisher
- Content should be valid XML/URDF (starts with `<?xml` and contains `<robot>`)
- Should not be empty or contain error messages

### 2. Validate TF Tree Structure

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# This creates frames.pdf - open it to see the transform tree
```

**List all TF frames:**
```bash
ros2 run tf2_ros tf2_echo base_link base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
```

**Expected TF structure for robot:**
```
map → odom → base_footprint → base_link → chassis
                           → left_wheel
                           → right_wheel
                           → caster_wheel_f
                           → caster_wheel_b
                           → lidar_frame (if LIDAR)
                           → camera_frame (if camera)
```

**What to look for:**
- Complete chain from `map` to all robot parts
- No missing transforms
- No circular dependencies
- Reasonable transform values (not NaN or infinite)

### 3. Validate URDF/Xacro Files

**Compile Xacro to URDF and check for errors:**
```bash
cd ~/lab_ws
source install/setup.bash
xacro src/labrobot/description/robot.urdf.xacro > /tmp/test_robot.urdf
```

**If errors occur, check for common issues:**
- Misspelled filenames in `<xacro:include>`
- Undefined variables/properties
- Missing macro definitions
- Syntax errors in XML

**Validate URDF syntax:**
```bash
check_urdf /tmp/test_robot.urdf
```

**What to look for:**
- "Successfully parsed urdf file" message
- No error messages about missing links/joints
- No warnings about missing inertial properties

### 4. Check Joint States

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

**Expected output:**
```yaml
header:
  stamp: {...}
  frame_id: ''
name: ['left_wheel_to_base_link', 'right_wheel_to_base_link']
position: [0.0, 0.0]  # or actual wheel positions
velocity: []
effort: []
```

**What to look for:**
- Joint names match those in URDF
- Position values are reasonable (not NaN)
- Topic is publishing regularly

### 5. RViz Configuration Debugging

**Check RViz robot model display:**
1. Open RViz
2. Add "RobotModel" display
3. Check for error messages in the display

**Common RViz robot model errors:**
- "No tf data" - TF tree incomplete
- "URDF file failed to parse" - Invalid URDF syntax
- "Link [name] has no geometry" - Missing visual elements

**RViz troubleshooting steps:**
1. Set "Fixed Frame" to `base_link` or `odom`
2. Check "Robot Description" topic is set to `/robot_description`
3. Expand RobotModel display and look for warnings on individual links
4. Try resetting the display (uncheck/check "RobotModel")

### 6. Launch File Debugging

**Compare working vs non-working launch files:**

**Basic launch (minimal):**
```python
# Should include:
- robot_state_publisher (with robot_description parameter)
- joint_state_publisher 
- static transform publishers for odom chain
```

**Full launch (with sensors):**
```python
# Additionally includes:
- Sensor nodes (LIDAR, camera)
- Additional TF publishers for sensors
- Bridge/republish nodes
```

**What to look for:**
- All required nodes are launching successfully
- No error messages during launch
- Parameters are passed correctly to nodes

### 7. Node and Parameter Inspection

**Check if robot_state_publisher is running:**
```bash
ros2 node list | grep robot_state_publisher
ros2 node info /robot_state_publisher
```

**Check robot_state_publisher parameters:**
```bash
ros2 param list /robot_state_publisher
ros2 param get /robot_state_publisher robot_description
```

**Check joint_state_publisher:**
```bash
ros2 node list | grep joint_state_publisher
ros2 topic info /joint_states
```

### 8. Common Issues and Solutions

#### Issue: Robot model not visible, only TF frames
**Cause:** Usually incomplete TF tree or missing sensor transforms
**Solution:** Use full launch file with all sensors, not basic launch

**Before (basic launch):**
```bash
ros2 launch labrobot pi_launch.py  # Only basic robot
```

**After (full launch):**
```bash
ros2 launch labrobot pi.lidar.depthsensor.launch.py  # Complete setup
```

#### Issue: "No transform from [frame] to [frame]" errors
**Cause:** Missing static transform publishers
**Solution:** Add required static transforms in launch file

```python
static_tf_pub = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'parent_frame', 'child_frame']
)
```

#### Issue: URDF parsing errors
**Cause:** Syntax errors, misspellings, or missing files
**Solution:** 
1. Check all `<xacro:include filename="...">` paths
2. Verify all property names are spelled correctly
3. Ensure all macro definitions exist

#### Issue: Joint state publisher warnings
**Cause:** Conflicting joint state publishers (GUI vs non-GUI)
**Solution:** Use only one joint state publisher per launch

```python
# For real robot (no GUI)
Node(package='joint_state_publisher', executable='joint_state_publisher')

# For simulation/testing (with GUI)
Node(package='joint_state_publisher', executable='joint_state_publisher_gui')
```

## Debugging Command Checklist

When robot model isn't showing in RViz, run these commands in order:

```bash
# 1. Check launch is working
ros2 node list

# 2. Check required topics exist
ros2 topic list | grep -E "(robot_description|joint_states|tf)"

# 3. Verify robot_description content
ros2 topic echo /robot_description --once | head -20

# 4. Check TF tree
ros2 run tf2_tools view_frames

# 5. Test URDF compilation
xacro src/labrobot/description/robot.urdf.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf

# 6. Monitor joint states
ros2 topic echo /joint_states --once

# 7. Check node parameters
ros2 param get /robot_state_publisher robot_description | head -10
```

## Success Indicators

When everything is working correctly:
- ✅ Robot model visible in RViz (orange chassis, blue wheels, etc.)
- ✅ All TF frames present and connected
- ✅ Sensor data displaying (LIDAR scans, camera images)
- ✅ No error messages in launch output
- ✅ `/robot_description` topic contains valid URDF
- ✅ All expected topics are publishing

## Launch File Comparison

**Use this hierarchy for testing:**

1. **Basic robot only:** `pi_launch.py`
   - Robot model + basic TF
   - Good for testing URDF issues

2. **Robot + LIDAR:** `pi.lidar.launch.py`
   - Adds LIDAR sensor and TF
   - Tests sensor integration

3. **Full setup:** `pi.lidar.depthsensor.launch.py`
   - All sensors + processing nodes
   - Complete production setup

Start with the basic launcher and work your way up if you encounter issues.

## Additional Resources

- [ROS2 URDF Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [TF2 Debugging](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html)
- [RViz User Guide](https://github.com/ros2/rviz/blob/jazzy/README.md)

---
*Last updated: August 2025*
*Based on ROS2 Jazzy + Gazebo Harmonic setup*

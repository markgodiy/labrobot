#!/usr/bin/env python3
"""
ROS 2 Autonomous Navigation Node for MicroPython Motor Controller
================================================================

This node processes LIDAR and depth camera data to make navigation decisions
and sends movement commands to the MicroPython motor controller via serial bridge.

Key Features:
- Obstacle avoidance using LIDAR data
- Depth camera integration for enhanced perception
- Serial communication via motor bridge
- Configurable navigation parameters
- Safety timeouts and emergency stop capabilities
- Real-time status monitoring

Subscribed Topics:
- /scan (sensor_msgs/LaserScan): LIDAR data
- /oak/rgb/image_raw (sensor_msgs/Image): RGB camera
- /oak/depth/image_raw (sensor_msgs/Image): Depth camera
- /oak/points (sensor_msgs/PointCloud2): Point cloud data
- /motor_controller/status (std_msgs/String): Motor controller status

Published Topics:
- /navigation/state (std_msgs/String): Navigation state
- /navigation/debug (std_msgs/String): Debug information

Services:
- /emergency_stop (std_srvs/Trigger): Emergency stop
- /set_autonomous_mode (std_srvs/SetBool): Enable/disable autonomous mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import numpy as np
from threading import Lock
import time
import cv2
from cv_bridge import CvBridge

# ROS 2 message types
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('min_obstacle_distance', 0.5)  # meters
        self.declare_parameter('lidar_min_range', 0.25)  # meters — ignore cage/chassis below this distance
        self.declare_parameter('max_speed', 70)  # percentage
        self.declare_parameter('default_speed', 90)  # percentage (90% minimum for effective movement)
        self.declare_parameter('rotation_speed', 90)  # percentage (90% minimum for effective turning)
        self.declare_parameter('scan_angle_range', 90)  # degrees (±45° from front)
        self.declare_parameter('depth_obstacle_threshold', 1000)  # mm
        self.declare_parameter('command_timeout', 2.0)  # seconds
        self.declare_parameter('autonomous_enabled', False)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.default_speed = self.get_parameter('default_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.scan_angle_range = self.get_parameter('scan_angle_range').value
        self.lidar_min_range = self.get_parameter('lidar_min_range').value
        self.depth_obstacle_threshold = self.get_parameter('depth_obstacle_threshold').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.autonomous_enabled = self.get_parameter('autonomous_enabled').value
        
        # State variables
        self.latest_scan = None
        self.latest_depth_image = None
        self.latest_rgb_image = None
        self.latest_controller_status = None
        self.latest_odometry = None  # Encoder-based odometry data
        self.navigation_active = False

        # Sensor freshness tracking — prevent stale data from driving robot indefinitely
        self.latest_scan_time = None
        self.latest_depth_time = None
        self.sensor_timeout = 2.0  # seconds before sensor data is considered stale
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        self.emergency_stop_active = False
        self.last_command_time = time.time()
        self.start_time = time.time()  # Track node startup time
        self.state_lock = Lock()
        
        # Navigation state
        self.current_action = "idle"
        self.obstacle_detected = False
        self.path_clear = True
        
        # Encoder-based movement tracking
        self.movement_start_position = None  # Starting position for current action
        self.expected_movement_distance = 0.0  # Expected distance for current movement
        self.movement_threshold = 0.05  # Minimum movement (meters) to consider progress
        self.movement_timeout = 5.0  # Time before considering robot stuck
        
        # Stuck detection and escape variables
        self.action_history = []  # Track recent actions to detect stuck patterns
        self.action_start_time = time.time()  # When current action started
        self.stuck_detection_time = 8.0  # seconds - how long before considering stuck
        self.escape_mode = False  # Flag for when in escape behavior
        self.escape_start_time = 0.0
        self.last_escape_direction = None  # Track last escape direction to alternate
        self.consecutive_turns = 0  # Count consecutive turn actions
        self.max_consecutive_turns = 6  # Max turns before forced escape
        
        # Post-escape forced forward mode
        self.post_escape_forward_time = None
        self.post_escape_forward_duration = 2.0  # Simple 2 second forward after escape
        
        # Motor bridge service clients
        self.motor_stop_client = self.create_client(Trigger, '/motor/stop')
        self.motor_emergency_stop_client = self.create_client(Trigger, '/motor/emergency_stop')
        self.motor_reset_emergency_stop_client = self.create_client(Trigger, '/motor/reset_emergency_stop')
        self.motor_autonomous_mode_client = self.create_client(SetBool, '/motor/set_autonomous_mode')
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.depth_subscriber = self.create_subscription(
            Image,
            '/oak/stereo/image_raw',  # OAK-D Lite publishes depth as stereo/image_raw
            self.depth_callback,
            sensor_qos
        )
        
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.rgb_callback,
            sensor_qos
        )
        
        # Subscribe to motor controller status
        self.controller_status_subscriber = self.create_subscription(
            String,
            '/motor_controller/status',
            self.controller_status_callback,
            10
        )
        
        # Publishers
        self.nav_state_publisher = self.create_publisher(String, '/navigation/state', 10)
        self.debug_publisher = self.create_publisher(String, '/navigation/debug', 10)
        
        # Publisher for sending commands to motor bridge via cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Services
        self.emergency_stop_service = self.create_service(
            Trigger,
            '/emergency_stop',
            self.emergency_stop_callback
        )
        
        self.autonomous_mode_service = self.create_service(
            SetBool,
            '/set_autonomous_mode',
            self.set_autonomous_mode_callback
        )
        
        self.reset_emergency_stop_service = self.create_service(
            Trigger,
            '/reset_emergency_stop',
            self.reset_emergency_stop_callback
        )
        
        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        self.get_logger().info(f"Autonomous Navigation Node started")
        self.get_logger().info(f"Serial port: {self.serial_port}")
        self.get_logger().info(f"Autonomous mode: {'ENABLED' if self.autonomous_enabled else 'DISABLED'}")
        
        # Wait for motor bridge to be available
        self.get_logger().info("Waiting for motor bridge services...")
        self.motor_stop_client.wait_for_service(timeout_sec=5.0)
        self.motor_emergency_stop_client.wait_for_service(timeout_sec=5.0)
        self.motor_reset_emergency_stop_client.wait_for_service(timeout_sec=5.0)
        self.motor_autonomous_mode_client.wait_for_service(timeout_sec=5.0)
        
        # Initialize controller
        self.initialize_controller()
    
    def initialize_controller(self):
        """Initialize communication with MicroPython controller"""
        try:
            # First, reset emergency stop in case it was set during startup
            self.get_logger().info("Resetting emergency stop on startup...")
            estop_reset_request = Trigger.Request()
            reset_future = self.motor_reset_emergency_stop_client.call_async(estop_reset_request)
            rclpy.spin_until_future_complete(self, reset_future, timeout_sec=2.0)
            
            if reset_future.result() is not None and reset_future.result().success:
                self.get_logger().info("Emergency stop reset successfully")
            else:
                self.get_logger().warn("Failed to reset emergency stop")
            
            # Wait a moment for the reset to take effect
            time.sleep(0.5)
            
            # Set autonomous mode via service call
            request = SetBool.Request()
            request.data = self.autonomous_enabled
            
            future = self.motor_autonomous_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info("MicroPython controller initialized successfully")
            else:
                self.get_logger().warn("Failed to initialize MicroPython controller")
        except Exception as e:
            self.get_logger().error(f"Controller initialization error: {e}")
    
    def send_movement_command(self, linear_x=0.0, angular_z=0.0):
        """Send movement command via cmd_vel topic"""
        try:
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.angular.z = angular_z
            self.cmd_vel_publisher.publish(twist_msg)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send movement command: {e}")
            return False
    
    def stop_movement(self):
        """Stop robot movement via cmd_vel (safe to call from timer callbacks)."""
        # Do NOT use spin_until_future_complete here — this is called from navigation_loop
        # which runs inside a timer callback while the executor is already spinning.
        # spin_until_future_complete on the same executor causes "Executor is already spinning".
        return self.send_movement_command(0.0, 0.0)
    
    def controller_status_callback(self, msg):
        """Process controller status updates"""
        try:
            with self.state_lock:
                self.latest_controller_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid controller status JSON")
    
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        with self.state_lock:
            self.latest_scan = msg
            self.latest_scan_time = time.time()

    def depth_callback(self, msg):
        """Process depth camera data"""
        with self.state_lock:
            self.latest_depth_image = msg
            self.latest_depth_time = time.time()
    
    def rgb_callback(self, msg):
        """Process RGB camera data"""
        with self.state_lock:
            self.latest_rgb_image = msg
    
    def analyze_lidar_obstacles(self, scan_msg):
        """Analyze LIDAR data for obstacles"""
        if not scan_msg:
            return True, 0.0, "no_data"
        
        # LIDAR is mounted 180° rotated on this robot.
        # Robot forward = LIDAR angle ±π = indices near 0 and n-1 (wraps around array boundary).
        # Robot left/right are also swapped vs standard LIDAR mounting.
        n = len(scan_msg.ranges)
        half_range = np.radians(self.scan_angle_range / 2)
        half_idx = int(half_range / scan_msg.angle_increment)

        # Front sector: wrap around index 0
        front_indices = list(range(n - half_idx, n)) + list(range(0, half_idx))
        front_ranges_raw = [scan_msg.ranges[i] for i in front_indices]
        valid_ranges = [r for r in front_ranges_raw if self.lidar_min_range < r < scan_msg.range_max]

        if not valid_ranges:
            return True, 0.0, "no_valid_data"

        min_distance = min(valid_ranges)
        avg_distance = np.mean(valid_ranges)

        # Check for obstacles
        obstacle_detected = min_distance < self.min_obstacle_distance

        # Determine best direction if obstacle detected.
        # With 180° mount: first quarter of scan = robot RIGHT, last quarter = robot LEFT.
        direction = "forward"
        if obstacle_detected:
            quarter_point = n // 4
            right_ranges = scan_msg.ranges[:quarter_point]   # LIDAR left = robot right
            left_ranges  = scan_msg.ranges[-quarter_point:]  # LIDAR right = robot left

            left_clear  = len([r for r in left_ranges  if r > self.min_obstacle_distance]) > len(left_ranges)  * 0.7
            right_clear = len([r for r in right_ranges if r > self.min_obstacle_distance]) > len(right_ranges) * 0.7

            if left_clear and not right_clear:
                direction = "left"
            elif right_clear and not left_clear:
                direction = "right"
            elif left_clear and right_clear:
                left_avg  = np.mean([r for r in left_ranges  if self.lidar_min_range < r < scan_msg.range_max])
                right_avg = np.mean([r for r in right_ranges if self.lidar_min_range < r < scan_msg.range_max])
                direction = "left" if left_avg > right_avg else "right"
            else:
                direction = "backward"
        
        return not obstacle_detected, min_distance, direction
    
    def analyze_depth_obstacles(self, depth_image_msg):
        """Analyze depth camera data for ground-level obstacles"""
        if not depth_image_msg:
            return True, float('inf'), "no_depth_data"
        
        try:
            # Convert ROS image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="passthrough")
            
            # Get image dimensions
            height, width = depth_image.shape
            
            # Define region of interest (bottom center of image for ground-level detection)
            roi_height = height // 3  # Bottom third of image
            roi_width = width // 2    # Center half of image
            roi_y_start = height - roi_height
            roi_x_start = width // 4
            roi_x_end = roi_x_start + roi_width
            
            # Extract ROI for analysis
            depth_roi = depth_image[roi_y_start:height, roi_x_start:roi_x_end]
            
            # Convert to meters (OAK-D typically gives depth in mm)
            if depth_image.dtype == np.uint16:
                depth_roi_meters = depth_roi.astype(np.float32) / 1000.0
            else:
                depth_roi_meters = depth_roi.astype(np.float32)
            
            # Remove invalid depth values (0 or too far)
            valid_depths = depth_roi_meters[(depth_roi_meters > 0.1) & (depth_roi_meters < 5.0)]
            
            if len(valid_depths) == 0:
                return True, float('inf'), "no_valid_depth"
            
            min_depth = np.min(valid_depths)
            avg_depth = np.mean(valid_depths)
            
            # Check for ground-level obstacles
            depth_threshold_meters = self.depth_obstacle_threshold / 1000.0  # Convert mm to meters
            ground_obstacle_detected = min_depth < depth_threshold_meters
            
            # Analyze left/right distribution for direction recommendation
            roi_center = roi_width // 2
            left_roi = depth_roi_meters[:, :roi_center]
            right_roi = depth_roi_meters[:, roi_center:]
            
            left_valid = left_roi[(left_roi > 0.1) & (left_roi < 5.0)]
            right_valid = right_roi[(right_roi > 0.1) & (right_roi < 5.0)]
            
            direction = "forward"
            if ground_obstacle_detected:
                left_min = np.min(left_valid) if len(left_valid) > 0 else float('inf')
                right_min = np.min(right_valid) if len(right_valid) > 0 else float('inf')
                
                if left_min > depth_threshold_meters and right_min <= depth_threshold_meters:
                    direction = "left"
                elif right_min > depth_threshold_meters and left_min <= depth_threshold_meters:
                    direction = "right"
                elif left_min > depth_threshold_meters and right_min > depth_threshold_meters:
                    direction = "left" if left_min > right_min else "right"
                else:
                    direction = "backward"
            
            return not ground_obstacle_detected, min_depth, direction
            
        except Exception as e:
            self.get_logger().warn(f"Depth analysis failed: {e}")
            return True, float('inf'), "depth_error"
    
    def navigation_loop(self):
        """Main navigation decision loop"""
        if not self.autonomous_enabled or self.emergency_stop_active:
            return

        with self.state_lock:
            now = time.time()
            # Discard sensor data older than sensor_timeout — prevents driving on stale LIDAR
            scan_data = self.latest_scan if (
                self.latest_scan_time and now - self.latest_scan_time < self.sensor_timeout
            ) else None
            depth_data = self.latest_depth_image if (
                self.latest_depth_time and now - self.latest_depth_time < self.sensor_timeout
            ) else None
        
        # Check for stuck behavior first
        is_stuck, stuck_reason = self.is_stuck()
        if is_stuck:
            self.execute_escape_behavior(stuck_reason)
            return
        
        # If in escape mode, continue escape sequence
        if self.escape_mode:
            self.execute_escape_behavior("continuing_escape")
            return
        
        # Check if we're in post-escape forced forward period
        if self.post_escape_forward_time is not None:
            elapsed_since_escape = time.time() - self.post_escape_forward_time
            if elapsed_since_escape < self.post_escape_forward_duration:
                # Force forward movement, ignore obstacle detection temporarily
                if self.current_action != "post_escape_forward":
                    actual_speed = max(90, self.default_speed)
                    linear_speed = actual_speed / 100.0
                    
                    if self.send_movement_command(linear_x=linear_speed):
                        self.update_action_history("post_escape_forward")
                        remaining_time = self.post_escape_forward_duration - elapsed_since_escape
                        self.get_logger().info(f"Post-escape forced forward: {remaining_time:.1f}s remaining at {actual_speed}% power")
                return
            else:
                # End post-escape forward period
                self.post_escape_forward_time = None
                self.get_logger().info("Post-escape forward period complete, resuming normal navigation")

        # If no sensor data, stop if not already idle
        if not scan_data and not depth_data:
            if self.current_action != "idle":
                self.stop_movement()
                self.update_action_history("idle")
            return

        # Analyze obstacles
        lidar_path_clear, min_lidar_distance, best_lidar_direction = self.analyze_lidar_obstacles(scan_data)
        depth_path_clear, min_depth_distance, best_depth_direction = self.analyze_depth_obstacles(depth_data)
        
        # Combine LIDAR and depth camera results
        path_clear = lidar_path_clear and depth_path_clear
        min_distance = min(min_lidar_distance, min_depth_distance)
        
        # Make navigation decision with improved logic
        if path_clear and min_distance > self.min_obstacle_distance:
            # Path is clear - move forward at 90% minimum
            if self.current_action not in ["forward"]:
                actual_speed = max(90, self.default_speed)
                linear_speed = actual_speed / 100.0
                
                if self.send_movement_command(linear_x=linear_speed):
                    self.update_action_history("forward")
                    distance_str = f"{min_distance:.2f}m" if min_distance != float('inf') else "open space"
                    self.get_logger().info(f"Moving forward at {actual_speed}% power (distance: {distance_str})")
        
        elif best_lidar_direction in ["left", "right"] or best_depth_direction in ["left", "right"]:
            # Obstacle detected - choose rotation direction intelligently
            preferred_direction = self.choose_turn_direction(best_lidar_direction, best_depth_direction)
            
            if preferred_direction and self.current_action != f"rotate_{preferred_direction}":
                # Use configured rotation speed (minimum 90% enforced)
                actual_rotation_speed = max(90, self.rotation_speed)
                angular_speed = actual_rotation_speed / 100.0
                
                if preferred_direction == "right":
                    angular_speed = -angular_speed  # Negative for right turn
                
                source = "LIDAR" if best_lidar_direction == preferred_direction else "depth"
                self.get_logger().info(f"{source} rotation: {preferred_direction}, {actual_rotation_speed}% power (twist: {angular_speed:.3f})")
                
                if self.send_movement_command(angular_z=angular_speed):
                    self.update_action_history(f"rotate_{preferred_direction}")
                    self.get_logger().info(f"Rotating {preferred_direction} to avoid obstacle (distance: {min_distance:.2f}m)")
        
        elif best_lidar_direction == "backward" or best_depth_direction == "backward":
            # No clear path - back up at configured speed (90% minimum)
            if self.current_action != "backward":
                # Use configured default speed for backing up (minimum 90% enforced)
                actual_speed = max(90, self.default_speed)
                linear_speed = -(actual_speed / 100.0)  # Negative for backward
                
                if self.send_movement_command(linear_x=linear_speed):
                    self.update_action_history("backward")
                    self.get_logger().info(f"Backing up at {actual_speed}% power (twist: {linear_speed:.3f}) - no clear path (distance: {min_distance:.2f}m)")
        
        else:
            # Stop and reassess
            if self.current_action != "idle":
                if self.stop_movement():
                    self.update_action_history("idle")
                    self.get_logger().info("Stopping - reassessing situation")
        
        # Update navigation state
        self.obstacle_detected = not path_clear
        self.path_clear = path_clear
        self.last_command_time = time.time()
    
    def publish_status(self):
        """Publish navigation status"""
        nav_state_msg = String()
        nav_state_msg.data = self.current_action
        self.nav_state_publisher.publish(nav_state_msg)
        
        # Publish debug information
        debug_msg = String()
        debug_data = {
            'autonomous_enabled': self.autonomous_enabled,
            'current_action': self.current_action,
            'obstacle_detected': self.obstacle_detected,
            'path_clear': self.path_clear,
            'emergency_stop': self.emergency_stop_active,
            'controller_connected': self.latest_controller_status is not None
        }
        debug_msg.data = json.dumps(debug_data)
        self.debug_publisher.publish(debug_msg)
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        self.emergency_stop_active = True
        self.autonomous_enabled = False

        # Send stop via cmd_vel (fire-and-forget, safe to call from callback context)
        self.send_movement_command(0.0, 0.0)

        # Also fire async estop to bridge — don't spin_until_future_complete (deadlock risk)
        try:
            estop_request = Trigger.Request()
            self.motor_emergency_stop_client.call_async(estop_request)
        except Exception as e:
            self.get_logger().error(f"Async estop send error: {e}")

        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        response.success = True
        response.message = "Emergency stop activated"
        return response
    
    def set_autonomous_mode_callback(self, request, response):
        """Service callback for setting autonomous mode"""
        self.autonomous_enabled = request.data
        self.get_logger().info(f"Autonomous mode {'ENABLED' if self.autonomous_enabled else 'DISABLED'}")

        # Fire async — don't spin_until_future_complete from within a callback (deadlock)
        try:
            mode_request = SetBool.Request()
            mode_request.data = self.autonomous_enabled
            self.motor_autonomous_mode_client.call_async(mode_request)
        except Exception as e:
            self.get_logger().error(f"Async autonomous mode send error: {e}")

        if not self.autonomous_enabled:
            self.send_movement_command(0.0, 0.0)
            self.update_action_history("idle")

        response.success = True
        response.message = f"Autonomous mode {'enabled' if self.autonomous_enabled else 'disabled'}"
        return response
    
    def reset_emergency_stop_callback(self, request, response):
        """Service callback for resetting emergency stop"""
        self.emergency_stop_active = False
        
        response.success = True
        response.message = "Emergency stop reset"
        self.get_logger().info("EMERGENCY STOP RESET")
        
        return response

    def choose_turn_direction(self, lidar_direction, depth_direction):
        """Simple, direct turn direction selection - no more overcomplicated logic"""
        # Available directions from sensors
        available_directions = []
        if lidar_direction in ["left", "right"]:
            available_directions.append(lidar_direction)
        if depth_direction in ["left", "right"] and depth_direction not in available_directions:
            available_directions.append(depth_direction)
        
        if not available_directions:
            return None
        
        # Rule 1: If we just did a left turn, do a right turn (and vice versa)
        if self.current_action == "rotate_left" or (len(self.action_history) > 0 and "left" in self.action_history[-1]['action']):
            if "right" in available_directions:
                self.get_logger().info("SIMPLE RULE: Last was left, choosing RIGHT")
                return "right"
        
        if self.current_action == "rotate_right" or (len(self.action_history) > 0 and "right" in self.action_history[-1]['action']):
            if "left" in available_directions:
                self.get_logger().info("SIMPLE RULE: Last was right, choosing LEFT")
                return "left"
        
        # Rule 2: Default to RIGHT (consistent, predictable)
        chosen = "right" if "right" in available_directions else "left"
        self.get_logger().info(f"SIMPLE RULE: Default to {chosen.upper()}")
        return chosen

    def update_action_history(self, new_action):
        """Update action history for stuck detection"""
        current_time = time.time()
        
        # If action changed, record the change
        if new_action != self.current_action:
            if hasattr(self, 'current_action') and self.current_action:
                self.action_history.append({
                    'action': self.current_action,
                    'duration': current_time - self.action_start_time,
                    'timestamp': current_time
                })
            
            # Keep only recent history (last 30 seconds)
            self.action_history = [h for h in self.action_history if current_time - h['timestamp'] < 30.0]
            
            # Reset consecutive turn counter if we did something other than turn
            if new_action not in ["rotate_left", "rotate_right", "navigate_left", "navigate_right"]:
                self.consecutive_turns = 0
            elif self.current_action not in ["rotate_left", "rotate_right", "navigate_left", "navigate_right"]:
                self.consecutive_turns = 1  # Starting a new turn sequence
            else:
                self.consecutive_turns += 1
            
            self.action_start_time = current_time
            self.current_action = new_action
    
    def is_stuck(self):
        """Simple stuck detection - if turning for more than 3 seconds, we're stuck"""
        current_time = time.time()
        action_duration = current_time - self.action_start_time
        
        # If we've been turning for more than 3 seconds, we're stuck
        if action_duration > 3.0 and self.current_action.startswith("rotate_"):
            self.get_logger().warning(f"STUCK: Been turning for {action_duration:.1f}s")
            return True, "stuck_turning"
        
        return False, "none"
    
    def execute_escape_behavior(self, stuck_reason):
        """Simple escape: back up, turn opposite direction, go forward"""
        current_time = time.time()
        
        if not self.escape_mode:
            self.escape_mode = True
            self.escape_start_time = current_time
            self.get_logger().info(f"ESCAPE: Starting escape sequence")
        
        escape_duration = current_time - self.escape_start_time
        
        if escape_duration < 1.5:
            # Phase 1: Back up quickly
            if self.current_action != "escape_backward":
                if self.send_movement_command(linear_x=-0.90):
                    self.current_action = "escape_backward"
                    self.get_logger().info("ESCAPE: Backing up")
                    
        elif escape_duration < 3.0:
            # Phase 2: Turn the opposite direction from what got us stuck
            if self.current_action != "escape_turn":
                # If we were stuck turning left, turn right (and vice versa)
                if "left" in str(self.current_action):
                    angular_speed = -0.90  # Turn right
                    turn_dir = "right"
                else:
                    angular_speed = 0.90   # Turn left  
                    turn_dir = "left"
                
                if self.send_movement_command(angular_z=angular_speed):
                    self.current_action = "escape_turn"
                    self.get_logger().info(f"ESCAPE: Turning {turn_dir}")
                    
        else:
            # Phase 3: End escape and go forward
            self.escape_mode = False
            self.action_history = []  # Clear history
            
            if self.current_action != "escape_forward":
                if self.send_movement_command(linear_x=0.90):
                    self.current_action = "escape_forward"
                    self.get_logger().info("ESCAPE: Complete, going forward")
                    
                    # Force forward for 2 seconds
                    self.post_escape_forward_time = time.time()
                    self.post_escape_forward_duration = 2.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AutonomousNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

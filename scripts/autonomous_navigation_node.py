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
        self.depth_obstacle_threshold = self.get_parameter('depth_obstacle_threshold').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.autonomous_enabled = self.get_parameter('autonomous_enabled').value
        
        # State variables
        self.latest_scan = None
        self.latest_depth_image = None
        self.latest_rgb_image = None
        self.latest_controller_status = None
        self.navigation_active = False
        
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
        self.post_escape_forward_duration = 4.0  # Force forward for 4 seconds after escape
        
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
        """Stop robot movement"""
        try:
            request = Trigger.Request()
            future = self.motor_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                return future.result().success
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to stop movement: {e}")
            return False
    
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
    
    def depth_callback(self, msg):
        """Process depth camera data"""
        with self.state_lock:
            self.latest_depth_image = msg
    
    def rgb_callback(self, msg):
        """Process RGB camera data"""
        with self.state_lock:
            self.latest_rgb_image = msg
    
    def analyze_lidar_obstacles(self, scan_msg):
        """Analyze LIDAR data for obstacles"""
        if not scan_msg:
            return True, 0.0, "no_data"
        
        # Convert angle range to indices
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        half_range = np.radians(self.scan_angle_range / 2)
        
        # Find indices for front-facing range
        center_index = len(scan_msg.ranges) // 2
        range_indices = int(half_range / angle_increment)
        start_idx = max(0, center_index - range_indices)
        end_idx = min(len(scan_msg.ranges), center_index + range_indices)
        
        # Get distances in front range
        front_ranges = scan_msg.ranges[start_idx:end_idx]
        valid_ranges = [r for r in front_ranges if scan_msg.range_min < r < scan_msg.range_max]
        
        if not valid_ranges:
            return True, 0.0, "no_valid_data"
        
        min_distance = min(valid_ranges)
        avg_distance = np.mean(valid_ranges)
        
        # Check for obstacles
        obstacle_detected = min_distance < self.min_obstacle_distance
        
        # Determine best direction if obstacle detected
        direction = "forward"
        if obstacle_detected:
            # Check left and right sides
            quarter_point = len(scan_msg.ranges) // 4
            left_ranges = scan_msg.ranges[:quarter_point]
            right_ranges = scan_msg.ranges[-quarter_point:]
            
            left_clear = len([r for r in left_ranges if r > self.min_obstacle_distance]) > len(left_ranges) * 0.7
            right_clear = len([r for r in right_ranges if r > self.min_obstacle_distance]) > len(right_ranges) * 0.7
            
            if left_clear and not right_clear:
                direction = "left"
            elif right_clear and not left_clear:
                direction = "right"
            elif left_clear and right_clear:
                # Choose based on average distance
                left_avg = np.mean([r for r in left_ranges if scan_msg.range_min < r < scan_msg.range_max])
                right_avg = np.mean([r for r in right_ranges if scan_msg.range_min < r < scan_msg.range_max])
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
            scan_data = self.latest_scan
            depth_data = self.latest_depth_image
        
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
        
        # Send emergency stop via motor bridge
        try:
            estop_request = Trigger.Request()
            future = self.motor_emergency_stop_client.call_async(estop_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None and future.result().success:
                response.success = True
                response.message = "Emergency stop activated"
                self.get_logger().warn("EMERGENCY STOP ACTIVATED")
            else:
                response.success = False
                response.message = "Failed to activate emergency stop"
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop error: {e}"
        
        return response
    
    def set_autonomous_mode_callback(self, request, response):
        """Service callback for setting autonomous mode"""
        self.autonomous_enabled = request.data
        
        # Send mode change via motor bridge
        try:
            mode_request = SetBool.Request()
            mode_request.data = self.autonomous_enabled
            future = self.motor_autonomous_mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None and future.result().success:
                response.success = True
                response.message = f"Autonomous mode {'enabled' if self.autonomous_enabled else 'disabled'}"
                self.get_logger().info(f"Autonomous mode {'ENABLED' if self.autonomous_enabled else 'DISABLED'}")
                
                if not self.autonomous_enabled:
                    # Stop current movement
                    self.stop_movement()
                    self.update_action_history("idle")
            else:
                response.success = False
                response.message = "Failed to set autonomous mode"
        except Exception as e:
            response.success = False
            response.message = f"Autonomous mode error: {e}"
        
        return response
    
    def reset_emergency_stop_callback(self, request, response):
        """Service callback for resetting emergency stop"""
        self.emergency_stop_active = False
        
        response.success = True
        response.message = "Emergency stop reset"
        self.get_logger().info("EMERGENCY STOP RESET")
        
        return response

    def choose_turn_direction(self, lidar_direction, depth_direction):
        """Choose turn direction intelligently, considering previous stuck patterns"""
        # Get available directions from sensors
        available_directions = []
        if lidar_direction in ["left", "right"]:
            available_directions.append(lidar_direction)
        if depth_direction in ["left", "right"] and depth_direction not in available_directions:
            available_directions.append(depth_direction)
        
        if not available_directions:
            return None
        
        # If only one direction available, use it
        if len(available_directions) == 1:
            chosen_direction = available_directions[0]
            self.get_logger().info(f"Only {chosen_direction} available from sensors")
            return chosen_direction
        
        # ANTI-STUCK LOGIC: Strong bias against directions that recently caused problems
        recent_actions = [h['action'] for h in self.action_history[-8:]]  # Look at last 8 actions
        left_recent = len([a for a in recent_actions if "left" in a.lower()])
        right_recent = len([a for a in recent_actions if "right" in a.lower()])
        
        # If we've been turning one direction a lot recently, FORCE the opposite
        if left_recent >= 4 and "right" in available_directions:
            self.get_logger().info(f"ANTI-STUCK: Forcing RIGHT turn (left used {left_recent} times recently)")
            return "right"
        elif right_recent >= 4 and "left" in available_directions:
            self.get_logger().info(f"ANTI-STUCK: Forcing LEFT turn (right used {right_recent} times recently)")
            return "left"
        
        # Post-escape bias: avoid the direction that got us stuck
        if hasattr(self, 'last_escape_time') and (time.time() - self.last_escape_time) < 45.0:
            if hasattr(self, 'last_escape_direction') and self.last_escape_direction:
                opposite_direction = "right" if self.last_escape_direction == "left" else "left"
                if opposite_direction in available_directions:
                    self.get_logger().info(f"POST-ESCAPE: Choosing {opposite_direction} (avoiding stuck direction {self.last_escape_direction})")
                    return opposite_direction
        
        # Default: prefer alternating behavior
        last_turn_direction = None
        for action in reversed(self.action_history[-5:]):
            if "rotate_" in action['action']:
                if "left" in action['action']:
                    last_turn_direction = "left"
                    break
                elif "right" in action['action']:
                    last_turn_direction = "right"
                    break
        
        if last_turn_direction == "left" and "right" in available_directions:
            self.get_logger().info("ALTERNATING: Last turn was left, choosing right")
            return "right"
        elif last_turn_direction == "right" and "left" in available_directions:
            self.get_logger().info("ALTERNATING: Last turn was right, choosing left")
            return "left"
        
        # If no pattern, prefer right (arbitrary but consistent choice)
        chosen_direction = "right" if "right" in available_directions else "left"
        self.get_logger().info(f"DEFAULT: Choosing {chosen_direction} (no clear pattern)")
        return chosen_direction

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
        """Detect if robot is stuck in a pattern or spinning"""
        current_time = time.time()
        
        # Check if we've been turning too long consecutively
        if self.consecutive_turns >= self.max_consecutive_turns:
            self.get_logger().warning(f"Detected excessive turning: {self.consecutive_turns} consecutive turns")
            return True, "excessive_turning"
        
        # Check if current action has been running too long (reduced time for faster detection)
        action_duration = current_time - self.action_start_time
        if action_duration > 5.0:  # Reduced from 8 seconds to 5 seconds
            if self.current_action in ["rotate_left", "rotate_right", "navigate_left", "navigate_right"]:
                self.get_logger().warning(f"Stuck in {self.current_action} for {action_duration:.1f}s")
                return True, "stuck_turning"
        
        # More aggressive oscillation detection
        if len(self.action_history) >= 3:
            recent_actions = [h['action'] for h in self.action_history[-3:]]
            
            # Check for any repetitive turning pattern
            left_turns = len([a for a in recent_actions if "left" in a.lower()])
            right_turns = len([a for a in recent_actions if "right" in a.lower()])
            
            if left_turns >= 2 and right_turns >= 1:
                self.get_logger().warning(f"Detected turning oscillation: {recent_actions}")
                return True, "oscillation"
            elif right_turns >= 2 and left_turns >= 1:
                self.get_logger().warning(f"Detected turning oscillation: {recent_actions}")
                return True, "oscillation"
        
        return False, "none"
    
    def execute_escape_behavior(self, stuck_reason):
        """Execute escape behavior when stuck"""
        current_time = time.time()
        
        # Start escape mode if not already in it
        if not self.escape_mode:
            self.escape_mode = True
            self.escape_start_time = current_time
            
            # Record what direction got us stuck for future avoidance
            if "left" in self.current_action.lower():
                self.problematic_direction = "left"
            elif "right" in self.current_action.lower():
                self.problematic_direction = "right"
            else:
                self.problematic_direction = None
                
            self.get_logger().info(f"Initiating escape behavior due to: {stuck_reason}")
            if hasattr(self, 'problematic_direction') and self.problematic_direction:
                self.get_logger().info(f"Recording problematic direction: {self.problematic_direction}")
        
        escape_duration = current_time - self.escape_start_time
        
        # Escape behavior sequence:
        # 1. Back up for 2.5 seconds (longer backup)
        # 2. Turn in preferred direction for 2.5 seconds 
        # 3. Try to move forward
        
        if escape_duration < 2.5:
            # Phase 1: Back up (longer backup)
            if self.current_action != "escape_backward":
                actual_speed = max(90, self.default_speed)
                linear_speed = -(actual_speed / 100.0)
                
                if self.send_movement_command(linear_x=linear_speed):
                    self.current_action = "escape_backward"
                    self.get_logger().info(f"Escape: backing up at {actual_speed}% power")
                    
        elif escape_duration < 5.0:
            # Phase 2: Turn (choose opposite of problematic direction if known)
            if self.current_action != "escape_turn":
                if hasattr(self, 'problematic_direction') and self.problematic_direction:
                    # Turn opposite to the problematic direction
                    if self.problematic_direction == "left":
                        turn_direction = "right"
                        angular_speed = -0.90  # Right turn at 90%
                    else:
                        turn_direction = "left"
                        angular_speed = 0.90   # Left turn at 90%
                else:
                    # Fallback: alternate from last escape direction
                    if self.last_escape_direction == "left":
                        turn_direction = "right"
                        angular_speed = -0.90
                    else:
                        turn_direction = "left"
                        angular_speed = 0.90
                
                if self.send_movement_command(angular_z=angular_speed):
                    self.current_action = "escape_turn"
                    self.last_escape_direction = turn_direction
                    self.get_logger().info(f"Escape: turning {turn_direction} at 90% power")
                    
        else:
            # Phase 3: End escape mode and try forward
            self.escape_mode = False
            self.consecutive_turns = 0  # Reset turn counter
            self.action_history = []    # Clear history to start fresh
            
            # Track when we last escaped and what direction caused the stuck state
            self.last_escape_time = time.time()
            
            if self.current_action != "escape_forward":
                actual_speed = max(90, self.default_speed)
                linear_speed = actual_speed / 100.0
                
                if self.send_movement_command(linear_x=linear_speed):
                    self.current_action = "escape_forward"
                    self.get_logger().info(f"Escape complete: trying forward at {actual_speed}% power")
                    
                    # Force a longer forward attempt after escape to avoid immediate re-stuck
                    self.post_escape_forward_time = time.time()
                    self.post_escape_forward_duration = 4.0  # Increased to 4 seconds

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

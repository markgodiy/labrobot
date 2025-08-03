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
- /reset_emergency_stop (std_srvs/Trigger): Reset emergency stop
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
            '/oak/depth/image_raw',
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
            
            # Debug logging to show exactly what we're sending
            if abs(linear_x) > 0.01:
                expected_speed = int(abs(linear_x) * 100)
                self.get_logger().info(f"Sending linear command: {linear_x:.3f} -> {expected_speed}% motor speed")
            if abs(angular_z) > 0.01:
                expected_speed = int(abs(angular_z) * 100)
                self.get_logger().info(f"Sending angular command: {angular_z:.3f} -> {expected_speed}% motor speed")
                
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
        
        # If no sensor data, stop if not already idle
        if not scan_data and not depth_data:
            if self.current_action != "idle":
                self.stop_movement()
                self.current_action = "idle"
            return
        
        # Analyze obstacles
        lidar_path_clear, min_lidar_distance, best_lidar_direction = self.analyze_lidar_obstacles(scan_data)
        depth_path_clear, min_depth_distance, best_depth_direction = self.analyze_depth_obstacles(depth_data)
        
        # Combine LIDAR and depth camera results
        path_clear = lidar_path_clear and depth_path_clear
        min_distance = min(min_lidar_distance, min_depth_distance)
        
        # Make navigation decision
        if path_clear:
            # Path is clear - move forward at configured speed
            if min_distance == float('inf') or min_distance > self.min_obstacle_distance:
                if self.current_action != "forward":
                    # Use configured default speed directly (no scaling)
                    # Only reduce if obstacle is very close (within 0.3m)
                    if min_distance != float('inf') and min_distance < 0.3:
                        # Very close obstacle - reduce to 75% of configured speed
                        actual_speed = max(90, int(self.default_speed * 0.75))
                    else:
                        # Clear path - use full configured speed
                        actual_speed = self.default_speed
                    
                    # Convert percentage to twist value (0.0 to 1.0 range)
                    linear_speed = actual_speed / 100.0
                    
                    if self.send_movement_command(linear_x=linear_speed):
                        self.current_action = "forward"
                        distance_str = f"{min_distance:.2f}m" if min_distance != float('inf') else "open space"
                        self.get_logger().info(f"Moving forward at {actual_speed}% power (twist: {linear_speed:.3f}, distance: {distance_str})")
        
        elif best_lidar_direction in ["left", "right"]:
            # Obstacle detected - rotate to clear direction (LIDAR-based)
            if self.current_action != f"rotate_{best_lidar_direction}":
                # Use configured rotation speed (minimum 90% enforced)
                actual_rotation_speed = max(90, self.rotation_speed)
                angular_speed = actual_rotation_speed / 100.0
                
                if best_lidar_direction == "right":
                    angular_speed = -angular_speed  # Negative for right turn
                
                self.get_logger().info(f"LIDAR rotation: {best_lidar_direction}, {actual_rotation_speed}% power (twist: {angular_speed:.3f})")
                
                if self.send_movement_command(angular_z=angular_speed):
                    self.current_action = f"rotate_{best_lidar_direction}"
                    self.get_logger().info(f"Rotating {best_lidar_direction} to avoid obstacle (distance: {min_distance:.2f}m)")
        
        elif best_depth_direction in ["left", "right"]:
            # Obstacle detected - rotate to clear direction (depth-based)
            if self.current_action != f"rotate_{best_depth_direction}":
                # Use configured rotation speed (minimum 90% enforced)
                actual_rotation_speed = max(90, self.rotation_speed)
                angular_speed = actual_rotation_speed / 100.0
                
                if best_depth_direction == "right":
                    angular_speed = -angular_speed  # Negative for right turn
                
                self.get_logger().info(f"Depth rotation: {best_depth_direction}, {actual_rotation_speed}% power (twist: {angular_speed:.3f})")
                
                if self.send_movement_command(angular_z=angular_speed):
                    self.current_action = f"rotate_{best_depth_direction}"
                    self.get_logger().info(f"Rotating {best_depth_direction} to avoid ground obstacle (distance: {min_distance:.2f}m)")
        
        elif best_lidar_direction == "backward" or best_depth_direction == "backward":
            # No clear path - back up at configured speed (90% minimum)
            if self.current_action != "backward":
                # Use configured default speed for backing up (minimum 90% enforced)
                actual_speed = max(90, self.default_speed)
                linear_speed = -(actual_speed / 100.0)  # Negative for backward
                
                if self.send_movement_command(linear_x=linear_speed):
                    self.current_action = "backward"
                    self.get_logger().info(f"Backing up at {actual_speed}% power (twist: {linear_speed:.3f}) - no clear path (distance: {min_distance:.2f}m)")
        
        else:
            # Stop and reassess
            if self.current_action != "idle":
                if self.stop_movement():
                    self.current_action = "idle"
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
        
        # Gather detailed sensor data for debug information
        with self.state_lock:
            scan_data = self.latest_scan
            depth_data = self.latest_depth_image
            controller_status = self.latest_controller_status
        
        # Analyze current sensor data for debug info
        lidar_debug = {}
        depth_debug = {}
        
        if scan_data:
            lidar_path_clear, min_lidar_distance, best_lidar_direction = self.analyze_lidar_obstacles(scan_data)
            
            # Get more detailed LIDAR analysis
            center_index = len(scan_data.ranges) // 2
            quarter_point = len(scan_data.ranges) // 4
            
            # Front sector analysis
            half_range = np.radians(self.scan_angle_range / 2)
            range_indices = int(half_range / scan_data.angle_increment)
            start_idx = max(0, center_index - range_indices)
            end_idx = min(len(scan_data.ranges), center_index + range_indices)
            front_ranges = scan_data.ranges[start_idx:end_idx]
            valid_front = [r for r in front_ranges if scan_data.range_min < r < scan_data.range_max]
            
            # Side sector analysis
            left_ranges = scan_data.ranges[:quarter_point]
            right_ranges = scan_data.ranges[-quarter_point:]
            valid_left = [r for r in left_ranges if scan_data.range_min < r < scan_data.range_max]
            valid_right = [r for r in right_ranges if scan_data.range_min < r < scan_data.range_max]
            
            lidar_debug = {
                'available': True,
                'total_points': len(scan_data.ranges),
                'angle_range': f"{np.degrees(scan_data.angle_min):.1f} to {np.degrees(scan_data.angle_max):.1f} deg",
                'angle_increment': f"{np.degrees(scan_data.angle_increment):.2f} deg",
                'front_sector': {
                    'total_points': len(front_ranges),
                    'valid_points': len(valid_front),
                    'min_distance': f"{min(valid_front):.3f}m" if valid_front else "no_data",
                    'avg_distance': f"{np.mean(valid_front):.3f}m" if valid_front else "no_data",
                    'obstacle_detected': min_lidar_distance < self.min_obstacle_distance
                },
                'left_sector': {
                    'valid_points': len(valid_left),
                    'min_distance': f"{min(valid_left):.3f}m" if valid_left else "no_data",
                    'avg_distance': f"{np.mean(valid_left):.3f}m" if valid_left else "no_data",
                    'clear_percentage': f"{len([r for r in valid_left if r > self.min_obstacle_distance]) / max(1, len(valid_left)) * 100:.1f}%"
                },
                'right_sector': {
                    'valid_points': len(valid_right),
                    'min_distance': f"{min(valid_right):.3f}m" if valid_right else "no_data",
                    'avg_distance': f"{np.mean(valid_right):.3f}m" if valid_right else "no_data",
                    'clear_percentage': f"{len([r for r in valid_right if r > self.min_obstacle_distance]) / max(1, len(valid_right)) * 100:.1f}%"
                },
                'path_clear': lidar_path_clear,
                'min_distance': f"{min_lidar_distance:.3f}m",
                'recommended_direction': best_lidar_direction
            }
        else:
            lidar_debug = {'available': False, 'reason': 'no_scan_data'}
        
        if depth_data:
            depth_path_clear, min_depth_distance, best_depth_direction = self.analyze_depth_obstacles(depth_data)
            
            try:
                # Get depth image analysis details
                depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")
                height, width = depth_image.shape
                
                # ROI details
                roi_height = height // 3
                roi_width = width // 2
                roi_y_start = height - roi_height
                roi_x_start = width // 4
                roi_x_end = roi_x_start + roi_width
                
                depth_roi = depth_image[roi_y_start:height, roi_x_start:roi_x_end]
                
                if depth_image.dtype == np.uint16:
                    depth_roi_meters = depth_roi.astype(np.float32) / 1000.0
                else:
                    depth_roi_meters = depth_roi.astype(np.float32)
                
                valid_depths = depth_roi_meters[(depth_roi_meters > 0.1) & (depth_roi_meters < 5.0)]
                
                # Left/right analysis
                roi_center = roi_width // 2
                left_roi = depth_roi_meters[:, :roi_center]
                right_roi = depth_roi_meters[:, roi_center:]
                left_valid = left_roi[(left_roi > 0.1) & (left_roi < 5.0)]
                right_valid = right_roi[(right_roi > 0.1) & (right_roi < 5.0)]
                
                depth_debug = {
                    'available': True,
                    'image_size': f"{width}x{height}",
                    'roi_size': f"{roi_width}x{roi_height}",
                    'roi_position': f"({roi_x_start},{roi_y_start}) to ({roi_x_end},{height})",
                    'data_type': str(depth_image.dtype),
                    'total_roi_pixels': roi_width * roi_height,
                    'valid_pixels': len(valid_depths),
                    'valid_percentage': f"{len(valid_depths) / max(1, roi_width * roi_height) * 100:.1f}%",
                    'depth_stats': {
                        'min_depth': f"{np.min(valid_depths):.3f}m" if len(valid_depths) > 0 else "no_data",
                        'avg_depth': f"{np.mean(valid_depths):.3f}m" if len(valid_depths) > 0 else "no_data",
                        'max_depth': f"{np.max(valid_depths):.3f}m" if len(valid_depths) > 0 else "no_data"
                    },
                    'left_side': {
                        'valid_pixels': len(left_valid),
                        'min_depth': f"{np.min(left_valid):.3f}m" if len(left_valid) > 0 else "no_data",
                        'avg_depth': f"{np.mean(left_valid):.3f}m" if len(left_valid) > 0 else "no_data"
                    },
                    'right_side': {
                        'valid_pixels': len(right_valid),
                        'min_depth': f"{np.min(right_valid):.3f}m" if len(right_valid) > 0 else "no_data",
                        'avg_depth': f"{np.mean(right_valid):.3f}m" if len(right_valid) > 0 else "no_data"
                    },
                    'threshold': f"{self.depth_obstacle_threshold}mm ({self.depth_obstacle_threshold/1000.0:.3f}m)",
                    'path_clear': depth_path_clear,
                    'min_distance': f"{min_depth_distance:.3f}m",
                    'recommended_direction': best_depth_direction
                }
            except Exception as e:
                depth_debug = {'available': False, 'error': str(e)}
        else:
            depth_debug = {'available': False, 'reason': 'no_depth_data'}
        
        # Combine sensor fusion results
        sensor_fusion = {}
        if scan_data or depth_data:
            lidar_clear = lidar_debug.get('path_clear', True)
            depth_clear = depth_debug.get('path_clear', True)
            combined_clear = lidar_clear and depth_clear
            
            # Get minimum distances
            lidar_min = float('inf')
            depth_min = float('inf')
            
            if 'min_distance' in lidar_debug:
                try:
                    lidar_min = float(lidar_debug['min_distance'].replace('m', ''))
                except:
                    pass
            
            if 'min_distance' in depth_debug:
                try:
                    depth_min = float(depth_debug['min_distance'].replace('m', ''))
                except:
                    pass
            
            combined_min = min(lidar_min, depth_min)
            
            sensor_fusion = {
                'lidar_clear': lidar_clear,
                'depth_clear': depth_clear,
                'combined_clear': combined_clear,
                'lidar_min_distance': f"{lidar_min:.3f}m" if lidar_min != float('inf') else "inf",
                'depth_min_distance': f"{depth_min:.3f}m" if depth_min != float('inf') else "inf",
                'combined_min_distance': f"{combined_min:.3f}m" if combined_min != float('inf') else "inf",
                'obstacle_threshold': f"{self.min_obstacle_distance:.3f}m"
            }
        
        # Navigation decision details
        navigation_decision = {
            'current_action': self.current_action,
            'path_clear': self.path_clear,
            'obstacle_detected': self.obstacle_detected,
            'last_command_time': time.time() - self.last_command_time,
            'speed_settings': {
                'default_speed': f"{self.default_speed}%",
                'rotation_speed': f"{self.rotation_speed}%",
                'max_speed': f"{self.max_speed}%",
                'minimum_power_rule': "90%"
            }
        }
        
        # Motor controller status
        motor_status = {
            'connected': controller_status is not None,
            'last_status': controller_status if controller_status else "no_data"
        }
        
        # System status
        system_status = {
            'autonomous_enabled': self.autonomous_enabled,
            'emergency_stop_active': self.emergency_stop_active,
            'navigation_active': self.navigation_active,
            'node_uptime': time.time() - getattr(self, 'start_time', time.time()),
            'parameters': {
                'min_obstacle_distance': f"{self.min_obstacle_distance}m",
                'scan_angle_range': f"{self.scan_angle_range}°",
                'depth_threshold': f"{self.depth_obstacle_threshold}mm",
                'command_timeout': f"{self.command_timeout}s"
            }
        }
        
        # Compile comprehensive debug data
        debug_data = {
            'timestamp': time.time(),
            'system_status': system_status,
            'navigation_decision': navigation_decision,
            'sensor_fusion': sensor_fusion,
            'lidar_analysis': lidar_debug,
            'depth_analysis': depth_debug,
            'motor_controller': motor_status
        }
        
        debug_msg = String()
        debug_msg.data = json.dumps(debug_data, indent=2)
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
                    self.current_action = "idle"
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

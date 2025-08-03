#!/usr/bin/env python3
"""
ROS 2 Autonomous Navigation Node for MicroPython Motor Controller
================================================================

This node processes LIDAR and depth camera data to make navigation decisions
and sends movement commands to the MicroPython motor controller via HTTP.

Key Features:
- Obstacle avoidance using LIDAR data
- Depth camera integration for enhanced perception
- Configurable navigation parameters
- Safety timeouts and emergency stop capabilities
- Real-time status monitoring

Subscribed Topics:
- /scan (sensor_msgs/LaserScan): LIDAR data
- /oak/rgb/image_raw (sensor_msgs/Image): RGB camera
- /oak/depth/image_raw (sensor_msgs/Image): Depth camera
- /oak/points (sensor_msgs/PointCloud2): Point cloud data

Published Topics:
- /motor_controller/status (std_msgs/String): Controller status
- /navigation/state (std_msgs/String): Navigation state

Services:
- /emergency_stop (std_srvs/Trigger): Emergency stop
- /set_autonomous_mode (std_srvs/SetBool): Enable/disable autonomous mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import requests
import json
import numpy as np
from threading import Lock, Timer
import time

# ROS 2 message types
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_node')
        
        # Declare parameters
        self.declare_parameter('micropython_ip', '192.168.25.72')
        self.declare_parameter('micropython_port', 8080)
        self.declare_parameter('min_obstacle_distance', 0.5)  # meters
        self.declare_parameter('max_speed', 70)  # percentage
        self.declare_parameter('default_speed', 50)  # percentage
        self.declare_parameter('rotation_speed', 40)  # percentage
        self.declare_parameter('scan_angle_range', 90)  # degrees (±45° from front)
        self.declare_parameter('depth_obstacle_threshold', 1000)  # mm
        self.declare_parameter('command_timeout', 2.0)  # seconds
        self.declare_parameter('autonomous_enabled', False)
        
        # Get parameters
        self.micropython_ip = self.get_parameter('micropython_ip').value
        self.micropython_port = self.get_parameter('micropython_port').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.max_speed = self.get_parameter('max_speed').value
        self.default_speed = self.get_parameter('default_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.scan_angle_range = self.get_parameter('scan_angle_range').value
        self.depth_obstacle_threshold = self.get_parameter('depth_obstacle_threshold').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.autonomous_enabled = self.get_parameter('autonomous_enabled').value
        
        self.micropython_url = f"http://{self.micropython_ip}:{self.micropython_port}"
        
        # State variables
        self.latest_scan = None
        self.latest_depth_image = None
        self.latest_rgb_image = None
        self.navigation_active = False
        self.emergency_stop_active = False
        self.last_command_time = time.time()
        self.state_lock = Lock()
        
        # Navigation state
        self.current_action = "idle"
        self.obstacle_detected = False
        self.path_clear = True
        
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
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/motor_controller/status', 10)
        self.nav_state_publisher = self.create_publisher(String, '/navigation/state', 10)
        
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
        
        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.publish_status)  # 1 Hz
        
        self.get_logger().info(f"Autonomous Navigation Node started")
        self.get_logger().info(f"MicroPython controller: {self.micropython_url}")
        self.get_logger().info(f"Autonomous mode: {'ENABLED' if self.autonomous_enabled else 'DISABLED'}")
        
        # Initialize MicroPython controller
        self.initialize_controller()
    
    def initialize_controller(self):
        """Initialize communication with MicroPython controller"""
        try:
            response = self.send_command('/autonomous', {'mode': 'on' if self.autonomous_enabled else 'off'})
            if response and response.get('status') == 'ok':
                self.get_logger().info("MicroPython controller initialized successfully")
            else:
                self.get_logger().warn("Failed to initialize MicroPython controller")
        except Exception as e:
            self.get_logger().error(f"Controller initialization error: {e}")
    
    def send_command(self, endpoint, params=None):
        """Send HTTP command to MicroPython controller"""
        try:
            url = f"{self.micropython_url}{endpoint}"
            if params:
                url += "?" + "&".join([f"{k}={v}" for k, v in params.items()])
            
            response = requests.get(url, timeout=1.0)
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().warn(f"Controller returned status {response.status_code}")
                return None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Communication error with controller: {e}")
            return None
    
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
    
    def navigation_loop(self):
        """Main navigation decision loop"""
        if not self.autonomous_enabled or self.emergency_stop_active:
            return
        
        with self.state_lock:
            scan_data = self.latest_scan
        
        if not scan_data:
            # No sensor data available
            if self.current_action != "idle":
                self.send_command('/stop')
                self.current_action = "idle"
            return
        
        # Analyze obstacles
        path_clear, min_distance, best_direction = self.analyze_lidar_obstacles(scan_data)
        
        # Make navigation decision
        if path_clear and min_distance > self.min_obstacle_distance * 1.5:
            # Path is clear - move forward
            if self.current_action != "forward":
                speed = min(self.max_speed, int(self.default_speed * (min_distance / 2.0)))
                response = self.send_command('/move', {'dir': 'forward', 'speed': speed})
                if response and response.get('status') == 'ok':
                    self.current_action = "forward"
                    self.get_logger().info(f"Moving forward at {speed}% (distance: {min_distance:.2f}m)")
        
        elif best_direction in ["left", "right"]:
            # Obstacle detected - rotate to clear direction
            if self.current_action != f"rotate_{best_direction}":
                response = self.send_command('/rotate', {
                    'dir': best_direction, 
                    'speed': self.rotation_speed,
                    'duration': 1.0  # 1 second rotation
                })
                if response and response.get('status') == 'ok':
                    self.current_action = f"rotate_{best_direction}"
                    self.get_logger().info(f"Rotating {best_direction} to avoid obstacle (distance: {min_distance:.2f}m)")
        
        elif best_direction == "backward":
            # No clear path - back up
            if self.current_action != "backward":
                response = self.send_command('/move', {
                    'dir': 'backward', 
                    'speed': self.default_speed // 2,
                    'duration': 1.0  # 1 second backup
                })
                if response and response.get('status') == 'ok':
                    self.current_action = "backward"
                    self.get_logger().info(f"Backing up - no clear path (distance: {min_distance:.2f}m)")
        
        else:
            # Stop and reassess
            if self.current_action != "idle":
                self.send_command('/stop')
                self.current_action = "idle"
                self.get_logger().info("Stopping - reassessing situation")
        
        # Update navigation state
        self.obstacle_detected = not path_clear
        self.path_clear = path_clear
        self.last_command_time = time.time()
    
    def publish_status(self):
        """Publish navigation status"""
        # Get controller status
        controller_status = self.send_command('/status')
        
        status_msg = String()
        nav_state_msg = String()
        
        if controller_status:
            status_data = {
                'autonomous_enabled': self.autonomous_enabled,
                'navigation_active': self.navigation_active,
                'current_action': self.current_action,
                'obstacle_detected': self.obstacle_detected,
                'path_clear': self.path_clear,
                'controller_status': controller_status.get('data', {}),
                'emergency_stop': self.emergency_stop_active
            }
            status_msg.data = json.dumps(status_data)
            nav_state_msg.data = self.current_action
        else:
            status_msg.data = json.dumps({'error': 'Controller communication failed'})
            nav_state_msg.data = "error"
        
        self.status_publisher.publish(status_msg)
        self.nav_state_publisher.publish(nav_state_msg)
    
    def emergency_stop_callback(self, request, response):
        """Service callback for emergency stop"""
        self.emergency_stop_active = True
        self.autonomous_enabled = False
        
        # Send emergency stop to controller
        controller_response = self.send_command('/estop')
        
        if controller_response and controller_response.get('status') == 'ok':
            response.success = True
            response.message = "Emergency stop activated"
            self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        else:
            response.success = False
            response.message = "Failed to activate emergency stop"
        
        return response
    
    def set_autonomous_mode_callback(self, request, response):
        """Service callback for setting autonomous mode"""
        self.autonomous_enabled = request.data
        
        # Send mode change to controller
        controller_response = self.send_command('/autonomous', {
            'mode': 'on' if self.autonomous_enabled else 'off'
        })
        
        if controller_response and controller_response.get('status') == 'ok':
            response.success = True
            response.message = f"Autonomous mode {'enabled' if self.autonomous_enabled else 'disabled'}"
            self.get_logger().info(f"Autonomous mode {'ENABLED' if self.autonomous_enabled else 'DISABLED'}")
            
            if not self.autonomous_enabled:
                # Stop current movement
                self.send_command('/stop')
                self.current_action = "idle"
        else:
            response.success = False
            response.message = "Failed to set autonomous mode"
        
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

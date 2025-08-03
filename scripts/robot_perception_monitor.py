#!/usr/bin/env python3
"""
Robot Perception Monitor
========================

A simple terminal-based monitoring tool to see what the robot is perceiving
and deciding in real-time. Shows LIDAR data, navigation decisions, and
motor commands in a user-friendly format.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import numpy as np
import time
import os

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        
        # Subscribe to topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.nav_state_sub = self.create_subscription(String, '/navigation/state', self.nav_state_callback, 10)
        self.nav_debug_sub = self.create_subscription(String, '/navigation/debug', self.nav_debug_callback, 10)
        self.motor_status_sub = self.create_subscription(String, '/motor_controller/status', self.motor_status_callback, 10)
        
        # Data storage
        self.latest_scan = None
        self.nav_state = "unknown"
        self.nav_debug = {}
        self.motor_status = {}
        
        # Display timer
        self.display_timer = self.create_timer(1.0, self.display_status)
        
        self.get_logger().info("Robot Monitor started - watching sensors and decisions...")
        
    def scan_callback(self, msg):
        """Process LIDAR scan data"""
        self.latest_scan = msg
        
    def nav_state_callback(self, msg):
        """Process navigation state"""
        self.nav_state = msg.data
        
    def nav_debug_callback(self, msg):
        """Process navigation debug info"""
        try:
            self.nav_debug = json.loads(msg.data)
        except:
            pass
            
    def motor_status_callback(self, msg):
        """Process motor controller status"""
        try:
            self.motor_status = json.loads(msg.data)
        except:
            pass
    
    def analyze_scan(self):
        """Analyze LIDAR scan for key information"""
        if not self.latest_scan:
            return "No LIDAR data", [], []
            
        ranges = np.array(self.latest_scan.ranges)
        valid_ranges = ranges[(ranges > self.latest_scan.range_min) & (ranges < self.latest_scan.range_max)]
        
        if len(valid_ranges) == 0:
            return "No valid readings", [], []
        
        # Analyze sectors
        total_points = len(ranges)
        sector_size = total_points // 8
        
        sectors = {
            'Front': ranges[3*sector_size:5*sector_size],
            'Front-Left': ranges[2*sector_size:3*sector_size], 
            'Front-Right': ranges[5*sector_size:6*sector_size],
            'Left': ranges[1*sector_size:2*sector_size],
            'Right': ranges[6*sector_size:7*sector_size],
            'Back': np.concatenate([ranges[:sector_size], ranges[7*sector_size:]])
        }
        
        sector_analysis = {}
        obstacles = []
        clear_paths = []
        
        for name, sector_ranges in sectors.items():
            valid_sector = sector_ranges[(sector_ranges > self.latest_scan.range_min) & 
                                      (sector_ranges < self.latest_scan.range_max)]
            
            if len(valid_sector) > 0:
                min_dist = np.min(valid_sector)
                avg_dist = np.mean(valid_sector)
                sector_analysis[name] = {'min': min_dist, 'avg': avg_dist}
                
                if min_dist < 0.5:  # Obstacle threshold
                    obstacles.append(f"{name}: {min_dist:.2f}m")
                elif min_dist > 2.0:  # Clear path threshold
                    clear_paths.append(f"{name}: {min_dist:.2f}m")
        
        return f"Min: {np.min(valid_ranges):.2f}m, Avg: {np.mean(valid_ranges):.2f}m", obstacles, clear_paths
    
    def display_status(self):
        """Display current robot status"""
        # Clear terminal
        os.system('clear')
        
        print("="*80)
        print("🤖 ROBOT PERCEPTION MONITOR - VERBOSE DEBUG MODE")
        print("="*80)
        
        # Current time
        print(f"⏰ Time: {time.strftime('%H:%M:%S')}")
        print()
        
        # Navigation state
        print(f"🧭 Navigation State: {self.nav_state.upper()}")
        
        # System status from verbose debug
        if self.nav_debug and 'system_status' in self.nav_debug:
            sys_status = self.nav_debug['system_status']
            autonomous = "✅ ON" if sys_status.get('autonomous_enabled', False) else "❌ OFF"
            emergency = "🚨 ACTIVE" if sys_status.get('emergency_stop_active', False) else "✅ CLEAR"
            uptime = sys_status.get('node_uptime', 0)
            
            print(f"🔄 Autonomous: {autonomous}")
            print(f"🛑 Emergency Stop: {emergency}")
            print(f"⏱️  Node Uptime: {uptime:.1f}s")
            print()
            
            # Navigation decision details
            if 'navigation_decision' in self.nav_debug:
                nav_decision = self.nav_debug['navigation_decision']
                print("🎯 NAVIGATION DECISION")
                print("-" * 40)
                print(f"Current Action: {nav_decision.get('current_action', 'unknown').upper()}")
                print(f"Path Clear: {'✅ YES' if nav_decision.get('path_clear', False) else '❌ NO'}")
                print(f"Obstacle Detected: {'⚠️  YES' if nav_decision.get('obstacle_detected', False) else '✅ NO'}")
                print(f"Last Command: {nav_decision.get('last_command_time', 0):.2f}s ago")
                
                if 'speed_settings' in nav_decision:
                    speed_settings = nav_decision['speed_settings']
                    print(f"Speed Settings: Default={speed_settings.get('default_speed', 'N/A')}, " +
                          f"Rotation={speed_settings.get('rotation_speed', 'N/A')}, " +
                          f"Min Power={speed_settings.get('minimum_power_rule', 'N/A')}")
                print()
            
            # Sensor fusion results
            if 'sensor_fusion' in self.nav_debug:
                fusion = self.nav_debug['sensor_fusion']
                print("🔬 SENSOR FUSION")
                print("-" * 40)
                print(f"LIDAR Clear: {'✅' if fusion.get('lidar_clear', False) else '❌'} " +
                      f"(min: {fusion.get('lidar_min_distance', 'N/A')})")
                print(f"Depth Clear: {'✅' if fusion.get('depth_clear', False) else '❌'} " +
                      f"(min: {fusion.get('depth_min_distance', 'N/A')})")
                print(f"Combined: {'✅ CLEAR' if fusion.get('combined_clear', False) else '❌ BLOCKED'} " +
                      f"(min: {fusion.get('combined_min_distance', 'N/A')})")
                print(f"Obstacle Threshold: {fusion.get('obstacle_threshold', 'N/A')}")
                print()
            
            # LIDAR analysis details
            if 'lidar_analysis' in self.nav_debug:
                lidar = self.nav_debug['lidar_analysis']
                print("📡 LIDAR ANALYSIS")
                print("-" * 40)
                if lidar.get('available', False):
                    print(f"Data Available: ✅ ({lidar.get('total_points', 0)} points)")
                    print(f"Angle Range: {lidar.get('angle_range', 'N/A')}")
                    
                    if 'front_sector' in lidar:
                        front = lidar['front_sector']
                        print(f"Front Sector: {front.get('valid_points', 0)} valid points")
                        print(f"  Min Distance: {front.get('min_distance', 'N/A')}")
                        print(f"  Avg Distance: {front.get('avg_distance', 'N/A')}")
                        print(f"  Obstacle: {'⚠️  YES' if front.get('obstacle_detected', False) else '✅ NO'}")
                    
                    if 'left_sector' in lidar and 'right_sector' in lidar:
                        left = lidar['left_sector']
                        right = lidar['right_sector']
                        print(f"Left Side: {left.get('clear_percentage', 'N/A')} clear " +
                              f"(min: {left.get('min_distance', 'N/A')})")
                        print(f"Right Side: {right.get('clear_percentage', 'N/A')} clear " +
                              f"(min: {right.get('min_distance', 'N/A')})")
                    
                    print(f"Recommended Direction: {lidar.get('recommended_direction', 'N/A').upper()}")
                else:
                    print(f"Data Available: ❌ ({lidar.get('reason', 'unknown')})")
                print()
            
            # Depth camera analysis
            if 'depth_analysis' in self.nav_debug:
                depth = self.nav_debug['depth_analysis']
                print("📷 DEPTH CAMERA ANALYSIS")
                print("-" * 40)
                if depth.get('available', False):
                    print(f"Data Available: ✅ ({depth.get('image_size', 'N/A')})")
                    print(f"ROI: {depth.get('roi_size', 'N/A')} at {depth.get('roi_position', 'N/A')}")
                    print(f"Valid Pixels: {depth.get('valid_pixels', 0)} " +
                          f"({depth.get('valid_percentage', 'N/A')})")
                    
                    if 'depth_stats' in depth:
                        stats = depth['depth_stats']
                        print(f"Depth Range: {stats.get('min_depth', 'N/A')} to {stats.get('max_depth', 'N/A')} " +
                              f"(avg: {stats.get('avg_depth', 'N/A')})")
                    
                    if 'left_side' in depth and 'right_side' in depth:
                        left = depth['left_side']
                        right = depth['right_side']
                        print(f"Left Side: {left.get('valid_pixels', 0)} pixels " +
                              f"(min: {left.get('min_depth', 'N/A')})")
                        print(f"Right Side: {right.get('valid_pixels', 0)} pixels " +
                              f"(min: {right.get('min_depth', 'N/A')})")
                    
                    print(f"Threshold: {depth.get('threshold', 'N/A')}")
                    print(f"Recommended Direction: {depth.get('recommended_direction', 'N/A').upper()}")
                else:
                    reason = depth.get('reason', depth.get('error', 'unknown'))
                    print(f"Data Available: ❌ ({reason})")
                print()
            
            # Motor controller status
            if 'motor_controller' in self.nav_debug:
                motor = self.nav_debug['motor_controller']
                print("🔧 MOTOR CONTROLLER")
                print("-" * 40)
                connected = "✅ CONNECTED" if motor.get('connected', False) else "❌ DISCONNECTED"
                print(f"Status: {connected}")
                
                last_status = motor.get('last_status', {})
                if isinstance(last_status, dict) and last_status:
                    print(f"Last Status: {json.dumps(last_status, indent=2)}")
                elif last_status and last_status != "no_data":
                    print(f"Last Status: {last_status}")
                print()
        
        # Fallback to old format if verbose debug not available
        elif self.nav_debug:
            autonomous = "✅ ON" if self.nav_debug.get('autonomous_enabled', False) else "❌ OFF"
            emergency = "🚨 ACTIVE" if self.nav_debug.get('emergency_stop', False) else "✅ CLEAR"
            obstacle = "⚠️  DETECTED" if self.nav_debug.get('obstacle_detected', False) else "✅ CLEAR"
            path = "✅ CLEAR" if self.nav_debug.get('path_clear', True) else "❌ BLOCKED"
            
            print(f"🔄 Autonomous: {autonomous}")
            print(f"🛑 Emergency Stop: {emergency}")
            print(f"⚠️  Obstacles: {obstacle}")
            print(f"🛤️  Path: {path}")
            print()
        
        # LIDAR basic analysis (fallback)
        scan_summary, obstacles, clear_paths = self.analyze_scan()
        print("📊 LIDAR SUMMARY")
        print("-" * 40)
        print(f"📡 Status: {scan_summary}")
        
        if obstacles:
            print(f"⚠️  Obstacles: {', '.join(obstacles)}")
        if clear_paths:
            print(f"✅ Clear Paths: {', '.join(clear_paths)}")
        
        print()
        print("Press Ctrl+C to stop monitoring")
        print("="*80)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = RobotMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n👋 Shutting down robot monitor...")
    finally:
        if 'monitor' in locals():
            monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

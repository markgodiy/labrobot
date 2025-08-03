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
        
        print("="*60)
        print("🤖 ROBOT PERCEPTION MONITOR")
        print("="*60)
        
        # Current time
        print(f"⏰ Time: {time.strftime('%H:%M:%S')}")
        print()
        
        # Navigation state
        print(f"🧭 Navigation State: {self.nav_state.upper()}")
        
        # Navigation debug info
        if self.nav_debug:
            autonomous = "✅ ON" if self.nav_debug.get('autonomous_enabled', False) else "❌ OFF"
            emergency = "🚨 ACTIVE" if self.nav_debug.get('emergency_stop', False) else "✅ CLEAR"
            obstacle = "⚠️  DETECTED" if self.nav_debug.get('obstacle_detected', False) else "✅ CLEAR"
            path = "✅ CLEAR" if self.nav_debug.get('path_clear', True) else "❌ BLOCKED"
            
            print(f"🔄 Autonomous: {autonomous}")
            print(f"🛑 Emergency: {emergency}")
            print(f"🚧 Obstacles: {obstacle}")
            print(f"🛤️  Path: {path}")
        print()
        
        # LIDAR analysis
        scan_summary, obstacles, clear_paths = self.analyze_scan()
        print(f"👁️  LIDAR: {scan_summary}")
        
        if obstacles:
            print("⚠️  OBSTACLES DETECTED:")
            for obs in obstacles[:5]:  # Show top 5
                print(f"   • {obs}")
        
        if clear_paths:
            print("✅ CLEAR PATHS:")
            for path in clear_paths[:3]:  # Show top 3
                print(f"   • {path}")
        print()
        
        # Motor status
        if self.motor_status:
            health = self.motor_status.get('health', 'unknown')
            moving = "🏃 MOVING" if self.motor_status.get('is_moving', False) else "🛑 STOPPED"
            speed = self.motor_status.get('current_speed', 0)
            estop = "🚨 ACTIVE" if self.motor_status.get('emergency_stop', False) else "✅ CLEAR"
            
            health_icon = "✅" if health == "healthy" else "⚠️" if health == "warning" else "❌"
            
            print(f"⚙️  Motor Status: {health_icon} {health.upper()}")
            print(f"🏃 Movement: {moving} (Speed: {speed}%)")
            print(f"🛑 E-Stop: {estop}")
        
        print()
        print("Press Ctrl+C to exit monitoring...")
        print("="*60)

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

#!/usr/bin/env python3
"""
Serial-based Autonomous Navigation Control Utility
==================================================

This utility provides easy control over the serial-based autonomous navigation system:
- Start/stop autonomous mode
- Emergency stop
- Monitor status
- Manual movement commands via cmd_vel

Usage:
  # Enable autonomous mode
  python3 serial_nav_control.py --autonomous on
  
  # Disable autonomous mode  
  python3 serial_nav_control.py --autonomous off
  
  # Emergency stop
  python3 serial_nav_control.py --emergency-stop
  
  # Monitor status
  python3 serial_nav_control.py --status
  
  # Manual control (when autonomous is off)
  python3 serial_nav_control.py --move forward --speed 0.5
  python3 serial_nav_control.py --rotate left --speed 0.3
  python3 serial_nav_control.py --stop
"""

import rclpy
from rclpy.node import Node
import json
import argparse
import time
import sys
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialNavigationController(Node):
    def __init__(self):
        super().__init__('serial_navigation_controller')
        
        # Service clients
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        self.motor_emergency_stop_client = self.create_client(Trigger, '/motor/emergency_stop')
        self.autonomous_mode_client = self.create_client(SetBool, '/set_autonomous_mode')
        self.motor_stop_client = self.create_client(Trigger, '/motor/stop')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Status subscribers
        self.motor_status_subscriber = self.create_subscription(
            String,
            '/motor_controller/status',
            self.motor_status_callback,
            10
        )
        
        self.nav_state_subscriber = self.create_subscription(
            String,
            '/navigation/state',
            self.nav_state_callback,
            10
        )
        
        self.nav_debug_subscriber = self.create_subscription(
            String,
            '/navigation/debug',
            self.nav_debug_callback,
            10
        )
        
        self.latest_motor_status = None
        self.latest_nav_state = None
        self.latest_nav_debug = None
    
    def motor_status_callback(self, msg):
        """Receive motor controller status updates"""
        try:
            self.latest_motor_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_motor_status = {'error': 'Invalid status format'}
    
    def nav_state_callback(self, msg):
        """Receive navigation state updates"""
        self.latest_nav_state = msg.data
    
    def nav_debug_callback(self, msg):
        """Receive navigation debug updates"""
        try:
            self.latest_nav_debug = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_nav_debug = {'error': 'Invalid debug format'}
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        print("🚨 EMERGENCY STOP ACTIVATED 🚨")
        
        # Try navigation emergency stop first
        if self.emergency_stop_client.wait_for_service(timeout_sec=2.0):
            request = Trigger.Request()
            future = self.emergency_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    print(f"✅ Navigation Emergency Stop: {result.message}")
                    return True
                else:
                    print(f"❌ Navigation Emergency Stop Failed: {result.message}")
        
        # Fallback to motor emergency stop
        if self.motor_emergency_stop_client.wait_for_service(timeout_sec=2.0):
            request = Trigger.Request()
            future = self.motor_emergency_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    print(f"✅ Motor Emergency Stop: {result.message}")
                    return True
                else:
                    print(f"❌ Motor Emergency Stop Failed: {result.message}")
        
        print("❌ All emergency stop methods failed")
        return False
    
    def set_autonomous_mode(self, enabled):
        """Enable or disable autonomous mode"""
        mode_str = "ENABLED" if enabled else "DISABLED"
        print(f"🤖 Setting autonomous mode: {mode_str}")
        
        if self.autonomous_mode_client.wait_for_service(timeout_sec=2.0):
            request = SetBool.Request()
            request.data = enabled
            future = self.autonomous_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    print(f"✅ Autonomous Mode: {result.message}")
                    return True
                else:
                    print(f"❌ Autonomous Mode Failed: {result.message}")
        
        print("❌ Autonomous mode service not available")
        return False
    
    def manual_move(self, direction, speed=0.5, duration=0):
        """Send manual movement command via cmd_vel"""
        print(f"🎮 Manual move: {direction} at {speed} speed")
        
        twist_msg = Twist()
        
        if direction == 'forward':
            twist_msg.linear.x = float(speed)
        elif direction == 'backward':
            twist_msg.linear.x = -float(speed)
        elif direction == 'left':
            twist_msg.angular.z = float(speed)
        elif direction == 'right':
            twist_msg.angular.z = -float(speed)
        else:
            print(f"❌ Invalid direction: {direction}")
            return False
        
        try:
            self.cmd_vel_publisher.publish(twist_msg)
            print(f"✅ Movement command sent")
            
            # If duration specified, stop after that time
            if duration > 0:
                time.sleep(duration)
                self.stop_movement()
            
            return True
        except Exception as e:
            print(f"❌ Movement command failed: {e}")
            return False
    
    def stop_movement(self):
        """Stop all movement"""
        print("⏹️  Stopping movement")
        
        # Send zero velocity
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Also try motor stop service
        if self.motor_stop_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            future = self.motor_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None and future.result().success:
                print(f"✅ Stop command: {future.result().message}")
                return True
        
        print("✅ Zero velocity command sent")
        return True
    
    def get_status(self, monitor=False):
        """Get and display system status"""
        if monitor:
            print("📊 Monitoring status (Press Ctrl+C to stop)...")
            try:
                while True:
                    self.display_status()
                    time.sleep(1.0)
                    # Spin to receive updates
                    rclpy.spin_once(self, timeout_sec=0.1)
            except KeyboardInterrupt:
                print("\n👋 Monitoring stopped")
        else:
            # Spin briefly to get latest status
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            self.display_status()
    
    def display_status(self):
        """Display current status with enhanced monitoring"""
        print("\n" + "="*80)
        print("🤖 SERIAL AUTONOMOUS NAVIGATION STATUS - ENHANCED MONITORING")
        print("="*80)
        
        # Motor controller status
        if self.latest_motor_status:
            if 'error' not in self.latest_motor_status:
                print(f"🔗 Motor Controller: ✅ CONNECTED")
                
                # Basic status
                if self.latest_motor_status.get('type') == 'status':
                    autonomous = self.latest_motor_status.get('autonomous_mode', False)
                    is_moving = self.latest_motor_status.get('is_moving', False)
                    emergency_stop = self.latest_motor_status.get('emergency_stop', True)
                    health_status = self.latest_motor_status.get('health_status', 'unknown')
                    
                    print(f"  🎯 Autonomous Mode: {'🟢 ENABLED' if autonomous else '🔴 DISABLED'}")
                    print(f"  🏃 Movement Status: {'🏃 MOVING' if is_moving else '⏸️  STOPPED'}")
                    print(f"  🚨 Emergency Stop: {'🚨 ACTIVE' if emergency_stop else '✅ NORMAL'}")
                    print(f"  💗 Health Status: {health_status.upper()}")
                    print(f"  ⚡ Current Speed: {self.latest_motor_status.get('current_speed', 0)}")
                    print(f"  🎯 Target Speed: {self.latest_motor_status.get('target_speed', 0)}")
                    print(f"  📶 WiFi Enabled: {'📶 YES' if self.latest_motor_status.get('wifi_enabled') else '📵 NO'}")
                    print(f"  ⏱️  Uptime: {self.latest_motor_status.get('uptime_ms', 0) / 1000:.1f}s")
                    print(f"  🕐 Last Command: {self.latest_motor_status.get('last_command_age_ms', 0)}ms ago")
                    
                    # Enhanced monitoring data
                    movement = self.latest_motor_status.get('movement', {})
                    if movement:
                        move_progress = movement.get('move_progress_percent', 0)
                        rotation_progress = movement.get('rotation_progress_percent', 0)
                        time_remaining = movement.get('time_remaining_ms', 0)
                        
                        if move_progress > 0:
                            print(f"  📈 Move Progress: {move_progress:.1f}% ({time_remaining}ms remaining)")
                        if rotation_progress > 0:
                            print(f"  🔄 Rotation Progress: {rotation_progress:.1f}% ({time_remaining}ms remaining)")
                    
                    hardware = self.latest_motor_status.get('hardware', {})
                    if hardware:
                        print(f"  💡 LED State: {'🟢 ON' if hardware.get('led_state') else '⚫ OFF'}")
                        print(f"  ⚙️  PWM Frequency: {hardware.get('motor_pwm_freq', 0)}Hz")
                
                # Heartbeat data
                elif self.latest_motor_status.get('type') == 'heartbeat':
                    commands = self.latest_motor_status.get('commands_received', 0)
                    errors = self.latest_motor_status.get('total_errors', 0)
                    serial_errors = self.latest_motor_status.get('serial_errors', 0)
                    json_errors = self.latest_motor_status.get('json_errors', 0)
                    
                    print(f"  💓 Heartbeat Data:")
                    print(f"    📊 Commands Received: {commands}")
                    print(f"    ❌ Total Errors: {errors}")
                    print(f"    📡 Serial Errors: {serial_errors}")
                    print(f"    � JSON Errors: {json_errors}")
                
                # Diagnostic data
                elif self.latest_motor_status.get('type') in ['diagnostics', 'periodic_diagnostics']:
                    print(f"  🔧 Diagnostic Data:")
                    
                    system = self.latest_motor_status.get('system', {})
                    if system:
                        print(f"    📊 Commands: {system.get('total_commands', 0)}")
                        print(f"    ❌ Errors: {system.get('total_errors', 0)}")
                        print(f"    📡 Serial Errors: {system.get('serial_read_errors', 0)}")
                        print(f"    � JSON Errors: {system.get('json_parse_errors', 0)}")
                        if system.get('last_error'):
                            print(f"    🚨 Last Error: {system.get('last_error')}")
                    
                    comm = self.latest_motor_status.get('communication', {})
                    if comm:
                        cmd_age = comm.get('last_command_age_ms', 0)
                        timeout = comm.get('command_timeout_ms', 3000)
                        print(f"    🕐 Command Age: {cmd_age}ms (timeout: {timeout}ms)")
                        print(f"    💓 Heartbeat Age: {comm.get('last_heartbeat_age_ms', 0)}ms")
                
                version = self.latest_motor_status.get('version')
                if version:
                    print(f"  📦 Version: {version}")
                    
            else:
                print(f"🔗 Motor Controller: ❌ ERROR - {self.latest_motor_status['error']}")
        else:
            print(f"🔗 Motor Controller: ❓ NO STATUS RECEIVED")
        
        # Navigation status (existing code)
        if self.latest_nav_debug:
            if 'error' not in self.latest_nav_debug:
                print(f"\n🧭 Navigation System:")
                print(f"  🎯 Current Action: {self.latest_nav_debug.get('current_action', 'unknown')}")
                print(f"  🤖 Autonomous Enabled: {'🟢 YES' if self.latest_nav_debug.get('autonomous_enabled') else '🔴 NO'}")
                print(f"  🚧 Obstacle Detected: {'🚫 YES' if self.latest_nav_debug.get('obstacle_detected') else '✅ NO'}")
                print(f"  🛤️  Path Clear: {'✅ YES' if self.latest_nav_debug.get('path_clear') else '🚫 NO'}")
                print(f"  🚨 Emergency Stop: {'🚨 ACTIVE' if self.latest_nav_debug.get('emergency_stop') else '✅ NORMAL'}")
                print(f"  🔗 Controller Connected: {'✅ YES' if self.latest_nav_debug.get('controller_connected') else '❌ NO'}")
            else:
                print(f"\n🧭 Navigation System: ❌ ERROR - {self.latest_nav_debug['error']}")
        else:
            print(f"\n🧭 Navigation System: ❓ NO DEBUG DATA RECEIVED")
        
        if self.latest_nav_state:
            print(f"  📊 Navigation State: {self.latest_nav_state}")
        
        print("="*80)

def main():
    parser = argparse.ArgumentParser(description='Serial Autonomous Navigation Control')
    parser.add_argument('--autonomous', choices=['on', 'off'], help='Enable/disable autonomous mode')
    parser.add_argument('--emergency-stop', action='store_true', help='Trigger emergency stop')
    parser.add_argument('--status', action='store_true', help='Show status')
    parser.add_argument('--monitor', action='store_true', help='Monitor status continuously')
    parser.add_argument('--move', choices=['forward', 'backward'], help='Manual movement')
    parser.add_argument('--rotate', choices=['left', 'right'], help='Manual rotation')
    parser.add_argument('--stop', action='store_true', help='Stop movement')
    parser.add_argument('--speed', type=float, default=0.5, help='Movement speed (0.0-1.0)')
    parser.add_argument('--duration', type=float, default=0, help='Movement duration in seconds (0=continuous)')
    
    args = parser.parse_args()
    
    # Initialize ROS 2
    rclpy.init()
    controller = SerialNavigationController()
    
    try:
        if args.emergency_stop:
            controller.emergency_stop()
        
        elif args.autonomous:
            controller.set_autonomous_mode(args.autonomous == 'on')
        
        elif args.move:
            controller.manual_move(args.move, args.speed, args.duration)
        
        elif args.rotate:
            controller.manual_move(args.rotate, args.speed, args.duration)
        
        elif args.stop:
            controller.stop_movement()
        
        elif args.status or args.monitor:
            controller.get_status(monitor=args.monitor)
        
        else:
            parser.print_help()
            print("\n" + "="*60)
            print("🤖 QUICK COMMANDS:")
            print("="*60)
            print("Emergency Stop:     python3 serial_nav_control.py --emergency-stop")
            print("Enable Autonomous:  python3 serial_nav_control.py --autonomous on")
            print("Disable Autonomous: python3 serial_nav_control.py --autonomous off")
            print("Show Status:        python3 serial_nav_control.py --status")
            print("Monitor Status:     python3 serial_nav_control.py --monitor")
            print("Manual Forward:     python3 serial_nav_control.py --move forward --speed 0.5")
            print("Manual Rotate:      python3 serial_nav_control.py --rotate left --speed 0.3")
            print("Stop Movement:      python3 serial_nav_control.py --stop")
            print("")
            print("ROS 2 Commands:")
            print("Emergency Stop:     ros2 service call /emergency_stop std_srvs/srv/Trigger")
            print("Autonomous Mode:    ros2 service call /set_autonomous_mode std_srvs/srv/SetBool \"data: true\"")
            print("Monitor Status:     ros2 topic echo /motor_controller/status")
            print("Monitor Navigation: ros2 topic echo /navigation/state")
    
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

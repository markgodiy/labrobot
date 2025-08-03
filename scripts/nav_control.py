#!/usr/bin/env python3
"""
Autonomous Navigation Control Utility
=====================================

This utility provides easy control over the autonomous navigation system:
- Start/stop autonomous mode
- Emergency stop
- Monitor status
- Manual movement commands

Usage:
  # Enable autonomous mode
  python3 nav_control.py --autonomous on
  
  # Disable autonomous mode  
  python3 nav_control.py --autonomous off
  
  # Emergency stop
  python3 nav_control.py --emergency-stop
  
  # Monitor status
  python3 nav_control.py --status
  
  # Manual control (when autonomous is off)
  python3 nav_control.py --move forward --speed 50
  python3 nav_control.py --rotate left --speed 40
  python3 nav_control.py --stop
"""

import rclpy
from rclpy.node import Node
import requests
import json
import argparse
import time
import sys
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Service clients
        self.emergency_stop_client = self.create_client(Trigger, '/emergency_stop')
        self.autonomous_mode_client = self.create_client(SetBool, '/set_autonomous_mode')
        
        # Status subscriber
        self.status_subscriber = self.create_subscription(
            String,
            '/motor_controller/status',
            self.status_callback,
            10
        )
        
        self.latest_status = None
        
        # Direct MicroPython communication (fallback)
        self.micropython_ip = '192.168.25.72'
        self.micropython_port = 8080
        self.micropython_url = f"http://{self.micropython_ip}:{self.micropython_port}"
    
    def status_callback(self, msg):
        """Receive status updates"""
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_status = {'error': 'Invalid status format'}
    
    def send_direct_command(self, endpoint, params=None):
        """Send direct command to MicroPython controller"""
        try:
            url = f"{self.micropython_url}{endpoint}"
            if params:
                url += "?" + "&".join([f"{k}={v}" for k, v in params.items()])
            
            response = requests.get(url, timeout=2.0)
            if response.status_code == 200:
                return response.json()
            else:
                return {'status': 'error', 'message': f'HTTP {response.status_code}'}
        except requests.exceptions.RequestException as e:
            return {'status': 'error', 'message': str(e)}
    
    def emergency_stop(self):
        """Trigger emergency stop"""
        print("🚨 EMERGENCY STOP ACTIVATED 🚨")
        
        # Try ROS 2 service first
        if self.emergency_stop_client.wait_for_service(timeout_sec=2.0):
            request = Trigger.Request()
            future = self.emergency_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    print(f"✅ ROS 2 Emergency Stop: {result.message}")
                    return True
                else:
                    print(f"❌ ROS 2 Emergency Stop Failed: {result.message}")
        
        # Fallback to direct command
        response = self.send_direct_command('/estop')
        if response['status'] == 'ok':
            print(f"✅ Direct Emergency Stop: {response['message']}")
            return True
        else:
            print(f"❌ Direct Emergency Stop Failed: {response['message']}")
            return False
    
    def set_autonomous_mode(self, enabled):
        """Enable or disable autonomous mode"""
        mode_str = "ENABLED" if enabled else "DISABLED"
        print(f"🤖 Setting autonomous mode: {mode_str}")
        
        # Try ROS 2 service first
        if self.autonomous_mode_client.wait_for_service(timeout_sec=2.0):
            request = SetBool.Request()
            request.data = enabled
            future = self.autonomous_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                result = future.result()
                if result.success:
                    print(f"✅ ROS 2 Autonomous Mode: {result.message}")
                    return True
                else:
                    print(f"❌ ROS 2 Autonomous Mode Failed: {result.message}")
        
        # Fallback to direct command
        response = self.send_direct_command('/autonomous', {'mode': 'on' if enabled else 'off'})
        if response['status'] == 'ok':
            print(f"✅ Direct Autonomous Mode: {response['message']}")
            return True
        else:
            print(f"❌ Direct Autonomous Mode Failed: {response['message']}")
            return False
    
    def manual_move(self, direction, speed=50, duration=0):
        """Send manual movement command"""
        print(f"🎮 Manual move: {direction} at {speed}% speed")
        
        if direction in ['forward', 'backward']:
            response = self.send_direct_command('/move', {
                'dir': direction,
                'speed': speed,
                'duration': duration
            })
        elif direction in ['left', 'right']:
            response = self.send_direct_command('/rotate', {
                'dir': direction,
                'speed': speed,
                'duration': duration
            })
        else:
            print(f"❌ Invalid direction: {direction}")
            return False
        
        if response['status'] == 'ok':
            print(f"✅ Movement command: {response['message']}")
            return True
        else:
            print(f"❌ Movement command failed: {response['message']}")
            return False
    
    def stop_movement(self):
        """Stop all movement"""
        print("⏹️  Stopping movement")
        
        response = self.send_direct_command('/stop')
        if response['status'] == 'ok':
            print(f"✅ Stop command: {response['message']}")
            return True
        else:
            print(f"❌ Stop command failed: {response['message']}")
            return False
    
    def get_status(self, monitor=False):
        """Get and display system status"""
        if monitor:
            print("📊 Monitoring status (Press Ctrl+C to stop)...")
            try:
                while True:
                    self.display_status()
                    time.sleep(1.0)
            except KeyboardInterrupt:
                print("\n👋 Monitoring stopped")
        else:
            self.display_status()
    
    def display_status(self):
        """Display current status"""
        # Get direct controller status
        controller_status = self.send_direct_command('/status')
        
        print("\n" + "="*50)
        print("🤖 AUTONOMOUS NAVIGATION STATUS")
        print("="*50)
        
        if controller_status and controller_status['status'] == 'ok':
            data = controller_status['data']
            print(f"Controller Connection: ✅ CONNECTED")
            print(f"Autonomous Mode: {'🟢 ENABLED' if data.get('autonomous_mode') else '🔴 DISABLED'}")
            print(f"Movement Status: {'🏃 MOVING' if data.get('is_moving') else '⏸️  STOPPED'}")
            print(f"Emergency Stop: {'🚨 ACTIVE' if data.get('emergency_stop') else '✅ NORMAL'}")
            print(f"Current Speed: {data.get('current_speed', 0)}")
            print(f"Last Command Age: {data.get('last_command_age_ms', 0)}ms")
        else:
            print(f"Controller Connection: ❌ DISCONNECTED")
        
        # Display ROS 2 status if available
        if self.latest_status:
            if 'error' not in self.latest_status:
                print(f"\nROS 2 Navigation:")
                print(f"  Current Action: {self.latest_status.get('current_action', 'unknown')}")
                print(f"  Obstacle Detected: {'🚫' if self.latest_status.get('obstacle_detected') else '✅'}")
                print(f"  Path Clear: {'✅' if self.latest_status.get('path_clear') else '🚫'}")
            else:
                print(f"\nROS 2 Status: ❌ {self.latest_status['error']}")
        
        print("="*50)

def main():
    parser = argparse.ArgumentParser(description='Autonomous Navigation Control')
    parser.add_argument('--autonomous', choices=['on', 'off'], help='Enable/disable autonomous mode')
    parser.add_argument('--emergency-stop', action='store_true', help='Trigger emergency stop')
    parser.add_argument('--status', action='store_true', help='Show status')
    parser.add_argument('--monitor', action='store_true', help='Monitor status continuously')
    parser.add_argument('--move', choices=['forward', 'backward'], help='Manual movement')
    parser.add_argument('--rotate', choices=['left', 'right'], help='Manual rotation')
    parser.add_argument('--stop', action='store_true', help='Stop movement')
    parser.add_argument('--speed', type=int, default=50, help='Movement speed (0-100)')
    parser.add_argument('--duration', type=float, default=0, help='Movement duration in seconds (0=continuous)')
    
    args = parser.parse_args()
    
    # Initialize ROS 2
    rclpy.init()
    controller = NavigationController()
    
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
            print("\n" + "="*50)
            print("🤖 QUICK COMMANDS:")
            print("="*50)
            print("Emergency Stop:     python3 nav_control.py --emergency-stop")
            print("Enable Autonomous:  python3 nav_control.py --autonomous on")
            print("Disable Autonomous: python3 nav_control.py --autonomous off")
            print("Show Status:        python3 nav_control.py --status")
            print("Monitor Status:     python3 nav_control.py --monitor")
            print("Manual Forward:     python3 nav_control.py --move forward --speed 50")
            print("Manual Rotate:      python3 nav_control.py --rotate left --speed 40")
            print("Stop Movement:      python3 nav_control.py --stop")
    
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

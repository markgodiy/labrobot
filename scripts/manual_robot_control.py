#!/usr/bin/env python3
"""
Manual Robot Movement Commands for Testing
==========================================

Direct serial control commands for testing robot movement.
Sends commands directly to the MicroPython controller.

Usage:
    python3 manual_robot_control.py [command] [options]

Commands:
    forward [speed] [duration]  - Move forward (speed: 0-100, duration in seconds)
    backward [speed] [duration] - Move backward  
    left [speed] [duration]     - Rotate left
    right [speed] [duration]    - Rotate right
    stop                        - Stop immediately
    reset-estop                 - Reset emergency stop
    status                      - Show robot status
    test-sequence              - Run a test movement sequence
    
Examples:
    python3 manual_robot_control.py forward 30 2    # Forward at 30% speed for 2 seconds
    python3 manual_robot_control.py left 25 1       # Rotate left at 25% speed for 1 second
    python3 manual_robot_control.py stop            # Stop now
    python3 manual_robot_control.py test-sequence   # Run test pattern
"""

import serial
import json
import sys
import time

class ManualRobotControl:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """Initialize serial connection to MicroPython controller"""
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=2.0)
            time.sleep(2)  # Wait for connection to stabilize
            print(f"✅ Connected to robot on {port}")
        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            sys.exit(1)
    
    def send_command(self, command):
        """Send JSON command to MicroPython controller"""
        try:
            command_json = json.dumps(command) + '\n'
            self.serial_port.write(command_json.encode())
            
            # Wait for response
            response_line = self.serial_port.readline().decode().strip()
            if response_line:
                response = json.loads(response_line)
                return response
            else:
                return {"success": False, "message": "No response"}
                
        except Exception as e:
            return {"success": False, "message": f"Communication error: {e}"}
    
    def move_forward(self, speed=50, duration=2.0):
        """Move robot forward"""
        command = {
            "cmd": "move",
            "dir": "forward",
            "speed": int(speed),
            "duration": int(duration * 1000)  # Convert to milliseconds
        }
        
        print(f"🚀 Moving forward: {speed}% speed for {duration}s")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Move command successful")
        else:
            print(f"❌ Move failed: {response.get('message', 'Unknown error')}")
    
    def move_backward(self, speed=50, duration=2.0):
        """Move robot backward"""
        command = {
            "cmd": "move", 
            "dir": "backward",
            "speed": int(speed),
            "duration": int(duration * 1000)
        }
        
        print(f"⬅️ Moving backward: {speed}% speed for {duration}s")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Move command successful")
        else:
            print(f"❌ Move failed: {response.get('message', 'Unknown error')}")
    
    def rotate_left(self, speed=40, duration=1.0):
        """Rotate robot left"""
        command = {
            "cmd": "rotate",
            "dir": "left", 
            "speed": int(speed),
            "duration": int(duration * 1000)
        }
        
        print(f"↪️ Rotating left: {speed}% speed for {duration}s")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Rotation command successful")
        else:
            print(f"❌ Rotation failed: {response.get('message', 'Unknown error')}")
    
    def rotate_right(self, speed=40, duration=1.0):
        """Rotate robot right"""
        command = {
            "cmd": "rotate",
            "dir": "right",
            "speed": int(speed), 
            "duration": int(duration * 1000)
        }
        
        print(f"↩️ Rotating right: {speed}% speed for {duration}s")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Rotation command successful")
        else:
            print(f"❌ Rotation failed: {response.get('message', 'Unknown error')}")
    
    def stop_robot(self):
        """Stop robot immediately"""
        command = {"cmd": "stop"}
        
        print("🛑 Stopping robot")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Robot stopped")
        else:
            print(f"❌ Stop failed: {response.get('message', 'Unknown error')}")
    
    def reset_emergency_stop(self):
        """Reset emergency stop"""
        command = {"cmd": "reset_estop"}
        
        print("🔄 Resetting emergency stop...")
        response = self.send_command(command)
        
        if response.get("success", False):
            print("✅ Emergency stop reset")
        else:
            print(f"❌ Reset failed: {response.get('message', 'Unknown error')}")
    
    def get_status(self):
        """Get robot status"""
        command = {"cmd": "status"}
        
        print("📊 Getting robot status...")
        response = self.send_command(command)
        
        if response.get("type") == "status":
            print(f"✅ Robot Status:")
            print(f"   Health: {response.get('health_status', 'unknown')}")
            print(f"   Moving: {response.get('is_moving', 'unknown')}")
            print(f"   Speed: {response.get('current_speed', 'unknown')}")
            print(f"   Emergency Stop: {response.get('emergency_stop', 'unknown')}")
            print(f"   Autonomous: {response.get('autonomous_mode', 'unknown')}")
            print(f"   Uptime: {response.get('uptime_ms', 0) / 1000:.1f}s")
        else:
            print(f"❌ Status failed: {response.get('message', 'Unknown error')}")
    
    def test_sequence(self):
        """Run a test movement sequence"""
        print("🧪 Starting test movement sequence...")
        
        # Forward
        self.move_forward(75, 2)
        time.sleep(2.5)
        
        # Turn left  
        self.rotate_left(75, 1)
        time.sleep(1.5)
        
        # Forward again
        self.move_forward(75, 1.5)
        time.sleep(2)
        
        # Turn right
        self.rotate_right(75, 1)
        time.sleep(1.5)
        
        # Backward
        self.move_backward(75, 1)
        time.sleep(1.5)
        
        # Stop
        self.stop_robot()
        
        print("✅ Test sequence completed!")
    
    def close(self):
        """Close serial connection"""
        if self.serial_port:
            self.serial_port.close()

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    
    command = sys.argv[1].lower()
    
    try:
        controller = ManualRobotControl()
        
        if command == 'forward':
            speed = int(sys.argv[2]) if len(sys.argv) > 2 else 75  # Default 75% minimum
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
            controller.move_forward(speed, duration)
            
        elif command == 'backward':
            speed = int(sys.argv[2]) if len(sys.argv) > 2 else 75  # Default 75% minimum
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
            controller.move_backward(speed, duration)
            
        elif command == 'left':
            speed = int(sys.argv[2]) if len(sys.argv) > 2 else 75  # Default 75% minimum
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.rotate_left(speed, duration)
            
        elif command == 'right':
            speed = int(sys.argv[2]) if len(sys.argv) > 2 else 75  # Default 75% minimum
            duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
            controller.rotate_right(speed, duration)
            
        elif command == 'stop':
            controller.stop_robot()
            
        elif command == 'reset-estop':
            controller.reset_emergency_stop()
            
        elif command == 'status':
            controller.get_status()
            
        elif command == 'test-sequence':
            controller.test_sequence()
            
        else:
            print(f"❌ Unknown command: {command}")
            print(__doc__)
    
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == '__main__':
    main()

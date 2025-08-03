#!/usr/bin/env python3
"""
Serial Motor Controller Bridge for MicroPython
==============================================

This node provides a bridge between ROS 2 and the MicroPython motor controller
via USB serial communication. It translates ROS 2 service calls and topics
into JSON commands sent over serial.

Features:
- Serial communication with MicroPython controller
- ROS 2 service interfaces for motor control
- Status monitoring and publishing
- Error handling and reconnection
- Thread-safe serial communication

Services:
- /motor/move (custom service): Move robot
- /motor/rotate (custom service): Rotate robot  
- /motor/stop (std_srvs/Trigger): Stop movement
- /motor/emergency_stop (std_srvs/Trigger): Emergency stop
- /motor/set_autonomous_mode (std_srvs/SetBool): Set autonomous mode

Topics:
- /motor_controller/status (std_msgs/String): Controller status
- /motor_controller/logs (std_msgs/String): Controller log messages
"""

import rclpy
from rclpy.node import Node
import serial
import json
import threading
import time
from threading import Lock
# Standard ROS 2 service and message imports
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialMotorBridge(Node):
    def __init__(self):
        super().__init__('serial_motor_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # Serial connection
        self.ser = None
        self.serial_lock = Lock()
        self.connected = False
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/motor_controller/status', 10)
        self.log_publisher = self.create_publisher(String, '/motor_controller/logs', 10)
        
        # Services
        self.stop_service = self.create_service(Trigger, '/motor/stop', self.handle_stop)
        self.emergency_stop_service = self.create_service(Trigger, '/motor/emergency_stop', self.handle_emergency_stop)
        self.autonomous_mode_service = self.create_service(SetBool, '/motor/set_autonomous_mode', self.handle_autonomous_mode)
        
        # Twist subscriber for direct movement commands
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.handle_twist,
            10
        )
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.request_status)
        
        # Initialize serial connection
        self.connect_serial()
        
        # Start background thread for reading serial data
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info(f"Serial Motor Bridge started on {self.serial_port} at {self.baudrate}")
    
    def connect_serial(self):
        """Connect to serial port"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                time.sleep(0.5)  # Wait for port to be fully released
            
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                exclusive=True  # Prevent multiple access
            )
            
            # Clear buffers to prevent garbled data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            self.connected = True
            self.get_logger().info(f"Connected to {self.serial_port}")
            
            # Give MicroPython more time to start and settle
            time.sleep(2.0)
            
            # Clear any startup messages
            self.ser.reset_input_buffer()
            
            # Send initial ping to verify connection
            response = self.send_command({"cmd": "status"}, timeout=3.0)
            if response and response.get("status") == "ok":
                self.get_logger().info("MicroPython controller responded successfully")
            else:
                self.get_logger().warn("No valid response from MicroPython controller")
                
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
    
    def send_command(self, cmd_dict, timeout=None):
        """Send JSON command to MicroPython controller and wait for response"""
        if not self.connected or not self.ser or not self.ser.is_open:
            self.get_logger().error("Serial port not connected")
            return None
        
        try:
            with self.serial_lock:
                # Clear input buffer before sending command
                self.ser.reset_input_buffer()
                
                # Send command
                cmd_json = json.dumps(cmd_dict) + '\n'
                self.ser.write(cmd_json.encode('utf-8'))
                self.ser.flush()
                
                # Wait for response with timeout
                start_time = time.time()
                response_timeout = timeout or self.timeout
                response_data = ""
                
                while time.time() - start_time < response_timeout:
                    if self.ser.in_waiting > 0:
                        # Read available data
                        data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        response_data += data
                        
                        # Look for complete JSON lines
                        lines = response_data.split('\n')
                        for line in lines[:-1]:  # Process all complete lines
                            line = line.strip()
                            if line and line.startswith('{') and line.endswith('}'):
                                try:
                                    response = json.loads(line)
                                    return response
                                except json.JSONDecodeError as e:
                                    self.get_logger().warn(f"JSON decode error: {e} for line: {line}")
                                    continue
                        
                        # Keep the last incomplete line
                        response_data = lines[-1]
                    else:
                        time.sleep(0.05)  # Longer delay to reduce CPU usage
                
                self.get_logger().warn(f"Timeout waiting for response to command: {cmd_dict}")
                if response_data.strip():
                    self.get_logger().warn(f"Partial response received: {response_data[:100]}...")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.connected = False
            return None
    
    def send_command_async(self, cmd_dict):
        """Send command without waiting for response (fire and forget)"""
        if not self.connected or not self.ser or not self.ser.is_open:
            return False
        
        try:
            with self.serial_lock:
                cmd_json = json.dumps(cmd_dict) + '\n'
                self.ser.write(cmd_json.encode('utf-8'))
                self.ser.flush()
                return True
        except Exception as e:
            self.get_logger().error(f"Serial send error: {e}")
            self.connected = False
            return False
    
    def read_serial_loop(self):
        """Background thread to read serial data"""
        response_buffer = ""
        
        while rclpy.ok():
            if not self.connected:
                time.sleep(self.reconnect_interval)
                self.connect_serial()
                continue
            
            try:
                if self.ser and self.ser.is_open:
                    # Check for available data with timeout
                    if self.ser.in_waiting > 0:
                        # Read available data in chunks
                        data = self.ser.read(min(self.ser.in_waiting, 1024)).decode('utf-8', errors='ignore')
                        response_buffer += data
                        
                        # Process complete lines
                        while '\n' in response_buffer:
                            line, response_buffer = response_buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                # Try to parse as JSON
                                if line.startswith('{') and line.endswith('}'):
                                    try:
                                        msg = json.loads(line)
                                        self.process_incoming_message(msg)
                                    except json.JSONDecodeError as e:
                                        # Log partial JSON for debugging
                                        if len(line) > 50:
                                            self.get_logger().warn(f"JSON decode error: {e} for line: {line[:50]}...")
                                        else:
                                            self.get_logger().warn(f"JSON decode error: {e} for line: {line}")
                                        self.publish_log(f"Raw data: {line}")
                                else:
                                    # Plain text message
                                    self.publish_log(line)
                    else:
                        # No data available, small sleep
                        time.sleep(0.1)
                else:
                    time.sleep(1.0)
                    
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.connected = False
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f"Unexpected error in read loop: {e}")
                time.sleep(1.0)
    
    def process_incoming_message(self, msg):
        """Process incoming JSON message from MicroPython"""
        msg_type = msg.get("type", "response")
        
        if msg_type == "status":
            # Publish status
            status_msg = String()
            status_msg.data = json.dumps(msg)
            self.status_publisher.publish(status_msg)
            
        elif msg_type == "log":
            # Publish log message
            self.publish_log(f"[{msg.get('level', 'INFO')}] {msg.get('message', '')}")
    
    def publish_log(self, message):
        """Publish log message"""
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
    
    def request_status(self):
        """Request status from controller"""
        if self.connected:
            self.send_command_async({"cmd": "status"})
    
    # Service handlers
    def handle_stop(self, request, response):
        """Handle stop service call"""
        result = self.send_command({"cmd": "stop"})
        
        if result and result.get("status") == "ok":
            response.success = True
            response.message = result.get("message", "Stopped")
            self.get_logger().info("Motor stop command sent")
        else:
            response.success = False
            response.message = "Failed to send stop command"
            self.get_logger().error("Failed to send stop command")
        
        return response
    
    def handle_emergency_stop(self, request, response):
        """Handle emergency stop service call"""
        result = self.send_command({"cmd": "estop"})
        
        if result and result.get("status") == "ok":
            response.success = True
            response.message = result.get("message", "Emergency stop activated")
            self.get_logger().warn("EMERGENCY STOP ACTIVATED")
        else:
            response.success = False
            response.message = "Failed to activate emergency stop"
            self.get_logger().error("Failed to activate emergency stop")
        
        return response
    
    def handle_autonomous_mode(self, request, response):
        """Handle autonomous mode service call"""
        mode = "on" if request.data else "off"
        result = self.send_command({"cmd": "autonomous", "mode": mode})
        
        if result and result.get("status") == "ok":
            response.success = True
            response.message = f"Autonomous mode {'enabled' if request.data else 'disabled'}"
            self.get_logger().info(f"Autonomous mode {'ENABLED' if request.data else 'DISABLED'}")
        else:
            response.success = False
            response.message = "Failed to set autonomous mode"
            self.get_logger().error("Failed to set autonomous mode")
        
        return response
    
    def handle_twist(self, msg):
        """Handle Twist message for direct movement control"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert twist to motor commands
        if abs(linear_x) > 0.1:  # Forward/backward movement
            direction = "forward" if linear_x > 0 else "backward"
            speed = min(100, int(abs(linear_x) * 100))  # Scale to percentage
            self.send_command_async({
                "cmd": "move",
                "dir": direction,
                "speed": speed
            })
        elif abs(angular_z) > 0.1:  # Rotation
            direction = "left" if angular_z > 0 else "right"
            speed = min(100, int(abs(angular_z) * 50))  # Scale to percentage
            self.send_command_async({
                "cmd": "rotate",
                "dir": direction,
                "speed": speed
            })
        else:
            # Stop if no significant movement
            self.send_command_async({"cmd": "stop"})
    
    # Public methods for other nodes to use
    def move_robot(self, direction, speed, duration=0):
        """Move robot in specified direction"""
        return self.send_command({
            "cmd": "move",
            "dir": direction,
            "speed": speed,
            "duration": duration
        })
    
    def rotate_robot(self, direction, speed, duration=0):
        """Rotate robot in specified direction"""
        return self.send_command({
            "cmd": "rotate",
            "dir": direction,
            "speed": speed,
            "duration": duration
        })
    
    def stop_robot(self):
        """Stop robot movement"""
        return self.send_command({"cmd": "stop"})
    
    def emergency_stop_robot(self):
        """Emergency stop robot"""
        return self.send_command({"cmd": "estop"})
    
    def set_autonomous_mode(self, enabled):
        """Set autonomous mode"""
        mode = "on" if enabled else "off"
        return self.send_command({"cmd": "autonomous", "mode": mode})
    
    def get_controller_status(self):
        """Get controller status"""
        return self.send_command({"cmd": "status"})

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SerialMotorBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down serial motor bridge...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            # Send emergency stop before shutdown
            try:
                if node.connected and node.ser and node.ser.is_open:
                    node.emergency_stop_robot()
                    time.sleep(0.5)  # Give time for command to be sent
                    node.ser.close()
            except Exception as e:
                print(f"Error during emergency stop: {e}")
            
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Error destroying node: {e}")
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during RCL shutdown: {e}")

if __name__ == '__main__':
    main()

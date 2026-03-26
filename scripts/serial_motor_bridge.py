#!/usr/bin/env python3
"""
Serial Motor Controller Bridge for MicroPython
==============================================

This node provides a bridge between ROS 2 and the MicroPython motor controller
via USB serial communication. It translates ROS 2 service calls and topics
into JSON commands sent over serial, and publishes encoder/odometry data.

Features:
- Serial communication with MicroPython controller
- ROS 2 service interfaces for motor control
- Encoder data and odometry publishing
- Status monitoring and publishing
- Error handling and reconnection
- Thread-safe serial communication

Services:
- /motor/move (custom service): Move robot
- /motor/rotate (custom service): Rotate robot  
- /motor/stop (std_srvs/Trigger): Stop movement
- /motor/emergency_stop (std_srvs/Trigger): Emergency stop
- /motor/set_autonomous_mode (std_srvs/SetBool): Set autonomous mode
- /motor/reset_emergency_stop (std_srvs/Trigger): Reset emergency stop

Topics:
- /motor_controller/status (std_msgs/String): Controller status
- /motor_controller/logs (std_msgs/String): Controller log messages
- /motor_controller/encoder_data (std_msgs/String): Raw encoder data
- /odom (nav_msgs/Odometry): Robot odometry from encoders
"""

import rclpy
from rclpy.node import Node
import serial
import json
import threading
import time
import math
from threading import Lock
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SerialMotorBridge(Node):
    def __init__(self):
        super().__init__('serial_motor_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('wheel_base_m', 0.347)      # center-to-center from URDF
        self.declare_parameter('wheel_radius_m', 0.0325)   # 65mm diameter / 2
        self.declare_parameter('ticks_per_rev', 3436)
        self.declare_parameter('encoder_poll_hz', 20.0)    # encoder query rate
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_min_range', 0.25)
        self.declare_parameter('min_obstacle_distance', 0.55)
        self.declare_parameter('scan_angle_deg', 90.0)
        self.declare_parameter('scan_stale_sec', 1.0)

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.wheel_base_m = self.get_parameter('wheel_base_m').value
        self.wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        encoder_poll_hz = self.get_parameter('encoder_poll_hz').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius_m) / self.ticks_per_rev
        self.lidar_min_range = self.get_parameter('lidar_min_range').value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.scan_angle_deg = self.get_parameter('scan_angle_deg').value
        self.scan_stale_sec = self.get_parameter('scan_stale_sec').value
        
        # LIDAR safety gate state
        self.path_clear = False   # blocked until first scan received
        self.rear_clear = False   # blocked until first scan received
        self.last_scan_time = None
        self._last_lidar_warn_time = 0.0

        # Serial connection
        self.ser = None
        self.serial_lock = Lock()
        self.connected = False
        
        # Publishers
        self.status_publisher = self.create_publisher(String, '/motor_controller/status', 10)
        self.log_publisher = self.create_publisher(String, '/motor_controller/logs', 10)
        self.encoder_data_publisher = self.create_publisher(String, '/motor_controller/encoder_data', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster for odom -> base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state (integrated from encoder counts)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_left_count = None   # None until first encoder reading
        self.last_right_count = None
        self.last_odom_time = None
        
        # Services
        self.stop_service = self.create_service(Trigger, '/motor/stop', self.handle_stop)
        self.emergency_stop_service = self.create_service(Trigger, '/motor/emergency_stop', self.handle_emergency_stop)
        self.reset_emergency_stop_service = self.create_service(Trigger, '/motor/reset_emergency_stop', self.handle_reset_emergency_stop)
        self.autonomous_mode_service = self.create_service(SetBool, '/motor/set_autonomous_mode', self.handle_autonomous_mode)
        
        # Twist subscriber for direct movement commands
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.handle_twist,
            10
        )
        
        # LIDAR safety — gate all forward motion on /scan
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self._scan_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        )

        # cmd_vel watchdog — stop motors if no cmd_vel arrives for 5s while moving
        self.last_cmd_vel_time = None  # None = not moving via cmd_vel
        self.cmd_vel_watchdog_timeout = 5.0

        # Timers
        self.status_timer = self.create_timer(1.0, self.request_status)
        self.encoder_timer = self.create_timer(1.0 / encoder_poll_hz, self.poll_encoders)
        self.cmd_vel_watchdog_timer = self.create_timer(1.0, self.check_cmd_vel_watchdog)
        
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
            
            # Give MicroPython time to settle before marking as connected
            time.sleep(2.0)
            
            # Clear startup messages that arrived during the wait
            self.ser.reset_input_buffer()
            
            # Verify connection with a ping before accepting service calls
            self.connected = True
            self.get_logger().info(f"Connected to {self.serial_port}")
            response = self.send_command({"cmd": "ping"}, timeout=3.0)
            if response and response.get("status") == "ok":
                self.get_logger().info("MicroPython controller responded successfully")
                # Always engage estop on bridge startup — Pi reboot must not resume motion
                self.send_command({"cmd": "estop"}, timeout=2.0)
                self.get_logger().warn("Startup estop sent — call /motor/reset_emergency_stop to enable motors")
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
                                    # Skip async push messages — keep reading for the command response
                                    if "type" in response:
                                        continue
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
                    if not self.serial_lock.acquire(blocking=False):
                        time.sleep(0.01)
                        continue
                    try:
                        if self.ser.in_waiting > 0:
                            data = self.ser.read(min(self.ser.in_waiting, 1024)).decode('utf-8', errors='ignore')
                            response_buffer += data
                            while '\n' in response_buffer:
                                line, response_buffer = response_buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    if line.startswith('{') and line.endswith('}'):
                                        try:
                                            msg = json.loads(line)
                                            self.process_incoming_message(msg)
                                        except json.JSONDecodeError as e:
                                            if len(line) > 50:
                                                self.get_logger().warn(f"JSON decode error: {e} for line: {line[:50]}...")
                                            else:
                                                self.get_logger().warn(f"JSON decode error: {e} for line: {line}")
                                            self.publish_log(f"Raw data: {line}")
                                    else:
                                        self.publish_log(line)
                        else:
                            time.sleep(0.1)
                    finally:
                        self.serial_lock.release()
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

        # Extract encoder data wherever it appears
        encoder_data = msg.get("encoder_data") or msg.get("encoder")
        if encoder_data and "left_count" in encoder_data:
            self._update_odometry(encoder_data)
            enc_msg = String()
            enc_msg.data = json.dumps(encoder_data)
            self.encoder_data_publisher.publish(enc_msg)

        if msg_type in ("status", "heartbeat"):
            status_msg = String()
            status_msg.data = json.dumps(msg)
            self.status_publisher.publish(status_msg)

        elif msg_type == "log":
            self.publish_log(f"[{msg.get('level', 'INFO')}] {msg.get('message', '')}")

    def _update_odometry(self, encoder_data):
        """Compute differential drive odometry from cumulative encoder counts."""
        left_count = encoder_data["left_count"]
        right_count = encoder_data["right_count"]

        if self.last_left_count is None:
            self.last_left_count = left_count
            self.last_right_count = right_count
            return

        delta_left  =  (left_count  - self.last_left_count)  * self.m_per_tick
        # Right motor is physically inverted (firmware-backward = physical-forward),
        # so negate the raw encoder delta to get physical wheel displacement.
        delta_right = -(right_count - self.last_right_count) * self.m_per_tick
        self.last_left_count  = left_count
        self.last_right_count = right_count

        linear  = (delta_left + delta_right) / 2.0
        # Angular uses (left - right) rather than (right - left) because the right
        # motor inversion flips which side is "outer" during a turn.
        angular = (delta_left - delta_right) / self.wheel_base_m

        self.odom_theta += angular
        self.odom_x += linear * math.cos(self.odom_theta)
        self.odom_y += linear * math.sin(self.odom_theta)

        ros_now = self.get_clock().now()
        now_msg = ros_now.to_msg()
        dt = (ros_now.nanoseconds - self.last_odom_time.nanoseconds) * 1e-9 \
             if self.last_odom_time is not None else 0.05
        self.last_odom_time = ros_now
        vx  = linear  / dt if dt > 0 else 0.0
        wz  = angular / dt if dt > 0 else 0.0

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz
        # Non-zero covariance required by AMCL for sensor fusion weighting.
        # Row-major 6×6 diagonal; indices: 0=x, 7=y, 14=z, 21=roll, 28=pitch, 35=yaw
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw
        odom.twist.covariance[0]  = 0.01  # vx
        odom.twist.covariance[35] = 0.05  # wz
        self.odom_publisher.publish(odom)

        # Broadcast TF: odom -> base_footprint
        tf = TransformStamped()
        tf.header.stamp = now_msg
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.odom_x
        tf.transform.translation.y = self.odom_y
        tf.transform.rotation.z = math.sin(self.odom_theta / 2.0)
        tf.transform.rotation.w = math.cos(self.odom_theta / 2.0)
        self.tf_broadcaster.sendTransform(tf)
    
    def publish_log(self, message):
        """Publish log message"""
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
    
    def request_status(self):
        """Request status from controller"""
        if self.connected:
            self.send_command_async({"cmd": "status"})

    def poll_encoders(self):
        """Poll encoder counts at the configured rate for odometry."""
        if self.connected:
            self.send_command_async({"cmd": "get_encoders"})
    
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
    
    def handle_reset_emergency_stop(self, request, response):
        """Handle reset emergency stop service call"""
        result = self.send_command({"cmd": "reset_estop"})
        if result and result.get("status") == "ok":
            response.success = True
            response.message = result.get("message", "Emergency stop reset")
            self.get_logger().info("Emergency stop reset")
        else:
            response.success = False
            response.message = "Failed to reset emergency stop"
            self.get_logger().error("Failed to reset emergency stop")
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
    
    def check_cmd_vel_watchdog(self):
        """Stop motors if cmd_vel has gone silent while robot was moving"""
        if self.last_cmd_vel_time is None:
            return
        if time.time() - self.last_cmd_vel_time > self.cmd_vel_watchdog_timeout:
            self.get_logger().warn("cmd_vel watchdog: no command for 5s, stopping motors")
            self.send_command_async({"cmd": "stop"})
            self.last_cmd_vel_time = None

    def _scan_callback(self, msg):
        """Update path_clear and rear_clear from LIDAR sectors."""
        self.last_scan_time = time.time()
        n = len(msg.ranges)
        if n == 0:
            return
        half_idx = int(math.radians(self.scan_angle_deg / 2.0) / msg.angle_increment)

        # Front sector: last half_idx indices + first half_idx indices (0° is forward)
        front_indices = list(range(max(0, n - half_idx), n)) + list(range(0, min(half_idx + 1, n)))
        front_valid = [msg.ranges[i] for i in front_indices
                       if self.lidar_min_range < msg.ranges[i] < msg.range_max]
        front_obstacle = bool(front_valid) and min(front_valid) < self.min_obstacle_distance
        if front_obstacle and self.path_clear:
            self.get_logger().warn(
                f"LIDAR: front obstacle at {min(front_valid):.2f}m — forward motion blocked"
            )
        self.path_clear = not front_obstacle

        # Rear sector: centred on π (index n//2), same angular width
        mid = n // 2
        rear_indices = list(range(max(0, mid - half_idx), min(n, mid + half_idx + 1)))
        rear_valid = [msg.ranges[i] for i in rear_indices
                      if self.lidar_min_range < msg.ranges[i] < msg.range_max]
        rear_obstacle = bool(rear_valid) and min(rear_valid) < self.min_obstacle_distance
        if rear_obstacle and self.rear_clear:
            self.get_logger().warn(
                f"LIDAR: rear obstacle at {min(rear_valid):.2f}m — backward motion blocked"
            )
        self.rear_clear = not rear_obstacle

    def handle_twist(self, msg):
        """Handle Twist message for direct movement control.

        Nav2 (DWB controller) outputs linear.x up to ~0.22 m/s and angular.z up
        to ~1.0 rad/s.  The motors need at least 85% PWM duty to move reliably,
        so we map the Nav2 full-scale output to 85–100% motor power rather than
        the raw percentage which would be 22% (below the movement threshold).
        """
        # --- LIDAR safety gate ---
        now = time.time()
        scan_stale = (self.last_scan_time is None or
                      now - self.last_scan_time > self.scan_stale_sec)
        if scan_stale:
            self.send_command_async({"cmd": "stop"})
            self.last_cmd_vel_time = None
            if now - self._last_lidar_warn_time > 5.0:
                self.get_logger().warn("LIDAR: no scan received — blocking all motion")
                self._last_lidar_warn_time = now
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        MAX_LINEAR_MS  = 0.22   # m/s — matches nav2_params max_vel_x
        MAX_ANGULAR_RS = 1.0    # rad/s — matches nav2_params max_vel_theta
        MOTOR_MIN      = 85     # minimum effective PWM %
        MOTOR_MAX      = 100

        # Motor wiring: verified via encoder test (2026-03-25).
        # left_dir=+1 → left encoder increases (physically forward)
        # right_dir=+1 → right encoder increases (physically forward)
        # Pin direction mapping:
        #   forward  = left_dir=-1, right_dir=-1
        #   backward = left_dir=+1, right_dir=+1
        #   CCW turn = left_dir=-1, right_dir=+1  (left back, right fwd)
        #   CW turn  = left_dir=+1, right_dir=-1  (left fwd, right back)
        if linear_x > 0.02 and not self.path_clear:
            self.send_command_async({"cmd": "stop"})
            self.last_cmd_vel_time = None
            return
        if linear_x < -0.02 and not self.rear_clear:
            self.send_command_async({"cmd": "stop"})
            self.last_cmd_vel_time = None
            return

        has_linear  = abs(linear_x) > 0.02
        has_angular = abs(angular_z) > 0.05

        if has_linear and has_angular:
            # Arc motion: drive both wheels in the linear direction but stop
            # the inner wheel to curve.  Pico only supports a single speed
            # value, so true differential PWM isn't possible without firmware
            # changes.  Stopping the inner wheel produces a smooth wide arc.
            base_dir = -1 if linear_x > 0 else 1      # per verified mapping
            ratio = min(1.0, abs(linear_x) / MAX_LINEAR_MS)
            speed = int(MOTOR_MIN + ratio * (MOTOR_MAX - MOTOR_MIN))
            if angular_z > 0:
                # CCW arc: stop left wheel, drive right
                left_dir, right_dir = 0, base_dir
            else:
                # CW arc: drive left, stop right wheel
                left_dir, right_dir = base_dir, 0
            self.send_command_async({
                "cmd": "set_speed",
                "left_dir": left_dir, "right_dir": right_dir, "speed": speed
            })
            self.last_cmd_vel_time = time.time()
        elif has_linear:
            left_dir, right_dir = (-1, -1) if linear_x > 0 else (1, 1)
            ratio = min(1.0, abs(linear_x) / MAX_LINEAR_MS)
            speed = int(MOTOR_MIN + ratio * (MOTOR_MAX - MOTOR_MIN))
            self.send_command_async({
                "cmd": "set_speed",
                "left_dir": left_dir, "right_dir": right_dir, "speed": speed
            })
            self.last_cmd_vel_time = time.time()
        elif has_angular:
            # angular_z > 0 = CCW (left turn); < 0 = CW (right turn)
            left_dir, right_dir = (-1, 1) if angular_z > 0 else (1, -1)
            ratio = min(1.0, abs(angular_z) / MAX_ANGULAR_RS)
            speed = int(MOTOR_MIN + ratio * (MOTOR_MAX - MOTOR_MIN))
            self.send_command_async({
                "cmd": "set_speed",
                "left_dir": left_dir, "right_dir": right_dir, "speed": speed
            })
            self.last_cmd_vel_time = time.time()
        else:
            # Explicit stop — clear watchdog
            self.send_command_async({"cmd": "stop"})
            self.last_cmd_vel_time = None
    
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

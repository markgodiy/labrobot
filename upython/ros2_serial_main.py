"""
ROS 2 Compatible MicroPython Motor Controller - Serial Communication
===================================================================

This code runs on the MicroPython device (Pico W) and communicates with
the ROS 2 system via USB serial connection. HTTP server is optional for
testing and manual control only.

Communication Protocol (Serial):
- Baud rate: 115200
- JSON messages over serial
- Commands: {"cmd": "move", "dir": "forward", "speed": 50, "duration": 2}
- Responses: {"status": "ok", "message": "Moving forward"}

Hardware Setup:
- USB connection to Pi for serial communication
- WiFi optional (for testing/manual control only)
- Motor A (Left): ENA=Pin(2), IN1=Pin(3), IN2=Pin(4)
- Motor B (Right): ENB=Pin(8), IN3=Pin(6), IN4=Pin(7)
"""

from machine import Pin, PWM, Timer, UART
from time import sleep, sleep_ms, ticks_ms, ticks_diff
import json
import sys

# Try to import network modules (optional for testing)
try:
    import network
    import socket
    from wifi_config import SSID, PASSWORD
    WIFI_AVAILABLE = True
except ImportError:
    WIFI_AVAILABLE = False
    print("WiFi modules not available - Serial mode only")

# Hardware setup
led = Pin('LED', Pin.OUT)

# Startup LED sequence
for _ in range(3):
    led.toggle()
    sleep(0.2)
led.off()

# Motor setup
ena_pwm = PWM(Pin(2))   # Left motor enable
in1 = Pin(3, Pin.OUT)   # Left motor direction 1
in2 = Pin(4, Pin.OUT)   # Left motor direction 2
ena_pwm.freq(1000)

in3 = Pin(6, Pin.OUT)   # Right motor direction 1
in4 = Pin(7, Pin.OUT)   # Right motor direction 2
enb_pwm = PWM(Pin(8))   # Right motor enable
enb_pwm.freq(1000)

# Serial communication setup
# USB serial is automatically configured, using sys.stdin/stdout
serial_buffer = ""

# Navigation state
class NavigationState:
    def __init__(self):
        self.current_speed = 0
        self.target_speed = 0
        self.is_moving = False
        self.autonomous_mode = False
        self.emergency_stop_active = True  # Start in emergency stop for safety
        self.last_command_time = ticks_ms()  # Initialize with current time
        self.command_timeout_ms = 3000  # 3 second timeout for serial
        self.ramp_step = 3000
        self.ramp_delay_ms = 50
        self.wifi_enabled = False
        
        # Movement tracking
        self.move_start_time = 0
        self.move_duration = 0
        self.rotation_start_time = 0
        self.rotation_duration = 0
        
        # Monitoring and diagnostics
        self.total_commands_received = 0
        self.total_errors = 0
        self.last_error_message = ""
        self.last_error_time = 0
        self.heartbeat_interval_ms = 5000  # Send heartbeat every 5 seconds
        self.last_heartbeat_time = ticks_ms()
        self.serial_read_errors = 0
        self.json_parse_errors = 0
        
nav_state = NavigationState()

# Safety timer
safety_timer = Timer(-1)

def safety_check(timer):
    """Safety check - stop if no command received within timeout"""
    global nav_state
    current_time = ticks_ms()
    if nav_state.autonomous_mode and not nav_state.emergency_stop_active:
        if ticks_diff(current_time, nav_state.last_command_time) > nav_state.command_timeout_ms:
            print("SAFETY: Command timeout - emergency stop")
            emergency_stop()

# Start safety timer (checks every 1000ms)
safety_timer.init(period=1000, mode=Timer.PERIODIC, callback=safety_check)

def send_serial_response(response_dict):
    """Send JSON response over serial"""
    try:
        response_json = json.dumps(response_dict)
        print(response_json)  # This goes to USB serial
        sys.stdout.flush()  # Ensure immediate transmission
    except Exception as e:
        print(f'{{"status": "error", "message": "JSON encoding error: {e}"}}')

def log_message(message, level="INFO"):
    """Send log message over serial"""
    log_dict = {
        "type": "log",
        "level": level,
        "message": message,
        "timestamp": ticks_ms()
    }
    send_serial_response(log_dict)

def log_error(message, error_type="GENERAL"):
    """Log error with tracking"""
    global nav_state
    nav_state.total_errors += 1
    nav_state.last_error_message = f"{error_type}: {message}"
    nav_state.last_error_time = ticks_ms()
    log_message(nav_state.last_error_message, "ERROR")

def send_heartbeat():
    """Send periodic heartbeat with system status"""
    global nav_state
    current_time = ticks_ms()
    
    if ticks_diff(current_time, nav_state.last_heartbeat_time) >= nav_state.heartbeat_interval_ms:
        nav_state.last_heartbeat_time = current_time
        
        heartbeat_data = {
            "type": "heartbeat",
            "timestamp": current_time,
            "uptime_ms": current_time,
            "health_status": "healthy" if not nav_state.emergency_stop_active else "emergency_stop",
            "is_moving": nav_state.is_moving,
            "autonomous_mode": nav_state.autonomous_mode,
            "commands_received": nav_state.total_commands_received,
            "total_errors": nav_state.total_errors,
            "serial_errors": nav_state.serial_read_errors,
            "json_errors": nav_state.json_parse_errors
        }
        
        send_serial_response(heartbeat_data)

def percent_to_pwm(percent):
    """Convert percentage to PWM value with minimum threshold"""
    percent = max(0, min(100, int(percent)))
    if percent > 0 and percent < 30:
        percent = 30  # Minimum speed for motor movement
    return int(percent * 65535 // 100)

def ramp_to_speed(new_speed, left_dir_func, right_dir_func):
    """Smooth speed ramping to prevent tipping"""
    global nav_state
    nav_state.target_speed = new_speed
    
    while nav_state.current_speed != nav_state.target_speed:
        if nav_state.emergency_stop_active:
            return
            
        if nav_state.current_speed < nav_state.target_speed:
            nav_state.current_speed = min(nav_state.current_speed + nav_state.ramp_step, nav_state.target_speed)
        else:
            nav_state.current_speed = max(nav_state.current_speed - nav_state.ramp_step, nav_state.target_speed)
        
        # Set motor directions
        left_dir_func()
        right_dir_func()
        
        # Apply speed
        ena_pwm.duty_u16(nav_state.current_speed)
        enb_pwm.duty_u16(nav_state.current_speed)
        
        sleep_ms(nav_state.ramp_delay_ms)

# Motor direction functions
def left_motor_forward():
    in1.high(); in2.low()

def left_motor_backward():
    in1.low(); in2.high()

def left_motor_stop():
    in1.low(); in2.low()

def right_motor_forward():
    in3.high(); in4.low()

def right_motor_backward():
    in3.low(); in4.high()

def right_motor_stop():
    in3.low(); in4.low()

# Navigation commands
def move_forward(speed=60, duration=0):
    """Move forward at specified speed"""
    global nav_state
    if nav_state.emergency_stop_active:
        return False
    
    pwm_speed = percent_to_pwm(speed)
    nav_state.is_moving = True
    nav_state.move_start_time = ticks_ms()
    nav_state.move_duration = duration * 1000 if duration > 0 else 0
    
    ramp_to_speed(pwm_speed, left_motor_forward, right_motor_forward)
    led.on()  # LED on during movement
    return True

def move_backward(speed=60, duration=0):
    """Move backward at specified speed"""
    global nav_state
    if nav_state.emergency_stop_active:
        return False
    
    pwm_speed = percent_to_pwm(speed)
    nav_state.is_moving = True
    nav_state.move_start_time = ticks_ms()
    nav_state.move_duration = duration * 1000 if duration > 0 else 0
    
    ramp_to_speed(pwm_speed, left_motor_backward, right_motor_backward)
    led.on()  # LED on during movement
    return True

def rotate_left(speed=50, duration=0):
    """Rotate left (counter-clockwise)"""
    global nav_state
    if nav_state.emergency_stop_active:
        return False
    
    pwm_speed = percent_to_pwm(speed)
    nav_state.is_moving = True
    nav_state.rotation_start_time = ticks_ms()
    nav_state.rotation_duration = duration * 1000 if duration > 0 else 0
    
    ramp_to_speed(pwm_speed, left_motor_backward, right_motor_forward)
    led.on()  # LED on during movement
    return True

def rotate_right(speed=50, duration=0):
    """Rotate right (clockwise)"""
    global nav_state
    if nav_state.emergency_stop_active:
        return False
    
    pwm_speed = percent_to_pwm(speed)
    nav_state.is_moving = True
    nav_state.rotation_start_time = ticks_ms()
    nav_state.rotation_duration = duration * 1000 if duration > 0 else 0
    
    ramp_to_speed(pwm_speed, left_motor_forward, right_motor_backward)
    led.on()  # LED on during movement
    return True

def smooth_stop():
    """Smooth stop with ramping"""
    global nav_state
    nav_state.is_moving = False
    ramp_to_speed(0, left_motor_stop, right_motor_stop)
    nav_state.current_speed = 0
    led.off()  # LED off when stopped

def emergency_stop():
    """Immediate emergency stop"""
    global nav_state
    nav_state.emergency_stop_active = True
    nav_state.is_moving = False
    nav_state.current_speed = 0
    nav_state.target_speed = 0
    
    # Immediate motor shutdown
    ena_pwm.duty_u16(0)
    enb_pwm.duty_u16(0)
    left_motor_stop()
    right_motor_stop()
    
    # Flash LED for emergency stop
    for _ in range(6):
        led.toggle()
        sleep_ms(100)
    led.off()
    
    log_message("EMERGENCY STOP ACTIVATED", "WARN")

def reset_emergency_stop():
    """Reset emergency stop state"""
    global nav_state
    nav_state.emergency_stop_active = False
    log_message("Emergency stop reset", "INFO")

def check_movement_timeout():
    """Check if timed movement should stop"""
    global nav_state
    current_time = ticks_ms()
    
    if nav_state.is_moving and nav_state.move_duration > 0:
        if ticks_diff(current_time, nav_state.move_start_time) >= nav_state.move_duration:
            smooth_stop()
            return True
    
    if nav_state.is_moving and nav_state.rotation_duration > 0:
        if ticks_diff(current_time, nav_state.rotation_start_time) >= nav_state.rotation_duration:
            smooth_stop()
            return True
    
    return False

def get_status():
    """Get current navigation status with enhanced monitoring data"""
    global nav_state
    current_time = ticks_ms()
    
    # Calculate movement progress for timed movements
    move_progress = 0
    rotation_progress = 0
    time_remaining_ms = 0
    
    if nav_state.is_moving:
        if nav_state.move_duration > 0:
            elapsed = ticks_diff(current_time, nav_state.move_start_time)
            move_progress = min(100, (elapsed / nav_state.move_duration) * 100)
            time_remaining_ms = max(0, nav_state.move_duration - elapsed)
        elif nav_state.rotation_duration > 0:
            elapsed = ticks_diff(current_time, nav_state.rotation_start_time)
            rotation_progress = min(100, (elapsed / nav_state.rotation_duration) * 100)
            time_remaining_ms = max(0, nav_state.rotation_duration - elapsed)
    
    # System health indicators
    last_command_age = ticks_diff(current_time, nav_state.last_command_time)
    health_status = "healthy"
    if nav_state.emergency_stop_active:
        health_status = "emergency_stop"
    elif last_command_age > nav_state.command_timeout_ms and nav_state.autonomous_mode:
        health_status = "timeout_warning"
    elif last_command_age > (nav_state.command_timeout_ms * 0.8) and nav_state.autonomous_mode:
        health_status = "timeout_approaching"
    
    return {
        "type": "status",
        "timestamp": current_time,
        "autonomous_mode": nav_state.autonomous_mode,
        "is_moving": nav_state.is_moving,
        "current_speed": nav_state.current_speed,
        "target_speed": nav_state.target_speed,
        "emergency_stop": nav_state.emergency_stop_active,
        "last_command_age_ms": last_command_age,
        "command_timeout_ms": nav_state.command_timeout_ms,
        "health_status": health_status,
        "wifi_enabled": nav_state.wifi_enabled,
        "uptime_ms": current_time,
        "movement": {
            "move_progress_percent": move_progress,
            "rotation_progress_percent": rotation_progress,
            "time_remaining_ms": time_remaining_ms,
            "move_duration_ms": nav_state.move_duration,
            "rotation_duration_ms": nav_state.rotation_duration
        },
        "hardware": {
            "led_state": led.value(),
            "motor_pwm_freq": ena_pwm.freq(),
            "ramp_step": nav_state.ramp_step,
            "ramp_delay_ms": nav_state.ramp_delay_ms
        },
        "version": "2.1.0"
    }

def get_diagnostics():
    """Get detailed diagnostic information"""
    global nav_state
    current_time = ticks_ms()
    
    return {
        "type": "diagnostics",
        "timestamp": current_time,
        "system": {
            "uptime_ms": current_time,
            "total_commands": nav_state.total_commands_received,
            "total_errors": nav_state.total_errors,
            "serial_read_errors": nav_state.serial_read_errors,
            "json_parse_errors": nav_state.json_parse_errors,
            "last_error": nav_state.last_error_message,
            "last_error_time": nav_state.last_error_time,
            "memory_free": None  # Could add gc.mem_free() if available
        },
        "communication": {
            "last_command_age_ms": ticks_diff(current_time, nav_state.last_command_time),
            "command_timeout_ms": nav_state.command_timeout_ms,
            "heartbeat_interval_ms": nav_state.heartbeat_interval_ms,
            "last_heartbeat_age_ms": ticks_diff(current_time, nav_state.last_heartbeat_time)
        },
        "motors": {
            "current_speed": nav_state.current_speed,
            "target_speed": nav_state.target_speed,
            "is_moving": nav_state.is_moving,
            "ramp_step": nav_state.ramp_step,
            "ramp_delay_ms": nav_state.ramp_delay_ms,
            "pwm_frequency": ena_pwm.freq()
        },
        "safety": {
            "emergency_stop_active": nav_state.emergency_stop_active,
            "autonomous_mode": nav_state.autonomous_mode,
            "safety_timer_active": True
        },
        "wifi": {
            "available": WIFI_AVAILABLE,
            "enabled": nav_state.wifi_enabled
        }
    }

def process_serial_command(command_dict):
    """Process command received over serial"""
    global nav_state
    nav_state.last_command_time = ticks_ms()
    nav_state.total_commands_received += 1
    
    try:
        cmd = command_dict.get('cmd', '').lower()
        
        if cmd == 'move':
            direction = command_dict.get('dir', 'forward')
            speed = command_dict.get('speed', 60)
            duration = command_dict.get('duration', 0)
            
            # Validate direction
            if direction not in ['forward', 'backward']:
                return {"status": "error", "message": f"Invalid direction: {direction}. Use 'forward' or 'backward'"}
            
            # Validate speed
            if not (0 <= speed <= 100):
                return {"status": "error", "message": f"Invalid speed: {speed}. Must be 0-100"}
            
            # Validate duration
            if duration < 0:
                return {"status": "error", "message": f"Invalid duration: {duration}. Must be >= 0"}
            
            success = False
            if direction == 'forward':
                success = move_forward(speed, duration)
            elif direction == 'backward':
                success = move_backward(speed, duration)
            
            if success:
                return {"status": "ok", "message": f"Moving {direction} at {speed}%", "command_id": nav_state.total_commands_received}
            else:
                return {"status": "error", "message": "Emergency stop active", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'rotate':
            direction = command_dict.get('dir', 'left')
            speed = command_dict.get('speed', 50)
            duration = command_dict.get('duration', 0)
            
            # Validate direction
            if direction not in ['left', 'right']:
                return {"status": "error", "message": f"Invalid direction: {direction}. Use 'left' or 'right'"}
            
            # Validate speed
            if not (0 <= speed <= 100):
                return {"status": "error", "message": f"Invalid speed: {speed}. Must be 0-100"}
            
            # Validate duration
            if duration < 0:
                return {"status": "error", "message": f"Invalid duration: {duration}. Must be >= 0"}
            
            success = False
            if direction == 'left':
                success = rotate_left(speed, duration)
            elif direction == 'right':
                success = rotate_right(speed, duration)
            
            if success:
                return {"status": "ok", "message": f"Rotating {direction} at {speed}%", "command_id": nav_state.total_commands_received}
            else:
                return {"status": "error", "message": "Emergency stop active", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'stop':
            smooth_stop()
            return {"status": "ok", "message": "Stopped", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'estop':
            emergency_stop()
            return {"status": "ok", "message": "Emergency stop activated", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'reset_estop':
            reset_emergency_stop()
            return {"status": "ok", "message": "Emergency stop reset", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'autonomous':
            mode = command_dict.get('mode', 'on')
            if mode not in ['on', 'off']:
                return {"status": "error", "message": f"Invalid mode: {mode}. Use 'on' or 'off'"}
            nav_state.autonomous_mode = (mode == 'on')
            return {"status": "ok", "message": f"Autonomous mode: {nav_state.autonomous_mode}", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'status':
            return get_status()
        
        elif cmd == 'diagnostics':
            return get_diagnostics()
        
        elif cmd == 'wifi':
            action = command_dict.get('action', 'status')
            if action == 'enable' and WIFI_AVAILABLE:
                return enable_wifi_server()
            elif action == 'disable':
                nav_state.wifi_enabled = False
                return {"status": "ok", "message": "WiFi disabled", "command_id": nav_state.total_commands_received}
            else:
                return {"status": "ok", "message": f"WiFi available: {WIFI_AVAILABLE}, enabled: {nav_state.wifi_enabled}", "command_id": nav_state.total_commands_received}
        
        elif cmd == 'ping':
            return {"status": "ok", "message": "pong", "timestamp": ticks_ms(), "command_id": nav_state.total_commands_received}
        
        else:
            return {"status": "error", "message": f"Unknown command: {cmd}", "available_commands": ["move", "rotate", "stop", "estop", "reset_estop", "autonomous", "status", "diagnostics", "wifi", "ping"]}
    
    except Exception as e:
        log_error(f"Command processing error: {e}", "COMMAND_PROCESSING")
        return {"status": "error", "message": f"Command processing error: {e}", "command_id": nav_state.total_commands_received}

def read_serial_input():
    """Read and process serial input - improved version with error tracking"""
    global nav_state
    
    try:
        # Check if there's input available
        if hasattr(sys.stdin, 'any'):
            # MicroPython specific
            if sys.stdin.any():
                line = sys.stdin.readline().strip()
                if line:
                    try:
                        command_dict = json.loads(line)
                        response = process_serial_command(command_dict)
                        send_serial_response(response)
                    except json.JSONDecodeError as e:
                        nav_state.json_parse_errors += 1
                        error_response = {
                            "status": "error", 
                            "message": f"Invalid JSON: {str(e)}", 
                            "raw_input": line[:50] + "..." if len(line) > 50 else line,
                            "parse_errors": nav_state.json_parse_errors
                        }
                        send_serial_response(error_response)
                        log_error(f"JSON parse error: {str(e)}", "JSON_PARSE")
        else:
            # Fallback method - try to read with short timeout
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line:
                    try:
                        command_dict = json.loads(line)
                        response = process_serial_command(command_dict)
                        send_serial_response(response)
                    except json.JSONDecodeError as e:
                        nav_state.json_parse_errors += 1
                        error_response = {
                            "status": "error", 
                            "message": f"Invalid JSON: {str(e)}", 
                            "raw_input": line[:50] + "..." if len(line) > 50 else line,
                            "parse_errors": nav_state.json_parse_errors
                        }
                        send_serial_response(error_response)
                        log_error(f"JSON parse error: {str(e)}", "JSON_PARSE")
    except Exception as e:
        nav_state.serial_read_errors += 1
        # Only log every 10th serial read error to avoid spam
        if nav_state.serial_read_errors % 10 == 0:
            log_error(f"Serial read error (count: {nav_state.serial_read_errors}): {str(e)}", "SERIAL_READ")

# Optional WiFi server for testing
def enable_wifi_server():
    """Enable WiFi server for testing (optional)"""
    if not WIFI_AVAILABLE:
        return {"status": "error", "message": "WiFi not available"}
    
    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(SSID, PASSWORD)
        
        # Wait for connection (timeout after 10 seconds)
        timeout = 0
        while not wlan.isconnected() and timeout < 20:
            sleep(0.5)
            timeout += 1
        
        if wlan.isconnected():
            nav_state.wifi_enabled = True
            ip = wlan.ifconfig()[0]
            return {"status": "ok", "message": f"WiFi enabled, IP: {ip}"}
        else:
            return {"status": "error", "message": "WiFi connection failed"}
    except Exception as e:
        return {"status": "error", "message": f"WiFi setup error: {e}"}

def main_loop():
    """Main control loop with enhanced monitoring"""
    log_message("MicroPython Motor Controller started - Serial mode", "INFO")
    log_message(f"WiFi available: {WIFI_AVAILABLE}", "INFO")
    log_message("System started in EMERGENCY STOP mode for safety", "WARN")
    log_message("Send reset_estop command to enable movement", "INFO")
    log_message(f"Heartbeat interval: {nav_state.heartbeat_interval_ms}ms", "INFO")
    
    # Send initial status
    send_serial_response(get_status())
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            
            # Check for movement timeouts
            check_movement_timeout()
            
            # Process serial commands
            read_serial_input()
            
            # Send periodic heartbeat
            send_heartbeat()
            
            # Every 1000 loops (~10 seconds), send diagnostics if in autonomous mode
            if loop_count % 1000 == 0 and nav_state.autonomous_mode:
                diagnostics = get_diagnostics()
                diagnostics["type"] = "periodic_diagnostics"
                send_serial_response(diagnostics)
            
            # Small delay to prevent excessive CPU usage
            sleep_ms(10)
    
    except KeyboardInterrupt:
        log_message("Controller stopped by user", "INFO")
        emergency_stop()
    except Exception as e:
        log_error(f"Main loop error: {e}", "MAIN_LOOP")
        emergency_stop()

# Run the main loop
if __name__ == "__main__":
    main_loop()

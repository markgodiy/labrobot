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
        self.emergency_stop_active = False
        self.last_command_time = 0
        self.command_timeout_ms = 3000  # 3 second timeout for serial
        self.ramp_step = 3000
        self.ramp_delay_ms = 50
        self.wifi_enabled = False
        
        # Movement tracking
        self.move_start_time = 0
        self.move_duration = 0
        self.rotation_start_time = 0
        self.rotation_duration = 0
        
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
    """Get current navigation status"""
    global nav_state
    return {
        "type": "status",
        "autonomous_mode": nav_state.autonomous_mode,
        "is_moving": nav_state.is_moving,
        "current_speed": nav_state.current_speed,
        "emergency_stop": nav_state.emergency_stop_active,
        "last_command_age_ms": ticks_diff(ticks_ms(), nav_state.last_command_time),
        "wifi_enabled": nav_state.wifi_enabled,
        "uptime_ms": ticks_ms()
    }

def process_serial_command(command_dict):
    """Process command received over serial"""
    global nav_state
    nav_state.last_command_time = ticks_ms()
    
    try:
        cmd = command_dict.get('cmd', '').lower()
        
        if cmd == 'move':
            direction = command_dict.get('dir', 'forward')
            speed = command_dict.get('speed', 60)
            duration = command_dict.get('duration', 0)
            
            success = False
            if direction == 'forward':
                success = move_forward(speed, duration)
            elif direction == 'backward':
                success = move_backward(speed, duration)
            
            if success:
                return {"status": "ok", "message": f"Moving {direction} at {speed}%"}
            else:
                return {"status": "error", "message": "Emergency stop active"}
        
        elif cmd == 'rotate':
            direction = command_dict.get('dir', 'left')
            speed = command_dict.get('speed', 50)
            duration = command_dict.get('duration', 0)
            
            success = False
            if direction == 'left':
                success = rotate_left(speed, duration)
            elif direction == 'right':
                success = rotate_right(speed, duration)
            
            if success:
                return {"status": "ok", "message": f"Rotating {direction} at {speed}%"}
            else:
                return {"status": "error", "message": "Emergency stop active"}
        
        elif cmd == 'stop':
            smooth_stop()
            return {"status": "ok", "message": "Stopped"}
        
        elif cmd == 'estop':
            emergency_stop()
            return {"status": "ok", "message": "Emergency stop activated"}
        
        elif cmd == 'reset_estop':
            reset_emergency_stop()
            return {"status": "ok", "message": "Emergency stop reset"}
        
        elif cmd == 'autonomous':
            mode = command_dict.get('mode', 'on')
            nav_state.autonomous_mode = (mode == 'on')
            return {"status": "ok", "message": f"Autonomous mode: {nav_state.autonomous_mode}"}
        
        elif cmd == 'status':
            return get_status()
        
        elif cmd == 'wifi':
            action = command_dict.get('action', 'status')
            if action == 'enable' and WIFI_AVAILABLE:
                return enable_wifi_server()
            elif action == 'disable':
                nav_state.wifi_enabled = False
                return {"status": "ok", "message": "WiFi disabled"}
            else:
                return {"status": "ok", "message": f"WiFi available: {WIFI_AVAILABLE}, enabled: {nav_state.wifi_enabled}"}
        
        elif cmd == 'ping':
            return {"status": "ok", "message": "pong", "timestamp": ticks_ms()}
        
        else:
            return {"status": "error", "message": f"Unknown command: {cmd}"}
    
    except Exception as e:
        return {"status": "error", "message": f"Command processing error: {e}"}

def read_serial_input():
    """Read and process serial input"""
    global serial_buffer
    
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline().strip()
            if line:
                try:
                    command_dict = json.loads(line)
                    response = process_serial_command(command_dict)
                    send_serial_response(response)
                except json.JSONDecodeError:
                    send_serial_response({"status": "error", "message": "Invalid JSON"})
        except Exception as e:
            send_serial_response({"status": "error", "message": f"Serial read error: {e}"})

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

# Try to import select for non-blocking serial read
try:
    import select
    SELECT_AVAILABLE = True
except ImportError:
    SELECT_AVAILABLE = False

def main_loop():
    """Main control loop"""
    log_message("MicroPython Motor Controller started - Serial mode", "INFO")
    log_message(f"WiFi available: {WIFI_AVAILABLE}", "INFO")
    
    # Send initial status
    send_serial_response(get_status())
    
    try:
        while True:
            # Check for movement timeouts
            check_movement_timeout()
            
            # Process serial commands
            if SELECT_AVAILABLE:
                read_serial_input()
            else:
                # Fallback for systems without select
                try:
                    if sys.stdin.readline():
                        # There's data available, but we can't process it without select
                        # This is a limitation on some MicroPython implementations
                        pass
                except:
                    pass
            
            # Small delay to prevent excessive CPU usage
            sleep_ms(10)
    
    except KeyboardInterrupt:
        log_message("Controller stopped by user", "INFO")
        emergency_stop()
    except Exception as e:
        log_message(f"Main loop error: {e}", "ERROR")
        emergency_stop()

# Run the main loop
if __name__ == "__main__":
    main_loop()

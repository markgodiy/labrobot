"""
ROS 2 Compatible MicroPython Motor Controller for Autonomous Navigation
=====================================================================

This code runs on the MicroPython device and receives navigation commands
from the ROS 2 system running on the Pi. The Pi processes LIDAR and depth 
camera data and sends movement commands via HTTP requests.

Communication Protocol:
- Pi ROS 2 node sends HTTP requests to MicroPython controller
- Commands: /move, /rotate, /stop, /estop, /status
- Parameters: direction, speed, duration, angle

Hardware Setup:
- Motor A (Left): ENA=Pin(2), IN1=Pin(3), IN2=Pin(4)
- Motor B (Right): ENB=Pin(8), IN3=Pin(6), IN4=Pin(7)
- LED: Pin('LED') for status indication
"""

from machine import Pin, PWM, Timer
from time import sleep, sleep_ms, ticks_ms, ticks_diff
import network
import socket
import json
from wifi_config import SSID, PASSWORD

# Hardware setup
led = Pin('LED', Pin.OUT)

# Startup LED sequence
for _ in range(6):
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

# Navigation state variables
class NavigationState:
    def __init__(self):
        self.current_speed = 0
        self.target_speed = 0
        self.is_moving = False
        self.autonomous_mode = False
        self.emergency_stop_active = False
        self.last_command_time = 0
        self.command_timeout_ms = 2000  # 2 second timeout
        self.ramp_step = 3000
        self.ramp_delay_ms = 50
        
        # Movement tracking
        self.move_start_time = 0
        self.move_duration = 0
        self.rotation_start_time = 0
        self.rotation_duration = 0
        
nav_state = NavigationState()

# Safety timer for autonomous mode
safety_timer = Timer(-1)

def safety_check(timer):
    """Safety check - stop if no command received within timeout"""
    global nav_state
    current_time = ticks_ms()
    if nav_state.autonomous_mode and not nav_state.emergency_stop_active:
        if ticks_diff(current_time, nav_state.last_command_time) > nav_state.command_timeout_ms:
            print("SAFETY: Command timeout - stopping motors")
            emergency_stop()

# Start safety timer (checks every 500ms)
safety_timer.init(period=500, mode=Timer.PERIODIC, callback=safety_check)

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
    return True

def smooth_stop():
    """Smooth stop with ramping"""
    global nav_state
    nav_state.is_moving = False
    ramp_to_speed(0, left_motor_stop, right_motor_stop)
    nav_state.current_speed = 0

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
    
    print("EMERGENCY STOP ACTIVATED")

def reset_emergency_stop():
    """Reset emergency stop state"""
    global nav_state
    nav_state.emergency_stop_active = False
    print("Emergency stop reset")

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

def connect_wifi():
    """Connect to WiFi network"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    
    print("Connecting to WiFi...")
    while not wlan.isconnected():
        sleep(0.5)
    
    ip = wlan.ifconfig()[0]
    print(f'Connected to WiFi, IP: {ip}')
    led.on()  # Solid LED when connected
    return ip

def parse_command_params(path):
    """Parse command parameters from URL path"""
    params = {}
    if '?' in path:
        query_string = path.split('?')[1]
        for param in query_string.split('&'):
            if '=' in param:
                key, value = param.split('=', 1)
                try:
                    # Try to convert to number
                    if '.' in value:
                        params[key] = float(value)
                    else:
                        params[key] = int(value)
                except ValueError:
                    params[key] = value
    return params

def get_status():
    """Get current navigation status"""
    global nav_state
    return {
        'autonomous_mode': nav_state.autonomous_mode,
        'is_moving': nav_state.is_moving,
        'current_speed': nav_state.current_speed,
        'emergency_stop': nav_state.emergency_stop_active,
        'last_command_age_ms': ticks_diff(ticks_ms(), nav_state.last_command_time)
    }

def process_ros2_command(path, params):
    """Process ROS 2 navigation commands"""
    global nav_state
    nav_state.last_command_time = ticks_ms()
    
    response = {'status': 'error', 'message': 'Unknown command'}
    
    if path.startswith('/move'):
        direction = params.get('dir', 'forward')
        speed = params.get('speed', 60)
        duration = params.get('duration', 0)
        
        success = False
        if direction == 'forward':
            success = move_forward(speed, duration)
        elif direction == 'backward':
            success = move_backward(speed, duration)
        
        if success:
            response = {'status': 'ok', 'message': f'Moving {direction} at {speed}%'}
        else:
            response = {'status': 'error', 'message': 'Emergency stop active'}
    
    elif path.startswith('/rotate'):
        direction = params.get('dir', 'left')
        speed = params.get('speed', 50)
        duration = params.get('duration', 0)
        
        success = False
        if direction == 'left':
            success = rotate_left(speed, duration)
        elif direction == 'right':
            success = rotate_right(speed, duration)
        
        if success:
            response = {'status': 'ok', 'message': f'Rotating {direction} at {speed}%'}
        else:
            response = {'status': 'error', 'message': 'Emergency stop active'}
    
    elif path.startswith('/stop'):
        smooth_stop()
        response = {'status': 'ok', 'message': 'Stopped'}
    
    elif path.startswith('/estop'):
        emergency_stop()
        response = {'status': 'ok', 'message': 'Emergency stop activated'}
    
    elif path.startswith('/reset_estop'):
        reset_emergency_stop()
        response = {'status': 'ok', 'message': 'Emergency stop reset'}
    
    elif path.startswith('/autonomous'):
        mode = params.get('mode', 'on')
        nav_state.autonomous_mode = (mode == 'on')
        response = {'status': 'ok', 'message': f'Autonomous mode: {nav_state.autonomous_mode}'}
    
    elif path.startswith('/status'):
        status = get_status()
        response = {'status': 'ok', 'data': status}
    
    return response

def start_ros2_bridge_server():
    """Start HTTP server for ROS 2 bridge communication"""
    ip = connect_wifi()
    
    # Try to bind to known IP, fallback to DHCP
    try:
        addr = socket.getaddrinfo('192.168.25.72', 8080)[0][-1]
        print('Using IP: 192.168.25.72:8080')
    except Exception:
        addr = socket.getaddrinfo(ip, 8080)[0][-1]
        print(f'Using DHCP IP: {ip}:8080')
    
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(5)
    print(f'ROS 2 Bridge Server listening on {addr}')
    
    try:
        while True:
            # Check for movement timeouts
            check_movement_timeout()
            
            # Handle incoming connections (non-blocking)
            s.settimeout(0.1)  # 100ms timeout for accept
            try:
                cl, client_addr = s.accept()
                try:
                    cl.settimeout(1.0)  # 1 second timeout for request
                    req = cl.recv(1024).decode()
                    
                    if req:
                        path = req.split(' ')[1] if ' ' in req else '/'
                        params = parse_command_params(path)
                        
                        # Process command
                        response = process_ros2_command(path.split('?')[0], params)
                        
                        # Send JSON response
                        response_json = json.dumps(response)
                        cl.send('HTTP/1.1 200 OK\r\n')
                        cl.send('Content-Type: application/json\r\n')
                        cl.send('Access-Control-Allow-Origin: *\r\n')
                        cl.send(f'Content-Length: {len(response_json)}\r\n')
                        cl.send('\r\n')
                        cl.send(response_json)
                        
                        print(f"Command: {path}, Response: {response['status']}")
                
                except Exception as e:
                    print(f"Request error: {e}")
                    try:
                        cl.send('HTTP/1.1 400 Bad Request\r\n\r\n')
                    except:
                        pass
                finally:
                    try:
                        cl.close()
                    except:
                        pass
            
            except OSError:
                # Timeout on accept - continue loop
                pass
    
    except KeyboardInterrupt:
        print("Server stopped by user")
    finally:
        emergency_stop()
        s.close()

# Main execution
if __name__ == "__main__":
    print("Starting ROS 2 Compatible Autonomous Navigation Controller")
    print("Commands available: /move, /rotate, /stop, /estop, /reset_estop, /autonomous, /status")
    start_ros2_bridge_server()

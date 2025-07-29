from machine import Pin, PWM
from time import sleep, sleep_ms
import network
import socket
from wifi_config import SSID, PASSWORD

led = Pin('LED', Pin.OUT)

# Blink LED at startup
for _ in range(6):
    led.toggle()
    sleep(0.2)
led.off()

# MotorA setup
ena_pwm = PWM(Pin(2))  # Enable pin for Motor A
in1 = Pin(3, Pin.OUT)
in2 = Pin(4, Pin.OUT)
ena_pwm.freq(1000)

# MotorB setup
in3 = Pin(6, Pin.OUT)
in4 = Pin(7, Pin.OUT)
enb_pwm = PWM(Pin(8))  # Enable pin for Motor B
enb_pwm.freq(1000)

# Speed ramping variables
current_speed = 0
target_speed = 0
ramp_step = 3000  # PWM units per step (adjust for faster/slower ramping)
ramp_delay_ms = 50  # Milliseconds between ramp steps

# Ramping configuration
def set_ramp_speed(fast=False):
    """Configure ramping speed: fast=True for quicker response, False for smoother"""
    global ramp_step, ramp_delay_ms
    if fast:
        ramp_step = 5000    # Faster ramping
        ramp_delay_ms = 30  # Shorter delays
    else:
        ramp_step = 3000    # Smoother ramping (default)
        ramp_delay_ms = 50  # Longer delays for smoothness

# Speed ramping function to prevent tipping
def ramp_to_speed(new_speed, direction_func):
    global current_speed, target_speed
    target_speed = new_speed
    
    # If we're already at the target, just set direction and return
    if current_speed == target_speed:
        direction_func()
        ena_pwm.duty_u16(current_speed)
        enb_pwm.duty_u16(current_speed)
        return
    
    # Ramp up or down to target speed
    while current_speed != target_speed:
        if current_speed < target_speed:
            # Ramp up
            current_speed = min(current_speed + ramp_step, target_speed)
        else:
            # Ramp down
            current_speed = max(current_speed - ramp_step, target_speed)
        
        # Set motor direction first (before applying speed)
        direction_func()
        
        # Apply ramped speed
        ena_pwm.duty_u16(current_speed)
        enb_pwm.duty_u16(current_speed)
        
        # Small delay for smooth ramping
        sleep_ms(ramp_delay_ms)

# Direction setting functions (no speed control, just direction)
def set_forward_direction():
    in1.high(); in2.low()  # Motor A forward
    in3.high(); in4.low()  # Motor B forward

def set_backward_direction():
    in1.low(); in2.high()  # Motor A backward
    in3.low(); in4.high()  # Motor B backward

def set_left_rotation():
    in1.low(); in2.high()   # Motor A (left wheel) backward
    in3.high(); in4.low()   # Motor B (right wheel) forward

def set_right_rotation():
    in1.high(); in2.low()   # Motor A (left wheel) forward
    in3.low(); in4.high()   # Motor B (right wheel) backward
# Motor functions with speed ramping - adjust these if hardware is wired backwards
def motor_forward(speed=None):
    if speed is None:
        speed = percent_to_pwm(60)  # Default 60% speed
    ramp_to_speed(speed, set_forward_direction)

def motor_backward(speed=None):
    if speed is None:
        speed = percent_to_pwm(60)  # Default 60% speed
    ramp_to_speed(speed, set_backward_direction)

def rotate_left(speed=None):
    if speed is None:
        speed = percent_to_pwm(60)  # Default 60% speed
    ramp_to_speed(speed, set_left_rotation)

def rotate_right(speed=None):
    if speed is None:
        speed = percent_to_pwm(60)  # Default 60% speed
    ramp_to_speed(speed, set_right_rotation)

def motor_stop():
    global current_speed
    # Ramp down to stop for smooth deceleration
    ramp_to_speed(0, lambda: None)  # No direction change needed when stopping
    # Ensure everything is off
    ena_pwm.duty_u16(0)
    enb_pwm.duty_u16(0)
    in1.low(); in2.low()
    in3.low(); in4.low()
    current_speed = 0

def emergency_stop():
    """Immediate stop without ramping - use only in emergencies"""
    global current_speed, target_speed
    current_speed = 0
    target_speed = 0
    ena_pwm.duty_u16(0)
    enb_pwm.duty_u16(0)
    in1.low(); in2.low()
    in3.low(); in4.low()

# Connect to Wi-Fi
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        sleep(0.5)
    ip = wlan.ifconfig()[0]
    print('Connected, IP:', ip)
    led.on()  # Solid LED when Wi-Fi is connected
    return ip

# Helper to convert percent to PWM value
def percent_to_pwm(percent):
    percent = max(0, min(100, int(percent)))
    # Ensure minimum speed for motor movement (below 30% motors might not turn)
    if percent > 0 and percent < 30:
        percent = 30
    return int(percent * 65535 // 100)

# Parse speed from query string
def get_speed_from_path(path):
    try:
        import re
        match = re.search(r'speed=(\d+)', path)
        if match:
            return percent_to_pwm(match.group(1))
    except ImportError:
        # Fallback for older MicroPython versions
        if 'speed=' in path:
            try:
                speed_part = path.split('speed=')[1].split('&')[0]
                return percent_to_pwm(speed_part)
            except (IndexError, ValueError):
                pass
    return percent_to_pwm(60)  # Default 60%

# Simple web server for motor control
def start_server():
    ip = connect_wifi()
    # Use static IP if available, else fallback to DHCP-assigned IP
    try:
        # Try to bind to static IP
        addr = socket.getaddrinfo('192.168.25.72', 80)[0][-1]
        print('Trying static IP: 192.168.25.72')
    except Exception:
        # Fallback to DHCP IP
        addr = socket.getaddrinfo(ip, 80)[0][-1]
        print(f'Falling back to DHCP IP: {ip}')
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(addr)
    s.listen(1)
    print('Listening on', addr)

    try:
        while True:
            cl, client_addr = s.accept()
            try:
                req = cl.recv(1024).decode()
                path = req.split(' ')[1] if ' ' in req else '/'
                cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
                # Motor control commands
                speed = get_speed_from_path(path)
                print(f"Path: {path}, Speed PWM: {speed}")  # Debug output
                if path.startswith('/forward'):
                    motor_forward(speed)  # Fixed: forward now calls forward
                elif path.startswith('/backward'):
                    motor_backward(speed)  # Fixed: backward now calls backward
                elif path.startswith('/left'):
                    rotate_left(speed)  # Fixed: left now calls left
                elif path.startswith('/right'):
                    rotate_right(speed)  # Fixed: right now calls right
                elif path.startswith('/stop'):
                    motor_stop()
                elif path.startswith('/estop'):
                    emergency_stop()
                elif path.startswith('/ramp_fast'):
                    set_ramp_speed(fast=True)
                elif path.startswith('/ramp_smooth'):
                    set_ramp_speed(fast=False)
                cl.send(f"""
<html>
<head>
    <title>PicoW Motor Control</title>
    <style>
        body {{ font-family: Arial, sans-serif; text-align: center; margin-top: 40px; }}
        .row {{ margin: 10px 0; }}
        button {{ width: 120px; height: 50px; font-size: 18px; margin: 5px; }}
        .slider {{ width: 300px; }}
    </style>
    <script>
        let speed = 60;
        function updateSpeed(val) {{
            speed = val;
            document.getElementById('speedval').innerText = speed + '%';
        }}
        function sendCmd(cmd) {{
            fetch(cmd + '?speed=' + speed);
        }}
        function startCmd(cmd) {{
            sendCmd(cmd);
        }}
        function stopCmd() {{
            sendCmd('/stop');
        }}
        function emergencyStop() {{
            fetch('/estop');
        }}
        function setRampSpeed(fast) {{
            if (fast) {{
                fetch('/ramp_fast');
            }} else {{
                fetch('/ramp_smooth');
            }}
        }}
    </script>
</head>
<body>
    <h2>Motor Control Panel</h2>
    <div class='row'>
        <label for='speed'>Speed: <span id='speedval'>60%</span></label><br>
        <input type='range' min='0' max='100' value='60' class='slider' id='speed' oninput='updateSpeed(this.value)'>
    </div>
    <div class='row'>
        <button onmousedown="startCmd('/forward')" onmouseup="stopCmd()" ontouchstart="startCmd('/forward')" ontouchend="stopCmd()">Forward</button>
    </div>
    <div class='row'>
        <button onmousedown="startCmd('/left')" onmouseup="stopCmd()" ontouchstart="startCmd('/left')" ontouchend="stopCmd()">Rotate_Left</button>
        <button onclick="stopCmd()" style='background-color:#f44336;color:white;'>Stop</button>
        <button onmousedown="startCmd('/right')" onmouseup="stopCmd()" ontouchstart="startCmd('/right')" ontouchend="stopCmd()">Rotate_Right</button>
    </div>
    <div class='row'>
        <button onmousedown="startCmd('/backward')" onmouseup="stopCmd()" ontouchstart="startCmd('/backward')" ontouchend="stopCmd()">Reverse</button>
    </div>
    <div class='row' style='margin-top:20px;'>
        <button onclick="emergencyStop()" style='background-color:#ff0000;color:white;font-weight:bold;'>EMERGENCY STOP</button>
    </div>
    <div class='row' style='margin-top:15px;'>
        <label>Ramping Mode:</label><br>
        <button onclick="setRampSpeed(false)" style='background-color:#4CAF50;color:white;margin:2px;'>Smooth</button>
        <button onclick="setRampSpeed(true)" style='background-color:#FF9800;color:white;margin:2px;'>Fast</button>
    </div>
    <p style='margin-top:30px;color:gray;'>PicoW IP: {ip}</p>
</body>
</html>
""")
            except Exception as e:
                cl.send('HTTP/1.0 400 Bad Request\r\nContent-type: text/html\r\n\r\n')
                cl.send(f"<html><body><h2>Error: {e}</h2></body></html>")
            finally:
                cl.close()
    finally:
        s.close()

# Run the server
if __name__ == "__main__":
    start_server()
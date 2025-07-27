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

# Improved motor functions: no timed movement, only stop when /stop is called
def motor_forward(speed=40000):
    in1.high(); in2.low()
    in3.high(); in4.low()
    ena_pwm.duty_u16(speed)
    enb_pwm.duty_u16(speed)

def motor_backward(speed=40000):
    in1.low(); in2.high()
    in3.low(); in4.high()
    ena_pwm.duty_u16(speed)
    enb_pwm.duty_u16(speed)

def rotate_right(speed=40000):
    in1.low(); in2.high()
    in3.high(); in4.low()
    ena_pwm.duty_u16(speed)
    enb_pwm.duty_u16(speed)

def rotate_left(speed=40000):
    in1.high(); in2.low()
    in3.low(); in4.high()
    ena_pwm.duty_u16(speed)
    enb_pwm.duty_u16(speed)

def motor_stop():
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
    return int(percent * 65535 // 100)

# Parse speed from query string
def get_speed_from_path(path):
    import ure
    match = ure.search(r'speed=(\d+)', path)
    if match:
        return percent_to_pwm(match.group(1))
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
            cl, addr = s.accept()
            try:
                req = cl.recv(1024).decode()
                path = req.split(' ')[1] if ' ' in req else '/'
                cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
                # Remap button actions to correct reversed hardware behavior
                speed = get_speed_from_path(path)
                if path.startswith('/forward'):
                    motor_backward(speed)
                elif path.startswith('/backward'):
                    motor_forward(speed)
                elif path.startswith('/left'):
                    rotate_right(speed)
                elif path.startswith('/right'):
                    rotate_left(speed)
                elif path.startswith('/stop'):
                    motor_stop()
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
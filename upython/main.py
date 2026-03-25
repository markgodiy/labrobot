"""
Labrobot Pico W — Combined Motor Control + Battery Monitor
===========================================================

Serial protocol (USB, 115200 baud):
  Pi → Pico:  {"cmd": "<command>", ...}\n
  Pico → Pi:  {"status": "ok"|"error", ...}\n
              {"type": "status"|"heartbeat"|"battery"|"log", ...}\n

Motor A (Left):  ENA=GP3, IN1=GP4, IN2=GP5
Motor B (Right): IN3=GP11, IN4=GP12, ENB=GP13
H-Bridge STBY:   tied HIGH on PCB (no GPIO control needed)

Encoder Left:    A=GP16, B=GP17  ← confirmed by pin_discover.py
Encoder Right:   A=GP18, B=GP19  ← confirmed by pin_discover.py
INA219 (I2C1):   SDA=GP14, SCL=GP15
"""

import gc
import select
import sys
import ujson
from machine import ADC, I2C, PWM, Pin, Timer
from time import sleep, sleep_ms, ticks_diff, ticks_ms

# Non-blocking stdin poll (sys.stdin.any() not available on MicroPython v1.26+)
_stdin_poll = select.poll()
_stdin_poll.register(sys.stdin, select.POLLIN)

# ---------------------------------------------------------------------------
# Robot physical parameters
# ---------------------------------------------------------------------------
WHEEL_DIAMETER_MM = 65.0
WHEEL_BASE_MM = 347.0          # center-to-center from URDF (was 150 — wrong)
PULSES_PER_REV = 3436
WHEEL_CIRCUMFERENCE_MM = 3.14159265 * WHEEL_DIAMETER_MM
MM_PER_PULSE = WHEEL_CIRCUMFERENCE_MM / PULSES_PER_REV

# ---------------------------------------------------------------------------
# Motor hardware
# ---------------------------------------------------------------------------
led = Pin("LED", Pin.OUT)

ena_pwm = PWM(Pin(3)); ena_pwm.freq(1000)   # Left enable
in1 = Pin(4, Pin.OUT)                        # Left dir 1
in2 = Pin(5, Pin.OUT)                        # Left dir 2

in3 = Pin(11, Pin.OUT)                       # Right dir 1
in4 = Pin(12, Pin.OUT)                       # Right dir 2
enb_pwm = PWM(Pin(13)); enb_pwm.freq(1000)  # Right enable

# ---------------------------------------------------------------------------
# Encoder hardware
# ---------------------------------------------------------------------------
left_enc_a  = Pin(16, Pin.IN, Pin.PULL_UP)
left_enc_b  = Pin(17, Pin.IN, Pin.PULL_UP)

right_enc_a = Pin(18, Pin.IN, Pin.PULL_UP)
right_enc_b = Pin(19, Pin.IN, Pin.PULL_UP)

# ---------------------------------------------------------------------------
# INA219 battery sensor (optional — degrades gracefully if absent)
# ---------------------------------------------------------------------------
_ina219_sensor = None
try:
    from ina219 import INA219
    _i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=100000)
    _ina219_sensor = INA219(_i2c, addr=0x40, r_shunt=0.00375)
except Exception as _e:
    pass  # battery monitoring unavailable


# ---------------------------------------------------------------------------
# Encoder state
# ---------------------------------------------------------------------------
class EncoderState:
    __slots__ = ("left_count", "right_count")

    def __init__(self):
        self.left_count  = 0
        self.right_count = 0


enc = EncoderState()


# Simple directional edge counting on A-channel only.
# nav.left_dir / nav.right_dir are set to +1/-1 by _move()/_rotate()
# and cleared to 0 by _smooth_stop()/_emergency_stop().
# Avoids the soft-IRQ timing problem: quadrature phase offset (~250 µs) is
# shorter than the MicroPython soft-IRQ delivery delay, so quadrature decoding
# in a handler always sees both channels in the same state → net 0 count.
def _update_left(_pin):
    enc.left_count += nav.left_dir


def _update_right(_pin):
    enc.right_count += nav.right_dir


def _encoder_data():
    l = enc.left_count  * MM_PER_PULSE
    r = enc.right_count * MM_PER_PULSE
    return {
        "left_count":        enc.left_count,
        "right_count":       enc.right_count,
        "left_distance_mm":  round(l, 2),
        "right_distance_mm": round(r, 2),
        "center_distance_mm": round((l + r) / 2.0, 2),
        "rotation_rad":      round((r - l) / WHEEL_BASE_MM, 5),
    }


# ---------------------------------------------------------------------------
# Navigation state
# ---------------------------------------------------------------------------
class NavState:
    def __init__(self):
        self.current_speed       = 0
        self.target_speed        = 0
        self.is_moving           = False
        self.autonomous_mode     = False
        self.estop               = True   # safe default — send reset_estop to enable
        self.last_cmd_time       = ticks_ms()
        self.cmd_timeout_ms      = 3000
        self.ramp_step           = 10000  # ~250ms 0→100% ramp (was 3000 = ~1s)
        self.ramp_delay_ms       = 50
        self.move_start_time     = 0
        self.move_duration       = 0
        self.rot_start_time      = 0
        self.rot_duration        = 0
        self.total_cmds          = 0
        self.total_errors        = 0
        self.serial_errors       = 0
        self.json_errors         = 0
        self.last_heartbeat_time = ticks_ms()
        self.heartbeat_ms        = 5000
        self.left_dir            = 0   # +1 forward, -1 backward, 0 stopped
        self.right_dir           = 0


nav = NavState()

# Attach encoder IRQs after nav is defined — handlers reference nav.left_dir/right_dir.
# B-channel IRQs removed — A-channel alone gives 2000 pulses/s per direction.
left_enc_a.irq( trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_update_left)
right_enc_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_update_right)


# ---------------------------------------------------------------------------
# Battery state
# ---------------------------------------------------------------------------
BATTERY_WH = 120.0
BATTERY_AH = 10.0
_BATT_SAMPLE_MS = 10_000   # read INA219 every 10 s

class BattState:
    def __init__(self):
        self.available       = _ina219_sensor is not None
        self.last_sample_ms  = ticks_ms()
        self.energy_wh       = 0.0
        self.ah_used         = 0.0
        self.smoothed_power  = 0.0
        self.peak_current    = 0.0
        self.last_time_s     = None
        self.last_reading    = {}
        # restore persisted counters
        try:
            import uos as os
            with open("power_state.json", "r") as f:
                st = ujson.load(f)
                self.energy_wh = float(st.get("energy_wh_total", 0.0))
                self.ah_used   = float(st.get("ah_used", 0.0))
        except Exception:
            pass


batt = BattState()


def _estimate_soc(v):
    thresholds = [(13.3,1.0),(13.0,0.9),(12.8,0.8),(12.5,0.7),(12.3,0.6),
                  (12.0,0.5),(11.8,0.4),(11.5,0.3),(11.3,0.2),(11.0,0.1)]
    for vt, soc in thresholds:
        if v >= vt:
            return soc
    return 0.05


def _format_hhmm(hours):
    if hours is None or hours < 0:
        return None
    s = int(hours * 3600)
    return "{:02d}:{:02d}".format(s // 3600, (s % 3600) // 60)


def _sample_battery():
    """Read INA219 and update BattState. Returns reading dict or {}."""
    if not batt.available:
        return {}
    import time as _time
    try:
        voltage = _ina219_sensor.bus_voltage()
        current = _ina219_sensor.system_draw_amp()
        power   = _ina219_sensor.system_power_watt()
        now_s   = _time.time()

        dt = (now_s - batt.last_time_s) if batt.last_time_s is not None else 1.0
        dt = max(dt, 0.001)
        batt.last_time_s = now_s

        batt.energy_wh      += power * (dt / 3600.0)
        batt.ah_used        += max(0.0, current) * (dt / 3600.0)
        batt.peak_current    = max(batt.peak_current, abs(current))

        alpha = dt / (10.0 + dt)
        batt.smoothed_power  = alpha * power + (1 - alpha) * batt.smoothed_power

        soc       = _estimate_soc(voltage)
        rem_wh    = soc * BATTERY_WH
        hrs_left  = (rem_wh / batt.smoothed_power) if batt.smoothed_power > 0.001 else None

        batt.last_reading = {
            "type":                   "battery",
            "bus_voltage":            round(voltage, 2),
            "current_draw_amp":       round(current, 3),
            "current_draw_watt":      round(power, 2),
            "batt_charge_percent":    int(round(soc * 100)),
            "time_remaining":         _format_hhmm(hrs_left),
            "energy_wh_total":        round(batt.energy_wh, 4),
            "ah_used":                round(batt.ah_used, 4),
            "smoothed_power_w":       round(batt.smoothed_power, 3),
            "peak_current_a":         round(batt.peak_current, 3),
            "charging":               current < 0,
            "c_rate":                 round(abs(current) / BATTERY_AH, 3),
        }
    except Exception as e:
        batt.last_reading = {"type": "battery", "error": str(e)}
    return batt.last_reading


def _persist_battery():
    try:
        with open("power_state.json.tmp", "w") as f:
            ujson.dump({"energy_wh_total": batt.energy_wh, "ah_used": batt.ah_used}, f)
        import uos as os
        try:
            os.replace("power_state.json.tmp", "power_state.json")
        except AttributeError:
            try:
                os.remove("power_state.json")
            except Exception:
                pass
            os.rename("power_state.json.tmp", "power_state.json")
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------
def _send(d):
    try:
        print(ujson.dumps(d))
        sys.stdout.flush()
    except Exception:
        pass


def _log(msg, level="INFO"):
    _send({"type": "log", "level": level, "message": msg, "ts": ticks_ms()})


# ---------------------------------------------------------------------------
# Safety timer
# ---------------------------------------------------------------------------
def _safety_check(_t):
    if nav.autonomous_mode and not nav.estop:
        if ticks_diff(ticks_ms(), nav.last_cmd_time) > nav.cmd_timeout_ms:
            _emergency_stop()


_safety_timer = Timer(-1)
_safety_timer.init(period=1000, mode=Timer.PERIODIC, callback=_safety_check)


# ---------------------------------------------------------------------------
# Motor control
# ---------------------------------------------------------------------------
def _set_dir(left_fn, right_fn):
    left_fn(); right_fn()


def _lf(): in1.high(); in2.low()
def _lb(): in1.low();  in2.high()
def _ls(): in1.low();  in2.low()
def _rf(): in3.high(); in4.low()
def _rb(): in3.low();  in4.high()
def _rs(): in3.low();  in4.low()


def _pct_to_pwm(pct):
    pct = max(0, min(100, int(pct)))
    if 0 < pct < 30:
        pct = 30
    return int(pct * 65535 // 100)


_ANTISTICTION_PWM = int(95 * 65535 // 100)  # high-power pulse to break left-motor backward stiction


def _ramp(new_speed, left_fn, right_fn):
    nav.target_speed = new_speed
    # Anti-stiction: left motor has high static friction in the backward direction
    # when starting from rest. A brief (~30ms) forward pulse frees the brushes so
    # the ramp can then accelerate smoothly backward.
    # Only fires when: starting from rest AND left motor is going backward.
    if nav.current_speed == 0 and new_speed > 0 and left_fn is _lb:
        saved_left_dir = nav.left_dir   # -1 (backward) — restore after pulse
        nav.left_dir = 1                # count forward pulse correctly
        _set_dir(_lf, _rs)              # left fwd, right off
        ena_pwm.duty_u16(_ANTISTICTION_PWM)
        enb_pwm.duty_u16(0)
        sleep_ms(150)                   # 150ms forward pulse — warms brushes; longer gives same counts (stall-limited)
        # Disable PWM before direction change — no shoot-through since PWM=0.
        # Skip the brake dead-time: a hard stop re-engages the stiction we just broke.
        ena_pwm.duty_u16(0)
        nav.left_dir = saved_left_dir   # restore -1 so encoder counts backward motion
        # Jump straight to target speed — gradual ramp can't restart a cold brush.
        nav.current_speed = nav.target_speed - 1
    while nav.current_speed != nav.target_speed:
        if nav.estop:
            return
        if nav.current_speed < nav.target_speed:
            nav.current_speed = min(nav.current_speed + nav.ramp_step, nav.target_speed)
        else:
            nav.current_speed = max(nav.current_speed - nav.ramp_step, nav.target_speed)
        _set_dir(left_fn, right_fn)
        ena_pwm.duty_u16(nav.current_speed)
        enb_pwm.duty_u16(nav.current_speed)
        sleep_ms(nav.ramp_delay_ms)


def _move(speed, lf, rf, duration):
    if nav.estop:
        return False
    nav.left_dir        = 1 if lf is _lf else -1
    nav.right_dir       = 1 if rf is _rf else -1
    nav.is_moving       = True
    nav.move_start_time = ticks_ms()
    nav.move_duration   = int(duration * 1000) if duration > 0 else 0
    _ramp(_pct_to_pwm(speed), lf, rf)
    led.on()
    return True


def _rotate(speed, lf, rf, duration):
    if nav.estop:
        return False
    nav.left_dir       = 1 if lf is _lf else -1
    nav.right_dir      = 1 if rf is _rf else -1
    nav.is_moving      = True
    nav.rot_start_time = ticks_ms()
    nav.rot_duration   = int(duration * 1000) if duration > 0 else 0
    _ramp(_pct_to_pwm(speed), lf, rf)
    led.on()
    return True


def _smooth_stop():
    nav.left_dir      = 0
    nav.right_dir     = 0
    nav.is_moving     = False
    nav.move_duration = 0
    nav.rot_duration  = 0
    _ramp(0, _ls, _rs)
    nav.current_speed = 0
    # LED stays on — operational state (estop cleared) persists after stop


def _emergency_stop():
    nav.left_dir      = 0
    nav.right_dir     = 0
    nav.estop         = True
    nav.is_moving     = False
    nav.current_speed = 0
    nav.target_speed  = 0
    ena_pwm.duty_u16(0)
    enb_pwm.duty_u16(0)
    _ls(); _rs()
    for _ in range(6):
        led.toggle(); sleep_ms(100)
    led.off()
    _log("EMERGENCY STOP ACTIVATED", "WARN")


# ---------------------------------------------------------------------------
# Pico system helpers
# ---------------------------------------------------------------------------
_temp_adc = ADC(4)

def _cpu_temp_c():
    v = _temp_adc.read_u16() * (3.3 / 65535)
    return round(27 - (v - 0.706) / 0.001721, 1)


# ---------------------------------------------------------------------------
# Status / heartbeat builders
# ---------------------------------------------------------------------------
def _status_dict():
    now = ticks_ms()
    d = {
        "type":              "status",
        "ts":                now,
        "autonomous_mode":   nav.autonomous_mode,
        "is_moving":         nav.is_moving,
        "current_speed":     nav.current_speed,
        "emergency_stop":    nav.estop,
        "last_cmd_age_ms":   ticks_diff(now, nav.last_cmd_time),
        "health":            "emergency_stop" if nav.estop else "healthy",
        "encoder":           _encoder_data(),
        "battery":           batt.last_reading if batt.available else None,
        "version":           "3.0.0",
        "cpu_temp_c":        _cpu_temp_c(),
        "led_on":            bool(led.value()),
    }
    return d


def _heartbeat_dict():
    now = ticks_ms()
    return {
        "type":       "heartbeat",
        "ts":         now,
        "uptime_ms":  now,
        "health":     "emergency_stop" if nav.estop else "healthy",
        "is_moving":  nav.is_moving,
        "cmds":       nav.total_cmds,
        "errors":     nav.total_errors,
        "encoder":    _encoder_data(),
        "battery":    batt.last_reading if batt.available else None,
        "cpu_temp_c": _cpu_temp_c(),
        "led_on":     bool(led.value()),
    }


# ---------------------------------------------------------------------------
# Command dispatcher
# ---------------------------------------------------------------------------
def _handle(cmd_dict):
    nav.last_cmd_time = ticks_ms()
    nav.total_cmds   += 1
    cmd = cmd_dict.get("cmd", "").lower()

    try:
        if cmd == "ping":
            return {"status": "ok", "message": "pong", "ts": ticks_ms()}

        elif cmd == "move":
            dir_  = cmd_dict.get("dir", "forward")
            speed = cmd_dict.get("speed", 60)
            dur   = cmd_dict.get("duration", 0)
            if dir_ not in ("forward", "backward"):
                return {"status": "error", "message": f"bad dir: {dir_}"}
            if not (0 <= speed <= 100):
                return {"status": "error", "message": f"bad speed: {speed}"}
            fns = (_lf, _rf) if dir_ == "forward" else (_lb, _rb)
            ok = _move(speed, fns[0], fns[1], dur)
            return {"status": "ok" if ok else "error",
                    "message": f"Moving {dir_}" if ok else "estop active"}

        elif cmd == "rotate":
            dir_  = cmd_dict.get("dir", "left")
            speed = cmd_dict.get("speed", 50)
            dur   = cmd_dict.get("duration", 0)
            if dir_ not in ("left", "right"):
                return {"status": "error", "message": f"bad dir: {dir_}"}
            fns = (_lb, _rf) if dir_ == "left" else (_lf, _rb)
            ok = _rotate(speed, fns[0], fns[1], dur)
            return {"status": "ok" if ok else "error",
                    "message": f"Rotating {dir_}" if ok else "estop active"}

        elif cmd == "set_speed":
            # Direct per-motor PWM for continuous velocity streaming.
            # No ramp, no blocking — encoder counting still works because
            # left_dir/right_dir are set before any IRQ fires.
            left_dir  = int(cmd_dict.get("left_dir",  0))  # +1 fwd, -1 back, 0 stop
            right_dir = int(cmd_dict.get("right_dir", 0))
            speed     = int(cmd_dict.get("speed",     0))  # 0–100 %
            if nav.estop:
                return {"status": "error", "message": "estop active"}
            nav.left_dir      = left_dir
            nav.right_dir     = right_dir
            nav.is_moving     = (left_dir != 0 or right_dir != 0)
            nav.move_duration = 0   # disable timed auto-stop
            nav.rot_duration  = 0
            if left_dir > 0:   _lf()
            elif left_dir < 0: _lb()
            else:              _ls()
            if right_dir > 0:   _rf()
            elif right_dir < 0: _rb()
            else:               _rs()
            pwm = _pct_to_pwm(speed) if speed > 0 else 0
            ena_pwm.duty_u16(pwm if left_dir  != 0 else 0)
            enb_pwm.duty_u16(pwm if right_dir != 0 else 0)
            nav.current_speed = pwm
            if nav.is_moving:
                led.on()
            return {"status": "ok"}

        elif cmd == "stop":
            _smooth_stop()
            return {"status": "ok", "message": "stopped"}

        elif cmd == "estop":
            _emergency_stop()
            return {"status": "ok", "message": "emergency stop activated"}

        elif cmd == "reset_estop":
            nav.estop = False
            led.on()
            _log("Emergency stop reset")
            return {"status": "ok", "message": "estop reset"}

        elif cmd == "autonomous":
            mode = cmd_dict.get("mode", "on")
            nav.autonomous_mode = (mode == "on")
            return {"status": "ok", "message": f"autonomous: {nav.autonomous_mode}"}

        elif cmd == "status":
            return _status_dict()

        elif cmd == "get_encoders":
            return {"status": "ok", "encoder_data": _encoder_data()}

        elif cmd == "reset_encoders":
            enc.left_count = 0; enc.right_count = 0
            return {"status": "ok", "message": "encoders reset"}

        elif cmd == "get_battery":
            reading = _sample_battery()
            batt.last_sample_ms = ticks_ms()
            return reading if reading else {"status": "error", "message": "battery unavailable"}

        elif cmd == "diagnostics":
            return {
                "type":          "diagnostics",
                "ts":            ticks_ms(),
                "total_cmds":    nav.total_cmds,
                "total_errors":  nav.total_errors,
                "serial_errors": nav.serial_errors,
                "json_errors":   nav.json_errors,
                "mem_free":      gc.mem_free(),
                "battery_avail": batt.available,
            }

        else:
            return {"status": "error", "message": f"unknown cmd: {cmd}",
                    "available": ["ping","move","rotate","set_speed","stop","estop",
                                  "reset_estop","autonomous","status","get_encoders",
                                  "reset_encoders","get_battery","diagnostics"]}
    except Exception as e:
        nav.total_errors += 1
        return {"status": "error", "message": str(e)}


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def _main():
    # Startup LED
    for _ in range(3):
        led.toggle(); sleep(0.2)
    led.off()

    _log("Labrobot Pico W v3.0.0 started")
    _log(f"battery_sensor={'ok' if batt.available else 'unavailable'}")
    _log("estop active — send reset_estop to enable motors", "WARN")
    if batt.available:
        _sample_battery()   # populate batt.last_reading before first status
    _send(_status_dict())

    _last_persist_ms = ticks_ms()

    while True:
        now = ticks_ms()

        # --- timed movement auto-stop ---
        if nav.is_moving:
            if nav.move_duration > 0 and ticks_diff(now, nav.move_start_time) >= nav.move_duration:
                _smooth_stop()
            elif nav.rot_duration > 0 and ticks_diff(now, nav.rot_start_time) >= nav.rot_duration:
                _smooth_stop()

        # --- serial input ---
        try:
            if _stdin_poll.poll(0):
                line = sys.stdin.readline().strip()
                if line:
                    try:
                        _send(_handle(ujson.loads(line)))
                    except ValueError as e:
                        nav.json_errors += 1
                        _send({"status": "error", "message": f"bad JSON: {e}"})
        except Exception as e:
            nav.serial_errors += 1

        # --- heartbeat ---
        if ticks_diff(now, nav.last_heartbeat_time) >= nav.heartbeat_ms:
            nav.last_heartbeat_time = now
            _send(_heartbeat_dict())

        # --- battery sample ---
        if batt.available and ticks_diff(now, batt.last_sample_ms) >= _BATT_SAMPLE_MS:
            batt.last_sample_ms = now
            reading = _sample_battery()
            if reading:
                _send(reading)   # broadcast as type:"battery"

        # --- persist battery counters every 5 min ---
        if ticks_diff(now, _last_persist_ms) >= 300_000:
            _last_persist_ms = now
            if batt.available:
                _persist_battery()

        gc.collect()
        sleep_ms(10)


_main()

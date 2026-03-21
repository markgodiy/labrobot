#!/usr/bin/env python3
"""
Robot Dashboard
==============
ROS 2 node that serves a live web dashboard showing robot metrics.
Access at  http://<robot-ip>:8080

Subscribes to:
  /motor_controller/status       std_msgs/String  — battery + estop state
  /motor_controller/encoder_data std_msgs/String  — raw encoder counts
  /odom                          nav_msgs/Odometry

Serves:
  GET /        — HTML dashboard (auto-refreshes at 500 ms via JS)
  GET /metrics — JSON snapshot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import json
import math
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

# ── shared metrics (written by ROS callbacks, read by HTTP thread) ────────────
_lock = threading.Lock()
_metrics: dict = {
    "ts": 0,
    "bridge_connected": False,
    "odom":     {"x": 0.0, "y": 0.0, "heading_deg": 0.0, "vx": 0.0, "wz": 0.0, "ts": 0},
    "encoders": {"left": 0, "right": 0, "ts": 0},
    "battery":  {"voltage": None, "current_mA": None, "power_mW": None, "ts": 0},
    "status":   {"emergency_stop": None, "ts": 0},
    "system":   {"cpu_temp_c": None, "mem_used_mb": None, "mem_total_mb": None, "uptime_s": 0},
}


def _read_sys_metrics() -> dict:
    out: dict = {}
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            out["cpu_temp_c"] = round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        out["cpu_temp_c"] = None
    try:
        mem: dict = {}
        with open("/proc/meminfo") as f:
            for line in f:
                parts = line.split()
                if parts[0] in ("MemTotal:", "MemAvailable:"):
                    mem[parts[0]] = int(parts[1])
        total = mem.get("MemTotal:", 0)
        avail = mem.get("MemAvailable:", 0)
        out["mem_total_mb"] = round(total / 1024)
        out["mem_used_mb"]  = round((total - avail) / 1024)
    except Exception:
        out["mem_total_mb"] = None
        out["mem_used_mb"]  = None
    try:
        with open("/proc/uptime") as f:
            out["uptime_s"] = int(float(f.read().split()[0]))
    except Exception:
        out["uptime_s"] = 0
    return out


# ── HTML ──────────────────────────────────────────────────────────────────────
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Dashboard</title>
<style>
  :root {
    --bg: #0f1117; --card: #1a1d27; --border: #2a2d3a;
    --text: #e2e8f0; --muted: #8892a4;
    --green: #22c55e; --red: #ef4444; --yellow: #eab308; --blue: #3b82f6;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text);
         font-family: 'Segoe UI', system-ui, sans-serif; padding: 16px; }
  header { display: flex; align-items: center; justify-content: space-between;
           margin-bottom: 20px; padding-bottom: 12px; border-bottom: 1px solid var(--border); }
  header h1 { font-size: 1.25rem; font-weight: 600; }
  .badge { font-size: .8rem; color: var(--muted); display: flex; align-items: center; gap: 6px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--muted); }
  .dot.green  { background: var(--green); box-shadow: 0 0 6px var(--green); }
  .dot.red    { background: var(--red); }
  .dot.yellow { background: var(--yellow); }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(270px, 1fr)); gap: 14px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }
  .card-title { font-size: .68rem; font-weight: 700; letter-spacing: 1px;
                text-transform: uppercase; color: var(--muted); margin-bottom: 14px; }
  .row { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 9px; }
  .lbl  { font-size: .8rem; color: var(--muted); }
  .val  { font-size: 1rem; font-weight: 600; font-variant-numeric: tabular-nums; }
  .unit { font-size: .7rem; color: var(--muted); margin-left: 2px; }
  .big  { font-size: 1.45rem; }
  .green { color: var(--green); } .red { color: var(--red); }
  .yellow{ color: var(--yellow);} .muted{ color: var(--muted); }
  .stale { opacity: .35; }
  hr { border: none; border-top: 1px solid var(--border); margin: 10px 0; }
  footer { font-size: .7rem; color: var(--muted); text-align: right; margin-top: 14px; }
</style>
</head>
<body>
<header>
  <h1>&#128421; Robot Dashboard</h1>
  <div class="badge"><span class="dot yellow" id="conn-dot"></span><span id="conn-text">Connecting…</span></div>
</header>

<div class="grid">

  <!-- Odometry -->
  <div class="card">
    <div class="card-title">Odometry</div>
    <div class="row"><span class="lbl">X</span>      <span><span class="val" id="ox">—</span><span class="unit">m</span></span></div>
    <div class="row"><span class="lbl">Y</span>      <span><span class="val" id="oy">—</span><span class="unit">m</span></span></div>
    <div class="row"><span class="lbl">Heading</span><span><span class="val" id="oh">—</span><span class="unit">deg</span></span></div>
    <hr>
    <div class="row"><span class="lbl">Linear vel</span> <span><span class="val" id="ovx">—</span><span class="unit">m/s</span></span></div>
    <div class="row"><span class="lbl">Angular vel</span><span><span class="val" id="owz">—</span><span class="unit">rad/s</span></span></div>
  </div>

  <!-- Encoders + bridge -->
  <div class="card">
    <div class="card-title">Encoders</div>
    <div class="row"><span class="lbl">Left count</span> <span><span class="val" id="el">—</span><span class="unit">ticks</span></span></div>
    <div class="row"><span class="lbl">Right count</span><span><span class="val" id="er">—</span><span class="unit">ticks</span></span></div>
    <hr>
    <div class="row"><span class="lbl">Bridge</span><span class="val" id="bridge">—</span></div>
    <div class="row"><span class="lbl">E-Stop</span><span class="val" id="estop">—</span></div>
  </div>

  <!-- Battery -->
  <div class="card">
    <div class="card-title">Battery (INA219)</div>
    <div class="row"><span class="lbl">Voltage</span><span><span class="val big" id="bv">—</span><span class="unit">V</span></span></div>
    <div class="row"><span class="lbl">Current</span><span><span class="val" id="bi">—</span><span class="unit">mA</span></span></div>
    <div class="row"><span class="lbl">Power</span>  <span><span class="val" id="bp">—</span><span class="unit">mW</span></span></div>
  </div>

  <!-- System -->
  <div class="card">
    <div class="card-title">Pi System</div>
    <div class="row"><span class="lbl">CPU temp</span><span><span class="val" id="ct">—</span><span class="unit">°C</span></span></div>
    <div class="row"><span class="lbl">Memory</span> <span class="val" id="mem">—</span></div>
    <div class="row"><span class="lbl">Uptime</span> <span class="val" id="up">—</span></div>
  </div>

</div>
<footer id="ts">Last update: —</footer>

<script>
const $ = id => document.getElementById(id);
let fails = 0;

const fmt = (v, d=3) => (v == null) ? '—' : Number(v).toFixed(d);
const age  = ts  => ts  ? (Date.now()/1000 - ts) : 9999;
const stale = (el, ts, maxAge=3) => el.classList.toggle('stale', age(ts) > maxAge);

function uptime(s) {
  if (!s) return '—';
  const h = Math.floor(s/3600), m = Math.floor(s%3600/60), sec = s%60;
  return h ? `${h}h ${m}m` : m ? `${m}m ${sec}s` : `${sec}s`;
}

async function poll() {
  try {
    const r = await fetch('/metrics', {signal: AbortSignal.timeout(2000)});
    if (!r.ok) throw new Error();
    const d = await r.json();
    fails = 0;

    // connection badge
    const dot = $('conn-dot'), txt = $('conn-text');
    dot.className = 'dot ' + (d.bridge_connected ? 'green' : 'red');
    txt.textContent = d.bridge_connected ? 'Bridge connected' : 'Bridge disconnected';

    // odometry
    const o = d.odom;
    $('ox').textContent  = fmt(o.x);
    $('oy').textContent  = fmt(o.y);
    $('oh').textContent  = fmt(o.heading_deg, 1);
    $('ovx').textContent = fmt(o.vx);
    $('owz').textContent = fmt(o.wz);
    stale($('ox'), o.ts);

    // encoders
    const enc = d.encoders;
    $('el').textContent = enc.ts > 0 ? fmt(enc.left,  0) : '—';
    $('er').textContent = enc.ts > 0 ? fmt(enc.right, 0) : '—';
    stale($('el'), enc.ts);

    // bridge + estop
    $('bridge').textContent = d.bridge_connected ? 'Connected' : 'Disconnected';
    $('bridge').className   = 'val ' + (d.bridge_connected ? 'green' : 'red');
    const es = d.status.emergency_stop;
    $('estop').textContent = es == null ? '—' : (es ? 'ACTIVE' : 'OK');
    $('estop').className   = 'val ' + (es === true ? 'red' : es === false ? 'green' : 'muted');
    stale($('estop'), d.status.ts);

    // battery
    const b = d.battery;
    $('bv').textContent = b.voltage    != null ? fmt(b.voltage, 2)    : '—';
    $('bi').textContent = b.current_mA != null ? fmt(b.current_mA, 0) : '—';
    $('bp').textContent = b.power_mW   != null ? fmt(b.power_mW, 0)   : '—';
    const vc = b.voltage == null ? '' : b.voltage < 11 ? 'red' : b.voltage < 12 ? 'yellow' : 'green';
    $('bv').className = 'val big ' + vc;
    stale($('bv'), b.ts, 20);

    // system
    const s = d.system;
    $('ct').textContent  = s.cpu_temp_c != null ? fmt(s.cpu_temp_c, 1) : '—';
    $('ct').className    = 'val ' + (s.cpu_temp_c == null ? '' : s.cpu_temp_c > 75 ? 'red' : s.cpu_temp_c > 60 ? 'yellow' : 'green');
    $('mem').textContent = s.mem_used_mb != null ? `${s.mem_used_mb} / ${s.mem_total_mb} MB` : '—';
    $('up').textContent  = uptime(s.uptime_s);

    $('ts').textContent  = 'Last update: ' + new Date().toLocaleTimeString();
  } catch(e) {
    if (++fails >= 3) {
      $('conn-dot').className = 'dot red';
      $('conn-text').textContent = 'Dashboard unreachable';
    }
  }
}

poll();
setInterval(poll, 500);
</script>
</body>
</html>"""


# ── HTTP handler ──────────────────────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass  # suppress per-request console noise

    def do_GET(self):
        if self.path == '/metrics':
            with _lock:
                body = json.dumps(_metrics).encode()
            self._respond(200, 'application/json', body)
        elif self.path in ('/', '/index.html'):
            self._respond(200, 'text/html; charset=utf-8', DASHBOARD_HTML.encode())
        else:
            self.send_response(404); self.end_headers()

    def _respond(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)


# ── ROS 2 node ────────────────────────────────────────────────────────────────
class RobotDashboardNode(Node):
    def __init__(self):
        super().__init__('robot_dashboard')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        self.create_subscription(String,   '/motor_controller/status',       self._on_status,   10)
        self.create_subscription(String,   '/motor_controller/encoder_data', self._on_encoders, 10)
        self.create_subscription(Odometry, '/odom',                          self._on_odom,     10)

        self.create_timer(5.0, self._update_sys)
        self._update_sys()

        server = HTTPServer(('', port), _Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Dashboard → http://0.0.0.0:{port}')

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        with _lock:
            _metrics['bridge_connected'] = True
            _metrics['ts'] = now
            estop = data.get('emergency_stop')
            if estop is not None:
                _metrics['status']['emergency_stop'] = estop
                _metrics['status']['ts'] = now
            # battery may arrive as 'batt' or 'battery' sub-dict
            batt = data.get('batt') or data.get('battery')
            if batt:
                _metrics['battery']['voltage']    = batt.get('bus_voltage')
                _metrics['battery']['current_mA'] = batt.get('current_mA') or batt.get('current')
                _metrics['battery']['power_mW']   = batt.get('power_mW')   or batt.get('power')
                _metrics['battery']['ts'] = now

    def _on_encoders(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        with _lock:
            _metrics['encoders']['left']  = data.get('left_count',  0)
            _metrics['encoders']['right'] = data.get('right_count', 0)
            _metrics['encoders']['ts'] = now

    def _on_odom(self, msg: Odometry):
        now = time.time()
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        heading_deg = math.degrees(2.0 * math.atan2(qz, qw))
        with _lock:
            _metrics['odom']['x']           = round(msg.pose.pose.position.x, 4)
            _metrics['odom']['y']           = round(msg.pose.pose.position.y, 4)
            _metrics['odom']['heading_deg'] = round(heading_deg, 2)
            _metrics['odom']['vx']          = round(msg.twist.twist.linear.x,  4)
            _metrics['odom']['wz']          = round(msg.twist.twist.angular.z, 4)
            _metrics['odom']['ts'] = now

    def _update_sys(self):
        sys_data = _read_sys_metrics()
        with _lock:
            _metrics['system'].update(sys_data)


# ── entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotDashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

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

Publishes:
  /cmd_vel                       geometry_msgs/Twist — teleop commands from dashboard

Serves:
  GET  /         — HTML dashboard (auto-refreshes at 500 ms via JS)
  GET  /metrics  — JSON snapshot
  GET  /remote   — Mobile-optimised remote control page
  GET  /logs     — Log ring-buffer (JSON, ?since=N for incremental)
  GET  /logs/download — Full log as plain-text file
  POST /cmd      — Publish a Twist or estop  {"linear":0.2,"angular":0.0}
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import json
import math
import threading
import time
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# ── shared metrics (written by ROS callbacks, read by HTTP thread) ────────────
_lock = threading.Lock()

# ── node reference (set when node starts, used by HTTP handler) ───────────────
_node: 'RobotDashboardNode | None' = None

# ── log ring-buffer ───────────────────────────────────────────────────────────
_log_lock   = threading.Lock()
_log_buffer: deque = deque(maxlen=500)
_log_total  = 0   # monotonic count, never resets
_metrics: dict = {
    "ts": 0,
    "bridge_connected": False,
    "odom":     {"x": 0.0, "y": 0.0, "heading_deg": 0.0, "vx": 0.0, "wz": 0.0, "ts": 0},
    "encoders": {"left": 0, "right": 0, "ts": 0},
    "battery":  {"voltage": None, "current_mA": None, "power_mW": None, "soc_pct": None, "ts": 0},
    "status":   {"emergency_stop": None, "ts": 0},
    "system":   {"cpu_temp_c": None, "mem_used_mb": None, "mem_total_mb": None, "uptime_s": 0},
    "pico":     {"cpu_temp_c": None, "led_on": None, "uptime_ms": None, "cmds": None, "errors": None, "version": None, "ts": 0},
    "nav":      {"enabled": False, "action": "idle", "obstacle": False, "path_clear": True, "ts": 0},
    "lidar":    {"pts": [], "front_m": None, "left_m": None, "right_m": None, "ts": 0},
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
  /* car dashboard 3-column layout */
  .dash { display: grid; grid-template-columns: 260px 1fr 260px; gap: 14px; align-items: start; }
  .dash-col { display: flex; flex-direction: column; gap: 14px; }
  @media (max-width: 900px) { .dash { grid-template-columns: 1fr; } }
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
  #ts { font-size: .7rem; color: var(--muted); }
  /* tooltips */
  .tip { position: relative; cursor: help; text-decoration: underline dotted; text-underline-offset: 3px; }
  .tip::after {
    content: attr(data-tip);
    position: absolute; bottom: calc(100% + 7px); left: 0;
    background: #1e2130; color: #e2e8f0;
    font-size: .72rem; line-height: 1.4;
    padding: 6px 9px; border-radius: 7px;
    border: 1px solid #3a3f5c;
    width: max-content; max-width: 230px; white-space: normal;
    pointer-events: none; opacity: 0;
    transition: opacity .15s ease; z-index: 20;
    box-shadow: 0 4px 12px rgba(0,0,0,.5);
  }
  .tip:hover::after { opacity: 1; }
  /* nav toggle button */
  .nav-btn { background:#2a2d3a; color:var(--text); border:1px solid var(--border);
             border-radius:5px; padding:2px 8px; font-size:.72rem; cursor:pointer; margin-left:6px; }
  .nav-btn:hover { background:#3a3f5c; }
  .nav-btn.active { background:#1a3a1a; border-color:var(--green); color:var(--green); }
  /* lidar canvas — fills center card */
  #lidar-canvas { display:block; width:100%; height:auto; border-radius:6px; background:#0a0c14; }
  /* control card */
  .dpad-mini { display:grid; grid-template-columns:repeat(3,44px); grid-template-rows:repeat(3,44px); gap:5px; margin:10px auto; }
  .dc-btn { background:#1e2130; border:1px solid var(--border); border-radius:8px;
            display:flex; align-items:center; justify-content:center;
            font-size:1.2rem; cursor:pointer; user-select:none; transition:background .08s; }
  .dc-btn:active, .dc-btn.pressed { background:#2a3050; border-color:var(--blue); }
  .dc-stop { background:#2a1a1a !important; border-color:#7f1d1d !important; color:var(--red); }
  .dc-stop:active { background:#4a1a1a !important; }
  /* log panel */
  .log-panel { margin-top: 14px; background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 16px; }
  .log-header { display: flex; align-items: center; gap: 10px; margin-bottom: 10px; }
  .logbox { height: 300px; overflow-y: auto; font-family: 'Consolas', 'Courier New', monospace;
            font-size: .74rem; line-height: 1.6; background: #0a0c14;
            border-radius: 6px; padding: 8px 10px; }
  .log-line { white-space: pre-wrap; word-break: break-all; }
  .log-dbg { color: var(--muted); }
  .log-inf { color: #c8d3e0; }
  .log-wrn { color: var(--yellow); }
  .log-err { color: var(--red); }
  .log-btn { background: #2a2d3a; color: var(--text); border: 1px solid var(--border);
             border-radius: 6px; padding: 4px 10px; font-size: .75rem; cursor: pointer;
             text-decoration: none; display: inline-block; }
  .log-btn:hover { background: #3a3f5c; }
  .log-sel { background: #2a2d3a; color: var(--text); border: 1px solid var(--border);
             border-radius: 6px; padding: 4px 8px; font-size: .75rem; cursor: pointer; }
</style>
</head>
<body>
<header>
  <h1>&#128421; Robot Dashboard</h1>
  <span id="ts">Last update: —</span>
  <div class="badge"><span class="dot yellow" id="conn-dot"></span><span id="conn-text">Connecting…</span></div>
</header>

<div class="dash">

  <!-- ── LEFT: navigation + position + battery ── -->
  <div class="dash-col">

    <!-- Navigation -->
    <div class="card">
      <div class="card-title">Navigation</div>
      <div class="row">
        <span class="lbl">Mode</span>
        <span style="display:flex;align-items:center">
          <span class="val" id="nav-mode">—</span>
          <button id="nav-toggle" class="nav-btn" onclick="toggleNav()">Enable</button>
        </span>
      </div>
      <div class="row"><span class="lbl">Action</span><span class="val" id="nav-action">—</span></div>
      <div class="row"><span class="lbl">Path</span><span class="val" id="nav-path">—</span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="Min distance in ±45° forward arc">Front</span><span><span class="val" id="lidar-front">—</span><span class="unit">m</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Min distance on the left side">Left</span><span><span class="val" id="lidar-left">—</span><span class="unit">m</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Min distance on the right side">Right</span><span><span class="val" id="lidar-right">—</span><span class="unit">m</span></span></div>
    </div>

    <!-- Odometry -->
    <div class="card">
      <div class="card-title">Odometry</div>
      <div class="row"><span class="lbl tip" data-tip="Left/right distance from start">X</span><span><span class="val" id="ox">—</span><span class="unit">m</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Forward/backward distance from start">Y</span><span><span class="val" id="oy">—</span><span class="unit">m</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Heading: 0°=forward at boot, +90°=left, -90°=right">Heading</span><span><span class="val" id="oh">—</span><span class="unit">deg</span></span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="Forward speed. Positive=forward, negative=reverse">Speed</span><span><span class="val" id="ovx">—</span><span class="unit">m/s</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Rotation speed. Positive=left, negative=right">Turn rate</span><span><span class="val" id="owz">—</span><span class="unit">rad/s</span></span></div>
    </div>

    <!-- Battery -->
    <div class="card">
      <div class="card-title">Battery (INA219)</div>
      <div class="row"><span class="lbl tip" data-tip="Pack voltage. Green ≥12V, yellow ≥11V, red &lt;11V. Full ≈ 13.2V.">Voltage</span><span><span class="val big" id="bv">—</span><span class="unit">V</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Current draw in milliamps (whole robot).">Current</span><span><span class="val" id="bi">—</span><span class="unit">mA</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Instantaneous power (V × I).">Power</span><span><span class="val" id="bp">—</span><span class="unit">mW</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Estimated charge remaining (voltage-based).">Charge</span><span><span class="val" id="bsoc">—</span><span class="unit">%</span></span></div>
    </div>

  </div>

  <!-- ── CENTER: LIDAR radar + control ── -->
  <div class="dash-col">

    <!-- LIDAR radar -->
    <div class="card">
      <div class="card-title">LIDAR — Top-Down View <span style="font-weight:400;text-transform:none;letter-spacing:0;font-size:.7rem;color:var(--muted)">● red &lt;0.5 m &nbsp;● yellow &lt;1 m &nbsp;● green &gt;1 m</span></div>
      <canvas id="lidar-canvas" width="520" height="520"></canvas>
    </div>

    <!-- Control -->
    <div class="card">
      <div class="card-title">Control</div>
      <div style="display:flex;align-items:center;gap:8px;margin-bottom:4px">
        <span class="lbl">Speed</span>
        <input type="range" id="ctrl-spd" min="0.05" max="0.5" step="0.05" value="0.25"
               style="flex:1;accent-color:var(--blue)"
               oninput="ctrlSpeed=+this.value;$('ctrl-spd-val').textContent=ctrlSpeed.toFixed(2)">
        <span class="val muted" id="ctrl-spd-val" style="font-size:.8rem;min-width:28px">0.25</span>
      </div>
      <div class="dpad-mini">
        <div></div>
        <div class="dc-btn" id="dc-fwd">▲</div>
        <div></div>
        <div class="dc-btn" id="dc-left">◄</div>
        <div class="dc-btn dc-stop" id="dc-stop">■</div>
        <div class="dc-btn" id="dc-right">►</div>
        <div></div>
        <div class="dc-btn" id="dc-back">▼</div>
        <div></div>
      </div>
      <a href="/remote" target="_blank" style="display:block;text-align:center;font-size:.75rem;color:var(--muted);margin-top:6px;text-decoration:none">&#128241; Mobile Remote →</a>
    </div>

  </div>

  <!-- ── RIGHT: system health ── -->
  <div class="dash-col">

    <!-- Encoders + bridge -->
    <div class="card">
      <div class="card-title">Encoders</div>
      <div class="row"><span class="lbl tip" data-tip="Left wheel ticks since boot. 3436 ticks = 1 rotation.">Left count</span><span><span class="val" id="el">—</span><span class="unit">ticks</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Right wheel ticks since boot. 3436 ticks = 1 rotation.">Right count</span><span><span class="val" id="er">—</span><span class="unit">ticks</span></span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="serial_motor_bridge connected to Pico over USB serial.">Bridge</span><span class="val" id="bridge">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="E-stop state. OK = motors allowed. ACTIVE = blocked.">E-Stop</span><span class="val" id="estop">—</span></div>
    </div>

    <!-- Pi System -->
    <div class="card">
      <div class="card-title">Pi System</div>
      <div class="row"><span class="lbl tip" data-tip="Pi CPU temp. Green &lt;60°C, yellow &lt;75°C, red ≥75°C.">CPU temp</span><span><span class="val" id="ct">—</span><span class="unit">°C</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="RAM used / total.">Memory</span><span class="val" id="mem">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Time since last boot.">Uptime</span><span class="val" id="up">—</span></div>
    </div>

    <!-- Pico W -->
    <div class="card">
      <div class="card-title">Pico W (MC)</div>
      <div class="row"><span class="lbl tip" data-tip="RP2040 die temperature. Normal 30–50°C.">CPU temp</span><span><span class="val" id="pct">—</span><span class="unit">°C</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="LED ON = e-stop cleared, motors allowed.">LED</span><span class="val" id="pled">—</span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="Pico uptime since USB power-on.">Uptime</span><span class="val" id="pup">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Total serial commands processed since boot.">Commands</span><span class="val" id="pcmds">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Parse/execution errors since boot.">Errors</span><span class="val" id="perrs">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Firmware version.">Version</span><span class="val muted" id="pver">—</span></div>
    </div>

  </div>

</div>

<div class="log-panel">
  <div class="log-header">
    <span class="card-title" style="margin:0">Log Stream</span>
    <span id="log-count" style="font-size:.72rem;color:var(--muted)">—</span>
    <div style="display:flex;gap:6px;margin-left:auto;align-items:center">
      <select id="log-filter" class="log-sel" onchange="logFilter=+this.value">
        <option value="10">All</option>
        <option value="20" selected>INFO+</option>
        <option value="30">WARN+</option>
        <option value="40">ERROR+</option>
      </select>
      <button class="log-btn" id="log-pause-btn" onclick="togglePause()">Pause</button>
      <button class="log-btn" onclick="$('logbox').innerHTML=''">Clear</button>
      <button class="log-btn" onclick="copyAllLogs()">Copy All</button>
      <a href="/logs/download" class="log-btn">Download</a>
    </div>
  </div>
  <div id="logbox" class="logbox"></div>
</div>

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
    $('bv').textContent   = b.voltage    != null ? fmt(b.voltage, 2)    : '—';
    $('bi').textContent   = b.current_mA != null ? fmt(b.current_mA, 0) : '—';
    $('bp').textContent   = b.power_mW   != null ? fmt(b.power_mW, 0)   : '—';
    $('bsoc').textContent = b.soc_pct    != null ? b.soc_pct            : '—';
    const vc = b.voltage == null ? '' : b.voltage < 11 ? 'red' : b.voltage < 12 ? 'yellow' : 'green';
    $('bv').className = 'val big ' + vc;
    stale($('bv'), b.ts, 20);

    // system
    const s = d.system;
    $('ct').textContent  = s.cpu_temp_c != null ? fmt(s.cpu_temp_c, 1) : '—';
    $('ct').className    = 'val ' + (s.cpu_temp_c == null ? '' : s.cpu_temp_c > 75 ? 'red' : s.cpu_temp_c > 60 ? 'yellow' : 'green');
    $('mem').textContent = s.mem_used_mb != null ? `${s.mem_used_mb} / ${s.mem_total_mb} MB` : '—';
    $('up').textContent  = uptime(s.uptime_s);

    // pico
    const p = d.pico;
    $('pct').textContent  = p.cpu_temp_c != null ? fmt(p.cpu_temp_c, 1) : '—';
    $('pct').className    = 'val ' + (p.cpu_temp_c == null ? '' : p.cpu_temp_c > 60 ? 'red' : p.cpu_temp_c > 45 ? 'yellow' : 'green');
    $('pled').textContent = p.led_on == null ? '—' : (p.led_on ? 'ON' : 'OFF');
    $('pled').className   = 'val ' + (p.led_on === true ? 'green' : p.led_on === false ? 'muted' : 'muted');
    $('pup').textContent  = p.uptime_ms != null ? uptime(Math.floor(p.uptime_ms / 1000)) : '—';
    $('pcmds').textContent = p.cmds  != null ? p.cmds  : '—';
    $('perrs').textContent = p.errors != null ? p.errors : '—';
    $('perrs').className   = 'val ' + (p.errors > 0 ? 'yellow' : '');
    $('pver').textContent  = p.version || '—';
    stale($('pct'), p.ts, 20);

    $('ts').textContent  = 'Last update: ' + new Date().toLocaleTimeString();

    // nav state
    const nav = d.nav || {};
    const navOn = nav.enabled === true;
    navEnabled = navOn;
    $('nav-mode').textContent = nav.ts ? (navOn ? 'Autonomous' : 'Manual') : '—';
    $('nav-mode').className   = 'val ' + (nav.ts ? (navOn ? 'green' : 'muted') : 'muted');
    const btn = $('nav-toggle');
    btn.textContent = navOn ? 'Disable' : 'Enable';
    btn.className   = 'nav-btn' + (navOn ? ' active' : '');
    const ACT_LBL = {idle:'— Idle', forward:'▲ Forward', backward:'▼ Backward',
      rotate_left:'↺ Rotate Left', rotate_right:'↻ Rotate Right',
      escape_backward:'⚡ Escape: Back', escape_turn:'⚡ Escape: Turn',
      escape_forward:'⚡ Escape: Fwd', post_escape_forward:'▲ Escape+'};
    const ACT_COL = {idle:'muted', forward:'green', backward:'yellow',
      rotate_left:'blue', rotate_right:'blue',
      escape_backward:'red', escape_turn:'red', escape_forward:'red', post_escape_forward:'yellow'};
    const act = nav.action || 'idle';
    $('nav-action').textContent = ACT_LBL[act] || act;
    $('nav-action').className   = 'val ' + (ACT_COL[act] || '');
    const pathOk = nav.path_clear !== false;
    $('nav-path').textContent = nav.ts ? (pathOk ? 'Clear' : 'Blocked') : '—';
    $('nav-path').className   = 'val ' + (nav.ts ? (pathOk ? 'green' : 'red') : 'muted');
    stale($('nav-mode'), nav.ts);

    // lidar clearances + radar
    const li = d.lidar || {};
    const dc = v => v == null ? '' : v < 0.5 ? 'red' : v < 1.0 ? 'yellow' : 'green';
    $('lidar-front').textContent = li.front_m != null ? fmt(li.front_m, 1) : '—';
    $('lidar-left').textContent  = li.left_m  != null ? fmt(li.left_m,  1) : '—';
    $('lidar-right').textContent = li.right_m != null ? fmt(li.right_m, 1) : '—';
    $('lidar-front').className = 'val ' + dc(li.front_m);
    $('lidar-left').className  = 'val ' + dc(li.left_m);
    $('lidar-right').className = 'val ' + dc(li.right_m);
    stale($('lidar-front'), li.ts);
    drawLidar(li.pts || [], li.ts);

  } catch(e) {
    if (++fails >= 3) {
      $('conn-dot').className = 'dot red';
      $('conn-text').textContent = 'Dashboard unreachable';
    }
  }
}

poll();
setInterval(poll, 500);

// ── robot control ─────────────────────────────────────────────────────────────
let ctrlSpeed    = 0.25;
let ctrlInterval = null;
let ctrlDir      = null;
const MOVES = {fwd:[1,0], back:[-1,0], left:[0,1], right:[0,-1]};

function sendVel(lin, ang) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear:lin, angular:ang})}).catch(()=>{});
}

function ctrlPress(dir) {
  if (ctrlDir) $('dc-'+ctrlDir).classList.remove('pressed');
  ctrlDir = dir;
  $('dc-'+dir).classList.add('pressed');
  const [lf,af] = MOVES[dir];
  const lin = lf * ctrlSpeed;
  const ang = af * Math.max(0.5, ctrlSpeed * 2.5);
  sendVel(lin, ang);
  if (!ctrlInterval) ctrlInterval = setInterval(() => sendVel(lin, ang), 100);
}

function ctrlRelease() {
  if (ctrlInterval) { clearInterval(ctrlInterval); ctrlInterval = null; }
  if (ctrlDir) { $('dc-'+ctrlDir).classList.remove('pressed'); ctrlDir = null; }
  sendVel(0, 0);
}

['fwd','back','left','right'].forEach(dir => {
  const el = $('dc-'+dir);
  el.addEventListener('mousedown', e => { e.preventDefault(); ctrlPress(dir); });
  el.addEventListener('mouseup',   e => { e.preventDefault(); ctrlRelease(); });
  el.addEventListener('mouseleave', ctrlRelease);
  el.addEventListener('touchstart', e => { e.preventDefault(); ctrlPress(dir); }, {passive:false});
  el.addEventListener('touchend',   e => { e.preventDefault(); ctrlRelease(); }, {passive:false});
  el.addEventListener('touchcancel', ctrlRelease);
});
$('dc-stop').addEventListener('mousedown', e => { e.preventDefault(); ctrlRelease(); });
$('dc-stop').addEventListener('touchstart', e => { e.preventDefault(); ctrlRelease(); }, {passive:false});

document.addEventListener('keydown', e => {
  if (e.repeat || e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  const map = {ArrowUp:'fwd', ArrowDown:'back', ArrowLeft:'left', ArrowRight:'right'};
  if (map[e.key]) { e.preventDefault(); ctrlPress(map[e.key]); }
  else if (e.key === ' ') { e.preventDefault(); ctrlRelease(); }
});
document.addEventListener('keyup', e => {
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) ctrlRelease();
});

// ── nav toggle ────────────────────────────────────────────────────────────────
let navEnabled = false;
function toggleNav() {
  fetch('/nav', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({autonomous: !navEnabled})}).catch(()=>{});
}

// ── lidar radar ───────────────────────────────────────────────────────────────
function drawLidar(pts, ts) {
  const canvas = $('lidar-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;
  const maxDist = 3.0;
  const scale = (Math.min(W, H) / 2 - 24) / maxDist;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#0a0c14';
  ctx.fillRect(0, 0, W, H);

  // Grid rings + distance labels
  for (let r = 1; r <= maxDist; r++) {
    ctx.strokeStyle = '#1e2538';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = '#3a4560';
    ctx.font = '9px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(r + 'm', cx + r * scale + 3, cy + 4);
  }

  // Cross hairs
  ctx.strokeStyle = '#1e2538';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx, cy - maxDist * scale - 8); ctx.lineTo(cx, cy + maxDist * scale + 8); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx - maxDist * scale - 8, cy); ctx.lineTo(cx + maxDist * scale + 8, cy); ctx.stroke();

  // Forward scan cone (±45°) — subtle blue tint
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, maxDist * scale, -3 * Math.PI / 4, -Math.PI / 4);
  ctx.closePath();
  ctx.fillStyle = '#3b82f60a';
  ctx.fill();
  ctx.strokeStyle = '#3b82f630';
  ctx.lineWidth = 1;
  ctx.stroke();

  // Obstacle threshold ring (0.5 m dashed red)
  ctx.strokeStyle = '#ef444470';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.arc(cx, cy, 0.5 * scale, 0, 2 * Math.PI); ctx.stroke();
  ctx.setLineDash([]);

  // No-data state
  if (!ts || age(ts) > 5) {
    ctx.fillStyle = '#3a4560';
    ctx.font = '13px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('No LIDAR data', cx, cy + 20);
    ctx.textAlign = 'left';
    // Draw robot even with no data
    ctx.fillStyle = '#3b82f6';
    ctx.beginPath(); ctx.moveTo(cx, cy - 9); ctx.lineTo(cx - 6, cy + 6); ctx.lineTo(cx + 6, cy + 6); ctx.closePath(); ctx.fill();
    return;
  }

  // Scan points — LIDAR mounted 180° rotated, so flip both axes
  for (const [angle, dist] of pts) {
    const d = Math.min(dist, maxDist);
    const px = cx + Math.sin(angle) * d * scale;
    const py = cy + Math.cos(angle) * d * scale;
    ctx.fillStyle = dist < 0.5 ? '#ef4444' : dist < 1.0 ? '#eab308' : '#22c55e';
    ctx.beginPath(); ctx.arc(px, py, 2, 0, 2 * Math.PI); ctx.fill();
  }

  // Robot icon (blue triangle pointing forward/up)
  ctx.fillStyle = '#3b82f6';
  ctx.beginPath(); ctx.moveTo(cx, cy - 9); ctx.lineTo(cx - 6, cy + 6); ctx.lineTo(cx + 6, cy + 6); ctx.closePath(); ctx.fill();

  // FWD label
  ctx.fillStyle = '#3b82f6';
  ctx.font = '9px monospace';
  ctx.textAlign = 'center';
  ctx.fillText('FWD', cx, cy - maxDist * scale - 8);
  ctx.textAlign = 'left';
}

// ── log stream ────────────────────────────────────────────────────────────────
let logSince  = 0;
let logPaused = false;
let logFilter = 20;
const LVL_NAME  = {10:'DEBUG', 20:'INFO ', 30:'WARN ', 40:'ERROR', 50:'FATAL'};
const LVL_CLASS = {10:'log-dbg', 20:'log-inf', 30:'log-wrn', 40:'log-err', 50:'log-err'};

function togglePause() {
  logPaused = !logPaused;
  const btn = $('log-pause-btn');
  btn.textContent = logPaused ? 'Resume' : 'Pause';
  btn.style.color = logPaused ? 'var(--yellow)' : '';
}

function copyAllLogs() {
  const lines = Array.from($('logbox').children).map(el => el.textContent);
  navigator.clipboard.writeText(lines.join('\\n')).catch(() => {
    const ta = document.createElement('textarea');
    ta.value = lines.join('\\n');
    document.body.appendChild(ta); ta.select();
    document.execCommand('copy'); document.body.removeChild(ta);
  });
}

async function pollLogs() {
  try {
    const r = await fetch('/logs?since=' + logSince, {signal: AbortSignal.timeout(2000)});
    if (!r.ok) return;
    const d = await r.json();
    $('log-count').textContent = d.total + ' entries';
    if (!d.entries.length) return;
    logSince = d.total;
    const box = $('logbox');
    const frag = document.createDocumentFragment();
    let added = 0;
    for (const e of d.entries) {
      if (e.level < logFilter) continue;
      const line = document.createElement('div');
      line.className = 'log-line ' + (LVL_CLASS[e.level] || 'log-inf');
      const t = new Date(e.t * 1000).toLocaleTimeString();
      const lv = LVL_NAME[e.level] || String(e.level);
      line.textContent = '[' + t + '] [' + lv + '] [' + e.node + '] ' + e.msg;
      frag.appendChild(line);
      added++;
    }
    if (!added) return;
    box.appendChild(frag);
    while (box.children.length > 2000) box.removeChild(box.firstChild);
    if (!logPaused) box.scrollTop = box.scrollHeight;
  } catch(e) {}
}

pollLogs();
setInterval(pollLogs, 1000);
</script>
</body>
</html>"""


# ── Mobile remote page ────────────────────────────────────────────────────────
REMOTE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>Robot Remote</title>
<style>
  :root { --bg:#0f1117; --card:#1a1d27; --border:#2a2d3a; --text:#e2e8f0; --muted:#8892a4;
          --green:#22c55e; --red:#ef4444; --yellow:#eab308; --blue:#3b82f6; }
  * { box-sizing:border-box; margin:0; padding:0; touch-action:manipulation; -webkit-tap-highlight-color:transparent; }
  html, body { height:100%; background:var(--bg); color:var(--text);
               font-family:'Segoe UI',system-ui,sans-serif; overflow:hidden; }
  body { display:flex; flex-direction:column; padding:14px; gap:12px; }
  header { display:flex; align-items:center; justify-content:space-between; flex-shrink:0; }
  header h1 { font-size:1rem; font-weight:600; }
  a.back { font-size:.8rem; color:var(--muted); text-decoration:none; }
  .speed-row { display:flex; align-items:center; gap:10px; flex-shrink:0; }
  .speed-row label { font-size:.8rem; color:var(--muted); white-space:nowrap; }
  input[type=range] { flex:1; accent-color:var(--blue); height:24px; }
  .speed-val { font-size:.9rem; font-weight:700; min-width:44px; text-align:right; }
  .dpad { display:grid; grid-template-columns:1fr 1fr 1fr; grid-template-rows:1fr 1fr 1fr;
          gap:10px; flex:1; }
  .dp { background:var(--card); border:2px solid var(--border); border-radius:16px;
        display:flex; align-items:center; justify-content:center;
        font-size:2.2rem; cursor:pointer; user-select:none; -webkit-user-select:none; }
  .dp.pressed { background:#1e2a50; border-color:var(--blue); }
  .dp-empty { visibility:hidden; }
  .dp-stop { background:#2a1a1a; border-color:#7f1d1d; color:var(--red); font-size:2rem; }
  .dp-stop.pressed { background:#4a1a1a; }
  .estop { background:#7f1d1d; border:2px solid var(--red); border-radius:16px;
           padding:18px; display:flex; align-items:center; justify-content:center;
           font-size:1.05rem; font-weight:800; letter-spacing:2px; color:#fef2f2;
           cursor:pointer; user-select:none; flex-shrink:0; }
  .estop:active { background:#991b1b; }
  .vel { text-align:center; font-size:.75rem; color:var(--muted);
         font-variant-numeric:tabular-nums; flex-shrink:0; }
</style>
</head>
<body>
<header>
  <h1>&#127918; Robot Remote</h1>
  <a class="back" href="/">&#8592; Dashboard</a>
</header>

<div class="speed-row">
  <label>Speed</label>
  <input type="range" id="spd" min="0.05" max="0.5" step="0.05" value="0.25"
         oninput="spd=+this.value;$('sv').textContent=spd.toFixed(2)+' m/s'">
  <span class="speed-val" id="sv">0.25 m/s</span>
</div>

<div class="dpad">
  <div class="dp dp-empty"></div>
  <div class="dp" id="b-fwd">&#9650;</div>
  <div class="dp dp-empty"></div>
  <div class="dp" id="b-left">&#9664;</div>
  <div class="dp dp-stop" id="b-stop">&#9632;</div>
  <div class="dp" id="b-right">&#9654;</div>
  <div class="dp dp-empty"></div>
  <div class="dp" id="b-back">&#9660;</div>
  <div class="dp dp-empty"></div>
</div>

<div class="estop" id="estop" ontouchstart="stop()" onclick="stop()">&#9940; E-STOP</div>

<div class="vel" id="vel">v: 0.00 m/s &nbsp; &#969;: 0.00 rad/s</div>

<script>
const $ = id => document.getElementById(id);
let spd = 0.25, iv = null, adir = null;
const MOVES = {fwd:[1,0], back:[-1,0], left:[0,1], right:[0,-1]};

function send(lin, ang) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear:lin, angular:ang})}).catch(()=>{});
  $('vel').textContent = 'v: ' + lin.toFixed(2) + ' m/s   \u03c9: ' + ang.toFixed(2) + ' rad/s';
}

function go(dir) {
  if (adir && adir !== dir) { $('b-'+adir).classList.remove('pressed'); }
  adir = dir;
  $('b-'+dir).classList.add('pressed');
  const [lf,af] = MOVES[dir];
  const lin = lf * spd, ang = af * Math.max(0.5, spd * 2.5);
  send(lin, ang);
  if (!iv) iv = setInterval(() => send(lin, ang), 100);
}

function stop() {
  if (iv) { clearInterval(iv); iv = null; }
  if (adir) { $('b-'+adir).classList.remove('pressed'); adir = null; }
  send(0, 0);
}

['fwd','back','left','right'].forEach(d => {
  const el = $('b-'+d);
  el.addEventListener('touchstart', e => { e.preventDefault(); go(d); }, {passive:false});
  el.addEventListener('touchend',   e => { e.preventDefault(); stop(); }, {passive:false});
  el.addEventListener('touchcancel', stop);
  el.addEventListener('mousedown', e => { e.preventDefault(); go(d); });
  el.addEventListener('mouseup',   stop);
  el.addEventListener('mouseleave', stop);
});
$('b-stop').addEventListener('touchstart', e => { e.preventDefault(); stop(); }, {passive:false});
$('b-stop').addEventListener('mousedown',  e => { e.preventDefault(); stop(); });

document.addEventListener('keydown', e => {
  if (e.repeat) return;
  const m = {ArrowUp:'fwd',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right'};
  if (m[e.key]) { e.preventDefault(); go(m[e.key]); }
  else if (e.key===' ') { e.preventDefault(); stop(); }
});
document.addEventListener('keyup', e => {
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) stop();
});
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
        elif self.path == '/remote':
            self._respond(200, 'text/html; charset=utf-8', REMOTE_HTML.encode())
        elif self.path.startswith('/logs/download'):
            _LVL = {10:'DEBUG', 20:'INFO ', 30:'WARN ', 40:'ERROR', 50:'FATAL'}
            with _log_lock:
                entries = list(_log_buffer)
            lines = []
            for e in entries:
                t  = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(e['t']))
                lv = _LVL.get(e['level'], str(e['level']))
                lines.append(f"[{t}] [{lv}] [{e['node']}] {e['msg']}")
            body = '\n'.join(lines).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Content-Disposition', 'attachment; filename="robot_log.txt"')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith('/logs'):
            qs    = parse_qs(urlparse(self.path).query)
            since = int(qs.get('since', [0])[0])
            with _log_lock:
                buf   = list(_log_buffer)
                total = _log_total
            buf_start = total - len(buf)
            entries   = [e for i, e in enumerate(buf) if buf_start + i >= since]
            body = json.dumps({"total": total, "entries": entries}).encode()
            self._respond(200, 'application/json', body)
        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        if self.path == '/cmd':
            try:
                length = int(self.headers.get('Content-Length', 0))
                data   = json.loads(self.rfile.read(length))
                if _node is not None:
                    twist = Twist()
                    twist.linear.x  = float(data.get('linear',  0.0))
                    twist.angular.z = float(data.get('angular', 0.0))
                    _node._cmd_pub.publish(twist)
                self._respond(200, 'application/json', b'{"ok":true}')
            except Exception as exc:
                self._respond(400, 'application/json',
                              json.dumps({'error': str(exc)}).encode())
        elif self.path == '/nav':
            try:
                length = int(self.headers.get('Content-Length', 0))
                data   = json.loads(self.rfile.read(length))
                if _node is not None:
                    req = SetBool.Request()
                    req.data = bool(data.get('autonomous', False))
                    _node._nav_mode_client.call_async(req)
                self._respond(200, 'application/json', b'{"ok":true}')
            except Exception as exc:
                self._respond(400, 'application/json',
                              json.dumps({'error': str(exc)}).encode())
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
        global _node
        super().__init__('robot_dashboard')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value
        _node = self

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.create_subscription(String,    '/motor_controller/status',       self._on_status,     10)
        self.create_subscription(String,    '/motor_controller/encoder_data', self._on_encoders,   10)
        self.create_subscription(Odometry,  '/odom',                          self._on_odom,       10)
        self.create_subscription(Log,       '/rosout',                        self._on_rosout,    100)
        self.create_subscription(String,    '/navigation/debug',              self._on_nav_debug,  10)
        self.create_subscription(LaserScan, '/scan',                          self._on_scan, sensor_qos)

        self._nav_mode_client = self.create_client(SetBool, '/set_autonomous_mode')
        self._last_scan_ts = 0.0  # throttle scan processing to ~2 Hz for dashboard

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
            # battery arrives as 'battery' sub-dict in status/heartbeat
            batt = data.get('battery') or data.get('batt')
            if batt and 'bus_voltage' in batt:
                current_a = batt.get('current_draw_amp')
                power_w   = batt.get('current_draw_watt')
                _metrics['battery']['voltage']    = batt.get('bus_voltage')
                _metrics['battery']['current_mA'] = round(current_a * 1000, 1) if current_a is not None else None
                _metrics['battery']['power_mW']   = round(power_w   * 1000, 1) if power_w   is not None else None
                _metrics['battery']['soc_pct']    = batt.get('batt_charge_percent')
                _metrics['battery']['ts'] = now
            # pico system metrics
            if data.get('cpu_temp_c') is not None:
                _metrics['pico']['cpu_temp_c'] = data.get('cpu_temp_c')
                _metrics['pico']['led_on']     = data.get('led_on')
                _metrics['pico']['ts']         = now
            if data.get('uptime_ms') is not None:
                _metrics['pico']['uptime_ms'] = data.get('uptime_ms')
                _metrics['pico']['cmds']      = data.get('cmds')
                _metrics['pico']['errors']    = data.get('errors')
            if data.get('version') is not None:
                _metrics['pico']['version'] = data.get('version')

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

    def _on_rosout(self, msg: Log):
        global _log_total
        entry = {
            "t":     msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            "level": msg.level,
            "node":  msg.name,
            "msg":   msg.msg,
        }
        with _log_lock:
            _log_buffer.append(entry)
            _log_total += 1

    def _on_nav_debug(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        with _lock:
            _metrics['nav']['enabled']    = data.get('autonomous_enabled', False)
            _metrics['nav']['action']     = data.get('current_action', 'idle')
            _metrics['nav']['obstacle']   = data.get('obstacle_detected', False)
            _metrics['nav']['path_clear'] = data.get('path_clear', True)
            _metrics['nav']['ts']         = now

    def _on_scan(self, msg: LaserScan):
        now = time.time()
        if now - self._last_scan_ts < 0.4:  # max ~2.5 Hz for dashboard
            return
        self._last_scan_ts = now

        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Filter minimum: 0.30 m excludes chassis frame posts (~10–15 cm from LIDAR)
        # while still catching real obstacles. Matches nav node's lidar_min_range intent.
        CHASSIS_MIN = 0.30

        # Downsample full 360° scan to ≤180 points
        step = max(1, n // 180)
        pts = []
        for i in range(0, n, step):
            r = ranges[i]
            if CHASSIS_MIN < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                pts.append([round(angle, 3), round(r, 2)])

        # Sector clearances — LIDAR mounted 180° rotated on robot:
        # Robot forward = LIDAR angle ±π = indices near 0 and n-1 (wraps around)
        # Left/right also swap relative to standard mounting
        half_idx  = int(math.radians(45) / abs(msg.angle_increment))
        front_idx = list(range(n - half_idx, n)) + list(range(0, half_idx))
        front_r   = [ranges[i] for i in front_idx if CHASSIS_MIN < ranges[i] < msg.range_max]
        left_r    = [r for r in ranges[:n // 4]       if CHASSIS_MIN < r < msg.range_max]
        right_r   = [r for r in ranges[n * 3 // 4:]  if CHASSIS_MIN < r < msg.range_max]

        with _lock:
            _metrics['lidar']['pts']     = pts
            _metrics['lidar']['front_m'] = round(min(front_r), 2) if front_r else None
            _metrics['lidar']['left_m']  = round(min(left_r),  2) if left_r  else None
            _metrics['lidar']['right_m'] = round(min(right_r), 2) if right_r else None
            _metrics['lidar']['ts']      = now

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

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
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import glob as _glob
import json
import math
import os as _os
import subprocess
import threading
import time
import urllib.request as _urllib_request
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
    "system":   {"cpu_temp_c": None, "mem_used_mb": None, "mem_total_mb": None, "uptime_s": 0,
                 "throttled_raw": None, "undervoltage_now": None, "throttled_now": None,
                 "undervoltage_ever": None, "throttled_ever": None},
    "pico":     {"cpu_temp_c": None, "led_on": None, "uptime_ms": None, "cmds": None, "errors": None, "version": None, "ts": 0},
    "nav":      {"enabled": False, "action": "idle", "obstacle": False, "path_clear": True, "ts": 0},
    "lidar":    {"pts": [], "front_m": None, "left_m": None, "right_m": None, "rear_m": None, "ts": 0},
    "follow_me":{"enabled": False, "person_detected": False, "distance_m": None, "lateral_m": None, "confidence": 0.0, "ts": 0},
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
    try:
        r = subprocess.run(["vcgencmd", "get_throttled"],
                           capture_output=True, text=True, timeout=1)
        val = int(r.stdout.strip().split("=")[1], 16)
        out["throttled_raw"]     = val
        out["undervoltage_now"]  = bool(val & 0x1)
        out["throttled_now"]     = bool(val & 0x4)
        out["undervoltage_ever"] = bool(val & 0x10000)
        out["throttled_ever"]    = bool(val & 0x40000)
    except Exception:
        out["throttled_raw"]     = None
        out["undervoltage_now"]  = None
        out["throttled_now"]     = None
        out["undervoltage_ever"] = None
        out["throttled_ever"]    = None
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
  /* page nav bar */
  .page-nav { display:flex; gap:8px; flex-wrap:wrap; margin-bottom:16px; }
  .page-nav a { display:inline-block; background:#1e2130; color:var(--text);
                border:1px solid var(--border); border-radius:7px;
                padding:6px 14px; font-size:.8rem; text-decoration:none; transition:background .1s; }
  .page-nav a:hover { background:#2a3050; border-color:var(--blue); }
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
<nav class="page-nav">
  <a href="/remote">&#128241; Mobile Remote</a>
  <a href="/follow">&#128100; Follow-Me</a>
  <a href="/mfollow">&#128241; Follow Mobile</a>
  <a href="/flashpico">&#9889; Flash Pico</a>
</nav>

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
      <div class="row"><span class="lbl tip" data-tip="Min distance behind the robot (used for backup decisions)">Rear</span><span><span class="val" id="lidar-rear">—</span><span class="unit">m</span></span></div>
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
    </div>

  </div>

  <!-- ── RIGHT: system health ── -->
  <div class="dash-col">

    <!-- Encoders + bridge -->
    <div class="card">
      <div class="card-title">Encoders</div>
      <div class="row"><span class="lbl tip" data-tip="Directional cumulative ticks since last reset. Negative = net backward. 3436 ticks = 1 rotation.">Left count</span><span><span class="val" id="el">—</span><span class="unit">ticks</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="Directional cumulative ticks since last reset. Negative = net backward. 3436 ticks = 1 rotation.">Right count</span><span><span class="val" id="er">—</span><span class="unit">ticks</span></span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="serial_motor_bridge connected to Pico over USB serial.">Bridge</span><span class="val" id="bridge">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="E-stop state. OK = motors allowed. ACTIVE = blocked.">E-Stop</span><span class="val" id="estop">—</span></div>
      <div class="row" style="margin-top:6px;gap:6px">
        <button class="log-btn" style="flex:1;background:#7a1111;color:#fff;border-color:#a33" onclick="sendEstop()">&#9632; E-STOP</button>
        <button class="log-btn" style="flex:1;background:#0a4a1a;color:#fff;border-color:#1a8a3a" onclick="resetEstop()">&#10003; Reset</button>
      </div>
    </div>

    <!-- Pi System -->
    <div class="card">
      <div class="card-title">Pi System</div>
      <div class="row"><span class="lbl tip" data-tip="Pi CPU temp. Green &lt;60°C, yellow &lt;75°C, red ≥75°C.">CPU temp</span><span><span class="val" id="ct">—</span><span class="unit">°C</span></span></div>
      <div class="row"><span class="lbl tip" data-tip="RAM used / total.">Memory</span><span class="val" id="mem">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Time since last boot.">Uptime</span><span class="val" id="up">—</span></div>
      <hr>
      <div class="row"><span class="lbl tip" data-tip="Power status from vcgencmd. Green=OK, yellow=past event, red=active throttle/undervoltage.">Power</span><span class="val" id="pi-pwr">—</span></div>
      <div class="row"><span class="lbl tip" data-tip="Undervoltage detected (now or since boot). Check USB power supply if not green.">Undervoltage</span><span class="val" id="pi-uv">—</span></div>
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
    if (s.throttled_raw == null) {
      $('pi-pwr').textContent = '—'; $('pi-pwr').className = 'val muted';
      $('pi-uv').textContent  = '—'; $('pi-uv').className  = 'val muted';
    } else {
      if (s.undervoltage_now || s.throttled_now) {
        $('pi-pwr').textContent = s.throttled_now ? 'THROTTLED' : 'UNDERVOLT';
        $('pi-pwr').className   = 'val red';
      } else if (s.undervoltage_ever || s.throttled_ever) {
        $('pi-pwr').textContent = 'past event';
        $('pi-pwr').className   = 'val yellow';
      } else {
        $('pi-pwr').textContent = 'OK';
        $('pi-pwr').className   = 'val green';
      }
      $('pi-uv').textContent = s.undervoltage_now ? 'NOW' : (s.undervoltage_ever ? 'past' : 'none');
      $('pi-uv').className   = 'val ' + (s.undervoltage_now ? 'red' : s.undervoltage_ever ? 'yellow' : 'green');
    }

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
    $('lidar-rear').textContent = li.rear_m  != null ? fmt(li.rear_m,  1) : '—';
    $('lidar-rear').className  = 'val ' + dc(li.rear_m);
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
function sendEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({reset: false})}).catch(()=>{});
}
function resetEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({reset: true})}).catch(()=>{});
}
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


# ── Follow-Me desktop page ────────────────────────────────────────────────────
FOLLOW_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Follow-Me — Robot</title>
<style>
:root{--bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--text:#e2e8f0;--muted:#8892a4;
      --green:#22c55e;--red:#ef4444;--yellow:#eab308;--blue:#3b82f6;--orange:#f97316;}
*{box-sizing:border-box;margin:0;padding:0;}
html,body{min-height:100vh;background:var(--bg);color:var(--text);
          font-family:'Segoe UI',system-ui,sans-serif;font-size:13px;}
body{display:grid;grid-template-columns:300px 1fr;grid-template-rows:auto 1fr;gap:10px;padding:10px;}
header{grid-column:1/-1;display:flex;align-items:center;justify-content:space-between;
       background:var(--card);border:1px solid var(--border);border-radius:10px;padding:8px 14px;}
header h1{font-size:1rem;font-weight:700;letter-spacing:.03em;}
header nav a{font-size:.78rem;color:var(--muted);text-decoration:none;margin-left:12px;}
header nav a:hover{color:var(--text);}
.left{display:flex;flex-direction:column;gap:10px;}
.right{display:flex;flex-direction:column;gap:10px;}
.card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:12px 14px;}
.card-title{font-size:.7rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
            color:var(--muted);margin-bottom:10px;}
.row{display:flex;justify-content:space-between;align-items:center;padding:3px 0;
     border-bottom:1px solid #1e2233;}
.row:last-child{border-bottom:none;}
.label{color:var(--muted);font-size:.8rem;}
.val{font-size:.85rem;font-weight:600;}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px;flex-shrink:0;}
.green{color:var(--green);} .red{color:var(--red);} .yellow{color:var(--yellow);} .blue{color:var(--blue);}
.badge{display:inline-block;padding:2px 8px;border-radius:20px;font-size:.7rem;font-weight:700;
       letter-spacing:.05em;text-transform:uppercase;}
.badge-green{background:#14532d;color:var(--green);border:1px solid #166534;}
.badge-red{background:#450a0a;color:var(--red);border:1px solid #7f1d1d;}
.badge-blue{background:#1e3a5f;color:var(--blue);border:1px solid #1d4ed8;}
.badge-muted{background:#1e2233;color:var(--muted);border:1px solid var(--border);}
.badge-yellow{background:#422006;color:var(--yellow);border:1px solid #92400e;}
/* Follow toggle button */
.follow-btn{width:100%;padding:14px;border-radius:8px;border:none;font-size:1rem;font-weight:700;
            cursor:pointer;margin-top:4px;transition:all .15s;}
.follow-btn.enable{background:#14532d;color:var(--green);border:2px solid #166534;}
.follow-btn.enable:hover{background:#166534;}
.follow-btn.disable{background:#450a0a;color:var(--red);border:2px solid #7f1d1d;}
.follow-btn.disable:hover{background:#7f1d1d;}
/* Person tracker canvas */
canvas.tracker{border-radius:8px;background:#0a0c14;display:block;width:100%;}
canvas.lidar-c{border-radius:50%;background:#0a0c14;display:block;margin:0 auto;}
/* Stale */
.stale{opacity:.4;}
</style>
</head>
<body>
<header>
  <h1>&#128100; Follow-Me Mode</h1>
  <nav>
    <a href="/">Dashboard</a>
    <a href="/remote">Remote</a>
    <a href="/mfollow">&#128241; Mobile</a>
  </nav>
</header>

<div class="left">
  <!-- Follow-Me Control -->
  <div class="card">
    <div class="card-title">Follow-Me Control</div>
    <div class="row">
      <span class="label">Mode</span>
      <span id="fm-mode-badge" class="badge badge-muted">IDLE</span>
    </div>
    <div class="row">
      <span class="label">Person</span>
      <span id="fm-person"></span>
    </div>
    <div class="row">
      <span class="label">Confidence</span>
      <span id="fm-conf" class="val">—</span>
    </div>
    <button id="fm-btn" class="follow-btn enable" onclick="toggleFollow()">Enable Follow-Me</button>
  </div>

  <!-- Person Tracker -->
  <div class="card">
    <div class="card-title">Person Tracker</div>
    <canvas id="tracker-canvas" class="tracker" width="268" height="160"></canvas>
    <div style="display:flex;justify-content:space-between;margin-top:8px;">
      <div>
        <div class="label">Distance</div>
        <div id="fm-dist" class="val" style="font-size:1.3rem;">—</div>
      </div>
      <div style="text-align:center;">
        <div class="label">Lateral</div>
        <div id="fm-lat" class="val" style="font-size:1.3rem;">—</div>
      </div>
      <div style="text-align:right;">
        <div class="label">Target</div>
        <div class="val" style="font-size:1.3rem;color:var(--muted);">1.0 m</div>
      </div>
    </div>
  </div>

  <!-- OAK-D -->
  <div class="card">
    <div class="card-title">OAK-D Lite</div>
    <div class="row"><span class="label">Detection</span><span id="fm-det"></span></div>
    <div class="row"><span class="label">Distance (z)</span><span id="fm-z" class="val">—</span></div>
    <div class="row"><span class="label">Lateral (x)</span><span id="fm-x" class="val">—</span></div>
    <div class="row"><span class="label">Confidence</span><span id="fm-score" class="val">—</span></div>
    <div class="row"><span class="label">Data age</span><span id="fm-age" class="val">—</span></div>
  </div>

  <!-- Battery -->
  <div class="card">
    <div class="card-title">Battery</div>
    <div class="row"><span class="label">Voltage</span><span id="b-v" class="val">—</span></div>
    <div class="row"><span class="label">Current</span><span id="b-i" class="val">—</span></div>
    <div class="row"><span class="label">Charge</span><span id="b-soc" class="val">—</span></div>
  </div>
</div>

<div class="right">
  <!-- LIDAR -->
  <div class="card">
    <div class="card-title">LIDAR — Top-Down
      <span style="margin-left:8px;font-weight:400;">
        <span style="color:var(--red);">&#9679;</span> &lt;0.5m &nbsp;
        <span style="color:var(--yellow);">&#9679;</span> &lt;1m &nbsp;
        <span style="color:var(--green);">&#9679;</span> &gt;1m
      </span>
    </div>
    <canvas id="lidar-canvas" class="lidar-c" width="420" height="420"></canvas>
    <div style="display:flex;justify-content:space-around;margin-top:8px;">
      <div style="text-align:center;"><div class="label">Front</div><div id="l-front" class="val">—</div></div>
      <div style="text-align:center;"><div class="label">Left</div><div id="l-left" class="val">—</div></div>
      <div style="text-align:center;"><div class="label">Right</div><div id="l-right" class="val">—</div></div>
      <div style="text-align:center;"><div class="label">Rear</div><div id="l-rear" class="val">—</div></div>
    </div>
  </div>

  <!-- Bridge + E-Stop -->
  <div class="card">
    <div class="card-title">Bridge &amp; Safety</div>
    <div class="row"><span class="label">Bridge</span><span id="s-bridge"></span></div>
    <div class="row"><span class="label">E-Stop</span><span id="s-estop"></span></div>
    <div style="display:flex;gap:8px;margin-top:8px;">
      <button onclick="sendEstop()" style="flex:1;padding:8px;background:#450a0a;color:var(--red);
        border:1px solid #7f1d1d;border-radius:6px;cursor:pointer;font-weight:700;">&#9632; E-STOP</button>
      <button onclick="resetEstop()" style="flex:1;padding:8px;background:#14532d;color:var(--green);
        border:1px solid #166534;border-radius:6px;cursor:pointer;font-weight:700;">&#10003; Reset</button>
    </div>
  </div>

  <!-- Pico W -->
  <div class="card">
    <div class="card-title">Pico W (MC)</div>
    <div class="row"><span class="label">CPU temp</span><span id="p-temp" class="val">—</span></div>
    <div class="row"><span class="label">Uptime</span><span id="p-up" class="val">—</span></div>
    <div class="row"><span class="label">Commands</span><span id="p-cmds" class="val">—</span></div>
    <div class="row"><span class="label">Errors</span><span id="p-err" class="val">—</span></div>
  </div>
</div>

<script>
const $ = id => document.getElementById(id);
const age = ts => ts ? (Date.now()/1000 - ts) : 999;
const fmt1 = v => v == null ? '—' : v.toFixed(1);
const fmt2 = v => v == null ? '—' : v.toFixed(2);
const distColor = d => {
  if (d == null) return 'var(--muted)';
  if (d < 0.4) return 'var(--red)';
  if (d < 0.8) return 'var(--yellow)';
  if (d <= 1.2) return 'var(--green)';
  if (d <= 2.0) return 'var(--yellow)';
  return 'var(--muted)';
};

let followEnabled = false;

function toggleFollow() {
  const want = !followEnabled;
  fetch('/follow_me', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({enabled: want})}).catch(()=>{});
}
function sendEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({reset: false})}).catch(()=>{});
}
function resetEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({reset: true})}).catch(()=>{});
}

// ── lidar radar ───────────────────────────────────────────────────────────────
function drawLidar(pts, ts) {
  const canvas = $('lidar-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W/2, cy = H/2;
  const maxDist = 3.0;
  const scale = (Math.min(W,H)/2 - 24) / maxDist;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#0a0c14'; ctx.fillRect(0,0,W,H);
  for (let r=1;r<=maxDist;r++){
    ctx.strokeStyle='#1e2538';ctx.lineWidth=1;
    ctx.beginPath();ctx.arc(cx,cy,r*scale,0,2*Math.PI);ctx.stroke();
    ctx.fillStyle='#3a4560';ctx.font='9px monospace';ctx.textAlign='left';
    ctx.fillText(r+'m',cx+r*scale+3,cy+4);
  }
  ctx.strokeStyle='#1e2538';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(cx,cy-maxDist*scale-8);ctx.lineTo(cx,cy+maxDist*scale+8);ctx.stroke();
  ctx.beginPath();ctx.moveTo(cx-maxDist*scale-8,cy);ctx.lineTo(cx+maxDist*scale+8,cy);ctx.stroke();
  ctx.beginPath();ctx.moveTo(cx,cy);ctx.arc(cx,cy,maxDist*scale,-3*Math.PI/4,-Math.PI/4);ctx.closePath();
  ctx.fillStyle='#3b82f60a';ctx.fill();ctx.strokeStyle='#3b82f630';ctx.lineWidth=1;ctx.stroke();
  ctx.strokeStyle='#ef444470';ctx.lineWidth=1;ctx.setLineDash([4,4]);
  ctx.beginPath();ctx.arc(cx,cy,0.5*scale,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);
  if (!ts || age(ts)>5){
    ctx.fillStyle='#3a4560';ctx.font='13px monospace';ctx.textAlign='center';
    ctx.fillText('No LIDAR data',cx,cy+20);ctx.textAlign='left';
    ctx.fillStyle='#3b82f6';ctx.beginPath();ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+6);ctx.lineTo(cx+6,cy+6);ctx.closePath();ctx.fill();
    return;
  }
  for (const [angle,dist] of pts){
    const d=Math.min(dist,maxDist);
    const px=cx+Math.sin(angle)*d*scale, py=cy+Math.cos(angle)*d*scale;
    ctx.fillStyle=dist<0.5?'#ef4444':dist<1.0?'#eab308':'#22c55e';
    ctx.beginPath();ctx.arc(px,py,2,0,2*Math.PI);ctx.fill();
  }
  ctx.fillStyle='#3b82f6';
  ctx.beginPath();ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+6);ctx.lineTo(cx+6,cy+6);ctx.closePath();ctx.fill();
  ctx.fillStyle='#3b82f6';ctx.font='9px monospace';ctx.textAlign='center';
  ctx.fillText('FWD',cx,cy-maxDist*scale-8);ctx.textAlign='left';
}

// ── person tracker canvas ─────────────────────────────────────────────────────
function drawTracker(dist, lateral, personDetected) {
  const canvas = $('tracker-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#0a0c14'; ctx.fillRect(0,0,W,H);

  const midY = H/2;
  const lateralRange = 1.5;  // ±1.5 m shown
  const distRange = 3.0;     // 0–3 m shown
  const margin = 20;

  // ── Lateral bar (top half) ─────────────────────────────────
  const barY = H * 0.28;
  const barH = 22;
  const barX = margin;
  const barW = W - margin*2;

  // Background bar
  ctx.fillStyle='#1e2233'; ctx.roundRect(barX, barY-barH/2, barW, barH, 4); ctx.fill();

  // Centre line
  ctx.strokeStyle='#3a4560'; ctx.lineWidth=1; ctx.setLineDash([3,3]);
  ctx.beginPath(); ctx.moveTo(W/2, barY-barH/2-4); ctx.lineTo(W/2, barY+barH/2+4); ctx.stroke();
  ctx.setLineDash([]);

  // Labels
  ctx.fillStyle='#3a4560'; ctx.font='9px monospace'; ctx.textAlign='left';
  ctx.fillText('-1.5m', barX, barY-barH/2-5);
  ctx.textAlign='center'; ctx.fillText('lateral', W/2, barY-barH/2-5);
  ctx.textAlign='right'; ctx.fillText('+1.5m', barX+barW, barY-barH/2-5);

  if (lateral != null) {
    const normLat = Math.max(-1, Math.min(1, lateral / lateralRange));
    const dotX = W/2 + normLat * (barW/2);
    const dotColor = Math.abs(lateral) < 0.1 ? '#22c55e' : Math.abs(lateral) < 0.5 ? '#eab308' : '#ef4444';
    // Highlight fill
    const fillX = Math.min(W/2, dotX), fillW = Math.abs(dotX - W/2);
    ctx.fillStyle = dotColor + '33';
    ctx.fillRect(fillX, barY-barH/2, fillW, barH);
    // Dot
    ctx.fillStyle = dotColor;
    ctx.beginPath(); ctx.arc(dotX, barY, 8, 0, 2*Math.PI); ctx.fill();
    // Direction arrow text
    ctx.fillStyle = dotColor; ctx.font='bold 10px monospace'; ctx.textAlign='center';
    ctx.fillText(lateral > 0.05 ? '▶ RIGHT' : lateral < -0.05 ? 'LEFT ◀' : '▲ CENTRED', W/2, barY+barH/2+14);
  } else {
    ctx.fillStyle='#3a4560'; ctx.font='11px monospace'; ctx.textAlign='center';
    ctx.fillText('no data', W/2, barY+4);
  }

  // ── Distance gauge (bottom half) ──────────────────────────
  const gaugeY = H * 0.72;
  const gaugeX = margin;
  const gaugeW = W - margin*2;
  const gaugeH = 18;

  // Colour zones
  const zones=[
    {from:0,   to:0.4,  col:'#7f1d1d'},
    {from:0.4, to:0.8,  col:'#78350f'},
    {from:0.8, to:1.2,  col:'#14532d'},
    {from:1.2, to:2.0,  col:'#78350f'},
    {from:2.0, to:3.0,  col:'#1e2233'},
  ];
  for (const z of zones){
    const x0 = gaugeX + (z.from/distRange)*gaugeW;
    const x1 = gaugeX + (z.to/distRange)*gaugeW;
    ctx.fillStyle = z.col;
    ctx.fillRect(x0, gaugeY-gaugeH/2, x1-x0, gaugeH);
  }
  ctx.strokeStyle='#2a2d3a'; ctx.lineWidth=1;
  ctx.strokeRect(gaugeX, gaugeY-gaugeH/2, gaugeW, gaugeH);

  // Target line at 1.0 m
  const targetX = gaugeX + (1.0/distRange)*gaugeW;
  ctx.strokeStyle='#22c55e80'; ctx.lineWidth=2; ctx.setLineDash([4,3]);
  ctx.beginPath(); ctx.moveTo(targetX, gaugeY-gaugeH/2-4); ctx.lineTo(targetX, gaugeY+gaugeH/2+4); ctx.stroke();
  ctx.setLineDash([]);

  // Zone labels
  ctx.fillStyle='#3a4560'; ctx.font='9px monospace'; ctx.textAlign='center';
  ['0','0.4','0.8','1.2','2.0','3.0'].forEach((v,i)=>{
    const vals=[0,0.4,0.8,1.2,2.0,3.0];
    ctx.fillText(v, gaugeX+(vals[i]/distRange)*gaugeW, gaugeY+gaugeH/2+11);
  });
  ctx.fillStyle='#22c55e80'; ctx.fillText('target', targetX, gaugeY-gaugeH/2-7);

  // Current distance marker
  if (dist != null) {
    const dx = gaugeX + Math.min(1, dist/distRange)*gaugeW;
    const dc = distColor(dist);
    ctx.fillStyle = dc;
    ctx.beginPath(); ctx.moveTo(dx, gaugeY-gaugeH/2-2); ctx.lineTo(dx-5, gaugeY-gaugeH/2-10); ctx.lineTo(dx+5, gaugeY-gaugeH/2-10); ctx.closePath(); ctx.fill();
    ctx.fillStyle = dc; ctx.font='bold 11px monospace'; ctx.textAlign='center';
    ctx.fillText(dist.toFixed(2)+'m', dx, gaugeY-gaugeH/2-12);
  } else {
    ctx.fillStyle='#3a4560'; ctx.font='11px monospace'; ctx.textAlign='center';
    ctx.fillText('no data', W/2, gaugeY+4);
  }

  ctx.textAlign='left';
}

// ── main poll ─────────────────────────────────────────────────────────────────
function fmts(s){
  if(s==null) return '—';
  const m=Math.floor(s/60), sec=Math.floor(s%60);
  return m>0 ? m+'m '+sec+'s' : sec+'s';
}

async function poll() {
  try {
    const r = await fetch('/metrics');
    const d = await r.json();
    const fm = d.follow_me || {};
    const li = d.lidar || {};
    const ba = d.battery || {};
    const st = d.status || {};
    const pi = d.pico || {};

    // Follow-Me state
    followEnabled = !!fm.enabled;
    const btn = $('fm-btn');
    if (followEnabled) {
      btn.textContent = 'Disable Follow-Me';
      btn.className = 'follow-btn disable';
      $('fm-mode-badge').className = 'badge badge-green';
      $('fm-mode-badge').textContent = 'FOLLOWING';
    } else {
      btn.textContent = 'Enable Follow-Me';
      btn.className = 'follow-btn enable';
      $('fm-mode-badge').className = 'badge badge-muted';
      $('fm-mode-badge').textContent = 'IDLE';
    }

    const pd = !!fm.person_detected;
    const fmStale = age(fm.ts) > 3;
    $('fm-person').innerHTML = pd
      ? '<span class="dot" style="background:var(--green)"></span><span class="green">Detected</span>'
      : '<span class="dot" style="background:var(--muted)"></span><span style="color:var(--muted)">None</span>';
    $('fm-conf').textContent = fm.confidence != null ? (fm.confidence*100).toFixed(0)+'%' : '—';
    $('fm-conf').style.color = fm.confidence > 0.7 ? 'var(--green)' : fm.confidence > 0.4 ? 'var(--yellow)' : 'var(--muted)';

    // OAK-D card
    $('fm-det').innerHTML = pd
      ? '<span class="badge badge-green">Person</span>'
      : '<span class="badge badge-muted">None</span>';
    const distV = fm.distance_m;
    const latV  = fm.lateral_m;
    $('fm-z').textContent = distV != null ? distV.toFixed(2)+' m' : '—';
    $('fm-z').style.color = distColor(distV);
    $('fm-x').textContent = latV  != null ? (latV >= 0 ? '+' : '')+latV.toFixed(2)+' m' : '—';
    $('fm-x').style.color = latV != null ? (Math.abs(latV)<0.1?'var(--green)':Math.abs(latV)<0.5?'var(--yellow)':'var(--red)') : 'var(--muted)';
    $('fm-score').textContent = fm.confidence != null ? (fm.confidence*100).toFixed(0)+'%' : '—';
    $('fm-age').textContent = fm.ts ? age(fm.ts).toFixed(1)+'s' : '—';
    $('fm-age').style.color = fmStale ? 'var(--red)' : 'var(--green)';

    // Distance/lateral large display
    $('fm-dist').textContent = distV != null ? distV.toFixed(2)+' m' : '—';
    $('fm-dist').style.color = distColor(distV);
    $('fm-lat').textContent  = latV  != null ? (latV >= 0 ? '+' : '')+latV.toFixed(2)+' m' : '—';

    // Person tracker canvas
    drawTracker(distV, latV, pd);

    // LIDAR
    drawLidar(li.pts || [], li.ts);
    const distFmt = v => v==null ? '<span style="color:var(--muted)">—</span>'
                                 : '<span style="color:'+(v<0.7?'var(--red)':v<1.0?'var(--yellow)':'var(--green)')+'">'+v.toFixed(1)+' m</span>';
    $('l-front').innerHTML = distFmt(li.front_m);
    $('l-left').innerHTML  = distFmt(li.left_m);
    $('l-right').innerHTML = distFmt(li.right_m);
    $('l-rear').innerHTML  = distFmt(li.rear_m);

    // Battery
    $('b-v').textContent   = ba.voltage   != null ? ba.voltage.toFixed(2)+' V' : '—';
    $('b-v').style.color   = ba.voltage   != null ? (ba.voltage>12.5?'var(--green)':ba.voltage>11.8?'var(--yellow)':'var(--red)') : 'var(--muted)';
    $('b-i').textContent   = ba.current_mA!= null ? ba.current_mA+' mA' : '—';
    $('b-soc').textContent = ba.soc_pct   != null ? ba.soc_pct+'%' : '—';

    // Bridge/estop
    const conn = !!d.bridge_connected;
    $('s-bridge').innerHTML = conn
      ? '<span class="dot" style="background:var(--green)"></span><span class="green">Connected</span>'
      : '<span class="dot" style="background:var(--red)"></span><span class="red">Disconnected</span>';
    const estop = st.emergency_stop;
    $('s-estop').innerHTML = estop===false
      ? '<span class="green">OK</span>'
      : estop===true
        ? '<span class="red">ACTIVE</span>'
        : '<span style="color:var(--muted)">—</span>';

    // Pico
    $('p-temp').textContent = pi.cpu_temp_c != null ? pi.cpu_temp_c.toFixed(1)+'°C' : '—';
    if (pi.uptime_ms != null){
      const s=Math.floor(pi.uptime_ms/1000), m=Math.floor(s/60), sec=s%60;
      $('p-up').textContent = m+'m '+sec+'s';
    }
    $('p-cmds').textContent = pi.cmds != null ? pi.cmds : '—';
    $('p-err').textContent  = pi.errors != null ? pi.errors : '—';
    $('p-err').style.color  = pi.errors > 0 ? 'var(--red)' : 'var(--green)';

  } catch(e) {}
}

poll();
setInterval(poll, 500);
</script>
</body>
</html>"""


# ── Follow-Me mobile page ─────────────────────────────────────────────────────
MFOLLOW_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>Follow-Me Mobile</title>
<style>
:root{--bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--text:#e2e8f0;--muted:#8892a4;
      --green:#22c55e;--red:#ef4444;--yellow:#eab308;--blue:#3b82f6;}
*{box-sizing:border-box;margin:0;padding:0;touch-action:manipulation;-webkit-tap-highlight-color:transparent;}
html,body{min-height:100vh;background:var(--bg);color:var(--text);
          font-family:'Segoe UI',system-ui,sans-serif;font-size:14px;}
body{display:flex;flex-direction:column;padding:8px;gap:8px;max-width:480px;margin:0 auto;}
.card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:10px 14px;}
.card-title{font-size:.68rem;font-weight:700;letter-spacing:.08em;text-transform:uppercase;
            color:var(--muted);margin-bottom:8px;}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;
     border-bottom:1px solid #1e2233;}
.row:last-child{border-bottom:none;}
.label{color:var(--muted);font-size:.82rem;}
.val{font-size:.9rem;font-weight:600;}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:5px;}
.badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.72rem;font-weight:700;letter-spacing:.05em;text-transform:uppercase;}
.badge-green{background:#14532d;color:var(--green);border:1px solid #166534;}
.badge-red{background:#450a0a;color:var(--red);border:1px solid #7f1d1d;}
.badge-muted{background:#1e2233;color:var(--muted);border:1px solid var(--border);}
/* Status bar */
.status-bar{display:flex;align-items:center;justify-content:space-between;flex-shrink:0;}
.status-bar h1{font-size:.95rem;font-weight:700;}
/* Toggle button */
.follow-btn{width:100%;padding:16px;border-radius:10px;border:none;font-size:1.1rem;font-weight:700;
            cursor:pointer;transition:all .15s;letter-spacing:.03em;}
.follow-btn.enable{background:#14532d;color:var(--green);border:2px solid #166534;}
.follow-btn.disable{background:#450a0a;color:var(--red);border:2px solid #7f1d1d;}
/* Big metrics */
.metrics-strip{display:flex;gap:0;}
.metric-cell{flex:1;text-align:center;padding:10px 4px;border-right:1px solid var(--border);}
.metric-cell:last-child{border-right:none;}
.metric-big{font-size:1.5rem;font-weight:700;line-height:1.1;}
.metric-label{font-size:.68rem;color:var(--muted);text-transform:uppercase;margin-top:3px;}
/* Lateral bar */
.lat-bar-wrap{position:relative;height:40px;margin:4px 0 2px;}
.lat-bar-bg{position:absolute;inset:8px 0;background:#1e2233;border-radius:4px;}
.lat-bar-fill{position:absolute;top:8px;bottom:8px;background:#3b82f630;transition:all .2s;}
.lat-dot{position:absolute;top:50%;width:22px;height:22px;border-radius:50%;
         transform:translate(-50%,-50%);transition:left .2s;}
.lat-labels{display:flex;justify-content:space-between;font-size:.65rem;color:var(--muted);margin-top:2px;}
/* LIDAR */
.lidar-wrap{display:flex;justify-content:center;}
canvas{border-radius:50%;background:#0a0c14;display:block;}
/* E-Stop */
.estop-btn{width:100%;padding:18px;background:#450a0a;color:var(--red);border:2px solid #7f1d1d;
           border-radius:10px;font-size:1.1rem;font-weight:700;cursor:pointer;letter-spacing:.05em;}
.reset-btn{width:100%;padding:12px;background:#14532d;color:var(--green);border:2px solid #166534;
           border-radius:10px;font-size:.9rem;font-weight:700;cursor:pointer;}
</style>
</head>
<body>

<!-- Status bar -->
<div class="card status-bar">
  <h1>&#128100; Follow-Me</h1>
  <div style="display:flex;align-items:center;gap:8px;">
    <span id="m-mode-badge" class="badge badge-muted">IDLE</span>
    <span id="m-conn-dot" class="dot" style="background:var(--muted)"></span>
  </div>
</div>

<!-- Toggle -->
<button id="m-fm-btn" class="follow-btn enable" onclick="toggleFollow()">&#9654; Enable Follow-Me</button>

<!-- Person stats -->
<div class="card">
  <div class="card-title">Person</div>
  <div class="metrics-strip">
    <div class="metric-cell">
      <div id="m-dist" class="metric-big" style="color:var(--muted)">—</div>
      <div class="metric-label">Distance</div>
    </div>
    <div class="metric-cell">
      <div id="m-lat-num" class="metric-big" style="color:var(--muted)">—</div>
      <div class="metric-label">Lateral</div>
    </div>
    <div class="metric-cell">
      <div id="m-conf" class="metric-big" style="color:var(--muted)">—</div>
      <div class="metric-label">Confidence</div>
    </div>
  </div>
  <!-- Lateral bar -->
  <div style="margin-top:6px;">
    <div class="lat-bar-wrap">
      <div class="lat-bar-bg"></div>
      <div id="m-lat-fill" class="lat-bar-fill" style="left:50%;width:0;"></div>
      <div id="m-lat-dot" class="lat-dot" style="left:50%;background:var(--muted);display:none;"></div>
    </div>
    <div class="lat-labels"><span>◀ 1.5m</span><span>Centre</span><span>1.5m ▶</span></div>
  </div>
</div>

<!-- LIDAR -->
<div class="card">
  <div class="card-title">LIDAR &nbsp;
    <span style="font-weight:400;font-size:.65rem;">
      <span style="color:var(--red);">&#9679;</span>&lt;0.5m
      <span style="color:var(--yellow);">&#9679;</span>&lt;1m
      <span style="color:var(--green);">&#9679;</span>&gt;1m
    </span>
  </div>
  <div class="lidar-wrap"><canvas id="m-lidar" width="300" height="300"></canvas></div>
  <div style="display:flex;justify-content:space-around;margin-top:8px;">
    <div style="text-align:center;"><div style="color:var(--muted);font-size:.68rem;">FRONT</div><div id="m-lf" class="val">—</div></div>
    <div style="text-align:center;"><div style="color:var(--muted);font-size:.68rem;">LEFT</div><div id="m-ll" class="val">—</div></div>
    <div style="text-align:center;"><div style="color:var(--muted);font-size:.68rem;">RIGHT</div><div id="m-lr" class="val">—</div></div>
    <div style="text-align:center;"><div style="color:var(--muted);font-size:.68rem;">REAR</div><div id="m-lrear" class="val">—</div></div>
  </div>
</div>

<!-- OAK-D + Battery strip -->
<div class="card">
  <div class="card-title">OAK-D &amp; Battery</div>
  <div class="row"><span class="label">Detection</span><span id="m-det"></span></div>
  <div class="row"><span class="label">Voltage</span><span id="m-bv" class="val">—</span></div>
  <div class="row"><span class="label">Charge</span><span id="m-bsoc" class="val">—</span></div>
  <div class="row"><span class="label">Bridge</span><span id="m-bridge"></span></div>
  <div class="row"><span class="label">E-Stop</span><span id="m-estop"></span></div>
</div>

<!-- E-Stop -->
<button class="estop-btn" onclick="sendEstop()">&#9632; EMERGENCY STOP</button>
<button class="reset-btn" onclick="resetEstop()">&#10003; Reset E-Stop</button>

<div style="text-align:center;padding:4px 0;">
  <a href="/follow" style="color:var(--muted);font-size:.75rem;text-decoration:none;">Desktop view &#8599;</a>
  &nbsp;&nbsp;
  <a href="/" style="color:var(--muted);font-size:.75rem;text-decoration:none;">Dashboard &#8599;</a>
</div>

<script>
const $ = id => document.getElementById(id);
const age = ts => ts ? (Date.now()/1000 - ts) : 999;
const distColor = d => {
  if (d==null) return 'var(--muted)';
  if (d<0.4) return 'var(--red)';
  if (d<0.8) return 'var(--yellow)';
  if (d<=1.2) return 'var(--green)';
  if (d<=2.0) return 'var(--yellow)';
  return 'var(--muted)';
};

let followEnabled = false;

function toggleFollow() {
  fetch('/follow_me', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({enabled: !followEnabled})}).catch(()=>{});
}
function sendEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({reset: false})}).catch(()=>{});
}
function resetEstop() {
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({reset: true})}).catch(()=>{});
}

function drawLidar(pts, ts) {
  const canvas = $('m-lidar');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const W=canvas.width, H=canvas.height, cx=W/2, cy=H/2;
  const maxDist=3.0, scale=(Math.min(W,H)/2-18)/maxDist;
  ctx.clearRect(0,0,W,H); ctx.fillStyle='#0a0c14'; ctx.fillRect(0,0,W,H);
  for(let r=1;r<=maxDist;r++){
    ctx.strokeStyle='#1e2538';ctx.lineWidth=1;
    ctx.beginPath();ctx.arc(cx,cy,r*scale,0,2*Math.PI);ctx.stroke();
    ctx.fillStyle='#3a4560';ctx.font='8px monospace';ctx.textAlign='left';
    ctx.fillText(r+'m',cx+r*scale+2,cy+4);
  }
  ctx.strokeStyle='#1e2538';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(cx,cy-maxDist*scale-6);ctx.lineTo(cx,cy+maxDist*scale+6);ctx.stroke();
  ctx.beginPath();ctx.moveTo(cx-maxDist*scale-6,cy);ctx.lineTo(cx+maxDist*scale+6,cy);ctx.stroke();
  ctx.strokeStyle='#ef444470';ctx.lineWidth=1;ctx.setLineDash([4,4]);
  ctx.beginPath();ctx.arc(cx,cy,0.5*scale,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);
  if(!ts||age(ts)>5){
    ctx.fillStyle='#3a4560';ctx.font='12px monospace';ctx.textAlign='center';
    ctx.fillText('No LIDAR',cx,cy+6);
  } else {
    for(const [angle,dist] of pts){
      const d=Math.min(dist,maxDist);
      const px=cx+Math.sin(angle)*d*scale, py=cy+Math.cos(angle)*d*scale;
      ctx.fillStyle=dist<0.5?'#ef4444':dist<1.0?'#eab308':'#22c55e';
      ctx.beginPath();ctx.arc(px,py,2.5,0,2*Math.PI);ctx.fill();
    }
  }
  ctx.fillStyle='#3b82f6';
  ctx.beginPath();ctx.moveTo(cx,cy-9);ctx.lineTo(cx-6,cy+6);ctx.lineTo(cx+6,cy+6);ctx.closePath();ctx.fill();
  ctx.fillStyle='#3b82f6';ctx.font='8px monospace';ctx.textAlign='center';
  ctx.fillText('FWD',cx,cy-maxDist*scale-5);ctx.textAlign='left';
}

async function poll() {
  try {
    const r = await fetch('/metrics');
    const d = await r.json();
    const fm = d.follow_me || {};
    const li = d.lidar || {};
    const ba = d.battery || {};
    const st = d.status || {};

    followEnabled = !!fm.enabled;
    const btn = $('m-fm-btn');
    if (followEnabled) {
      btn.textContent = '⏹ Disable Follow-Me';
      btn.className = 'follow-btn disable';
      $('m-mode-badge').className = 'badge badge-green';
      $('m-mode-badge').textContent = 'FOLLOWING';
    } else {
      btn.textContent = '▶ Enable Follow-Me';
      btn.className = 'follow-btn enable';
      $('m-mode-badge').className = 'badge badge-muted';
      $('m-mode-badge').textContent = 'IDLE';
    }

    $('m-conn-dot').style.background = d.bridge_connected ? 'var(--green)' : 'var(--red)';

    const pd = !!fm.person_detected;
    const distV = fm.distance_m, latV = fm.lateral_m;

    $('m-dist').textContent = distV != null ? distV.toFixed(2)+' m' : '—';
    $('m-dist').style.color = distColor(distV);
    $('m-lat-num').textContent = latV != null ? (latV>=0?'+':'')+latV.toFixed(2)+' m' : '—';
    $('m-lat-num').style.color = latV!=null?(Math.abs(latV)<0.1?'var(--green)':Math.abs(latV)<0.5?'var(--yellow)':'var(--red)'):'var(--muted)';
    $('m-conf').textContent = fm.confidence!=null ? (fm.confidence*100).toFixed(0)+'%' : '—';
    $('m-conf').style.color = fm.confidence>0.7?'var(--green)':fm.confidence>0.4?'var(--yellow)':'var(--muted)';

    // Lateral bar
    const dot = $('m-lat-dot'), fill = $('m-lat-fill');
    if (latV != null) {
      const norm = Math.max(-1, Math.min(1, latV/1.5));
      const pct = 50 + norm*50;
      const dc = Math.abs(latV)<0.1?'var(--green)':Math.abs(latV)<0.5?'var(--yellow)':'var(--red)';
      dot.style.display='block'; dot.style.left=pct+'%'; dot.style.background=dc;
      fill.style.left = Math.min(50,pct)+'%';
      fill.style.width = Math.abs(pct-50)+'%';
      fill.style.background = dc+'40';
    } else {
      dot.style.display='none'; fill.style.width='0';
    }

    $('m-det').innerHTML = pd
      ? '<span class="badge badge-green">Person</span>'
      : '<span class="badge badge-muted">None</span>';

    $('m-bv').textContent  = ba.voltage  != null ? ba.voltage.toFixed(2)+' V' : '—';
    $('m-bv').style.color  = ba.voltage  != null ? (ba.voltage>12.5?'var(--green)':ba.voltage>11.8?'var(--yellow)':'var(--red)') : 'var(--muted)';
    $('m-bsoc').textContent = ba.soc_pct != null ? ba.soc_pct+'%' : '—';

    $('m-bridge').innerHTML = d.bridge_connected
      ? '<span style="color:var(--green)">Connected</span>'
      : '<span style="color:var(--red)">Disconnected</span>';
    const estop = st.emergency_stop;
    $('m-estop').innerHTML = estop===false ? '<span style="color:var(--green)">OK</span>'
                           : estop===true  ? '<span style="color:var(--red)">ACTIVE</span>'
                           : '<span style="color:var(--muted)">—</span>';

    drawLidar(li.pts||[], li.ts);

    const df = v => v==null?'—':v.toFixed(1)+' m';
    const dc = v => v==null?'var(--muted)':v<0.7?'var(--red)':v<1.0?'var(--yellow)':'var(--green)';
    $('m-lf').textContent = df(li.front_m); $('m-lf').style.color = dc(li.front_m);
    $('m-ll').textContent = df(li.left_m);  $('m-ll').style.color = dc(li.left_m);
    $('m-lr').textContent = df(li.right_m); $('m-lr').style.color = dc(li.right_m);
    $('m-lrear').textContent = df(li.rear_m); $('m-lrear').style.color = dc(li.rear_m);

  } catch(e) {}
}

poll();
setInterval(poll, 400);
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
  :root{--bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--text:#e2e8f0;--muted:#8892a4;
        --green:#22c55e;--red:#ef4444;--yellow:#eab308;--blue:#3b82f6;}
  *{box-sizing:border-box;margin:0;padding:0;touch-action:manipulation;-webkit-tap-highlight-color:transparent;}
  html,body{height:100%;background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;overflow:hidden;}
  body{display:flex;flex-direction:column;padding:10px;gap:7px;}
  header{display:flex;align-items:center;justify-content:space-between;flex-shrink:0;}
  header h1{font-size:.9rem;font-weight:600;}
  a.back{font-size:.78rem;color:var(--muted);text-decoration:none;}
  .top-row{display:flex;align-items:center;gap:8px;flex-shrink:0;}
  .top-row label{font-size:.75rem;color:var(--muted);white-space:nowrap;}
  input[type=range]{flex:1;accent-color:var(--blue);height:22px;}
  .spd-val{font-size:.82rem;font-weight:700;min-width:46px;text-align:right;}
  .axswap{font-size:.72rem;background:var(--card);border:1px solid var(--border);
          border-radius:6px;padding:3px 8px;color:var(--muted);cursor:pointer;white-space:nowrap;flex-shrink:0;}
  .axswap.on{border-color:var(--yellow);color:var(--yellow);}
  /* LIDAR */
  .lidar-wrap{display:flex;justify-content:center;flex-shrink:0;}
  canvas{border-radius:50%;background:#0a0c14;display:block;}
  /* D-pad */
  .dpad{display:grid;grid-template-columns:1fr 1fr 1fr;grid-template-rows:1fr 1fr 1fr;
        gap:7px;flex:1;min-height:0;}
  .dp{background:var(--card);border:2px solid var(--border);border-radius:12px;
      display:flex;align-items:center;justify-content:center;
      font-size:1.8rem;cursor:pointer;user-select:none;-webkit-user-select:none;}
  .dp.pressed{background:#1e2a50;border-color:var(--blue);}
  .dp-diag{font-size:1.3rem;color:#3a4560;}
  .dp-diag.pressed{color:var(--text);background:#1e2a50;border-color:var(--blue);}
  .dp-stop{background:#2a1a1a;border-color:#7f1d1d;color:var(--red);}
  .dp-stop.pressed{background:#4a1a1a;}
  /* E-Stop */
  .estop-btn{border-radius:12px;padding:11px 16px;display:flex;flex-direction:column;
             align-items:center;justify-content:center;gap:2px;
             cursor:pointer;user-select:none;flex-shrink:0;border:2px solid;}
  .estop-btn:active{filter:brightness(1.3);}
  .es-ok{background:#0d2b1a;border-color:var(--green);color:var(--green);}
  .es-act{background:#2a1a1a;border-color:var(--red);color:var(--red);}
  .es-lbl{font-size:.92rem;font-weight:800;letter-spacing:1px;}
  .es-sub{font-size:.68rem;opacity:.75;}
  .vel{text-align:center;font-size:.68rem;color:var(--muted);font-variant-numeric:tabular-nums;flex-shrink:0;}
</style>
</head>
<body>
<header>
  <h1>&#127918; Robot Remote</h1>
  <a class="back" href="/">&#8592; Dashboard</a>
</header>

<div class="top-row">
  <label>Speed</label>
  <input type="range" id="spd" min="0.05" max="0.5" step="0.05" value="0.25"
    oninput="spd=+this.value;document.getElementById('sv').textContent=spd.toFixed(2)+' m/s'">
  <span class="spd-val" id="sv">0.25 m/s</span>
  <button class="axswap" id="axbtn" onclick="toggleAxes()">Axes: Normal</button>
</div>

<div class="lidar-wrap">
  <canvas id="lc" width="200" height="200"></canvas>
</div>

<div class="dpad">
  <div class="dp dp-diag" id="b-fl">&#8598;</div>
  <div class="dp" id="b-fwd">&#9650;</div>
  <div class="dp dp-diag" id="b-fr">&#8599;</div>
  <div class="dp" id="b-left">&#9664;</div>
  <div class="dp dp-stop" id="b-stop">&#9632;</div>
  <div class="dp" id="b-right">&#9654;</div>
  <div class="dp dp-diag" id="b-bl">&#8601;</div>
  <div class="dp" id="b-back">&#9660;</div>
  <div class="dp dp-diag" id="b-br">&#8600;</div>
</div>

<div class="estop-btn es-ok" id="estop-btn" onclick="toggleEstop()">
  <span class="es-lbl" id="es-lbl">&#10003; Movement Allowed</span>
  <span class="es-sub" id="es-sub">Tap to activate E-Stop</span>
</div>

<div class="vel" id="vel">v: 0.00 m/s &nbsp; &#969;: 0.00 rad/s</div>

<script>
const $ = id => document.getElementById(id);
let spd = 0.25, iv = null, adir = null, arcTog = false, axSwap = false;

// [lin_factor, ang_factor] — diagonal corners alternate fwd+turn for arc approx
const MOVES_NORMAL = {
  fwd:[1,0], back:[-1,0], left:[0,1], right:[0,-1],
  fl:[1,1],  fr:[1,-1],   bl:[-1,-1], br:[-1,1]
};
// Swapped axes: ▲ turns, ◄► moves fwd/back (for chassis mounted 90deg off)
const MOVES_SWAP = {
  fwd:[0,1], back:[0,-1], left:[-1,0], right:[1,0],
  fl:[1,1],  fr:[1,-1],   bl:[-1,-1], br:[-1,1]
};
let MOVES = MOVES_NORMAL;

function toggleAxes() {
  axSwap = !axSwap;
  MOVES = axSwap ? MOVES_SWAP : MOVES_NORMAL;
  const btn = $('axbtn');
  btn.textContent = axSwap ? 'Axes: Swapped' : 'Axes: Normal';
  btn.className = axSwap ? 'axswap on' : 'axswap';
  stop();
}

function send(lin, ang) {
  fetch('/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({linear:lin, angular:ang})}).catch(()=>{});
  $('vel').textContent = 'v: '+lin.toFixed(2)+' m/s   \u03c9: '+ang.toFixed(2)+' rad/s';
}

function go(dir) {
  if (adir && adir !== dir) $('b-'+adir).classList.remove('pressed');
  adir = dir;
  $('b-'+dir).classList.add('pressed');
  const [lf, af] = MOVES[dir];
  const isDiag = lf !== 0 && af !== 0;
  const lin = lf * spd;
  const ang = af * Math.max(0.5, spd * 2.5);
  if (!isDiag) {
    send(lin, ang);
    if (!iv) iv = setInterval(() => send(lin, ang), 100);
  } else {
    arcTog = false; send(lin, 0);
    if (!iv) iv = setInterval(() => { arcTog = !arcTog; send(arcTog ? 0 : lin, arcTog ? ang : 0); }, 150);
  }
}

function stop() {
  if (iv) { clearInterval(iv); iv = null; }
  if (adir) { $('b-'+adir).classList.remove('pressed'); adir = null; }
  send(0, 0);
}

['fwd','back','left','right','fl','fr','bl','br'].forEach(d => {
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
  else if (e.key === ' ') { e.preventDefault(); stop(); }
});
document.addEventListener('keyup', e => {
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) stop();
});

// ── E-Stop ────────────────────────────────────────────────────────────────────
let estopped = false;
function toggleEstop() {
  stop();
  fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({reset: estopped})}).catch(()=>{});
}
function setEstopUI(active) {
  estopped = active;
  const btn = $('estop-btn'), lbl = $('es-lbl'), sub = $('es-sub');
  if (active) {
    btn.className = 'estop-btn es-act';
    lbl.textContent = '\u26a0 Movement Disabled';
    sub.textContent = 'Tap to Reset E-Stop';
  } else {
    btn.className = 'estop-btn es-ok';
    lbl.textContent = '\u2713 Movement Allowed';
    sub.textContent = 'Tap to Activate E-Stop';
  }
}

// ── LIDAR radar ───────────────────────────────────────────────────────────────
const cvs = $('lc'), ctx = cvs.getContext('2d');
const CW = cvs.width, CH = cvs.height, cx = CW/2, cy = CH/2, maxR = cx - 6;
const MAX_D = 3.0;
function drawLidar(pts, ts) {
  ctx.clearRect(0, 0, CW, CH);
  ctx.strokeStyle = '#1e2233'; ctx.lineWidth = 1;
  [1,2,3].forEach(i => { ctx.beginPath(); ctx.arc(cx, cy, maxR*i/3, 0, 6.283); ctx.stroke(); });
  ctx.beginPath(); ctx.moveTo(cx,4); ctx.lineTo(cx,CH-4); ctx.moveTo(4,cy); ctx.lineTo(CW-4,cy); ctx.stroke();
  const age = ts > 0 ? (Date.now()/1000 - ts) : 99;
  ctx.globalAlpha = age > 2 ? 0.3 : 1.0;
  if (pts && pts.length) {
    pts.forEach(([a, d]) => {
      const dist = Math.min(d, MAX_D), s = (maxR / MAX_D) * dist;
      const px = cx + Math.sin(a) * s, py = cy + Math.cos(a) * s;
      const t = dist / MAX_D;
      ctx.fillStyle = 'rgb('+Math.round(239*(1-t)+34*t)+','+Math.round(68*(1-t)+197*t)+','+Math.round(68*(1-t)+94*t)+')';
      ctx.fillRect(px-1.5, py-1.5, 3, 3);
    });
  }
  ctx.globalAlpha = 1.0;
  ctx.fillStyle = '#94a3b8';
  ctx.beginPath(); ctx.moveTo(cx, cy-9); ctx.lineTo(cx-6, cy+6); ctx.lineTo(cx+6, cy+6); ctx.closePath(); ctx.fill();
}

// ── Poll metrics ──────────────────────────────────────────────────────────────
function poll() {
  fetch('/metrics').then(r => r.json()).then(d => {
    const li = d.lidar || {};
    drawLidar(li.pts || [], li.ts || 0);
    const es = (d.status || {}).emergency_stop;
    if (es != null) setEstopUI(es);
  }).catch(() => {});
}
drawLidar([], 0); poll(); setInterval(poll, 400);
</script>
</body>
</html>"""


FLASHPICO_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flash Pico W</title>
<style>
  :root{--bg:#0f1117;--card:#1a1d27;--border:#2a2d3a;--text:#e2e8f0;--muted:#8892a4;
        --green:#22c55e;--red:#ef4444;--yellow:#eab308;--blue:#3b82f6;}
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;padding:20px;max-width:600px;margin:0 auto;}
  header{display:flex;align-items:baseline;justify-content:space-between;margin-bottom:20px;}
  h1{font-size:1.15rem;}
  a.back{font-size:.8rem;color:var(--muted);text-decoration:none;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:16px;margin-bottom:12px;}
  .card-title{font-size:.78rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-bottom:10px;display:flex;align-items:center;gap:8px;}
  .step{width:20px;height:20px;border-radius:50%;background:var(--border);color:var(--muted);font-size:.7rem;font-weight:700;display:inline-flex;align-items:center;justify-content:center;flex-shrink:0;}
  .step.active{background:var(--blue);color:#fff;}
  .step.done{background:var(--green);color:#000;}
  .badge{display:inline-block;padding:3px 10px;border-radius:12px;font-size:.82rem;font-weight:600;}
  .b-none{background:#2a2d3a;color:var(--muted);}
  .b-boot{background:#1e3a5f;color:#60a5fa;}
  .b-mp{background:#14532d;color:var(--green);}
  .b-run{background:#1a3a1a;color:#86efac;}
  .det{font-size:.8rem;color:var(--muted);margin-top:6px;}
  .btn{display:block;width:100%;padding:11px;border:none;border-radius:8px;font-size:.9rem;font-weight:600;cursor:pointer;margin-top:10px;transition:opacity .15s;}
  .btn:disabled{opacity:.3;cursor:not-allowed;}
  .btn-blue{background:var(--blue);color:#fff;}
  .btn-green{background:var(--green);color:#000;}
  .btn-yellow{background:var(--yellow);color:#000;}
  .hint{font-size:.8rem;color:var(--muted);margin-bottom:4px;}
  pre{background:#0a0c14;border-radius:6px;padding:10px;font-size:.76rem;color:#94a3b8;min-height:50px;max-height:160px;overflow-y:auto;margin-top:8px;white-space:pre-wrap;word-break:break-all;}
</style>
</head>
<body>
<header><h1>Flash Pico W</h1><a class="back" href="/">&#8592; Dashboard</a></header>

<div class="card">
  <div class="card-title"><span class="step" id="s1">1</span>Detect</div>
  <div>Status: <span class="badge b-none" id="badge">Scanning&hellip;</span></div>
  <div class="det" id="det"></div>
</div>

<div class="card">
  <div class="card-title"><span class="step" id="s2">2</span>Flash MicroPython</div>
  <p class="hint">Hold <b>BOOTSEL</b> while plugging in USB, then release. Page detects automatically.</p>
  <button class="btn btn-blue" id="btn-flash" disabled onclick="doFlash()">Flash MicroPython</button>
  <pre id="out-flash">Waiting for bootloader (RPI-RP2)&hellip;</pre>
</div>

<div class="card">
  <div class="card-title"><span class="step" id="s3">3</span>Upload Firmware</div>
  <p class="hint">Uploads <code>main.py</code> and <code>ina219.py</code>, then resets the Pico.</p>
  <button class="btn btn-green" id="btn-upload" disabled onclick="doUpload()">Upload Firmware</button>
  <button class="btn" style="background:#2a2d3a;color:var(--yellow);margin-top:6px;font-size:.8rem;padding:8px;" onclick="doRelease()">Release Serial Port (kill bridge)</button>
  <pre id="out-upload">Waiting for MicroPython&hellip;</pre>
</div>

<div class="card">
  <div class="card-title"><span class="step" id="s4">4</span>Verify</div>
  <p class="hint">Checks that the motor bridge has reconnected successfully.</p>
  <button class="btn btn-yellow" id="btn-verify" disabled onclick="doVerify()">Verify Connection</button>
  <button class="btn" style="background:#2a2d3a;color:var(--green);margin-top:6px;font-size:.8rem;padding:8px;" onclick="doRestart()">Restart Bridge</button>
  <pre id="out-verify">Upload firmware first&hellip;</pre>
</div>

<script>
function ss(n,c){document.getElementById('s'+n).className='step'+(c?' '+c:'');}
function be(id,en){document.getElementById(id).disabled=!en;}
function ot(id,t){document.getElementById(id).textContent=t;}

function updateUI(s){
  const b=document.getElementById('badge'), d=document.getElementById('det');
  b.className='badge';
  if(s.state==='bootloader'){
    b.className+=' b-boot'; b.textContent='Bootloader ('+(s.chip||'?')+')';
    d.textContent='Ready to flash — click button below';
    ss(1,'done');ss(2,'active');ss(3,'');ss(4,'');
    be('btn-flash',true);be('btn-upload',false);be('btn-verify',false);
  } else if(s.state==='micropython'){
    b.className+=' b-mp'; b.textContent='MicroPython (raw REPL)';
    d.textContent='Port: '+(s.port||'?')+' — ready to upload';
    ss(1,'done');ss(2,'done');ss(3,'active');ss(4,'');
    be('btn-flash',false);be('btn-upload',true);be('btn-verify',false);
    ot('out-flash','MicroPython detected.');
  } else if(s.state==='running'){
    b.className+=' b-run'; b.textContent='Firmware Running';
    d.textContent='Bridge connected — Pico is ready.';
    ss(1,'done');ss(2,'done');ss(3,'done');ss(4,'active');
    be('btn-flash',false);be('btn-upload',true);be('btn-verify',true);
    ot('out-flash','MicroPython detected.');
    ot('out-upload','Firmware running — click Upload to re-flash.');
  } else {
    b.className+=' b-none'; b.textContent='Not Detected';
    d.textContent='Connect Pico via USB (hold BOOTSEL for bootloader mode)';
    ss(1,'');ss(2,'');ss(3,'');ss(4,'');
    be('btn-flash',false);be('btn-upload',false);be('btn-verify',false);
  }
}

async function doFlash(){
  be('btn-flash',false);
  ot('out-flash','Downloading firmware (may take ~30s first time)...');
  try{
    const r=await fetch('/pico/flash',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const d=await r.json();
    ot('out-flash',(d.output||'')+(d.error?'\\nError: '+d.error:''));
    if(d.ok) ss(2,'done'); else be('btn-flash',true);
  }catch(e){ot('out-flash','Error: '+e);be('btn-flash',true);}
}

async function doUpload(){
  be('btn-upload',false);
  ot('out-upload','Uploading files...');
  try{
    const r=await fetch('/pico/upload',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const d=await r.json();
    ot('out-upload',(d.output||'')+(d.error?'\\nError: '+d.error:''));
    if(d.ok){ss(3,'done');ss(4,'active');be('btn-verify',true);}
    else be('btn-upload',true);
  }catch(e){ot('out-upload','Error: '+e);be('btn-upload',true);}
}

async function doRelease(){
  const out=document.getElementById('out-upload');
  out.textContent='Releasing serial port...';
  try{
    const r=await fetch('/pico/release_port',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const d=await r.json();
    out.textContent=d.output||d.error||JSON.stringify(d);
  }catch(e){out.textContent='Error: '+e;}
}

async function doVerify(){
  be('btn-verify',false);
  ot('out-verify','Checking bridge connection...');
  try{
    const r=await fetch('/pico/verify',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const d=await r.json();
    ot('out-verify',(d.output||'')+(d.error?'\\nError: '+d.error:''));
    if(d.ok) ss(4,'done'); else be('btn-verify',true);
  }catch(e){ot('out-verify','Error: '+e);be('btn-verify',true);}
}

async function doRestart(){
  ot('out-verify','Restarting bridge...');
  try{
    const r=await fetch('/bridge/restart',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const d=await r.json();
    ot('out-verify',(d.output||'')+(d.error?'\nError: '+d.error:''));
    if(d.ok) setTimeout(()=>{be('btn-verify',true);ot('out-verify',d.output+'\nClick Verify to confirm.');},1000);
  }catch(e){ot('out-verify','Error: '+e);}
}

function poll(){fetch('/pico/status').then(r=>r.json()).then(updateUI).catch(()=>{});}
poll(); setInterval(poll,2500);
</script>
</body>
</html>"""


# ── Pico W flash helpers ───────────────────────────────────────────────────────
_UF2_CACHE = '/tmp/pico_micropython.uf2'
# Label → firmware URL mapping (Pico W = RP2040, Pico 2 W = RP2350)
_UF2_BY_LABEL = {
    'RPI-RP2': 'https://micropython.org/resources/firmware/RPI_PICO_W-20251209-v1.27.0.uf2',
    'RP2350':  'https://micropython.org/resources/firmware/RPI_PICO2_W-20251209-v1.27.0.uf2',
}
_BOOTLOADER_LABELS = list(_UF2_BY_LABEL.keys())
_UPYTHON_CANDIDATES = [
    _os.path.expanduser('~/lab_ws/src/labrobot/upython'),
    _os.path.expanduser('~/dev_ws/src/labrobot/upython'),
    _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), 'upython'),
]


def _upython_dir():
    for d in _UPYTHON_CANDIDATES:
        if _os.path.isdir(d):
            return d
    return None


def _pico_bootloader_label():
    """Return the detected bootloader label (RPI-RP2 or RP2350) or None."""
    for lbl in _BOOTLOADER_LABELS:
        if _os.path.exists(f'/dev/disk/by-label/{lbl}'):
            return lbl
    return None



def _pico_device_path():
    """Return block device path for Pico bootloader (e.g. /dev/sda1), or None."""
    lbl = _pico_bootloader_label()
    if not lbl:
        return None
    path = f'/dev/disk/by-label/{lbl}'
    if _os.path.exists(path):
        return _os.path.realpath(path)
    return None


def _pico_find_port():
    """Return first /dev/ttyACM* port if bridge is not connected (likely raw MicroPython)."""
    with _lock:
        bridge_ok = _metrics.get('bridge_connected', False)
    if bridge_ok:
        return None
    ports = sorted(_glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None


def _pico_status_dict():
    """Detect current Pico W state."""
    lbl = _pico_bootloader_label()
    if lbl:
        return {'state': 'bootloader', 'port': None, 'mount': None, 'chip': lbl}
    with _lock:
        bridge_ok = _metrics.get('bridge_connected', False)
    if bridge_ok:
        ports = sorted(_glob.glob('/dev/ttyACM*'))
        return {'state': 'running', 'port': ports[0] if ports else None, 'mount': None}
    ports = sorted(_glob.glob('/dev/ttyACM*'))
    if ports:
        return {'state': 'micropython', 'port': ports[0], 'mount': None}
    return {'state': 'none', 'port': None, 'mount': None}


def _pico_do_flash():
    lbl = _pico_bootloader_label()
    if not lbl:
        return {'ok': False, 'output': '', 'error': 'No Pico bootloader detected'}
    uf2_url   = _UF2_BY_LABEL[lbl]
    uf2_cache = f'/tmp/pico_micropython_{lbl}.uf2'
    if not _os.path.exists(uf2_cache):
        try:
            _urllib_request.urlretrieve(uf2_url, uf2_cache + '.tmp')
            _os.rename(uf2_cache + '.tmp', uf2_cache)
        except Exception as e:
            return {'ok': False, 'output': '', 'error': f'Download failed: {e}'}
    device = _pico_device_path()
    if not device:
        return {'ok': False, 'output': '', 'error': 'Pico bootloader device not found'}
    try:
        import shutil
        with open(uf2_cache, 'rb') as src, open(device, 'wb') as dst:
            shutil.copyfileobj(src, dst)
            dst.flush()
        return {'ok': True, 'output': f'Firmware written to {device}\nPico is rebooting — wait a few seconds.', 'error': ''}
    except PermissionError:
        return {'ok': False, 'output': '', 'error': f'Permission denied on {device}. Run this once on the Pi:\n  sudo tee /etc/udev/rules.d/99-pico-flash.rules <<EOF\nSUBSYSTEM=="block", ENV{{ID_FS_LABEL}}=="RP2350", MODE="0660", OWNER="botmanager"\nSUBSYSTEM=="block", ENV{{ID_FS_LABEL}}=="RPI-RP2", MODE="0660", OWNER="botmanager"\nEOF\n  sudo udevadm control --reload-rules && sudo udevadm trigger\nThen replug Pico in BOOTSEL mode.'}
    except Exception as e:
        return {'ok': False, 'output': '', 'error': str(e)}


def _mpremote_cp(port, src, dest, retries=8):
    """Copy file via mpremote, retrying if port is busy (bridge retry window)."""
    last_out = ''
    for attempt in range(retries):
        r = subprocess.run(
            ['mpremote', 'connect', port, 'cp', src, dest],
            capture_output=True, text=True, timeout=20)
        combined = (r.stdout + r.stderr).strip()
        if 'in use' in combined or 'failed to access' in combined:
            last_out = combined
            time.sleep(1.5)
            continue
        if r.returncode == 0 and 'failed' not in combined.lower() and 'error' not in combined.lower():
            return True, combined
        return False, combined or 'unknown error'
    return False, f'Port busy after {retries} attempts: {last_out}'


def _pico_do_upload():
    port = _pico_find_port()
    if not port:
        return {'ok': False, 'output': '', 'error': 'No free ttyACM port found'}
    ud = _upython_dir()
    if not ud:
        return {'ok': False, 'output': '', 'error': f'upython directory not found. Checked: {_UPYTHON_CANDIDATES}'}
    lines = []
    for fname in ('main.py', 'ina219.py'):
        src = _os.path.join(ud, fname)
        if not _os.path.exists(src):
            lines.append(f'SKIP {fname} (not found at {src})')
            continue
        ok, msg = _mpremote_cp(port, src, f':{fname}')
        if ok:
            lines.append(f'OK   {fname}')
        else:
            lines.append(f'FAIL {fname}: {msg}')
            return {'ok': False, 'output': '\n'.join(lines), 'error': msg}
    # Soft reset so new main.py runs
    subprocess.run(['mpremote', 'connect', port, 'reset'],
                   capture_output=True, text=True, timeout=5)
    lines.append('Pico reset — waiting for bridge to reconnect...')
    return {'ok': True, 'output': '\n'.join(lines), 'error': ''}


def _pico_do_verify():
    with _lock:
        connected = _metrics.get('bridge_connected', False)
        pico = dict(_metrics.get('pico', {}))
    if connected:
        ver = pico.get('version', '?')
        temp = pico.get('cpu_temp_c', '?')
        return {'ok': True, 'output': f'Bridge connected.\nFirmware version: {ver}\nPico CPU temp: {temp} C', 'error': ''}
    return {'ok': False, 'output': '', 'error': 'Bridge not connected yet. Wait a few seconds and try again.'}


def _bridge_restart():
    import shlex
    scripts_path = os.path.join(os.path.expanduser('~'), 'lab_ws', 'install', 'labrobot', 'share', 'labrobot', 'scripts')
    bridge_script = os.path.join(scripts_path, 'serial_motor_bridge.py')
    cmd = (
        f'source /opt/ros/jazzy/setup.bash && '
        f'source ~/lab_ws/install/setup.bash && '
        f'python3 {bridge_script} --ros-args '
        f'-p serial_port:=/dev/ttyACM0 -p baudrate:=115200 -p timeout:=1.0 '
        f'-p reconnect_interval:=5.0 -p wheel_base_m:=0.347 -p wheel_radius_m:=0.0325 '
        f'-p ticks_per_rev:=3436 -p encoder_poll_hz:=20.0 '
        f'-p odom_frame:=odom -p base_frame:=base_footprint '
        f'>> /tmp/bridge.log 2>&1'
    )
    subprocess.Popen(['bash', '-c', f'nohup bash -c {shlex.quote(cmd)} &>/dev/null &'],
                     start_new_session=True)
    return {'ok': True, 'output': 'Bridge process started — wait ~3s then click Verify.', 'error': ''}


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
        elif self.path == '/follow':
            self._respond(200, 'text/html; charset=utf-8', FOLLOW_HTML.encode())
        elif self.path == '/mfollow':
            self._respond(200, 'text/html; charset=utf-8', MFOLLOW_HTML.encode())
        elif self.path == '/flashpico':
            self._respond(200, 'text/html; charset=utf-8', FLASHPICO_HTML.encode())
        elif self.path == '/pico/status':
            self._respond(200, 'application/json', json.dumps(_pico_status_dict()).encode())
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
        elif self.path == '/estop':
            try:
                length = int(self.headers.get('Content-Length', 0))
                data   = json.loads(self.rfile.read(length))
                if _node is not None:
                    req = Trigger.Request()
                    if data.get('reset', False):
                        _node._reset_estop_client.call_async(req)
                    else:
                        _node._estop_client.call_async(req)
                self._respond(200, 'application/json', b'{"ok":true}')
            except Exception as exc:
                self._respond(400, 'application/json',
                              json.dumps({'error': str(exc)}).encode())
        elif self.path == '/follow_me':
            try:
                length = int(self.headers.get('Content-Length', 0))
                data   = json.loads(self.rfile.read(length))
                if _node is not None:
                    req = SetBool.Request()
                    req.data = bool(data.get('enabled', False))
                    _node._follow_me_client.call_async(req)
                self._respond(200, 'application/json', b'{"ok":true}')
            except Exception as exc:
                self._respond(400, 'application/json',
                              json.dumps({'error': str(exc)}).encode())
        elif self.path == '/pico/release_port':
            try:
                r = subprocess.run(['pkill', '-9', '-f', 'serial_motor_bridge'],
                                   capture_output=True, text=True, timeout=5)
                self._respond(200, 'application/json',
                              json.dumps({'ok': True, 'output': 'Serial bridge killed — port is free. Click Upload now.'}).encode())
            except Exception as e:
                self._respond(200, 'application/json',
                              json.dumps({'ok': False, 'error': str(e)}).encode())
        elif self.path == '/pico/flash':
            result = _pico_do_flash()
            self._respond(200, 'application/json', json.dumps(result).encode())
        elif self.path == '/pico/upload':
            result = _pico_do_upload()
            self._respond(200, 'application/json', json.dumps(result).encode())
        elif self.path == '/pico/verify':
            result = _pico_do_verify()
            self._respond(200, 'application/json', json.dumps(result).encode())
        elif self.path == '/bridge/restart':
            result = _bridge_restart()
            self._respond(200, 'application/json', json.dumps(result).encode())
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
        self.create_subscription(String,    '/navigation/debug',              self._on_nav_debug,       10)
        self.create_subscription(LaserScan, '/scan',                          self._on_scan,      sensor_qos)
        self.create_subscription(String,    '/follow_me/debug',               self._on_follow_me_debug, 10)

        self._nav_mode_client    = self.create_client(SetBool, '/set_autonomous_mode')
        self._follow_me_client   = self.create_client(SetBool, '/set_follow_me_mode')
        self._estop_client       = self.create_client(Trigger, '/motor/emergency_stop')
        self._reset_estop_client = self.create_client(Trigger, '/motor/reset_emergency_stop')
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

    def _on_follow_me_debug(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        now = time.time()
        with _lock:
            _metrics['follow_me']['enabled']         = data.get('enabled', False)
            _metrics['follow_me']['person_detected']  = data.get('person_detected', False)
            _metrics['follow_me']['distance_m']       = data.get('distance_m')
            _metrics['follow_me']['lateral_m']        = data.get('lateral_m')
            _metrics['follow_me']['confidence']       = data.get('confidence', 0.0)
            _metrics['follow_me']['ts']               = now

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
        # Rear: robot rear = LIDAR angle 0 = index n//2 (center of array)
        rear_idx  = list(range(max(0, n // 2 - half_idx), min(n, n // 2 + half_idx)))
        rear_r    = [ranges[i] for i in rear_idx if CHASSIS_MIN < ranges[i] < msg.range_max]

        with _lock:
            _metrics['lidar']['pts']     = pts
            _metrics['lidar']['front_m'] = round(min(front_r), 2) if front_r else None
            _metrics['lidar']['left_m']  = round(min(left_r),  2) if left_r  else None
            _metrics['lidar']['right_m'] = round(min(right_r), 2) if right_r else None
            _metrics['lidar']['rear_m']  = round(min(rear_r),  2) if rear_r  else None
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

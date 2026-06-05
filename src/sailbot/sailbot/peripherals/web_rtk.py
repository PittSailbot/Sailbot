#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
BAUD_RATE = 460800          # GNSS serial baud rate
HTTP_PORT = 5000            # Flask HTTP listen port
 
import os
import time
import base64
import socket
import threading
from collections import deque
from binascii import hexlify
import glob
import math
import json
 
from flask import Flask, render_template_string, request, redirect, url_for, jsonify
import serial
 
app = Flask(__name__)
 
CONFIG_FILE = "rtk_config.json"
 
# ---------- Global state ----------
serial_port = None
serial_thread = None
ntrip_thread = None
stop_event = threading.Event()
serial_lock = threading.Lock()
 
last_gga = ""           # Latest GGA sentence
last_gga_time = 0.0     # Timestamp of latest GGA (time.time)
 
status = {
    "serial": "Disconnected",
    "ntrip":  "Disconnected",
    "message": "",
}
 
# All parameters are empty by default; you fill them in the web UI
config = {
    "serial_port": "",
    "caster_host": "",
    "caster_port": "",
    "mountpoint": "",
    "username": "",
    "password": "",
}
 
def load_config_from_file():
    """Load config from JSON file if present."""
    global config
    if not os.path.exists(CONFIG_FILE):
        return
    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            for k in config.keys():
                if k in data and isinstance(data[k], str):
                    config[k] = data[k]
        print("Config loaded from", CONFIG_FILE)
    except Exception as e:
        print("Failed to load config:", e)
 
def save_config_to_file():
    """Save current config to JSON file."""
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
        print("Config saved to", CONFIG_FILE)
    except Exception as e:
        print("Failed to save config:", e)
 
# try to load last config on startup
load_config_from_file()
 
# RTK status information (parsed from GGA)
rtk_info = {
    "fix_quality": "0",
    "fix_text":    "Invalid",
    "lat": "",         # WGS84 latitude
    "lon": "",         # WGS84 longitude
    "map_lat": "",     # GCJ-02 latitude (for Bing/China)
    "map_lon": "",     # GCJ-02 longitude
    "event": "",
    "offset_cm": "",   # position offset in cm
}
 
# reference point for offset (first Float/Fix)
bias_ref_lat = None
bias_ref_lon = None
 
# Data buffers
nmea_buffer  = deque(maxlen=200)   # NMEA text
rtcm_buffer  = deque(maxlen=200)   # RTCM hex preview
sourcetable_mountpoints = []       # Mountpoint list from NTRIP sourcetable
 
QUALITY_MAP = {
    "0": "Invalid",
    "1": "GPS Fix",
    "2": "DGPS",
    "4": "RTK Fixed",
    "5": "RTK Float",
}
 
# ---------- China / GCJ-02 transform ----------
PI = 3.14159265358979323846
A  = 6378245.0
EE = 0.00669342162296594323
 
def _out_of_china(lat, lon):
    if lon < 72.004 or lon > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False
 
def _transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320.0 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret
 
def _transform_lon(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret
 
def wgs84_to_gcj02(lat, lon):
    if _out_of_china(lat, lon):
        return lat, lon
    dlat = _transform_lat(lon - 105.0, lat - 35.0)
    dlon = _transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sqrtmagic) * PI)
    dlon = (dlon * 180.0) / (A / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglon = lon + dlon
    return mglat, mglon
 
def distance_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c
 
# ---------- Serial port scan ----------
def scan_serial_ports():
    patterns = [
        "/dev/ttyS*",
        "/dev/ttyAMA*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyXRUSB*",
    ]
    ports = []
    for pat in patterns:
        for p in glob.glob(pat):
            if os.path.exists(p):
                ports.append(p)
    return sorted(set(ports))
 
def list_existing_serials():
    return scan_serial_ports()
 
# ---------- HTML template ----------
HTML = """
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Raspberry Pi RTK NTRIP Tool</title>
<style>
body {
    font-family: sans-serif;
    max-width: 100%;
    margin: 20px;
    background: #87CEEB; /* sky blue */
}
h2 { margin-top: 0; }
label { display: inline-block; width: 130px; }
input[type=text], input[type=number], input[type=password] {
    width: 260px; padding: 4px; margin: 4px 0;
}
button { padding: 6px 16px; margin: 0 4px; }
fieldset { margin-top: 15px; }
/* Top buttons */
.top-buttons {
    display: flex;
    gap: 12px;
    margin-bottom: 16px;
}
.top-buttons form { margin: 0; }
/* Layout */
#main { display: flex; gap: 24px; align-items: flex-start; }
.col-left  { flex: 0 0 380px; min-width: 280px; }
.col-right { flex: 1.0; min-width: 0; }
/* Data stream */
.stream-row {
    display: flex;
    gap: 16px;
    margin-top: 12px;
}
.stream-col {
    flex: 1;
    min-width: 0;
}
.stream-box {
    width: 100%;
    height: 220px;
    background:#111;
    padding:6px;
    overflow-y:scroll;
    white-space:pre;
    font-size:12px;
}
#nmea_box { color:#0f0; }
#rtcm_box { color:#0ff; }
.status-box span.label { font-weight: bold; }
.status-box div { margin: 2px 0; }
.event-ok    { color: #00ff00; font-weight: bold; }
.event-float { color: #ffa500; font-weight: bold; }
/* Top RTK banner */
#rtk_banner {
    margin: 6px 0 12px;
    padding: 8px 12px;
    border-radius: 6px;
    font-weight: bold;
    font-size: 18px;
    background: #333;
    color: #ccc;
}
.banner-fixed {
    background: #004d00;
    color: #00ff00;
}
.banner-float {
    background: #4a2d00;
    color: #ffa500;
}
.banner-dgps {
    background: #002b36;
    color: #00e5ff;
}
.banner-none {
    background: #333;
    color: #ccc;
}
.fix-rtk   { color: #00ff00; font-weight:bold; font-size: 18px; }
.fix-float { color: #ffa500; font-weight:bold; font-size: 18px; }
.fix-dgps  { color: #00e5ff; font-weight:bold; font-size: 18px; }
</style>
</head>
<body>
<h2>Raspberry Pi RTK NTRIP Tool</h2>
<div class="top-buttons">
  <form method="post" action="{{ url_for('start') }}">
    <button type="submit">Start RTK</button>
  </form>
  <form method="post" action="{{ url_for('stop') }}">
    <button type="submit">Stop RTK</button>
  </form>
</div>
<div id="main">
<div class="col-left">
<form method="post" action="{{ url_for('start') }}">
  <fieldset>
    <legend>Serial Settings</legend>
    <label>Serial device:</label>
    <select name="serial_port_select" onchange="document.getElementById('serial_port').value=this.value;">
      <option value="">-- Select /dev/tty* --</option>
      {% for p in serial_ports %}
      <option value="{{ p }}" {% if p == config.serial_port %}selected{% endif %}>{{ p }}</option>
      {% endfor %}
    </select><br>
    <label>Serial (manual):</label>
    <input type="text" id="serial_port" name="serial_port"
           value="{{ config.serial_port }}" placeholder="/dev/ttyUSB0">
    <span> Baud: {{ baud }}</span>
  </fieldset>
  <fieldset>
    <legend>NTRIP Caster</legend>
    <label>Address:</label>
    <input type="text" name="caster_host" value="{{ config.caster_host }}"
           placeholder="IP or hostname"><br>
    <label>Port:</label>
    <input type="number" name="caster_port" value="{{ config.caster_port }}" placeholder="2101 / 8001"><br>
    <label>Mount Point:</label>
    <input type="text" name="mountpoint" id="mountpoint_input"
           value="{{ config.mountpoint }}" placeholder="e.g. HKCL_32 / RTCM33_GRCEJ"><br>
    <label>Source Table:</label>
    <select id="mountpoint_select"
            onchange="if(this.value){document.getElementById('mountpoint_input').value=this.value;}">
      <option value="">-- Click Update NTRIP Source Table --</option>
      {% for mp in sourcetable_mountpoints %}
      <option value="{{ mp }}">{{ mp }}</option>
      {% endfor %}
    </select><br>
    <button type="button" onclick="updateSourceTable()">Update NTRIP Source Table</button><br><br>
    <label>Username:</label>
    <input type="text" name="username" value="{{ config.username }}" placeholder=""><br>
    <label>Password:</label>
    <input type="text" name="password" value="{{ config.password }}" autocomplete="off" placeholder="">
  </fieldset>
  <fieldset>
    <legend>Send GNSS Command</legend>
    <label>Command:</label>
    <input type="text" name="cmd" id="cmd_input" value="$PQTMSRR*4B">
    <button type="submit" formaction="{{ url_for('send_cmd') }}">Send</button>
    <button type="button" onclick="document.getElementById('cmd_input').value='$PQTMSRR*4B';">
      PQTMSRR
    </button>
  </fieldset>
  <div style="margin-top:10px;">
    <button type="submit">Start RTK</button>
    <button type="submit" formaction="{{ url_for('save_config') }}">Save config</button>
    <button type="submit" formaction="{{ url_for('load_config_route') }}">Load config</button>
  </div>
</form>
</div> <!-- col-left -->
<div class="col-right">
<h3>Status</h3>
<div id="rtk_banner" class="banner-none">
  Waiting for GNSS fix...
</div>
<div class="status-box">
  <div><span class="label">Serial:</span> <span id="serial_status">{{ status.serial }}</span></div>
  <div><span class="label">NTRIP:</span> <span id="ntrip_status">{{ status.ntrip }}</span></div>
  <div><span class="label">Message:</span> <span id="msg_status">{{ status.message }}</span></div>
  <div><span class="label">Fix:</span> <span id="fix_text">{{ rtk_info.fix_text }}</span></div>
  <div><span class="label">Latitude (WGS84):</span> <span id="lat_text">{{ rtk_info.lat }}</span></div>
  <div><span class="label">Longitude (WGS84):</span> <span id="lon_text">{{ rtk_info.lon }}</span></div>
  <div><span class="label">Map Lat (GCJ/China):</span> <span id="map_lat_text">{{ rtk_info.map_lat }}</span></div>
  <div><span class="label">Map Lon (GCJ/China):</span> <span id="map_lon_text">{{ rtk_info.map_lon }}</span></div>
  <div><span class="label">Last GGA time:</span> <span id="gga_time">{{ last_gga_time }}</span></div>
  <div><span class="label">RTK Event:</span>
      <span id="event_text" class="">
        {{ rtk_info.event }}
      </span>
  </div>
  <div><span class="label">Position offset (cm):</span> <span id="offset_text">{{ rtk_info.offset_cm }}</span></div>
  <div>
    <span class="label">Bing Map:</span>
    <a id="bing_link" href="#" target="_blank" style="opacity:0.5;pointer-events:none;">
      No coordinate yet
    </a>
  </div>
</div>
<div class="stream-row">
  <div class="stream-col">
    <h3>RTCM Stream (from caster, hex)</h3>
    <div id="rtcm_box" class="stream-box"></div>
  </div>
  <div class="stream-col">
    <h3>Live NMEA Stream (from receiver)</h3>
    <div id="nmea_box" class="stream-box"></div>
  </div>
</div>
</div> <!-- col-right -->
</div> <!-- main -->
<script>
function updateStatus() {
  fetch('/api/status').then(r => r.json()).then(s => {
    document.getElementById('serial_status').textContent = s.serial;
    document.getElementById('ntrip_status').textContent = s.ntrip;
    document.getElementById('msg_status').textContent = s.message;
    document.getElementById('lat_text').textContent = s.lat;
    document.getElementById('lon_text').textContent = s.lon;
    document.getElementById('map_lat_text').textContent = s.map_lat || '';
    document.getElementById('map_lon_text').textContent = s.map_lon || '';
    document.getElementById('gga_time').textContent = s.last_gga_time;
    document.getElementById('offset_text').textContent = s.offset_cm || '';
    const fixSpan = document.getElementById('fix_text');
    fixSpan.textContent = s.fix_text;
    fixSpan.className = '';
    const banner = document.getElementById('rtk_banner');
    banner.className = 'banner-none';
    banner.textContent = 'Waiting for GNSS fix...';
    const ev = document.getElementById('event_text');
    ev.textContent = s.event;
    ev.className = '';
    if (s.fix_text === 'RTK Fixed') {
      fixSpan.className = 'fix-rtk';
      banner.className = 'banner-fixed';
      banner.textContent = 'RTK FIXED (cm-level)';
      ev.className = 'event-ok';
    } else if (s.fix_text === 'RTK Float') {
      fixSpan.className = 'fix-float';
      banner.className = 'banner-float';
      banner.textContent = 'RTK FLOAT (decimeter-level)';
      ev.className = 'event-float';
    } else if (s.fix_text === 'DGPS') {
      fixSpan.className = 'fix-dgps';
      banner.className = 'banner-dgps';
      banner.textContent = 'DGPS (no RTK yet)';
    }
    const bingLink = document.getElementById('bing_link');
    const useLat = s.map_lat || s.lat;
    const useLon = s.map_lon || s.lon;
    if (useLat && useLon) {
      const coord = useLat + ',' + useLon;
      const url = 'https://www.bing.com/maps?q=' + encodeURIComponent(coord);
      bingLink.href = url;
      bingLink.textContent = coord + ' (Open in Bing)';
      bingLink.style.opacity = '1';
      bingLink.style.pointerEvents = 'auto';
    } else {
      bingLink.href = '#';
      bingLink.textContent = 'No coordinate yet';
      bingLink.style.opacity = '0.5';
      bingLink.style.pointerEvents = 'none';
    }
  });
}
function updateNmea() {
  fetch('/api/nmea').then(r => r.text()).then(t => {
    const box = document.getElementById('nmea_box');
    box.textContent = t;
    box.scrollTop = box.scrollHeight;
  });
}
function updateRtcm() {
  fetch('/api/rtcm').then(r => r.text()).then(t => {
    const box = document.getElementById('rtcm_box');
    box.textContent = t;
    box.scrollTop = box.scrollHeight;
  });
}
function updateSourceTable() {
  const host = document.querySelector('input[name="caster_host"]').value;
  const port = document.querySelector('input[name="caster_port"]').value;
  const user = document.querySelector('input[name="username"]').value;
  const pwd  = document.querySelector('input[name="password"]').value;
  if (!host || !port) {
    alert('Please fill caster host and port first.');
    return;
  }
  fetch('/api/update_sourcetable', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({
      caster_host: host,
      caster_port: port,
      username:    user,
      password:    pwd
    })
  })
  .then(r => r.json())
  .then(data => {
    const sel = document.getElementById('mountpoint_select');
    sel.innerHTML = '';
    if (!data.ok) {
      alert('Update NTRIP source table failed: ' + (data.error || 'unknown error'));
      const opt = document.createElement('option');
      opt.value = '';
      opt.textContent = '-- No STR lines found --';
      sel.appendChild(opt);
      return;
    }
    const firstOpt = document.createElement('option');
    firstOpt.value = '';
    firstOpt.textContent = '-- Select mount point --';
    sel.appendChild(firstOpt);
    data.mountpoints.forEach(mp => {
      const opt = document.createElement('option');
      opt.value = mp;
      opt.textContent = mp;
      sel.appendChild(opt);
    });
  })
  .catch(err => {
    alert('Update NTRIP source table error: ' + err);
  });
}
setInterval(updateStatus, 1000);
setInterval(updateNmea, 1000);
setInterval(updateRtcm, 1000);
updateStatus();
updateNmea();
updateRtcm();
</script>
</body>
</html>
"""
 
# ---------- Helper functions ----------
def parse_lat_lon_from_gga(fields):
    lat = lon = ""
    if len(fields) > 4 and fields[2] and fields[3]:
        v = fields[2]
        deg = int(v[0:2])
        minutes = float(v[2:])
        lat_val = deg + minutes / 60.0
        if fields[3] == 'S':
            lat_val = -lat_val
        lat = f"{lat_val:.8f}"
    if len(fields) > 6 and fields[4] and fields[5]:
        v = fields[4]
        deg = int(v[0:3])
        minutes = float(v[3:])
        lon_val = deg + minutes / 60.0
        if fields[5] == 'W':
            lon_val = -lon_val
        lon = f"{lon_val:.8f}"
    return lat, lon
 
def fetch_ntrip_sourcetable(host, port, user="", password=""):
    mp_list = []
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10)
    s.connect((host, int(port)))
 
    req  = "GET / HTTP/1.1\r\n"
    req += f"Host: {host}\r\n"
    req += "User-Agent: NTRIP client\r\n"
    req += "Ntrip-Version: Ntrip/2.0\r\n"
    req += "Connection: close\r\n"
    if user or password:
        auth_str = f"{user}:{password}"
        auth_b64 = base64.b64encode(auth_str.encode()).decode()
        req += f"Authorization: Basic {auth_b64}\r\n"
    req += "\r\n"
 
    s.sendall(req.encode("ascii", errors="ignore"))
 
    data = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        data += chunk
    s.close()
 
    text = data.decode(errors="ignore")
    for line in text.splitlines():
        line_stripped = line.strip()
        if line_stripped.upper().startswith("STR;"):
            parts = line_stripped.split(";")
            if len(parts) > 1 and parts[1]:
                mp_list.append(parts[1])
 
    seen = set()
    ordered = []
    for mp in mp_list:
        if mp not in seen:
            seen.add(mp)
            ordered.append(mp)
    return ordered
 
# ---------- Serial thread ----------
def serial_reader():
    global last_gga, last_gga_time, serial_port, rtk_info
    global bias_ref_lat, bias_ref_lon
 
    status["serial"] = "Connected, reading NMEA..."
    prev_fix_quality = rtk_info["fix_quality"]
 
    try:
        while not stop_event.is_set():
            with serial_lock:
                if serial_port is None:
                    status["serial"] = "Serial closed"
                    break
                line = serial_port.readline()
            if not line:
                continue
 
            try:
                s = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
 
            if not s:
                continue
 
            nmea_buffer.append(s)
 
            if s.startswith("$GPGGA") or s.startswith("$GNGGA"):
                last_gga = s
                last_gga_time = time.time()
 
                fields = s.split(',')
                if len(fields) > 6:
                    fix_q = fields[6]
                    rtk_info["fix_quality"] = fix_q
                    rtk_info["fix_text"] = QUALITY_MAP.get(fix_q, "Unknown")
 
                    lat, lon = parse_lat_lon_from_gga(fields)
                    if lat:
                        rtk_info["lat"] = lat
                    if lon:
                        rtk_info["lon"] = lon
 
                    try:
                        if lat and lon:
                            wlat = float(lat)
                            wlon = float(lon)
 
                            mlat, mlon = wgs84_to_gcj02(wlat, wlon)
                            rtk_info["map_lat"] = f"{mlat:.8f}"
                            rtk_info["map_lon"] = f"{mlon:.8f}"
 
                            # use first Float/Fix point as reference
                            if fix_q in ("4", "5"):
                                if bias_ref_lat is None or bias_ref_lon is None:
                                    bias_ref_lat = wlat
                                    bias_ref_lon = wlon
                                d_m = distance_m(bias_ref_lat, bias_ref_lon, wlat, wlon)
                                rtk_info["offset_cm"] = f"{d_m * 100.0:.1f} cm"
                    except Exception:
                        rtk_info["map_lat"] = ""
                        rtk_info["map_lon"] = ""
 
                    if fix_q != prev_fix_quality:
                        if fix_q == "4":
                            rtk_info["event"] = f"RTK FIXED at {rtk_info['lat']}, {rtk_info['lon']}"
                        elif fix_q == "5":
                            rtk_info["event"] = f"RTK FLOAT at {rtk_info['lat']}, {rtk_info['lon']}"
                        elif fix_q in ("1", "2"):
                            rtk_info["event"] = "GNSS fix acquired"
                        else:
                            rtk_info["event"] = "No valid fix"
                        prev_fix_quality = fix_q
 
    except Exception as e:
        status["serial"] = f"Serial error: {e}"
    finally:
        with serial_lock:
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
                serial_port = None
        status["serial"] = "Disconnected"
 
# ---------- NTRIP thread ----------
def ntrip_client():
    global status, last_gga, last_gga_time, serial_port
 
    host = (config.get("caster_host") or "").strip()
    port = config.get("caster_port") or ""
    mount = (config.get("mountpoint") or "").lstrip("/")
    user = (config.get("username") or "").strip()
    password = (config.get("password") or "").strip()
 
    if not host or not port or not mount:
        status["ntrip"] = "Caster not configured"
        return
 
    while not stop_event.is_set():
        try:
            status["ntrip"] = f"Connecting {host}:{port}..."
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, int(port)))
 
            auth_str = f"{user}:{password}"
            auth_enc = base64.b64encode(auth_str.encode()).decode()
            request = (
                f"GET /{mount} HTTP/1.1\r\n"
                f"User-Agent: NTRIP client\r\n"
                f"Accept: */*\r\n"
                f"Connection: keep-alive\r\n"
                f"Authorization: Basic {auth_enc}\r\n"
                "\r\n"
            )
            sock.sendall(request.encode("ascii", errors="ignore"))
            print(f"Connected to NTRIP {host}:{port}")
            status["ntrip"] = "Connected, waiting RTCM..."
 
            sock.settimeout(0.1)
            last_gga_sent = 0.0
 
            while not stop_event.is_set():
                try:
                    data = sock.recv(1024)
                    if not data:
                        print("NTRIP server closed connection")
                        status["ntrip"] = "NTRIP closed, reconnecting..."
                        break
                    rtcm_buffer.append(hexlify(data[:64]).decode())
                    with serial_lock:
                        if serial_port is not None:
                            serial_port.write(data)
                except socket.timeout:
                    pass
 
                if last_gga and (time.time() - last_gga_sent > 5):
                    send_line = last_gga + "\r\n"
                    try:
                        sock.sendall(send_line.encode("ascii", errors="ignore"))
                        print(f"[GGA sent to NTRIP] {last_gga}")
                        last_gga_sent = time.time()
                    except Exception as e:
                        print(f"Send GGA failed: {e}")
                        status["ntrip"] = f"NTRIP error: send GGA failed ({e})"
                        break
 
                time.sleep(0.01)
 
            try:
                sock.close()
            except Exception:
                pass
 
        except Exception as e:
            status["ntrip"] = f"NTRIP error: {e}"
            try:
                sock.close()
            except Exception:
                pass
 
        if not stop_event.is_set():
            time.sleep(1)
 
    status["ntrip"] = "Disconnected"
 
# ---------- Flask routes ----------
@app.route("/", methods=["GET"])
def index():
    if last_gga_time:
        gga_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(last_gga_time))
    else:
        gga_time_str = ""
    return render_template_string(
        HTML,
        config=config,
        status=status,
        rtk_info=rtk_info,
        last_gga_time=gga_time_str,
        serial_ports=list_existing_serials(),
        baud=BAUD_RATE,
        sourcetable_mountpoints=sourcetable_mountpoints,
    )
 
@app.route("/start", methods=["POST"])
def start():
    global serial_port, serial_thread, ntrip_thread, stop_event
    global bias_ref_lat, bias_ref_lon
 
    sp_form  = request.form.get("serial_port") or request.form.get("serial_port_select")
    sp_final = (sp_form or "").strip()
    config["serial_port"] = sp_final
 
    config["caster_host"] = (request.form.get("caster_host") or "").strip()
    config["caster_port"] = request.form.get("caster_port") or ""
    config["mountpoint"]  = (request.form.get("mountpoint") or "").strip()
    config["username"]    = (request.form.get("username") or "").strip()
    config["password"]    = (request.form.get("password") or "").strip()
 
    if not config["serial_port"]:
        status["serial"] = "No serial port selected"
        status["message"] = "Please select a /dev/tty* for GNSS."
        return redirect(url_for("index"))
 
    stop_event.set()
    time.sleep(0.5)
    stop_event = threading.Event()
    status["message"] = ""
 
    try:
        with serial_lock:
            if serial_port is not None:
                try:
                    serial_port.close()
                except Exception:
                    pass
            serial_port = serial.Serial(config["serial_port"], BAUD_RATE, timeout=1)
        status["serial"] = f"Connected {config['serial_port']} @ {BAUD_RATE}"
    except Exception as e:
        status["serial"] = f"Failed to open serial: {e}"
        status["message"] = "Check serial device & permission"
        return redirect(url_for("index"))
 
    rtk_info["fix_quality"] = "0"
    rtk_info["fix_text"] = "Invalid"
    rtk_info["lat"] = ""
    rtk_info["lon"] = ""
    rtk_info["map_lat"] = ""
    rtk_info["map_lon"] = ""
    rtk_info["event"] = ""
    rtk_info["offset_cm"] = ""
    bias_ref_lat = None
    bias_ref_lon = None
 
    nmea_buffer.clear()
    rtcm_buffer.clear()
 
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    ntrip_thread = threading.Thread(target=ntrip_client, daemon=True)
    ntrip_thread.start()
 
    status["message"] = "RTK started"
    return redirect(url_for("index"))
 
@app.route("/stop", methods=["POST"])
def stop():
    global stop_event, serial_port
    stop_event.set()
    with serial_lock:
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
            serial_port = None
    status["serial"] = "Disconnected"
    status["ntrip"] = "Disconnected"
    status["message"] = "RTK stopped"
    return redirect(url_for("index"))
 
@app.route("/save_config", methods=["POST"])
def save_config():
    """Save current form values into config file."""
    sp_form  = request.form.get("serial_port") or request.form.get("serial_port_select")
    config["serial_port"] = (sp_form or "").strip()
    config["caster_host"] = (request.form.get("caster_host") or "").strip()
    config["caster_port"] = request.form.get("caster_port") or ""
    config["mountpoint"]  = (request.form.get("mountpoint") or "").strip()
    config["username"]    = (request.form.get("username") or "").strip()
    config["password"]    = (request.form.get("password") or "").strip()
 
    save_config_to_file()
    status["message"] = "Config saved to file"
    return redirect(url_for("index"))
 
@app.route("/load_config", methods=["POST"])
def load_config_route():
    """Reload config from file and refresh page."""
    load_config_from_file()
    status["message"] = "Config loaded from file"
    return redirect(url_for("index"))
 
@app.route("/send_cmd", methods=["POST"])
def send_cmd():
    cmd = (request.form.get("cmd") or "").strip()
    if not cmd:
        status["message"] = "No command to send"
        return redirect(url_for("index"))
 
    to_send = cmd
    if not to_send.endswith("\r") and not to_send.endswith("\n"):
        to_send = to_send + "\r\n"
 
    with serial_lock:
        if serial_port is None:
            status["message"] = "Serial not open, cannot send command"
        else:
            try:
                serial_port.write(to_send.encode("ascii", errors="ignore"))
                status["message"] = f"Sent command: {cmd}"
            except Exception as e:
                status["message"] = f"Send cmd error: {e}"
    return redirect(url_for("index"))
 
@app.route("/api/status")
def api_status():
    if last_gga_time:
        gga_time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(last_gga_time))
    else:
        gga_time_str = ""
    return jsonify({
        "serial": status["serial"],
        "ntrip": status["ntrip"],
        "message": status["message"],
        "fix_text": rtk_info["fix_text"],
        "lat": rtk_info["lat"],
        "lon": rtk_info["lon"],
        "map_lat": rtk_info["map_lat"],
        "map_lon": rtk_info["map_lon"],
        "last_gga_time": gga_time_str,
        "event": rtk_info["event"],
        "offset_cm": rtk_info["offset_cm"],
    })
 
@app.route("/api/nmea")
def api_nmea():
    return "\n".join(nmea_buffer)
 
@app.route("/api/rtcm")
def api_rtcm():
    return "\n".join(rtcm_buffer)
 
@app.route("/api/update_sourcetable", methods=["POST"])
def api_update_sourcetable():
    global sourcetable_mountpoints
    data = request.get_json(silent=True) or {}
 
    host = (data.get("caster_host") or "").strip()
    port = data.get("caster_port") or ""
    user = (data.get("username") or "").strip()
    password = (data.get("password") or "").strip()
 
    if not host or not port:
        return jsonify({"ok": False, "error": "Host/port not set", "mountpoints": []})
 
    try:
        mp_list = fetch_ntrip_sourcetable(host, port, user=user, password=password)
        sourcetable_mountpoints = mp_list
        if not mp_list:
            return jsonify({
                "ok": False,
                "error": "No STR lines found in sourcetable",
                "mountpoints": []
            })
        return jsonify({"ok": True, "mountpoints": mp_list})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "mountpoints": []})
 
# ---------- main ----------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=HTTP_PORT, debug=False)
#!/usr/bin/env python3
"""
Tactical Web Ground Control & Evaluation Station (Web C2) for Octopus Interceptor & Talon Target.
Powered by FastAPI, WebSockets, HTML5 Canvas Radar, and Chart.js.

Usage:
  python3 tools/web_gcs.py [--port 8080]
"""

import os
import sys
import json
import math
import time
import asyncio
import threading
import subprocess
import urllib.request
from collections import deque

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["MAVLINK20"] = "1"

import numpy as np
from pymavlink import mavutil
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))

app = FastAPI(title="Octopus Tactical Web GCS")

# Shared state
state_lock = threading.Lock()
telemetry_data = {
    "interceptor": {
        "connected": False,
        "armed": False,
        "mode": "STANDBY",
        "pos": [0.0, 0.0, 0.0], # North, East, Down
        "vel": [0.0, 0.0, 0.0],
        "att": [0.0, 0.0, 0.0], # Roll, Pitch, Yaw in degrees
        "speed": 0.0,
        "alt": 0.0,
        "last_update": 0.0
    },
    "talon": {
        "connected": False,
        "mode": "square",
        "target_speed": 22.0,
        "target_alt": 100.0,
        "pos": [0.0, 0.0, 0.0], # North, East, Down
        "vel": [0.0, 0.0, 0.0],
        "att": [0.0, 0.0, 0.0],
        "speed": 0.0,
        "alt": 0.0,
        "heading": 0.0,
        "last_update": 0.0
    },
    "tactical": {
        "separation_distance": 0.0,
        "closing_speed": 0.0,
        "guidance_phase": "STANDBY",
        "fps_mode": 50,
        "lock_percent": 0.0,
        "net_deployed": False
    }
}

lock_start_time = None
history_times = deque(maxlen=200)
history_dist = deque(maxlen=200)
history_v_int = deque(maxlen=200)
history_v_tgt = deque(maxlen=200)
int_trail = deque(maxlen=300)
tgt_trail = deque(maxlen=300)

# ---------------------------------------------------- Gazebo Transport Worker ---

def gz_transport_worker_thread(world="ankara", interceptor_name="interceptor_0", target_name="talon1718_1"):
    global telemetry_data
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except Exception as e:
        print("Gazebo transport import notice in Web GCS:", e, flush=True)
        return

    node = Node()
    prev_pos = None
    prev_t = 0.0

    def on_pose(msg):
        nonlocal prev_pos, prev_t
        now = time.time()
        for p in msg.pose:
            if p.name == interceptor_name:
                pos = p.position
                ori = p.orientation

                # Quaternion -> Roll, Pitch, Yaw
                qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                sinp = 2 * (qw * qy - qz * qx)
                pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                # Numerical velocity from pose
                vx, vy, vz = 0.0, 0.0, 0.0
                if prev_pos is not None and now > prev_t:
                    dt = max(0.001, now - prev_t)
                    vx = (pos.x - prev_pos[0]) / dt
                    vy = (pos.y - prev_pos[1]) / dt
                    vz = (pos.z - prev_pos[2]) / dt

                prev_pos = (pos.x, pos.y, pos.z)
                prev_t = now
                spd = float(math.hypot(vx, vy))

                with state_lock:
                    inter = telemetry_data["interceptor"]
                    inter["connected"] = True
                    inter["last_update"] = now
                    # Gazebo ENU -> NED for local convention:
                    inter["pos"] = [round(pos.y, 3), round(pos.x, 3), round(-pos.z, 3)]
                    inter["vel"] = [round(vy, 3), round(vx, 3), round(-vz, 3)]
                    inter["speed"] = round(spd, 2)
                    inter["alt"] = round(pos.z, 2)
                    inter["att"] = [round(roll, 1), round(pitch, 1), round(yaw, 1)]
                    int_trail.append([round(pos.x, 2), round(pos.y, 2)])

    topic = f"/world/{world}/dynamic_pose/info"
    node.subscribe(Pose_V, topic, on_pose)
    while True:
        time.sleep(1.0)


# ----------------------------------------------------------- MAVLink Worker ---

def mavlink_worker_thread(primary_port=14545, fallback_ports=(14540, 14550, 14551)):
    global telemetry_data
    ports_to_try = [primary_port] + list(fallback_ports)
    link = None
    port_idx = 0

    while True:
        if link is None:
            port = ports_to_try[port_idx % len(ports_to_try)]
            try:
                link = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
            except Exception:
                port_idx += 1
                time.sleep(1.0)
                continue

        try:
            msg = link.recv_match(blocking=True, timeout=1.0)
            if not msg:
                continue

            mtype = msg.get_type()
            now = time.time()

            with state_lock:
                inter = telemetry_data["interceptor"]
                inter["connected"] = True
                inter["last_update"] = now

                if mtype == "LOCAL_POSITION_NED":
                    inter["pos"] = [round(msg.x, 3), round(msg.y, 3), round(msg.z, 3)]
                    inter["vel"] = [round(msg.vx, 3), round(msg.vy, 3), round(msg.vz, 3)]
                    inter["speed"] = round(float(np.hypot(msg.vx, msg.vy)), 2)
                    inter["alt"] = round(-msg.z, 2)
                    int_trail.append([round(msg.y, 2), round(msg.x, 2)])

                elif mtype == "ATTITUDE":
                    inter["att"] = [
                        round(math.degrees(msg.roll), 1),
                        round(math.degrees(msg.pitch), 1),
                        round(math.degrees(msg.yaw), 1)
                    ]

                elif mtype == "HEARTBEAT":
                    inter["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    custom = msg.custom_mode
                    main_mode = (custom >> 16) & 0xFF
                    sub_mode = (custom >> 24) & 0xFF
                    if main_mode == 1:
                        inter["mode"] = "MANUAL"
                    elif main_mode == 2:
                        inter["mode"] = "ALTCTL"
                    elif main_mode == 3:
                        inter["mode"] = "POSCTL"
                    elif main_mode == 4:
                        if sub_mode == 3:
                            inter["mode"] = "HOLD"
                        elif sub_mode == 4:
                            inter["mode"] = "TAKEOFF"
                        elif sub_mode == 5:
                            inter["mode"] = "LAND"
                        else:
                            inter["mode"] = "AUTO"
                    elif main_mode == 6:
                        inter["mode"] = "INTERCEPT"
                    else:
                        inter["mode"] = f"MODE_{main_mode}"

        except Exception:
            link = None
            port_idx += 1
            time.sleep(0.5)


# -------------------------------------------------------- Target Poller ---

def target_poller_thread(http_port=8000, rate=20.0):
    global telemetry_data
    dt = 1.0 / rate
    url = f"http://127.0.0.1:{http_port}/target"

    while True:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "WebGCS"})
            with urllib.request.urlopen(req, timeout=0.4) as resp:
                if resp.status == 200:
                    d = json.loads(resp.read().decode("utf-8"))
                    now = time.time()
                    with state_lock:
                        talon = telemetry_data["talon"]
                        talon["connected"] = d.get("valid", False)
                        if talon["connected"]:
                            talon["last_update"] = now
                            # Position
                            if "pos_east_m" in d and "pos_north_m" in d:
                                de = d["pos_east_m"]
                                dn = d["pos_north_m"]
                            else:
                                lat0, lon0 = 39.930898, 32.729591
                                dn = (d.get("lat", lat0) - lat0) * 111132.95
                                de = (d.get("lon", lon0) - lon0) * (111319.49 * math.cos(math.radians(lat0)))

                            alt = d.get("alt_rel_m", 0.0)
                            talon["pos"] = [round(dn, 3), round(de, 3), round(-alt, 3)]
                            talon["vel"] = [
                                round(d.get("vn_mps", 0.0), 3),
                                round(d.get("ve_mps", 0.0), 3),
                                round(d.get("vd_mps", 0.0), 3)
                            ]
                            talon["speed"] = round(d.get("ground_speed_mps", 0.0), 2)
                            talon["alt"] = round(alt, 2)
                            talon["heading"] = round(d.get("heading_deg", 0.0), 1)
                            talon["mode"] = d.get("mode", "square")
                            talon["target_speed"] = round(d.get("target_speed", 22.0), 1)
                            talon["target_alt"] = round(d.get("target_alt", 100.0), 1)
                            talon["att"] = [
                                round(d.get("roll_deg", 0.0), 1),
                                round(d.get("pitch_deg", 0.0), 1),
                                round(d.get("heading_deg", 0.0), 1)
                            ]
                            tgt_trail.append([round(de, 2), round(dn, 2)])
        except Exception:
            with state_lock:
                telemetry_data["talon"]["connected"] = False

        time.sleep(dt)


# ---------------------------------------------------- Tactical Computations ---

def tactical_loop_thread(rate=30.0):
    global telemetry_data, lock_start_time
    dt = 1.0 / rate
    t0 = time.time()

    while True:
        now = time.time()
        with state_lock:
            inter = telemetry_data["interceptor"]
            talon = telemetry_data["talon"]
            tac = telemetry_data["tactical"]

            # Connection timeouts
            if now - inter["last_update"] > 2.0:
                inter["connected"] = False
            if now - talon["last_update"] > 2.0:
                talon["connected"] = False

            if inter["connected"] and talon["connected"]:
                p_int = np.array(inter["pos"])
                p_tgt = np.array(talon["pos"])
                v_int = np.array(inter["vel"])
                v_tgt = np.array(talon["vel"])

                d_vec = p_tgt - p_int
                dist = float(np.linalg.norm(d_vec))
                r_unit = d_vec / max(dist, 0.01)
                closing_vel = float(np.dot(v_int - v_tgt, r_unit))

                tac["separation_distance"] = round(dist, 2)
                tac["closing_speed"] = round(closing_vel, 2)

                # Phase determination
                if dist > 40.0:
                    tac["guidance_phase"] = "MIDCOURSE (GPS İntikali)"
                    tac["fps_mode"] = 50
                elif dist > 20.0:
                    tac["guidance_phase"] = "TERMINAL (Optik 50 FPS)"
                    tac["fps_mode"] = 50
                elif dist > 5.5:
                    tac["guidance_phase"] = "TERMINAL (Yüksek Hız 80 FPS)"
                    tac["fps_mode"] = 80
                else:
                    tac["guidance_phase"] = "AĞ MENZİLİNDE (KİLİTLİ)"
                    tac["fps_mode"] = 80

                # Net lock progress
                if dist <= 5.5:
                    if lock_start_time is None:
                        lock_start_time = now
                    elapsed = now - lock_start_time
                    pct = min(100.0, (elapsed / 0.40) * 100.0)
                    tac["lock_percent"] = round(pct, 1)
                    if pct >= 100.0:
                        tac["net_deployed"] = True
                else:
                    lock_start_time = None
                    tac["lock_percent"] = 0.0

                # History
                t_rel = round(now - t0, 2)
                history_times.append(t_rel)
                history_dist.append(round(dist, 2))
                history_v_int.append(inter["speed"])
                history_v_tgt.append(talon["speed"])

        time.sleep(dt)


# ----------------------------------------------------------- REST Endpoints ---

@app.post("/api/talon/cmd")
async def talon_cmd(payload: dict):
    """Forward command to target_sim.py HTTP server."""
    try:
        url = "http://127.0.0.1:8000/cmd"
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return JSONResponse({"status": "ok", "applied": json.loads(resp.read().decode())})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/interceptor/cmd")
async def interceptor_cmd(payload: dict):
    """Execute command via px4cmd.sh."""
    action = payload.get("action", "")
    px4cmd = os.path.join(SCRIPT_DIR, "px4cmd.sh")

    if action == "takeoff":
        cmd = [px4cmd, "0", "commander", "takeoff"]
    elif action == "intercept":
        cmd = [px4cmd, "0", "commander", "mode", "ext1"]
    elif action == "hold":
        cmd = [px4cmd, "0", "commander", "mode", "posctl"]
    elif action == "land":
        cmd = [px4cmd, "0", "commander", "mode", "auto:land"]
    else:
        return JSONResponse({"status": "error", "message": f"Unknown action: {action}"}, status_code=400)

    try:
        subprocess.Popen(cmd)
        return JSONResponse({"status": "ok", "action": action})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/tools/hud")
async def launch_hud():
    """Launch HUD viewer."""
    try:
        cmd = [os.path.join(SCRIPT_DIR, "view.sh")]
        subprocess.Popen(cmd)
        return JSONResponse({"status": "ok", "message": "HUD başlatıldı"})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/tools/qgc")
async def launch_qgc():
    """Launch QGroundControl."""
    qgc_path = "/home/kayra/Applications/QGroundControl-x86_64.AppImage"
    if os.path.exists(qgc_path):
        subprocess.Popen([qgc_path])
        return JSONResponse({"status": "ok", "message": "QGroundControl başlatıldı"})
    return JSONResponse({"status": "error", "message": "QGC AppImage bulunamadı"}, status_code=404)


# ------------------------------------------------------------- WebSocket ---

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            with state_lock:
                packet = {
                    "interceptor": telemetry_data["interceptor"],
                    "talon": telemetry_data["talon"],
                    "tactical": telemetry_data["tactical"],
                    "trails": {
                        "interceptor": list(int_trail),
                        "talon": list(tgt_trail)
                    },
                    "charts": {
                        "times": list(history_times),
                        "dist": list(history_dist),
                        "v_int": list(history_v_int),
                        "v_tgt": list(history_v_tgt)
                    }
                }
            await ws.send_text(json.dumps(packet))
            await asyncio.sleep(0.04) # 25 Hz stream
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# --------------------------------------------------------- Single Page HTML ---

HTML_CONTENT = """<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>OCTOPUS C2 // Taktik Görev & Önleme İstasyonu</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
  <style>
    @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;800&family=Rajdhani:wght@500;600;700&display=swap');
    
    body {
      background-color: #0b0f17;
      color: #e2e8f0;
      font-family: 'Rajdhani', sans-serif;
      overflow-x: hidden;
    }
    .mono { font-family: 'JetBrains Mono', monospace; }
    
    /* Neon Glow Effects */
    .glow-cyan { text-shadow: 0 0 10px rgba(0, 229, 255, 0.6); }
    .glow-green { text-shadow: 0 0 10px rgba(0, 255, 136, 0.6); }
    .glow-amber { text-shadow: 0 0 10px rgba(255, 183, 0, 0.6); }
    .glow-red { text-shadow: 0 0 10px rgba(255, 59, 48, 0.6); }
    
    .card-glass {
      background: rgba(16, 23, 38, 0.85);
      backdrop-filter: blur(8px);
      border: 1px solid rgba(45, 55, 72, 0.7);
    }
    
    /* Custom Scrollbar */
    ::-webkit-scrollbar { width: 6px; height: 6px; }
    ::-webkit-scrollbar-track { background: #0b0f17; }
    ::-webkit-scrollbar-thumb { background: #2d3748; border-radius: 3px; }
  </style>
</head>
<body class="p-3">

  <!-- TOP HEADER -->
  <header class="card-glass rounded-lg p-3 mb-3 flex flex-wrap items-center justify-between border-b-2 border-cyan-500/30">
    <div class="flex items-center space-x-3">
      <div class="w-10 h-10 rounded bg-cyan-950 border border-cyan-400 flex items-center justify-center text-cyan-400 font-bold text-xl">
        <i class="fa-solid fa-crosshairs animate-pulse"></i>
      </div>
      <div>
        <h1 class="text-xl font-bold tracking-wider text-cyan-400">OCTOPUS INTERCEPTOR C2</h1>
        <p class="text-xs text-slate-400 tracking-widest font-mono">100 HZ PREDICTIVE GUIDANCE // TACTICAL EVALUATION BENCH</p>
      </div>
    </div>

    <!-- Quick Status Badges -->
    <div class="flex items-center space-x-3 text-xs mono">
      <button onclick="launchHUD()" class="bg-slate-800 hover:bg-slate-700 text-slate-200 px-3 py-1.5 rounded border border-slate-600 transition flex items-center space-x-1.5">
        <i class="fa-solid fa-video text-amber-400"></i><span>HUD VİZÖRÜ</span>
      </button>
      <button onclick="launchQGC()" class="bg-slate-800 hover:bg-slate-700 text-slate-200 px-3 py-1.5 rounded border border-slate-600 transition flex items-center space-x-1.5">
        <i class="fa-solid fa-satellite-dish text-blue-400"></i><span>QGROUNDCONTROL</span>
      </button>

      <div id="badge-talon" class="bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2">
        <span class="w-2 h-2 rounded-full bg-red-500 animate-ping"></span>
        <span id="txt-talon-badge">TALON: BEKLENİYOR</span>
      </div>

      <div id="badge-interceptor" class="bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2">
        <span class="w-2 h-2 rounded-full bg-red-500 animate-ping"></span>
        <span id="txt-int-badge">ÖNLEYİCİ: BEKLENİYOR</span>
      </div>
    </div>
  </header>

  <!-- MAIN WORKSPACE: 3 COLUMNS -->
  <div class="grid grid-cols-12 gap-3 mb-3">

    <!-- LEFT COLUMN: TALON CONTROLLER (3 Cols) -->
    <div class="col-span-12 lg:col-span-3 space-y-3">
      
      <!-- Scenario Selector -->
      <div class="card-glass rounded-lg p-3.5 border-l-4 border-red-500">
        <div class="flex items-center justify-between mb-2.5">
          <h2 class="text-base font-bold text-red-400 tracking-wide uppercase"><i class="fa-solid fa-plane text-sm mr-2"></i>Hedef Talon 1718 Senaryoları</h2>
          <span id="talon-mode-badge" class="mono text-xs bg-red-900/60 text-red-200 px-2 py-0.5 rounded border border-red-700">SQUARE</span>
        </div>
        
        <div class="grid grid-cols-2 gap-2 text-xs font-semibold">
          <button onclick="setTalonMode('square')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-regular fa-square text-red-400"></i><span>Kare Devriye</span>
          </button>
          <button onclick="setTalonMode('weave')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-water text-amber-400"></i><span>S-Kaçış (Weave)</span>
          </button>
          <button onclick="setTalonMode('circle')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrows-spin text-purple-400"></i><span>Dairesel Orbit</span>
          </button>
          <button onclick="setTalonMode('straight')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-right-long text-cyan-400"></i><span>Düz Hat Kaçış</span>
          </button>
          <button onclick="setTalonMode('dive')" class="bg-red-950/70 hover:bg-red-900 hover:border-red-400 p-2 rounded border border-red-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-trend-down text-red-400"></i><span>Acil Dalış (-30m)</span>
          </button>
          <button onclick="setTalonMode('climb')" class="bg-emerald-950/70 hover:bg-emerald-900 hover:border-emerald-400 p-2 rounded border border-emerald-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-trend-up text-emerald-400"></i><span>Tırmanış (+30m)</span>
          </button>
        </div>
      </div>

      <!-- Sliders -->
      <div class="card-glass rounded-lg p-3.5">
        <h3 class="text-sm font-bold text-slate-300 uppercase tracking-wide mb-3"><i class="fa-solid fa-sliders text-xs mr-2"></i>Dinamik Uçuş Parametreleri</h3>
        
        <div class="space-y-3 text-xs mono">
          <div>
            <div class="flex justify-between text-slate-300 mb-1">
              <span>Hedef Sürati:</span>
              <span id="lbl-spd-val" class="font-bold text-amber-400">22.0 m/s (79 km/h)</span>
            </div>
            <input id="slider-spd" type="range" min="14" max="36" step="1" value="22" oninput="onSpeedChange(this.value)" class="w-full accent-amber-400 bg-slate-800 h-1.5 rounded cursor-pointer">
          </div>

          <div>
            <div class="flex justify-between text-slate-300 mb-1">
              <span>Hedef İrtifası:</span>
              <span id="lbl-alt-val" class="font-bold text-cyan-400">100.0 m</span>
            </div>
            <input id="slider-alt" type="range" min="30" max="180" step="5" value="100" oninput="onAltChange(this.value)" class="w-full accent-cyan-400 bg-slate-800 h-1.5 rounded cursor-pointer">
          </div>
        </div>
      </div>

      <!-- Manual Steering Pad -->
      <div class="card-glass rounded-lg p-3.5">
        <h3 class="text-sm font-bold text-slate-300 uppercase tracking-wide mb-2.5"><i class="fa-solid fa-gamepad text-xs mr-2"></i>Manuel Dümen Masası (Bas-Tut)</h3>
        
        <div class="grid grid-cols-3 gap-1.5 text-xs font-bold text-center">
          <div></div>
          <button onmousedown="sendManualClimb(4)" onmouseup="sendManualClimb(0)" class="bg-slate-800 hover:bg-cyan-900 border border-slate-700 hover:border-cyan-400 p-2 rounded transition active:scale-95">
            <i class="fa-solid fa-arrow-up"></i><div class="text-[10px] mt-0.5">TIRMAN</div>
          </button>
          <div></div>

          <button onmousedown="sendManualTurn(0.4)" onmouseup="sendManualTurn(0)" class="bg-slate-800 hover:bg-cyan-900 border border-slate-700 hover:border-cyan-400 p-2 rounded transition active:scale-95">
            <i class="fa-solid fa-arrow-left"></i><div class="text-[10px] mt-0.5">SOLA DÖN</div>
          </button>
          <button onclick="setTalonMode('straight')" class="bg-slate-800 hover:bg-slate-700 border border-slate-600 p-2 rounded transition text-[11px] flex flex-col items-center justify-center">
            <i class="fa-solid fa-circle-dot text-amber-400 mb-0.5"></i>DÜZELT
          </button>
          <button onmousedown="sendManualTurn(-0.4)" onmouseup="sendManualTurn(0)" class="bg-slate-800 hover:bg-cyan-900 border border-slate-700 hover:border-cyan-400 p-2 rounded transition active:scale-95">
            <i class="fa-solid fa-arrow-right"></i><div class="text-[10px] mt-0.5">SAĞA DÖN</div>
          </button>

          <div></div>
          <button onmousedown="sendManualClimb(-4)" onmouseup="sendManualClimb(0)" class="bg-slate-800 hover:bg-cyan-900 border border-slate-700 hover:border-cyan-400 p-2 rounded transition active:scale-95">
            <i class="fa-solid fa-arrow-down"></i><div class="text-[10px] mt-0.5">DAL</div>
          </button>
          <div></div>
        </div>
      </div>

    </div>

    <!-- CENTER COLUMN: 2D TACTICAL RADAR (6 Cols) -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3.5 flex flex-col">
      <div class="flex items-center justify-between mb-2">
        <div class="flex items-center space-x-2">
          <span class="w-3 h-3 rounded-full bg-cyan-400 animate-pulse"></span>
          <h2 class="text-base font-bold text-cyan-400 tracking-wider uppercase">2D TAKTİK RADAR & KUŞBAKIŞI YÖRÜNGE HARİTASI</h2>
        </div>
        <div class="flex items-center space-x-2 text-xs mono">
          <span class="text-slate-400">Ölçek:</span>
          <button onclick="zoomRadar(1.2)" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700">+</button>
          <button onclick="zoomRadar(0.8)" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700">-</button>
          <button onclick="resetRadarZoom()" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700">SIFIRLA</button>
        </div>
      </div>

      <!-- Radar Canvas Container -->
      <div class="relative flex-1 w-full bg-[#080d14] rounded-lg border border-cyan-900/60 overflow-hidden min-h-[380px] flex items-center justify-center">
        <canvas id="radarCanvas" class="w-full h-full"></canvas>
        
        <!-- Legend Overlay -->
        <div class="absolute bottom-2 left-2 bg-slate-950/80 p-2 rounded border border-slate-800 text-[11px] mono space-y-1">
          <div class="flex items-center space-x-2"><span class="w-2.5 h-2.5 rounded bg-red-500 inline-block"></span><span class="text-slate-300">Talon 1718 (Hedef)</span></div>
          <div class="flex items-center space-x-2"><span class="w-2.5 h-2.5 rounded bg-cyan-400 inline-block"></span><span class="text-slate-300">Octopus 100 (Önleyici)</span></div>
          <div class="flex items-center space-x-2"><span class="w-2.5 h-2.5 border border-dashed border-emerald-400 inline-block"></span><span class="text-slate-300">5 Metre Ağ Yakalama Halkası</span></div>
          <div class="flex items-center space-x-2"><span class="w-3 h-0.5 bg-amber-400 inline-block"></span><span class="text-slate-300">Görüş Hattı (LOS)</span></div>
        </div>

        <div id="radar-fps-badge" class="absolute top-2 right-2 mono text-xs bg-slate-900/80 px-2 py-1 rounded border border-slate-700 text-cyan-400">
          60 FPS RADAR
        </div>
      </div>
    </div>

    <!-- RIGHT COLUMN: INTERCEPTOR COMMAND & TELEMETRY (3 Cols) -->
    <div class="col-span-12 lg:col-span-3 space-y-3">
      
      <!-- Interceptor Command Deck -->
      <div class="card-glass rounded-lg p-3.5 border-l-4 border-cyan-500">
        <h2 class="text-base font-bold text-cyan-400 tracking-wide uppercase mb-2.5"><i class="fa-solid fa-rocket text-sm mr-2"></i>Önleyici Komuta Masası</h2>
        
        <div class="grid grid-cols-2 gap-2 text-xs font-bold">
          <button onclick="sendInterceptorCmd('takeoff')" class="bg-blue-600 hover:bg-blue-500 text-white p-2.5 rounded transition shadow-lg shadow-blue-900/30 flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-plane-departure"></i><span>ARM & KALKIŞ</span>
          </button>
          <button onclick="sendInterceptorCmd('intercept')" class="bg-emerald-600 hover:bg-emerald-500 text-white p-2.5 rounded transition shadow-lg shadow-emerald-900/30 flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-bullseye text-amber-300 animate-spin"></i><span>ÖNLEMEYİ BAŞLAT</span>
          </button>
          <button onclick="sendInterceptorCmd('hold')" class="bg-amber-600 hover:bg-amber-500 text-white p-2 rounded transition flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-pause"></i><span>HAVADA TUT (HOLD)</span>
          </button>
          <button onclick="sendInterceptorCmd('land')" class="bg-red-700 hover:bg-red-600 text-white p-2 rounded transition flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-plane-arrival"></i><span>İNİŞ YAP (LAND)</span>
          </button>
        </div>
      </div>

      <!-- Separation & Fire Control Telemetry Card -->
      <div class="card-glass rounded-lg p-3.5 border border-cyan-500/30">
        <div class="flex items-center justify-between mb-2">
          <span class="text-xs uppercase tracking-wider text-slate-400">Hedefe Olan Mesafe</span>
          <span id="txt-fps-tag" class="mono text-[11px] bg-slate-800 text-amber-300 px-2 py-0.5 rounded border border-slate-700">50 FPS MODU</span>
        </div>
        
        <!-- Big Distance Value -->
        <div class="text-center py-2 bg-slate-950/60 rounded border border-slate-800 mb-3">
          <div id="txt-distance" class="text-4xl font-extrabold mono text-cyan-400 glow-cyan">--- m</div>
          <div id="txt-phase" class="text-xs font-bold uppercase tracking-widest text-slate-400 mt-1">Güdüm Safhası: STANDBY</div>
        </div>

        <!-- Telemetry Items -->
        <div class="space-y-2 mono text-xs">
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Kapanma Hızı (Vc):</span>
            <span id="txt-vc" class="font-bold text-slate-200">--- m/s</span>
          </div>
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Önleyici İtki (Pitch):</span>
            <span id="txt-pitch" class="font-bold text-slate-200">---°</span>
          </div>
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Önleyici Hızı:</span>
            <span id="txt-v-int" class="font-bold text-cyan-400">--- m/s</span>
          </div>
          <div class="flex justify-between">
            <span class="text-slate-400">Talon Hızı:</span>
            <span id="txt-v-tgt" class="font-bold text-red-400">--- m/s</span>
          </div>
        </div>

        <!-- Net Fire Control Lock Meter -->
        <div class="mt-3 pt-3 border-t border-slate-800">
          <div class="flex justify-between text-xs mono mb-1">
            <span class="text-slate-400 font-bold">AĞ FIRLATMA KİLİDİ:</span>
            <span id="txt-lock-pct" class="font-bold text-emerald-400">0%</span>
          </div>
          <div class="w-full bg-slate-900 h-3 rounded-full overflow-hidden border border-slate-700">
            <div id="bar-lock" class="bg-gradient-to-r from-amber-500 to-emerald-400 h-full w-0 transition-all duration-100"></div>
          </div>
          <div id="txt-net-status" class="text-center text-xs font-bold uppercase tracking-wider text-slate-500 mt-1.5">
            AĞ DURUMU: HAZIR
          </div>
        </div>

      </div>

    </div>

  </div>

  <!-- BOTTOM CHARTS: 2 REAL-TIME PERFORMANCE PLOTS -->
  <div class="grid grid-cols-12 gap-3">
    
    <!-- Distance Plot -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3">
      <div class="flex justify-between items-center mb-1">
        <h3 class="text-xs font-bold uppercase tracking-wider text-cyan-400"><i class="fa-solid fa-chart-line mr-1.5"></i>Ayrılma Mesafesi Zaman Eğrisi [m]</h3>
        <span class="text-[10px] mono text-slate-400">Zamanla Kapanma Performansı</span>
      </div>
      <div class="h-44">
        <canvas id="chartDistance"></canvas>
      </div>
    </div>

    <!-- Speed Comparison Plot -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3">
      <div class="flex justify-between items-center mb-1">
        <h3 class="text-xs font-bold uppercase tracking-wider text-amber-400"><i class="fa-solid fa-gauge-high mr-1.5"></i>Hız Kıyaslama Eğrisi [m/s]</h3>
        <span class="text-[10px] mono text-slate-400">Mavi: Octopus // Kırmızı: Talon</span>
      </div>
      <div class="h-44">
        <canvas id="chartSpeed"></canvas>
      </div>
    </div>

  </div>

  <!-- JAVASCRIPT ENGINE -->
  <script>
    // --- WebSocket Telemetry Link ---
    let ws = null;
    let latestData = null;
    let radarZoom = 1.0;

    function connectWebSocket() {
      const loc = window.location;
      const wsUri = (loc.protocol === "https:" ? "wss:" : "ws:") + "//" + loc.host + "/ws";
      ws = new WebSocket(wsUri);

      ws.onopen = () => {
        console.log("WebSocket connected.");
      };

      ws.onmessage = (evt) => {
        try {
          const data = JSON.parse(evt.data);
          latestData = data;
          updateDashboard(data);
        } catch (e) {
          console.error("Parse error:", e);
        }
      };

      ws.onclose = () => {
        setTimeout(connectWebSocket, 1000);
      };
    }

    // --- Dashboard Updates ---
    function updateDashboard(d) {
      const inter = d.interceptor;
      const talon = d.talon;
      const tac = d.tactical;

      // Status Badges
      const badgeTalon = document.getElementById("badge-talon");
      const txtTalonBadge = document.getElementById("txt-talon-badge");
      if (talon.connected) {
        badgeTalon.className = "bg-emerald-950/70 border border-emerald-600 text-emerald-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtTalonBadge.innerText = `TALON: AKTİF [${talon.mode.toUpperCase()}]`;
      } else {
        badgeTalon.className = "bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtTalonBadge.innerText = "TALON: BAĞLANTI YOK";
      }

      const badgeInt = document.getElementById("badge-interceptor");
      const txtIntBadge = document.getElementById("txt-int-badge");
      if (inter.connected) {
        badgeInt.className = "bg-cyan-950/70 border border-cyan-500 text-cyan-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtIntBadge.innerText = `ÖNLEYİCİ: ${inter.mode} [ARMED: ${inter.armed ? 'EVET' : 'HAYIR'}]`;
      } else {
        badgeInt.className = "bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtIntBadge.innerText = "ÖNLEYİCİ: BAĞLANTI YOK";
      }

      document.getElementById("talon-mode-badge").innerText = talon.mode.toUpperCase();

      // Sliders label
      document.getElementById("lbl-spd-val").innerText = `${talon.speed.toFixed(1)} m/s (${(talon.speed * 3.6).toFixed(0)} km/h)`;
      document.getElementById("lbl-alt-val").innerText = `${talon.alt.toFixed(1)} m`;

      // Always update vehicle-specific metrics independently:
      if (talon.connected) {
        document.getElementById("txt-v-tgt").innerText = `${talon.speed.toFixed(1)} m/s (${(talon.speed * 3.6).toFixed(0)} km/h)`;
      } else {
        document.getElementById("txt-v-tgt").innerText = "--- m/s";
      }

      if (inter.connected) {
        document.getElementById("txt-v-int").innerText = `${inter.speed.toFixed(1)} m/s (${(inter.speed * 3.6).toFixed(0)} km/h)`;
        document.getElementById("txt-pitch").innerText = `${inter.att[1].toFixed(1)}° (Roll: ${inter.att[0].toFixed(1)}°)`;
      } else {
        document.getElementById("txt-v-int").innerText = "--- m/s";
        document.getElementById("txt-pitch").innerText = "---°";
      }

      // Tactical Engagement Telemetry
      if (inter.connected && talon.connected) {
        const dist = tac.separation_distance;
        const distEl = document.getElementById("txt-distance");
        distEl.innerText = `${dist.toFixed(1)} m`;

        if (dist > 40.0) {
          distEl.className = "text-4xl font-extrabold mono text-cyan-400 glow-cyan";
        } else if (dist > 20.0) {
          distEl.className = "text-4xl font-extrabold mono text-amber-400 glow-amber";
        } else if (dist > 5.5) {
          distEl.className = "text-4xl font-extrabold mono text-orange-400 glow-amber";
        } else {
          distEl.className = "text-4xl font-extrabold mono text-emerald-400 glow-green animate-pulse";
        }

        document.getElementById("txt-phase").innerText = `Güdüm Safhası: ${tac.guidance_phase}`;
        document.getElementById("txt-fps-tag").innerText = `${tac.fps_mode} FPS MODU`;
        document.getElementById("txt-vc").innerText = `${tac.closing_speed > 0 ? '+' : ''}${tac.closing_speed.toFixed(1)} m/s (${(tac.closing_speed*3.6).toFixed(0)} km/h)`;

        // Net Fire Control Progress
        const lockPct = tac.lock_percent;
        document.getElementById("txt-lock-pct").innerText = `${lockPct.toFixed(0)}%`;
        document.getElementById("bar-lock").style.width = `${lockPct}%`;

        const netStatus = document.getElementById("txt-net-status");
        if (tac.net_deployed || lockPct >= 100) {
          netStatus.innerText = "💥 AĞ FIRLATILDI! HEDEF YAKALANDI!";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-emerald-400 glow-green animate-bounce mt-1.5";
        } else if (dist <= 5.5) {
          netStatus.innerText = "🎯 KİLİTLENİLİYOR...";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-amber-400 glow-amber mt-1.5";
        } else {
          netStatus.innerText = "AĞ DURUMU: HAZIR";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-slate-500 mt-1.5";
        }
      } else {
        const distEl = document.getElementById("txt-distance");
        distEl.innerText = talon.connected ? "ÖNLEYİCİ BEKLENİYOR" : "--- m";
        distEl.className = "text-2xl font-bold mono text-slate-400";
        document.getElementById("txt-phase").innerText = "Güdüm Safhası: BEKLENİYOR";
        document.getElementById("txt-vc").innerText = "--- m/s";
      }

      // Charts update
      if (d.charts && d.charts.times.length > 0) {
        updateCharts(d.charts);
      }
    }

    // --- Command Dispatchers ---
    function setTalonMode(mode) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: mode})
      });
    }

    function onSpeedChange(val) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({speed: parseFloat(val)})
      });
    }

    function onAltChange(val) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({alt: parseFloat(val)})
      });
    }

    function sendManualTurn(rate) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: "manual", turn_rate: rate})
      });
    }

    function sendManualClimb(rate) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: "manual", climb_rate: rate})
      });
    }

    function sendInterceptorCmd(action) {
      fetch("/api/interceptor/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({action: action})
      });
    }

    function launchHUD() {
      fetch("/api/tools/hud", {method: "POST"});
    }

    function launchQGC() {
      fetch("/api/tools/qgc", {method: "POST"});
    }

    // --- 2D Tactical Radar Canvas ---
    const canvas = document.getElementById("radarCanvas");
    const ctx = canvas.getContext("2d");

    function resizeCanvas() {
      canvas.width = canvas.parentElement.clientWidth;
      canvas.height = canvas.parentElement.clientHeight;
    }
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();

    function zoomRadar(factor) {
      radarZoom *= factor;
      radarZoom = Math.max(0.2, Math.min(5.0, radarZoom));
    }
    function resetRadarZoom() {
      radarZoom = 1.0;
    }

    function drawRadar() {
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);

      // Center
      const cx = w / 2;
      const cy = h / 2;
      const scale = (Math.min(w, h) / 1200.0) * radarZoom; // pixels per meter

      // Tactical Grid
      ctx.strokeStyle = "rgba(0, 229, 255, 0.1)";
      ctx.lineWidth = 1;

      // Concentric Distance Rings (every 200m)
      const ringSteps = [100, 250, 500, 750, 1000];
      for (const r of ringSteps) {
        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
        ctx.stroke();

        ctx.fillStyle = "rgba(0, 229, 255, 0.4)";
        ctx.font = "10px monospace";
        ctx.fillText(`${r}m`, cx + r * scale + 4, cy - 4);
      }

      // Compass Crosshairs
      ctx.beginPath();
      ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
      ctx.moveTo(0, cy); ctx.lineTo(w, cy);
      ctx.stroke();

      // Cardinal Labels
      ctx.fillStyle = "rgba(0, 229, 255, 0.8)";
      ctx.font = "bold 12px Rajdhani, sans-serif";
      ctx.fillText("N (KUZEY)", cx - 24, 20);
      ctx.fillText("S (GÜNEY)", cx - 24, h - 10);
      ctx.fillText("E (DOĞU)", w - 60, cy - 8);
      ctx.fillText("W (BATI)", 10, cy - 8);

      if (!latestData) {
        requestAnimationFrame(drawRadar);
        return;
      }

      const trails = latestData.trails || {};
      const inter = latestData.interceptor;
      const talon = latestData.talon;

      // Transform world (East, North) -> screen (x, y)
      // Screen X = cx + East * scale
      // Screen Y = cy - North * scale
      const toScreen = (east, north) => [cx + east * scale, cy - north * scale];

      // Draw Talon Trail (Red)
      if (trails.talon && trails.talon.length > 1) {
        ctx.strokeStyle = "rgba(255, 68, 68, 0.6)";
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        trails.talon.forEach((pt, i) => {
          const [sx, sy] = toScreen(pt[0], pt[1]);
          if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
        });
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // Draw Interceptor Trail (Cyan)
      if (trails.interceptor && trails.interceptor.length > 1) {
        ctx.strokeStyle = "rgba(0, 229, 255, 0.8)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        trails.interceptor.forEach((pt, i) => {
          const [sx, sy] = toScreen(pt[0], pt[1]);
          if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
        });
        ctx.stroke();
      }

      // Target Talon (Red Marker & 5m Capture Ring)
      if (talon.connected) {
        const [tx, ty] = toScreen(talon.pos[1], talon.pos[0]);

        // 5m Capture Circle
        ctx.strokeStyle = "rgba(0, 255, 136, 0.8)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.arc(tx, ty, Math.max(8, 5.0 * scale), 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);

        // Aircraft Icon (Triangle pointing along heading)
        const rad = (90 - talon.heading) * Math.PI / 180.0;
        ctx.save();
        ctx.translate(tx, ty);
        ctx.rotate(-rad + Math.PI / 2);
        ctx.fillStyle = "#ff3b30";
        ctx.beginPath();
        ctx.moveTo(0, -10);
        ctx.lineTo(8, 8);
        ctx.lineTo(0, 4);
        ctx.lineTo(-8, 8);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        // Label
        ctx.fillStyle = "#ff6b6b";
        ctx.font = "bold 11px monospace";
        ctx.fillText(`TALON [${talon.speed.toFixed(0)}m/s]`, tx + 12, ty - 6);
      }

      // Interceptor (Cyan Marker)
      if (inter.connected) {
        const [ix, iy] = toScreen(inter.pos[1], inter.pos[0]);

        ctx.fillStyle = "#00e5ff";
        ctx.beginPath();
        ctx.arc(ix, iy, 6, 0, 2 * Math.PI);
        ctx.fill();

        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.5;
        ctx.stroke();

        ctx.fillStyle = "#63b3ed";
        ctx.font = "bold 11px monospace";
        ctx.fillText(`OCTOPUS [${inter.speed.toFixed(0)}m/s]`, ix + 10, iy + 14);

        // Line-of-sight (LOS) from Interceptor to Talon
        if (talon.connected) {
          const [tx, ty] = toScreen(talon.pos[1], talon.pos[0]);
          ctx.strokeStyle = "rgba(255, 234, 0, 0.7)";
          ctx.lineWidth = 1.5;
          ctx.setLineDash([2, 4]);
          ctx.beginPath();
          ctx.moveTo(ix, iy);
          ctx.lineTo(tx, ty);
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }

      requestAnimationFrame(drawRadar);
    }
    requestAnimationFrame(drawRadar);

    // --- Chart.js Setup ---
    const ctxDist = document.getElementById("chartDistance").getContext("2d");
    const chartDist = new Chart(ctxDist, {
      type: "line",
      data: {
        labels: [],
        datasets: [{
          label: "Ayrılma Mesafesi (m)",
          data: [],
          borderColor: "#00e5ff",
          backgroundColor: "rgba(0, 229, 255, 0.1)",
          borderWidth: 2,
          pointRadius: 0,
          fill: true,
          tension: 0.2
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { display: false },
          y: { grid: { color: "rgba(255,255,255,0.05)" }, ticks: { color: "#94a3b8", font: { family: "monospace", size: 10 } } }
        },
        plugins: { legend: { display: false } }
      }
    });

    const ctxSpd = document.getElementById("chartSpeed").getContext("2d");
    const chartSpd = new Chart(ctxSpd, {
      type: "line",
      data: {
        labels: [],
        datasets: [
          {
            label: "Önleyici Hızı",
            data: [],
            borderColor: "#00e5ff",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          },
          {
            label: "Talon Hızı",
            data: [],
            borderColor: "#ff3b30",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { display: false },
          y: { grid: { color: "rgba(255,255,255,0.05)" }, ticks: { color: "#94a3b8", font: { family: "monospace", size: 10 } } }
        },
        plugins: {
          legend: { labels: { color: "#cbd5e1", font: { family: "monospace", size: 10 } } }
        }
      }
    });

    function updateCharts(c) {
      chartDist.data.labels = c.times;
      chartDist.data.datasets[0].data = c.dist;
      chartDist.update("none");

      chartSpd.data.labels = c.times;
      chartSpd.data.datasets[0].data = c.v_int;
      chartSpd.data.datasets[1].data = c.v_tgt;
      chartSpd.update("none");
    }

    // Start WebSocket
    connectWebSocket();
  </script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
async def get_index():
    return HTMLResponse(content=HTML_CONTENT)


# ----------------------------------------------------------------- Main ---

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080, help="Web GCS HTTP/WS port")
    args = parser.parse_args()

    # Start background threads
    threading.Thread(target=gz_transport_worker_thread, daemon=True).start()
    threading.Thread(target=mavlink_worker_thread, daemon=True).start()
    threading.Thread(target=target_poller_thread, daemon=True).start()
    threading.Thread(target=tactical_loop_thread, daemon=True).start()

    print(f"================================================================")
    print(f"  OCTOPUS TACTICAL WEB GCS STARTED!")
    print(f"  Tarayıcınızdan açın: http://localhost:{args.port}")
    print(f"================================================================", flush=True)

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()

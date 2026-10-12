#!/usr/bin/env python3
"""
Master 25-Run Automated Combat Flight Testbench & Telemetry Analytics Suite.

Executes 25 comprehensive flight scenarios testing the Octopus interceptor drone
against the Talon-1718 target aircraft flying at base speed of 28 m/s with
dynamic evasive speed variations (22 to 34 m/s), S-turn weaves, break turns,
vertical dives/climbs, throttle traps, and compound 3D maneuvers.

Features:
- Unattended batch execution of 25 distinct scenarios.
- 50 Hz multi-channel telemetry recording per run.
- Automated metric calculation: Miss distance, Net firing envelope, Pitch oscillations,
  Gimbal tracking error, Closing velocity stability.
- Generates live batch summary JSON and Markdown reports.
- Supports running individual runs (--run N), ranges (--range 1-5), or groups (--group weave).
"""

import os
import sys
import time
import math
import json
import csv
import glob
import signal
import urllib.request
import subprocess
import threading
import argparse
from datetime import datetime

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "../../.."))
BUILD_DIR = os.path.join(PX4_DIR, "build/px4_sitl_default")
TELEM_DIR = os.path.join(SIM_DIR, "build/telemetry")
SUMMARY_JSON = os.path.join(TELEM_DIR, "batch_25_summary.json")
SUMMARY_MD = os.path.join(TELEM_DIR, "batch_25_report.md")
LOG_FILE = os.path.join(SIM_DIR, "build/sim_test.log")
os.makedirs(TELEM_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 25-SCENARIO COMPREHENSIVE TEST MATRIX (BASE SPEED: 28.0 m/s)
# -----------------------------------------------------------------------------

SCENARIOS = [
    # Group A: Steady & Variable High-Speed Pursuit (RUN 01 - 04)
    {
        "id": 1, "group": "speed", "name": "Baseline 28 m/s Level Pursuit",
        "speed": 28.0, "alt": 60.0, "pattern": "square", "side": 3000.0,
        "desc": "Temel yüksek hızlı düz takip (28 m/s, irtifa 60m). Doğrusal yaklaşma ve sepet oturma.",
        "dyn_cmds": []
    },
    {
        "id": 2, "group": "speed", "name": "Fast Sprint 32 m/s Pursuit",
        "speed": 32.0, "alt": 80.0, "pattern": "square", "side": 3000.0,
        "desc": "Yüksek hızlı sprint takip (32 m/s, irtifa 80m). İleri itki marjı ve azami hız kapatma.",
        "dyn_cmds": []
    },
    {
        "id": 3, "group": "speed", "name": "Low-Speed 24 m/s Pursuit",
        "speed": 24.0, "alt": 50.0, "pattern": "square", "side": 3000.0,
        "desc": "Alt hız sınırı takibi (24 m/s, irtifa 50m). Yavaş uçuşta burun eğim stabilitesi.",
        "dyn_cmds": []
    },
    {
        "id": 4, "group": "speed", "name": "Periodic Throttle Pulse (24-32 m/s)",
        "speed": 28.0, "alt": 65.0, "pattern": "square", "side": 3000.0,
        "desc": "Düz hatta periyodik gaz dalgalanması (24 ile 32 m/s arası hız salınımı).",
        "dyn_cmds": [
            {"t": 15, "speed": 32.0}, {"t": 25, "speed": 24.0},
            {"t": 35, "speed": 30.0}, {"t": 45, "speed": 25.0}
        ]
    },

    # Group B: Serpentine Weaves & S-Turns (RUN 05 - 08)
    {
        "id": 5, "group": "weave", "name": "Mild Weave at 28 m/s",
        "speed": 28.0, "alt": 60.0, "pattern": "weave", "weave_amp": 20.0, "weave_wavelength": 450.0,
        "desc": "Hafif S-dönüşü kaçış dalgası (genlik 20m, 28 m/s). Görüş hattı açı hızı takibi.",
        "dyn_cmds": []
    },
    {
        "id": 6, "group": "weave", "name": "Standard Tactical Weave at 28 m/s",
        "speed": 28.0, "alt": 65.0, "pattern": "weave", "weave_amp": 35.0, "weave_wavelength": 380.0,
        "desc": "Standart taktik kaçınma dalgası (genlik 35m, 28 m/s, ~28 deg bank). Yanal APN ivmesi.",
        "dyn_cmds": []
    },
    {
        "id": 7, "group": "weave", "name": "Aggressive S-Turn Weave at 28 m/s",
        "speed": 28.0, "alt": 70.0, "pattern": "weave", "weave_amp": 50.0, "weave_wavelength": 300.0,
        "desc": "Agresif sert S-dönüşü (genlik 50m, 28 m/s, ~38 deg bank). Fishtailing sönümleme.",
        "dyn_cmds": []
    },
    {
        "id": 8, "group": "weave", "name": "Fast Weave Sprint at 32 m/s",
        "speed": 32.0, "alt": 80.0, "pattern": "weave", "weave_amp": 35.0, "weave_wavelength": 360.0,
        "desc": "Yüksek hızda kaçınma dalgası (genlik 35m, 32 m/s). Yüksek hızda viraj yakalama.",
        "dyn_cmds": []
    },

    # Group C: Coordinated & Break Turns (RUN 09 - 12)
    {
        "id": 9, "group": "turn", "name": "Coordinated Orbit Left at 28 m/s",
        "speed": 28.0, "alt": 70.0, "pattern": "square",
        "desc": "Sola koordineli daire kaçışı (28 m/s, 30 deg bank). Sürekli açısal hız takibi.",
        "dyn_cmds": [{"t": 12, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.35, "speed": 28.0}]
    },
    {
        "id": 10, "group": "turn", "name": "High-G Break Turn Right at 28 m/s",
        "speed": 28.0, "alt": 60.0, "pattern": "square",
        "desc": "Sağa ani sert kırılma dönüşü (28 m/s, 45 deg bank). Gimbal açısal hız doyumu testi.",
        "dyn_cmds": [{"t": 15, "mode": "circle", "circle_dir": -1.0, "turn_rate": 0.55, "speed": 28.0}]
    },
    {
        "id": 11, "group": "turn", "name": "Fast Break Turn Left at 32 m/s",
        "speed": 32.0, "alt": 80.0, "pattern": "square",
        "desc": "32 m/s yüksek hızda sert sola dönüş (45 deg bank). Merkezkaç ivme telafisi.",
        "dyn_cmds": [{"t": 15, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.50, "speed": 32.0}]
    },
    {
        "id": 12, "group": "turn", "name": "Break Turn with Speed Drop (28 -> 20 m/s)",
        "speed": 28.0, "alt": 65.0, "pattern": "square",
        "desc": "Viraj içinde ani hız kesme (28'den 20 m/s'ye fren). Viraj içi kapanma hızı şoku.",
        "dyn_cmds": [{"t": 15, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.45, "speed": 20.0}]
    },

    # Group D: Vertical Jinks & Pitch Dynamics (RUN 13 - 16)
    {
        "id": 13, "group": "vertical", "name": "Shallow Dive Evasion (75m -> 45m)",
        "speed": 28.0, "alt": 75.0, "pattern": "square",
        "desc": "Sığ dikey dalış (75m'den 45m'ye, Vz = -3 m/s, 28 m/s). Eğimli uçuşta irtifa kaybı.",
        "dyn_cmds": [{"t": 15, "mode": "dive", "alt": 45.0, "speed": 28.0}]
    },
    {
        "id": 14, "group": "vertical", "name": "Steep Dive Evasion (90m -> 35m)",
        "speed": 28.0, "alt": 90.0, "pattern": "square",
        "desc": "Dik dikey dalış (90m'den 35m'ye, Vz = -6 m/s, 28 m/s). Hızlı dalışta zemin emniyeti.",
        "dyn_cmds": [{"t": 15, "mode": "dive", "alt": 35.0, "speed": 29.0}]
    },
    {
        "id": 15, "group": "vertical", "name": "Zoom Climb Evasion (50m -> 110m)",
        "speed": 28.0, "alt": 50.0, "pattern": "square",
        "desc": "Dik tırmanış kaçışı (50m'den 110m'ye, Vz = +6 m/s, 28 m/s). Dikey eksen APN tepkisi.",
        "dyn_cmds": [{"t": 15, "mode": "climb", "alt": 110.0, "speed": 27.0}]
    },
    {
        "id": 16, "group": "vertical", "name": "Vertical Rollercoaster Jink (Dive then Climb)",
        "speed": 28.0, "alt": 70.0, "pattern": "square",
        "desc": "Önce dalış (40m'ye), ardından dik tırmanış (95m'ye). Dikey yön değiştirme stabilitesi.",
        "dyn_cmds": [
            {"t": 12, "mode": "dive", "alt": 40.0, "speed": 28.0},
            {"t": 28, "mode": "climb", "alt": 95.0, "speed": 28.0}
        ]
    },

    # Group E: Throttle Jinks / Speed Shock (RUN 17 - 20)
    {
        "id": 17, "group": "throttle", "name": "Hard Airbrake Trap (28 -> 16 m/s)",
        "speed": 28.0, "alt": 60.0, "pattern": "square",
        "desc": "Yaklaşma anında ani sert hava freni (28'den 16 m/s'ye düşüş). Pitch flip klamplama testi.",
        "dyn_cmds": [{"t": 18, "speed": 16.0}]
    },
    {
        "id": 18, "group": "throttle", "name": "Sprint Escape (24 -> 34 m/s)",
        "speed": 24.0, "alt": 65.0, "pattern": "square",
        "desc": "Hedefe 20m kala ani tam gaz kaçış (24'ten 34 m/s'ye sprint). Maksimum ivme yetişme testi.",
        "dyn_cmds": [{"t": 16, "speed": 34.0}]
    },
    {
        "id": 19, "group": "throttle", "name": "Cyclic Speed Jitter (+/- 5 m/s Steps)",
        "speed": 28.0, "alt": 60.0, "pattern": "square",
        "desc": "Adımlı hız dalgalanmaları (28 -> 33 -> 23 -> 30 m/s). Kapanma hızı V_c türevi takibi.",
        "dyn_cmds": [
            {"t": 12, "speed": 33.0}, {"t": 22, "speed": 23.0},
            {"t": 32, "speed": 30.0}, {"t": 42, "speed": 25.0}
        ]
    },
    {
        "id": 20, "group": "throttle", "name": "Deceleration & Re-acceleration Pulse",
        "speed": 28.0, "alt": 65.0, "pattern": "square",
        "desc": "Hedef yavaşlayıp sepete çekerken aniden gaz açma (28 -> 18 -> 32 m/s tuzağı).",
        "dyn_cmds": [{"t": 16, "speed": 18.0}, {"t": 28, "speed": 32.0}]
    },

    # Group F: Compound 3D Extreme Combat Evasion (RUN 21 - 25)
    {
        "id": 21, "group": "combat", "name": "Spiral Dive Combat Evasion",
        "speed": 28.0, "alt": 85.0, "pattern": "square",
        "desc": "Spiral dalış (35 deg bank ile dönerek -5 m/s dalış, 28 m/s). 3D bileşik açı hızı.",
        "dyn_cmds": [{"t": 15, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.40, "climb_rate": -5.0, "alt": 40.0, "speed": 28.0}]
    },
    {
        "id": 22, "group": "combat", "name": "Climbing S-Turn Combat Evasion",
        "speed": 28.0, "alt": 55.0, "pattern": "weave", "weave_amp": 35.0, "weave_wavelength": 350.0,
        "desc": "Tırmanışlı S-dönüşü (S-çizerken aynı anda +5 m/s tırmanış, 28 m/s). 3D APN takibi.",
        "dyn_cmds": [{"t": 14, "mode": "climb", "climb_rate": 5.0, "alt": 110.0, "speed": 28.0}]
    },
    {
        "id": 23, "group": "combat", "name": "Evasive Reversals with Speed Drop",
        "speed": 28.0, "alt": 65.0, "pattern": "square",
        "desc": "Hızlı yön değiştirmeler + hız düşürme (Sola 40 deg, sonra Sağa 40 deg, hız 22 m/s).",
        "dyn_cmds": [
            {"t": 14, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.50, "speed": 24.0},
            {"t": 26, "mode": "circle", "circle_dir": -1.0, "turn_rate": 0.50, "speed": 22.0}
        ]
    },
    {
        "id": 24, "group": "combat", "name": "3D Corkscrew Jink",
        "speed": 28.0, "alt": 55.0, "pattern": "square",
        "desc": "Tirbüşon manevrası (Spiral tırmanış + dalgalı gaz 25..32 m/s).",
        "dyn_cmds": [
            {"t": 14, "mode": "circle", "circle_dir": -1.0, "turn_rate": 0.42, "climb_rate": 4.5, "alt": 105.0, "speed": 32.0},
            {"t": 30, "speed": 25.0}
        ]
    },
    {
        "id": 25, "group": "combat", "name": "Full Combat Evasion Gauntlet",
        "speed": 28.0, "alt": 75.0, "pattern": "square",
        "desc": "Nihai kaçış senaryosu: Sert Break Turn -> Spiral Dalış -> Ani Tırmanış (28-34 m/s).",
        "dyn_cmds": [
            {"t": 12, "mode": "circle", "circle_dir": 1.0, "turn_rate": 0.55, "speed": 32.0},
            {"t": 24, "mode": "dive", "alt": 38.0, "speed": 26.0},
            {"t": 36, "mode": "climb", "alt": 100.0, "speed": 30.0}
        ]
    }
]


def run_px4_cmd(*args):
    px4cmd = os.path.join(SCRIPT_DIR, "px4cmd.sh")
    cmd = [px4cmd, "0"] + list(args)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return res.stdout.strip()
    except Exception as e:
        return f"Error: {e}"


def send_target_http_cmd(cmd_dict):
    try:
        data = json.dumps(cmd_dict).encode("utf-8")
        req = urllib.request.Request("http://127.0.0.1:8000/cmd", data=data,
                                     headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=1.0) as r:
            return r.status == 200
    except Exception:
        return False


def check_target_http(timeout=0.5):
    try:
        with urllib.request.urlopen("http://127.0.0.1:8000/target", timeout=timeout) as r:
            if r.status == 200:
                return json.loads(r.read().decode())
    except Exception:
        pass
    return None


class TelemetryRecorder:
    def __init__(self, out_file):
        self.out_file = out_file
        self.records = []
        self.running = False
        self.lock = threading.Lock()
        self.sim_time = 0.0

        # State tracking
        self.cmd_acc = np.zeros(3)
        self.cmd_yaw = 0.0
        self.cmd_gmb_yaw = 0.0
        self.cmd_gmb_pitch = 0.0
        self.cmd_barrel_pitch = 0.0
        self.phase_str = "STANDBY"
        self.target_source = "NONE"
        self.basket_rv = 999.0
        self.target_rt = 999.0
        self.closing_vc = 0.0
        self.net_fired = False
        self.firing_counter = 0

        # Gazebo subscribers
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.msgs10.clock_pb2 import Clock
            from gz.msgs10.double_pb2 import Double
            self.node = Node()
            self.node.subscribe(Clock, "/world/ankara/clock", self._on_clock)
            self.node.subscribe(Pose_V, "/world/ankara/dynamic_pose/info", self._on_poses)
            self.node.subscribe(Double, "/model/interceptor_0/command/seeker_yaw", lambda m: setattr(self, "cmd_gmb_yaw", m.data))
            self.node.subscribe(Double, "/model/interceptor_0/command/seeker_pitch", lambda m: setattr(self, "cmd_gmb_pitch", m.data))
            self.node.subscribe(Double, "/model/interceptor_0/command/net_launcher_pitch", lambda m: setattr(self, "cmd_barrel_pitch", m.data))
        except Exception:
            self.node = None

        self.poses = {}
        self.prev_poses = {}
        self.prev_times = {}

    def _on_clock(self, msg):
        self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_poses(self, msg):
        now = self.sim_time
        with self.lock:
            for p in msg.pose:
                name = p.name
                if name in ("interceptor_0", "talon1718_1") or "gimbal_pitch_link" in name:
                    pos = np.array([p.position.x, p.position.y, p.position.z])
                    ori = np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])
                    vel = np.zeros(3)
                    acc = np.zeros(3)
                    if name in self.prev_poses and now > self.prev_times.get(name, 0.0):
                        dt = max(0.001, now - self.prev_times[name])
                        prev_pos, _, prev_vel = self.prev_poses[name]
                        vel = (pos - prev_pos) / dt
                        acc = (vel - prev_vel) / dt
                    self.poses[name] = {"pos": pos, "ori": ori, "vel": vel, "acc": acc, "time": now}
                    self.prev_poses[name] = (pos, ori, vel)
                    self.prev_times[name] = now

    def update_from_intercept_status(self, status_text):
        with self.lock:
            for line in status_text.splitlines():
                if "Guidance Phase:" in line:
                    self.phase_str = line.split("Guidance Phase:")[-1].strip()
                if "Target Source:" in line:
                    parts = line.split(",")
                    self.target_source = parts[0].split(":")[-1].strip()
                if "Target Distance:" in line:
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            self.target_rt = float(parts[0].split(":")[-1].split("m")[0].strip())
                            self.basket_rv = float(parts[1].split(":")[-1].split("m")[0].strip())
                            self.closing_vc = float(parts[2].split(":")[-1].split("m")[0].strip())
                        except ValueError:
                            pass
                if "Net Gun Status:" in line:
                    self.net_fired = "FIRED" in line
                    if "Envelope Counter:" in line:
                        try:
                            self.firing_counter = int(line.split("Envelope Counter:")[-1].split("/")[0].strip())
                        except ValueError:
                            pass
                if "Command Accel:" in line:
                    try:
                        acc_part = line.split("Command Accel: [")[1].split("]")[0]
                        self.cmd_acc = np.array([float(x.strip()) for x in acc_part.split(",")])
                        self.cmd_yaw = float(line.split("Yaw Cmd:")[-1].split("deg")[0].strip())
                    except Exception:
                        pass

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join(timeout=1.5)

    def _run_loop(self):
        dt = 0.020  # 50 Hz
        while self.running:
            time.sleep(dt)
            with self.lock:
                p_int = self.poses.get("interceptor_0")
                p_tgt = self.poses.get("talon1718_1")
                if not p_int or not p_tgt:
                    continue

                # Euler angles from quaternions
                def q2rpy(q):
                    sr_cp = 2.0 * (q[0] * q[1] + q[2] * q[3])
                    cr_cp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
                    r = math.atan2(sr_cp, cr_cp)
                    sp = 2.0 * (q[0] * q[2] - q[3] * q[1])
                    p = math.asin(max(-1.0, min(1.0, sp)))
                    sy_cp = 2.0 * (q[0] * q[3] + q[1] * q[2])
                    cy_cp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
                    y = math.atan2(sy_cp, cy_cp)
                    return r, p, y

                r_int, p_int_pitch, y_int = q2rpy(p_int["ori"])
                r_tgt, p_tgt_pitch, y_tgt = q2rpy(p_tgt["ori"])
                tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (p_int["ori"][1]**2 + p_int["ori"][2]**2)))))

                row = {
                    "sim_time": round(self.sim_time, 3),
                    "phase": self.phase_str,
                    "target_source": self.target_source,
                    "int_pos_n": round(float(p_int["pos"][1]), 3),
                    "int_pos_e": round(float(p_int["pos"][0]), 3),
                    "int_pos_d": round(float(-p_int["pos"][2]), 3),
                    "int_vel_n": round(float(p_int["vel"][1]), 3),
                    "int_vel_e": round(float(p_int["vel"][0]), 3),
                    "int_vel_d": round(float(-p_int["vel"][2]), 3),
                    "int_pitch_deg": round(math.degrees(p_int_pitch), 1),
                    "int_tilt_deg": round(tilt_deg, 1),
                    "tgt_pos_n": round(float(p_tgt["pos"][1]), 3),
                    "tgt_pos_e": round(float(p_tgt["pos"][0]), 3),
                    "tgt_pos_d": round(float(-p_tgt["pos"][2]), 3),
                    "tgt_speed": round(float(np.linalg.norm(p_tgt["vel"])), 2),
                    "basket_dist_rv": round(self.basket_rv, 2),
                    "target_dist_rt": round(self.target_rt, 2),
                    "closing_speed_vc": round(self.closing_vc, 2),
                    "cmd_acc_norm": round(float(np.linalg.norm(self.cmd_acc)), 2),
                    "cmd_gmb_pitch_deg": round(math.degrees(self.cmd_gmb_pitch), 1),
                    "cmd_gmb_yaw_deg": round(math.degrees(self.cmd_gmb_yaw), 1),
                    "net_fired": 1 if self.net_fired else 0
                }
                self.records.append(row)

    def save(self):
        if not self.records:
            return None
        with open(self.out_file, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.records[0].keys()))
            w.writeheader()
            w.writerows(self.records)
        return self.out_file

    def analyze(self):
        if not self.records:
            return {}
        rec = self.records
        active = [r for r in rec if r["phase"] not in ("STANDBY", "")] or rec
        pitches = [r["int_pitch_deg"] for r in active]
        tilts = [r["int_tilt_deg"] for r in active]
        rvs = [r["basket_dist_rv"] for r in active if r["basket_dist_rv"] < 800]
        vcs = [r["closing_speed_vc"] for r in active]

        p_diff = np.diff(pitches)
        reversals = sum(1 for i in range(1, len(p_diff)) if p_diff[i] * p_diff[i-1] < -0.2)
        dur = max(1.0, active[-1]["sim_time"] - active[0]["sim_time"])
        rev_rate = reversals / dur

        min_rv = min(rvs) if rvs else 999.0
        max_pitch_diff = (max(pitches) - min(pitches)) if pitches else 0.0
        net_fired = any(r["net_fired"] == 1 for r in rec)

        # Performance score (0 - 100)
        score = 0
        if net_fired:
            score += 50
        if min_rv <= 2.2:
            score += 25
        elif min_rv <= 3.5:
            score += 15
        if rev_rate < 1.0 and max_pitch_diff < 15.0:
            score += 25
        elif rev_rate < 1.5:
            score += 15

        return {
            "duration_s": round(dur, 1),
            "samples": len(rec),
            "min_basket_rv_m": round(min_rv, 2),
            "mean_closing_vc": round(float(np.mean(vcs)) if vcs else 0.0, 1),
            "max_tilt_deg": round(max(tilts) if tilts else 0.0, 1),
            "pitch_osc_deg": round(max_pitch_diff, 1),
            "pitch_rev_hz": round(rev_rate, 2),
            "net_fired": net_fired,
            "score": score
        }


def execute_single_run(scen, max_duration=75.0):
    run_id = scen["id"]
    name = scen["name"]
    print(f"\n{'='*80}")
    print(f"▶ [RUN-{run_id:02d}/25] {name}")
    print(f"  Hız: {scen['speed']} m/s | İrtifa: {scen['alt']} m | Rota: {scen['pattern']}")
    print(f"  Açıklama: {scen['desc']}")
    print(f"{'='*80}", flush=True)

    # 1. Clean previous state
    subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)
    time.sleep(1.0)

    # 2. Build environment
    target_args = f"--alt {scen['alt']} --speed {scen['speed']} --pattern {scen['pattern']}"
    if "weave_amp" in scen:
        target_args += f" --weave-amp {scen['weave_amp']}"
    if "weave_wavelength" in scen:
        target_args += f" --weave-wavelength {scen['weave_wavelength']}"
    if "side" in scen:
        target_args += f" --side {scen['side']}"

    sim_env = dict(os.environ)
    sim_env.update({
        "HEADLESS": "1",
        "VIEW": "0",
        "PATTERN": scen["pattern"],
        "TARGET_ARGS": target_args,
        "PX4_GZ_STANDALONE": "1"
    })

    sim_script = os.path.join(SIM_DIR, "sim.sh")
    with open(LOG_FILE, "w") as lf:
        proc = subprocess.Popen(["bash", sim_script], env=sim_env, stdout=lf, stderr=subprocess.STDOUT)

    # 3. Setup Telemetry
    csv_filename = os.path.join(TELEM_DIR, f"run_{run_id:02d}_{scen['group']}.csv")
    rec = TelemetryRecorder(csv_filename)

    # 4. Wait for Target and PX4
    t0 = time.time()
    while time.time() - t0 < 30.0:
        td = check_target_http()
        if td and td.get("valid"):
            break
        time.sleep(1.0)

    t_px4 = time.time()
    while time.time() - t_px4 < 35.0:
        out = run_px4_cmd("commander", "check")
        if "Preflight check: OK" in out:
            break
        time.sleep(1.0)

    rec.start()

    # 5. Arm & Takeoff & Intercept
    run_px4_cmd("commander", "arm")
    time.sleep(0.5)
    run_px4_cmd("commander", "takeoff")
    time.sleep(4.0)
    run_px4_cmd("commander", "mode", "ext1")

    # 6. Pursuit & Dynamic Maneuver Execution
    t_start = time.time()
    dyn_cmds = list(scen.get("dyn_cmds", []))
    fired = False

    while time.time() - t_start < max_duration:
        t_el = time.time() - t_start
        st = run_px4_cmd("intercept", "status")
        rec.update_from_intercept_status(st)

        # Apply scheduled dynamic commands
        if dyn_cmds and t_el >= dyn_cmds[0]["t"]:
            cmd = dyn_cmds.pop(0)
            send_target_http_cmd(cmd)

        # Check net gun fired
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, "r") as f:
                if ">>> NET GUN FIRED!" in f.read():
                    fired = True

        if fired and (t_el >= 35.0 or t_el >= max_duration):
            break

        time.sleep(1.0)

    rec.stop()
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except Exception:
        proc.kill()
    subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)

    csv_path = rec.save()
    analysis = rec.analyze()
    analysis["run_id"] = run_id
    analysis["name"] = name
    analysis["group"] = scen["group"]
    analysis["csv_path"] = csv_path

    # Print run summary
    status_icon = "✅ CAPTURED" if analysis.get("net_fired") else "⚠️ CLOSE PASS"
    print(f"\n[RUN-{run_id:02d} SONUCU]: {status_icon}")
    print(f"  Min Sepet Mesafesi: {analysis.get('min_basket_rv_m')} m | Ağ Atıldı mı: {analysis.get('net_fired')}")
    print(f"  Pitch Salınımı: {analysis.get('pitch_osc_deg')}° ({analysis.get('pitch_rev_hz')} Hz) | Skor: {analysis.get('score')}/100")
    print(f"  CSV Kaydı: {csv_path}\n", flush=True)

    return analysis


def generate_batch_reports(results):
    # Save JSON summary
    with open(SUMMARY_JSON, "w") as f:
        json.dump(results, f, indent=2)

    # Save Markdown report
    captured_count = sum(1 for r in results if r.get("net_fired"))
    avg_score = float(np.mean([r.get("score", 0) for r in results])) if results else 0.0
    avg_rv = float(np.mean([r.get("min_basket_rv_m", 999) for r in results])) if results else 0.0

    lines = [
        "# 3D Blended APN & Sanal Sepet Güdüm 25-Koşu Otomatik Değerlendirme Raporu",
        "",
        f"**Tarih:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  ",
        f"**Toplam Koşu:** {len(results)} / 25  ",
        f"**Yakalama Oranı (Net Fired):** {captured_count} / {len(results)} (%{captured_count*100.0/max(1, len(results)):.1f})  ",
        f"**Ortalama 3D APN Güdüm Skoru:** {avg_score:.1f} / 100  ",
        f"**Ortalama En Yakın Sepet Mesafesi (R_v):** {avg_rv:.2f} m (Hedef: <= 2.2m)  ",
        "",
        "## Koşu Sonuç Tablosu",
        "",
        "| ID | Grup | Senaryo Adı | Sepet Hatası R_v [m] | Kapanma V_c [m/s] | Pitch Salınım [°] | Ağ Atıldı mı? | APN Skoru | Durum |",
        "| :---: | :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |"
    ]

    for r in results:
        status_sym = "✅ Başarılı (Captured)" if r.get("net_fired") else "⚠️ İyileştirme Gerek"
        lines.append(f"| RUN-{r['run_id']:02d} | `{r['group']}` | {r['name']} | "
                     f"{r.get('min_basket_rv_m', '-')} m | {r.get('mean_closing_vc', '-')} m/s | "
                     f"{r.get('pitch_osc_deg', '-')}° | {'EVET' if r.get('net_fired') else 'HAYIR'} | "
                     f"{r.get('score', 0)}/100 | {status_sym} |")

    lines.extend([
        "",
        "## 3D Blended APN Algoritması Grup Bazlı Analizi",
        "",
        "- **Speed Grubu (RUN 01-04):** Düz hat yüksek hızda APN yaklaşma rotası, V_c_des regülasyonu ve sepet oturma PID stabilitesi.",
        "- **Weave Grubu (RUN 05-08):** Kaçınma S-dönüşlerinde yanal APN ivmesi a_cmd_apn_lat ve R_core (2.5m) fishtailing sönümlemesi.",
        "- **Turn Grubu (RUN 09-12):** Yüksek-G sert virajlarda merkezkaç ivmesi a_t_perp telafisi ve Seeker Gimbal takip doyması.",
        "- **Vertical Grubu (RUN 13-16):** Dikey dalış/tırmanışta önleyici pitch açısı ile irtifa kontrolü ayrışması ve dikey APN tepkisi.",
        "- **Throttle Grubu (RUN 17-20):** Hedef ani fren yaptığında geriye takla klamplaması (a_long >= -4.5 m/s²) ve sepet anti-windup koruması.",
        "- **Combat Grubu (RUN 21-25):** 3D uzayda bileşik APN güdüm dayanıklılığı, blending filtre sürekliliği ve MITLA 2.0 nihai yakalama zarfı.",
        ""
    ])

    with open(SUMMARY_MD, "w") as f:
        f.write("\n".join(lines))

    print(f"Raporlar güncellendi:\n  JSON: {SUMMARY_JSON}\n  Markdown: {SUMMARY_MD}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Master 25-Run Combat Flight Test Runner")
    parser.add_argument("--all", action="store_true", help="25 koşunun tamamını sırayla koşturur")
    parser.add_argument("--run", type=int, help="Belirli bir koşuyu çalıştırır (1 - 25)")
    parser.add_argument("--range", type=str, help="Belirli bir koşu aralığını çalıştırır (örn: 1-5 veya 10-15)")
    parser.add_argument("--group", choices=["speed", "weave", "turn", "vertical", "throttle", "combat"],
                        help="Belirli bir manevra grubunu koşturur")
    parser.add_argument("--duration", type=float, default=70.0, help="Koşu başına azami süre [saniye] (varsayılan: 70)")
    args = parser.parse_args()

    # Determine targets to run
    targets = []
    if args.run:
        targets = [s for s in SCENARIOS if s["id"] == args.run]
    elif args.range:
        try:
            start_id, end_id = map(int, args.range.split("-"))
            targets = [s for s in SCENARIOS if start_id <= s["id"] <= end_id]
        except Exception:
            print("Geçersiz aralık formatı! Örnek: --range 1-5")
            return 1
    elif args.group:
        targets = [s for s in SCENARIOS if s["group"] == args.group]
    else:
        # Default: run all 25
        targets = SCENARIOS

    print("=" * 80)
    print(f"  OCTOPUS INTERCEPTOR - 25-KOŞULU OTONOM TEST SUITE'İ")
    print(f"  Çalıştırılacak Koşu Sayısı: {len(targets)}")
    print(f"  Talon Standart Uçuş Hızı: 28.0 m/s (Dinamik Aralık: 22 - 34 m/s)")
    print("=" * 80, flush=True)

    results = []
    # Load existing results if resuming
    if os.path.exists(SUMMARY_JSON):
        try:
            with open(SUMMARY_JSON, "r") as f:
                results = json.load(f)
        except Exception:
            results = []

    for scen in targets:
        # Remove previous result for this ID if exists
        results = [r for r in results if r.get("run_id") != scen["id"]]
        res = execute_single_run(scen, max_duration=args.duration)
        results.append(res)
        generate_batch_reports(results)
        time.sleep(2.0)

    print("\n" + "=" * 80)
    print("🏁 TÜM SEÇİLİ TEST KOŞULARI TAMAMLANDI!")
    print(f"Sonuç Raporu: {SUMMARY_MD}")
    print("=" * 80)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Octopus Interceptor & Talon Target Tactical Ground Control & Evaluation Station.

Provides:
1. Real-time tactical 2D radar and trajectory tracking (pyqtgraph)
2. Interactive scenario injection & flight mode steering for target Talon 1718 (Square, Weave, Orbit, Dive, Climb, Manual)
3. One-click mission control for Interceptor (Arm & Takeoff, Engage Mode EXT1, Hold, Land)
4. Telemetry visualization: Separation distance, closing velocity, relative altitude, net lock timer
5. Performance graphs (Separation distance vs time, speeds vs time)

Usage:
  python3 tools/gcs_gui.py
"""

import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["MAVLINK20"] = "1"

import json
import math
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
from pymavlink import mavutil

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))
BUILD_DIR = os.path.join(PX4_DIR, "build", "px4_sitl_default")


# ------------------------------------------------------------- MAVLink Worker ---

class MavlinkWorker(QtCore.QThread):
    telemetry_received = QtCore.pyqtSignal(dict)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self, port=14540):
        super().__init__()
        self.port = port
        self.running = True

    def run(self):
        link = None
        while self.running:
            if link is None:
                try:
                    link = mavutil.mavlink_connection(f"udpin:0.0.0.0:{self.port}")
                    self.connected.emit(True)
                except Exception:
                    time.sleep(1.0)
                    continue

            try:
                msg = link.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue

                mtype = msg.get_type()
                if mtype in ("LOCAL_POSITION_NED", "ATTITUDE", "HEARTBEAT"):
                    self.telemetry_received.emit({
                        "type": mtype,
                        "msg": msg
                    })
            except Exception:
                link = None
                self.connected.emit(False)
                time.sleep(0.5)

    def stop(self):
        self.running = False


# ---------------------------------------------------- Gazebo Transport Worker ---

class GzTransportWorker(QtCore.QThread):
    pose_received = QtCore.pyqtSignal(dict)

    def __init__(self, world="ankara", interceptor_name="interceptor_0"):
        super().__init__()
        self.world = world
        self.interceptor_name = interceptor_name
        self.running = True

    def run(self):
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_v_pb2 import Pose_V
        except Exception:
            return

        node = Node()
        prev_pos = None
        prev_t = 0.0

        def on_pose(msg):
            nonlocal prev_pos, prev_t
            now = time.time()
            for p in msg.pose:
                if p.name == self.interceptor_name:
                    pos = p.position
                    ori = p.orientation
                    qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
                    sinr_cosp = 2 * (qw * qx + qy * qz)
                    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                    sinp = 2 * (qw * qy - qz * qx)
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

                    siny_cosp = 2 * (qw * qz + qx * qy)
                    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                    vx, vy, vz = 0.0, 0.0, 0.0
                    if prev_pos is not None and now > prev_t:
                        dt = max(0.001, now - prev_t)
                        vx = (pos.x - prev_pos[0]) / dt
                        vy = (pos.y - prev_pos[1]) / dt
                        vz = (pos.z - prev_pos[2]) / dt
                    prev_pos = (pos.x, pos.y, pos.z)
                    prev_t = now

                    self.pose_received.emit({
                        "north": pos.y,
                        "east": pos.x,
                        "down": -pos.z,
                        "vx": vy,
                        "vy": vx,
                        "vz": -vz,
                        "roll": roll,
                        "pitch": pitch,
                        "yaw": yaw
                    })

        topic = f"/world/{self.world}/dynamic_pose/info"
        node.subscribe(Pose_V, topic, on_pose)
        while self.running:
            time.sleep(0.5)

    def stop(self):
        self.running = False


# -------------------------------------------------------- Target HTTP Worker ---

class TargetHttpWorker(QtCore.QThread):
    target_telemetry_received = QtCore.pyqtSignal(dict)
    connected = QtCore.pyqtSignal(bool)

    def __init__(self, port=8000, poll_rate=20.0):
        super().__init__()
        self.url = f"http://127.0.0.1:{port}/target"
        self.dt = 1.0 / poll_rate
        self.running = True

    def run(self):
        while self.running:
            try:
                req = urllib.request.Request(self.url, headers={"User-Agent": "GCS-GUI"})
                with urllib.request.urlopen(req, timeout=0.5) as resp:
                    if resp.status == 200:
                        data = json.loads(resp.read().decode("utf-8"))
                        self.target_telemetry_received.emit(data)
                        self.connected.emit(True)
            except Exception:
                self.connected.emit(False)
            time.sleep(self.dt)

    def stop(self):
        self.running = False


# ------------------------------------------------------------- Main Window ---

class InterceptorGCS(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Octopus Interceptor & Talon Tactical GCS")
        self.resize(1500, 920)

        # Tactical Dark Theme Palette
        self.setStyleSheet("""
            QMainWindow {
                background-color: #12151a;
            }
            QWidget {
                background-color: #12151a;
                color: #e2e8f0;
                font-family: 'Segoe UI', 'Ubuntu', sans-serif;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #2d3748;
                border-radius: 6px;
                margin-top: 12px;
                padding-top: 10px;
                font-weight: bold;
                color: #63b3ed;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #2b3544;
                border: 1px solid #4a5568;
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: bold;
                color: #edf2f7;
            }
            QPushButton:hover {
                background-color: #3b4759;
                border-color: #63b3ed;
            }
            QPushButton:pressed {
                background-color: #1a202c;
            }
            QPushButton#takeoffBtn {
                background-color: #2b6cb0;
                border-color: #3182ce;
            }
            QPushButton#takeoffBtn:hover {
                background-color: #3182ce;
            }
            QPushButton#interceptBtn {
                background-color: #276749;
                border-color: #38a169;
                color: #e6fffa;
            }
            QPushButton#interceptBtn:hover {
                background-color: #2f855a;
            }
            QPushButton#holdBtn {
                background-color: #b7791f;
                border-color: #d69e2e;
            }
            QPushButton#holdBtn:hover {
                background-color: #d69e2e;
            }
            QPushButton#landBtn {
                background-color: #9b2c2c;
                border-color: #e53e3e;
            }
            QPushButton#landBtn:hover {
                background-color: #c53030;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #2d3748;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #3182ce;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #63b3ed;
                border: 1px solid #2b6cb0;
                width: 14px;
                margin-top: -4px;
                margin-bottom: -4px;
                border-radius: 7px;
            }
            QLabel {
                color: #cbd5e0;
            }
            QProgressBar {
                border: 1px solid #2d3748;
                border-radius: 4px;
                text-align: center;
                color: #ffffff;
                font-weight: bold;
                background-color: #1a202c;
            }
            QProgressBar::chunk {
                background-color: #38a169;
                border-radius: 3px;
            }
        """)

        # Data Histories for Plotting
        self.max_history = 300
        self.history_time = deque(maxlen=self.max_history)
        self.history_dist = deque(maxlen=self.max_history)
        self.history_v_int = deque(maxlen=self.max_history)
        self.history_v_tgt = deque(maxlen=self.max_history)

        self.int_trail_x = deque(maxlen=500)
        self.int_trail_y = deque(maxlen=500)
        self.tgt_trail_x = deque(maxlen=500)
        self.tgt_trail_y = deque(maxlen=500)

        # State Variables
        self.int_pos = np.zeros(3)
        self.int_vel = np.zeros(3)
        self.int_att = np.zeros(3)
        self.int_armed = False
        self.int_mode_str = "STANDBY"
        self.last_int_time = 0.0

        self.tgt_pos = np.zeros(3)
        self.tgt_vel = np.zeros(3)
        self.tgt_speed = 0.0
        self.tgt_alt = 0.0
        self.tgt_heading = 0.0
        self.tgt_mode = "square"
        self.last_tgt_time = 0.0

        self.lock_start_time = None
        self.net_deployed = False

        self.setup_ui()

        self.gz_worker = GzTransportWorker(world="ankara", interceptor_name="interceptor_0")
        self.gz_worker.pose_received.connect(self.on_gz_interceptor_pose)
        self.gz_worker.start()

        self.mavlink_worker = MavlinkWorker(port=14545)
        self.mavlink_worker.telemetry_received.connect(self.on_mavlink_msg)
        self.mavlink_worker.connected.connect(self.on_mavlink_status)
        self.mavlink_worker.start()

        self.http_worker = TargetHttpWorker(port=8000)
        self.http_worker.target_telemetry_received.connect(self.on_target_telemetry)
        self.http_worker.connected.connect(self.on_target_http_status)
        self.http_worker.start()

        # UI Update Timer (30 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(33)

        self.t0 = time.time()

    def setup_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        # 1. Top Header Bar
        header_layout = QtWidgets.QHBoxLayout()
        title_lbl = QtWidgets.QLabel("OCTOPUS INTERCEPTOR & TALON TACTICAL TEST BENCH")
        title_lbl.setStyleSheet("font-size: 16px; font-weight: bold; color: #63b3ed; letter-spacing: 1px;")
        header_layout.addWidget(title_lbl)
        header_layout.addStretch()

        self.btn_hud = QtWidgets.QPushButton("🎥 Kamera HUD")
        self.btn_hud.setToolTip("Ön kameranın monochrome HUD vizörünü açar (view.sh)")
        self.btn_hud.clicked.connect(self.launch_hud)
        header_layout.addWidget(self.btn_hud)

        self.btn_qgc = QtWidgets.QPushButton("🛰️ QGroundControl")
        self.btn_qgc.setToolTip("QGroundControl Yer Kontrol İstasyonunu başlatır")
        self.btn_qgc.clicked.connect(self.launch_qgc)
        header_layout.addWidget(self.btn_qgc)

        self.status_talon = QtWidgets.QLabel("TALON: CONNECTING...")
        self.status_talon.setStyleSheet("background-color: #2d3748; padding: 4px 8px; border-radius: 4px; font-weight: bold;")
        header_layout.addWidget(self.status_talon)

        self.status_int = QtWidgets.QLabel("INTERCEPTOR: CONNECTING...")
        self.status_int.setStyleSheet("background-color: #2d3748; padding: 4px 8px; border-radius: 4px; font-weight: bold;")
        header_layout.addWidget(self.status_int)

        main_layout.addLayout(header_layout)

        # 2. Main Middle Workspace (Split into Left Controls, Center Radar, Right Telemetry)
        middle_layout = QtWidgets.QHBoxLayout()

        # --- LEFT PANEL: Talon Target Tactical Control ---
        left_panel = QtWidgets.QGroupBox("HEDEF İHA (TALON 1718) KONTROLÜ")
        left_panel.setMinimumWidth(350)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setSpacing(10)

        # Scenario Buttons
        scen_group = QtWidgets.QGroupBox("Uçuş Senaryosu / Deseni")
        scen_layout = QtWidgets.QGridLayout(scen_group)

        self.btn_scen_square = QtWidgets.QPushButton("🔲 Standart Kare (2x2 km)")
        self.btn_scen_square.clicked.connect(lambda: self.send_talon_mode("square"))
        scen_layout.addWidget(self.btn_scen_square, 0, 0)

        self.btn_scen_weave = QtWidgets.QPushButton("〰️ S-Dönüş / Zig-Zag")
        self.btn_scen_weave.clicked.connect(lambda: self.send_talon_mode("weave"))
        scen_layout.addWidget(self.btn_scen_weave, 0, 1)

        self.btn_scen_orbit = QtWidgets.QPushButton("🔄 Dairesel Dönüş (Orbit)")
        self.btn_scen_orbit.clicked.connect(lambda: self.send_talon_mode("circle"))
        scen_layout.addWidget(self.btn_scen_orbit, 1, 0)

        self.btn_scen_straight = QtWidgets.QPushButton("➡️ Düz Hat Kaçış")
        self.btn_scen_straight.clicked.connect(lambda: self.send_talon_mode("straight"))
        scen_layout.addWidget(self.btn_scen_straight, 1, 1)

        self.btn_scen_dive = QtWidgets.QPushButton("📉 Acil Dalış (Dive)")
        self.btn_scen_dive.setStyleSheet("background-color: #742a2a;")
        self.btn_scen_dive.clicked.connect(lambda: self.send_talon_mode("dive"))
        scen_layout.addWidget(self.btn_scen_dive, 2, 0)

        self.btn_scen_climb = QtWidgets.QPushButton("📈 Hızlı Tırmanış (Climb)")
        self.btn_scen_climb.setStyleSheet("background-color: #285e61;")
        self.btn_scen_climb.clicked.connect(lambda: self.send_talon_mode("climb"))
        scen_layout.addWidget(self.btn_scen_climb, 2, 1)

        left_layout.addWidget(scen_group)

        # Dynamic Sliders
        param_group = QtWidgets.QGroupBox("Dinamik Parametreler")
        param_layout = QtWidgets.QVBoxLayout(param_group)

        # Speed Slider
        spd_header = QtWidgets.QHBoxLayout()
        spd_header.addWidget(QtWidgets.QLabel("Hedef Sürati:"))
        self.lbl_spd_val = QtWidgets.QLabel("22 m/s")
        self.lbl_spd_val.setStyleSheet("font-weight: bold; color: #48bb78;")
        spd_header.addWidget(self.lbl_spd_val)
        param_layout.addLayout(spd_header)

        self.slider_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_speed.setRange(15, 38)
        self.slider_speed.setValue(22)
        self.slider_speed.valueChanged.connect(self.on_speed_slider)
        param_layout.addWidget(self.slider_speed)

        # Altitude Slider
        alt_header = QtWidgets.QHBoxLayout()
        alt_header.addWidget(QtWidgets.QLabel("Hedef İrtifası:"))
        self.lbl_alt_val = QtWidgets.QLabel("100 m")
        self.lbl_alt_val.setStyleSheet("font-weight: bold; color: #4299e1;")
        alt_header.addWidget(self.lbl_alt_val)
        param_layout.addLayout(alt_header)

        self.slider_alt = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_alt.setRange(30, 220)
        self.slider_alt.setValue(100)
        self.slider_alt.valueChanged.connect(self.on_alt_slider)
        param_layout.addWidget(self.slider_alt)

        left_layout.addWidget(param_group)

        # Manual Steering D-Pad
        manual_group = QtWidgets.QGroupBox("Manuel Kumanda (Yön Verme)")
        manual_layout = QtWidgets.QGridLayout(manual_group)

        btn_left = QtWidgets.QPushButton("⬅️ Sol Dönüş")
        btn_left.pressed.connect(lambda: self.send_manual_turn(0.4))
        btn_left.released.connect(lambda: self.send_manual_turn(0.0))
        manual_layout.addWidget(btn_left, 1, 0)

        btn_straight = QtWidgets.QPushButton("⏸️ Düzelt")
        btn_straight.clicked.connect(lambda: self.send_manual_turn(0.0))
        manual_layout.addWidget(btn_straight, 1, 1)

        btn_right = QtWidgets.QPushButton("➡️ Sağ Dönüş")
        btn_right.pressed.connect(lambda: self.send_manual_turn(-0.4))
        btn_right.released.connect(lambda: self.send_manual_turn(0.0))
        manual_layout.addWidget(btn_right, 1, 2)

        btn_climb_man = QtWidgets.QPushButton("⬆️ Tırman (+)")
        btn_climb_man.pressed.connect(lambda: self.send_manual_climb(3.5))
        btn_climb_man.released.connect(lambda: self.send_manual_climb(0.0))
        manual_layout.addWidget(btn_climb_man, 0, 1)

        btn_dive_man = QtWidgets.QPushButton("⬇️ Dal (-)")
        btn_dive_man.pressed.connect(lambda: self.send_manual_climb(-3.5))
        btn_dive_man.released.connect(lambda: self.send_manual_climb(0.0))
        manual_layout.addWidget(btn_dive_man, 2, 1)

        left_layout.addWidget(manual_group)
        left_layout.addStretch()
        middle_layout.addWidget(left_panel)

        # --- CENTER PANEL: 2D Tactical Radar Map (pyqtgraph) ---
        center_panel = QtWidgets.QGroupBox("2D TAKTİK RADAR VE KUŞBAKIŞI TAKİP EKRANI")
        center_layout = QtWidgets.QVBoxLayout(center_panel)

        pg.setConfigOptions(antialias=True)
        self.radar_plot = pg.PlotWidget()
        self.radar_plot.setBackground("#161b22")
        self.radar_plot.showGrid(x=True, y=True, alpha=0.3)
        self.radar_plot.setLabel("left", "Kuzey (North) [m]")
        self.radar_plot.setLabel("bottom", "Doğu (East) [m]")
        self.radar_plot.setAspectLocked(True)

        # Plot items
        self.tgt_trail_curve = self.radar_plot.plot(pen=pg.mkPen(color="#ff4444", width=2, style=QtCore.Qt.DashLine))
        self.int_trail_curve = self.radar_plot.plot(pen=pg.mkPen(color="#00e5ff", width=2))
        self.los_curve = self.radar_plot.plot(pen=pg.mkPen(color="#ffea00", width=1.5, style=QtCore.Qt.DotLine))

        self.tgt_marker = self.radar_plot.plot(pen=None, symbol="t", symbolSize=16, symbolBrush="#ff3333")
        self.int_marker = self.radar_plot.plot(pen=None, symbol="d", symbolSize=14, symbolBrush="#00e5ff")

        # Capture zone circle around Talon (radius 5m)
        self.capture_circle = self.radar_plot.plot(pen=pg.mkPen(color="#00ff88", width=1, style=QtCore.Qt.DashLine))

        center_layout.addWidget(self.radar_plot)
        middle_layout.addWidget(center_panel, stretch=2)

        # --- RIGHT PANEL: Interceptor Controls & Telemetry ---
        right_panel = QtWidgets.QGroupBox("ÖNLEYİCİ GÖREV YÖNETİMİ & KİLİTLENME")
        right_panel.setMinimumWidth(350)
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setSpacing(10)

        # Interceptor Commands
        cmd_group = QtWidgets.QGroupBox("Önleyici Komuta Masası")
        cmd_layout = QtWidgets.QGridLayout(cmd_group)

        self.btn_takeoff = QtWidgets.QPushButton("🚀 ARM / KALKIŞ")
        self.btn_takeoff.setObjectName("takeoffBtn")
        self.btn_takeoff.clicked.connect(self.cmd_takeoff)
        cmd_layout.addWidget(self.btn_takeoff, 0, 0)

        self.btn_intercept = QtWidgets.QPushButton("🎯 ÖNLEME (EXT1)")
        self.btn_intercept.setObjectName("interceptBtn")
        self.btn_intercept.clicked.connect(self.cmd_intercept)
        cmd_layout.addWidget(self.btn_intercept, 0, 1)

        self.btn_hold = QtWidgets.QPushButton("⏸️ ASKIYA AL (HOLD)")
        self.btn_hold.setObjectName("holdBtn")
        self.btn_hold.clicked.connect(self.cmd_hold)
        cmd_layout.addWidget(self.btn_hold, 1, 0)

        self.btn_land = QtWidgets.QPushButton("🛬 İNİŞ (LAND)")
        self.btn_land.setObjectName("landBtn")
        self.btn_land.clicked.connect(self.cmd_land)
        cmd_layout.addWidget(self.btn_land, 1, 1)

        right_layout.addWidget(cmd_group)

        # Tactical Engagement Metrics
        metric_group = QtWidgets.QGroupBox("Önleme Telemetrisi")
        metric_layout = QtWidgets.QVBoxLayout(metric_group)

        # Separation Distance Display
        metric_layout.addWidget(QtWidgets.QLabel("Hedefe Olan Mesafe (Separation):"))
        self.lbl_dist_big = QtWidgets.QLabel("--- m")
        self.lbl_dist_big.setStyleSheet("""
            font-size: 28px;
            font-weight: bold;
            color: #63b3ed;
            background-color: #1a202c;
            border-radius: 6px;
            padding: 4px 8px;
            text-align: center;
        """)
        self.lbl_dist_big.setAlignment(QtCore.Qt.AlignCenter)
        metric_layout.addWidget(self.lbl_dist_big)

        # Closing Speed
        self.lbl_vc = QtWidgets.QLabel("Kapanma Hızı (Closing Spd): --- m/s")
        self.lbl_vc.setStyleSheet("font-size: 14px; font-weight: bold; color: #cbd5e0;")
        metric_layout.addWidget(self.lbl_vc)

        # Guidance Phase Indicator
        self.lbl_phase = QtWidgets.QLabel("Güdüm Safhası: STANDBY")
        self.lbl_phase.setStyleSheet("font-size: 13px; font-weight: bold; color: #a0aec0;")
        metric_layout.addWidget(self.lbl_phase)

        # Pitch / Tilt
        self.lbl_tilt = QtWidgets.QLabel("Önleyici Yatış (Tilt/Pitch): ---°")
        metric_layout.addWidget(self.lbl_tilt)

        # Net Lock Progress
        metric_layout.addWidget(QtWidgets.QLabel("Ağ Fırlatma Kilit Sayacı (400 ms):"))
        self.lock_progress = QtWidgets.QProgressBar()
        self.lock_progress.setRange(0, 100)
        self.lock_progress.setValue(0)
        metric_layout.addWidget(self.lock_progress)

        self.lbl_net_status = QtWidgets.QLabel("AĞ DURUMU: HAZIR")
        self.lbl_net_status.setStyleSheet("font-weight: bold; color: #a0aec0;")
        self.lbl_net_status.setAlignment(QtCore.Qt.AlignCenter)
        metric_layout.addWidget(self.lbl_net_status)

        right_layout.addWidget(metric_group)
        right_layout.addStretch()
        middle_layout.addWidget(right_panel)

        main_layout.addLayout(middle_layout, stretch=3)

        # 3. Bottom Strip: Time-Series Graphs
        bottom_layout = QtWidgets.QHBoxLayout()

        # Plot 1: Distance vs Time
        self.plot_dist = pg.PlotWidget(title="Mesafe vs Zaman [m]")
        self.plot_dist.setBackground("#161b22")
        self.plot_dist.showGrid(x=True, y=True, alpha=0.3)
        self.plot_dist.setLabel("left", "Mesafe [m]")
        self.plot_dist.setLabel("bottom", "Zaman [s]")
        self.curve_dist = self.plot_dist.plot(pen=pg.mkPen(color="#00e5ff", width=2))
        bottom_layout.addWidget(self.plot_dist)

        # Plot 2: Speeds vs Time
        self.plot_speed = pg.PlotWidget(title="Hızlar vs Zaman [m/s]")
        self.plot_speed.setBackground("#161b22")
        self.plot_speed.showGrid(x=True, y=True, alpha=0.3)
        self.plot_speed.setLabel("left", "Hız [m/s]")
        self.plot_speed.setLabel("bottom", "Zaman [s]")
        self.plot_speed.addLegend()
        self.curve_v_int = self.plot_speed.plot(pen=pg.mkPen(color="#00e5ff", width=2), name="Önleyici")
        self.curve_v_tgt = self.plot_speed.plot(pen=pg.mkPen(color="#ff4444", width=2, style=QtCore.Qt.DashLine), name="Talon")
        bottom_layout.addWidget(self.plot_speed)

        main_layout.addLayout(bottom_layout, stretch=1)

    # -------------------------------------------------------- Data Callbacks ---

    def on_mavlink_msg(self, data):
        mtype = data["type"]
        msg = data["msg"]

        if mtype == "LOCAL_POSITION_NED":
            self.int_pos[0] = msg.x
            self.int_pos[1] = msg.y
            self.int_pos[2] = msg.z
            self.int_vel[0] = msg.vx
            self.int_vel[1] = msg.vy
            self.int_vel[2] = msg.vz
            self.int_trail_x.append(msg.y) # East
            self.int_trail_y.append(msg.x) # North
            self.last_int_time = time.time()

        elif mtype == "ATTITUDE":
            self.int_att[0] = math.degrees(msg.roll)
            self.int_att[1] = math.degrees(msg.pitch)
            self.int_att[2] = math.degrees(msg.yaw)

        elif mtype == "HEARTBEAT":
            self.int_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def on_mavlink_status(self, connected):
        if connected:
            self.status_int.setText(f"INTERCEPTOR: MAVLINK [{self.int_mode_str}]")
            self.status_int.setStyleSheet("background-color: #276749; color: #e6fffa; padding: 4px 8px; border-radius: 4px; font-weight: bold;")
        elif self.last_int_time <= 0:
            self.status_int.setText("INTERCEPTOR: KOPUK")
            self.status_int.setStyleSheet("background-color: #742a2a; color: #fff5f5; padding: 4px 8px; border-radius: 4px; font-weight: bold;")

    def on_gz_interceptor_pose(self, d):
        self.int_pos[0] = d["north"]
        self.int_pos[1] = d["east"]
        self.int_pos[2] = d["down"]
        self.int_vel[0] = d["vx"]
        self.int_vel[1] = d["vy"]
        self.int_vel[2] = d["vz"]
        self.int_att[0] = d["roll"]
        self.int_att[1] = d["pitch"]
        self.int_att[2] = d["yaw"]
        self.int_trail_x.append(d["east"])
        self.int_trail_y.append(d["north"])
        self.last_int_time = time.time()
        self.status_int.setText(f"INTERCEPTOR: AKTİF [{self.int_mode_str}]")
        self.status_int.setStyleSheet("background-color: #276749; color: #e6fffa; padding: 4px 8px; border-radius: 4px; font-weight: bold;")

    def on_target_telemetry(self, d):
        if not d.get("valid", False):
            return
        # Talon telemetry: ENU from target_sim -> ENU p[0]=East, p[1]=North, p[2]=Up
        # Convert to local NED for matching:
        ve = d.get("ve_mps", 0.0)
        vn = d.get("vn_mps", 0.0)
        vd = d.get("vd_mps", 0.0)
        self.tgt_vel = np.array([vn, ve, vd])

        # Lat/Lon or rel alt
        alt_rel = d.get("alt_rel_m", 0.0)
        heading = d.get("heading_deg", 0.0)
        spd = d.get("ground_speed_mps", 0.0)

        if "pos_east_m" in d and "pos_north_m" in d:
            d_east = d["pos_east_m"]
            d_north = d["pos_north_m"]
        else:
            lat0, lon0 = 39.930898, 32.729591
            d_north = (d["lat"] - lat0) * 111132.95
            d_east = (d["lon"] - lon0) * (111319.49 * math.cos(math.radians(lat0)))

        self.tgt_pos[0] = d_north
        self.tgt_pos[1] = d_east
        self.tgt_pos[2] = -alt_rel # NED z

        self.tgt_trail_x.append(d_east)
        self.tgt_trail_y.append(d_north)

        self.tgt_speed = spd
        self.tgt_alt = alt_rel
        self.tgt_heading = heading
        self.tgt_mode = d.get("mode", "square")
        self.last_tgt_time = time.time()

    def on_target_http_status(self, connected):
        if connected:
            self.status_talon.setText(f"TALON: AKTİF [{self.tgt_mode.upper()}]")
            self.status_talon.setStyleSheet("background-color: #276749; color: #e6fffa; padding: 4px 8px; border-radius: 4px; font-weight: bold;")
        else:
            self.status_talon.setText("TALON: BAĞLANTI YOK")
            self.status_talon.setStyleSheet("background-color: #742a2a; color: #fff5f5; padding: 4px 8px; border-radius: 4px; font-weight: bold;")

    # ------------------------------------------------------------- GUI Update ---

    def update_gui(self):
        t_now = time.time() - self.t0

        # Compute relative separation and closing velocity
        d_rel = self.tgt_pos - self.int_pos
        dist = float(np.linalg.norm(d_rel))
        v_int_mag = float(np.linalg.norm(self.int_vel[:2]))
        v_tgt_mag = self.tgt_speed

        # Closing speed: Vc = - d(dist)/dt approx = dot(v_int - v_tgt, r_unit)
        r_unit = d_rel / max(dist, 0.01)
        closing_speed = float(np.dot(self.int_vel - self.tgt_vel, r_unit))

        # Update Distance Display & Color Codes
        if self.last_int_time > 0 and self.last_tgt_time > 0:
            self.lbl_dist_big.setText(f"{dist:.1f} m")
            if dist > 40.0:
                self.lbl_dist_big.setStyleSheet("font-size: 28px; font-weight: bold; color: #63b3ed; background-color: #1a202c; border-radius: 6px; padding: 4px 8px;")
                self.lbl_phase.setText("Güdüm Safhası: MIDCOURSE (GPS İntikali)")
                self.lbl_phase.setStyleSheet("font-size: 13px; font-weight: bold; color: #63b3ed;")
            elif dist > 20.0:
                self.lbl_dist_big.setStyleSheet("font-size: 28px; font-weight: bold; color: #ecc94b; background-color: #1a202c; border-radius: 6px; padding: 4px 8px;")
                self.lbl_phase.setText("Güdüm Safhası: TERMINAL (Optik 50 FPS)")
                self.lbl_phase.setStyleSheet("font-size: 13px; font-weight: bold; color: #ecc94b;")
            elif dist > 5.0:
                self.lbl_dist_big.setStyleSheet("font-size: 28px; font-weight: bold; color: #ed8936; background-color: #1a202c; border-radius: 6px; padding: 4px 8px;")
                self.lbl_phase.setText("Güdüm Safhası: TERMINAL (Yüksek Hız 80 FPS)")
                self.lbl_phase.setStyleSheet("font-size: 13px; font-weight: bold; color: #ed8936;")
            else:
                self.lbl_dist_big.setStyleSheet("font-size: 28px; font-weight: bold; color: #48bb78; background-color: #1a202c; border-radius: 6px; padding: 4px 8px;")
                self.lbl_phase.setText("Güdüm Safhası: AĞ ATIM MENZİLİ (KİLİTLİ)")
                self.lbl_phase.setStyleSheet("font-size: 13px; font-weight: bold; color: #48bb78;")

            self.lbl_vc.setText(f"Kapanma Hızı (Vc): {closing_speed:+.1f} m/s ({closing_speed*3.6:+.0f} km/h)")
            self.lbl_tilt.setText(f"Önleyici Pitch: {self.int_att[1]:.1f}° | Hız: {v_int_mag:.1f} m/s")

            # Fire Control Progress: when dist <= 5.0m
            if dist <= 5.5:
                if self.lock_start_time is None:
                    self.lock_start_time = time.time()
                elapsed = time.time() - self.lock_start_time
                pct = int(min(100.0, (elapsed / 0.40) * 100))
                self.lock_progress.setValue(pct)
                if pct >= 100:
                    self.lbl_net_status.setText("💥 AĞ FIRLATILDI! HEDEF YAKALANDI!")
                    self.lbl_net_status.setStyleSheet("font-weight: bold; color: #48bb78; font-size: 14px;")
            else:
                self.lock_start_time = None
                self.lock_progress.setValue(0)
                self.lbl_net_status.setText("AĞ DURUMU: HAZIR")
                self.lbl_net_status.setStyleSheet("font-weight: bold; color: #a0aec0; font-size: 13px;")

            # Histories for plots
            self.history_time.append(t_now)
            self.history_dist.append(dist)
            self.history_v_int.append(v_int_mag)
            self.history_v_tgt.append(v_tgt_mag)

            self.curve_dist.setData(list(self.history_time), list(self.history_dist))
            self.curve_v_int.setData(list(self.history_time), list(self.history_v_int))
            self.curve_v_tgt.setData(list(self.history_time), list(self.history_v_tgt))

        # Update Radar Map
        if len(self.tgt_trail_x) > 0:
            self.tgt_trail_curve.setData(list(self.tgt_trail_x), list(self.tgt_trail_y))
            self.tgt_marker.setData([self.tgt_pos[1]], [self.tgt_pos[0]])

            # Draw capture ring (r=5m) around Talon
            angles = np.linspace(0, 2*np.pi, 24)
            cx = self.tgt_pos[1] + 5.0 * np.cos(angles)
            cy = self.tgt_pos[0] + 5.0 * np.sin(angles)
            self.capture_circle.setData(cx, cy)

        if len(self.int_trail_x) > 0:
            self.int_trail_curve.setData(list(self.int_trail_x), list(self.int_trail_y))
            self.int_marker.setData([self.int_pos[1]], [self.int_pos[0]])

        # Draw Line-of-Sight
        if self.last_int_time > 0 and self.last_tgt_time > 0:
            self.los_curve.setData([self.int_pos[1], self.tgt_pos[1]], [self.int_pos[0], self.tgt_pos[0]])

    # --------------------------------------------------- Control Dispatchers ---

    def send_talon_cmd(self, payload):
        threading.Thread(target=self._send_http_post, args=(payload,), daemon=True).start()

    def _send_http_post(self, payload):
        try:
            req = urllib.request.Request(
                "http://127.0.0.1:8000/cmd",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}
            )
            urllib.request.urlopen(req, timeout=0.8)
        except Exception as e:
            print("HTTP POST error:", e)

    def send_talon_mode(self, mode_name):
        self.send_talon_cmd({"mode": mode_name})

    def on_speed_slider(self, val):
        self.lbl_spd_val.setText(f"{val} m/s")
        self.send_talon_cmd({"speed": float(val)})

    def on_alt_slider(self, val):
        self.lbl_alt_val.setText(f"{val} m")
        self.send_talon_cmd({"alt": float(val)})

    def send_manual_turn(self, turn_rate):
        self.send_talon_cmd({"mode": "manual", "turn_rate": turn_rate})

    def send_manual_climb(self, climb_rate):
        self.send_talon_cmd({"mode": "manual", "climb_rate": climb_rate})

    # Interceptor Commands (via px4cmd.sh)
    def cmd_takeoff(self):
        cmd = [os.path.join(SCRIPT_DIR, "px4cmd.sh"), "0", "commander", "takeoff"]
        subprocess.Popen(cmd)

    def cmd_intercept(self):
        cmd = [os.path.join(SCRIPT_DIR, "px4cmd.sh"), "0", "commander", "mode", "ext1"]
        subprocess.Popen(cmd)

    def cmd_hold(self):
        cmd = [os.path.join(SCRIPT_DIR, "px4cmd.sh"), "0", "commander", "mode", "posctl"]
        subprocess.Popen(cmd)

    def cmd_land(self):
        cmd = [os.path.join(SCRIPT_DIR, "px4cmd.sh"), "0", "commander", "mode", "auto:land"]
        subprocess.Popen(cmd)

    def launch_hud(self):
        cmd = [os.path.join(SCRIPT_DIR, "view.sh")]
        subprocess.Popen(cmd)

    def launch_qgc(self):
        qgc_path = "/home/kayra/Applications/QGroundControl-x86_64.AppImage"
        if os.path.exists(qgc_path):
            subprocess.Popen([qgc_path])
        else:
            QtWidgets.QMessageBox.warning(self, "Hata", f"QGroundControl bulunamadı:\n{qgc_path}")

    def closeEvent(self, event):
        self.gz_worker.stop()
        self.mavlink_worker.stop()
        self.http_worker.stop()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    gcs = InterceptorGCS()
    gcs.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

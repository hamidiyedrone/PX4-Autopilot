#!/usr/bin/env python3
"""
Control Panel (PySide6)

Left: interactive control over the simulated Talon target:
- Flight mode switching: Straight, S-Turn (Weave), Orbit
- Directional D-Pad remote control (climb/descend, turn left/right while held)
- Speed and altitude adjustment sliders & presets
- Real-time telemetry monitoring

Right: interceptor (PX4) control over MAVLink:
- Flight mode selection from the modes PX4 reports (AVAILABLE_MODES, incl. external modes)
- Arm/Disarm, Takeoff
- Live telemetry and the latest PX4 status text / command rejection

Connects to target_sim.py HTTP API at http://localhost:8000 and to the PX4 GCS link
(udp 14550) as the ground station, so QGroundControl cannot run at the same time.
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

os.environ.setdefault("MAVLINK20", "1")  # AVAILABLE_MODES is a MAVLink 2 message
from pymavlink import mavutil  # noqa: E402
from PySide6.QtCore import QEvent, QObject, QSize, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QIcon, QKeySequence, QShortcut, QTransform
from PySide6.QtWidgets import (
	QApplication,
	QComboBox,
	QGridLayout,
	QGroupBox,
	QHBoxLayout,
	QLabel,
	QMainWindow,
	QPushButton,
	QSlider,
	QSizePolicy,
	QSpinBox,
	QStyle,
	QToolButton,
	QVBoxLayout,
	QWidget,
)


# AVAILABLE_MODES is only in the development dialect of pymavlink (same CRC as PX4)
mavutil.set_dialect("development")
mav = mavutil.mavlink

class TelemetryWorker(QObject):
	"""Worker thread to poll target telemetry and dispatch commands asynchronously."""

	telemetry_received = Signal(dict)

	def __init__(self, host="127.0.0.1", port=8000):
		super().__init__()
		self.host = host
		self.port = port
		self.base_url = f"http://{host}:{port}"
		self._running = True

	def poll(self):
		if not self._running:
			return
		url = f"{self.base_url}/target"
		try:
			req = urllib.request.Request(url, headers={"User-Agent": "ControlPanel/1.0"})
			with urllib.request.urlopen(req, timeout=0.35) as resp:
				if resp.status == 200:
					data = json.loads(resp.read().decode("utf-8"))
					self.telemetry_received.emit(data)
					return
		except Exception:
			pass

	def send_command(self, cmd_dict):
		url = f"{self.base_url}/cmd"
		try:
			post_data = json.dumps(cmd_dict).encode("utf-8")
			req = urllib.request.Request(
				url,
				data=post_data,
				headers={"Content-Type": "application/json", "User-Agent": "ControlPanel/1.0"},
				method="POST",
			)
			with urllib.request.urlopen(req, timeout=0.5) as resp:
				pass
		except Exception:
			# Target might still be booting up
			pass


class MavlinkWorker(QObject):
	"""Runs in its own thread: reads the PX4 MAVLink link, keeps the mode list and telemetry."""

	modes_received = Signal(list)  # [(name, custom_mode), ...]
	telemetry_received = Signal(dict)
	message_received = Signal(str)

	# PX4 sends a name only for non-standard modes
	STANDARD_MODE_NAMES = {
		1: "Position",
		2: "Orbit",
		3: "Cruise",
		4: "Altitude",
		5: "Return",
		6: "Mission",
		7: "Land",
		8: "Takeoff",
	}

	def __init__(self, url):
		super().__init__()
		self.url = url
		self.conn = None
		self.target_system = None
		self.target_component = None
		self._running = True

	def stop(self):
		self._running = False

	def run(self):
		self.conn = mavutil.mavlink_connection(self.url, source_system=255, source_component=mav.MAV_COMP_ID_MISSIONPLANNER)
		modes = {}
		number_modes = 0
		modes_seq = None
		modes_requested = 0.0
		state = {}
		last_emit = 0.0
		last_heartbeat_sent = 0.0

		while self._running:
			msg = self.conn.recv_match(blocking=True, timeout=0.1)
			now = time.monotonic()

			# GCS heartbeat: PX4 counts the GCS as connected (data link loss failsafe)
			if now - last_heartbeat_sent > 1.0 and self.target_system is not None:
				last_heartbeat_sent = now
				self.conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, mav.MAV_STATE_ACTIVE)

			if msg is not None:
				t = msg.get_type()

				if t == "HEARTBEAT":
					# lock onto the first autopilot, ignore forwarded GCS/companion heartbeats
					if msg.autopilot == mav.MAV_AUTOPILOT_INVALID:
						continue
					if self.target_system is None:
						self.target_system = msg.get_srcSystem()
						self.target_component = msg.get_srcComponent()
					if msg.get_srcSystem() != self.target_system:
						continue
					state["armed"] = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
					state["custom_mode"] = msg.custom_mode
					state["last_heartbeat"] = now

				elif t == "AVAILABLE_MODES_MONITOR":
					if modes_seq is not None and msg.seq != modes_seq:
						modes.clear()
						modes_requested = 0.0  # re-request, e.g. an external mode registered
					modes_seq = msg.seq

				elif t == "AVAILABLE_MODES":
					number_modes = msg.number_modes
					name = msg.mode_name.rstrip("\0") if isinstance(msg.mode_name, str) else ""
					if not name:
						name = self.STANDARD_MODE_NAMES.get(msg.standard_mode, f"Mode {msg.custom_mode:#x}")
					modes[msg.mode_index] = (name, msg.custom_mode)
					if len(modes) == number_modes:
						self.modes_received.emit(
							[modes[i] for i in sorted(modes) if not modes[i][0].startswith("(")]
						)

				elif t == "GLOBAL_POSITION_INT":
					state["alt_rel_m"] = msg.relative_alt / 1000.0
					state["heading_deg"] = msg.hdg / 100.0 if msg.hdg != 65535 else None

				elif t == "VFR_HUD":
					state["ground_speed_mps"] = msg.groundspeed
					state["climb_mps"] = msg.climb

				elif t == "SYS_STATUS":
					state["battery_pct"] = msg.battery_remaining

				elif t == "GPS_RAW_INT":
					state["gps_sats"] = msg.satellites_visible
					state["gps_fix"] = msg.fix_type

				elif t == "STATUSTEXT":
					self.message_received.emit(msg.text)

				elif t == "COMMAND_ACK" and msg.result != mav.MAV_RESULT_ACCEPTED:
					cmd = mav.enums["MAV_CMD"].get(msg.command)
					res = mav.enums["MAV_RESULT"].get(msg.result)
					self.message_received.emit(
						f"{cmd.name if cmd else msg.command}: {res.name if res else msg.result}"
					)

			if self.target_system is None:
				continue

			# (re)request the full mode list until it is complete
			if (not number_modes or len(modes) < number_modes) and now - modes_requested > 3.0:
				modes.clear()
				modes_requested = now
				self.send_command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_AVAILABLE_MODES, 0)

			if now - last_emit > 0.2:
				last_emit = now
				state["connected"] = now - state.get("last_heartbeat", 0.0) < 3.0
				self.telemetry_received.emit(dict(state))

		self.conn.close()

	def send_command(self, command, *params):
		if self.conn is None or self.target_system is None:
			return
		p = list(params) + [0] * (7 - len(params))
		self.conn.mav.command_long_send(self.target_system, self.target_component, command, 0, *p)

	def set_mode(self, custom_mode):
		main_mode = (custom_mode >> 16) & 0xFF
		sub_mode = (custom_mode >> 24) & 0xFF
		self.send_command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode)


class ControlPanelWindow(QMainWindow):
	HOLD_PERIOD_MS = 100  # nudge rate while a direction is held
	YAW_RATE_DPS = 20.0  # matches target_sim straight-mode max turn rate
	ALT_RATE_MPS = 4.0

	def __init__(self, host="127.0.0.1", port=8000, mavlink_url="udpin:0.0.0.0:14550"):
		super().__init__()
		self.host = host
		self.port = port
		self.current_mode = "straight"
		self.target_speed = 25.0
		self.target_alt = 100.0
		self.target_heading = 0.0

		self.setWindowTitle("Control Panel")
		self.setFixedSize(800, 450)

		self.worker = TelemetryWorker(host, port)
		self.poll_timer = QTimer(self)
		self.poll_timer.timeout.connect(self.worker.poll)
		self.worker.telemetry_received.connect(self.on_telemetry)

		# Directions currently held via D-pad buttons or arrow keys
		self.held = set()
		self.hold_timer = QTimer(self)
		self.hold_timer.setInterval(self.HOLD_PERIOD_MS)
		self.hold_timer.timeout.connect(self._send_hold_nudge)

		# Interceptor (PX4) link in its own thread
		self.drone_armed = False
		self.drone_custom_mode = None
		self.mav_worker = MavlinkWorker(mavlink_url)
		self.mav_thread = QThread(self)
		self.mav_worker.moveToThread(self.mav_thread)
		self.mav_thread.started.connect(self.mav_worker.run)
		self.mav_worker.modes_received.connect(self.on_modes)
		self.mav_worker.telemetry_received.connect(self.on_drone_telemetry)
		self.mav_worker.message_received.connect(self.on_drone_message)

		self._init_ui()
		self._setup_shortcuts()

		self.poll_timer.start(180)  # ~5.5 Hz polling
		self.mav_thread.start()

	def _init_ui(self):
		central = QWidget(self)
		self.setCentralWidget(central)
		root_layout = QHBoxLayout(central)

		# Left half: target control
		target_group = QGroupBox("Target")
		main_layout = QVBoxLayout(target_group)
		root_layout.addWidget(target_group, 1)

		# Right half: interceptor control
		self._init_interceptor_ui(root_layout)

		# 1. Flight Mode Selection
		mode_group = QGroupBox("Flight Mode")
		btn_row = QHBoxLayout(mode_group)

		self.btn_straight = QPushButton("Straight")
		self.btn_straight.setToolTip("Fly a straight line [Shortcut: 1]")
		self.btn_straight.clicked.connect(lambda: self.send_cmd({"mode": "straight"}))

		self.btn_sturn = QPushButton("S-Turn")
		self.btn_sturn.setToolTip("Weave left and right [Shortcut: 2]")
		self.btn_sturn.clicked.connect(lambda: self.send_cmd({"mode": "s_turn"}))

		self.btn_orbit = QPushButton("Orbit")
		self.btn_orbit.setToolTip("Fly a circle [Shortcut: 3]")
		self.btn_orbit.clicked.connect(lambda: self.send_cmd({"mode": "orbit"}))

		for b in (self.btn_straight, self.btn_sturn, self.btn_orbit):
			b.setCheckable(True)
			btn_row.addWidget(b)

		main_layout.addWidget(mode_group)

		# 2. Directional D-Pad: nudges continuously while held
		pad_group = QGroupBox("Direction")
		grid = QGridLayout(pad_group)
		grid.setSpacing(2)
		grid.setAlignment(Qt.AlignCenter)

		# One arrow icon rotated for all four directions so they look identical
		up_arrow = self.style().standardIcon(QStyle.SP_ArrowUp).pixmap(20, 20)
		pad_buttons = [
			("up", 0, 0, 1, "Climb while held [Shortcut: Up]"),
			("left", 270, 1, 0, "Turn left while held [Shortcut: Left]"),
			("right", 90, 1, 2, "Turn right while held [Shortcut: Right]"),
			("down", 180, 2, 1, "Descend while held [Shortcut: Down]"),
		]
		for direction, angle, row, col, tip in pad_buttons:
			b = QToolButton()
			b.setIcon(QIcon(up_arrow.transformed(QTransform().rotate(angle))))
			b.setFixedSize(32, 32)
			b.setIconSize(QSize(20, 20))
			b.setToolTip(tip)
			b.pressed.connect(lambda d=direction: self._hold_start(d))
			b.released.connect(lambda d=direction: self._hold_stop(d))
			grid.addWidget(b, row, col)


		# 3. Speed & Altitude Setpoints
		set_group = QGroupBox("Speed && Altitude")
		set_layout = QVBoxLayout(set_group)
		set_layout.setSpacing(4)

		# Speed row
		spd_layout = QHBoxLayout()
		lbl_spd_title = QLabel("Speed")
		self.slider_speed = QSlider(Qt.Horizontal)
		self.slider_speed.setRange(15, 38)
		self.slider_speed.setValue(25)
		self.lbl_speed_val = QLabel("25 m/s")

		self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)
		self.slider_speed.sliderReleased.connect(self._on_speed_slider_released)

		spd_layout.addWidget(lbl_spd_title)
		spd_layout.addStretch()
		spd_layout.addWidget(self.lbl_speed_val)
		set_layout.addLayout(spd_layout)
		set_layout.addWidget(self.slider_speed)

		# Speed presets
		spd_presets = QHBoxLayout()
		spd_presets.setSpacing(4)
		for spd in [18, 22, 25, 30, 35]:
			b = QPushButton(f"{spd}")
			b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
			b.clicked.connect(lambda checked=False, s=spd: self._set_speed(s))
			spd_presets.addWidget(b)
		set_layout.addLayout(spd_presets)

		# Altitude row
		alt_layout = QHBoxLayout()
		lbl_alt_title = QLabel("Altitude")
		self.slider_alt = QSlider(Qt.Horizontal)
		self.slider_alt.setRange(20, 220)
		self.slider_alt.setValue(100)
		self.lbl_alt_val = QLabel("100 m")

		self.slider_alt.valueChanged.connect(self._on_alt_slider_changed)
		self.slider_alt.sliderReleased.connect(self._on_alt_slider_released)

		alt_layout.addWidget(lbl_alt_title)
		alt_layout.addStretch()
		alt_layout.addWidget(self.lbl_alt_val)
		set_layout.addLayout(alt_layout)
		set_layout.addWidget(self.slider_alt)

		# Altitude presets
		alt_presets = QHBoxLayout()
		alt_presets.setSpacing(4)
		for alt in [40, 70, 100, 130, 160]:
			b = QPushButton(f"{alt}")
			b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
			b.clicked.connect(lambda checked=False, a=alt: self._set_alt(a))
			alt_presets.addWidget(b)
		set_layout.addLayout(alt_presets)

		# D-pad and setpoints side by side
		control_row = QHBoxLayout()
		control_row.addWidget(pad_group)
		control_row.addWidget(set_group, 1)
		main_layout.addLayout(control_row)

		# 4. Live Telemetry Monitor
		telem_group = QGroupBox("Telemetry")
		telem_layout = QGridLayout(telem_group)
		telem_layout.setVerticalSpacing(2)

		self.tel_speed = QLabel("-- m/s")
		self.tel_alt = QLabel("-- m")
		self.tel_heading = QLabel("--°")
		self.tel_roll = QLabel("--°")
		telem_items = [
			("Speed:", self.tel_speed, 0, 0),
			("Alt:", self.tel_alt, 0, 2),
			("Heading:", self.tel_heading, 1, 0),
			("Roll:", self.tel_roll, 1, 2),
		]
		for title, value, row, col in telem_items:
			telem_layout.addWidget(QLabel(title), row, col)
			telem_layout.addWidget(value, row, col + 1)

		main_layout.addWidget(telem_group)
		main_layout.addStretch()

	def _init_interceptor_ui(self, root_layout):
		interceptor_group = QGroupBox("Interceptor")
		layout = QVBoxLayout(interceptor_group)
		root_layout.addWidget(interceptor_group, 1)

		# 1. Flight mode list as reported by PX4 (AVAILABLE_MODES, includes external modes)
		mode_group = QGroupBox("Flight Mode")
		mode_layout = QHBoxLayout(mode_group)
		self.combo_mode = QComboBox()
		self.combo_mode.setPlaceholderText("Waiting for PX4...")
		self.combo_mode.activated.connect(self._on_mode_selected)
		mode_layout.addWidget(self.combo_mode)
		layout.addWidget(mode_group)

		# 2. Commands
		cmd_group = QGroupBox("Commands")
		cmd_layout = QHBoxLayout(cmd_group)
		self.btn_arm = QPushButton("Arm")
		self.btn_arm.clicked.connect(self._on_arm_clicked)
		self.btn_takeoff = QPushButton("Takeoff")
		self.btn_takeoff.setToolTip("Take off to MIS_TAKEOFF_ALT")
		self.btn_takeoff.clicked.connect(self._on_takeoff_clicked)
		cmd_layout.addWidget(self.btn_arm)
		cmd_layout.addWidget(self.btn_takeoff)
		layout.addWidget(cmd_group)

		# 3. Live telemetry
		telem_group = QGroupBox("Telemetry")
		telem_layout = QGridLayout(telem_group)
		telem_layout.setVerticalSpacing(2)

		self.drone_tel = {}
		drone_items = [
			("Link:", "link", 0, 0),
			("Armed:", "armed", 0, 2),
			("Mode:", "mode", 1, 0),
			("GPS:", "gps", 1, 2),
			("Alt:", "alt", 2, 0),
			("Speed:", "speed", 2, 2),
			("Climb:", "climb", 3, 0),
			("Heading:", "heading", 3, 2),
			("Battery:", "battery", 4, 0),
		]
		for title, key, row, col in drone_items:
			self.drone_tel[key] = QLabel("--")
			telem_layout.addWidget(QLabel(title), row, col)
			telem_layout.addWidget(self.drone_tel[key], row, col + 1)

		self.lbl_drone_msg = QLabel("")
		self.lbl_drone_msg.setWordWrap(True)
		telem_layout.addWidget(self.lbl_drone_msg, 5, 0, 1, 4)

		layout.addWidget(telem_group)
		layout.addStretch()

	def _on_mode_selected(self, index):
		custom_mode = self.combo_mode.itemData(index)
		if custom_mode is not None:
			self.mav_worker.set_mode(custom_mode)

	def _on_arm_clicked(self):
		arm = 0 if self.drone_armed else 1
		self.mav_worker.send_command(mav.MAV_CMD_COMPONENT_ARM_DISARM, arm)

	def _on_takeoff_clicked(self):
		# altitude NaN: PX4 takes MIS_TAKEOFF_ALT (relative to home)
		nan = float("nan")
		self.mav_worker.send_command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan)

	def on_modes(self, modes):
		self.combo_mode.clear()
		for name, custom_mode in modes:
			self.combo_mode.addItem(name, custom_mode)
		self._show_current_mode()

	def _show_current_mode(self):
		index = self.combo_mode.findData(self.drone_custom_mode)
		self.combo_mode.setCurrentIndex(index)
		self.drone_tel["mode"].setText(self.combo_mode.itemText(index) if index >= 0 else "--")

	def on_drone_telemetry(self, data):
		connected = data.get("connected", False)
		self.drone_tel["link"].setText("OK" if connected else "Lost")

		self.drone_armed = data.get("armed", False)
		self.drone_tel["armed"].setText("Yes" if self.drone_armed else "No")
		self.btn_arm.setText("Disarm" if self.drone_armed else "Arm")

		if data.get("custom_mode") != self.drone_custom_mode:
			self.drone_custom_mode = data.get("custom_mode")
			self._show_current_mode()

		def fmt(key, spec, unit):
			value = data.get(key)
			return f"{value:{spec}}{unit}" if value is not None else "--"

		self.drone_tel["alt"].setText(fmt("alt_rel_m", ".1f", " m"))
		self.drone_tel["speed"].setText(fmt("ground_speed_mps", ".1f", " m/s"))
		self.drone_tel["climb"].setText(fmt("climb_mps", ".1f", " m/s"))
		self.drone_tel["heading"].setText(fmt("heading_deg", ".0f", "°"))
		battery = data.get("battery_pct")
		self.drone_tel["battery"].setText(f"{battery}%" if battery is not None and battery >= 0 else "--")
		if "gps_fix" in data:
			fix = {0: "No GPS", 1: "No fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
			self.drone_tel["gps"].setText(f"{fix.get(data['gps_fix'], data['gps_fix'])}, {data['gps_sats']} sat")

	def on_drone_message(self, text):
		self.lbl_drone_msg.setText(text)

	def closeEvent(self, event):
		self.mav_worker.stop()
		self.mav_thread.quit()
		self.mav_thread.wait(1000)
		super().closeEvent(event)

	def _setup_shortcuts(self):
		QShortcut(QKeySequence("1"), self, lambda: self.send_cmd({"mode": "straight"}))
		QShortcut(QKeySequence("2"), self, lambda: self.send_cmd({"mode": "s_turn"}))
		QShortcut(QKeySequence("3"), self, lambda: self.send_cmd({"mode": "orbit"}))
		QShortcut(QKeySequence("W"), self, lambda: self._nudge_speed(2.0))
		QShortcut(QKeySequence("S"), self, lambda: self._nudge_speed(-2.0))

		# Arrow keys act like the D-pad (hold to nudge); filtered app-wide so
		# focused sliders/buttons do not swallow them.
		self.arrow_keys = {Qt.Key_Up: "up", Qt.Key_Down: "down", Qt.Key_Left: "left", Qt.Key_Right: "right"}
		QApplication.instance().installEventFilter(self)

	def eventFilter(self, obj, event):
		if event.type() in (QEvent.KeyPress, QEvent.KeyRelease) and event.key() in self.arrow_keys:
			if not event.isAutoRepeat():
				direction = self.arrow_keys[event.key()]
				if event.type() == QEvent.KeyPress:
					self._hold_start(direction)
				else:
					self._hold_stop(direction)
			return True
		return super().eventFilter(obj, event)

	def _hold_start(self, direction):
		self.held.add(direction)
		self._send_hold_nudge()
		self.hold_timer.start()

	def _hold_stop(self, direction):
		self.held.discard(direction)
		if not self.held:
			self.hold_timer.stop()

	def _send_hold_nudge(self):
		dt = self.HOLD_PERIOD_MS / 1000.0
		cmd = {}
		alt_sign = ("up" in self.held) - ("down" in self.held)
		if alt_sign:
			cmd["nudge_alt"] = alt_sign * self.ALT_RATE_MPS * dt
		# positive nudge_yaw = left turn (ENU yaw)
		yaw_sign = ("left" in self.held) - ("right" in self.held)
		if yaw_sign:
			cmd["nudge_yaw"] = yaw_sign * self.YAW_RATE_DPS * dt
		if cmd:
			self.send_cmd(cmd)

	def _nudge_speed(self, delta):
		cur = self.slider_speed.value()
		new_v = max(15, min(38, cur + int(delta)))
		self._set_speed(new_v)

	def _set_speed(self, spd):
		self.slider_speed.setValue(int(spd))
		self.lbl_speed_val.setText(f"{spd} m/s")
		self.send_cmd({"speed": float(spd)})

	def _set_alt(self, alt):
		self.slider_alt.setValue(int(alt))
		self.lbl_alt_val.setText(f"{alt} m")
		self.send_cmd({"alt": float(alt)})

	def _on_speed_slider_changed(self, val):
		self.lbl_speed_val.setText(f"{val} m/s")

	def _on_speed_slider_released(self):
		self.send_cmd({"speed": float(self.slider_speed.value())})

	def _on_alt_slider_changed(self, val):
		self.lbl_alt_val.setText(f"{val} m")

	def _on_alt_slider_released(self):
		self.send_cmd({"alt": float(self.slider_alt.value())})

	def send_cmd(self, cmd_dict):
		self.worker.send_command(cmd_dict)

	def on_telemetry(self, data):
		if not data.get("valid", False):
			return

		mode = data.get("mode", "straight")
		self.current_mode = mode

		# Highlight active mode button
		self.btn_straight.setChecked(mode == "straight")
		self.btn_sturn.setChecked(mode in ("s_turn", "weave"))
		self.btn_orbit.setChecked(mode == "orbit")

		# Update telemetry labels
		spd = data.get("ground_speed_mps", 0.0)
		alt = data.get("alt_rel_m", 0.0)
		hdg = data.get("heading_deg", 0.0)
		roll = data.get("roll_deg", 0.0)

		self.tel_speed.setText(f"{spd:4.1f} m/s")
		self.tel_alt.setText(f"{alt:5.1f} m")
		self.tel_heading.setText(f"{hdg:5.1f}°")
		self.tel_roll.setText(f"{roll:4.1f}°")


def main():
	parser = argparse.ArgumentParser(description="Control Panel")
	parser.add_argument("--host", default="127.0.0.1", help="Target sim HTTP host")
	parser.add_argument("--port", type=int, default=8000, help="Target sim HTTP port")
	parser.add_argument("--mavlink", default="udpin:0.0.0.0:14550", help="Interceptor PX4 MAVLink link (GCS link, like QGC)")
	args = parser.parse_args()

	app = QApplication(sys.argv)
	win = ControlPanelWindow(host=args.host, port=args.port, mavlink_url=args.mavlink)
	win.show()
	sys.exit(app.exec())


if __name__ == "__main__":
	main()

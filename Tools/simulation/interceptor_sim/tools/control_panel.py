#!/usr/bin/env python3
"""
Control Panel (PySide6)

Tab 1: Flight Control
- Left: interactive control over the simulated Talon target:
  - Flight mode switching: Straight, S-Turn (Weave), Orbit
  - Directional D-Pad remote control (climb/descend, turn left/right while held)
  - Speed and altitude adjustment sliders & presets
  - Real-time telemetry monitoring
- Right: interceptor (PX4) control over MAVLink:
  - Flight mode selection from the modes PX4 reports (AVAILABLE_MODES, incl. external modes)
  - Arm/Disarm, Takeoff
  - Live telemetry and the latest PX4 status text / command rejection

Tab 2: Parameters (INT_*)
- Live synchronization with PX4 via MAVLink (PARAM_REQUEST_READ / PARAM_VALUE / PARAM_SET)
- Edit parameters and save all to PX4 and persistent storage
- View default values, min/max allowable ranges, and full documentation from module.yaml
- Real-time search/filter bar

Connects to target_sim.py HTTP API at http://localhost:8000 and to the PX4 GCS link
(udp 14550) as the ground station, so QGroundControl cannot run at the same time.
"""

import argparse
import copy
import json
import math
import os
import struct
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

import yaml

os.environ.setdefault("MAVLINK20", "1")  # AVAILABLE_MODES is a MAVLink 2 message
from pymavlink import mavutil  # noqa: E402
from PySide6.QtCore import QEvent, QObject, QSize, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QIcon, QKeySequence, QShortcut, QTransform
from PySide6.QtWidgets import (
	QApplication,
	QComboBox,
	QDoubleSpinBox,
	QGridLayout,
	QGroupBox,
	QHBoxLayout,
	QHeaderView,
	QLabel,
	QLineEdit,
	QMainWindow,
	QProgressBar,
	QPushButton,
	QSlider,
	QSizePolicy,
	QSpinBox,
	QStyle,
	QTableWidget,
	QTableWidgetItem,
	QTabWidget,
	QToolButton,
	QVBoxLayout,
	QWidget,
)

# AVAILABLE_MODES is only in the development dialect of pymavlink (same CRC as PX4)
mavutil.set_dialect("development")
mav = mavutil.mavlink


def load_intercept_param_metadata():
	"""Load intercept mode parameter definitions, types, defaults, and descriptions from module.yaml."""
	yaml_path = os.path.abspath(
		os.path.join(os.path.dirname(__file__), "../../../..", "src/modules/intercept/module.yaml")
	)
	params = {}
	if os.path.isfile(yaml_path):
		try:
			with open(yaml_path, "r", encoding="utf-8") as f:
				data = yaml.safe_load(f)
			for group in data.get("parameters", []):
				for name, info in group.get("definitions", {}).items():
					desc = info.get("description", {})
					short_desc = desc.get("short", "") if isinstance(desc, dict) else str(desc)
					long_desc = desc.get("long", "") if isinstance(desc, dict) else ""
					params[name] = {
						"name": name,
						"type": info.get("type", "float"),
						"default": info.get("default", 0.0),
						"min": info.get("min", None),
						"max": info.get("max", None),
						"decimal": info.get("decimal", 2),
						"short": short_desc,
						"long": long_desc,
					}
		except Exception as e:
			print(f"Warning: Failed to parse module.yaml ({e}), using built-in defaults.")

	# Safe fallback if module.yaml could not be read
	if not params:
		params = {
			"INT_ENABLE": {
				"name": "INT_ENABLE", "type": "boolean", "default": 1, "min": 0, "max": 1,
				"short": "Enable intercept flight mode",
				"long": "Enables the target intercept autonomous flight mode."
			},
			"INT_STANDOFF": {
				"name": "INT_STANDOFF", "type": "float", "default": 3.0, "min": 1.0, "max": 10.0, "decimal": 1,
				"short": "Desired terminal standoff distance",
				"long": "Horizontal distance in meters to maintain behind target during close pursuit."
			},
			"INT_LOST_TOUT": {
				"name": "INT_LOST_TOUT", "type": "float", "default": 2.0, "min": 0.5, "max": 10.0, "decimal": 1,
				"short": "Visual contact loss timeout",
				"long": "Time in seconds to steer towards target position and re-acquire visual before reverting to approach."
			},
			"INT_APP_MIN_VEL": {
				"name": "INT_APP_MIN_VEL", "type": "float", "default": 6.0, "min": 2.0, "max": 15.0, "decimal": 1,
				"short": "Minimum closing speed in approach guidance",
				"long": "Minimum relative closing speed (m/s) maintained in midcourse position approach guidance."
			},
			"INT_APP_KP_Z": {
				"name": "INT_APP_KP_Z", "type": "float", "default": 0.8, "min": 0.1, "max": 3.0, "decimal": 2,
				"short": "Approach mode altitude tracking proportional gain",
				"long": "Proportional gain (m/s per meter altitude error) for midcourse approach altitude tracking."
			},
			"INT_APP_YAWRATE": {
				"name": "INT_APP_YAWRATE", "type": "float", "default": 42.0, "min": 5.0, "max": 90.0, "decimal": 1,
				"short": "Maximum heading slew rate in approach",
				"long": "Maximum heading change rate (deg/s) in midcourse approach guidance."
			},
			"INT_TRM_KP_FWD": {
				"name": "INT_TRM_KP_FWD", "type": "float", "default": 0.8, "min": 0.1, "max": 5.0, "decimal": 2,
				"short": "Terminal standoff distance error velocity gain",
				"long": "Proportional speed gain commanding forward pursuit speed offset from horizontal distance error."
			},
			"INT_TRM_KD_FWD": {
				"name": "INT_TRM_KD_FWD", "type": "float", "default": 1.2, "min": 0.1, "max": 10.0, "decimal": 2,
				"short": "Terminal relative closing speed damping gain",
				"long": "Derivative damping gain on optical relative closing speed (m/s) matching target velocity."
			},
			"INT_TRM_DIST_TOL": {
				"name": "INT_TRM_DIST_TOL", "type": "float", "default": 0.20, "min": 0.0, "max": 1.0, "decimal": 2,
				"short": "Terminal standoff distance deadband tolerance",
				"long": "Distance deadband tolerance (+/- meters) around desired standoff in close pursuit to prevent pitch oscillations."
			},
			"INT_TRM_KP_LAT": {
				"name": "INT_TRM_KP_LAT", "type": "float", "default": 1.0, "min": 0.1, "max": 5.0, "decimal": 2,
				"short": "Terminal lateral position proportional gain",
				"long": "Proportional gain (m/s per meter lateral offset) for centering target horizontally."
			},
			"INT_TRM_LAT_MAX": {
				"name": "INT_TRM_LAT_MAX", "type": "float", "default": 1.5, "min": 0.0, "max": 5.0, "decimal": 1,
				"short": "Terminal maximum lateral strafe speed",
				"long": "Maximum lateral velocity correction (m/s) to maintain coordinated flight without excessive banking."
			},
			"INT_TRM_BRK_LAT": {
				"name": "INT_TRM_BRK_LAT", "type": "float", "default": 0.0, "min": 0.0, "max": 10.0, "decimal": 1,
				"short": "Terminal lateral cornering brake speed reduction",
				"long": "Maximum forward speed reduction (m/s) when target shifts laterally off-boresight in close pursuit."
			},
			"INT_TRM_BRK_TOL": {
				"name": "INT_TRM_BRK_TOL", "type": "float", "default": 0.25, "min": 0.05, "max": 0.45, "decimal": 2,
				"short": "Terminal corner braking deadband tolerance",
				"long": "Lateral screen offset deadband (+/- from center 0.50) before corner braking activates. 0.25 gives corridor [0.25, 0.75]."
			},
			"INT_TRM_KP_Z": {
				"name": "INT_TRM_KP_Z", "type": "float", "default": 0.6, "min": 0.1, "max": 5.0, "decimal": 2,
				"short": "Terminal altitude tracking proportional gain",
				"long": "Proportional gain (m/s per meter altitude error) for terminal standoff altitude tracking."
			},
			"INT_TRM_Z_TOL": {
				"name": "INT_TRM_Z_TOL", "type": "float", "default": 0.3, "min": 0.05, "max": 1.0, "decimal": 2,
				"short": "Terminal altitude tracking deadband tolerance",
				"long": "Altitude deadband tolerance (+/- meters) in close pursuit to prevent optical pitch oscillations."
			},
			"INT_TRM_ACC_Z": {
				"name": "INT_TRM_ACC_Z", "type": "float", "default": 1.2, "min": 0.2, "max": 5.0, "decimal": 1,
				"short": "Terminal close pursuit vertical acceleration limit",
				"long": "Maximum vertical acceleration (m/s^2) in close pursuit to prevent pitch oscillations."
			},
			"INT_TRM_YAWRATE": {
				"name": "INT_TRM_YAWRATE", "type": "float", "default": 18.0, "min": 5.0, "max": 60.0, "decimal": 1,
				"short": "Terminal maximum heading slew rate",
				"long": "Maximum heading change rate (deg/s) in terminal close pursuit to eliminate yaw hunting."
			},
		}
	return params


AUTOTUNE_PROFILES = {
	"forward": {
		"title": "Forward Standoff & Speed (KP_FWD, KD_FWD, DIST_TOL)",
		"domain": "forward",
		"params": ["INT_TRM_KP_FWD", "INT_TRM_KD_FWD", "INT_TRM_DIST_TOL"],
		"candidates": [
			{"name": "Current Baseline", "params": {"INT_TRM_KP_FWD": 0.8, "INT_TRM_KD_FWD": 1.2, "INT_TRM_DIST_TOL": 0.20}},
			{"name": "Soft / Anti-Jitter", "params": {"INT_TRM_KP_FWD": 0.6, "INT_TRM_KD_FWD": 1.4, "INT_TRM_DIST_TOL": 0.30}},
			{"name": "High Damping", "params": {"INT_TRM_KP_FWD": 0.8, "INT_TRM_KD_FWD": 1.8, "INT_TRM_DIST_TOL": 0.20}},
			{"name": "Responsive Crisp", "params": {"INT_TRM_KP_FWD": 1.1, "INT_TRM_KD_FWD": 1.5, "INT_TRM_DIST_TOL": 0.15}},
			{"name": "Tight Tracking", "params": {"INT_TRM_KP_FWD": 1.4, "INT_TRM_KD_FWD": 2.0, "INT_TRM_DIST_TOL": 0.10}},
			{"name": "Loose Deadband", "params": {"INT_TRM_KP_FWD": 1.0, "INT_TRM_KD_FWD": 1.6, "INT_TRM_DIST_TOL": 0.35}},
		],
	},
	"lateral": {
		"title": "Lateral Strafe & Alignment (KP_LAT, LAT_MAX, BRK_LAT, YAWRATE)",
		"domain": "lateral",
		"params": ["INT_TRM_KP_LAT", "INT_TRM_LAT_MAX", "INT_TRM_BRK_LAT", "INT_TRM_YAWRATE"],
		"candidates": [
			{"name": "Current Baseline", "params": {"INT_TRM_KP_LAT": 0.2, "INT_TRM_LAT_MAX": 0.3, "INT_TRM_BRK_LAT": 1.5, "INT_TRM_YAWRATE": 18.0}},
			{"name": "Soft Drift (Low Brake)", "params": {"INT_TRM_KP_LAT": 0.3, "INT_TRM_LAT_MAX": 0.5, "INT_TRM_BRK_LAT": 0.8, "INT_TRM_YAWRATE": 18.0}},
			{"name": "Smooth Strafe", "params": {"INT_TRM_KP_LAT": 0.4, "INT_TRM_LAT_MAX": 0.6, "INT_TRM_BRK_LAT": 0.4, "INT_TRM_YAWRATE": 20.0}},
			{"name": "Zero Corner Brake", "params": {"INT_TRM_KP_LAT": 0.5, "INT_TRM_LAT_MAX": 0.8, "INT_TRM_BRK_LAT": 0.0, "INT_TRM_YAWRATE": 20.0}},
			{"name": "Agile Strafe", "params": {"INT_TRM_KP_LAT": 0.6, "INT_TRM_LAT_MAX": 1.0, "INT_TRM_BRK_LAT": 0.5, "INT_TRM_YAWRATE": 22.0}},
			{"name": "Tight Corridor", "params": {"INT_TRM_KP_LAT": 0.8, "INT_TRM_LAT_MAX": 1.2, "INT_TRM_BRK_LAT": 0.0, "INT_TRM_YAWRATE": 24.0}},
		],
	},
	"altitude": {
		"title": "Altitude Tracking (KP_Z, Z_TOL, ACC_Z)",
		"domain": "altitude",
		"params": ["INT_TRM_KP_Z", "INT_TRM_Z_TOL", "INT_TRM_ACC_Z"],
		"candidates": [
			{"name": "Current Baseline", "params": {"INT_TRM_KP_Z": 0.6, "INT_TRM_Z_TOL": 0.30, "INT_TRM_ACC_Z": 1.2}},
			{"name": "Gentle Vertical", "params": {"INT_TRM_KP_Z": 0.4, "INT_TRM_Z_TOL": 0.40, "INT_TRM_ACC_Z": 0.8}},
			{"name": "Crisp Altitude", "params": {"INT_TRM_KP_Z": 0.8, "INT_TRM_Z_TOL": 0.25, "INT_TRM_ACC_Z": 1.6}},
			{"name": "Tight Deadband", "params": {"INT_TRM_KP_Z": 1.0, "INT_TRM_Z_TOL": 0.15, "INT_TRM_ACC_Z": 1.8}},
			{"name": "Dynamic Climb/Dive", "params": {"INT_TRM_KP_Z": 1.3, "INT_TRM_Z_TOL": 0.20, "INT_TRM_ACC_Z": 2.4}},
		],
	},
	"full": {
		"title": "Full Multi-Stage Suite (Forward -> Lateral -> Altitude)",
		"domain": "full",
		"params": [],
		"candidates": [],
	},
}


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
	"""Runs in its own thread: reads the PX4 MAVLink link, handles modes, telemetry and parameters."""

	modes_received = Signal(list)  # [(name, custom_mode), ...]
	telemetry_received = Signal(dict)
	message_received = Signal(str)
	param_received = Signal(str, float, int)  # param_name, value, param_type

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

				elif t == "PARAM_VALUE":
					name = msg.param_id
					if isinstance(name, bytes):
						name = name.decode("ascii", errors="ignore").rstrip("\x00")
					elif isinstance(name, str):
						name = name.rstrip("\x00")

					val = msg.param_value
					ptype = msg.param_type
					if ptype in (
						mav.MAV_PARAM_TYPE_INT32,
						mav.MAV_PARAM_TYPE_UINT32,
						mav.MAV_PARAM_TYPE_INT8,
						mav.MAV_PARAM_TYPE_UINT8,
						mav.MAV_PARAM_TYPE_INT16,
						mav.MAV_PARAM_TYPE_UINT16,
					):
						try:
							val = float(struct.unpack("i", struct.pack("f", val))[0])
						except Exception:
							pass
					self.param_received.emit(name, val, ptype)

				elif t == "ATTITUDE":
					state["pitch_deg"] = math.degrees(msg.pitch)
					state["roll_deg"] = math.degrees(msg.roll)
					state["yaw_deg"] = math.degrees(msg.yaw)
					state["pitchspeed_dps"] = math.degrees(msg.pitchspeed)
					state["rollspeed_dps"] = math.degrees(msg.rollspeed)
					state["yawspeed_dps"] = math.degrees(msg.yawspeed)

				elif t == "GLOBAL_POSITION_INT":
					state["alt_rel_m"] = msg.relative_alt / 1000.0
					state["heading_deg"] = msg.hdg / 100.0 if msg.hdg != 65535 else None
					state["lat"] = msg.lat / 1e7
					state["lon"] = msg.lon / 1e7
					state["vx"] = msg.vx / 100.0
					state["vy"] = msg.vy / 100.0
					state["vz"] = msg.vz / 100.0

				elif t == "LOCAL_POSITION_NED":
					state["ned_x"] = msg.x
					state["ned_y"] = msg.y
					state["ned_z"] = msg.z
					state["ned_vx"] = msg.vx
					state["ned_vy"] = msg.vy
					state["ned_vz"] = msg.vz

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
		if self.conn is None or self.target_system is None or self.target_component is None:
			return
		p = list(params) + [0] * (7 - len(params))
		self.conn.mav.command_long_send(self.target_system, self.target_component, command, 0, *p)

	def set_mode(self, custom_mode):
		main_mode = (custom_mode >> 16) & 0xFF
		sub_mode = (custom_mode >> 24) & 0xFF
		self.send_command(mav.MAV_CMD_DO_SET_MODE, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main_mode, sub_mode)

	def request_param(self, name):
		if self.conn is None or self.target_system is None or self.target_component is None:
			return
		param_id = name.encode("ascii")[:16]
		self.conn.mav.param_request_read_send(
			self.target_system,
			self.target_component,
			param_id,
			-1,
		)

	def set_param(self, name, value, is_int=False):
		if self.conn is None or self.target_system is None or self.target_component is None:
			return
		param_id = name.encode("ascii")[:16]
		if is_int:
			packed_val = struct.unpack("f", struct.pack("i", int(value)))[0]
			ptype = mav.MAV_PARAM_TYPE_INT32
		else:
			packed_val = float(value)
			ptype = mav.MAV_PARAM_TYPE_REAL32
		self.conn.mav.param_set_send(
			self.target_system,
			self.target_component,
			param_id,
			packed_val,
			ptype,
		)

	def save_params_to_storage(self):
		"""Command PX4 to persist current parameters into non-volatile storage."""
		if self.conn is None or self.target_system is None or self.target_component is None:
			return
		self.send_command(mav.MAV_CMD_PREFLIGHT_STORAGE, 1.0)


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
		self.resize(960, 600)
		self.setMinimumSize(850, 520)

		# Parameter management state
		self.param_metadata = load_intercept_param_metadata()
		self.px4_param_values = {}
		self.param_editors = {}
		self.param_px4_items = {}
		self._initial_params_requested = False

		self.worker = TelemetryWorker(host, port)
		self.poll_timer = QTimer(self)
		self.poll_timer.timeout.connect(self.worker.poll)
		self.worker.telemetry_received.connect(self.on_telemetry)

		# Directions currently held via D-pad buttons or arrow keys
		self.held = set()
		self.hold_timer = QTimer(self)
		self.hold_timer.setInterval(self.HOLD_PERIOD_MS)
		self.hold_timer.timeout.connect(self._send_hold_nudge)

		# Telemetry snapshots for live relative distance & rate metrics
		self.last_target_telem = {}
		self.last_drone_telem = {}

		# Auto-Tune state machine
		self.autotune_timer = QTimer(self)
		self.autotune_timer.setInterval(100)  # 10 Hz control loop
		self.autotune_timer.timeout.connect(self._on_autotune_tick)

		self.autotune_state = "IDLE"  # "IDLE", "APPLYING", "STABILIZING", "TESTING"
		self.autotune_profile_key = "forward"
		self.autotune_candidate_idx = 0
		self.autotune_candidates = []
		self.autotune_active_candidate = None
		self.autotune_trial_duration = 10.0
		self.autotune_phase_start = 0.0
		self.autotune_trial_start_time = 0.0
		self.autotune_samples = {}
		self.autotune_results = []
		self.best_trial_idx = -1
		self.initial_params_backup = {}

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
		self.mav_worker.param_received.connect(self.on_param_received)

		self._init_ui()
		self._setup_shortcuts()

		self.poll_timer.start(180)  # ~5.5 Hz polling
		self.mav_thread.start()

	def _init_ui(self):
		self.tabs = QTabWidget(self)
		self.setCentralWidget(self.tabs)

		# Tab 1: Flight Control
		flight_tab = QWidget()
		flight_layout = QHBoxLayout(flight_tab)

		# Left half: target control
		target_group = QGroupBox("Target (Talon)")
		main_layout = QVBoxLayout(target_group)
		flight_layout.addWidget(target_group, 1)

		# Right half: interceptor control
		self._init_interceptor_ui(flight_layout)

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
		set_group = QGroupBox("Speed & Altitude")
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

		self.tabs.addTab(flight_tab, "Flight Control")

		# Tab 2: Intercept Parameters
		params_tab = QWidget()
		self._init_params_ui(params_tab)
		self.tabs.addTab(params_tab, "Parameters")

		# Tab 3: Auto-Tune
		tune_tab = QWidget()
		self._init_autotune_ui(tune_tab)
		self.tabs.addTab(tune_tab, "Auto-Tune")

		self.tabs.currentChanged.connect(self._on_tab_changed)

	def _init_interceptor_ui(self, root_layout):
		interceptor_group = QGroupBox("Interceptor (PX4)")
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

	def _init_params_ui(self, parent):
		layout = QVBoxLayout(parent)
		layout.setContentsMargins(10, 10, 10, 10)
		layout.setSpacing(8)

		# Top Action & Filter Row
		top_row = QHBoxLayout()
		lbl_search = QLabel("Filter:")
		self.txt_filter = QLineEdit()
		self.txt_filter.setPlaceholderText("Search by name or description...")
		self.txt_filter.setClearButtonEnabled(True)
		self.txt_filter.textChanged.connect(self._on_filter_changed)

		self.btn_refresh_params = QPushButton("Refresh")
		self.btn_refresh_params.setToolTip("Fetch all intercept parameters from PX4")
		self.btn_refresh_params.clicked.connect(self._on_refresh_params)

		self.btn_save_all_params = QPushButton("Save")
		self.btn_save_all_params.setStyleSheet("font-weight: bold;")
		self.btn_save_all_params.setToolTip("Send all parameters to PX4 and persist to storage")
		self.btn_save_all_params.clicked.connect(self._on_save_all_params)

		self.btn_reset_defaults = QPushButton("Reset Defaults")
		self.btn_reset_defaults.setToolTip("Reset all values to module.yaml defaults")
		self.btn_reset_defaults.clicked.connect(self._on_reset_defaults)

		top_row.addWidget(lbl_search)
		top_row.addWidget(self.txt_filter, 1)
		top_row.addWidget(self.btn_refresh_params)
		top_row.addWidget(self.btn_save_all_params)
		top_row.addWidget(self.btn_reset_defaults)
		layout.addLayout(top_row)

		# Parameters Table
		headers = ["Parameter", "Value", "PX4 Value", "Default", "Range", "Description"]
		self.table_params = QTableWidget(len(self.param_metadata), len(headers))
		self.table_params.setHorizontalHeaderLabels(headers)
		self.table_params.verticalHeader().setVisible(False)
		self.table_params.setAlternatingRowColors(True)
		self.table_params.setSelectionBehavior(QTableWidget.SelectRows)
		self.table_params.setSelectionMode(QTableWidget.SingleSelection)

		for row, (name, meta) in enumerate(self.param_metadata.items()):
			# Col 0: Name
			item_name = QTableWidgetItem(name)
			font = item_name.font()
			font.setBold(True)
			item_name.setFont(font)
			item_name.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
			self.table_params.setItem(row, 0, item_name)

			# Col 1: Editor widget
			is_bool = (meta.get("type") == "boolean")
			if is_bool:
				editor = QComboBox()
				editor.addItem("0 (Disabled)")
				editor.addItem("1 (Enabled)")
				editor.setCurrentIndex(1 if int(meta.get("default", 1)) == 1 else 0)
			else:
				editor = QDoubleSpinBox()
				editor.setDecimals(meta.get("decimal", 2))
				min_v = meta.get("min", -999.0) if meta.get("min") is not None else -999.0
				max_v = meta.get("max", 999.0) if meta.get("max") is not None else 999.0
				editor.setRange(min_v, max_v)
				editor.setSingleStep(0.1 if meta.get("decimal", 2) <= 1 else 0.05)
				editor.setValue(float(meta.get("default", 0.0)))
			self.param_editors[name] = editor
			self.table_params.setCellWidget(row, 1, editor)

			# Col 2: PX4 Value
			item_px4 = QTableWidgetItem("--")
			item_px4.setTextAlignment(Qt.AlignCenter)
			item_px4.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
			self.param_px4_items[name] = item_px4
			self.table_params.setItem(row, 2, item_px4)

			# Col 3: Default
			item_def = QTableWidgetItem(str(meta.get("default", "-")))
			item_def.setTextAlignment(Qt.AlignCenter)
			item_def.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
			self.table_params.setItem(row, 3, item_def)

			# Col 4: Range
			if meta.get("min") is not None and meta.get("max") is not None:
				range_str = f"[{meta['min']}, {meta['max']}]"
			else:
				range_str = "-"
			item_range = QTableWidgetItem(range_str)
			item_range.setTextAlignment(Qt.AlignCenter)
			item_range.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
			self.table_params.setItem(row, 4, item_range)

			# Col 5: Description
			item_desc = QTableWidgetItem(meta.get("short", ""))
			item_desc.setToolTip(meta.get("long", meta.get("short", "")))
			item_desc.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
			self.table_params.setItem(row, 5, item_desc)

		# Column widths
		header = self.table_params.horizontalHeader()
		header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
		header.setSectionResizeMode(1, QHeaderView.Fixed)
		self.table_params.setColumnWidth(1, 130)
		header.setSectionResizeMode(2, QHeaderView.Fixed)
		self.table_params.setColumnWidth(2, 105)
		header.setSectionResizeMode(3, QHeaderView.Fixed)
		self.table_params.setColumnWidth(3, 85)
		header.setSectionResizeMode(4, QHeaderView.Fixed)
		self.table_params.setColumnWidth(4, 95)
		header.setSectionResizeMode(5, QHeaderView.Stretch)

		layout.addWidget(self.table_params, 1)

		# Status Bar Row
		status_row = QHBoxLayout()
		self.lbl_param_status = QLabel("Ready.")
		self.lbl_param_count = QLabel(f"{len(self.param_metadata)} parameters")
		self.lbl_param_count.setStyleSheet("color: gray;")
		status_row.addWidget(self.lbl_param_status, 1)
		status_row.addWidget(self.lbl_param_count)
		layout.addLayout(status_row)

	def _init_autotune_ui(self, parent):
		layout = QVBoxLayout(parent)
		layout.setContentsMargins(10, 10, 10, 10)
		layout.setSpacing(8)

		# 1. Configuration & Control Row
		cfg_group = QGroupBox("Auto-Tune Configuration")
		cfg_layout = QHBoxLayout(cfg_group)
		cfg_layout.setSpacing(10)

		lbl_profile = QLabel("Profile:")
		self.combo_tune_profile = QComboBox()
		for key, prof in AUTOTUNE_PROFILES.items():
			self.combo_tune_profile.addItem(prof["title"], key)

		lbl_dur = QLabel("Trial Duration:")
		self.spin_trial_dur = QSpinBox()
		self.spin_trial_dur.setRange(6, 30)
		self.spin_trial_dur.setValue(10)
		self.spin_trial_dur.setSuffix(" s")

		self.btn_start_tune = QPushButton("Start Auto-Tune")
		self.btn_start_tune.setStyleSheet("font-weight: bold; background-color: #2e7d32; color: white;")
		self.btn_start_tune.clicked.connect(self._start_autotune)

		self.btn_stop_tune = QPushButton("Stop / Abort")
		self.btn_stop_tune.setStyleSheet("font-weight: bold; background-color: #c62828; color: white;")
		self.btn_stop_tune.setEnabled(False)
		self.btn_stop_tune.clicked.connect(self._stop_autotune)

		self.btn_apply_best = QPushButton("Apply Best to PX4")
		self.btn_apply_best.setStyleSheet("font-weight: bold; background-color: #1565c0; color: white;")
		self.btn_apply_best.setEnabled(False)
		self.btn_apply_best.clicked.connect(self._apply_best_params)

		cfg_layout.addWidget(lbl_profile)
		cfg_layout.addWidget(self.combo_tune_profile, 1)
		cfg_layout.addWidget(lbl_dur)
		cfg_layout.addWidget(self.spin_trial_dur)
		cfg_layout.addWidget(self.btn_start_tune)
		cfg_layout.addWidget(self.btn_stop_tune)
		cfg_layout.addWidget(self.btn_apply_best)
		layout.addWidget(cfg_group)

		# 2. Live Status & Metrics Card
		status_group = QGroupBox("Live Status & Active Metrics")
		status_layout = QVBoxLayout(status_group)
		status_layout.setSpacing(6)

		progress_row = QHBoxLayout()
		self.lbl_tune_state = QLabel("Status: Ready. Start auto-tune when interceptor is airborne behind target.")
		self.lbl_tune_state.setStyleSheet("font-weight: bold;")
		self.progress_suite = QProgressBar()
		self.progress_suite.setRange(0, 100)
		self.progress_suite.setValue(0)
		self.progress_suite.setTextVisible(True)
		self.progress_suite.setFormat("Suite: %p%")
		self.progress_suite.setFixedWidth(180)

		self.progress_trial = QProgressBar()
		self.progress_trial.setRange(0, 100)
		self.progress_trial.setValue(0)
		self.progress_trial.setTextVisible(True)
		self.progress_trial.setFormat("Trial: %p%")
		self.progress_trial.setFixedWidth(160)

		progress_row.addWidget(self.lbl_tune_state, 1)
		progress_row.addWidget(self.progress_trial)
		progress_row.addWidget(self.progress_suite)
		status_layout.addLayout(progress_row)

		grid_box = QGridLayout()
		grid_box.setVerticalSpacing(4)
		grid_box.setHorizontalSpacing(15)

		self.lbl_live_dist = QLabel("-- m")
		self.lbl_live_closing = QLabel("-- m/s")
		self.lbl_live_pitchrate = QLabel("-- °/s")
		self.lbl_live_yawrate = QLabel("-- °/s")
		self.lbl_live_alt_err = QLabel("-- m")
		self.lbl_live_min_max = QLabel("-- / -- m")

		metrics = [
			("Horizontal Standoff:", self.lbl_live_dist, 0, 0),
			("Closing Speed:", self.lbl_live_closing, 0, 2),
			("Pitch Rate (θ̇):", self.lbl_live_pitchrate, 0, 4),
			("Altitude Offset (Δz):", self.lbl_live_alt_err, 1, 0),
			("Min / Max Standoff:", self.lbl_live_min_max, 1, 2),
			("Yaw Rate (ψ̇):", self.lbl_live_yawrate, 1, 4),
		]
		for title, val_lbl, r, c in metrics:
			lbl_title = QLabel(title)
			lbl_title.setStyleSheet("color: #7f8c8d; font-size: 11px;")
			val_lbl.setStyleSheet("font-weight: bold; font-size: 13px;")
			grid_box.addWidget(lbl_title, r, c)
			grid_box.addWidget(val_lbl, r, c + 1)

		status_layout.addLayout(grid_box)
		layout.addWidget(status_group)

		# 3. Sweep Results & Leaderboard Table
		res_group = QGroupBox("Sweep Results & Leaderboard")
		res_layout = QVBoxLayout(res_group)
		res_layout.setContentsMargins(6, 6, 6, 6)

		headers = [
			"#",
			"Candidate",
			"Parameters Tested",
			"Mean Err",
			"Min Dist",
			"Pitch RMS",
			"Score (J)",
			"Rating",
		]
		self.table_tune_results = QTableWidget(0, len(headers))
		self.table_tune_results.setHorizontalHeaderLabels(headers)
		self.table_tune_results.verticalHeader().setVisible(False)
		self.table_tune_results.setAlternatingRowColors(True)
		self.table_tune_results.setSelectionBehavior(QTableWidget.SelectRows)
		self.table_tune_results.setSelectionMode(QTableWidget.SingleSelection)

		th = self.table_tune_results.horizontalHeader()
		th.setSectionResizeMode(0, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(0, 35)
		th.setSectionResizeMode(1, QHeaderView.ResizeToContents)
		th.setSectionResizeMode(2, QHeaderView.Stretch)
		th.setSectionResizeMode(3, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(3, 85)
		th.setSectionResizeMode(4, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(4, 85)
		th.setSectionResizeMode(5, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(5, 95)
		th.setSectionResizeMode(6, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(6, 80)
		th.setSectionResizeMode(7, QHeaderView.Fixed)
		self.table_tune_results.setColumnWidth(7, 160)

		res_layout.addWidget(self.table_tune_results, 1)
		layout.addWidget(res_group, 1)

	def _start_autotune(self):
		if self.mav_worker.conn is None or self.mav_worker.target_system is None:
			self.lbl_tune_state.setText("Cannot start: PX4 is not connected.")
			return

		prof_key = self.combo_tune_profile.currentData()
		self.autotune_profile_key = prof_key
		self.autotune_trial_duration = float(self.spin_trial_dur.value())

		self.autotune_candidates = []
		if prof_key == "full":
			for k in ("forward", "lateral", "altitude"):
				p_info = AUTOTUNE_PROFILES[k]
				for c in p_info["candidates"]:
					entry = copy.deepcopy(c)
					entry["domain"] = p_info["domain"]
					self.autotune_candidates.append(entry)
		else:
			p_info = AUTOTUNE_PROFILES[prof_key]
			for c in p_info["candidates"]:
				entry = copy.deepcopy(c)
				entry["domain"] = p_info["domain"]
				self.autotune_candidates.append(entry)

		if not self.autotune_candidates:
			self.lbl_tune_state.setText("No candidates found for selected profile.")
			return

		# Populate candidate 0 with current PX4 parameter values
		first_cand = self.autotune_candidates[0]
		for p_name in first_cand["params"]:
			if p_name in self.px4_param_values:
				first_cand["params"][p_name] = self.px4_param_values[p_name]

		# Backup initial parameters
		self.initial_params_backup = {}
		for cand in self.autotune_candidates:
			for p_name in cand["params"]:
				if p_name not in self.initial_params_backup:
					self.initial_params_backup[p_name] = self.px4_param_values.get(p_name)

		self.autotune_results = []
		self.best_trial_idx = -1
		self.table_tune_results.setRowCount(0)
		self.progress_suite.setValue(0)
		self.progress_trial.setValue(0)
		self.autotune_candidate_idx = 0

		self.btn_start_tune.setEnabled(False)
		self.btn_stop_tune.setEnabled(True)
		self.btn_apply_best.setEnabled(False)
		self.combo_tune_profile.setEnabled(False)
		self.spin_trial_dur.setEnabled(False)

		self._prepare_candidate(0)
		self.autotune_timer.start(100)

	def _stop_autotune(self):
		self.autotune_timer.stop()
		self.autotune_state = "IDLE"
		self.send_cmd({"mode": "straight", "speed": 25.0, "alt": 100.0})

		self.btn_start_tune.setEnabled(True)
		self.btn_stop_tune.setEnabled(False)
		self.btn_apply_best.setEnabled(len(self.autotune_results) > 0)
		self.combo_tune_profile.setEnabled(True)
		self.spin_trial_dur.setEnabled(True)
		self.progress_trial.setValue(0)
		self.lbl_tune_state.setText("Auto-Tune stopped by user.")

	def _prepare_candidate(self, idx):
		cand = self.autotune_candidates[idx]
		self.autotune_active_candidate = cand

		for p_name, val in cand["params"].items():
			meta = self.param_metadata.get(p_name, {})
			is_bool = (meta.get("type") == "boolean")
			self.mav_worker.set_param(p_name, val, is_int=is_bool)

		self.autotune_state = "APPLYING"
		self.autotune_subphase = None
		self.autotune_phase_start = time.monotonic()
		self.lbl_tune_state.setText(
			f"Trial {idx + 1}/{len(self.autotune_candidates)}: Applying '{cand['name']}' to PX4..."
		)

	def _compute_relative_distance(self):
		if not self.last_target_telem or not self.last_drone_telem:
			return None, None
		t_lat = self.last_target_telem.get("lat")
		t_lon = self.last_target_telem.get("lon")
		t_alt = self.last_target_telem.get("alt_rel_m")
		i_lat = self.last_drone_telem.get("lat")
		i_lon = self.last_drone_telem.get("lon")
		i_alt = self.last_drone_telem.get("alt_rel_m")
		if t_lat is None or t_lon is None or i_lat is None or i_lon is None:
			return None, None
		dlat = math.radians(t_lat - i_lat)
		dlon = math.radians(t_lon - i_lon)
		lat_mid = math.radians((t_lat + i_lat) / 2.0)
		dx = dlon * math.cos(lat_mid) * 6371000.0
		dy = dlat * 6371000.0
		dist_hor = math.hypot(dx, dy)
		dz = (i_alt - t_alt) if (t_alt is not None and i_alt is not None) else 0.0
		return dist_hor, dz

	def _on_autotune_tick(self):
		now = time.monotonic()
		dist_hor, dz = self._compute_relative_distance()

		# Update live metrics
		if dist_hor is not None:
			self.lbl_live_dist.setText(f"{dist_hor:.2f} m")
		i_spd = self.last_drone_telem.get("ground_speed_mps", 0.0)
		t_spd = self.last_target_telem.get("ground_speed_mps", 0.0)
		cl_spd = i_spd - t_spd
		self.lbl_live_closing.setText(f"{cl_spd:+.2f} m/s")

		pr = abs(self.last_drone_telem.get("pitchspeed_dps", 0.0))
		yr = abs(self.last_drone_telem.get("yawspeed_dps", 0.0))
		self.lbl_live_pitchrate.setText(f"{pr:.2f} °/s")
		self.lbl_live_yawrate.setText(f"{yr:.2f} °/s")
		if dz is not None:
			self.lbl_live_alt_err.setText(f"{dz:+.2f} m")

		if self.autotune_state == "IDLE":
			return

		# Failsafe check: if drone disarmed, abort
		if not self.drone_armed:
			self._stop_autotune()
			self.lbl_tune_state.setText("Aborted: Drone is disarmed.")
			return

		cand = self.autotune_active_candidate
		idx = self.autotune_candidate_idx
		tot = len(self.autotune_candidates)

		if self.autotune_state == "APPLYING":
			if now - self.autotune_phase_start >= 1.5:
				self.autotune_state = "STABILIZING"
				self.autotune_phase_start = now
				self.send_cmd({"mode": "straight", "speed": 25.0, "alt": 100.0})
				self.lbl_tune_state.setText(f"Trial {idx + 1}/{tot}: Stabilizing behind target (25 m/s)...")

		elif self.autotune_state == "STABILIZING":
			if now - self.autotune_phase_start >= 2.0:
				self.autotune_state = "TESTING"
				self.autotune_trial_start_time = now
				self.autotune_samples = {
					"dist_err": [],
					"dist_hor": [],
					"pitchspeed": [],
					"yawspeed": [],
					"alt_err": [],
				}

		elif self.autotune_state == "TESTING":
			t_test = now - self.autotune_trial_start_time
			dur = self.autotune_trial_duration
			frac = min(1.0, t_test / dur)
			self.progress_trial.setValue(int(frac * 100))

			domain = cand.get("domain", "forward")
			desc = ""
			subphase = ""
			cmd = None

			if domain == "forward":
				if t_test < 0.45 * dur:
					subphase = "fwd_decel"
					desc = "Decelerating Target (21 m/s)"
					cmd = {"speed": 21.0}
				elif t_test < 0.85 * dur:
					subphase = "fwd_accel"
					desc = "Accelerating Target (28 m/s)"
					cmd = {"speed": 28.0}
				else:
					subphase = "fwd_cruise"
					desc = "Settling to Cruise (25 m/s)"
					cmd = {"speed": 25.0}
			elif domain == "lateral":
				if t_test < 0.80 * dur:
					subphase = "lat_drift"
					desc = "Lateral Drift ±1.5m (25 m/s)"
					cmd = {"mode": "s_turn", "wave_amp": 1.5, "wave_len": 120.0, "speed": 25.0}
				else:
					subphase = "lat_straight"
					desc = "Aligning Straight (25 m/s)"
					cmd = {"mode": "straight", "speed": 25.0}
			elif domain == "altitude":
				if t_test < 0.50 * dur:
					subphase = "alt_climb"
					desc = "Target Climbing (108 m)"
					cmd = {"alt": 108.0}
				else:
					subphase = "alt_descend"
					desc = "Target Descending (100 m)"
					cmd = {"alt": 100.0}

			if getattr(self, "autotune_subphase", None) != subphase:
				self.autotune_subphase = subphase
				if cmd:
					self.send_cmd(cmd)

			self.lbl_tune_state.setText(f"Trial {idx + 1}/{tot}: {desc} ({t_test:.1f}s / {dur:.0f}s)")

			# Record metrics
			if dist_hor is not None:
				standoff = float(self.px4_param_values.get("INT_STANDOFF", 3.0))
				self.autotune_samples["dist_err"].append(abs(dist_hor - standoff))
				self.autotune_samples["dist_hor"].append(dist_hor)
			self.autotune_samples["pitchspeed"].append(pr)
			self.autotune_samples["yawspeed"].append(yr)
			if dz is not None:
				self.autotune_samples["alt_err"].append(abs(dz))

			if self.autotune_samples["dist_hor"]:
				mn = min(self.autotune_samples["dist_hor"])
				mx = max(self.autotune_samples["dist_hor"])
				self.lbl_live_min_max.setText(f"{mn:.2f} / {mx:.2f} m")

			if t_test >= dur:
				self._evaluate_trial()
				self.autotune_candidate_idx += 1
				suite_pct = int((self.autotune_candidate_idx / tot) * 100)
				self.progress_suite.setValue(suite_pct)

				if self.autotune_candidate_idx < tot:
					self._prepare_candidate(self.autotune_candidate_idx)
				else:
					self._finish_autotune()

	def _evaluate_trial(self):
		cand = self.autotune_active_candidate
		idx = self.autotune_candidate_idx
		domain = cand.get("domain", "forward")

		dist_err_list = self.autotune_samples.get("dist_err", [])
		dist_hor_list = self.autotune_samples.get("dist_hor", [])
		pitch_list = self.autotune_samples.get("pitchspeed", [])
		yaw_list = self.autotune_samples.get("yawspeed", [])
		alt_err_list = self.autotune_samples.get("alt_err", [])

		mean_err = (sum(dist_err_list) / len(dist_err_list)) if dist_err_list else 9.99
		min_dist = min(dist_hor_list) if dist_hor_list else 0.0
		max_dist = max(dist_hor_list) if dist_hor_list else 99.0
		pitch_rms = (
			math.sqrt(sum(x * x for x in pitch_list) / len(pitch_list))
			if pitch_list
			else 0.0
		)
		yaw_rms = (
			math.sqrt(sum(x * x for x in yaw_list) / len(yaw_list))
			if yaw_list
			else 0.0
		)
		alt_rms = (
			math.sqrt(sum(x * x for x in alt_err_list) / len(alt_err_list))
			if alt_err_list
			else 0.0
		)

		if domain == "forward":
			pen = 0.0
			if min_dist < 1.8:
				pen += (1.8 - min_dist) * 200.0
			if max_dist > 15.0:
				pen += (max_dist - 15.0) * 15.0
			score = 2.0 * mean_err + 1.2 * pitch_rms + pen

			if min_dist < 1.5:
				rating = "COLLISION RISK"
			elif max_dist > 20.0:
				rating = "LOST CONTACT"
			elif score < 2.5:
				rating = "OPTIMAL"
			elif score < 4.5:
				rating = "STABLE"
			elif score < 6.5:
				rating = "ACCEPTABLE"
			else:
				rating = "OSCILLATORY"

		elif domain == "lateral":
			# Lateral tuning focuses on yaw stability, smooth tracking, and staying locked behind target
			score = 1.0 * yaw_rms + 1.2 * mean_err + 0.4 * pitch_rms

			if max_dist > 20.0:
				rating = "LOST CONTACT"
			elif yaw_rms > 12.0:
				rating = "YAW JITTER"
			elif score < 3.5:
				rating = "OPTIMAL"
			elif score < 6.0:
				rating = "STABLE"
			elif score < 9.0:
				rating = "ACCEPTABLE"
			else:
				rating = "SLUGGISH"

		elif domain == "altitude":
			score = 3.0 * alt_rms + 1.0 * pitch_rms

			if max_dist > 20.0:
				rating = "LOST CONTACT"
			elif alt_rms > 3.0:
				rating = "ALT ERROR"
			elif score < 2.5:
				rating = "OPTIMAL"
			elif score < 4.5:
				rating = "STABLE"
			else:
				rating = "ACCEPTABLE"
		else:
			score = 2.0 * mean_err + 1.2 * pitch_rms
			rating = "ACCEPTABLE"

		result = {
			"trial_num": idx + 1,
			"name": cand["name"],
			"domain": domain,
			"params": dict(cand["params"]),
			"mean_err": mean_err,
			"min_dist": min_dist,
			"max_dist": max_dist,
			"pitch_rms": pitch_rms,
			"yaw_rms": yaw_rms,
			"alt_rms": alt_rms,
			"score": score,
			"rating": rating,
		}
		self.autotune_results.append(result)

		param_str = ", ".join(f"{k.replace('INT_TRM_', '')}={v:.2f}" for k, v in cand["params"].items())

		row = self.table_tune_results.rowCount()
		self.table_tune_results.insertRow(row)

		item_num = QTableWidgetItem(str(idx + 1))
		item_num.setTextAlignment(Qt.AlignCenter)
		self.table_tune_results.setItem(row, 0, item_num)

		item_cand = QTableWidgetItem(cand["name"])
		self.table_tune_results.setItem(row, 1, item_cand)

		item_params = QTableWidgetItem(param_str)
		item_params.setToolTip(param_str)
		self.table_tune_results.setItem(row, 2, item_params)

		item_err = QTableWidgetItem(f"{mean_err:.2f} m")
		item_err.setTextAlignment(Qt.AlignCenter)
		self.table_tune_results.setItem(row, 3, item_err)

		item_min = QTableWidgetItem(f"{min_dist:.2f} m")
		item_min.setTextAlignment(Qt.AlignCenter)
		self.table_tune_results.setItem(row, 4, item_min)

		item_pitch = QTableWidgetItem(f"{pitch_rms:.2f} °/s")
		item_pitch.setTextAlignment(Qt.AlignCenter)
		self.table_tune_results.setItem(row, 5, item_pitch)

		item_score = QTableWidgetItem(f"{score:.2f}")
		item_score.setTextAlignment(Qt.AlignCenter)
		font = item_score.font()
		font.setBold(True)
		item_score.setFont(font)
		self.table_tune_results.setItem(row, 6, item_score)

		item_rat = QTableWidgetItem(rating)
		item_rat.setTextAlignment(Qt.AlignCenter)
		self.table_tune_results.setItem(row, 7, item_rat)

		self._update_leaderboard()

	def _update_leaderboard(self):
		if not self.autotune_results:
			return

		best_idx = -1
		min_score = 99999.0
		for i, res in enumerate(self.autotune_results):
			if res["min_dist"] >= 1.8 and res["score"] < min_score:
				min_score = res["score"]
				best_idx = i

		self.best_trial_idx = best_idx

		for r in range(self.table_tune_results.rowCount()):
			res = self.autotune_results[r]
			is_best = (r == best_idx)
			rating_str = res["rating"]

			item_rat = self.table_tune_results.item(r, 7)
			if item_rat:
				font = item_rat.font()
				font.setBold(True)
				item_rat.setFont(font)

				if is_best:
					item_rat.setText(f"BEST ★ ({rating_str})")
					item_rat.setForeground(QColor("#2ecc71"))
				elif rating_str == "COLLISION RISK" or res["min_dist"] < 1.8:
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#e74c3c"))
				elif rating_str == "LOST CONTACT":
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#e67e22"))
				elif rating_str == "OPTIMAL":
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#2ecc71"))
				elif rating_str == "STABLE":
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#3498db"))
				elif rating_str == "ACCEPTABLE":
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#f39c12"))
				else:
					item_rat.setText(rating_str)
					item_rat.setForeground(QColor("#e67e22"))

		self.btn_apply_best.setEnabled(best_idx >= 0)

	def _finish_autotune(self):
		self.autotune_state = "IDLE"
		self.autotune_timer.stop()
		self.send_cmd({"mode": "straight", "speed": 25.0, "alt": 100.0})

		self.btn_start_tune.setEnabled(True)
		self.btn_stop_tune.setEnabled(False)
		self.btn_apply_best.setEnabled(self.best_trial_idx >= 0)
		self.combo_tune_profile.setEnabled(True)
		self.spin_trial_dur.setEnabled(True)
		self.progress_trial.setValue(100)

		if self.best_trial_idx >= 0:
			best = self.autotune_results[self.best_trial_idx]
			self.lbl_tune_state.setText(
				f"Sweep Complete! Best: Trial #{best['trial_num']} ({best['name']}) [Score: {best['score']:.2f}]. Click 'Apply Best' to save."
			)
		else:
			self.lbl_tune_state.setText("Sweep Complete. No collision-free candidate found.")

	def _apply_best_params(self):
		if self.best_trial_idx < 0 or self.best_trial_idx >= len(self.autotune_results):
			return
		best = self.autotune_results[self.best_trial_idx]

		for p_name, val in best["params"].items():
			meta = self.param_metadata.get(p_name, {})
			is_bool = (meta.get("type") == "boolean")
			self.mav_worker.set_param(p_name, val, is_int=is_bool)

			if p_name in self.param_editors:
				editor = self.param_editors[p_name]
				if is_bool:
					editor.setCurrentIndex(1 if int(val) == 1 else 0)
				else:
					editor.setValue(float(val))

		self.mav_worker.save_params_to_storage()
		self.lbl_tune_state.setText(
			f"Applied Trial #{best['trial_num']} ({best['name']}) parameters to PX4 and saved to persistent storage!"
		)

	def _on_tab_changed(self, index):
		# Auto-request params when switching to Tab 2 (Params) or Tab 3 (Auto-Tune) if connected and not yet requested
		if index in (1, 2) and not self._initial_params_requested and self.mav_worker.target_system is not None:
			self._initial_params_requested = True
			self._on_refresh_params()

	def _on_filter_changed(self, text):
		pattern = text.strip().lower()
		for row in range(self.table_params.rowCount()):
			name_item = self.table_params.item(row, 0)
			if not name_item:
				continue
			name = name_item.text()
			meta = self.param_metadata.get(name, {})
			short_desc = meta.get("short", "").lower()
			long_desc = meta.get("long", "").lower()
			name_lower = name.lower()

			match = (pattern in name_lower) or (pattern in short_desc) or (pattern in long_desc)
			self.table_params.setRowHidden(row, not match)

	def _on_refresh_params(self):
		if self.mav_worker.conn is None or self.mav_worker.target_system is None:
			self.lbl_param_status.setText("PX4 is not connected yet.")
			return
		self.lbl_param_status.setText("Requesting parameters from PX4...")
		for name in self.param_metadata:
			self.mav_worker.request_param(name)

	def _on_save_all_params(self):
		if self.mav_worker.conn is None or self.mav_worker.target_system is None:
			self.lbl_param_status.setText("Error: PX4 is not connected, unable to save.")
			return
		count = 0
		for name, meta in self.param_metadata.items():
			if name not in self.param_editors:
				continue
			is_bool = (meta.get("type") == "boolean")
			editor = self.param_editors[name]
			if is_bool:
				val = 1.0 if editor.currentIndex() == 1 else 0.0
			else:
				val = float(editor.value())
			self.mav_worker.set_param(name, val, is_int=is_bool)
			count += 1
		self.mav_worker.save_params_to_storage()
		self.lbl_param_status.setText(f"{count} parameters sent to PX4 and saved.")

	def _on_reset_defaults(self):
		for name, meta in self.param_metadata.items():
			if name not in self.param_editors:
				continue
			default_val = meta.get("default", 0)
			is_bool = (meta.get("type") == "boolean")
			editor = self.param_editors[name]
			if is_bool:
				editor.setCurrentIndex(1 if int(default_val) == 1 else 0)
			else:
				editor.setValue(float(default_val))
		self.lbl_param_status.setText("Values reset to defaults. Click 'Save' to apply to PX4.")

	def on_param_received(self, name, val, ptype):
		if name not in self.param_metadata:
			return
		self.px4_param_values[name] = val
		meta = self.param_metadata[name]
		is_bool = (meta.get("type") == "boolean")

		# Update the PX4 Value column
		if name in self.param_px4_items:
			item = self.param_px4_items[name]
			if is_bool:
				text = "1 (Enabled)" if int(val) == 1 else "0 (Disabled)"
			else:
				dec = meta.get("decimal", 2)
				text = f"{val:.{dec}f}"
			item.setText(text)
			item.setForeground(QColor("#27ae60"))  # Green text indicating confirmed from PX4

		# Update the editor widget value if not actively focused
		if name in self.param_editors:
			editor = self.param_editors[name]
			if not editor.hasFocus():
				if is_bool:
					editor.blockSignals(True)
					editor.setCurrentIndex(1 if int(val) == 1 else 0)
					editor.blockSignals(False)
				else:
					editor.blockSignals(True)
					editor.setValue(val)
					editor.blockSignals(False)

		if hasattr(self, "lbl_param_status"):
			self.lbl_param_status.setText(f"'{name}' = {val} updated from PX4.")

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
		self.last_drone_telem = data
		connected = data.get("connected", False)
		self.drone_tel["link"].setText("OK" if connected else "Lost")

		# On initial PX4 connection, automatically fetch intercept parameters
		if connected and not self._initial_params_requested:
			self._initial_params_requested = True
			QTimer.singleShot(600, self._on_refresh_params)

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
		if hasattr(self, "lbl_param_status"):
			upper = text.upper()
			if any(k in upper for k in ("PARAM", "STORAGE", "SAVED", "REJECT", "INT_")):
				self.lbl_param_status.setText(f"PX4: {text}")

	def closeEvent(self, event):
		self.autotune_timer.stop()
		self.mav_worker.stop()
		self.mav_thread.quit()
		self.mav_thread.wait(1000)
		super().closeEvent(event)

	def _setup_shortcuts(self):
		QShortcut(QKeySequence("1"), self, lambda: self._shortcut_cmd({"mode": "straight"}))
		QShortcut(QKeySequence("2"), self, lambda: self._shortcut_cmd({"mode": "s_turn"}))
		QShortcut(QKeySequence("3"), self, lambda: self._shortcut_cmd({"mode": "orbit"}))
		QShortcut(QKeySequence("W"), self, lambda: self._shortcut_speed(2.0))
		QShortcut(QKeySequence("S"), self, lambda: self._shortcut_speed(-2.0))

		# Arrow keys act like the D-pad (hold to nudge); filtered app-wide
		self.arrow_keys = {Qt.Key_Up: "up", Qt.Key_Down: "down", Qt.Key_Left: "left", Qt.Key_Right: "right"}
		QApplication.instance().installEventFilter(self)

	def _should_ignore_flight_shortcuts(self):
		"""Disable flight shortcuts when navigating the parameter screen or typing in inputs."""
		if hasattr(self, "tabs") and self.tabs.currentIndex() != 0:
			return True
		focus = QApplication.focusWidget()
		if isinstance(focus, (QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox)):
			return True
		return False

	def _shortcut_cmd(self, cmd_dict):
		if not self._should_ignore_flight_shortcuts():
			self.send_cmd(cmd_dict)

	def _shortcut_speed(self, delta):
		if not self._should_ignore_flight_shortcuts():
			self._nudge_speed(delta)

	def eventFilter(self, obj, event):
		if event.type() in (QEvent.KeyPress, QEvent.KeyRelease) and event.key() in self.arrow_keys:
			# Pass arrow keys to Qt table / spinbox widgets when on parameters tab
			if self._should_ignore_flight_shortcuts():
				return super().eventFilter(obj, event)
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
		self.last_target_telem = data

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

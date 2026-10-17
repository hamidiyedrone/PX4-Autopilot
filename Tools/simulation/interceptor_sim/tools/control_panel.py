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

Tab 3: Net
- Net and launcher parameters of tools/net_launcher.py (HTTP :8003): applied from the next
  shot, saved to the net: section of interceptor.yaml on request
- The net in the air (phase, distance, spread, speed), Fire / Clear

Tab 4: Wind & Turbulence
- Real-time wind velocity and turbulence simulation settings (Gazebo transport & simulated wind).

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
import threading
import time
import urllib.error
import urllib.request
from urllib.parse import urlencode

import numpy as np
import yaml

try:
	from gz.transport13 import Node as GzNode
	from gz.msgs10.wind_pb2 import Wind as GzWind
	GZ_TRANSPORT_AVAILABLE = True
except ImportError:
	GzNode = None
	GzWind = None
	GZ_TRANSPORT_AVAILABLE = False


def get_compass_direction(deg):
	deg = float(deg) % 360.0
	dirs = ["K (N)", "KKB (NNE)", "KD (NE)", "DKD (ENE)", "D (E)", "DGD (ESE)", "GD (SE)", "GGD (SSE)",
	        "G (S)", "GGB (SSW)", "GB (SW)", "BGB (WSW)", "B (W)", "BKB (WNW)", "KB (NW)", "KKB (NNW)"]
	ix = int((deg + 11.25) / 22.5) % 16
	return dirs[ix]


os.environ.setdefault("MAVLINK20", "1")  # AVAILABLE_MODES is a MAVLink 2 message
from pymavlink import mavutil  # noqa: E402
from PySide6.QtCore import QEvent, QObject, QSize, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QIcon, QKeySequence, QShortcut, QTransform
from PySide6.QtWidgets import (
	QApplication,
	QCheckBox,
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
				"name": "INT_STANDOFF", "type": "float", "default": 4.0, "min": 1.0, "max": 10.0, "decimal": 1,
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
			"INT_APP_DIST_BHD": {
				"name": "INT_APP_DIST_BHD", "type": "float", "default": 30.0, "min": 5.0, "max": 100.0, "decimal": 1,
				"short": "Approach aim point distance behind target",
				"long": "Distance in meters behind target used as intermediate aim waypoint during approach phase."
			},
			"INT_APP_MIN_SPD": {
				"name": "INT_APP_MIN_SPD", "type": "float", "default": 15.0, "min": 5.0, "max": 40.0, "decimal": 1,
				"short": "Approach phase minimum forward speed",
				"long": "Minimum commanded forward speed (m/s) maintained during approach guidance."
			},
			"INT_TRM_KP_FWD": {
				"name": "INT_TRM_KP_FWD", "type": "float", "default": 14.0, "min": 0.1, "max": 15.0, "decimal": 2,
				"short": "Terminal standoff distance error velocity gain",
				"long": "Proportional speed gain commanding forward pursuit speed offset from horizontal distance error."
			},
			"INT_TRM_KD_FWD": {
				"name": "INT_TRM_KD_FWD", "type": "float", "default": 5.0, "min": 0.1, "max": 10.0, "decimal": 2,
				"short": "Terminal relative closing speed damping gain",
				"long": "Derivative damping gain on optical relative closing speed (m/s) matching target velocity."
			},
			"INT_TRM_FWD_TOL": {
				"name": "INT_TRM_FWD_TOL", "type": "float", "default": 0.50, "min": 0.05, "max": 1.0, "decimal": 2,
				"short": "Terminal standoff distance deadband tolerance",
				"long": "Distance deadband tolerance (+/- meters) around desired standoff in close pursuit to prevent pitch oscillations."
			},
			"INT_TRM_MAX_APPR": {
				"name": "INT_TRM_MAX_APPR", "type": "float", "default": 4.0, "min": 1.0, "max": 15.0, "decimal": 1,
				"short": "Terminal maximum pursuit approach speed",
				"long": "Maximum forward overtaking speed (m/s) added above target speed when closing distance from behind in close pursuit."
			},
			"INT_TRM_YAW_TOL": {
				"name": "INT_TRM_YAW_TOL", "type": "float", "default": 0.005, "min": 0.005, "max": 0.15, "decimal": 3,
				"short": "Terminal yaw optical tracking deadband tolerance",
				"long": "Optical screen error tolerance (+/- from center 0.50) for yaw tracking to suppress chattering and fishtailing."
			},
			"INT_TRM_YAW_SMTH": {
				"name": "INT_TRM_YAW_SMTH", "type": "float", "default": 0.35, "min": 0.05, "max": 1.0, "decimal": 2,
				"short": "Terminal yaw rate centering smoothing factor",
				"long": "Minimum yaw rate scale factor [0.05, 1.0] applied when target is near optical boresight center to prevent overshooting."
			},
			"INT_CAM_HFOV": {
				"name": "INT_CAM_HFOV", "type": "float", "default": 80.0, "min": 20.0, "max": 150.0, "decimal": 1,
				"short": "Camera horizontal field of view",
				"long": "Horizontal field of view in degrees of the tracking camera mounted on the vehicle."
			},
			"INT_CAM_VFOV": {
				"name": "INT_CAM_VFOV", "type": "float", "default": 60.0, "min": 15.0, "max": 120.0, "decimal": 1,
				"short": "Camera vertical field of view",
				"long": "Vertical field of view in degrees of the tracking camera mounted on the vehicle."
			},
			"INT_TRM_BRK_LAT": {
				"name": "INT_TRM_BRK_LAT", "type": "float", "default": 2.0, "min": 0.0, "max": 10.0, "decimal": 1,
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
				"name": "INT_TRM_YAWRATE", "type": "float", "default": 24.0, "min": 5.0, "max": 60.0, "decimal": 1,
				"short": "Terminal maximum heading slew rate",
				"long": "Maximum heading change rate (deg/s) in terminal close pursuit to eliminate yaw hunting."
			},
			"INT_TRM_TC_GAIN": {
				"name": "INT_TRM_TC_GAIN", "type": "float", "default": 1.0, "min": 0.0, "max": 1.5, "decimal": 2,
				"short": "Turn coordinator roll feedback gain",
				"long": "Feeds back roll angle into yaw guidance to assist bank-to-turn entry and actively drive steady-state roll back to zero."
			},
			"INT_GMB_SERVO": {
				"name": "INT_GMB_SERVO", "type": "int32", "default": 1, "min": 0, "max": 8, "decimal": 0,
				"short": "Gimbal pitch servo output index (1-8, 0=disabled)", "long": "Actuator servo channel for camera gimbal elevation."
			},
			"INT_TRT_SERVO": {
				"name": "INT_TRT_SERVO", "type": "int32", "default": 2, "min": 0, "max": 8, "decimal": 0,
				"short": "Turret pitch servo output index (1-8, 0=disabled)", "long": "Actuator servo channel for net gun turret elevation."
			},
			"INT_TRG_SERVO": {
				"name": "INT_TRG_SERVO", "type": "int32", "default": 3, "min": 0, "max": 8, "decimal": 0,
				"short": "Net trigger servo output index (1-8, 0=disabled)", "long": "Actuator servo channel for firing net gun."
			},
			"INT_TURRET_PITCH": {
				"name": "INT_TURRET_PITCH", "type": "float", "default": 0.0, "min": 0.0, "max": 45.0, "decimal": 1,
				"short": "Net gun turret pitch tilt angle (deg)", "long": "Manual turret pitch angle."
			},
			"INT_TRT_MIN_P": {
				"name": "INT_TRT_MIN_P", "type": "float", "default": 0.0, "min": -30.0, "max": 90.0, "decimal": 1,
				"short": "Turret minimum pitch angle (deg)", "long": "Minimum allowable turret pitch elevation."
			},
			"INT_TRT_MAX_P": {
				"name": "INT_TRT_MAX_P", "type": "float", "default": 45.0, "min": 0.0, "max": 90.0, "decimal": 1,
				"short": "Turret maximum pitch angle (deg)", "long": "Maximum allowable turret pitch elevation."
			},
			"INT_GMB_MIN_P": {
				"name": "INT_GMB_MIN_P", "type": "float", "default": -60.0, "min": -90.0, "max": 0.0, "decimal": 1,
				"short": "Gimbal minimum pitch angle (deg)", "long": "Minimum allowable seeker gimbal elevation."
			},
			"INT_GMB_MAX_P": {
				"name": "INT_GMB_MAX_P", "type": "float", "default": 60.0, "min": 0.0, "max": 90.0, "decimal": 1,
				"short": "Gimbal maximum pitch angle (deg)", "long": "Maximum allowable seeker gimbal elevation."
			},
			"INT_GMB_KP": {
				"name": "INT_GMB_KP", "type": "float", "default": 60.0, "min": 0.0, "max": 200.0, "decimal": 1,
				"short": "Gimbal PI proportional gain (Kp)", "long": "Gimbal pitch optical tracking proportional gain."
			},
			"INT_GMB_KI": {
				"name": "INT_GMB_KI", "type": "float", "default": 10.0, "min": 0.0, "max": 50.0, "decimal": 1,
				"short": "Gimbal PI integral gain (Ki)", "long": "Gimbal pitch optical tracking integral gain."
			},
			"INT_TRT_MODE": {
				"name": "INT_TRT_MODE", "type": "int32", "default": 0, "min": 0, "max": 1, "decimal": 0,
				"short": "Turret control mode (0=Manual, 1=Auto-Aim)", "long": "Turret auto-aim (gimbal + offset)."
			},
			"INT_TRT_OFFSET": {
				"name": "INT_TRT_OFFSET", "type": "float", "default": -2.0, "min": -45.0, "max": 45.0, "decimal": 1,
				"short": "Turret gimbal offset (+/- deg)", "long": "Angular offset added directly to gimbal pitch for turret aim."
			},
			"INT_AUTO_FIRE": {
				"name": "INT_AUTO_FIRE", "type": "int32", "default": 0, "min": 0, "max": 1, "decimal": 0,
				"short": "Auto-fire net when in range (0=Off, 1=On)", "long": "Autonomous firing on target intercept."
			},
			"INT_FIRE_DIST": {
				"name": "INT_FIRE_DIST", "type": "float", "default": 20.0, "min": 5.0, "max": 40.0, "decimal": 1,
				"short": "Auto-fire effective distance threshold (m)", "long": "Distance threshold for auto firing net."
			},
			"INT_FIRE_TOL": {
				"name": "INT_FIRE_TOL", "type": "float", "default": 3.0, "min": 0.5, "max": 15.0, "decimal": 1,
				"short": "Auto-fire angular aim tolerance (deg)", "long": "Boresight angle tolerance for auto fire."
			},
			"INT_TRG_PULSE": {
				"name": "INT_TRG_PULSE", "type": "float", "default": 0.5, "min": 0.1, "max": 3.0, "decimal": 2,
				"short": "Net trigger servo pulse duration (s)", "long": "Pulse time before trigger resets to safe."
			},
			"INT_TRIGGER": {
				"name": "INT_TRIGGER", "type": "int32", "default": 0, "min": 0, "max": 1, "decimal": 0,
				"short": "Manual trigger command parameter (1=Fire net)",
				"long": "Command parameter to manually pulse net trigger servo."
			},
		}
	return params





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


class DetectorWorker(QObject):
	"""Worker to query and update sim_detector noise settings asynchronously."""

	noise_received = Signal(dict)

	def __init__(self, host="127.0.0.1", port=8002):
		super().__init__()
		self.host = host
		self.port = port
		self.base_url = f"http://{host}:{port}"
		self._is_polling = False

	def poll(self):
		if self._is_polling:
			return
		self._is_polling = True

		def _worker():
			try:
				url = f"{self.base_url}/noise"
				req = urllib.request.Request(url, headers={"User-Agent": "ControlPanel/1.0"})
				with urllib.request.urlopen(req, timeout=0.3) as resp:
					if resp.status == 200:
						data = json.loads(resp.read().decode("utf-8"))
						if "noise" in data:
							self.noise_received.emit(data["noise"])
			except Exception:
				pass
			finally:
				self._is_polling = False

		threading.Thread(target=_worker, daemon=True).start()

	def send_command(self, cmd_dict):
		def _worker():
			try:
				url = f"{self.base_url}/cmd"
				post_data = json.dumps(cmd_dict).encode("utf-8")
				req = urllib.request.Request(
					url,
					data=post_data,
					headers={"Content-Type": "application/json", "User-Agent": "ControlPanel/1.0"},
					method="POST",
				)
				with urllib.request.urlopen(req, timeout=0.5) as resp:
					if resp.status == 200:
						data = json.loads(resp.read().decode("utf-8"))
						if "noise" in data:
							self.noise_received.emit(data["noise"])
			except Exception:
				pass

		threading.Thread(target=_worker, daemon=True).start()


class NetWorker(QObject):
	"""Worker to query the net launcher (tools/net_launcher.py) and change its net parameters."""

	status_received = Signal(dict)
	params_received = Signal(dict)
	message = Signal(str)

	def __init__(self, host="127.0.0.1", port=8003):
		super().__init__()
		self.base_url = f"http://{host}:{port}"
		self._is_polling = False

	def _request(self, path, body=None, timeout=0.5):
		data = None if body is None else json.dumps(body).encode("utf-8")
		req = urllib.request.Request(
			f"{self.base_url}{path}",
			data=data,
			headers={"Content-Type": "application/json", "User-Agent": "ControlPanel/1.0"},
			method="GET" if body is None else "POST",
		)
		with urllib.request.urlopen(req, timeout=timeout) as resp:
			return json.loads(resp.read().decode("utf-8"))

	def poll(self):
		if self._is_polling:
			return
		self._is_polling = True

		def _worker():
			try:
				self.status_received.emit(self._request("/status", timeout=0.3))
			except Exception:
				pass
			finally:
				self._is_polling = False

		threading.Thread(target=_worker, daemon=True).start()

	def _call(self, path, body, signal, ok_text):
		def _worker():
			try:
				data = self._request(path, body, timeout=1.0)
				if signal is not None:
					signal.emit(data)
				self.message.emit(ok_text)
			except urllib.error.HTTPError as e:
				try:
					err = json.loads(e.read().decode("utf-8")).get("error", str(e))
				except Exception:
					err = str(e)
				self.message.emit(f"Error: {err}")
			except Exception as e:
				self.message.emit(f"Net launcher not reachable ({e})")

		threading.Thread(target=_worker, daemon=True).start()

	def fetch_params(self):
		self._call("/params", None, self.params_received, "Parameters read from the launcher.")

	def send_params(self, values):
		self._call("/params", values, self.params_received, "Applied: used from the next shot.")

	def save_params(self):
		self._call("/params/save", {}, None, "Saved to interceptor.yaml (net: section).")


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

	def __init__(self, host="127.0.0.1", port=8000, mavlink_url="udpin:0.0.0.0:14550", detector_port=8002, world="ankara"):
		super().__init__()
		self.host = host
		self.port = port
		self.detector_port = detector_port
		self.world_name = world
		self.current_mode = "straight"
		self.target_speed = 25.0
		self.target_alt = 100.0
		self.target_heading = 0.0

		self.setWindowTitle("Control Panel")
		self.resize(980, 680)
		self.setMinimumSize(850, 560)

		# Parameter management state
		self.param_metadata = load_intercept_param_metadata()
		self.px4_param_values = {}
		self.param_editors = {}
		self.param_px4_items = {}
		self._initial_params_requested = False

		self.worker = TelemetryWorker(host, port)
		self.detector_worker = DetectorWorker(host, detector_port)
		self.poll_timer = QTimer(self)
		self.poll_timer.timeout.connect(self.worker.poll)
		self.poll_timer.timeout.connect(self.detector_worker.poll)
		self.worker.telemetry_received.connect(self.on_telemetry)
		self.detector_worker.noise_received.connect(self.on_detector_noise)

		# Net launcher (tools/net_launcher.py): net parameters and the net in the air
		self.net_worker = NetWorker(host, 8003)
		self.net_params_loaded = False
		self.poll_timer.timeout.connect(self.net_worker.poll)
		self.net_worker.status_received.connect(self.on_net_status)
		self.net_worker.params_received.connect(self.on_net_params)
		self.net_worker.message.connect(lambda text: self.lbl_net_status.setText(text))

		# Directions currently held via D-pad buttons or arrow keys
		self.held = set()
		self.hold_timer = QTimer(self)
		self.hold_timer.setInterval(self.HOLD_PERIOD_MS)
		self.hold_timer.timeout.connect(self._send_hold_nudge)

		# Telemetry snapshots for live relative distance & rate metrics
		self.last_target_telem = {}
		self.last_drone_telem = {}


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

		# Wind & Turbulence Simulation State
		self.wind_enabled = True
		self.wind_speed_mps = 0.0
		self.wind_direction_deg = 0.0
		self.wind_vertical_mps = 0.0
		self.wind_turbulence_pct = 0.0
		self.wind_gust_speed_mps = 3.0
		self.wind_gust_period_s = 6.0
		self._load_initial_wind_from_yaml()

		# Turbulence dynamic state
		self.turb_state = np.zeros(3, dtype=float)
		self.manual_gust_active = False
		self.manual_gust_start_time = 0.0
		self.manual_gust_duration_s = 4.0
		self.manual_gust_peak_mps = 6.0
		self.last_wind_update_time = time.time()
		self.current_wind_vector = np.zeros(3, dtype=float)
		self._last_published_wind = np.array([999.0, 999.0, 999.0])
		self._last_published_time = 0.0
		self._last_wind_badge_tier = -1
		self._wind_was_enabled = None

		# Gazebo Transport Node for Wind
		self.gz_node = None
		self.gz_pub_wind = None
		if GZ_TRANSPORT_AVAILABLE:
			try:
				self.gz_node = GzNode()
				self.gz_pub_wind = self.gz_node.advertise(f"/world/{self.world_name}/wind", GzWind)
			except Exception as e:
				print(f"ControlPanel: Gazebo wind transport warning: {e}", flush=True)

		self.wind_timer = QTimer(self)
		self.wind_timer.setInterval(100)  # 10 Hz (optimized for UI & transport efficiency)
		self.wind_timer.timeout.connect(self._on_wind_tick)
		self.wind_timer.start()

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
		self._init_vision_noise_ui(main_layout)
		main_layout.addStretch()

		self.tabs.addTab(flight_tab, "Flight Control")

		# Tab 2: Intercept Parameters
		params_tab = QWidget()
		self._init_params_ui(params_tab)
		self.tabs.addTab(params_tab, "Parameters")

		# Tab 3: Net launcher
		net_tab = QWidget()
		self._init_net_ui(net_tab)
		self.tabs.addTab(net_tab, "Net")

		# Tab 4: Wind & Turbulence (environmental conditions)
		wind_tab = QWidget()
		self._init_wind_ui(wind_tab)
		self.tabs.addTab(wind_tab, "Wind & Turbulence")

		self.tabs.currentChanged.connect(self._on_tab_changed)

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
		self._init_turret_ui(layout)
		layout.addStretch()

	def _init_turret_ui(self, parent_layout):
		turret_group = QGroupBox("Net Gun Turret")
		turret_layout = QVBoxLayout(turret_group)
		turret_layout.setSpacing(4)

		# Top row: Angle readout
		top_row = QHBoxLayout()
		top_row.addWidget(QLabel("Pitch Angle:"))
		self.lbl_turret_val = QLabel("0.0°")
		self.lbl_turret_val.setStyleSheet("font-weight: bold; font-size: 13px;")
		top_row.addWidget(self.lbl_turret_val)
		top_row.addStretch()
		turret_layout.addLayout(top_row)

		# Mode, Offset, and Auto-Fire row
		mode_row = QHBoxLayout()
		mode_row.addWidget(QLabel("Aim:"))
		self.combo_turret_mode = QComboBox()
		self.combo_turret_mode.addItems(["Manual", "Auto-Aim (Gimbal + Offset)"])
		self.combo_turret_mode.setFixedHeight(24)
		mode_row.addWidget(self.combo_turret_mode, 1)

		mode_row.addWidget(QLabel("Offset:"))
		self.spin_turret_offset = QDoubleSpinBox()
		self.spin_turret_offset.setRange(-45.0, 45.0)
		self.spin_turret_offset.setSingleStep(0.5)
		self.spin_turret_offset.setDecimals(1)
		self.spin_turret_offset.setSuffix("°")
		self.spin_turret_offset.setValue(0.0)
		self.spin_turret_offset.setFixedWidth(70)
		self.spin_turret_offset.setFixedHeight(24)
		self.spin_turret_offset.setEnabled(False)
		self.spin_turret_offset.setToolTip("Angular offset added directly to gimbal pitch for turret auto-aim (INT_TRT_OFFSET)")
		mode_row.addWidget(self.spin_turret_offset)

		self.chk_auto_fire = QCheckBox("Auto-Fire")
		self.chk_auto_fire.setToolTip("Autonomous net firing when target is locked inside effective corridor")
		mode_row.addWidget(self.chk_auto_fire)
		turret_layout.addLayout(mode_row)

		# Slider and Spinbox row
		ctrl_row = QHBoxLayout()
		self.slider_turret_pitch = QSlider(Qt.Horizontal)
		self.slider_turret_pitch.setRange(0, 450)  # 0.0 to 45.0 degrees (0.1 deg res)
		self.slider_turret_pitch.setValue(0)
		self.spin_turret_pitch = QDoubleSpinBox()
		self.spin_turret_pitch.setRange(0.0, 45.0)
		self.spin_turret_pitch.setSingleStep(1.0)
		self.spin_turret_pitch.setDecimals(1)
		self.spin_turret_pitch.setSuffix("°")
		self.spin_turret_pitch.setValue(0.0)

		ctrl_row.addWidget(self.slider_turret_pitch, 1)
		ctrl_row.addWidget(self.spin_turret_pitch)
		turret_layout.addLayout(ctrl_row)

		# Preset buttons row (0° Stowed, 15°, 30°, 45° Max)
		presets_row = QHBoxLayout()
		presets_row.setSpacing(4)
		for angle, label in [(0.0, "0° (Stowed)"), (15.0, "15°"), (30.0, "30°"), (45.0, "45° (Max)")]:
			btn = QPushButton(label)
			btn.setFixedHeight(24)
			btn.clicked.connect(lambda _, a=angle: self._set_turret_pitch(a))
			presets_row.addWidget(btn)
		turret_layout.addLayout(presets_row)

		# Fire Net button row (plain standard buttons)
		action_row = QHBoxLayout()
		self.btn_fire_net = QPushButton("Fire Net")
		self.btn_fire_net.setFixedHeight(26)
		self.btn_clear_nets = QPushButton("Clear")
		self.btn_clear_nets.setFixedWidth(60)
		self.btn_clear_nets.setFixedHeight(26)
		action_row.addWidget(self.btn_fire_net, 1)
		action_row.addWidget(self.btn_clear_nets)
		turret_layout.addLayout(action_row)

		parent_layout.addWidget(turret_group)

		# Connections
		self.combo_turret_mode.currentIndexChanged.connect(self._on_turret_mode_changed)
		self.spin_turret_offset.valueChanged.connect(self._on_turret_offset_changed)
		self.chk_auto_fire.toggled.connect(self._on_auto_fire_changed)
		self.slider_turret_pitch.valueChanged.connect(self._on_turret_slider_changed)
		self.slider_turret_pitch.sliderReleased.connect(self._on_turret_slider_released)
		self.spin_turret_pitch.valueChanged.connect(self._on_turret_spin_changed)
		self.btn_fire_net.clicked.connect(self._on_fire_net)
		self.btn_clear_nets.clicked.connect(self._on_clear_nets)

	def _on_turret_mode_changed(self, idx):
		if hasattr(self, "mav_worker") and self.mav_worker is not None:
			self.mav_worker.set_param("INT_TRT_MODE", idx, is_int=True)
		is_auto = (idx == 1)
		self.slider_turret_pitch.setEnabled(not is_auto)
		self.spin_turret_pitch.setEnabled(not is_auto)
		self.spin_turret_offset.setEnabled(is_auto)

	def _on_turret_offset_changed(self, val):
		if hasattr(self, "mav_worker") and self.mav_worker is not None:
			self.mav_worker.set_param("INT_TRT_OFFSET", float(val))

	def _on_auto_fire_changed(self, checked):
		val = 1 if checked else 0
		if hasattr(self, "mav_worker") and self.mav_worker is not None:
			self.mav_worker.set_param("INT_AUTO_FIRE", val, is_int=True)

	def _on_fire_net(self):
		if hasattr(self, "mav_worker") and self.mav_worker is not None:
			self.mav_worker.set_param("INT_TRIGGER", 1, is_int=True)

		def _worker():
			try:
				url = "http://127.0.0.1:8003/fire"
				req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
				with urllib.request.urlopen(req, timeout=1.0) as resp:
					pass
			except Exception:
				pass

		threading.Thread(target=_worker, daemon=True).start()

	def _on_clear_nets(self):
		def _worker():
			try:
				url = "http://127.0.0.1:8003/clear"
				req = urllib.request.Request(url, data=b"{}", headers={"Content-Type": "application/json"}, method="POST")
				with urllib.request.urlopen(req, timeout=1.0) as resp:
					pass
			except Exception:
				pass

		threading.Thread(target=_worker, daemon=True).start()

	def _on_turret_slider_changed(self, val):
		deg = val / 10.0
		self.lbl_turret_val.setText(f"{deg:.1f}°")
		if abs(self.spin_turret_pitch.value() - deg) > 0.05:
			self.spin_turret_pitch.blockSignals(True)
			self.spin_turret_pitch.setValue(deg)
			self.spin_turret_pitch.blockSignals(False)
		self._send_turret_pitch_to_px4(deg)

	def _on_turret_slider_released(self):
		deg = self.slider_turret_pitch.value() / 10.0
		self._send_turret_pitch_to_px4(deg)

	def _on_turret_spin_changed(self, deg):
		slider_val = int(round(deg * 10.0))
		self.lbl_turret_val.setText(f"{deg:.1f}°")
		if self.slider_turret_pitch.value() != slider_val:
			self.slider_turret_pitch.blockSignals(True)
			self.slider_turret_pitch.setValue(slider_val)
			self.slider_turret_pitch.blockSignals(False)
		self._send_turret_pitch_to_px4(deg)

	def _set_turret_pitch(self, deg):
		self.spin_turret_pitch.setValue(deg)

	def _send_turret_pitch_to_px4(self, deg):
		deg = max(0.0, min(45.0, float(deg)))
		if hasattr(self, "mav_worker") and self.mav_worker is not None:
			self.mav_worker.set_param("INT_TURRET_PITCH", deg)

	# net parameters of tools/net_launcher.py (EDITABLE_PARAMS): key, label, unit, display scale,
	# display min, max, decimals, step, tooltip
	NET_PARAMS = [
		("open_diameter", "Open Diameter", " m", 1.0, 1.0, 6.0, 2, 0.1, "Open hexagon, corner to corner"),
		("mesh_size", "Mesh Size", " cm", 100.0, 5.0, 60.0, 0, 1.0, "Mesh opening, sets the total cord length"),
		("cord_diameter", "Cord Diameter", " mm", 1000.0, 0.3, 3.0, 2, 0.1, "Braided Dyneema cord"),
		("ball_diameter", "Ball Diameter", " mm", 1000.0, 6.0, 30.0, 1, 1.0, "Steel balls at the 6 corners"),
		("muzzle_speed", "Muzzle Speed", " m/s", 1.0, 10.0, 300.0, 0, 5.0, "Ball speed relative to the vehicle"),
		("splay_deg", "Splay Angle", "°", 1.0, 5.0, 45.0, 1, 1.0, "Ball launch angle off the barrel axis"),
	]

	def _init_net_ui(self, parent):
		layout = QVBoxLayout(parent)
		layout.setContentsMargins(10, 10, 10, 10)
		layout.setSpacing(8)

		# 1. Net & launcher parameters
		params_group = QGroupBox("Net & Launcher")
		params_layout = QVBoxLayout(params_group)
		grid = QGridLayout()
		grid.setHorizontalSpacing(10)
		grid.setVerticalSpacing(4)

		self.net_editors = {}
		for row, (key, label, unit, scale, lo, hi, dec, step, tip) in enumerate(self.NET_PARAMS):
			editor = QDoubleSpinBox()
			editor.setRange(lo, hi)
			editor.setDecimals(dec)
			editor.setSingleStep(step)
			editor.setSuffix(unit)
			editor.setToolTip(tip)
			editor.setFixedWidth(110)
			self.net_editors[key] = editor
			hint = QLabel(tip)
			hint.setStyleSheet("color: gray;")
			grid.addWidget(QLabel(label + ":"), row, 0)
			grid.addWidget(editor, row, 1)
			grid.addWidget(hint, row, 2)
		grid.setColumnStretch(2, 1)
		params_layout.addLayout(grid)

		self.lbl_net_derived = QLabel("Net -- g, balls 6 x -- g, cord -- m")
		self.lbl_net_derived.setStyleSheet("font-weight: bold;")
		params_layout.addWidget(self.lbl_net_derived)

		btn_row = QHBoxLayout()
		btn_reload = QPushButton("Reload")
		btn_reload.setToolTip("Read the current parameters from the net launcher")
		btn_reload.clicked.connect(self.net_worker.fetch_params)
		btn_apply = QPushButton("Apply")
		btn_apply.setStyleSheet("font-weight: bold;")
		btn_apply.setToolTip("Send to the net launcher, used from the next shot")
		btn_apply.clicked.connect(self._on_net_apply)
		btn_save = QPushButton("Save to interceptor.yaml")
		btn_save.setToolTip("Write the applied parameters to the net: section of interceptor.yaml")
		btn_save.clicked.connect(self.net_worker.save_params)
		btn_row.addWidget(btn_reload)
		btn_row.addStretch()
		btn_row.addWidget(btn_apply)
		btn_row.addWidget(btn_save)
		params_layout.addLayout(btn_row)
		layout.addWidget(params_group)

		# 2. The net in the air (the launcher keeps one net at a time)
		flight_group = QGroupBox("Net in Flight")
		flight_layout = QVBoxLayout(flight_group)
		fgrid = QGridLayout()
		fgrid.setVerticalSpacing(2)
		self.net_tel = {}
		items = [
			("Launcher:", "launcher", 0, 0), ("Shots:", "shots", 0, 2),
			("Net:", "id", 1, 0), ("Phase:", "phase", 1, 2),
			("Flight Time:", "t", 2, 0), ("Distance:", "dist", 2, 2),
			("Spread:", "spread", 3, 0), ("Speed:", "speed", 3, 2),
		]
		for title, key, row, col in items:
			self.net_tel[key] = QLabel("--")
			fgrid.addWidget(QLabel(title), row, col)
			fgrid.addWidget(self.net_tel[key], row, col + 1)
		flight_layout.addLayout(fgrid)

		action_row = QHBoxLayout()
		btn_fire = QPushButton("Fire Net")
		btn_fire.setFixedHeight(26)
		btn_fire.clicked.connect(self._on_fire_net)
		btn_clear = QPushButton("Clear")
		btn_clear.setFixedWidth(60)
		btn_clear.setFixedHeight(26)
		btn_clear.clicked.connect(self._on_clear_nets)
		action_row.addWidget(btn_fire, 1)
		action_row.addWidget(btn_clear)
		flight_layout.addLayout(action_row)
		layout.addWidget(flight_group)

		self.lbl_net_status = QLabel("Waiting for the net launcher...")
		self.lbl_net_status.setStyleSheet("color: gray;")
		layout.addWidget(self.lbl_net_status)
		layout.addStretch()

	def _on_net_apply(self):
		values = {}
		for key, _, _, scale, *_ in self.NET_PARAMS:
			values[key] = self.net_editors[key].value() / scale
		self.net_worker.send_params(values)

	def on_net_params(self, data):
		for key, _, _, scale, *_ in self.NET_PARAMS:
			value = data.get("params", {}).get(key)
			editor = self.net_editors[key]
			if value is not None and not editor.hasFocus():
				editor.blockSignals(True)
				editor.setValue(float(value) * scale)
				editor.blockSignals(False)

		d = data.get("derived", {})
		if d:
			net_g, ball_g = d["net_mass_kg"] * 1000.0, d["ball_mass_kg"] * 1000.0
			self.lbl_net_derived.setText(
				f"Net {net_g:.0f} g, balls 6 x {ball_g:.1f} g, total {net_g + 6 * ball_g:.0f} g, "
				f"cord {d['cord_length_m']:.0f} m"
			)

	def on_net_status(self, data):
		if not self.net_params_loaded:
			self.net_params_loaded = True
			self.net_worker.fetch_params()

		self.net_tel["launcher"].setText("Ready" if data.get("ready") else "No vehicle pose")
		self.net_tel["shots"].setText(str(data.get("launch_count", "--")))
		net = data.get("net")
		if net is None:
			for key in ("id", "phase", "t", "dist", "spread", "speed"):
				self.net_tel[key].setText("--")
			return

		self.net_tel["id"].setText(f"#{net['id']}")
		self.net_tel["phase"].setText(str(net["phase"]).upper())
		self.net_tel["t"].setText(f"{net['t']:.1f} s")
		self.net_tel["dist"].setText(f"{net['dist']:.1f} m")
		self.net_tel["spread"].setText(f"{net['spread']:.2f} m")
		self.net_tel["speed"].setText(f"{net['speed']:.1f} m/s")

	def _load_initial_wind_from_yaml(self):
		yaml_path = os.path.abspath(
			os.path.join(os.path.dirname(__file__), "..", "interceptor.yaml")
		)
		if os.path.isfile(yaml_path):
			try:
				with open(yaml_path, "r", encoding="utf-8") as f:
					data = yaml.safe_load(f) or {}
				w = data.get("wind", {})
				if w:
					self.wind_enabled = w.get("enabled", True)
					self.wind_speed_mps = float(w.get("speed_mps", 0.0))
					self.wind_direction_deg = float(w.get("direction_deg", 0.0))
					self.wind_vertical_mps = float(w.get("vertical_mps", 0.0))
					self.wind_turbulence_pct = float(w.get("turbulence_pct", 0.0))
					self.wind_gust_speed_mps = float(w.get("gust_speed_mps", 3.0))
					self.wind_gust_period_s = float(w.get("gust_period_s", 6.0))
			except Exception:
				pass

	def _init_wind_ui(self, parent):
		layout = QVBoxLayout(parent)
		layout.setContentsMargins(12, 12, 12, 12)
		layout.setSpacing(10)

		# 1. Master Toggle & Live Status Banner
		status_group = QGroupBox("Rüzgar Durumu & Canlı Telemetri (Live Wind Monitor)")
		status_layout = QVBoxLayout(status_group)
		status_layout.setSpacing(8)

		top_row = QHBoxLayout()
		self.chk_wind_enable = QCheckBox("Rüzgar Simülasyonu Aktif (Enable Wind)")
		self.chk_wind_enable.setStyleSheet("font-weight: bold; font-size: 13px;")
		self.chk_wind_enable.setChecked(self.wind_enabled)
		self.chk_wind_enable.toggled.connect(self._on_wind_enabled_toggled)
		top_row.addWidget(self.chk_wind_enable)
		top_row.addStretch()

		self.lbl_wind_badge = QLabel("0.0 m/s (Sakin)")
		self.lbl_wind_badge.setStyleSheet(
			"border: 1px solid #7f8c8d; font-weight: bold; padding: 4px 12px; border-radius: 4px; font-size: 13px;"
		)
		top_row.addWidget(self.lbl_wind_badge)
		status_layout.addLayout(top_row)

		grid_telemetry = QGridLayout()
		grid_telemetry.setHorizontalSpacing(15)
		grid_telemetry.setVerticalSpacing(4)

		self.lbl_tel_wind_speed = QLabel("0.0 m/s  (0.0 km/h)")
		self.lbl_tel_wind_dir = QLabel("0°  (Kuzey - N)")
		self.lbl_tel_wind_vector = QLabel("Vx: 0.0, Vy: 0.0, Vz: 0.0 m/s")
		self.lbl_tel_wind_gust = QLabel("0.0 m/s  (Sakin)")

		for val in (self.lbl_tel_wind_speed, self.lbl_tel_wind_dir, self.lbl_tel_wind_vector, self.lbl_tel_wind_gust):
			val.setStyleSheet("font-weight: bold;")

		grid_telemetry.addWidget(QLabel("Toplam Rüzgar:"), 0, 0)
		grid_telemetry.addWidget(self.lbl_tel_wind_speed, 0, 1)
		grid_telemetry.addWidget(QLabel("Rüzgar Yönü:"), 0, 2)
		grid_telemetry.addWidget(self.lbl_tel_wind_dir, 0, 3)

		grid_telemetry.addWidget(QLabel("Vektör (ENU):"), 1, 0)
		grid_telemetry.addWidget(self.lbl_tel_wind_vector, 1, 1)
		grid_telemetry.addWidget(QLabel("Anlık Hamle:"), 1, 2)
		grid_telemetry.addWidget(self.lbl_tel_wind_gust, 1, 3)

		status_layout.addLayout(grid_telemetry)
		layout.addWidget(status_group)

		# 2. Steady Wind Controls
		steady_group = QGroupBox("Temel Rüzgar Ayarları (Steady Wind)")
		steady_layout = QVBoxLayout(steady_group)
		steady_layout.setSpacing(6)

		# Speed row
		row_spd = QHBoxLayout()
		row_spd.addWidget(QLabel("Yatay Rüzgar Hızı (Horizontal Speed):"))
		self.spin_wind_speed = QDoubleSpinBox()
		self.spin_wind_speed.setRange(0.0, 30.0)
		self.spin_wind_speed.setSingleStep(0.5)
		self.spin_wind_speed.setSuffix(" m/s")
		self.spin_wind_speed.setValue(self.wind_speed_mps)
		self.slider_wind_speed = QSlider(Qt.Horizontal)
		self.slider_wind_speed.setRange(0, 300)
		self.slider_wind_speed.setValue(int(self.wind_speed_mps * 10))

		self.spin_wind_speed.valueChanged.connect(self._on_wind_speed_spin_changed)
		self.slider_wind_speed.valueChanged.connect(self._on_wind_speed_slider_changed)

		row_spd.addWidget(self.slider_wind_speed, 1)
		row_spd.addWidget(self.spin_wind_speed)
		steady_layout.addLayout(row_spd)

		# Direction row
		row_dir = QHBoxLayout()
		row_dir.addWidget(QLabel("Rüzgar Yönü (Direction):"))
		self.spin_wind_dir = QSpinBox()
		self.spin_wind_dir.setRange(0, 360)
		self.spin_wind_dir.setSingleStep(5)
		self.spin_wind_dir.setSuffix("°")
		self.spin_wind_dir.setValue(int(self.wind_direction_deg))
		self.slider_wind_dir = QSlider(Qt.Horizontal)
		self.slider_wind_dir.setRange(0, 360)
		self.slider_wind_dir.setValue(int(self.wind_direction_deg))

		self.spin_wind_dir.valueChanged.connect(self._on_wind_dir_spin_changed)
		self.slider_wind_dir.valueChanged.connect(self._on_wind_dir_slider_changed)

		row_dir.addWidget(self.slider_wind_dir, 1)
		row_dir.addWidget(self.spin_wind_dir)
		steady_layout.addLayout(row_dir)

		# Quick Direction Buttons
		row_dir_btns = QHBoxLayout()
		row_dir_btns.setSpacing(6)
		dir_presets = [
			("Kuzey (0°)", 0),
			("Doğu (90°)", 90),
			("Güney (180°)", 180),
			("Batı (270°)", 270),
			("Pist Baş Rüzgarı (68°)", 68),
			("Pist Yan Rüzgarı (158°)", 158),
		]
		for name, deg in dir_presets:
			btn = QPushButton(name)
			btn.setStyleSheet("padding: 3px 6px; font-size: 11px;")
			btn.clicked.connect(lambda _, d=deg: self._set_wind_direction(d))
			row_dir_btns.addWidget(btn)
		steady_layout.addLayout(row_dir_btns)

		# Vertical airflow row
		row_vert = QHBoxLayout()
		row_vert.addWidget(QLabel("Dikey Hava Akımı (Updraft / Downdraft):"))
		self.spin_wind_vert = QDoubleSpinBox()
		self.spin_wind_vert.setRange(-5.0, 5.0)
		self.spin_wind_vert.setSingleStep(0.2)
		self.spin_wind_vert.setSuffix(" m/s")
		self.spin_wind_vert.setValue(self.wind_vertical_mps)
		self.slider_wind_vert = QSlider(Qt.Horizontal)
		self.slider_wind_vert.setRange(-50, 50)
		self.slider_wind_vert.setValue(int(self.wind_vertical_mps * 10))

		self.spin_wind_vert.valueChanged.connect(self._on_wind_vert_spin_changed)
		self.slider_wind_vert.valueChanged.connect(self._on_wind_vert_slider_changed)

		row_vert.addWidget(self.slider_wind_vert, 1)
		row_vert.addWidget(self.spin_wind_vert)
		steady_layout.addLayout(row_vert)

		layout.addWidget(steady_group)

		# 3. Turbulence & Gust Controls
		turb_group = QGroupBox("Türbülans ve Rüzgar Hamlesi (Turbulence & Gusts)")
		turb_layout = QVBoxLayout(turb_group)
		turb_layout.setSpacing(6)

		row_turb = QHBoxLayout()
		row_turb.addWidget(QLabel("Türbülans Şiddeti (Intensity):"))
		self.spin_wind_turb = QSpinBox()
		self.spin_wind_turb.setRange(0, 100)
		self.spin_wind_turb.setSingleStep(5)
		self.spin_wind_turb.setSuffix("%")
		self.spin_wind_turb.setValue(int(self.wind_turbulence_pct))
		self.slider_wind_turb = QSlider(Qt.Horizontal)
		self.slider_wind_turb.setRange(0, 100)
		self.slider_wind_turb.setValue(int(self.wind_turbulence_pct))

		self.spin_wind_turb.valueChanged.connect(self._on_wind_turb_spin_changed)
		self.slider_wind_turb.valueChanged.connect(self._on_wind_turb_slider_changed)

		row_turb.addWidget(self.slider_wind_turb, 1)
		row_turb.addWidget(self.spin_wind_turb)
		turb_layout.addLayout(row_turb)

		# Turbulence preset buttons
		row_turb_btns = QHBoxLayout()
		row_turb_btns.setSpacing(6)
		turb_presets = [
			("Sakin (%0)", 0),
			("Hafif (%20)", 20),
			("Orta (%50)", 50),
			("Şiddetli (%80)", 80),
		]
		for name, pct in turb_presets:
			btn = QPushButton(name)
			btn.setStyleSheet("padding: 3px 6px; font-size: 11px;")
			btn.clicked.connect(lambda _, p=pct: self._set_wind_turbulence(p))
			row_turb_btns.addWidget(btn)
		turb_layout.addLayout(row_turb_btns)

		grid_gust = QGridLayout()
		grid_gust.addWidget(QLabel("Maksimum Hamle Hızı (Gust Peak):"), 0, 0)
		self.spin_gust_speed = QDoubleSpinBox()
		self.spin_gust_speed.setRange(0.0, 15.0)
		self.spin_gust_speed.setSingleStep(0.5)
		self.spin_gust_speed.setSuffix(" m/s")
		self.spin_gust_speed.setValue(self.wind_gust_speed_mps)
		self.spin_gust_speed.valueChanged.connect(lambda v: setattr(self, "wind_gust_speed_mps", v))
		grid_gust.addWidget(self.spin_gust_speed, 0, 1)

		grid_gust.addWidget(QLabel("Hamle Periyodu (Gust Period):"), 0, 2)
		self.spin_gust_period = QDoubleSpinBox()
		self.spin_gust_period.setRange(1.0, 20.0)
		self.spin_gust_period.setSingleStep(0.5)
		self.spin_gust_period.setSuffix(" s")
		self.spin_gust_period.setValue(self.wind_gust_period_s)
		self.spin_gust_period.valueChanged.connect(lambda v: setattr(self, "wind_gust_period_s", v))
		grid_gust.addWidget(self.spin_gust_period, 0, 3)

		turb_layout.addLayout(grid_gust)

		# Trigger Gust Action
		row_trigger = QHBoxLayout()
		self.btn_trigger_gust = QPushButton("Ani Rüzgar Hamlesi Gönder (Trigger 4s Gust)")
		self.btn_trigger_gust.setStyleSheet("padding: 6px; font-weight: bold;")
		self.btn_trigger_gust.clicked.connect(self._trigger_manual_gust)
		self.prog_gust = QProgressBar()
		self.prog_gust.setRange(0, 100)
		self.prog_gust.setValue(0)
		self.prog_gust.setVisible(False)
		self.prog_gust.setFixedHeight(20)
		row_trigger.addWidget(self.btn_trigger_gust)
		row_trigger.addWidget(self.prog_gust)
		turb_layout.addLayout(row_trigger)

		layout.addWidget(turb_group)

		# 4. Weather Presets
		presets_group = QGroupBox("Hazır Hava Durumu Senaryoları (Weather Presets)")
		presets_layout = QHBoxLayout(presets_group)
		presets_layout.setSpacing(8)

		presets = [
			("Sakin (Calm)\n0 m/s, %0", 0.0, 0, 0.0, 0, 0.0),
			("Hafif Esinti\n4 m/s D, %15", 4.0, 90, 0.0, 15, 2.0),
			("Şiddetli Yan Rüzgar\n10 m/s (158°), %35", 10.0, 158, 0.0, 35, 5.0),
			("Fırtınalı & Hamleli\n15 m/s G, %75", 15.0, 180, -1.0, 75, 8.0),
			("Termal Yukarı Akım\n4 m/s, +3.5 m/s", 4.0, 68, 3.5, 25, 3.0),
		]
		for title, spd, dir_deg, vert, turb, gst in presets:
			btn = QPushButton(title)
			btn.setStyleSheet("padding: 8px; font-size: 11px;")
			btn.clicked.connect(lambda _, s=spd, d=dir_deg, v=vert, t=turb, g=gst: self._apply_weather_preset(s, d, v, t, g))
			presets_layout.addWidget(btn)

		layout.addWidget(presets_group)

		# 5. Save & Reset actions
		row_bottom = QHBoxLayout()
		self.btn_reset_wind = QPushButton("Rüzgarı Sıfırla (Reset Calm)")
		self.btn_reset_wind.clicked.connect(lambda: self._apply_weather_preset(0.0, 0, 0.0, 0, 0.0))
		self.btn_save_wind = QPushButton("Varsayılan Olarak interceptor.yaml'a Kaydet")
		self.btn_save_wind.clicked.connect(self._save_wind_to_yaml)
		self.lbl_wind_save_status = QLabel("")

		row_bottom.addWidget(self.btn_reset_wind)
		row_bottom.addWidget(self.btn_save_wind)
		row_bottom.addWidget(self.lbl_wind_save_status)
		row_bottom.addStretch()

		layout.addLayout(row_bottom)
		layout.addStretch()

	def _set_wind_direction(self, deg):
		self.wind_direction_deg = float(deg)
		self.spin_wind_dir.blockSignals(True)
		self.slider_wind_dir.blockSignals(True)
		self.spin_wind_dir.setValue(int(deg))
		self.slider_wind_dir.setValue(int(deg))
		self.spin_wind_dir.blockSignals(False)
		self.slider_wind_dir.blockSignals(False)

	def _set_wind_turbulence(self, pct):
		self.wind_turbulence_pct = float(pct)
		self.spin_wind_turb.blockSignals(True)
		self.slider_wind_turb.blockSignals(True)
		self.spin_wind_turb.setValue(int(pct))
		self.slider_wind_turb.setValue(int(pct))
		self.spin_wind_turb.blockSignals(False)
		self.slider_wind_turb.blockSignals(False)

	def _apply_weather_preset(self, spd, dir_deg, vert, turb, gst):
		self.wind_enabled = True
		self.chk_wind_enable.setChecked(True)
		self.wind_speed_mps = float(spd)
		self.wind_direction_deg = float(dir_deg)
		self.wind_vertical_mps = float(vert)
		self.wind_turbulence_pct = float(turb)
		self.wind_gust_speed_mps = float(gst)

		self.spin_wind_speed.setValue(spd)
		self.slider_wind_speed.setValue(int(spd * 10))
		self._set_wind_direction(dir_deg)
		self.spin_wind_vert.setValue(vert)
		self.slider_wind_vert.setValue(int(vert * 10))
		self._set_wind_turbulence(turb)
		self.spin_gust_speed.setValue(gst)

	def _trigger_manual_gust(self):
		self.manual_gust_active = True
		self.manual_gust_start_time = time.time()
		self.prog_gust.setValue(100)
		self.prog_gust.setVisible(True)
		self.btn_trigger_gust.setEnabled(False)

	def _on_wind_enabled_toggled(self, checked):
		self.wind_enabled = checked
		if not checked:
			self._wind_was_enabled = None  # Force immediate disable publish

	def _on_wind_speed_spin_changed(self, val):
		self.wind_speed_mps = val
		self.slider_wind_speed.blockSignals(True)
		self.slider_wind_speed.setValue(int(val * 10))
		self.slider_wind_speed.blockSignals(False)

	def _on_wind_speed_slider_changed(self, val):
		speed = val / 10.0
		self.wind_speed_mps = speed
		self.spin_wind_speed.blockSignals(True)
		self.spin_wind_speed.setValue(speed)
		self.spin_wind_speed.blockSignals(False)

	def _on_wind_dir_spin_changed(self, val):
		self.wind_direction_deg = float(val)
		self.slider_wind_dir.blockSignals(True)
		self.slider_wind_dir.setValue(val)
		self.slider_wind_dir.blockSignals(False)

	def _on_wind_dir_slider_changed(self, val):
		self.wind_direction_deg = float(val)
		self.spin_wind_dir.blockSignals(True)
		self.spin_wind_dir.setValue(val)
		self.spin_wind_dir.blockSignals(False)

	def _on_wind_vert_spin_changed(self, val):
		self.wind_vertical_mps = val
		self.slider_wind_vert.blockSignals(True)
		self.slider_wind_vert.setValue(int(val * 10))
		self.slider_wind_vert.blockSignals(False)

	def _on_wind_vert_slider_changed(self, val):
		vert = val / 10.0
		self.wind_vertical_mps = vert
		self.spin_wind_vert.blockSignals(True)
		self.spin_wind_vert.setValue(vert)
		self.spin_wind_vert.blockSignals(False)

	def _on_wind_turb_spin_changed(self, val):
		self.wind_turbulence_pct = float(val)
		self.slider_wind_turb.blockSignals(True)
		self.slider_wind_turb.setValue(val)
		self.slider_wind_turb.blockSignals(False)

	def _on_wind_turb_slider_changed(self, val):
		self.wind_turbulence_pct = float(val)
		self.spin_wind_turb.blockSignals(True)
		self.spin_wind_turb.setValue(val)
		self.spin_wind_turb.blockSignals(False)

	def _on_wind_tick(self):
		now = time.time()
		dt = max(0.001, min(0.2, now - self.last_wind_update_time))
		self.last_wind_update_time = now

		if not self.wind_enabled:
			self.current_wind_vector[:] = [0.0, 0.0, 0.0]
			if self._wind_was_enabled is not False:
				self._wind_was_enabled = False
				self._update_wind_live_labels(0.0, 0.0, 0.0, 0.0, 0.0)
				if self.gz_pub_wind is not None:
					try:
						msg = GzWind()
						msg.enable_wind = False
						msg.linear_velocity.x = 0.0
						msg.linear_velocity.y = 0.0
						msg.linear_velocity.z = 0.0
						self.gz_pub_wind.publish(msg)
					except Exception:
						pass
			return

		self._wind_was_enabled = True

		# Base steady wind in ENU (0 deg = North -> blows toward -Y)
		rad = math.radians(self.wind_direction_deg)
		base_vx = -self.wind_speed_mps * math.sin(rad)
		base_vy = -self.wind_speed_mps * math.cos(rad)
		base_vz = self.wind_vertical_mps

		# Turbulence model
		turb_intensity = self.wind_turbulence_pct / 100.0
		gust_mag = 0.0

		if turb_intensity > 0.001:
			tau = max(0.5, self.wind_gust_period_s / 3.0)
			alpha = max(0.02, min(0.9, dt / tau))
			sigma = turb_intensity * (self.wind_gust_speed_mps * 0.5 + self.wind_speed_mps * 0.25)
			raw_noise = np.random.normal(0.0, max(0.01, sigma), 3)
			self.turb_state = (1.0 - alpha) * self.turb_state + alpha * raw_noise

			# Periodic gust swell
			swell = turb_intensity * 0.3 * self.wind_gust_speed_mps * math.sin(2.0 * math.pi * now / max(0.5, self.wind_gust_period_s))
			self.turb_state[0] += swell * (-math.sin(rad))
			self.turb_state[1] += swell * (-math.cos(rad))
			gust_mag = float(np.linalg.norm(self.turb_state))
		else:
			self.turb_state[:] = [0.0, 0.0, 0.0]

		# Manual gust pulse
		manual_gust_v = np.zeros(3)
		if self.manual_gust_active:
			elapsed = now - self.manual_gust_start_time
			if elapsed < self.manual_gust_duration_s:
				pulse = math.sin(math.pi * elapsed / self.manual_gust_duration_s) ** 2
				amp = max(self.manual_gust_peak_mps, self.wind_gust_speed_mps)
				manual_gust_v[0] = -amp * pulse * math.sin(rad)
				manual_gust_v[1] = -amp * pulse * math.cos(rad)
				manual_gust_v[2] = 0.3 * amp * pulse
				gust_mag += amp * pulse
				if hasattr(self, "prog_gust"):
					self.prog_gust.setValue(int(100 * (1.0 - elapsed / self.manual_gust_duration_s)))
			else:
				self.manual_gust_active = False
				if hasattr(self, "prog_gust"):
					self.prog_gust.setValue(0)
					self.prog_gust.setVisible(False)
				if hasattr(self, "btn_trigger_gust"):
					self.btn_trigger_gust.setEnabled(True)

		total_vx = base_vx + self.turb_state[0] + manual_gust_v[0]
		total_vy = base_vy + self.turb_state[1] + manual_gust_v[1]
		total_vz = base_vz + self.turb_state[2] + manual_gust_v[2]

		self.current_wind_vector[:] = [total_vx, total_vy, total_vz]
		total_speed = math.sqrt(total_vx**2 + total_vy**2 + total_vz**2)

		self._update_wind_live_labels(total_speed, total_vx, total_vy, total_vz, gust_mag)

		# Publish to Gazebo only if noticeably changed or periodic keepalive (1.0s)
		wind_diff = float(np.linalg.norm(self.current_wind_vector - self._last_published_wind))
		if (wind_diff > 0.05 or (now - self._last_published_time) >= 1.0) and self.gz_pub_wind is not None:
			self._last_published_wind[:] = self.current_wind_vector
			self._last_published_time = now
			try:
				msg = GzWind()
				msg.enable_wind = True
				msg.linear_velocity.x = float(total_vx)
				msg.linear_velocity.y = float(total_vy)
				msg.linear_velocity.z = float(total_vz)
				self.gz_pub_wind.publish(msg)
			except Exception:
				pass

	def _update_wind_live_labels(self, speed, vx, vy, vz, gust=0.0):
		if not hasattr(self, "lbl_tel_wind_speed"):
			return
		kmh = speed * 3.6
		self.lbl_tel_wind_speed.setText(f"{speed:.1f} m/s  ({kmh:.1f} km/h)")
		compass = get_compass_direction(self.wind_direction_deg)
		self.lbl_tel_wind_dir.setText(f"{self.wind_direction_deg:.0f}°  ({compass})")
		self.lbl_tel_wind_vector.setText(f"Vx: {vx:+.1f}, Vy: {vy:+.1f}, Vz: {vz:+.1f} m/s")

		if not self.wind_enabled or speed < 0.2:
			text = "0.0 m/s (Sakin)"
			gust_text = "0.0 m/s (Sakin)"
		elif speed < 6.0:
			text = f"{speed:.1f} m/s (Hafif)"
			gust_text = f"±{gust:.1f} m/s"
		elif speed < 12.0:
			text = f"{speed:.1f} m/s (Orta/Sert)"
			gust_text = f"±{gust:.1f} m/s (Hamleli)"
		else:
			text = f"{speed:.1f} m/s (Fırtınalı)"
			gust_text = f"±{gust:.1f} m/s (Kuvvetli)"

		self.lbl_wind_badge.setText(text)
		self.lbl_tel_wind_gust.setText(gust_text)

	def _save_wind_to_yaml(self):
		yaml_path = os.path.abspath(
			os.path.join(os.path.dirname(__file__), "..", "interceptor.yaml")
		)
		try:
			data = {}
			if os.path.isfile(yaml_path):
				with open(yaml_path, "r", encoding="utf-8") as f:
					data = yaml.safe_load(f) or {}
			data["wind"] = {
				"enabled": bool(self.wind_enabled),
				"speed_mps": round(float(self.wind_speed_mps), 1),
				"direction_deg": round(float(self.wind_direction_deg), 1),
				"vertical_mps": round(float(self.wind_vertical_mps), 1),
				"turbulence_pct": round(float(self.wind_turbulence_pct), 1),
				"gust_speed_mps": round(float(self.wind_gust_speed_mps), 1),
				"gust_period_s": round(float(self.wind_gust_period_s), 1),
			}
			with open(yaml_path, "w", encoding="utf-8") as f:
				yaml.dump(data, f, default_flow_style=False, sort_keys=False)
			self.lbl_wind_save_status.setText("interceptor.yaml kaydedildi.")
			self.lbl_wind_save_status.setStyleSheet("font-weight: bold;")
			QTimer.singleShot(3000, lambda: self.lbl_wind_save_status.setText(""))
		except Exception as e:
			self.lbl_wind_save_status.setText(f"Hata: {e}")
			self.lbl_wind_save_status.setStyleSheet("font-weight: bold;")

	def _init_vision_noise_ui(self, parent_layout):
		noise_group = QGroupBox("Vision Detector Noise")
		noise_layout = QVBoxLayout(noise_group)
		noise_layout.setSpacing(4)

		# Top row: Enable toggle & status
		top_row = QHBoxLayout()
		self.chk_noise_enable = QCheckBox("Enable Noise")
		self.chk_noise_enable.setChecked(True)
		self.chk_noise_enable.toggled.connect(self._on_noise_toggled)
		top_row.addWidget(self.chk_noise_enable)
		top_row.addStretch()
		self.lbl_noise_status = QLabel("Ready")
		self.lbl_noise_status.setStyleSheet("color: gray;")
		top_row.addWidget(self.lbl_noise_status)
		noise_layout.addLayout(top_row)

		# Sliders grid
		grid = QGridLayout()
		grid.setHorizontalSpacing(8)
		grid.setVerticalSpacing(3)

		# 1. BBox Jitter (0.0 to 15.0 px, default 4.0 px)
		grid.addWidget(QLabel("BBox Jitter:"), 0, 0)
		self.slider_bbox_jitter = QSlider(Qt.Horizontal)
		self.slider_bbox_jitter.setRange(0, 150)
		self.slider_bbox_jitter.setValue(40)
		self.lbl_bbox_jitter_val = QLabel("4.0 px")
		self.lbl_bbox_jitter_val.setFixedWidth(50)
		self.slider_bbox_jitter.valueChanged.connect(
			lambda v: self.lbl_bbox_jitter_val.setText(f"{v/10.0:.1f} px")
		)
		self.slider_bbox_jitter.sliderReleased.connect(
			lambda: self._send_noise_cmd({"bbox_px": self.slider_bbox_jitter.value() / 10.0})
		)
		grid.addWidget(self.slider_bbox_jitter, 0, 1)
		grid.addWidget(self.lbl_bbox_jitter_val, 0, 2)

		# 2. Dropout Rate (0 to 30 %, default 5%)
		grid.addWidget(QLabel("Dropout:"), 1, 0)
		self.slider_dropout = QSlider(Qt.Horizontal)
		self.slider_dropout.setRange(0, 30)
		self.slider_dropout.setValue(5)
		self.lbl_dropout_val = QLabel("5 %")
		self.lbl_dropout_val.setFixedWidth(50)
		self.slider_dropout.valueChanged.connect(
			lambda v: self.lbl_dropout_val.setText(f"{v} %")
		)
		self.slider_dropout.sliderReleased.connect(
			lambda: self._send_noise_cmd({"dropout_rate": self.slider_dropout.value() / 100.0})
		)
		grid.addWidget(self.slider_dropout, 1, 1)
		grid.addWidget(self.lbl_dropout_val, 1, 2)

		# 3. Latency (0 to 5 frames, default 1 frame)
		grid.addWidget(QLabel("Latency:"), 2, 0)
		self.slider_latency = QSlider(Qt.Horizontal)
		self.slider_latency.setRange(0, 5)
		self.slider_latency.setValue(1)
		self.lbl_latency_val = QLabel("1 frame")
		self.lbl_latency_val.setFixedWidth(50)
		self.slider_latency.valueChanged.connect(
			lambda v: self.lbl_latency_val.setText(f"{v} frame" if v == 1 else f"{v} frames")
		)
		self.slider_latency.sliderReleased.connect(
			lambda: self._send_noise_cmd({"latency_frames": self.slider_latency.value()})
		)
		grid.addWidget(self.slider_latency, 2, 1)
		grid.addWidget(self.lbl_latency_val, 2, 2)

		# 4. Range Noise (0.00 to 0.50 m, default 0.15 m)
		grid.addWidget(QLabel("Range Noise:"), 3, 0)
		self.slider_range_noise = QSlider(Qt.Horizontal)
		self.slider_range_noise.setRange(0, 50)
		self.slider_range_noise.setValue(15)
		self.lbl_range_noise_val = QLabel("±0.15 m")
		self.lbl_range_noise_val.setFixedWidth(50)
		self.slider_range_noise.valueChanged.connect(
			lambda v: self.lbl_range_noise_val.setText(f"±{v/100.0:.2f} m")
		)
		self.slider_range_noise.sliderReleased.connect(
			lambda: self._send_noise_cmd({"range_noise_m": self.slider_range_noise.value() / 100.0})
		)
		grid.addWidget(self.slider_range_noise, 3, 1)
		grid.addWidget(self.lbl_range_noise_val, 3, 2)

		noise_layout.addLayout(grid)

		# Presets row
		preset_row = QHBoxLayout()
		preset_row.setSpacing(4)

		btn_clean = QPushButton("Clean (0 px)")
		btn_clean.setToolTip("Disable jitter, dropout, and latency")
		btn_clean.clicked.connect(lambda: self._apply_noise_preset(0.0, 0.0, 0, 0.0))

		btn_default = QPushButton("Default")
		btn_default.setToolTip("Standard noise: 4.0 px jitter, 5% drop, 1 frame delay")
		btn_default.clicked.connect(lambda: self._apply_noise_preset(4.0, 0.05, 1, 0.15))

		btn_stress = QPushButton("Stress")
		btn_stress.setToolTip("Stress test: 10.0 px jitter, 15% drop, 2 frames delay")
		btn_stress.clicked.connect(lambda: self._apply_noise_preset(10.0, 0.15, 2, 0.35))

		for b in (btn_clean, btn_default, btn_stress):
			b.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
			preset_row.addWidget(b)

		noise_layout.addLayout(preset_row)
		parent_layout.addWidget(noise_group)

	def _on_noise_toggled(self, checked):
		self.slider_bbox_jitter.setEnabled(checked)
		self.slider_dropout.setEnabled(checked)
		self.slider_latency.setEnabled(checked)
		self.slider_range_noise.setEnabled(checked)
		self._send_noise_cmd({"enabled": checked})

	def _apply_noise_preset(self, bbox_px, dropout_rate, latency_frames, range_noise_m):
		self.chk_noise_enable.setChecked(True)
		self.slider_bbox_jitter.setValue(int(bbox_px * 10))
		self.slider_dropout.setValue(int(dropout_rate * 100))
		self.slider_latency.setValue(int(latency_frames))
		self.slider_range_noise.setValue(int(range_noise_m * 100))
		self._send_noise_cmd({
			"enabled": True,
			"bbox_px": bbox_px,
			"dropout_rate": dropout_rate,
			"latency_frames": latency_frames,
			"range_noise_m": range_noise_m,
		})

	def _send_noise_cmd(self, cmd_dict):
		self.detector_worker.send_command(cmd_dict)
		self.lbl_noise_status.setText("Applied")

	def on_detector_noise(self, data):
		if not self.slider_bbox_jitter.isSliderDown() and "bbox_px" in data:
			val = int(data["bbox_px"] * 10)
			self.slider_bbox_jitter.blockSignals(True)
			self.slider_bbox_jitter.setValue(val)
			self.slider_bbox_jitter.blockSignals(False)
			self.lbl_bbox_jitter_val.setText(f"{data['bbox_px']:.1f} px")

		if not self.slider_dropout.isSliderDown() and "dropout_rate" in data:
			val = int(data["dropout_rate"] * 100)
			self.slider_dropout.blockSignals(True)
			self.slider_dropout.setValue(val)
			self.slider_dropout.blockSignals(False)
			self.lbl_dropout_val.setText(f"{val} %")

		if not self.slider_latency.isSliderDown() and "latency_frames" in data:
			val = int(data["latency_frames"])
			self.slider_latency.blockSignals(True)
			self.slider_latency.setValue(val)
			self.slider_latency.blockSignals(False)
			self.lbl_latency_val.setText(f"{val} frame" if val == 1 else f"{val} frames")

		if not self.slider_range_noise.isSliderDown() and "range_noise_m" in data:
			val = int(data["range_noise_m"] * 100)
			self.slider_range_noise.blockSignals(True)
			self.slider_range_noise.setValue(val)
			self.slider_range_noise.blockSignals(False)
			self.lbl_range_noise_val.setText(f"±{data['range_noise_m']:.2f} m")

		if "enabled" in data:
			en = bool(data["enabled"])
			if self.chk_noise_enable.isChecked() != en:
				self.chk_noise_enable.blockSignals(True)
				self.chk_noise_enable.setChecked(en)
				self.chk_noise_enable.blockSignals(False)
				self.slider_bbox_jitter.setEnabled(en)
				self.slider_dropout.setEnabled(en)
				self.slider_latency.setEnabled(en)
				self.slider_range_noise.setEnabled(en)

		self.lbl_noise_status.setText("Connected")

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
			ptype = meta.get("type", "float")
			is_bool = (ptype == "boolean")
			is_int = (ptype in ("int32", "int"))
			if is_bool:
				editor = QComboBox()
				editor.addItem("0 (Disabled)")
				editor.addItem("1 (Enabled)")
				editor.setCurrentIndex(1 if int(meta.get("default", 1)) == 1 else 0)
			elif is_int:
				editor = QSpinBox()
				min_v = int(meta.get("min", -999)) if meta.get("min") is not None else -9999
				max_v = int(meta.get("max", 999)) if meta.get("max") is not None else 9999
				editor.setRange(min_v, max_v)
				editor.setValue(int(meta.get("default", 0)))
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

	def _on_tab_changed(self, index):
		# Auto-request params when switching to Tab 2 (Params) if connected and not yet requested
		if index == 1 and not self._initial_params_requested and self.mav_worker.target_system is not None:
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
			ptype = meta.get("type", "float")
			is_bool = (ptype == "boolean")
			is_int = (ptype in ("int32", "int"))
			editor = self.param_editors[name]
			if is_bool:
				val = 1.0 if editor.currentIndex() == 1 else 0.0
			elif is_int:
				val = int(editor.value())
			else:
				val = float(editor.value())
			self.mav_worker.set_param(name, val, is_int=(is_bool or is_int))
			count += 1
		if not self.drone_armed:
			self.mav_worker.save_params_to_storage()
			self.lbl_param_status.setText(f"{count} parameters applied and saved to persistent storage.")
		else:
			self.lbl_param_status.setText(f"{count} parameters applied live (storage save requires Disarm).")

	def _on_reset_defaults(self):
		for name, meta in self.param_metadata.items():
			if name not in self.param_editors:
				continue
			default_val = meta.get("default", 0)
			ptype = meta.get("type", "float")
			is_bool = (ptype == "boolean")
			is_int = (ptype in ("int32", "int"))
			editor = self.param_editors[name]
			if is_bool:
				editor.setCurrentIndex(1 if int(default_val) == 1 else 0)
			elif is_int:
				editor.setValue(int(default_val))
			else:
				editor.setValue(float(default_val))
		self.lbl_param_status.setText("Values reset to defaults. Click 'Save' to apply to PX4.")

	def on_param_received(self, name, val, ptype):
		if name not in self.param_metadata:
			return
		self.px4_param_values[name] = val
		meta = self.param_metadata[name]
		ptype_meta = meta.get("type", "float")
		is_bool = (ptype_meta == "boolean")
		is_int = (ptype_meta in ("int32", "int"))

		# Update the PX4 Value column
		if name in self.param_px4_items:
			item = self.param_px4_items[name]
			if is_bool:
				text = "1 (Enabled)" if int(val) == 1 else "0 (Disabled)"
			elif is_int:
				text = f"{int(val)}"
			else:
				dec = meta.get("decimal", 2)
				text = f"{val:.{dec}f}"
			item.setText(text)
			item.setForeground(QColor("#27ae60"))  # Green text indicating confirmed from PX4

		# Update the editor widget value if not actively focused
		if name in self.param_editors:
			editor = self.param_editors[name]
			if not editor.hasFocus():
				editor.blockSignals(True)
				if is_bool:
					editor.setCurrentIndex(1 if int(val) == 1 else 0)
				elif is_int:
					editor.setValue(int(val))
				else:
					editor.setValue(float(val))
				editor.blockSignals(False)

		if name == "INT_TURRET_PITCH":
			deg = float(val)
			if hasattr(self, "spin_turret_pitch") and not self.spin_turret_pitch.hasFocus():
				self.spin_turret_pitch.blockSignals(True)
				self.spin_turret_pitch.setValue(deg)
				self.spin_turret_pitch.blockSignals(False)
			if hasattr(self, "slider_turret_pitch") and not self.slider_turret_pitch.isSliderDown():
				self.slider_turret_pitch.blockSignals(True)
				self.slider_turret_pitch.setValue(int(round(deg * 10.0)))
				self.slider_turret_pitch.blockSignals(False)
			if hasattr(self, "lbl_turret_val"):
				self.lbl_turret_val.setText(f"{deg:.1f}°")

		if name == "INT_TRT_MODE" and hasattr(self, "combo_turret_mode"):
			self.combo_turret_mode.blockSignals(True)
			self.combo_turret_mode.setCurrentIndex(1 if int(val) == 1 else 0)
			self.combo_turret_mode.blockSignals(False)
			is_auto = (int(val) == 1)
			self.slider_turret_pitch.setEnabled(not is_auto)
			self.spin_turret_pitch.setEnabled(not is_auto)
			if hasattr(self, "spin_turret_offset"):
				self.spin_turret_offset.setEnabled(is_auto)

		if name == "INT_TRT_OFFSET" and hasattr(self, "spin_turret_offset"):
			if not self.spin_turret_offset.hasFocus():
				self.spin_turret_offset.blockSignals(True)
				self.spin_turret_offset.setValue(float(val))
				self.spin_turret_offset.blockSignals(False)

		if name == "INT_AUTO_FIRE" and hasattr(self, "chk_auto_fire"):
			self.chk_auto_fire.blockSignals(True)
			self.chk_auto_fire.setChecked(int(val) == 1)
			self.chk_auto_fire.blockSignals(False)

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
	parser.add_argument("--detector-port", type=int, default=8002, help="Detector HTTP control port")
	parser.add_argument("--mavlink", default="udpin:0.0.0.0:14550", help="Interceptor PX4 MAVLink link (GCS link, like QGC)")
	parser.add_argument("--world", default=os.getenv("WORLD", "ankara"), help="Gazebo world name")
	args = parser.parse_args()

	app = QApplication(sys.argv)
	win = ControlPanelWindow(host=args.host, port=args.port, mavlink_url=args.mavlink, detector_port=args.detector_port, world=args.world)
	win.show()
	sys.exit(app.exec())


if __name__ == "__main__":
	main()

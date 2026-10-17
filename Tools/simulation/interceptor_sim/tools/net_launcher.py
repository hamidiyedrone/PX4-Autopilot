#!/usr/bin/env python3
"""
Net launcher of the interceptor: fires a physically simulated net from the turret muzzle.

The net (tools/net_physics.py) is a 3 m hexagonal cord net with 12 mm steel balls at its
6 corners, the balls splaying out of the muzzle (speed and angle from the config). It is integrated in
Gazebo sim time and drawn as markers through the services /sensors/marker_array (the nose
camera sees it, also headless) and /marker_array (Gazebo GUI, when open). The net is not a Gazebo model, so it cannot push the
vehicle and needs no spawning (no launch delay).

Net parameters: the `net:` section of interceptor.yaml (keys of NetParams).

HTTP API on port 8003:
  POST /fire   -> launch a net (removes the previous one: one net at a time)
  POST /clear  -> remove all nets
  GET  /status -> launcher state and the net in the air
  GET  /params -> net parameters (EDITABLE_PARAMS) and derived masses
  POST /params -> {name: value, ...}: new parameters for the next shot
  POST /params/save -> write the current parameters to the `net:` section of the config

Telemetry for the HUD: /interceptor/net_telemetry (JSON list, one entry per net).
"""

import argparse
import ctypes
import json
import math
import multiprocessing
import os
import queue
import re
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import numpy as np
import yaml
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.wind_pb2 import Wind
from gz.transport13 import Node

from net_physics import Net, NetParams

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# turret geometry in the interceptor body frame (FLU, z along the fuselage towards the nose)
TURRET_PIVOT = np.array([0.042, 0.0, 0.3988])
BARREL_LENGTH = 0.095

RENDER_RATE = 60.0         # marker / telemetry rate [Hz sim time]
MARKER_SERVICES = ("/sensors/marker_array", "/marker_array")
# the marker services may not answer within any useful time and a blocking gz request keeps
# the Python GIL while it waits: the markers are sent from their own process, which only
# waits this long per request (the markers are applied whether or not the answer arrives)
MARKER_TIMEOUT_MS = 5
GROUND_LINGER = 15.0       # keep a landed net this long [s]
MAX_LIFETIME = 40.0        # [s]

CORD_COLOR = (0.08, 0.08, 0.08)
# Gazebo appends the points of a modified LINE_LIST marker to the old ones instead of
# replacing them (the cords leave a trail of old frames), so every frame draws the cords
# as a new marker, alternating between these ids, and deletes the previous one
CORD_IDS = (100, 101)
BALL_COLOR = (0.62, 0.63, 0.66)

# net parameters the control panel edits (tools/control_panel.py, Net tab)
EDITABLE_PARAMS = ("open_diameter", "mesh_size", "cord_diameter", "ball_diameter", "muzzle_speed", "splay_deg")


def quat_to_rot(w, x, y, z):
	return np.array([
		[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
		[2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
		[2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
	])


def load_net_params(path):
	try:
		with open(path) as f:
			cfg = yaml.safe_load(f) or {}
	except OSError:
		cfg = {}
	return NetParams(**(cfg.get("net") or {}))


def save_net_params(path, params, keys=EDITABLE_PARAMS):
	"""Write the values of `keys` into the `net:` section of the config, keeping its layout and
	comments (missing keys are appended to the section)."""
	with open(path) as f:
		lines = f.read().split("\n")

	section, last, done = False, None, set()
	for i, line in enumerate(lines):
		if re.match(r"\S", line):
			section = line.startswith("net:")
			if section:
				last = i
			continue
		if not section:
			continue
		if line.strip():
			last = i
		m = re.match(r"^(\s+)(\w+):(\s*)([^\s#]+)(.*)$", line)
		if m and m.group(2) in keys:
			lines[i] = f"{m.group(1)}{m.group(2)}:{m.group(3)}{getattr(params, m.group(2)):g}{m.group(5)}"
			done.add(m.group(2))

	if last is None:
		lines += ["", "net:"]
		last = len(lines) - 1
	missing = [f"  {k}: {getattr(params, k):g}" for k in keys if k not in done]
	lines[last + 1:last + 1] = missing

	with open(path, "w") as f:
		f.write("\n".join(lines))


def marker_process(draw_queue):
	"""Sends the marker frames of the launcher to Gazebo: the latest frame, deletions are never
	dropped. Each frame draws the cords as a new marker (see CORD_IDS)."""
	try:
		ctypes.CDLL("libc.so.6").prctl(1, signal.SIGTERM)    # PR_SET_PDEATHSIG: end with the launcher
	except OSError:
		pass
	node = Node()
	node.service_list()    # the first call waits ~2 s for discovery: not on the first shot
	services, services_checked = [], 0.0
	cord_id = {}     # ns -> cord marker id currently shown
	while True:
		items = [draw_queue.get()]
		while True:
			try:
				items.append(draw_queue.get_nowait())
			except queue.Empty:
				break
		# all deletions and the latest frame, without the nets deleted after that frame
		deletes, frame, deleted_since = [], None, set()
		for kind, data in items:
			req = Marker_V()
			req.ParseFromString(data)
			if kind == "delete":
				deletes.extend(req.marker)
				deleted_since.update(m.ns for m in req.marker)
			else:
				frame, deleted_since = req, set()
		markers = deletes + ([m for m in frame.marker if m.ns not in deleted_since] if frame else [])

		stale = []
		for m in markers:
			if m.action == Marker.DELETE_MARKER:
				cord_id.pop(m.ns, None)
			elif m.type == Marker.LINE_LIST:
				old = cord_id.get(m.ns)
				m.id = CORD_IDS[1] if old == CORD_IDS[0] else CORD_IDS[0]
				cord_id[m.ns] = m.id
				if old is not None:
					d = Marker()
					d.ns, d.id, d.action = m.ns, old, Marker.DELETE_MARKER
					stale.append(d)

		now = time.monotonic()
		if now - services_checked > 2.0:
			services_checked = now
			available = set(node.service_list())
			services = [s for s in MARKER_SERVICES if s in available]

		req = Marker_V()
		req.marker.extend(stale + markers)
		for service in services:
			try:
				node.request(service, req, Marker_V, Boolean, MARKER_TIMEOUT_MS)
			except Exception:
				pass


class LaunchedNet:
	def __init__(self, net_id, net, t_launch):
		self.net_id = net_id
		self.net = net
		self.t_launch = t_launch
		self.t_ground = None
		self.lag_warned = False

	@property
	def name(self):
		return f"net_{self.net_id}"


class NetLauncher:
	def __init__(self, world, interceptor, http_port, params, config_path):
		self.world = world
		self.interceptor = interceptor
		self.params = params
		self.config_path = config_path

		self.node = Node()
		self.lock = threading.Lock()
		self.sim_time = 0.0

		self.int_pose = None
		self.int_vel = np.zeros(3)
		self.prev_int_pos = None
		self.prev_int_time = None
		self.turret_pitch_deg = 0.0
		self.trigger_active = False

		self.launch_count = 0
		self.nets = []
		self.rng = np.random.default_rng()

		Net(params, np.zeros(3), np.array([1.0, 0.0, 0.0]), np.zeros(3)).step()   # compile the kernel now, not on the first shot
		self.telem_pub = self.node.advertise("/interceptor/net_telemetry", StringMsg)
		# markers go through blocking service calls: a drawing process sends the latest frame
		self.draw_queue = multiprocessing.get_context("spawn").Queue()
		multiprocessing.get_context("spawn").Process(target=marker_process, args=(self.draw_queue,),
		                                             daemon=True).start()

		self.node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)
		self.node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", self._on_pose)
		self.node.subscribe(Double, f"/model/{interceptor}/servo_1", self._on_turret_servo)
		self.node.subscribe(Double, f"/model/{interceptor}/command/turret_pitch", self._on_turret_servo)
		self.node.subscribe(Double, f"/model/{interceptor}/servo_2", self._on_trigger_servo)
		self.current_wind = np.zeros(3, dtype=float)
		self.node.subscribe(Wind, f"/world/{world}/wind", self._on_wind)

		self.server = ThreadingHTTPServer(("0.0.0.0", http_port), self._make_handler())
		threading.Thread(target=self.server.serve_forever, daemon=True).start()
		p = params
		print(f"NetLauncher: HTTP port {http_port}; net {p.open_diameter:.1f} m, {p.net_mass * 1000:.0f} g "
		      f"(cord {p.cord_diameter * 1000:.2f} mm), 6 x {p.ball_mass * 1000:.1f} g balls, "
		      f"{p.muzzle_speed:.0f} m/s, splay {p.splay_deg:.0f} deg", flush=True)

	# --- Gazebo input ---

	def _on_clock(self, msg):
		with self.lock:
			self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

	def _on_pose(self, msg):
		with self.lock:
			for p in msg.pose:
				if p.name == self.interceptor:
					self.int_pose = p
					pos = np.array([p.position.x, p.position.y, p.position.z])
					if self.prev_int_pos is not None:
						dt = self.sim_time - self.prev_int_time
						if 0.001 < dt < 0.2:
							self.int_vel = (pos - self.prev_int_pos) / dt
					if self.prev_int_pos is None or self.sim_time > self.prev_int_time:
						self.prev_int_pos = pos
						self.prev_int_time = self.sim_time
					break

	def _on_turret_servo(self, msg):
		with self.lock:
			self.turret_pitch_deg = math.degrees(msg.data)

	def _on_trigger_servo(self, msg):
		should_fire = False
		with self.lock:
			if msg.data > 0.5:
				if not self.trigger_active:
					self.trigger_active = True
					should_fire = True
			else:
				self.trigger_active = False

		if should_fire:
			print(f"NetLauncher: Trigger signal received from PX4 servo_2 ({msg.data:.2f} rad), firing net!", flush=True)
			self.fire()

	def _on_wind(self, msg):
		with self.lock:
			self.current_wind = np.array([msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z], dtype=float)

	# --- commands ---

	def fire(self):
		with self.lock:
			if self.int_pose is None:
				return {"success": False, "error": "interceptor pose not yet received"}

			p = self.int_pose
			p_int = np.array([p.position.x, p.position.y, p.position.z])
			R_int = quat_to_rot(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)

			# barrel tilted about body +y by the turret pitch
			pitch = math.radians(self.turret_pitch_deg)
			u_barrel = np.array([math.sin(pitch), 0.0, math.cos(pitch)])
			muzzle = p_int + R_int @ (TURRET_PIVOT + BARREL_LENGTH * u_barrel)
			axis = R_int @ u_barrel

			net = Net(self.params, muzzle, axis, self.int_vel, self.rng)
			self.launch_count += 1
			launched = LaunchedNet(self.launch_count, net, self.sim_time)
			# one net at a time, like the real launcher: a new shot removes the previous net
			for n in self.nets:
				self._delete_markers(n)
			self.nets = [launched]
			turret = self.turret_pitch_deg
			v_int = np.linalg.norm(self.int_vel)

		print(f"Net #{launched.net_id} fired: turret {turret:.1f} deg, vehicle {v_int:.1f} m/s", flush=True)
		return {
			"success": True,
			"net_id": launched.net_id,
			"speed_mps": self.params.muzzle_speed,
			"turret_pitch_deg": round(turret, 1),
			"muzzle_pos": [round(float(x), 2) for x in muzzle],
		}

	def params_info(self):
		with self.lock:
			p = self.params
		return {
			"params": {k: getattr(p, k) for k in EDITABLE_PARAMS},
			"derived": {
				"net_mass_kg": p.net_mass,
				"ball_mass_kg": p.ball_mass,
				"cord_length_m": p.cord_length,
			},
		}

	def set_params(self, values):
		"""New net parameters for the next shot (a net in the air keeps its own)."""
		values = {k: float(v) for k, v in values.items() if k in EDITABLE_PARAMS}
		if any(not math.isfinite(v) or v <= 0.0 for v in values.values()) or values.get("splay_deg", 0.0) >= 90.0:
			raise ValueError("net parameters must be positive, splay below 90 deg")
		with self.lock:
			self.params = NetParams(**{**vars(self.params), **values})
			p = self.params
		print(f"NetLauncher: parameters {', '.join(f'{k}={getattr(p, k):g}' for k in EDITABLE_PARAMS)}, "
		      f"net {p.net_mass * 1000:.0f} g", flush=True)
		return self.params_info()

	def save_params(self):
		with self.lock:
			p = self.params
		save_net_params(self.config_path, p)
		print(f"NetLauncher: parameters saved to {self.config_path}", flush=True)
		return {"saved": True, "path": self.config_path}

	def status(self):
		with self.lock:
			data = {
				"ready": self.int_pose is not None,
				"launch_count": self.launch_count,
				"active_count": len(self.nets),
				"turret_pitch_deg": round(self.turret_pitch_deg, 1),
				"net": None,
			}
			if self.nets:
				n = self.nets[-1]
				data["net"] = {
					"id": n.net_id,
					"t": round(n.net.t, 2),
					"phase": n.net.phase(),
					"dist": round(float(np.linalg.norm(n.net.centroid() - n.net.muzzle)), 1),
					"spread": round(n.net.spread(), 2),
					"speed": round(float(np.linalg.norm(n.net.velocity())), 1),
				}
		return data

	def clear_all(self):
		with self.lock:
			for n in self.nets:
				self._delete_markers(n)
			self.nets = []

	# --- drawing ---

	def _marker(self, launched, marker_id, mtype, color):
		m = Marker()
		m.ns = launched.name
		m.id = marker_id
		m.action = Marker.ADD_MODIFY
		m.type = mtype
		m.visibility = Marker.ALL
		m.lifetime.sec = 2
		m.material.ambient.r, m.material.ambient.g, m.material.ambient.b = color
		m.material.ambient.a = 1.0
		m.material.diffuse.r, m.material.diffuse.g, m.material.diffuse.b = color
		m.material.diffuse.a = 1.0
		return m

	def _draw(self, launched):
		net = launched.net
		cords = self._marker(launched, CORD_IDS[0], Marker.LINE_LIST, CORD_COLOR)   # id set when sent
		for i, j in net.edges:
			for k in (i, j):
				pt = cords.point.add()
				pt.x, pt.y, pt.z = (float(c) for c in net.x[k])
		msgs = [cords]

		d = float(self.params.ball_diameter)
		for n, k in enumerate(net.corners):
			ball = self._marker(launched, 1 + n, Marker.SPHERE, BALL_COLOR)
			ball.material.specular.r = ball.material.specular.g = ball.material.specular.b = 0.9
			ball.material.specular.a = 1.0
			ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = (float(c) for c in net.x[k])
			ball.pose.orientation.w = 1.0
			ball.scale.x = ball.scale.y = ball.scale.z = d
			msgs.append(ball)

		return msgs

	def _delete_markers(self, launched):
		req = Marker_V()
		for marker_id in (*range(1, 7), *CORD_IDS):
			m = req.marker.add()
			m.ns = launched.name
			m.id = marker_id
			m.action = Marker.DELETE_MARKER
		self.draw_queue.put(("delete", req.SerializeToString()))

	def _submit_frame(self, markers):
		req = Marker_V()
		req.marker.extend(markers)
		self.draw_queue.put(("frame", req.SerializeToString()))

	def _telemetry(self, launched):
		net = launched.net
		c = net.centroid()
		return {
			"id": launched.net_id,
			"name": launched.name,
			"t": round(net.t, 3),
			"pos": [round(float(v), 3) for v in c],
			"radius": round(net.spread() / 2.0, 3),
			"phase": net.phase(),
			"dist": round(float(np.linalg.norm(c - net.muzzle)), 2),
			"speed": round(float(np.linalg.norm(net.velocity())), 2),
			"nodes": np.round(net.x, 3).tolist(),
			"corners": [int(k) for k in net.corners],
			"edges": net.edges.tolist(),
		}

	# --- main loop ---

	def run(self):
		last_render = -1.0
		while True:
			time.sleep(0.005)
			with self.lock:
				t_sim = self.sim_time
				nets = list(self.nets)

			# integrate every net up to the current sim time (bounded catch-up per loop)
			for n in nets:
				lag = t_sim - (n.t_launch + n.net.t)
				n.net.advance(min(lag, 0.1), wind=self.current_wind)
				if lag > 0.2 and not n.lag_warned:
					n.lag_warned = True
					print(f"NetLauncher: net #{n.net_id} physics {lag:.2f} s behind sim time "
					      f"({len(nets)} nets in flight)", flush=True)
				if n.t_ground is None and n.net.on_ground():
					n.t_ground = n.net.t

			expired = [n for n in nets
			           if n.net.t > MAX_LIFETIME or (n.t_ground is not None and n.net.t - n.t_ground > GROUND_LINGER)]
			for n in expired:
				self._delete_markers(n)
				nets.remove(n)
			if expired:
				with self.lock:
					self.nets = [n for n in self.nets if n not in expired]

			rate = RENDER_RATE if nets else 5.0
			if t_sim - last_render >= 1.0 / rate or t_sim < last_render:
				last_render = t_sim
				markers = [m for n in nets for m in self._draw(n)]
				with self.lock:
					# a shot or /clear may have removed nets meanwhile: do not draw them again
					live = {n.name for n in self.nets}
					nets = [n for n in nets if n.name in live]
					markers = [m for m in markers if m.ns in live]
					if markers:
						self._submit_frame(markers)
				msg = StringMsg()
				msg.data = json.dumps([self._telemetry(n) for n in nets])
				self.telem_pub.publish(msg)

	# --- HTTP ---

	def _make_handler(self):
		launcher = self

		class Handler(BaseHTTPRequestHandler):
			def log_message(self, *args):
				pass

			def _send_json(self, data, code=200):
				body = json.dumps(data).encode()
				self.send_response(code)
				self.send_header("Content-Type", "application/json")
				self.send_header("Access-Control-Allow-Origin", "*")
				self.send_header("Content-Length", str(len(body)))
				self.end_headers()
				self.wfile.write(body)

			def do_OPTIONS(self):
				self.send_response(200)
				self.send_header("Access-Control-Allow-Origin", "*")
				self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
				self.send_header("Access-Control-Allow-Headers", "Content-Type")
				self.end_headers()

			def do_GET(self):
				path = urlparse(self.path).path
				if path in ("/", "/status"):
					self._send_json(launcher.status())
				elif path == "/params":
					self._send_json(launcher.params_info())
				elif path == "/fire":
					self._send_json(launcher.fire())
				else:
					self._send_json({"error": "not found"}, 404)

			def do_POST(self):
				path = urlparse(self.path).path
				if path == "/fire":
					self._send_json(launcher.fire())
				elif path == "/clear":
					launcher.clear_all()
					self._send_json({"cleared": True})
				elif path == "/params":
					try:
						length = int(self.headers.get("Content-Length", 0))
						values = json.loads(self.rfile.read(length) or b"{}")
						self._send_json(launcher.set_params(values))
					except (ValueError, TypeError, AttributeError) as e:
						self._send_json({"error": str(e)}, 400)
				elif path == "/params/save":
					try:
						self._send_json(launcher.save_params())
					except OSError as e:
						self._send_json({"error": str(e)}, 500)
				else:
					self._send_json({"error": "not found"}, 404)

		return Handler


def main():
	ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("--world", default="ankara", help="Gazebo world name")
	ap.add_argument("--interceptor", default="interceptor_0", help="interceptor model name")
	ap.add_argument("--http-port", type=int, default=8003, help="HTTP API port")
	ap.add_argument("--config", default=os.path.join(SIM_DIR, "interceptor.yaml"),
	                help="vehicle config with the `net:` section")
	a = ap.parse_args()

	launcher = NetLauncher(a.world, a.interceptor, a.http_port, load_net_params(a.config), a.config)
	launcher.run()


if __name__ == "__main__":
	main()

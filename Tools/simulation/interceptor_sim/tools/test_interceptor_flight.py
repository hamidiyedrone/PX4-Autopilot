#!/usr/bin/env python3
"""
Headless flight check of the interceptor: private gz partition + PX4 SITL
instance (default 6, so a running simulation is not touched).

1. arm, take off to --alt, hover: altitude hold and attitude noise
2. offboard velocity --speed m/s forward: reached speed, tilt, altitude hold

    python3 tools/test_interceptor_flight.py [--speed 20] [--alt 5]
"""

import argparse
import math
import os
import subprocess
import sys
import time

from pymavlink import mavutil

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BUILD_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", "..", "build", "px4_sitl_default"))
WORLD = "ankara"


def gz_env(partition):
	env = dict(os.environ)
	out = subprocess.run(["bash", "-c", f". {BUILD_DIR}/rootfs/gz_env.sh && env"], capture_output=True, text=True).stdout
	env.update(line.split("=", 1) for line in out.splitlines() if "=" in line)
	env["GZ_SIM_RESOURCE_PATH"] = f"{SIM_DIR}/models:{SIM_DIR}/worlds:" + env.get("GZ_SIM_RESOURCE_PATH", "")
	env["GZ_PARTITION"] = partition
	env["GZ_IP"] = "127.0.0.1"
	return env


class Vehicle:
	def __init__(self, port):
		self.m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}")
		self.state = {}

	def poll(self, duration=0.0):
		"""Read messages for `duration` seconds (at least the ones already queued)."""
		t_end = time.time() + duration

		for _ in range(100000):
			msg = self.m.recv_match(blocking=False)

			if msg is not None:
				self.state[msg.get_type()] = msg
				continue

			if time.time() >= t_end:
				return

			time.sleep(0.002)

	def command(self, cmd, *params):
		p = list(params) + [0] * (7 - len(params))
		self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)

	def velocity(self, vx, vy, vz, yaw):
		# local NED velocity + yaw, everything else ignored
		mask = 0b0000_1001_1100_0111
		self.m.mav.set_position_target_local_ned_send(0, self.m.target_system, self.m.target_component,
							      mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
							      0, 0, 0, vx, vy, vz, 0, 0, 0, yaw, 0)

	def sample(self):
		lp, att = self.state.get("LOCAL_POSITION_NED"), self.state.get("ATTITUDE")
		return lp, att


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--speed", type=float, default=20.0)
	ap.add_argument("--alt", type=float, default=5.0)
	ap.add_argument("--instance", type=int, default=6)
	a = ap.parse_args()

	work = os.path.join(SIM_DIR, "build", "interceptor_test")
	os.makedirs(os.path.join(work, "instance"), exist_ok=True)
	env = gz_env(f"icpt_test_{os.getpid()}")
	gz = subprocess.Popen(["gz", "sim", "-s", "-r", "--verbose=1", f"{SIM_DIR}/worlds/{WORLD}.sdf"],
			      env=env, stdout=open(os.path.join(work, "gz.log"), "w"), stderr=subprocess.STDOUT)
	px4 = None

	try:
		name = f"interceptor_{a.instance}"

		for _ in range(60):
			r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/create", "--reqtype", "gz.msgs.EntityFactory",
					    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
					    f'name: "{name}", sdf: \'<sdf version="1.9"><include><uri>model://interceptor</uri>'
					    f'<pose>0 0 0.22 0 0 0</pose></include></sdf>\''], env=env, capture_output=True, text=True)

			if "data: true" in r.stdout:
				break

			time.sleep(1)
		else:
			sys.exit("could not spawn the interceptor")

		penv = dict(env, PX4_SYS_AUTOSTART="4051", PX4_GZ_STANDALONE="1", PX4_GZ_WORLD=WORLD, PX4_GZ_MODEL_NAME=name)
		px4 = subprocess.Popen([f"{BUILD_DIR}/bin/px4", "-i", str(a.instance), "-d", f"{BUILD_DIR}/etc"],
				       cwd=os.path.join(work, "instance"), env=penv,
				       stdout=open(os.path.join(work, "px4.log"), "w"), stderr=subprocess.STDOUT)

		v = Vehicle(14540 + a.instance)
		v.m.wait_heartbeat(timeout=60)
		print("heartbeat, waiting for position estimate")
		t0 = time.time()

		while time.time() - t0 < 60:
			v.poll(0.5)

			if v.state.get("LOCAL_POSITION_NED") is not None and time.time() - t0 > 8:
				break

		# arm (retry until the preflight checks pass) and take off
		for _ in range(30):
			v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
			v.poll(1.0)
			hb = v.state.get("HEARTBEAT")

			if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
				break
		else:
			sys.exit("arming failed, see build/interceptor_test/px4.log")

		# altitude NaN: PX4 takes MIS_TAKEOFF_ALT (relative to home)
		v.m.mav.param_set_send(v.m.target_system, v.m.target_component, b"MIS_TAKEOFF_ALT", a.alt,
				       mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
		v.poll(0.5)
		nan = float("nan")
		v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan)
		v.poll(8.0)

		alts, tilts = [], []
		t0 = time.time()

		while time.time() - t0 < 8:
			v.poll(0.2)
			lp, att = v.sample()
			alts.append(-lp.z)
			tilts.append(math.degrees(math.acos(math.cos(att.roll) * math.cos(att.pitch))))

		print(f"hover:   altitude {sum(alts) / len(alts):5.2f} m (min {min(alts):.2f}, max {max(alts):.2f}), "
		      f"tilt mean {sum(tilts) / len(tilts):.1f} deg")

		# offboard forward velocity (north), keep altitude
		yaw = v.state["ATTITUDE"].yaw

		for _ in range(30):  # stream setpoints before switching
			v.velocity(0, 0, 0, yaw)
			time.sleep(0.05)

		v.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6)
		t0 = time.time()
		print("   t   speed  tilt  alt")

		while time.time() - t0 < 12:
			vx = a.speed * math.cos(yaw)
			vy = a.speed * math.sin(yaw)
			v.velocity(vx, vy, 0, yaw)
			v.poll(0.05)
			t = time.time() - t0

			if abs(t - round(t)) < 0.03:
				lp, att = v.sample()
				print(f"{t:4.0f}  {math.hypot(lp.vx, lp.vy):5.1f}  {math.degrees(math.acos(math.cos(att.roll) * math.cos(att.pitch))):4.0f}"
				      f"  {-lp.z:5.1f}")

		hb = v.state.get("HEARTBEAT")
		print("mode", hb.custom_mode if hb else "?", " log dir:", os.path.join(work, "instance", "log"))
	finally:
		if px4:
			px4.terminate()

		gz.terminate()

		for p in (px4, gz):
			if p:
				try:
					p.wait(5)
				except subprocess.TimeoutExpired:
					p.kill()


if __name__ == "__main__":
	main()

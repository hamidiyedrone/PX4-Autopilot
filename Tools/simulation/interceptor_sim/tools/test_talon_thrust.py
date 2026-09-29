#!/usr/bin/env python3
"""
Headless check that the Talon can accelerate on the runway: starts its own
gz server in a private GZ_PARTITION (does not touch a running simulation),
spawns talon1718 at the world's spawn pose, commands full throttle and
prints the ground distance covered and the height over time.

    python3 tools/test_talon_thrust.py [--world ankara] [--speed 1600] [--time 8]
"""

import argparse
import math
import os
import subprocess
import sys
import time

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BUILD_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", "..", "build", "px4_sitl_default"))


def gz_env():
	env = dict(os.environ)
	out = subprocess.run(["bash", "-c", f". {BUILD_DIR}/rootfs/gz_env.sh && env"], capture_output=True, text=True).stdout
	env.update(line.split("=", 1) for line in out.splitlines() if "=" in line)
	env["GZ_SIM_RESOURCE_PATH"] = f"{SIM_DIR}/models:{SIM_DIR}/worlds:" + env.get("GZ_SIM_RESOURCE_PATH", "")
	env["GZ_PARTITION"] = f"thrust_test_{os.getpid()}"
	env["GZ_IP"] = "127.0.0.1"
	return env


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--world", default="ankara")
	ap.add_argument("--speed", type=float, default=1600.0)
	ap.add_argument("--time", type=float, default=8.0)
	a = ap.parse_args()

	env = gz_env()
	os.environ.update({k: env[k] for k in ("GZ_PARTITION", "GZ_IP")})  # for our own Node
	log = open(os.path.join(SIM_DIR, "build", "thrust_test_server.log"), "w")
	server = subprocess.Popen(["gz", "sim", "-s", "-r", "--verbose=3", f"{SIM_DIR}/worlds/{a.world}.sdf"],
				  env=env, stdout=log, stderr=subprocess.STDOUT)

	try:
		node = Node()
		poses = {}

		def on_pose(msg):
			for p in msg.pose:
				poses[p.name] = (p.position.x, p.position.y, p.position.z)

		node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)

		pose_env = open(f"{SIM_DIR}/worlds/{a.world}.env").read().split("=", 1)[1].strip()
		x, y, z, r, p, yaw = pose_env.split(",")
		req = EntityFactory()
		req.name = "talon1718_1"
		req.sdf = (f'<sdf version="1.9"><include><uri>model://talon1718</uri>'
			   f'<pose>{x} {y} {z} {r} {p} {yaw}</pose></include></sdf>')

		for _ in range(60):
			ok, rep = node.request(f"/world/{a.world}/create", req, EntityFactory, Boolean, 2000)

			if ok and rep.data:
				break

			time.sleep(1)
		else:
			sys.exit("could not spawn talon1718 (see build/thrust_test_server.log)")

		pub = node.advertise("/talon1718_1/command/motor_speed", Actuators)
		for _ in range(100):
			if "talon1718_1" in poses:
				break

			time.sleep(0.1)
		else:
			sys.exit(f"no pose for talon1718_1, got: {sorted(poses)[:10]}")

		time.sleep(2)  # settle on the skids
		start = poses["talon1718_1"]
		print(f"spawned at {start}")
		cmd = Actuators()
		cmd.velocity.append(a.speed)
		t0 = time.time()
		next_print = 0.0

		while time.time() - t0 < a.time:
			pub.publish(cmd)
			t = time.time() - t0

			if t >= next_print:
				cur = poses.get("talon1718_1", start)
				dist = math.hypot(cur[0] - start[0], cur[1] - start[1])
				print(f"t={t:4.1f}s  ground distance {dist:6.2f} m  height {cur[2] - start[2]:+6.2f} m")
				next_print += 1.0

			time.sleep(0.02)

	finally:
		server.terminate()

		try:
			server.wait(5)
		except subprocess.TimeoutExpired:
			server.kill()


if __name__ == "__main__":
	main()

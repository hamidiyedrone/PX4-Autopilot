#!/usr/bin/env python3
"""
Ground station relay: target position from the competition server to the interceptor.

Polls the target JSON (format of tools/target_sim.py, GET /target) and sends every new
sample to the interceptor as MAVLink FOLLOW_TARGET, which PX4 publishes as the
follow_target uORB topic (mavlink_receiver). A sample is sent once: PX4 stamps
follow_target with the reception time, so resending an old sample would look fresh.

The server rate is low (2 Hz) and the position is not stamped on board, so the flight
mode has to predict the target forward from the last sample and its velocity.

    python3 tools/ground_relay.py [--url http://localhost:8000/target] [--link udpout:127.0.0.1:14580]

--link is any pymavlink connection string: in SITL the interceptor's onboard link
(udp 14580), on the field the telemetry radio (e.g. /dev/ttyUSB0,57600) or a
mavlink-router endpoint shared with QGroundControl.
"""

import argparse
import json
import math
import time
import urllib.request

from pymavlink import mavutil

mav = mavutil.mavlink

# FOLLOW_TARGET est_capabilities bits
CAP_POS, CAP_VEL, CAP_ATT = 1, 2, 8


def euler_to_quat(roll, pitch, yaw):
	"""NED/FRD attitude quaternion [w, x, y, z] from roll, pitch, yaw [rad]."""
	cr, sr = math.cos(roll / 2), math.sin(roll / 2)
	cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
	cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
	return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def fetch(url, timeout):
	with urllib.request.urlopen(url, timeout=timeout) as r:
		return json.loads(r.read())


def send_follow_target(link, d, t_boot_ms):
	caps = CAP_POS
	vel = [0.0, 0.0, 0.0]
	att = [1.0, 0.0, 0.0, 0.0]

	if all(k in d for k in ("vn_mps", "ve_mps", "vd_mps")):
		vel = [d["vn_mps"], d["ve_mps"], d["vd_mps"]]
		caps |= CAP_VEL

	if all(k in d for k in ("roll_deg", "pitch_deg", "heading_deg")):
		att = euler_to_quat(math.radians(d["roll_deg"]), math.radians(d["pitch_deg"]), math.radians(d["heading_deg"]))
		caps |= CAP_ATT

	link.mav.follow_target_send(t_boot_ms, caps, int(round(d["lat"] * 1e7)), int(round(d["lon"] * 1e7)),
				    float(d["alt_amsl_m"]), vel, [0.0, 0.0, 0.0], att, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0)


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--url", default="http://localhost:8000/target", help="target JSON")
	ap.add_argument("--link", default="udpout:127.0.0.1:14580", help="MAVLink connection to the interceptor")
	ap.add_argument("--poll", type=float, default=10.0, help="server poll rate [Hz]")
	ap.add_argument("--stale", type=float, default=2.0, help="warn when no new sample for this long [s]")
	ap.add_argument("--sysid", type=int, default=250, help="MAVLink system id of the relay")
	a = ap.parse_args()

	link = mavutil.mavlink_connection(a.link, source_system=a.sysid, source_component=mav.MAV_COMP_ID_USER1)
	t0 = time.monotonic()
	last_key, last_new = None, time.monotonic()
	sent, errors, last_print, stale_warned = 0, 0, t0, False
	print(f"relay: {a.url} -> FOLLOW_TARGET on {a.link}", flush=True)

	while True:
		time.sleep(1.0 / a.poll)
		now = time.monotonic()

		try:
			d = fetch(a.url, timeout=0.5)
		except (OSError, ValueError) as e:
			errors += 1
			if errors == 1 or errors % 50 == 0:
				print(f"server error ({errors}): {e}", flush=True)
			continue

		if not d.get("valid", True) or "lat" not in d:
			continue

		# a new sample has a new timestamp; the whole record when the server gives none
		key = d.get("utc", d.get("sim_time_s", json.dumps(d, sort_keys=True)))

		if key != last_key:
			send_follow_target(link, d, int((now - t0) * 1000))
			last_key, last_new, stale_warned = key, now, False
			sent += 1

		elif now - last_new > a.stale and not stale_warned:
			print(f"no new target sample for {now - last_new:.1f} s", flush=True)
			stale_warned = True

		if now - last_print >= 10:
			print(f"sent {sent}  lat {d['lat']:.6f} lon {d['lon']:.6f} alt {d['alt_amsl_m']:.1f} m  "
			      f"v {d.get('ground_speed_mps', float('nan')):.1f} m/s", flush=True)
			last_print = now


if __name__ == "__main__":
	main()

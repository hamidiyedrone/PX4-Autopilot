#!/usr/bin/env python3
"""
Render a test video of an interceptor approach on the Talon target, with the
same camera, labels and pose files as tools/gen_dataset.py (one per frame).

The Talon flies straight and level at --speed. The camera starts --d-start
behind and --el-start below it, closes in (constant relative closing rate,
so the target grows evenly), weaves between the lower right and lower left
(--weave degrees, --weaves full cycles) and settles exactly behind the Talon
at --d-end for the last --settle seconds. The camera always looks at the
target, with a gentle roll like a manoeuvring drone.

Runs its own headless gz server in a private GZ_PARTITION.

    python3 tools/gen_test_video.py                   # 60 s, 30 fps
    python3 tools/gen_test_video.py --stride 6        # quick preview: every 6th frame (10 s time lapse)

Output (<out>):
    test.mp4, labels/<frame>.txt (YOLO pose), poses.jsonl, camera.json, keypoints_3d.json
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

from gen_dataset import (CAMERA, FLIP_IDX, KEYPOINTS, SIM_DIR, TARGET, Sim, bounding_box, keypoint_visibility,
			 look_rotation, project, relative_pose_cv, rot_rpy, rot_to_quat, talon_geometry)


def smoothstep(x):
	x = min(1.0, max(0.0, x))
	return x * x * (3 - 2 * x)


def scene_at(t, a):
	"""Target and camera pose at time t of the approach."""
	target_pos = np.array([-a.speed * a.duration / 2 + a.speed * t, 0.0, a.alt])
	R_t = rot_rpy(0.0, math.radians(-2.0), 0.0)  # level cruise, slight nose-up trim

	s = t / a.duration
	settle = smoothstep((t - (a.duration - a.settle - a.blend)) / a.blend)  # 0 -> 1 into the final stretch
	dist = a.d_start * (a.d_end / a.d_start) ** min(1.0, t / (a.duration - a.settle))
	az = math.radians(a.weave) * math.sin(2 * math.pi * a.weaves * s) * (1 - settle)
	el = math.radians(a.el_start) * (1 - settle)

	# direction target -> camera: behind (-x), weave to the side, below
	d = np.array([-math.cos(el) * math.cos(az), -math.cos(el) * math.sin(az), math.sin(el)])
	cam_pos = target_pos + d * dist
	roll = math.radians(a.roll) * math.sin(2 * math.pi * a.weaves * s + 0.6) * (1 - settle)
	R_c = look_rotation(target_pos - cam_pos, roll)

	return dict(target_pos=target_pos, R_t=R_t, cam_pos=cam_pos, R_c=R_c, dist=dist,
		    az=math.degrees(az), el=math.degrees(el))


def encoder(path, width, height, fps):
	"""gst-launch reading raw RGB frames on stdin and writing an H.264 mp4."""
	return subprocess.Popen(
		["gst-launch-1.0", "-q", "-e", "fdsrc", "fd=0", f"blocksize={width * height * 3}",
		 "!", "rawvideoparse", f"width={width}", f"height={height}", "format=rgb", f"framerate={fps}/1",
		 "!", "videoconvert", "!", "x264enc", "bitrate=10000", "speed-preset=medium", "key-int-max=30",
		 "!", "video/x-h264,profile=high", "!", "h264parse", "!", "mp4mux", "!", "filesink", f"location={path}"],
		stdin=subprocess.PIPE)


def main():
	ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("--out", default=os.path.join(SIM_DIR, "build", "test_video"))
	ap.add_argument("--world", default="ankara")
	ap.add_argument("--width", type=int, default=1280)
	ap.add_argument("--height", type=int, default=720)
	ap.add_argument("--hfov", type=float, default=60.0, help="horizontal field of view [deg]")
	ap.add_argument("--fps", type=int, default=30)
	ap.add_argument("--duration", type=float, default=60.0, help="[s]")
	ap.add_argument("--stride", type=int, default=1, help="render every n-th frame (preview)")
	ap.add_argument("--speed", type=float, default=25.0, help="target speed [m/s]")
	ap.add_argument("--alt", type=float, default=100.0, help="target height [m]")
	ap.add_argument("--d-start", type=float, default=150.0, help="start distance [m]")
	ap.add_argument("--d-end", type=float, default=5.0, help="final distance, right behind the target [m]")
	ap.add_argument("--el-start", type=float, default=-30.0, help="camera elevation seen from the target [deg]")
	ap.add_argument("--weave", type=float, default=45.0, help="lower right / lower left swing [deg]")
	ap.add_argument("--weaves", type=float, default=1.5, help="full right-left cycles over the video")
	ap.add_argument("--roll", type=float, default=10.0, help="camera roll swing [deg]")
	ap.add_argument("--settle", type=float, default=6.0, help="final seconds exactly behind the target")
	ap.add_argument("--blend", type=float, default=8.0, help="seconds to move from the weave to behind")
	a = ap.parse_args()

	points, kp3d = talon_geometry()
	f = a.width / 2 / math.tan(math.radians(a.hfov) / 2)
	K = {"width": a.width, "height": a.height, "fx": f, "fy": f, "cx": a.width / 2, "cy": a.height / 2,
	     "dist": [0.0] * 5, "hfov_deg": a.hfov}

	os.makedirs(os.path.join(a.out, "labels"), exist_ok=True)

	with open(os.path.join(a.out, "camera.json"), "w") as fh:
		json.dump(K, fh, indent=1)

	with open(os.path.join(a.out, "keypoints_3d.json"), "w") as fh:
		json.dump({"frame": "Talon body frame: origin CG, x nose, y left wing, z up [m]",
			   "names": KEYPOINTS, "flip_idx": FLIP_IDX,
			   "points": {k: [round(float(c), 4) for c in p] for k, p in zip(KEYPOINTS, kp3d)}}, fh, indent=1)

	frames = range(0, int(round(a.duration * a.fps)), a.stride)
	video = os.path.join(a.out, "test.mp4")
	sim = Sim(a.world, a.width, a.height, a.hfov, True, os.path.join(a.out, "gz_server.log"))
	enc = encoder(video, a.width, a.height, a.fps)
	poses = open(os.path.join(a.out, "poses.jsonl"), "w")
	t_start = time.time()

	try:
		for n, i in enumerate(frames):
			t = i / a.fps
			s = scene_at(t, a)

			for _ in range(5):
				if sim.set_pose(TARGET, s["target_pos"], s["R_t"]) and sim.set_pose(CAMERA, s["cam_pos"], s["R_c"]):
					rendered = sim.capture()

					if rendered is not None:
						break
			else:
				sys.exit(f"frame {i}: no image")

			rgb, depth_img = rendered
			enc.stdin.write(np.ascontiguousarray(rgb).tobytes())

			R, tr = relative_pose_cv(s)
			uv, z = project(points, R, tr, K)
			box = bounding_box(uv, z, K)
			kp_uv, kp_z = project(kp3d, R, tr, K)
			vis = keypoint_visibility(kp_uv, kp_z, depth_img, K) if box is not None else [0] * len(KEYPOINTS)
			name = f"frame_{i:05d}"

			with open(os.path.join(a.out, "labels", name + ".txt"), "w") as fh:
				if box is not None:
					x0, y0, x1, y1 = box
					line = (f"0 {(x0 + x1) / 2 / a.width:.6f} {(y0 + y1) / 2 / a.height:.6f} "
						f"{(x1 - x0) / a.width:.6f} {(y1 - y0) / a.height:.6f}")

					for (u, v), vv in zip(kp_uv, vis):
						line += f" {u / a.width:.6f} {v / a.height:.6f} {vv}" if vv else " 0 0 0"

					fh.write(line + "\n")

			poses.write(json.dumps({
				"frame": i, "time_s": round(t, 4), "video_frame": n,
				"t_cam": [round(float(v), 4) for v in tr],
				"q_cam": [round(float(v), 6) for v in rot_to_quat(R)],
				"distance_m": round(s["dist"], 3), "azimuth_deg": round(s["az"], 2), "elevation_deg": round(s["el"], 2),
				"box_px": [round(float(v), 1) for v in box] if box else None,
				"keypoints_px": [[round(float(u), 2), round(float(v), 2), int(vv)] for (u, v), vv in zip(kp_uv, vis)],
				"target_world": {"p": [round(float(v), 4) for v in s["target_pos"]],
						 "q": [round(float(v), 6) for v in rot_to_quat(s["R_t"])]},
				"camera_world_gz": {"p": [round(float(v), 4) for v in s["cam_pos"]],
						    "q": [round(float(v), 6) for v in rot_to_quat(s["R_c"])]},
			}) + "\n")

			if (n + 1) % 100 == 0 or n + 1 == len(frames):
				rate = (n + 1) / (time.time() - t_start)
				print(f"{n + 1}/{len(frames)}  {rate:.1f} frame/s  t={t:.1f}s  d={s['dist']:.1f}m  "
				      f"az={s['az']:+.0f}  el={s['el']:+.0f}")
	finally:
		poses.close()
		enc.stdin.close()
		enc.wait()
		sim.close()

	print(f"video: {video} ({len(frames)} frames, {len(frames) / a.fps:.1f} s)")


if __name__ == "__main__":
	main()

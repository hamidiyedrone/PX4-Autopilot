#!/usr/bin/env python3
"""
Generate a labelled image dataset of the Talon target (YOLO pose or detect).

Runs its own headless gz server in a private GZ_PARTITION (does not touch a
running simulation). For every sample the Talon (static copy) gets a random
position and flight attitude, and a camera is placed around it at a
log-uniform random distance. Lighting / colour variation is left to the
training augmentation (changing the sun at runtime is not reliable in
Harmonic). Labels are exact: the Talon geometry is projected with the camera
model, no manual labelling.

Viewing directions (from the target to the camera, in the Talon body frame):
- never within --nose-cone degrees of the nose axis (no head-on views)
- --front-ratio of the images from the front hemisphere (camera on the nose
  side), the rest from the rear hemisphere; uniform on the sphere otherwise
- camera between --el-min and --el-max degrees of elevation relative to the
  target (world frame, i.e. the height difference), and no views of the --exclude-views groups (default
  top: seen from above in the Talon body frame, e.g. when it banks)

Keypoints (--task pose): nose, wing_l, wing_r, vtail_l, vtail_r, tail, taken
from the Talon meshes. Visibility per keypoint from a depth camera at the
same pose: 2 visible, 1 hidden behind the aircraft itself, 0 outside the
image.

Ground truth pose of every image goes to poses.jsonl, in OpenCV camera
convention (x right, y down, z forward); the target frame is the Talon body
frame (origin at the CG, x to the nose, y to the left wing, z up), the same
frame as keypoints_3d.json. camera.json has the pinhole intrinsics (no
distortion).

Image names carry the viewing direction group
(front / left / rear / right / top / bottom, none = no target in view).

    python3 tools/gen_dataset.py --count 2000 --out build/dataset
    python3 tools/gen_dataset.py --width 1920 --height 1080 --hfov 70 --dmin 5 --dmax 300

Output:
    <out>/images/{train,val}/<view>_<index>.jpg
    <out>/labels/{train,val}/<view>_<index>.txt
          pose:   0 cx cy w h  x1 y1 v1 ... x6 y6 v6   (normalised)
          detect: 0 cx cy w h
    <out>/poses.jsonl, <out>/camera.json, <out>/keypoints_3d.json
    <out>/meta.csv, <out>/data.yaml, <out>/README.md
"""

import argparse
import csv
import json
import math
import os
import random
import re
import struct
import subprocess
import sys
import time

import numpy as np
from PIL import Image

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.image_pb2 import Image as ImageMsg
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BUILD_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", "..", "build", "px4_sitl_default"))
MODEL_DIR = os.path.join(SIM_DIR, "models", "talon1718")

TARGET = "dataset_target"
CAMERA = "dataset_camera"
STEP = 0.004  # world max_step_size
CAMERA_RATE = 25.0

KEYPOINTS = ["nose", "wing_l", "wing_r", "vtail_l", "vtail_r", "tail"]
FLIP_IDX = [0, 2, 1, 4, 3, 5]

# gz camera frame (x forward, y left, z up) -> OpenCV camera frame (x right, y down, z forward)
GZ_TO_CV = np.array([[0.0, -1.0, 0.0],
		     [0.0, 0.0, -1.0],
		     [1.0, 0.0, 0.0]])


# ------------------------------------------------------------------ geometry ---

def rot_rpy(roll, pitch, yaw):
	cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
	return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
			 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
			 [-sp, cp * sr, cp * cr]])


def rot_to_quat(R):
	w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
	x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
	y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
	z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
	return w, x, y, z


def look_rotation(forward, roll):
	"""Camera rotation (gz camera looks along +x, y left, z up) for a view direction and roll."""
	f = forward / np.linalg.norm(forward)
	up = np.array([0.0, 0.0, 1.0]) if abs(f[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
	left = np.cross(up, f)
	left /= np.linalg.norm(left)
	up = np.cross(f, left)
	R = np.column_stack([f, left, up])
	return R @ rot_rpy(roll, 0, 0)


def read_stl(path):
	with open(path, "rb") as f:
		f.seek(80)
		n = struct.unpack("<I", f.read(4))[0]
		rec = np.frombuffer(f.read(n * 50), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
	return rec["v"].reshape(-1, 3).astype(np.float64)


def talon_geometry():
	"""All visual vertices in the model frame (for the box) and the 6 keypoints."""
	sdf = open(os.path.join(MODEL_DIR, "model.sdf")).read()
	link_pose = {m.group(1): np.array([float(v) for v in m.group(2).split()[:3]])
		     for m in re.finditer(r'<link name="(\w+)">\s*<pose>([^<]+)</pose>', sdf)}
	pts = []

	for link in re.finditer(r'<link name="(\w+)">(.*?)</link>', sdf, re.S):
		offset = link_pose.get(link.group(1), np.zeros(3))

		for uri in re.findall(r"<uri>model://talon1718/meshes/([^<]+)</uri>", link.group(2)):
			v = np.unique(np.round(read_stl(os.path.join(MODEL_DIR, "meshes", uri)), 3), axis=0)
			pts.append(v + offset)

	pts = np.concatenate(pts)
	x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
	tail_region = x < x.min() + 0.3
	kp = {
		"nose": pts[np.argmax(x)],
		"wing_l": pts[np.argmax(y)],
		"wing_r": pts[np.argmin(y)],
		# top outer corner of each V-tail surface
		"vtail_l": pts[np.argmax(np.where(tail_region & (y > 0), z, -np.inf))],
		"vtail_r": pts[np.argmax(np.where(tail_region & (y < 0), z, -np.inf))],
		"tail": pts[np.argmin(x)],  # rear end of the fuselage (motor shaft / prop hub)
	}
	return pts, np.array([kp[k] for k in KEYPOINTS])


def view_group(az_deg, el_deg):
	if el_deg > 45:
		return "top"

	if el_deg < -45:
		return "bottom"

	az = az_deg % 360
	return "front" if az < 45 or az >= 315 else "left" if az < 135 else "rear" if az < 225 else "right"


# ---------------------------------------------------------------- simulation ---

def gz_env(partition):
	env = dict(os.environ)
	out = subprocess.run(["bash", "-c", f". {BUILD_DIR}/rootfs/gz_env.sh && env"], capture_output=True, text=True).stdout
	env.update(line.split("=", 1) for line in out.splitlines() if "=" in line)
	env["GZ_SIM_RESOURCE_PATH"] = f"{SIM_DIR}/models:{SIM_DIR}/worlds:" + env.get("GZ_SIM_RESOURCE_PATH", "")
	env["GZ_PARTITION"] = partition
	env["GZ_IP"] = "127.0.0.1"
	return env


class Sim:
	def __init__(self, world, width, height, hfov, depth, log_path):
		self.world = world
		env = gz_env(f"dataset_{os.getpid()}")
		os.environ.update({k: env[k] for k in ("GZ_PARTITION", "GZ_IP")})
		self.log = open(log_path, "w")
		# paused (no -r): the script steps the world itself so that pose and image always match
		self.server = subprocess.Popen(["gz", "sim", "-s", "--headless-rendering", "--verbose=2",
						f"{SIM_DIR}/worlds/{world}.sdf"], env=env, stdout=self.log, stderr=subprocess.STDOUT)
		self.node = Node()
		self.frames = {"camera": None, "depth": None}
		self.use_depth = depth

		camera = f"""<horizontal_fov>{math.radians(hfov)}</horizontal_fov>
			<image><width>{width}</width><height>{height}</height><format>{{fmt}}</format></image>
			<clip><near>0.5</near><far>8000</far></clip>"""
		sensors = f"""<sensor name="camera" type="camera"><always_on>1</always_on><update_rate>{CAMERA_RATE}</update_rate>
			<camera>{camera.format(fmt="R8G8B8")}</camera></sensor>"""

		if depth:
			sensors += f"""<sensor name="depth" type="depth_camera"><always_on>1</always_on><update_rate>{CAMERA_RATE}</update_rate>
			<camera>{camera.format(fmt="R_FLOAT32")}</camera></sensor>"""

		self.wait_service(f"/world/{world}/create")
		self.spawn(TARGET, '<include><uri>model://talon1718</uri><static>true</static></include>')
		self.spawn(CAMERA, f'<model name="{CAMERA}"><static>true</static><link name="link">{sensors}</link></model>')

		base = f"/world/{world}/model/{CAMERA}/link/link/sensor"
		self.node.subscribe(ImageMsg, f"{base}/camera/image", lambda m: self.frames.__setitem__("camera", m))

		if depth:
			self.node.subscribe(ImageMsg, f"{base}/depth/depth_image", lambda m: self.frames.__setitem__("depth", m))

		self.sim_time = 0.0

		# render first frames (loads the scene and the terrain texture)
		for _ in range(200):
			if self.capture(timeout=0.5) is not None:
				break
		else:
			sys.exit("no camera image, see " + log_path)

	def wait_service(self, name, timeout=60):
		t0 = time.time()

		while time.time() - t0 < timeout:
			if name in self.node.service_list():
				return

			time.sleep(0.5)

		sys.exit(f"gz service {name} not available")

	def spawn(self, name, sdf):
		req = EntityFactory()
		req.name = name
		req.sdf = f'<sdf version="1.9">{sdf}</sdf>'

		for _ in range(20):
			ok, rep = self.node.request(f"/world/{self.world}/create", req, EntityFactory, Boolean, 5000)

			if ok and rep.data:
				return

			time.sleep(0.5)

		sys.exit(f"could not spawn {name}")

	def set_pose(self, name, pos, R):
		req = Pose()
		req.name = name
		req.position.x, req.position.y, req.position.z = pos
		req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = rot_to_quat(R)
		ok, rep = self.node.request(f"/world/{self.world}/set_pose", req, Pose, Boolean, 2000)
		return ok and rep.data

	def step(self, n):
		req = WorldControl()
		req.pause = True
		req.multi_step = n
		self.node.request(f"/world/{self.world}/control", req, WorldControl, Boolean, 5000)
		self.sim_time += n * STEP

	def capture(self, timeout=10.0):
		"""Step until frames rendered after the last pose change arrive: (rgb, depth or None)."""
		t_request = self.sim_time
		self.step(int(1 / CAMERA_RATE / STEP) + 1)
		t0 = time.time()
		fresh = lambda m: m is not None and m.header.stamp.sec + m.header.stamp.nsec * 1e-9 > t_request + 1e-6

		while time.time() - t0 < timeout:
			rgb, depth = self.frames["camera"], self.frames["depth"]

			if fresh(rgb) and (not self.use_depth or fresh(depth)):
				rgb_np = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, 3)
				depth_np = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width) \
					if self.use_depth else None
				return rgb_np, depth_np

			time.sleep(0.005)

		return None

	def close(self):
		self.server.terminate()

		try:
			self.server.wait(5)
		except subprocess.TimeoutExpired:
			self.server.kill()


# -------------------------------------------------------------------- sample ---

def sample_direction(rng, a, front):
	"""Unit view direction (target -> camera) in the target body frame: never within the nose cone,
	in the front or rear hemisphere, uniform on the sphere otherwise."""
	cos_cone = math.cos(math.radians(a.nose_cone))

	while True:
		z = rng.uniform(-1, 1)
		phi = rng.uniform(-math.pi, math.pi)
		d = np.array([math.sqrt(1 - z * z) * math.cos(phi), math.sqrt(1 - z * z) * math.sin(phi), z])

		if (d[0] > 0) == front and d[0] < cos_cone:
			return d


def sample_scene(rng, a):
	"""Random target pose and camera pose; returns a dict with everything needed."""
	target_pos = np.array([rng.uniform(-2000, 2000), rng.uniform(-2000, 2000), rng.uniform(a.alt_min, a.alt_max)])
	R_t = rot_rpy(math.radians(max(-60, min(60, rng.gauss(0, 25)))),
		      math.radians(rng.uniform(-25, 25)), rng.uniform(-math.pi, math.pi))

	# the hemisphere is drawn once per sample so that rejections below do not change --front-ratio
	front = rng.random() < a.front_ratio
	sin_el_min, sin_el_max = math.sin(math.radians(a.el_min)), math.sin(math.radians(a.el_max))

	while True:
		d_body = sample_direction(rng, a, front)
		d_world = R_t @ d_body
		az = math.degrees(math.atan2(d_body[1], d_body[0]))
		el = math.degrees(math.asin(d_body[2]))

		if not sin_el_min <= d_world[2] <= sin_el_max or view_group(az, el) in a.exclude_views:
			continue

		dist = math.exp(rng.uniform(math.log(a.dmin), math.log(a.dmax)))
		cam_pos = target_pos + d_world * dist

		if cam_pos[2] > 2.0:  # camera above ground
			break

	# aim so that the target lands at a random place in the frame, random camera roll (drone tilt)
	hfov = math.radians(a.hfov)
	vfov = 2 * math.atan(math.tan(hfov / 2) * a.height / a.width)
	negative = rng.random() < a.negatives
	sx, sy = rng.uniform(-0.8, 0.8), rng.uniform(-0.8, 0.8)

	if negative:  # look well away from the target
		sx, sy = rng.choice([-1, 1]) * rng.uniform(1.6, 3.0), rng.uniform(-1, 1)

	R_look = look_rotation(target_pos - cam_pos, math.radians(rng.uniform(-30, 30)))
	R_c = R_look @ rot_rpy(0, sy * vfov / 2, sx * hfov / 2)

	return dict(target_pos=target_pos, R_t=R_t, cam_pos=cam_pos, R_c=R_c, az=az, el=el, dist=dist,
		    front=bool(d_body[0] > 0), negative=negative)


def relative_pose_cv(s):
	"""Target pose in the OpenCV camera frame: (R, t)."""
	R = GZ_TO_CV @ s["R_c"].T @ s["R_t"]
	t = GZ_TO_CV @ s["R_c"].T @ (s["target_pos"] - s["cam_pos"])
	return R, t


def project(points_model, R, t, K):
	"""Pinhole projection of model-frame points, OpenCV convention. Returns (uv, depth)."""
	p = points_model @ R.T + t
	uv = p[:, :2] / p[:, 2:3] * [K["fx"], K["fy"]] + [K["cx"], K["cy"]]
	return uv, p[:, 2]


def bounding_box(uv, depth, K):
	"""Pixel box of the projected target (clipped to the image) or None."""
	uv = uv[depth > 0.5]

	if len(uv) == 0:
		return None

	x0, x1 = max(0.0, uv[:, 0].min()), min(float(K["width"]), uv[:, 0].max())
	y0, y1 = max(0.0, uv[:, 1].min()), min(float(K["height"]), uv[:, 1].max())

	if x1 - x0 < 1 or y1 - y0 < 1:
		return None

	return x0, y0, x1, y1


def keypoint_visibility(uv, depth, depth_img, K):
	"""2 visible, 1 inside the image but hidden (self-occlusion), 0 outside the image / behind the camera."""
	vis = []

	for (u, v), z in zip(uv, depth):
		if z <= 0.5 or not (0 <= u < K["width"] and 0 <= v < K["height"]):
			vis.append(0)
			continue

		if depth_img is None:
			vis.append(2)
			continue

		# visible if anything around its pixel shows the keypoint depth or something further away
		# (background, or the same surface): keypoints sit on rounded extremities (nose dome,
		# wing tips) and are usually on the silhouette, where the pixel itself can hit the
		# nearer side of the rounded surface. A keypoint hidden behind the aircraft has only
		# nearer surfaces around it.
		r = 3
		ui, vi = int(u), int(v)
		win = depth_img[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1]
		tol = max(0.03, 2.0 * z / K["fx"])
		vis.append(2 if np.any(~np.isfinite(win) | (win >= z - tol)) else 1)

	return vis


def write_readme(out, K, pose_task):
	"""Short description of the folder for whoever gets the dataset."""
	label = ("`0 cx cy w h` + 6 keypoint `x y v` (normalize). v: 2 görünür, 1 uçağın kendisi kapatıyor, "
		 "0 kadraj dışı" if pose_task else "`0 cx cy w h` (normalize)")
	text = f"""# Talon 1718 veri seti (sentetik, Gazebo)

Tek sınıf: `uav` (X-UAV Talon 1718). YOLO {"pose" if pose_task else "detect"} formatı.

- `images/train`, `images/val`: JPG, {K["width"]}x{K["height"]}, yatay FOV {K["hfov_deg"]:g}°
- `labels/train`, `labels/val`: {label}. Boş dosya = görüntüde hedef yok.
- `data.yaml`: Ultralytics için hazır, klasörün kendisini kullanır.
- Dosya adı öneki bakış yönü (uçağın eksenine göre): `front`, `left`, `rear`, `right`, `top`, `bottom`; `none` = hedef yok.
- `meta.csv`: her görüntünün açısı, mesafesi, kutu boyutu.
"""

	if pose_task:
		text += f"""- Keypoint sırası: {", ".join(KEYPOINTS)} (flip_idx: {FLIP_IDX})
- `poses.jsonl`: her görüntünün gerçek 6DoF pozu (OpenCV kamera ekseni: x sağ, y aşağı, z ileri).
- `camera.json`: kamera iç parametreleri (bozulma yok). `keypoints_3d.json`: keypoint'lerin uçak ekseninde 3B konumu [m].
"""

	with open(os.path.join(out, "README.md"), "w") as fh:
		fh.write(text)


def main():
	ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	ap.add_argument("--count", type=int, default=1000)
	ap.add_argument("--out", default=os.path.join(SIM_DIR, "build", "dataset"))
	ap.add_argument("--task", choices=["pose", "detect"], default="pose")
	ap.add_argument("--world", default="ankara")
	ap.add_argument("--width", type=int, default=1280)
	ap.add_argument("--height", type=int, default=720)
	ap.add_argument("--hfov", type=float, default=60.0, help="horizontal field of view [deg]")
	ap.add_argument("--dmin", type=float, default=5.0, help="min camera distance [m]")
	ap.add_argument("--dmax", type=float, default=150.0, help="max camera distance [m]")
	ap.add_argument("--front-ratio", type=float, default=1 / 3, help="share of views from the front hemisphere")
	ap.add_argument("--nose-cone", type=float, default=25.0, help="no views within this angle of the nose axis [deg]")
	ap.add_argument("--el-max", type=float, default=30.0,
			help="max camera elevation above the target, world frame [deg]")
	ap.add_argument("--el-min", type=float, default=-75.0,
			help="min camera elevation relative to the target (below it), world frame [deg]")
	ap.add_argument("--exclude-views", default="top",
			help="comma separated view groups never generated (front,left,rear,right,top,bottom), '' for none")
	ap.add_argument("--alt-min", type=float, default=15.0, help="min target height above ground [m]")
	ap.add_argument("--alt-max", type=float, default=150.0)
	ap.add_argument("--negatives", type=float, default=0.05, help="fraction of frames without the target")
	ap.add_argument("--val", type=float, default=0.1, help="validation fraction")
	ap.add_argument("--min-box", type=float, default=2.0, help="drop boxes smaller than this [px]")
	ap.add_argument("--seed", type=int, default=1)
	a = ap.parse_args()
	a.exclude_views = {v for v in a.exclude_views.split(",") if v}

	rng = random.Random(a.seed)
	points, kp3d = talon_geometry()
	pose_task = a.task == "pose"
	f = a.width / 2 / math.tan(math.radians(a.hfov) / 2)
	K = {"width": a.width, "height": a.height, "fx": f, "fy": f, "cx": a.width / 2, "cy": a.height / 2,
	     "dist": [0.0] * 5, "hfov_deg": a.hfov}

	for split in ("train", "val"):
		os.makedirs(os.path.join(a.out, "images", split), exist_ok=True)
		os.makedirs(os.path.join(a.out, "labels", split), exist_ok=True)

	with open(os.path.join(a.out, "camera.json"), "w") as fh:
		json.dump(K, fh, indent=1)

	with open(os.path.join(a.out, "keypoints_3d.json"), "w") as fh:
		json.dump({"frame": "Talon body frame: origin CG, x nose, y left wing, z up [m]",
			   "names": KEYPOINTS, "flip_idx": FLIP_IDX,
			   "points": {k: [round(float(c), 4) for c in p] for k, p in zip(KEYPOINTS, kp3d)}}, fh, indent=1)

	meta_path = os.path.join(a.out, "meta.csv")
	new_meta = not os.path.exists(meta_path)
	meta = open(meta_path, "a", newline="")
	writer = csv.writer(meta)
	poses = open(os.path.join(a.out, "poses.jsonl"), "a")

	if new_meta:
		writer.writerow(["image", "split", "view", "hemisphere", "azimuth_deg", "elevation_deg", "distance_m",
				 "target_roll_deg", "target_pitch_deg", "box_w_px", "box_h_px", "labelled", "kp_visible"])

	# continue numbering if the folder already has images
	existing = [int(m.group(1)) for d in ("train", "val")
		    for fn in os.listdir(os.path.join(a.out, "images", d))
		    if (m := re.search(r"_(\d+)\.jpg$", fn))]
	index = max(existing, default=-1) + 1

	sim = Sim(a.world, a.width, a.height, a.hfov, pose_task, os.path.join(a.out, "gz_server.log"))
	counts = {}
	max_check_err = 0.0
	t_start = time.time()

	try:
		done = 0

		while done < a.count:
			s = sample_scene(rng, a)

			if not (sim.set_pose(TARGET, s["target_pos"], s["R_t"]) and sim.set_pose(CAMERA, s["cam_pos"], s["R_c"])):
				continue

			frames = sim.capture()

			if frames is None:
				print("frame timeout, skipping", file=sys.stderr)
				continue

			rgb, depth_img = frames
			R, t = relative_pose_cv(s)
			uv, z = project(points, R, t, K)
			box = None if s["negative"] else bounding_box(uv, z, K)

			if box is not None and min(box[2] - box[0], box[3] - box[1]) < a.min_box:
				continue

			kp_uv, kp_z = project(kp3d, R, t, K)
			vis = keypoint_visibility(kp_uv, kp_z, depth_img, K) if box is not None else [0] * len(KEYPOINTS)

			# self check: the saved pose + intrinsics must reproduce the keypoints independently of
			# the gz frame conversion used for the labels
			p_world = kp3d @ s["R_t"].T + s["target_pos"]
			p_gz = (p_world - s["cam_pos"]) @ s["R_c"]
			check = np.column_stack([K["cx"] - f * p_gz[:, 1] / p_gz[:, 0], K["cy"] - f * p_gz[:, 2] / p_gz[:, 0]])
			max_check_err = max(max_check_err, float(np.abs(check - kp_uv)[p_gz[:, 0] > 0.5].max(initial=0)))

			view = view_group(s["az"], s["el"]) if box is not None else "none"
			name = f"{view}_{index:06d}"
			split = "val" if rng.random() < a.val else "train"
			Image.fromarray(rgb).save(os.path.join(a.out, "images", split, name + ".jpg"), quality=92)

			with open(os.path.join(a.out, "labels", split, name + ".txt"), "w") as fh:
				if box is not None:
					x0, y0, x1, y1 = box
					line = (f"0 {(x0 + x1) / 2 / a.width:.6f} {(y0 + y1) / 2 / a.height:.6f} "
						f"{(x1 - x0) / a.width:.6f} {(y1 - y0) / a.height:.6f}")

					if pose_task:
						for (u, v), vv in zip(kp_uv, vis):
							line += f" {u / a.width:.6f} {v / a.height:.6f} {vv}" if vv else " 0 0 0"

					fh.write(line + "\n")

			R_t = s["R_t"]
			roll, pitch = math.degrees(math.atan2(R_t[2, 1], R_t[2, 2])), math.degrees(-math.asin(R_t[2, 0]))
			yaw = math.degrees(math.atan2(R_t[1, 0], R_t[0, 0]))
			poses.write(json.dumps({
				"image": name + ".jpg", "split": split, "view": view,
				"t_cam": [round(float(v), 4) for v in t],
				"q_cam": [round(float(v), 6) for v in rot_to_quat(R)],
				"rpy_target_deg": [round(roll, 2), round(pitch, 2), round(yaw, 2)],
				"distance_m": round(s["dist"], 3), "azimuth_deg": round(s["az"], 2), "elevation_deg": round(s["el"], 2),
				"keypoints_px": [[round(float(u), 2), round(float(v), 2), int(vv)] for (u, v), vv in zip(kp_uv, vis)],
				"target_world": {"p": [round(float(v), 4) for v in s["target_pos"]],
						 "q": [round(float(v), 6) for v in rot_to_quat(s["R_t"])]},
				"camera_world_gz": {"p": [round(float(v), 4) for v in s["cam_pos"]],
						    "q": [round(float(v), 6) for v in rot_to_quat(s["R_c"])]},
			}) + "\n")
			writer.writerow([name + ".jpg", split, view, "front" if s["front"] else "rear",
					 f"{s['az']:.1f}", f"{s['el']:.1f}", f"{s['dist']:.1f}", f"{roll:.1f}", f"{pitch:.1f}",
					 f"{box[2] - box[0]:.1f}" if box else "", f"{box[3] - box[1]:.1f}" if box else "",
					 int(box is not None), sum(v == 2 for v in vis)])
			counts[view] = counts.get(view, 0) + 1
			index += 1
			done += 1

			if done % 50 == 0 or done == a.count:
				rate = done / (time.time() - t_start)
				print(f"{done}/{a.count}  {rate:.1f} img/s  " + " ".join(f"{k}:{v}" for k, v in sorted(counts.items())))
	finally:
		meta.close()
		poses.close()
		sim.close()

	if max_check_err > 1.0:
		print(f"WARNING: pose / keypoint self check error {max_check_err:.2f} px", file=sys.stderr)

	with open(os.path.join(a.out, "data.yaml"), "w") as fh:
		# no "path": Ultralytics then uses the folder of data.yaml, so the dataset can be moved
		fh.write("train: images/train\nval: images/val\n")

		if pose_task:
			fh.write(f"kpt_shape: [{len(KEYPOINTS)}, 3]\nflip_idx: {FLIP_IDX}\n")

		fh.write("names:\n  0: uav\n")

	write_readme(a.out, K, pose_task)


if __name__ == "__main__":
	main()

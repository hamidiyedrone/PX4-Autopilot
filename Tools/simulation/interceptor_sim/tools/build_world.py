#!/usr/bin/env python3
"""
Generate the Gazebo world used by the interceptor simulation.

Terrain: elevation (DEM) + ortho photo of "McMillan Airfield" from Gazebo Fuel
(OpenRobotics, CC-BY 4.0). Its dry, rolling terrain is a good stand-in for the
Central Anatolian steppe; only the geographic origin is moved to Ankara.

The Fuel model is a DEM <heightmap>, which Gazebo Harmonic neither renders
(headless) nor collides with (dartsim), and it squeezes the 5.8 x 7.1 km area
into a 5.7 km square. So the photo is put on a textured triangle mesh in true
metres instead (models/terrain_mcmillan, git-ignored because of the
downloaded photo).

By default the ground is flat (FLAT_TERRAIN): only the photo is used and the
collision is an infinite plane at z = 0. With FLAT_TERRAIN = False the DEM
relief is used for the mesh and its collision, with the runway strip
flattened so that runway take-offs do not bounce on the 25-30 m DEM grid.

The world magnetic field is computed from PX4's own WMM tables for the chosen
location, otherwise the simulated magnetometer disagrees with what PX4
expects there (arming checks / heading offset).

    python3 tools/build_world.py
      -> worlds/ankara.sdf, worlds/ankara.env (spawn pose), models/terrain_mcmillan/
"""

import io
import math
import os
import re
import shutil
import urllib.request
import zipfile

import numpy as np
from PIL import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))

WORLD_NAME = "ankara"
LAT_DEG = 39.9334       # Ankara, TODO: set to the competition field
LON_DEG = 32.8597
# AMSL altitude of the world origin (runway centre, z = 0). Keep it 0: PX4 FW TECS starts on
# ground before the GPS origin exists (altitude 0) and, in auto modes, only slews its altitude
# reference towards the real AMSL altitude at the climb rate limit (Ankara is ~900 m).
ELEVATION_M = 0.0

TERRAIN_ZIP = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/mcmillan_airfield.zip"
FUEL_DIR = os.path.join(SIM_DIR, "build", "fuel", "mcmillan_airfield")
TERRAIN_MODEL = "terrain_mcmillan"

# runway thresholds in ortho photo pixels (measured on mcmillan_color.png)
RUNWAY_PX = [(1950, 2778), (2875, 3123)]   # NW, SE
RUNWAY_WIDTH = 40.0     # [m] flattened strip
RUNWAY_BLEND = 40.0     # [m] transition to the natural terrain
SPAWN_FROM_THRESHOLD = 60.0  # [m] target spawn point, from the NW threshold towards SE
FLAT_TERRAIN = True     # ignore the DEM relief, flat ground with the photo only


def fetch_terrain():
	if not os.path.exists(os.path.join(FUEL_DIR, "media", "textures", "mcmillan_color.png")):
		print("downloading", TERRAIN_ZIP, "(~60 MB)")
		data = urllib.request.urlopen(TERRAIN_ZIP, timeout=600).read()
		zipfile.ZipFile(io.BytesIO(data)).extractall(FUEL_DIR)

	return (os.path.join(FUEL_DIR, "media", "mcmillan_elevation.tif"),
		os.path.join(FUEL_DIR, "media", "textures", "mcmillan_color.png"))


def wmm(lat, lon):
	"""Declination [deg], inclination [deg], intensity [T], same lookup as PX4 geo_mag_declination.cpp."""
	src = open(os.path.join(PX4_DIR, "src/lib/world_magnetic_model/geo_magnetic_tables.hpp")).read()

	def table(name):
		body = re.search(name + r"\[19\]\[37\]\s*\{(.*?)\n\};", src, re.S).group(1)
		rows = [re.sub(r"/\*.*?\*/|//.*", "", r) for r in re.findall(r"\{([^{}]*)\}", body)]
		return [[int(v) for v in r.split(",") if v.strip()] for r in rows]

	def scale(name):
		return float(re.search(name + r"\s*=\s*([\d.eE+-]+)f", src).group(1))

	def lookup(tab):
		lat_i = min(int((lat + 90) // 10), 17)
		lon_i = min(int((lon + 180) // 10), 35)
		fy = (lat + 90 - lat_i * 10) / 10
		fx = (lon + 180 - lon_i * 10) / 10
		sw, se = tab[lat_i][lon_i], tab[lat_i][lon_i + 1]
		nw, ne = tab[lat_i + 1][lon_i], tab[lat_i + 1][lon_i + 1]
		return (sw * (1 - fx) + se * fx) * (1 - fy) + (nw * (1 - fx) + ne * fx) * fy

	dec = lookup(table("declination_table")) * scale("WMM_DECLINATION_SCALE_TO_DEGREES")
	inc = lookup(table("inclination_table")) * scale("WMM_INCLINATION_SCALE_TO_DEGREES")
	tot = lookup(table("totalintensity_table")) * scale("WMM_TOTALINTENSITY_SCALE_TO_NANOTESLA") * 1e-9
	return dec, inc, tot


def bilinear(grid, r, c):
	r = np.clip(r, 0, grid.shape[0] - 1.001)
	c = np.clip(c, 0, grid.shape[1] - 1.001)
	r0, c0 = int(r), int(c)
	fr, fc = r - r0, c - c0
	return ((grid[r0, c0] * (1 - fc) + grid[r0, c0 + 1] * fc) * (1 - fr)
		+ (grid[r0 + 1, c0] * (1 - fc) + grid[r0 + 1, c0 + 1] * fc) * fr)


def build_terrain(dem_path, tex_path):
	"""Textured OBJ terrain in world metres, origin at the runway centre. Returns spawn pose."""
	im = Image.open(dem_path)
	dem = np.asarray(im, dtype=np.float64)
	rows, cols = dem.shape
	tex_w, tex_h = Image.open(tex_path).size

	# GeoTIFF: ModelTiepoint (top-left lon/lat) + ModelPixelScale (deg/pixel)
	lon_tl, lat_tl = im.tag_v2[33922][3], im.tag_v2[33922][4]
	s_lon, s_lat = im.tag_v2[33550][0], im.tag_v2[33550][1]
	lat_mid = lat_tl - s_lat * rows / 2
	m_lon = s_lon * 111320.0 * math.cos(math.radians(lat_mid))  # metres per pixel east
	m_lat = s_lat * 110574.0                                    # metres per pixel north

	# pixel (continuous, edge based) -> metres east/north of the top-left corner
	to_en = lambda r, c: (c * m_lon, -r * m_lat)

	# runway in DEM pixel coordinates (the photo covers the same bounds as the DEM)
	rw = [(py / tex_h * rows, px / tex_w * cols) for px, py in RUNWAY_PX]
	rw_en = np.array([to_en(r, c) for r, c in rw])
	origin_en = rw_en.mean(axis=0)
	rw_h = [bilinear(dem, r - 0.5, c - 0.5) for r, c in rw]  # vertices sit at pixel centres
	origin_h = 0.5 * (rw_h[0] + rw_h[1])

	# vertices at pixel centres
	rr, cc = np.meshgrid(np.arange(rows) + 0.5, np.arange(cols) + 0.5, indexing="ij")
	x = cc * m_lon - origin_en[0]
	y = -rr * m_lat - origin_en[1]
	z = dem - origin_h

	# flatten the runway strip to a straight slope between its thresholds
	a, b = rw_en[0] - origin_en, rw_en[1] - origin_en
	ab = b - a
	t = np.clip(((x - a[0]) * ab[0] + (y - a[1]) * ab[1]) / (ab @ ab), 0, 1)
	d = np.hypot(x - (a[0] + t * ab[0]), y - (a[1] + t * ab[1]))
	w = np.clip(1 - (d - RUNWAY_WIDTH / 2) / RUNWAY_BLEND, 0, 1)
	w = w * w * (3 - 2 * w)  # smoothstep
	z_rw = (rw_h[0] + t * (rw_h[1] - rw_h[0])) - origin_h
	z = w * z_rw + (1 - w) * z

	if FLAT_TERRAIN:
		z = np.zeros_like(z)
		rw_h = [origin_h, origin_h]

	# normals
	gy, gx = np.gradient(z, -m_lat, m_lon)
	n = np.dstack([-gx, -gy, np.ones_like(z)])
	n /= np.linalg.norm(n, axis=2, keepdims=True)

	u = cc / cols
	v = 1.0 - rr / rows  # OBJ texture origin is bottom-left

	model_dir = os.path.join(SIM_DIR, "models", TERRAIN_MODEL)
	mesh_dir = os.path.join(model_dir, "meshes")
	os.makedirs(mesh_dir, exist_ok=True)
	shutil.copyfile(tex_path, os.path.join(mesh_dir, "terrain.png"))

	with open(os.path.join(mesh_dir, "terrain.mtl"), "w") as f:
		f.write("newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nillum 1\nmap_Kd terrain.png\n")

	idx = np.arange(rows * cols).reshape(rows, cols) + 1  # OBJ is 1-based
	q = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], axis=-1).reshape(-1, 4)

	with open(os.path.join(mesh_dir, "terrain.obj"), "w") as f:
		f.write("# generated by build_world.py from McMillan Airfield (OpenRobotics, CC-BY 4.0)\n")
		f.write("mtllib terrain.mtl\nusemtl terrain\n")
		np.savetxt(f, np.column_stack([x.ravel(), y.ravel(), z.ravel()]), fmt="v %.2f %.2f %.2f")
		np.savetxt(f, np.column_stack([u.ravel(), v.ravel()]), fmt="vt %.6f %.6f")
		np.savetxt(f, n.reshape(-1, 3), fmt="vn %.4f %.4f %.4f")
		# counter-clockwise seen from above: row grows southwards
		tris = np.concatenate([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])
		np.savetxt(f, np.repeat(tris, 3, axis=1), fmt="f %d/%d/%d %d/%d/%d %d/%d/%d")

	with open(os.path.join(model_dir, "model.config"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<model>
  <name>{TERRAIN_MODEL}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>McMillan Airfield terrain mesh generated from the OpenRobotics Fuel model (CC-BY 4.0)</description>
</model>
""")

	if FLAT_TERRAIN:
		collision = "<plane><normal>0 0 1</normal><size>1 1</size></plane>"
	else:
		collision = f"<mesh><uri>model://{TERRAIN_MODEL}/meshes/terrain.obj</uri></mesh>"

	with open(os.path.join(model_dir, "model.sdf"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_world.py -->
<sdf version="1.9">
  <model name="{TERRAIN_MODEL}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>{collision}</geometry>
        <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://{TERRAIN_MODEL}/meshes/terrain.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""")

	yaw = math.atan2(ab[1], ab[0])
	spawn = a + ab / np.linalg.norm(ab) * SPAWN_FROM_THRESHOLD
	extent = (x.max() - x.min(), y.max() - y.min())
	print(f"terrain {extent[0]:.0f} x {extent[1]:.0f} m, z {z.min():.0f}..{z.max():.0f} m, "
	      f"runway {np.linalg.norm(ab):.0f} m, heading {90 - math.degrees(yaw):.1f} deg true")
	return spawn[0], spawn[1], z_rw_at(rw_h, origin_h, SPAWN_FROM_THRESHOLD / np.linalg.norm(ab)), yaw


def z_rw_at(rw_h, origin_h, t):
	return rw_h[0] + t * (rw_h[1] - rw_h[0]) - origin_h


def main():
	dem_path, tex_path = fetch_terrain()
	sx, sy, sz, syaw = build_terrain(dem_path, tex_path)

	dec, inc, tot = wmm(LAT_DEG, LON_DEG)
	h = tot * math.cos(math.radians(inc))
	# world frame is ENU
	b_east = h * math.sin(math.radians(dec))
	b_north = h * math.cos(math.radians(dec))
	b_up = -tot * math.sin(math.radians(inc))

	print(f"WMM at {LAT_DEG}, {LON_DEG}: declination {dec:.2f} deg, inclination {inc:.2f} deg, {tot * 1e9:.0f} nT")

	world = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_world.py, do not edit by hand -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <!-- WMM at the world origin, ENU [T] -->
    <magnetic_field>{b_east:.4e} {b_north:.4e} {b_up:.4e}</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.62 0.75 0.9 1</background>
      <shadows>true</shadows>
      <sky>
        <clouds>
          <speed>6</speed>
        </clouds>
      </sky>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 1000 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.4 0.3 -0.85</direction>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
    </light>
    <!-- McMillan Airfield terrain (OpenRobotics, CC-BY 4.0), origin at the runway centre -->
    <include>
      <uri>model://{TERRAIN_MODEL}</uri>
      <name>terrain</name>
    </include>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{LAT_DEG}</latitude_deg>
      <longitude_deg>{LON_DEG}</longitude_deg>
      <elevation>{ELEVATION_M}</elevation>
    </spherical_coordinates>
  </world>
</sdf>
"""
	worlds = os.path.join(SIM_DIR, "worlds")
	os.makedirs(worlds, exist_ok=True)

	with open(os.path.join(worlds, WORLD_NAME + ".sdf"), "w") as f:
		f.write(world)

	# default target spawn: on the runway, facing down the runway. The Talon origin is its
	# CG, 0.12 m above the belly -> +0.15 m
	with open(os.path.join(worlds, WORLD_NAME + ".env"), "w") as f:
		f.write(f"TARGET_POSE_DEFAULT={sx:.2f},{sy:.2f},{sz + 0.15:.2f},0,0,{syaw:.4f}\n")

	print(f"wrote worlds/{WORLD_NAME}.sdf, target spawn {sx:.1f} {sy:.1f} {sz + 0.15:.2f} yaw {syaw:.3f}")


if __name__ == "__main__":
	main()

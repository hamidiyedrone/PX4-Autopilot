#!/usr/bin/env python3
"""
Generate the Gazebo world used by the interceptor simulation.

Ground: flat, textured with the ortho photo of "McMillan Airfield" from
Gazebo Fuel (OpenRobotics, CC-BY 4.0). Its dry steppe look is a good stand-in
for Central Anatolia; only the geographic origin is moved to Ankara.

The Fuel model itself is a DEM <heightmap>, which Gazebo Harmonic neither
renders (headless) nor collides with (dartsim), and it squeezes the
5.8 x 7.1 km area into a 5.7 km square. So the photo is put on a flat quad in
true metres instead (models/terrain_mcmillan, git-ignored because of the
downloaded photo); the collision is an infinite plane at z = 0. The DEM is
only read for its geographic bounds.

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

Image.MAX_IMAGE_PIXELS = None  # the ortho photo is 5000 x 4995

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))

WORLD_NAME = "ankara"
# world origin = target spawn point on the runway start (Etimesgut, Ankara)
LAT_DEG = 39.930898
LON_DEG = 32.729591
RUNWAY_HEADING_DEG = 67.5   # [deg true] take-off direction; the ortho photo is rotated to match
# AMSL altitude of the world origin (target spawn point, z = 0). Keep it 0: PX4 FW TECS starts on
# ground before the GPS origin exists (altitude 0) and, in auto modes, only slews its altitude
# reference towards the real AMSL altitude at the climb rate limit (Ankara is ~900 m).
ELEVATION_M = 0.0

TERRAIN_ZIP = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/mcmillan_airfield.zip"
FUEL_DIR = os.path.join(SIM_DIR, "build", "fuel", "mcmillan_airfield")
TERRAIN_MODEL = "terrain_mcmillan"

# runway thresholds in ortho photo pixels (measured on mcmillan_color.png)
RUNWAY_PX = [(1950, 2778), (2875, 3123)]   # NW, SE
SPAWN_FROM_THRESHOLD = 60.0  # [m] target spawn point, from the NW threshold towards SE
INTERCEPTOR_OFFSET = 40.0    # [m] interceptor spawn, to the left of the target spawn (off the runway)


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


def build_terrain(dem_path, tex_path):
	"""Flat textured ground in world metres. The photo is rotated so that its runway points to
	RUNWAY_HEADING_DEG and shifted so that the target spawn point on it is the world origin.
	Returns the spawn pose."""
	im = Image.open(dem_path)
	cols, rows = im.size
	tex_w, tex_h = Image.open(tex_path).size

	# GeoTIFF: ModelTiepoint (top-left lon/lat) + ModelPixelScale (deg/pixel);
	# the photo covers the same bounds as the DEM
	lat_tl = im.tag_v2[33922][4]
	s_lon, s_lat = im.tag_v2[33550][0], im.tag_v2[33550][1]
	lat_mid = lat_tl - s_lat * rows / 2
	width = cols * s_lon * 111320.0 * math.cos(math.radians(lat_mid))  # [m] east-west
	height = rows * s_lat * 110574.0                                  # [m] north-south

	# photo pixel -> metres east/north of the top-left corner
	to_en = lambda px, py: np.array([px / tex_w * width, -py / tex_h * height])
	rw = [to_en(px, py) for px, py in RUNWAY_PX]
	ab = rw[1] - rw[0]
	spawn = rw[0] + ab / np.linalg.norm(ab) * SPAWN_FROM_THRESHOLD
	yaw = math.radians(90 - RUNWAY_HEADING_DEG)  # ENU
	rot = yaw - math.atan2(ab[1], ab[0])
	R = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
	# photo corners (bottom-left, bottom-right, top-right, top-left) in world metres
	corners = [R @ (np.array(c) - spawn) for c in ((0, -height), (width, -height), (width, 0), (0, 0))]
	verts = "\n".join(f"v {c[0]:.2f} {c[1]:.2f} 0" for c in corners)

	model_dir = os.path.join(SIM_DIR, "models", TERRAIN_MODEL)
	mesh_dir = os.path.join(model_dir, "meshes")
	os.makedirs(mesh_dir, exist_ok=True)
	shutil.copyfile(tex_path, os.path.join(mesh_dir, "terrain.png"))

	with open(os.path.join(mesh_dir, "terrain.mtl"), "w") as f:
		f.write("newmtl terrain\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nillum 1\nmap_Kd terrain.png\n")

	with open(os.path.join(mesh_dir, "terrain.obj"), "w") as f:
		f.write(f"""# generated by build_world.py from McMillan Airfield (OpenRobotics, CC-BY 4.0)
mtllib terrain.mtl
usemtl terrain
{verts}
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
""")

	with open(os.path.join(model_dir, "model.config"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<model>
  <name>{TERRAIN_MODEL}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>McMillan Airfield ortho photo on flat ground, from the OpenRobotics Fuel model (CC-BY 4.0)</description>
</model>
""")

	with open(os.path.join(model_dir, "model.sdf"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_world.py -->
<sdf version="1.9">
  <model name="{TERRAIN_MODEL}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
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

	print(f"ground {width:.0f} x {height:.0f} m, runway {np.linalg.norm(ab):.0f} m, photo rotated by "
	      f"{math.degrees(rot):.1f} deg, runway heading {RUNWAY_HEADING_DEG:.1f} deg true")
	return 0.0, 0.0, 0.0, yaw


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
	# interceptor: x, y, yaw only; sim.sh takes the height from the generated model
	ix = sx - INTERCEPTOR_OFFSET * math.sin(syaw)
	iy = sy + INTERCEPTOR_OFFSET * math.cos(syaw)

	with open(os.path.join(worlds, WORLD_NAME + ".env"), "w") as f:
		f.write(f"TARGET_POSE_DEFAULT={sx:.2f},{sy:.2f},{sz + 0.15:.2f},0,0,{syaw:.4f}\n")
		f.write(f"INTERCEPTOR_XY_DEFAULT={ix:.2f},{iy:.2f},{syaw:.4f}\n")

	print(f"wrote worlds/{WORLD_NAME}.sdf, target spawn {sx:.1f} {sy:.1f} {sz + 0.15:.2f} yaw {syaw:.3f}")


if __name__ == "__main__":
	main()

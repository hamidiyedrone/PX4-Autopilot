#!/usr/bin/env python3
"""
Build the Gazebo model `talon1718` from the STL parts extracted from
"x-uav talon.step" by step_to_parts.py.

Aerodynamics are taken from advanced_plane (AdvancedLiftDrag) and only
re-scaled to the Talon geometry (area, span, MAC). The V-tail is modelled as
two ruddervators driven by PX4 "Left/Right V-Tail" control surfaces.

    python3 tools/build_talon.py build/talon_parts models/talon1718
"""

import argparse
import json
import os
import struct

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------- inputs ---
# which STL part is what (see build/talon_parts/views.png)
PART_BODY = "part_02"
PART_AIL_LEFT = "part_00"    # +X in STEP == +y (left) in Gazebo
PART_AIL_RIGHT = "part_03"
PART_CANOPY = "part_01"      # front window ("Cam - Pencere")
CANOPY_TRANSPARENCY = 0.35

MASS = 3.0              # [kg] take-off mass, TODO: weigh the real aircraft
PROP_RADIUS = 0.127     # [m] 10" pusher prop
MAX_THRUST = 22.0       # [N] static thrust at SIM_GZ_EC_MAX (T/W ~0.75)
MOTOR_CONSTANT = 8.54858e-06   # same motor model as advanced_plane / x500
AIL_LIMIT = 0.45        # [rad] ~25 deg
RUDV_LIMIT = 0.45
SKID_CLEARANCE = 0.02   # [m] fuselage collision box above the skid contact points

# STEP frame: X = span (left), Y = up, Z = nose  ->  Gazebo: x fwd, y left, z up
R_STEP_TO_GZ = np.array([[0, 0, 1],
			 [1, 0, 0],
			 [0, 1, 0]], dtype=float)


def read_stl(path):
	with open(path, "rb") as f:
		f.seek(80)
		n = struct.unpack("<I", f.read(4))[0]
		rec = np.frombuffer(f.read(n * 50), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
	return rec["v"].astype(np.float64)


def write_stl(path, tris):
	v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
	n = np.cross(v1 - v0, v2 - v0)
	ln = np.linalg.norm(n, axis=1, keepdims=True)
	n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
	rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
	rec["n"] = n
	rec["v"] = tris

	with open(path, "wb") as f:
		f.write(b"talon1718".ljust(80, b"\0"))
		f.write(struct.pack("<I", len(tris)))
		f.write(rec.tobytes())


def top_view_raster(tris_list, res=0.002):
	"""Rasterise the x-y projection of the triangles, returns (mask, x0, y0, res)."""
	pts = np.concatenate([t.reshape(-1, 3) for t in tris_list])
	x0, y0 = pts[:, 0].min() - res, pts[:, 1].min() - res
	w = int((pts[:, 0].max() - x0) / res) + 2
	h = int((pts[:, 1].max() - y0) / res) + 2
	img = Image.new("1", (w, h), 0)
	draw = ImageDraw.Draw(img)

	for tris in tris_list:
		for t in tris:
			draw.polygon([((p[0] - x0) / res, (p[1] - y0) / res) for p in t], fill=1)

	return np.asarray(img.convert("L")) > 0, x0, y0, res  # mask[iy, ix]


def hinge_line(tris):
	"""Hinge = front (max x) edge of a control surface; returns point, unit axis, span."""
	p = tris.reshape(-1, 3)
	y_in, y_out = np.sort([p[:, 1].min(), p[:, 1].max()]) if p[:, 1].mean() > 0 else np.sort([p[:, 1].max(), p[:, 1].min()])[::-1]
	ends = []

	for y in (y_in, y_out):
		sl = p[np.abs(p[:, 1] - y) < 0.01]
		front = sl[sl[:, 0] > sl[:, 0].max() - 0.004]
		ends.append([sl[:, 0].max(), y, front[:, 2].mean()])

	a, b = np.array(ends)
	axis = (b - a) / np.linalg.norm(b - a)

	if axis[1] < 0:  # always point to +y (same convention as advanced_plane: positive = trailing edge up)
		axis = -axis

	return (a + b) / 2, axis, abs(y_out - y_in)


def fmt(v, n=4):
	return " ".join(f"{x:.{n}f}" for x in np.ravel(v))


def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("parts_dir")
	ap.add_argument("model_dir")
	a = ap.parse_args()

	parts = {p["name"]: p for p in json.load(open(os.path.join(a.parts_dir, "parts.json")))}
	load = lambda n: read_stl(os.path.join(a.parts_dir, n + ".stl")) @ R_STEP_TO_GZ.T

	body, ail_l, ail_r = load(PART_BODY), load(PART_AIL_LEFT), load(PART_AIL_RIGHT)

	# --- origin = centre of gravity: volume centroid of the body, forced onto the symmetry plane
	cg = R_STEP_TO_GZ @ np.array(parts[PART_BODY]["centroid"])
	cg[1] = 0.5 * (body[..., 1].min() + body[..., 1].max())
	body, ail_l, ail_r = body - cg, ail_l - cg, ail_r - cg

	# --- inertia: uniform density solid scaled to MASS (foam airframe), rotated to Gazebo frame
	vol = parts[PART_BODY]["volume_m3"]
	inertia = R_STEP_TO_GZ @ np.array(parts[PART_BODY]["inertia_unit_density"]) @ R_STEP_TO_GZ.T * MASS / vol
	inertia[np.abs(inertia) < 1e-9] = 0.0
	inertia[0, 1] = inertia[1, 0] = inertia[1, 2] = inertia[2, 1] = 0.0  # symmetric about x-z plane

	# --- wing geometry from the top view (wing + fuselage carry-through, tail excluded)
	allp = body.reshape(-1, 3)
	span = allp[:, 1].max() - allp[:, 1].min()
	mask, x0, y0, res = top_view_raster([body, ail_l, ail_r])
	xs = x0 + (np.arange(mask.shape[1]) + 0.5) * res
	wing_tip = allp[np.abs(allp[:, 1]) > 0.8 * span / 2]
	wing_x_min, wing_x_max = wing_tip[:, 0].min() - 0.05, wing_tip[:, 0].max() + 0.05
	wing_cols = (xs > wing_x_min) & (xs < wing_x_max)
	chord = mask[:, wing_cols].sum(axis=1) * res
	area = chord.sum() * res
	tail = mask[:, xs < wing_x_min - 0.05].sum() * res * res
	mac = (chord ** 2).sum() * res / area
	ar = span ** 2 / area
	le_root = allp[(np.abs(allp[:, 1]) > 0.1) & (np.abs(allp[:, 1]) < 0.15), 0].max()

	# --- control surfaces
	ail_l_p, ail_l_axis, ail_span = hinge_line(ail_l)
	ail_r_p, ail_r_axis, _ = hinge_line(ail_r)

	# V-tail: two surfaces at the rear, dihedral read from the body mesh
	tailp = allp[allp[:, 0] < allp[:, 0].min() + 0.25]
	tl = tailp[tailp[:, 1] > 0.03]
	k = np.polyfit(tl[:, 1], tl[:, 2], 1)[0]
	vtail_dihedral = np.arctan(k)
	tail_x = tailp[:, 0].min() + 0.03
	tail_root_z = tl[:, 2].min()
	rv_l_p = np.array([tail_x, 0.10, tail_root_z + 0.10 * k])
	rv_r_p = rv_l_p * [1, -1, 1]
	rv_l_axis = np.array([0.0, np.cos(vtail_dihedral), np.sin(vtail_dihedral)])
	rv_r_axis = rv_l_axis * [1, -1, 1]

	# --- motor: pusher, prop hub (~10 mm thick) sits on the tip of the motor
	# shaft that sticks out of the rear end of the fuselage
	fus = allp[np.abs(allp[:, 1]) < 0.05]
	shaft = fus[fus[:, 0] < fus[:, 0].min() + 0.01]
	mot_x = shaft[:, 0].min() + 0.006
	mot_z = 0.5 * (shaft[:, 2].min() + shaft[:, 2].max())
	max_rot = float(np.sqrt(MAX_THRUST / MOTOR_CONSTANT))

	# --- collisions: simple boxes
	# fuselage box ends SKID_CLEARANCE above the belly so that only the low friction
	# skids touch the ground (otherwise the box friction holds the aircraft on the runway)
	fus_bottom = fus[:, 2].min() + SKID_CLEARANCE
	fus_box = [fus[:, 0].max() - fus[:, 0].min(), 0.10, fus[:, 2].max() - fus_bottom]
	fus_ctr = [(fus[:, 0].max() + fus[:, 0].min()) / 2, 0, (fus[:, 2].max() + fus_bottom) / 2]
	wing = allp[np.abs(allp[:, 1]) > 0.1]
	wing = wing[(wing[:, 0] > wing_x_min) & (wing[:, 0] < wing_x_max)]
	wing_box = [wing[:, 0].max() - wing[:, 0].min(), span, 0.03]
	wing_ctr = [(wing[:, 0].max() + wing[:, 0].min()) / 2, 0, np.median(wing[:, 2])]
	belly_z = allp[:, 2].min()

	# --- point where AdvancedLiftDrag applies the forces. The advanced_plane
	# derivatives were tuned with cp 0.12 m behind the CG at mac 0.22 m; keep
	# that ratio, otherwise lift ahead of the CG makes pitch nearly neutral.
	ac_x = -0.12 / 0.22 * mac

	# ---------------------------------------------------------------- write ---
	mesh_dir = os.path.join(a.model_dir, "meshes")
	os.makedirs(mesh_dir, exist_ok=True)
	write_stl(os.path.join(mesh_dir, "aileron_left.stl"), ail_l - ail_l_p)
	write_stl(os.path.join(mesh_dir, "aileron_right.stl"), ail_r - ail_r_p)
	write_stl(os.path.join(mesh_dir, "canopy.stl"), load(PART_CANOPY) - cg)

	# body is split by the paint colours of the CAD faces (white, nav lights, motor shaft)
	body_visuals = []

	for i, cm in enumerate(parts[PART_BODY]["colour_meshes"]):
		fname = "body_%d.stl" % i
		write_stl(os.path.join(mesh_dir, fname), read_stl(os.path.join(a.parts_dir, cm["file"])) @ R_STEP_TO_GZ.T - cg)
		body_visuals.append((fname, cm["colour"], cm["rgb"]))

	for old in os.listdir(mesh_dir):  # meshes of an older generator version
		if old.startswith("body") and old not in [v[0] for v in body_visuals]:
			os.remove(os.path.join(mesh_dir, old))

	geo = dict(body_visuals=body_visuals, ail_rgb=parts[PART_AIL_LEFT]["rgb"],
		   canopy_rgb=parts[PART_CANOPY]["rgb"], mass=MASS, I=inertia, belly_z=belly_z, area=area, mac=mac, ar=ar, ac_x=ac_x,
		   ail_l_p=ail_l_p, ail_l_axis=ail_l_axis, ail_r_p=ail_r_p, ail_r_axis=ail_r_axis,
		   rv_l_p=rv_l_p, rv_l_axis=rv_l_axis, rv_r_p=rv_r_p, rv_r_axis=rv_r_axis,
		   mot=[mot_x, 0, mot_z], max_rot=max_rot, fus_box=fus_box, fus_ctr=fus_ctr,
		   wing_box=wing_box, wing_ctr=wing_ctr)

	with open(os.path.join(a.model_dir, "model.sdf"), "w") as f:
		f.write(sdf(geo))

	with open(os.path.join(a.model_dir, "model.config"), "w") as f:
		f.write(MODEL_CONFIG)

	report = {
		"span_m": span, "wing_area_m2": area, "tail_area_top_view_m2": tail, "mac_m": mac,
		"aspect_ratio": ar, "length_m": allp[:, 0].max() - allp[:, 0].min(),
		"cg_in_step_frame_m": (R_STEP_TO_GZ.T @ cg).tolist(),
		"cg_behind_root_le_m": le_root, "cg_percent_mac": 100 * le_root / mac,
		"aileron_span_m": ail_span, "vtail_dihedral_deg": np.degrees(vtail_dihedral),
		"motor_pos_m": geo["mot"], "max_rot_vel_rad_s": max_rot,
		"inertia_kgm2": np.diag(inertia).tolist(), "belly_z_m": belly_z,
		"wing_loading_kg_m2": MASS / area,
		"stall_speed_est_m_s": float(np.sqrt(2 * MASS * 9.81 / (1.2041 * area * 1.2))),
	}

	for k_, v in report.items():
		print(f"{k_:26s} {np.round(v, 4).tolist() if isinstance(v, (list, np.ndarray)) else round(float(v), 4)}")

	with open(os.path.join(a.parts_dir, "talon_report.json"), "w") as f:
		json.dump(report, f, indent=1, default=float)


MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>talon1718</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>X-UAV Talon 1718 target, geometry from CAD, aerodynamics adapted from advanced_plane</description>
</model>
"""


def surface_link(name, p, mesh=None, color="0.1 0.1 0.1 1"):
	color = color if isinstance(color, str) else fmt(list(color) + [1], 3)
	visual = "" if mesh is None else f"""
      <visual name="{name}_visual">
        <geometry><mesh><uri>model://talon1718/meshes/{mesh}</uri></mesh></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>"""
	return f"""
    <link name="{name}">
      <pose>{fmt(p)} 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz></inertia>
      </inertial>{visual}
    </link>"""


def servo_joint(name, child, axis, limit):
	return f"""
    <joint name="{name}" type="revolute">
      <parent>base_link</parent>
      <child>{child}</child>
      <axis>
        <xyz expressed_in="__model__">{fmt(axis)}</xyz>
        <limit><lower>{-limit}</lower><upper>{limit}</upper></limit>
        <dynamics><damping>1.0</damping></dynamics>
      </axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{name}</joint_name>
      <sub_topic>{name}</sub_topic>
      <p_gain>10</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0</d_gain>
    </plugin>"""


def skid(name, x, y, z):
	return f"""
      <collision name="{name}">
        <pose>{x:.4f} {y:.4f} {z + 0.015:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.015</radius></sphere></geometry>
        <surface>
          <friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>
          <contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
        </surface>
      </collision>"""


def box_collision(name, ctr, size):
	return f"""
      <collision name="{name}">
        <pose>{fmt(ctr)} 0 0 0</pose>
        <geometry><box><size>{fmt(size)}</size></box></geometry>
      </collision>"""


def control_surface(name, index, direction, c):
	return f"""
      <control_surface>
        <name>{name}</name>
        <index>{index}</index>
        <direction>{direction}</direction>
        <CD_ctrl>{c[0]:.6f}</CD_ctrl>
        <CY_ctrl>{c[1]:.6f}</CY_ctrl>
        <CL_ctrl>{c[2]:.6f}</CL_ctrl>
        <Cell_ctrl>{c[3]:.6f}</Cell_ctrl>
        <Cem_ctrl>{c[4]:.6f}</Cem_ctrl>
        <Cen_ctrl>{c[5]:.6f}</Cen_ctrl>
      </control_surface>"""


def sdf(g):
	I = g["I"]
	bz = g["belly_z"]
	fx, fz = g["fus_ctr"][0], g["fus_ctr"][2]
	flen = g["fus_box"][0]

	# advanced_plane control derivatives [CD CY CL Cell Cem Cen] per degree.
	# Ailerons are used as-is. The V-tail surfaces each provide half of the
	# advanced_plane elevator + rudder effect; signs match PX4 CA_SV_CS2/3
	# (left: TRQ_P=+0.5 TRQ_Y=+0.5, right: TRQ_P=+0.5 TRQ_Y=-0.5).
	ail_l = [-0.000059, 0.000171, -0.011940, -0.003331, 0.001498, -0.000057]
	ail_r = [-0.000059, -0.000171, -0.011940, 0.003331, 0.001498, 0.000057]
	elev = [0.0, 0.0, -0.010696 / 2, 0.0, 0.025798 / 2, 0.0]
	rud = [0.0, -0.003913 / 2, 0.0, -0.000257 / 2, 0.0, 0.001613 / 2]
	rv_l = [e + r for e, r in zip(elev, rud)]
	rv_r = [e - r for e, r in zip(elev, rud)]

	body_visuals = "".join(f"""
      <visual name="body_{i}_visual"><!-- {cname} -->
        <geometry><mesh><uri>model://talon1718/meshes/{fname}</uri></mesh></geometry>
        <material><ambient>{fmt(list(rgb) + [1], 3)}</ambient><diffuse>{fmt(list(rgb) + [1], 3)}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>""" for i, (fname, cname, rgb) in enumerate(g["body_visuals"]))
	body_visuals += f"""
      <visual name="canopy_visual">
        <geometry><mesh><uri>model://talon1718/meshes/canopy.stl</uri></mesh></geometry>
        <material><ambient>{fmt(list(g["canopy_rgb"]) + [1], 3)}</ambient><diffuse>{fmt(list(g["canopy_rgb"]) + [1], 3)}</diffuse><specular>0.9 0.9 0.9 1</specular></material>
        <transparency>{CANOPY_TRANSPARENCY}</transparency>
      </visual>"""

	return f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_talon.py, do not edit by hand -->
<sdf version="1.9">
  <model name="talon1718">
    <pose>0 0 {-bz + 0.03:.3f} 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>{g["mass"]:.3f}</mass>
        <inertia>
          <ixx>{I[0, 0]:.6f}</ixx><ixy>0</ixy><ixz>{I[0, 2]:.6f}</ixz>
          <iyy>{I[1, 1]:.6f}</iyy><iyz>0</iyz><izz>{I[2, 2]:.6f}</izz>
        </inertia>
      </inertial>
{body_visuals}{box_collision("fuselage_collision", g["fus_ctr"], g["fus_box"])}{box_collision("wing_collision", g["wing_ctr"], g["wing_box"])}{skid("skid_front", fx + 0.35 * flen, 0, bz)}{skid("skid_left", fx - 0.1 * flen, 0.04, bz)}{skid("skid_right", fx - 0.1 * flen, -0.04, bz)}
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.0003394</stddev><dynamic_bias_stddev>3.8785e-05</dynamic_bias_stddev><dynamic_bias_correlation_time>1000</dynamic_bias_correlation_time></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.004</stddev><dynamic_bias_stddev>0.006</dynamic_bias_stddev><dynamic_bias_correlation_time>300</dynamic_bias_correlation_time></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>50</update_rate>
        <air_pressure><pressure><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></pressure></air_pressure>
      </sensor>
      <sensor name="magnetometer_sensor" type="magnetometer">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <magnetometer>
          <x><noise type="gaussian"><stddev>0.0001</stddev></noise></x>
          <y><noise type="gaussian"><stddev>0.0001</stddev></noise></y>
          <z><noise type="gaussian"><stddev>0.0001</stddev></noise></z>
        </magnetometer>
      </sensor>
      <sensor name="navsat_sensor" type="navsat">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <link name="rotor_puller">
      <pose>{fmt(g["mot"])} 0 1.5708 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.000166704</iyy><iyz>0</iyz><izz>0.000167604</izz></inertia>
      </inertial>
      <visual name="rotor_puller_visual">
        <geometry><mesh><scale>{PROP_RADIUS / 0.129:.3f} {PROP_RADIUS / 0.129:.3f} 1</scale><uri>model://rc_cessna/meshes/iris_prop_ccw.dae</uri></mesh></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_puller_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_puller</child>
      <axis>
        <xyz expressed_in="__model__">1 0 0</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
      </axis>
    </joint>
{surface_link("left_aileron", g["ail_l_p"], "aileron_left.stl", g["ail_rgb"])}{surface_link("right_aileron", g["ail_r_p"], "aileron_right.stl", g["ail_rgb"])}{surface_link("left_ruddervator", g["rv_l_p"])}{surface_link("right_ruddervator", g["rv_r_p"])}
{servo_joint("servo_0", "left_aileron", g["ail_l_axis"], AIL_LIMIT)}{servo_joint("servo_1", "right_aileron", g["ail_r_axis"], AIL_LIMIT)}{servo_joint("servo_2", "left_ruddervator", g["rv_l_axis"], RUDV_LIMIT)}{servo_joint("servo_3", "right_ruddervator", g["rv_r_axis"], RUDV_LIMIT)}

    <!-- stability derivatives from advanced_plane, reference geometry from the Talon CAD -->
    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
      <a0>0.0</a0>
      <CL0>0.15188</CL0>
      <AR>{g["ar"]:.3f}</AR>
      <eff>0.97</eff>
      <CLa>5.015</CLa>
      <CD0>0.029</CD0>
      <Cem0>0.075</Cem0>
      <Cema>-0.463966</Cema>
      <CYb>-0.258244</CYb>
      <Cellb>-0.039250</Cellb>
      <Cenb>0.100826</Cenb>
      <CDp>0.0</CDp>
      <CYp>0.065861</CYp>
      <CLp>0.0</CLp>
      <Cellp>-0.487407</Cellp>
      <Cemp>0.0</Cemp>
      <Cenp>-0.040416</Cenp>
      <CDq>0.055166</CDq>
      <CYq>0.0</CYq>
      <CLq>7.971792</CLq>
      <Cellq>0.0</Cellq>
      <Cemq>-12.140140</Cemq>
      <Cenq>0.0</Cenq>
      <CDr>0.0</CDr>
      <CYr>0.230299</CYr>
      <CLr>0.0</CLr>
      <Cellr>0.078165</Cellr>
      <Cemr>0.0</Cemr>
      <Cenr>-0.089947</Cenr>
      <alpha_stall>0.3391428111</alpha_stall>
      <CLa_stall>-3.85</CLa_stall>
      <CDa_stall>-0.9233984055</CDa_stall>
      <Cema_stall>0</Cema_stall>
      <cp>{g["ac_x"]:.4f} 0.0 0.0</cp>
      <area>{g["area"]:.4f}</area>
      <mac>{g["mac"]:.4f}</mac>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <num_ctrl_surfaces>4</num_ctrl_surfaces>{control_surface("servo_0", 0, 1, ail_l)}{control_surface("servo_1", 1, 1, ail_r)}{control_surface("servo_2", 2, 1, rv_l)}{control_surface("servo_3", 3, 1, rv_r)}
    </plugin>

    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_puller_joint</jointName>
      <linkName>rotor_puller</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{g["max_rot"]:.0f}</maxRotVelocity>
      <motorConstant>{MOTOR_CONSTANT}</motorConstant>
      <momentConstant>0.01</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
  </model>
</sdf>
"""


if __name__ == "__main__":
	main()

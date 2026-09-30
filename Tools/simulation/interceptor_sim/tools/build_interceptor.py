#!/usr/bin/env python3
"""
Generate the interceptor Gazebo model and its PX4 airframe from interceptor.yaml.

Everything is built from primitives: the CG and inertia are computed from the
component masses, then the motors and wings are placed relative to the CG.
Motors use the x500 motor model scaled to the configured maximum thrust, with
the x500 propeller mesh scaled to the configured diameter.

Aerodynamics: the wing plates use LiftDrag (same axes as PX4's quadtailsitter:
chord along the fuselage, lift towards -x, i.e. up when pitched forward); the
body drag is quadratic per body axis with the Hydrodynamics system. Since the
motor model has no thrust loss with inflow speed, extra axial drag is added so
that level flight at full thrust reaches aero.top_speed.

    python3 tools/build_interceptor.py [interceptor.yaml]
      -> models/interceptor/, PX4 airframe 4051_gz_interceptor, report on stdout
"""

import math
import os
import sys

import numpy as np
import yaml

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))
AIRFRAME = os.path.join(PX4_DIR, "ROMFS", "px4fmu_common", "init.d-posix", "airframes", "4051_gz_interceptor")
RHO = 1.2041
G = 9.81

# x500 13" propeller mesh (scale 1): length and the offset of the mesh origin from the hub
PROP_MESH_LENGTH = 0.33
PROP_MESH_OFFSET = np.array([-0.026, -0.173, -0.0189])


def box_inertia(m, sx, sy, sz):
	return np.diag([m * (sy * sy + sz * sz), m * (sx * sx + sz * sz), m * (sx * sx + sy * sy)]) / 12


def parallel_axis(I, m, p):
	p = np.asarray(p, dtype=float)
	return I + m * (np.dot(p, p) * np.eye(3) - np.outer(p, p))


def fmt(v, n=4):
	return " ".join(f"{x:.{n}f}" for x in np.ravel(v))


def main():
	cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIM_DIR, "interceptor.yaml")
	c = yaml.safe_load(open(cfg_path))
	fus, mot, wing, aero = c["fuselage"], c["motors"], c["wings"], c["aero"]
	r_f = fus["diameter"] / 2
	L = fus["length"]
	n_mot = mot["count"]
	assert n_mot == 4, "only the + quad layout is implemented"

	# ---- CG along the axis (from the tail end). Fixed parts: shell + components; motors, arms
	# and wings sit at a given offset from the CG, so solve m_tot*cg = sum(m_i z_i) + m_rel*(cg + d)
	fixed = [(fus["mass"], L / 2)] + [(p["mass"], p["z"]) for p in c["components"].values()]
	m_fixed = sum(m for m, _ in fixed)
	m_mot = n_mot * mot["mass"] + mot["arm_mass"]
	m_wing = wing["mass"]
	m_tot = m_fixed + m_mot + m_wing
	cg = (sum(m * z for m, z in fixed) + m_mot * mot["z_from_cg"] + m_wing * wing["z_from_cg"]) / m_fixed
	z_mot = mot["z_from_cg"]          # CG frame from here on
	z_wing = wing["z_from_cg"]

	# ---- inertia about the CG
	I = np.zeros((3, 3))
	# fuselage: thin cylindrical shell
	I += parallel_axis(np.diag([fus["mass"] * (r_f ** 2 / 2 + L ** 2 / 12)] * 2 + [fus["mass"] * r_f ** 2]),
			   fus["mass"], [0, 0, L / 2 - cg])

	for p in c["components"].values():
		I += parallel_axis(box_inertia(p["mass"], *p["size"]), p["mass"], [0, 0, p["z"] - cg])

	R = mot["arm_radius"]
	arm_len = R - r_f
	motor_pos = [np.array([R, 0, z_mot]), np.array([-R, 0, z_mot]), np.array([0, R, z_mot]), np.array([0, -R, z_mot])]
	arm_m = mot["arm_mass"] / n_mot

	for p in motor_pos:
		I += parallel_axis(np.zeros((3, 3)), mot["mass"], p)
		along_x = abs(p[0]) > 0
		rod = np.diag([0, arm_m * arm_len ** 2 / 12, arm_m * arm_len ** 2 / 12]) if along_x else \
			np.diag([arm_m * arm_len ** 2 / 12, 0, arm_m * arm_len ** 2 / 12])
		I += parallel_axis(rod, arm_m, p * (r_f + arm_len / 2) / R + [0, 0, z_mot * (1 - (r_f + arm_len / 2) / R)])

	wing_y = r_f + wing["span_out"] / 2
	for s in (1, -1):
		I += parallel_axis(box_inertia(m_wing / 2, wing["thickness"], wing["span_out"], wing["chord"]),
				   m_wing / 2, [0, s * wing_y, z_wing])

	I[np.abs(I) < 1e-9] = 0

	# ---- propulsion
	t_max = mot["max_thrust_n"]
	k = mot["motor_constant"]
	w_max = math.sqrt(t_max / k)
	w_min = 150.0
	weight = m_tot * G
	w_hover = math.sqrt(weight / n_mot / k)
	thr_hover = (w_hover - w_min) / (w_max - w_min)
	prop_d = mot["prop_diameter_inch"] * 0.0254
	prop_scale = prop_d / PROP_MESH_LENGTH

	# clearances
	clear_fus = R - r_f - prop_d / 2
	clear_prop = R * math.sqrt(2) - prop_d

	# ---- aerodynamics
	a_side = fus["diameter"] * L + 2 * arm_len * mot["arm_diameter"]
	a_axial = math.pi * r_f ** 2
	c_side = 0.5 * RHO * aero["cd_body_side"] * a_side
	c_axial = 0.5 * RHO * aero["cd_body_axial"] * a_axial
	# extra axial drag so that level flight at full thrust tops out at aero.top_speed:
	# thrust T at tilt th with T cos(th) = weight, horizontal balance
	# T sin(th) = c_z (v sin th)^2 sin th + c_x (v cos th)^2 cos th
	T = n_mot * t_max
	cos_th = min(1.0, weight / T)
	sin_th = math.sqrt(1 - cos_th ** 2)
	v = aero["top_speed"]
	c_axial_total = max(c_axial, (T * sin_th - c_side * v * v * cos_th ** 3) / (v * v * sin_th ** 3))
	wing_area = 2 * wing["chord"] * wing["span_out"]
	wing_ar = (2 * wing["span_out"] + fus["diameter"]) ** 2 / wing_area
	wing_cla = 2 * math.pi * wing_ar / (wing_ar + 2)

	# ---- write model
	model_dir = os.path.join(SIM_DIR, "models", c["name"])
	os.makedirs(model_dir, exist_ok=True)

	with open(os.path.join(model_dir, "model.config"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<model>
  <name>{c["name"]}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Net interceptor quad (bullet fuselage, + motors, wing plates), generated from interceptor.yaml</description>
</model>
""")

	geo = dict(c=c, cg=cg, m=m_tot, I=I, motor_pos=motor_pos, z_mot=z_mot, z_wing=z_wing, wing_y=wing_y,
		   w_max=w_max, prop_scale=prop_scale, c_side=c_side, c_axial=c_axial_total,
		   wing_area=wing_area, wing_cla=wing_cla, arm_len=arm_len)

	with open(os.path.join(model_dir, "model.sdf"), "w") as f:
		f.write(sdf(geo))

	with open(AIRFRAME, "w") as f:
		f.write(airframe(geo, thr_hover, w_min))

	print(f"mass            {m_tot:.3f} kg   CG {cg:.3f} m above the tail end")
	print(f"inertia         Ixx {I[0, 0]:.4f}  Iyy {I[1, 1]:.4f}  Izz {I[2, 2]:.4f} kg m^2")
	print(f"motors          plane {z_mot:+.3f} m from CG ({cg + z_mot:.3f} m above tail), arm radius {R:.3f} m")
	print(f"wings           centre {z_wing:+.3f} m from CG, area {wing_area:.4f} m^2, CLa {wing_cla:.2f}")
	print(f"thrust          {n_mot} x {t_max:.1f} N, T/W {T / weight:.2f}, hover throttle {thr_hover:.2f}, "
	      f"max {w_max:.0f} rad/s")
	print(f"prop clearance  to fuselage {clear_fus * 100:.1f} cm, between props {clear_prop * 100:.1f} cm"
	      + ("   WARNING: overlap" if min(clear_fus, clear_prop) < 0 else ""))
	print(f"max level tilt  {math.degrees(math.acos(cos_th)):.0f} deg, body drag side {c_side:.4f}, "
	      f"axial {c_axial:.4f} -> {c_axial_total:.4f} (top speed {v:.0f} m/s)")
	print(f"wrote {os.path.relpath(model_dir, SIM_DIR)}/ and {os.path.relpath(AIRFRAME, PX4_DIR)}")


def cylinder_visual(name, pose, radius, length, color):
	return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""


def sdf(g):
	c, cg, I = g["c"], g["cg"], g["I"]
	fus, mot, wing = c["fuselage"], c["motors"], c["wings"]
	r_f = fus["diameter"] / 2
	L_cyl = fus["length"] - fus["nose_length"]
	body = "0.12 0.12 0.13 1"
	dark = "0.05 0.05 0.05 1"
	vis = cylinder_visual("fuselage_visual", f"0 0 {L_cyl / 2 - cg:.4f} 0 0 0", r_f, L_cyl, body)
	vis += f"""
      <visual name="nose_visual">
        <pose>0 0 {L_cyl - cg:.4f} 0 0 0</pose>
        <geometry><ellipsoid><radii>{r_f:.4f} {r_f:.4f} {fus["nose_length"]:.4f}</radii></ellipsoid></geometry>
        <material><ambient>{body}</ambient><diffuse>{body}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <visual name="camera_lens_visual">
        <pose>0 0 {fus["length"] - cg - 0.004:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.012</radius></sphere></geometry>
        <material><ambient>0.02 0.02 0.05 1</ambient><diffuse>0.02 0.02 0.05 1</diffuse><specular>0.9 0.9 0.9 1</specular></material>
      </visual>"""

	for i, p in enumerate(g["motor_pos"]):
		mid = p * (r_f + g["arm_len"] / 2) / mot["arm_radius"]
		rpy = "0 1.5708 0" if abs(p[0]) > 0 else "1.5708 0 0"
		vis += cylinder_visual(f"arm_{i}_visual", f"{mid[0]:.4f} {mid[1]:.4f} {g['z_mot']:.4f} {rpy}",
				       mot["arm_diameter"] / 2, g["arm_len"], body)
		vis += cylinder_visual(f"motor_{i}_visual", f"{p[0]:.4f} {p[1]:.4f} {g['z_mot']:.4f} 0 0 0", 0.018, 0.045, dark)

	for s, side in ((1, "left"), (-1, "right")):
		vis += f"""
      <visual name="wing_{side}_visual">
        <pose>0 {s * g["wing_y"]:.4f} {g["z_wing"]:.4f} 0 0 0</pose>
        <geometry><box><size>{wing["thickness"]} {wing["span_out"]} {wing["chord"]}</size></box></geometry>
        <material><ambient>{body}</ambient><diffuse>{body}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""

	# collisions: fuselage (it stands on its tail), arm tips so that it does not tip over into the props
	col = f"""
      <collision name="fuselage_collision">
        <pose>0 0 {fus["length"] / 2 - cg:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r_f:.4f}</radius><length>{fus["length"]:.4f}</length></cylinder></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>"""

	for i, p in enumerate(g["motor_pos"]):
		col += f"""
      <collision name="motor_{i}_collision">
        <pose>{p[0]:.4f} {p[1]:.4f} {g['z_mot']:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.018</radius><length>0.045</length></cylinder></geometry>
      </collision>"""

	camera = ""

	if c.get("camera", {}).get("enabled"):
		cam = c["camera"]
		camera = f"""
      <sensor name="nose_camera" type="camera">
        <pose>0 0 {fus["length"] - cg:.4f} 0 -1.5708 0</pose>
        <always_on>1</always_on>
        <update_rate>{cam["rate"]}</update_rate>
        <camera>
          <horizontal_fov>{math.radians(cam["hfov_deg"]):.4f}</horizontal_fov>
          <image><width>{cam["width"]}</width><height>{cam["height"]}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>3000</far></clip>
        </camera>
      </sensor>"""

	rotors = ""
	plugins = ""
	directions = ["ccw", "ccw", "cw", "cw"]  # front/back and left/right pairs spin the same way
	s = g["prop_scale"]
	prop_off = PROP_MESH_OFFSET * s

	for i, (p, d) in enumerate(zip(g["motor_pos"], directions)):
		rotors += f"""
    <link name="rotor_{i}">
      <pose>{p[0]:.4f} {p[1]:.4f} {g['z_mot'] - 0.035:.4f} 0 0 0</pose>
      <inertial>
        <mass>0.008</mass>
        <inertia><ixx>1e-07</ixx><ixy>0</ixy><ixz>0</ixz><iyy>7e-06</iyy><iyz>0</iyz><izz>7e-06</izz></inertia>
      </inertial>
      <visual name="rotor_{i}_visual">
        <pose>{fmt(prop_off)} 0 0 0</pose>
        <geometry><mesh><scale>{s:.4f} {s:.4f} {s:.4f}</scale><uri>model://x500_base/meshes/1345_prop_{d}.stl</uri></mesh></geometry>
        <material><ambient>0.55 0.55 0.55 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>
      </visual>
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
      </axis>
    </joint>"""
		plugins += f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{d}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{g["w_max"]:.1f}</maxRotVelocity>
      <motorConstant>{mot["motor_constant"]}</motorConstant>
      <momentConstant>{mot["moment_constant"]}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""

	noise = lambda sd, b, t: (f"<noise type=\"gaussian\"><mean>0</mean><stddev>{sd}</stddev>"
				  f"<dynamic_bias_stddev>{b}</dynamic_bias_stddev>"
				  f"<dynamic_bias_correlation_time>{t}</dynamic_bias_correlation_time></noise>")
	gyro = "".join(f"<{a}>{noise(0.0003394, 3.8785e-05, 1000)}</{a}>" for a in "xyz")
	acc = "".join(f"<{a}>{noise(0.004, 0.006, 300)}</{a}>" for a in "xyz")

	return f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_interceptor.py from interceptor.yaml, do not edit by hand -->
<sdf version="1.9">
  <model name="{c["name"]}">
    <pose>0 0 {cg + 0.01:.3f} 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>{g["m"]:.4f}</mass>
        <inertia>
          <ixx>{I[0, 0]:.6f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{I[1, 1]:.6f}</iyy><iyz>0</iyz><izz>{I[2, 2]:.6f}</izz>
        </inertia>
      </inertial>{vis}{col}
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>base_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
        <imu><angular_velocity>{gyro}</angular_velocity><linear_acceleration>{acc}</linear_acceleration></imu>
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
      </sensor>{camera}
    </link>
{rotors}
{plugins}

    <!-- wing plates: chord along the fuselage, lift towards -x (up when pitched forward) -->
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>base_link</link_name>
      <air_density>{RHO}</air_density>
      <area>{g["wing_area"]:.4f}</area>
      <a0>0</a0>
      <cla>{g["wing_cla"]:.3f}</cla>
      <cda>0.65</cda>
      <cma>0</cma>
      <alpha_stall>0.6</alpha_stall>
      <cla_stall>-1.5</cla_stall>
      <cda_stall>0.9</cda_stall>
      <cp>0 0 {g["z_wing"]:.4f}</cp>
      <forward>0 0 1</forward>
      <upward>-1 0 0</upward>
    </plugin>

    <!-- body drag: quadratic per body axis (x/y cross flow, z axial incl. propeller inflow losses) -->
    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
      <link_name>base_link</link_name>
      <xUabsU>{-g["c_side"]:.5f}</xUabsU>
      <yVabsV>{-g["c_side"]:.5f}</yVabsV>
      <zWabsW>{-g["c_axial"]:.5f}</zWabsW>
      <kPabsP>-0.0005</kPabsP>
      <mQabsQ>-0.0005</mQabsQ>
      <nRabsR>-0.0002</nRabsR>
      <disable_added_mass>true</disable_added_mass>
      <disable_coriolis>true</disable_coriolis>
    </plugin>
  </model>
</sdf>
"""


def airframe(g, thr_hover, w_min):
	c = g["c"]
	lines = []

	# PX4 body frame is FRD: x forward, y right, z down; KM > 0 for ccw
	for i, (p, d) in enumerate(zip(g["motor_pos"], ["ccw", "ccw", "cw", "cw"])):
		lines += [f"param set-default CA_ROTOR{i}_PX {p[0]:.3f}",
			  f"param set-default CA_ROTOR{i}_PY {-p[1] + 0.0:.3f}",
			  f"param set-default CA_ROTOR{i}_PZ {-p[2] + 0.0:.3f}",
			  f"param set-default CA_ROTOR{i}_KM {0.05 if d == 'ccw' else -0.05}"]

	for i in range(1, 5):
		lines += [f"param set-default SIM_GZ_EC_FUNC{i} {100 + i}",
			  f"param set-default SIM_GZ_EC_MIN{i} {w_min:.0f}",
			  f"param set-default SIM_GZ_EC_MAX{i} {g['w_max']:.0f}"]

	body = "\n".join(lines)
	return f"""#!/bin/sh
#
# @name Net interceptor quad (Gazebo)
#
# @type Quadrotor +
#
# Generated by Tools/simulation/interceptor_sim/tools/build_interceptor.py from
# interceptor.yaml, do not edit by hand. Model: Tools/simulation/interceptor_sim/models/{c["name"]}
#

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=default}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={c["name"]}}}

param set-default SIM_GZ_EN 1

param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4
{body}

param set-default MPC_THR_HOVER {thr_hover:.2f}

# fast and aggressive: strong tilt, high speeds / accelerations
param set-default MPC_TILTMAX_AIR 70
param set-default MPC_XY_VEL_MAX 20
param set-default MPC_XY_CRUISE 15
param set-default MPC_ACC_HOR_MAX 15
param set-default MPC_ACC_HOR 10
param set-default MPC_Z_VEL_MAX_UP 8

# manual flight from the ground station: Altitude / Stabilized modes are limited by tilt only
param set-default MPC_MAN_TILT_MAX 70
param set-default MPC_VEL_MANUAL 20

# the wing plates carry part of the weight at speed; with decoupled tilt the controller would
# reduce thrust at constant tilt, lose horizontal thrust and wind the velocity integrator down
param set-default MPC_ACC_DECOUPLE 0

# flies at up to MPC_TILTMAX_AIR: do not flag that as an attitude failure
param set-default FD_FAIL_P 85
param set-default FD_FAIL_R 85

# autonomous: no GCS needed to arm, no datalink-loss failsafe
param set-default NAV_DLL_ACT 0
"""


if __name__ == "__main__":
	main()

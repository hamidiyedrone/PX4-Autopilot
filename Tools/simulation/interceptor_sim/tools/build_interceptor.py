#!/usr/bin/env python3
"""
Generate the Octopus interceptor Gazebo model and its PX4 airframe from interceptor.yaml.

Ukrainian Octopus interceptor drone:
- Slender cylindrical fuselage (100 cm length, 14 cm diameter)
- 4 aerodynamic wing-shaped arms in Quadrotor "X" layout (50 cm span between opposite motors)
- 10" propellers, high-performance BrotherHobby brushless motors
- 4 carbon landing legs extending past the tail (drone stands upright on its 4 legs)
- 4 stabilizing tail fins at the tail with flared endplates
- Nose camera / seeker on the thrust axis
- Flown as a quadrotor tailsitter (nose up in hover, tilted forward at high speed)

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
BUILD_AIRFRAME = os.path.join(PX4_DIR, "build", "px4_sitl_default", "etc", "init.d-posix", "airframes", "4051_gz_interceptor")
RHO = 1.2041
G = 9.81

# x500 13" propeller mesh (scale 1): length and offset of mesh origin from hub
PROP_MESH_LENGTH = 0.33
PROP_MESH_OFFSET = np.array([-0.026, -0.173, -0.0189])


def box_inertia(m, sx, sy, sz):
	return np.diag([m * (sy * sy + sz * sz), m * (sx * sx + sz * sz), m * (sx * sx + sy * sy)]) / 12


def parallel_axis(I, m, p):
	p = np.asarray(p, dtype=float)
	return I + m * (np.dot(p, p) * np.eye(3) - np.outer(p, p))


def rotate_z(I, angle_rad):
	c, s = math.cos(angle_rad), math.sin(angle_rad)
	R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
	return R @ I @ R.T


def fmt(v, n=4):
	return " ".join(f"{x:.{n}f}" for x in np.ravel(v))


def cylinder_visual(name, pose, radius, length, color):
	return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""


def main():
	cfg_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(SIM_DIR, "interceptor.yaml")
	c = yaml.safe_load(open(cfg_path))
	fus, mot, wing, aero = c["fuselage"], c["motors"], c["wings"], c["aero"]
	fins = c.get("tail_fins", {"count": 4, "mass": 0.120, "span_out": 0.09, "chord": 0.16, "thickness": 0.006, "z_from_tail": 0.04})
	r_f = fus["diameter"] / 2
	L = fus["length"]
	n_mot = mot["count"]
	assert n_mot == 4, "only 4-motor quad layout is implemented"

	# Layout: Quadrotor "X"
	R = mot["arm_radius"]  # distance from fuselage axis to motor axis (0.25 m for 50 cm span)
	d_xy = R / math.sqrt(2)
	arm_len = R - r_f
	wing_chord = wing["chord"]
	wing_thick = wing["thickness"]
	z_wing = wing.get("z_from_cg", -0.04)
	# Motor bottom aligns directly with the upper (leading) edge of the wing surfaces
	z_mot = z_wing + wing_chord / 2

	# 4 motor positions in Gazebo body FLU (+x forward, +y left, +z up towards nose):
	# Motor 0: Front-Right (+x, -y) -> CCW
	# Motor 1: Back-Left (-x, +y)   -> CCW
	# Motor 2: Front-Left (+x, +y)  -> CW
	# Motor 3: Back-Right (-x, -y)  -> CW
	motor_angles = [-math.pi / 4, 3 * math.pi / 4, math.pi / 4, -3 * math.pi / 4]
	directions = ["ccw", "ccw", "cw", "cw"]
	motor_pos = [
		np.array([d_xy, -d_xy, z_mot]),
		np.array([-d_xy, d_xy, z_mot]),
		np.array([d_xy, d_xy, z_mot]),
		np.array([-d_xy, -d_xy, z_mot]),
	]

	# ---- CG along the fuselage axis (measured from the tail end z=0)
	# Fixed parts: shell + internal components + tail fins + gimbal
	z_fin_center = fins["z_from_tail"] + fins["chord"] / 2
	gimbal = c.get("gimbal", {})
	gimbal_enabled = gimbal.get("enabled", False)
	m_gimbal = gimbal.get("mass", 0.120) if gimbal_enabled else 0.0
	z_gimbal = L - 0.015

	fixed = [(fus["mass"], L / 2)] + [(p["mass"], p["z"]) for p in c["components"].values()] + [(fins["mass"], z_fin_center)]
	if m_gimbal > 0:
		fixed.append((m_gimbal, z_gimbal))

	m_fixed = sum(m for m, _ in fixed)
	m_mot = n_mot * mot["mass"]
	m_arms = mot["arm_mass"]  # 4 wings + nacelles + landing legs
	m_tot = m_fixed + m_mot + m_arms

	# Solve for CG: m_tot * cg = sum(m_fixed * z) + (m_mot + m_arms) * (cg + z_mot)
	cg = (sum(m * z for m, z in fixed) + m_mot * z_mot + m_arms * z_wing) / m_fixed

	# ---- Inertia about CG
	I = np.zeros((3, 3))
	# Fuselage: thin cylindrical shell
	I += parallel_axis(np.diag([fus["mass"] * (r_f ** 2 / 2 + L ** 2 / 12)] * 2 + [fus["mass"] * r_f ** 2]),
			   fus["mass"], [0, 0, L / 2 - cg])

	# Internal components
	for p in c["components"].values():
		I += parallel_axis(box_inertia(p["mass"], *p["size"]), p["mass"], [0, 0, p["z"] - cg])

	if m_gimbal > 0:
		I += parallel_axis(np.diag([1e-5, 1e-5, 1e-5]), m_gimbal, [0, 0, z_gimbal - cg])

	# 4 Tail fins
	m_single_fin = fins["mass"] / 4
	fin_span = fins["span_out"]
	fin_chord = fins["chord"]
	fin_thick = fins["thickness"]
	r_fin_mid = r_f + fin_span / 2
	for angle in motor_angles:
		# Local box along radial axis
		I_fin_local = box_inertia(m_single_fin, fin_span, fin_thick, fin_chord)
		I_fin_rot = rotate_z(I_fin_local, angle)
		pos_fin = np.array([r_fin_mid * math.cos(angle), r_fin_mid * math.sin(angle), z_fin_center - cg])
		I += parallel_axis(I_fin_rot, m_single_fin, pos_fin)

	# 4 Motors + Wing arms
	m_arm_single = m_arms / n_mot
	r_arm_mid = r_f + arm_len / 2
	wing_chord = wing["chord"]
	wing_thick = wing["thickness"]

	for i, (p, angle) in enumerate(zip(motor_pos, motor_angles)):
		# Motor concentrated mass
		I += parallel_axis(np.zeros((3, 3)), mot["mass"], p)
		# Wing arm box: span along radial, thickness normal, chord along z
		I_arm_local = box_inertia(m_arm_single, arm_len, wing_thick, wing_chord)
		I_arm_rot = rotate_z(I_arm_local, angle)
		p_arm_mid = np.array([r_arm_mid * math.cos(angle), r_arm_mid * math.sin(angle), z_wing])
		I += parallel_axis(I_arm_rot, m_arm_single, p_arm_mid)

	I[np.abs(I) < 1e-9] = 0

	# ---- Propulsion
	t_max = mot["max_thrust_n"]
	k = mot["motor_constant"]
	w_max = math.sqrt(t_max / k)
	w_min = 150.0
	weight = m_tot * G
	w_hover = math.sqrt(weight / n_mot / k)
	thr_hover = weight / (n_mot * t_max)
	prop_d = mot["prop_diameter_inch"] * 0.0254
	prop_scale = prop_d / PROP_MESH_LENGTH

	# Clearances
	clear_fus = arm_len - prop_d / 2
	clear_prop = 2 * d_xy - prop_d

	# ---- Aerodynamics
	# Side area: fuselage projected area + wing arms projected side area
	a_side = fus["diameter"] * L + 2 * arm_len * wing_chord * math.sqrt(2)
	a_axial = math.pi * r_f ** 2
	c_side = 0.5 * RHO * aero["cd_body_side"] * a_side
	c_axial = 0.5 * RHO * aero["cd_body_axial"] * a_axial

	# Drag balance for top_speed in level flight
	T = n_mot * t_max
	cos_th = min(1.0, weight / T)
	sin_th = math.sqrt(1 - cos_th ** 2)
	v = aero["top_speed"]
	c_axial_total = max(c_axial, (T * sin_th - c_side * v * v * cos_th ** 3) / (v * v * sin_th ** 3))

	# Total wing area (all 4 wings)
	wing_area_total = 4 * arm_len * wing_chord
	wing_pair_area = 2 * arm_len * wing_chord
	wing_ar = arm_len / wing_chord
	wing_cla = 2 * math.pi * wing_ar / (wing_ar + 2)

	# Ground contact: landing legs extend past the tail end
	leg_clearance = wing.get("leg_clearance", 0.05)
	z_ground_rel_cg = -cg - leg_clearance
	spawn_z = cg + leg_clearance + 0.01  # spawn cleanly standing on legs

	# ---- Write model
	model_dir = os.path.join(SIM_DIR, "models", c["name"])
	os.makedirs(model_dir, exist_ok=True)

	with open(os.path.join(model_dir, "model.config"), "w") as f:
		f.write(f"""<?xml version="1.0"?>
<model>
  <name>{c["name"]}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Ukrainian Octopus interceptor drone (slender cylinder, X-wing arms, 10" props, tail fins), generated from interceptor.yaml</description>
</model>
""")

	geo = dict(c=c, cg=cg, m=m_tot, I=I, motor_pos=motor_pos, motor_angles=motor_angles,
		   directions=directions, z_mot=z_mot, z_wing=z_wing, w_max=w_max,
		   prop_scale=prop_scale, c_side=c_side, c_axial=c_axial_total,
		   wing_area_total=wing_area_total, wing_pair_area=wing_pair_area,
		   wing_cla=wing_cla, arm_len=arm_len, r_arm_mid=r_arm_mid,
		   z_ground_rel_cg=z_ground_rel_cg, spawn_z=spawn_z,
		   gimbal=gimbal, m_gimbal=m_gimbal)

	with open(os.path.join(model_dir, "model.sdf"), "w") as f:
		f.write(sdf(geo))

	airframe_content = airframe(geo, thr_hover, w_min)
	with open(AIRFRAME, "w") as f:
		f.write(airframe_content)

	if os.path.exists(os.path.dirname(BUILD_AIRFRAME)):
		with open(BUILD_AIRFRAME, "w") as f:
			f.write(airframe_content)

	if c.get("camera", {}).get("enabled"):
		with open(AIRFRAME + ".post", "w") as f:
			f.write(post(c["camera"]["detect_port"]))
		if os.path.exists(os.path.dirname(BUILD_AIRFRAME)):
			with open(BUILD_AIRFRAME + ".post", "w") as f:
				f.write(post(c["camera"]["detect_port"]))
	elif os.path.exists(AIRFRAME + ".post"):
		os.remove(AIRFRAME + ".post")

	print("=" * 68)
	print(f"OCTOPUS INTERCEPTOR MODEL GENERATED ({c['name']})")
	print("=" * 68)
	print(f"Fuselage:       Length {L * 100:.1f} cm, Diameter {fus['diameter'] * 100:.1f} cm")
	print(f"Motor span:     {2 * R * 100:.1f} cm diagonal (opposite motors), arm radius {R * 100:.1f} cm")
	print(f"Total mass:     {m_tot:.3f} kg (CG {cg * 100:.1f} cm above tail end)")
	print(f"Inertia:        Ixx {I[0, 0]:.4f}  Iyy {I[1, 1]:.4f}  Izz {I[2, 2]:.4f} kg m^2")
	print(f"Propeller:      {mot['prop_diameter_inch']}\" ({prop_d * 100:.1f} cm)")
	print(f"Prop clearance: to fuselage {clear_fus * 100:.1f} cm, between adjacent props {clear_prop * 100:.1f} cm")
	print(f"Thrust:         {n_mot} x {t_max:.1f} N = {T:.1f} N total (T/W {T / weight:.2f}, hover throttle {thr_hover:.2f})")
	print(f"Wings:          4 wing arms in X layout, total area {wing_area_total:.4f} m^2 (chord {wing_chord * 100:.0f} cm)")
	print(f"Tail fins:      4 stabilizing fins at tail (chord {fin_chord * 100:.0f} cm, span {fin_span * 100:.0f} cm)")
	print(f"Standing pose:  Leg bottom {abs(z_ground_rel_cg):.3f} m below CG, spawn z = {spawn_z:.3f} m")
	print(f"Max level tilt: {math.degrees(math.acos(cos_th)):.0f} deg, top speed {v:.0f} m/s")
	print(f"Wrote {os.path.relpath(model_dir, SIM_DIR)}/ and {os.path.relpath(AIRFRAME, PX4_DIR)}")
	print("=" * 68)


def sdf(g):
	c, cg, I = g["c"], g["cg"], g["I"]
	fus, mot, wing = c["fuselage"], c["motors"], c["wings"]
	fins = c.get("tail_fins", {"span_out": 0.09, "chord": 0.16, "thickness": 0.006, "z_from_tail": 0.04})
	r_f = fus["diameter"] / 2
	L_cyl = fus["length"] - fus["nose_length"]

	# Visual colors (matching the real Octopus drone in uploaded photos)
	body_grey = "0.55 0.56 0.58 1"
	accent_white = "0.85 0.85 0.86 1"
	accent_dark = "0.15 0.15 0.16 1"
	carbon_black = "0.06 0.06 0.07 1"
	led_green = "0.1 0.9 0.2 1"
	led_red = "0.9 0.1 0.1 1"

	# Fuselage visuals
	vis = cylinder_visual("fuselage_visual", f"0 0 {L_cyl / 2 - cg:.4f} 0 0 0", r_f, L_cyl, body_grey)
	# Contrast white band near the tail
	band_len = 0.12
	band_z = 0.18 + band_len / 2 - cg
	vis += cylinder_visual("accent_band_visual", f"0 0 {band_z:.4f} 0 0 0", r_f + 0.0005, band_len, accent_white)
	# Dark ring near the nose cone base
	ring_len = 0.02
	ring_z = L_cyl - ring_len / 2 - cg
	vis += cylinder_visual("accent_ring_visual", f"0 0 {ring_z:.4f} 0 0 0", r_f + 0.0005, ring_len, accent_dark)

	# Nose cone
	vis += f"""
      <visual name="nose_visual">
        <pose>0 0 {L_cyl - cg:.4f} 0 0 0</pose>
        <geometry><ellipsoid><radii>{r_f:.4f} {r_f:.4f} {fus["nose_length"]:.4f}</radii></ellipsoid></geometry>
        <material><ambient>{body_grey}</ambient><diffuse>{body_grey}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>"""

	# Optical seeker turret on nose tip
	cam_lens_z = fus["length"] - cg
	gimbal = g.get("gimbal", {})
	gimbal_enabled = gimbal.get("enabled", False)

	if not gimbal_enabled:
		vis += f"""
      <visual name="seeker_turret_visual">
        <pose>0 0 {cam_lens_z - 0.015:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.022</radius><length>0.035</length></cylinder></geometry>
        <material><ambient>{accent_dark}</ambient><diffuse>{accent_dark}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <visual name="camera_lens_visual">
        <pose>0 0 {cam_lens_z:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.014</radius></sphere></geometry>
        <material><ambient>0.02 0.02 0.05 1</ambient><diffuse>0.02 0.02 0.05 1</diffuse><specular>0.95 0.95 0.95 1</specular></material>
      </visual>"""
	else:
		vis += f"""
      <visual name="gimbal_base_collar_visual">
        <pose>0 0 {cam_lens_z - 0.020:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.024</radius><length>0.015</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.22 1</ambient><diffuse>0.2 0.2 0.22 1</diffuse></material>
      </visual>"""

	# 4 Tail fins at the rear
	fin_span = fins["span_out"]
	fin_chord = fins["chord"]
	fin_thick = fins["thickness"]
	r_fin_mid = r_f + fin_span / 2
	z_fin_center = fins["z_from_tail"] + fin_chord / 2 - cg
	for i, angle in enumerate(g["motor_angles"]):
		fx = r_fin_mid * math.cos(angle)
		fy = r_fin_mid * math.sin(angle)
		vis += f"""
      <visual name="tail_fin_{i}_visual">
        <pose>{fx:.4f} {fy:.4f} {z_fin_center:.4f} 0 0 {angle:.4f}</pose>
        <geometry><box><size>{fin_span:.4f} {fin_thick:.4f} {fin_chord:.4f}</size></box></geometry>
        <material><ambient>{body_grey}</ambient><diffuse>{body_grey}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""
		# Flared tip / endplate on tail fin
		ex = (r_f + fin_span) * math.cos(angle)
		ey = (r_f + fin_span) * math.sin(angle)
		vis += f"""
      <visual name="tail_endplate_{i}_visual">
        <pose>{ex:.4f} {ey:.4f} {z_fin_center:.4f} 0 0 {angle:.4f}</pose>
        <geometry><box><size>0.006 0.035 {fin_chord:.4f}</size></box></geometry>
        <material><ambient>{body_grey}</ambient><diffuse>{body_grey}</diffuse><specular>0.2 0.2 0.2 1</specular></material>
      </visual>"""

	# 4 Wing-shaped motor arms, nacelles, and landing legs
	arm_len = g["arm_len"]
	r_arm_mid = g["r_arm_mid"]
	wing_chord = wing["chord"]
	wing_thick = wing["thickness"]
	nacelle_r = wing.get("nacelle_radius", 0.024)
	nacelle_l = wing.get("nacelle_length", 0.12)
	z_ground = g["z_ground_rel_cg"]

	col = f"""
      <collision name="fuselage_collision">
        <pose>0 0 {fus["length"] / 2 - cg:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>{r_f:.4f}</radius><length>{fus["length"]:.4f}</length></cylinder></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>"""

	for i, (p, angle) in enumerate(zip(g["motor_pos"], g["motor_angles"])):
		# Wing arm blade
		wx = r_arm_mid * math.cos(angle)
		wy = r_arm_mid * math.sin(angle)
		vis += f"""
      <visual name="wing_arm_{i}_visual">
        <pose>{wx:.4f} {wy:.4f} {g['z_wing']:.4f} 0 0 {angle:.4f}</pose>
        <geometry><box><size>{arm_len:.4f} {wing_thick:.4f} {wing_chord:.4f}</size></box></geometry>
        <material><ambient>{body_grey}</ambient><diffuse>{body_grey}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>"""

		z_wing_top = g["z_wing"] + wing_chord / 2
		z_wing_bot = g["z_wing"] - wing_chord / 2
		z_mot_base = z_wing_top  # motor base sits directly at the top edge of the wing
		motor_h = 0.028
		z_motor_mid = z_mot_base + motor_h / 2
		z_rotor = z_mot_base + motor_h + 0.008

		# Streamlined motor nacelle pod (matches wing chord from top edge to bottom edge)
		nacelle_l = wing_chord
		z_nacelle = g["z_wing"]
		vis += cylinder_visual(f"nacelle_{i}_visual", f"{p[0]:.4f} {p[1]:.4f} {z_nacelle:.4f} 0 0 0", nacelle_r, nacelle_l, body_grey)

		# Motor bell (BrotherHobby motor mounted on top of nacelle / upper edge of wing)
		vis += cylinder_visual(f"motor_{i}_visual", f"{p[0]:.4f} {p[1]:.4f} {z_motor_mid:.4f} 0 0 0", 0.0175, motor_h, carbon_black)

		# Navigation LED (Starboard = Green on right/motors 0,3; Port = Red on left/motors 1,2)
		led_col = led_green if p[1] <= 0 else led_red
		z_led = z_mot_base - 0.015
		vis += f"""
      <visual name="nav_led_{i}_visual">
        <pose>{p[0]:.4f} {p[1]:.4f} {z_led:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.007</radius></sphere></geometry>
        <material><ambient>{led_col}</ambient><diffuse>{led_col}</diffuse><emissive>{led_col}</emissive></material>
      </visual>"""

		# Carbon landing rod extending rearward from bottom of nacelle past the tail end
		z_leg_top = z_wing_bot
		leg_len = z_leg_top - z_ground
		z_leg_mid = (z_leg_top + z_ground) / 2
		vis += cylinder_visual(f"landing_leg_{i}_visual", f"{p[0]:.4f} {p[1]:.4f} {z_leg_mid:.4f} 0 0 0", 0.0035, leg_len, carbon_black)
		vis += f"""
      <visual name="leg_foot_{i}_visual">
        <pose>{p[0]:.4f} {p[1]:.4f} {z_ground:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.007</radius></sphere></geometry>
        <material><ambient>{carbon_black}</ambient><diffuse>{carbon_black}</diffuse></material>
      </visual>"""

		# Collision: 4 foot tip contact spheres for ground standing stability
		col += f"""
      <collision name="leg_foot_{i}_collision">
        <pose>{p[0]:.4f} {p[1]:.4f} {z_ground:.4f} 0 0 0</pose>
        <geometry><sphere><radius>0.009</radius></sphere></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>"""

	# Nose camera sensor
	camera = ""
	if c.get("camera", {}).get("enabled"):
		cam = c["camera"]
		cam_pose = "0 0 0 0 -1.5708 0" if gimbal_enabled else f"0 0 {fus['length'] - cg:.4f} 0 -1.5708 0"
		camera = f"""
      <sensor name="nose_camera" type="camera">
        <pose>{cam_pose}</pose>
        <always_on>1</always_on>
        <update_rate>{cam["rate"]}</update_rate>
        <camera>
          <horizontal_fov>{math.radians(cam["hfov_deg"]):.4f}</horizontal_fov>
          <image><width>{cam["width"]}</width><height>{cam["height"]}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>3000</far></clip>
        </camera>
      </sensor>"""

	# Rotors and Multicopter Motor Model plugins
	rotors = ""
	plugins = ""
	s = g["prop_scale"]
	prop_off = PROP_MESH_OFFSET * s
	z_rotor = g["z_wing"] + wing_chord / 2 + 0.028 + 0.008

	for i, (p, d) in enumerate(zip(g["motor_pos"], g["directions"])):
		rotors += f"""
    <link name="rotor_{i}">
      <pose>{p[0]:.4f} {p[1]:.4f} {z_rotor:.4f} 0 0 0</pose>
      <inertial>
        <mass>0.012</mass>
        <inertia><ixx>2e-07</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz></inertia>
      </inertial>
      <visual name="rotor_{i}_visual">
        <pose>{fmt(prop_off)} 0 0 0</pose>
        <geometry><mesh><scale>{s:.4f} {s:.4f} {s:.4f}</scale><uri>model://x500_base/meshes/1345_prop_{d}.stl</uri></mesh></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
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

	# Aerodynamic LiftDrag for the X-wing arms:
	inv_sqrt2 = 1.0 / math.sqrt(2)

	gimbal_sdf = ""
	gimbal_plugins = ""
	if gimbal_enabled:
		m_g = g.get("m_gimbal", 0.12) / 2
		yaw_lim = math.radians(gimbal.get("yaw_limit_deg", 45))
		pitch_lim = math.radians(gimbal.get("pitch_limit_deg", 45))
		p_gain = gimbal.get("p_gain", 5.0)
		i_gain = gimbal.get("i_gain", 0.1)
		d_gain = gimbal.get("d_gain", 0.05)

		gimbal_sdf = f"""
    <!-- 2-Axis Gimbal: Yaw Joint & Link (+/- {gimbal.get("yaw_limit_deg", 45)} deg around fuselage z) -->
    <link name="gimbal_yaw_link">
      <pose>0 0 {cam_lens_z - 0.010:.4f} 0 0 0</pose>
      <inertial>
        <mass>{m_g:.4f}</mass>
        <inertia>
          <ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz>
        </inertia>
      </inertial>
      <visual name="gimbal_yaw_yoke_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry><cylinder><radius>0.022</radius><length>0.016</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.22 1</ambient><diffuse>0.2 0.2 0.22 1</diffuse></material>
      </visual>
    </link>

    <joint name="gimbal_yaw_joint" type="revolute">
      <parent>base_link</parent>
      <child>gimbal_yaw_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>{-yaw_lim:.4f}</lower>
          <upper>{yaw_lim:.4f}</upper>
          <effort>5</effort>
          <velocity>10</velocity>
        </limit>
        <dynamics><damping>0.03</damping><friction>0.005</friction></dynamics>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
        </ode>
      </physics>
    </joint>

    <!-- 2-Axis Gimbal: Pitch Joint & Link (+/- {gimbal.get("pitch_limit_deg", 45)} deg around cross y) with Camera -->
    <link name="gimbal_pitch_link">
      <pose>0 0 {cam_lens_z:.4f} 0 0 0</pose>
      <inertial>
        <mass>{m_g:.4f}</mass>
        <inertia>
          <ixx>0.0005</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0005</iyy><iyz>0</iyz><izz>0.0005</izz>
        </inertia>
      </inertial>
      <visual name="seeker_turret_visual">
        <pose>0 0 -0.010 0 0 0</pose>
        <geometry><sphere><radius>0.020</radius></sphere></geometry>
        <material><ambient>{accent_dark}</ambient><diffuse>{accent_dark}</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>
      <visual name="camera_lens_visual">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><cylinder><radius>0.012</radius><length>0.010</length></cylinder></geometry>
        <material><ambient>0.02 0.02 0.05 1</ambient><diffuse>0.02 0.02 0.05 1</diffuse><specular>0.95 0.95 0.95 1</specular></material>
      </visual>{camera}
    </link>

    <joint name="gimbal_pitch_joint" type="revolute">
      <parent>gimbal_yaw_link</parent>
      <child>gimbal_pitch_link</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>{-pitch_lim:.4f}</lower>
          <upper>{pitch_lim:.4f}</upper>
          <effort>5</effort>
          <velocity>10</velocity>
        </limit>
        <dynamics><damping>0.03</damping><friction>0.005</friction></dynamics>
      </axis>
      <physics>
        <ode>
          <implicit_spring_damper>1</implicit_spring_damper>
        </ode>
      </physics>
    </joint>"""

		gimbal_plugins = f"""
    <!-- 2-Axis Gimbal Joint Position Controllers -->
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>gimbal_yaw_joint</joint_name>
      <sub_topic>command/seeker_yaw</sub_topic>
      <p_gain>{p_gain}</p_gain>
      <i_gain>{i_gain}</i_gain>
      <d_gain>{d_gain}</d_gain>
      <i_max>1.0</i_max>
      <i_min>-1.0</i_min>
      <cmd_max>2.0</cmd_max>
      <cmd_min>-2.0</cmd_min>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>gimbal_pitch_joint</joint_name>
      <sub_topic>command/seeker_pitch</sub_topic>
      <p_gain>{p_gain}</p_gain>
      <i_gain>{i_gain}</i_gain>
      <d_gain>{d_gain}</d_gain>
      <i_max>1.0</i_max>
      <i_min>-1.0</i_min>
      <cmd_max>2.0</cmd_max>
      <cmd_min>-2.0</cmd_min>
    </plugin>"""

	return f"""<?xml version="1.0"?>
<!-- Generated by Tools/simulation/interceptor_sim/tools/build_interceptor.py from interceptor.yaml, do not edit by hand -->
<sdf version="1.9">
  <model name="{c["name"]}">
    <pose>0 0 {g["spawn_z"]:.3f} 0 0 0</pose>
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
      </sensor>{"" if gimbal_enabled else camera}
    </link>
{rotors}
{gimbal_sdf}
{plugins}
{gimbal_plugins}

    <!-- Wing arms LiftDrag Pair A (45 deg / -135 deg diagonal) -->
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>base_link</link_name>
      <air_density>{RHO}</air_density>
      <area>{g["wing_pair_area"]:.4f}</area>
      <a0>0</a0>
      <cla>{g["wing_cla"]:.3f}</cla>
      <cda>0.65</cda>
      <cma>0</cma>
      <alpha_stall>0.6</alpha_stall>
      <cla_stall>-1.5</cla_stall>
      <cda_stall>0.9</cda_stall>
      <cp>0 0 {g["z_wing"]:.4f}</cp>
      <forward>0 0 1</forward>
      <upward>{-inv_sqrt2:.4f} {inv_sqrt2:.4f} 0</upward>
    </plugin>

    <!-- Wing arms LiftDrag Pair B (-45 deg / +135 deg diagonal) -->
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <link_name>base_link</link_name>
      <air_density>{RHO}</air_density>
      <area>{g["wing_pair_area"]:.4f}</area>
      <a0>0</a0>
      <cla>{g["wing_cla"]:.3f}</cla>
      <cda>0.65</cda>
      <cma>0</cma>
      <alpha_stall>0.6</alpha_stall>
      <cla_stall>-1.5</cla_stall>
      <cda_stall>0.9</cda_stall>
      <cp>0 0 {g["z_wing"]:.4f}</cp>
      <forward>0 0 1</forward>
      <upward>{-inv_sqrt2:.4f} {-inv_sqrt2:.4f} 0</upward>
    </plugin>

    <!-- Body & fin aerodynamics: quadratic drag per body axis, angular damping for tail stability -->
    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
      <link_name>base_link</link_name>
      <xUabsU>{-g["c_side"]:.5f}</xUabsU>
      <yVabsV>{-g["c_side"]:.5f}</yVabsV>
      <zWabsW>{-g["c_axial"]:.5f}</zWabsW>
      <kPabsP>-0.0020</kPabsP>
      <mQabsQ>-0.0050</mQabsQ>
      <nRabsR>-0.0050</nRabsR>
      <disable_added_mass>true</disable_added_mass>
      <disable_coriolis>true</disable_coriolis>
    </plugin>
  </model>
</sdf>
"""


def post(detect_port):
	"""Startup script sourced at the end of PX4 boot."""
	return f"""#
# Generated by Tools/simulation/interceptor_sim/tools/build_interceptor.py from
# interceptor.yaml, do not edit by hand.
#
# The detector (Tools/simulation/interceptor_sim/tools/sim_detector.py) sends its detections
# to this port; on the vehicle the Jetson is on a serial port instead (TGV_CFG).
target_vision start -u {detect_port}
intercept start

# HUD link of tools/view_hud.py (the camera window), next to QGroundControl's 14550
mavlink start -u $((18590+px4_instance)) -o $((14551+px4_instance)) -r 400000 -m custom

# Web GCS telemetry link (dedicated onboard stream)
mavlink start -u $((18585+px4_instance)) -o $((14545+px4_instance)) -r 400000 -m onboard
"""


def airframe(g, thr_hover, w_min):
	c = g["c"]
	lines = []

	# PX4 body frame is FRD: x forward, y right, z down; KM > 0 for ccw
	# In Gazebo FLU: x forward, y left, z up
	# For Quadrotor X:
	# Motor 0 (Front-Right): Gazebo [+d_xy, -d_xy, z_mot] -> PX4 FRD [+d_xy, +d_xy, -z_mot], CCW (KM > 0)
	# Motor 1 (Back-Left):   Gazebo [-d_xy, +d_xy, z_mot] -> PX4 FRD [-d_xy, -d_xy, -z_mot], CCW (KM > 0)
	# Motor 2 (Front-Left):  Gazebo [+d_xy, +d_xy, z_mot] -> PX4 FRD [+d_xy, -d_xy, -z_mot], CW  (KM < 0)
	# Motor 3 (Back-Right):  Gazebo [-d_xy, -d_xy, z_mot] -> PX4 FRD [-d_xy, +d_xy, -z_mot], CW  (KM < 0)
	for i, (p, d) in enumerate(zip(g["motor_pos"], g["directions"])):
		lines += [
			f"param set-default CA_ROTOR{i}_PX {p[0]:.3f}",
			f"param set-default CA_ROTOR{i}_PY {-p[1] + 0.0:.3f}",
			f"param set-default CA_ROTOR{i}_PZ {-p[2] + 0.0:.3f}",
			f"param set-default CA_ROTOR{i}_KM {0.05 if d == 'ccw' else -0.05}"
		]

	for i in range(1, 5):
		lines += [
			f"param set-default SIM_GZ_EC_FUNC{i} {100 + i}",
			f"param set-default SIM_GZ_EC_MIN{i} {w_min:.0f}",
			f"param set-default SIM_GZ_EC_MAX{i} {g['w_max']:.0f}"
		]

	cam = ""
	if c.get("camera", {}).get("enabled"):
		cam = "\n# nose camera on the thrust axis\nparam set-default TGV_MNT_PITCH 90\n"

	gimbal = c.get("gimbal", {})
	if gimbal.get("enabled", False):
		cam += f"""
# 2-axis nose seeker gimbal (+/- {gimbal.get("yaw_limit_deg", 45)} deg, autonomous seeker control)
param set-default MNT_MODE_OUT 0
param set-default MNT_RANGE_YAW {gimbal.get("yaw_limit_deg", 45):.1f}
param set-default MNT_RANGE_PITCH {gimbal.get("pitch_limit_deg", 45):.1f}
"""

	body = "\n".join(lines)
	return f"""#!/bin/sh
#
# @name Octopus interceptor quad (Gazebo)
#
# @type Quadrotor x
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

param set-default THR_MDL_FAC 0.0
param set-default MPC_THR_HOVER {thr_hover:.2f}
{cam}
# fast and aggressive: strong tilt, high speeds / accelerations
param set-default MPC_TILTMAX_AIR 78
param set-default MPC_XY_VEL_MAX 45
param set-default MPC_XY_CRUISE 38
param set-default MPC_ACC_HOR_MAX 5.5
param set-default MPC_ACC_HOR 3.5
param set-default MPC_Z_VEL_MAX_UP 12
param set-default MPC_Z_VEL_MAX_DN 4.0

# manual flight from the ground station: Altitude / Stabilized modes are limited by tilt only
param set-default MPC_MAN_TILT_MAX 78
param set-default MPC_VEL_MANUAL 35

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

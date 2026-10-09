#!/usr/bin/env python3
"""
HUD Camera Viewer: interceptor nose camera with a classic monochrome HUD.

Subscribes directly to the Gazebo camera image topic and dynamic poses, and reads the
flight data from PX4 over MAVLink (own link, udp 14551, opened by the airframe .post):
  - Heading tape, pitch ladder and bank scale (degrees)
  - Ground speed, altitude above home, vertical speed
  - Roll, pitch and tilt (thrust axis from vertical) in degrees
  - Flight mode, armed state, battery, GPS
  - Target bounding box (from Gazebo ground truth)
  - Center boresight reticle and LOS line
Values that PX4 has not sent for a while are shown as ---.

Keys: h toggles the flight data, q / Esc quits.

Usage:
  python3 tools/view_hud.py [--world ankara] [--max-range 50] [--mavlink udpin:0.0.0.0:14551]
"""

import argparse
from collections import deque
import math
import os
import re
import struct
import sys
import threading
import time

os.environ["MAVLINK20"] = "1"
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import cv2
import numpy as np

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.image_pb2 import Image as ImageMsg
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

# AVAILABLE_MODES is only in the development dialect of pymavlink (same CRC as PX4)
mavutil.set_dialect("development")
mav = mavutil.mavlink

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_DIR = os.path.join(SIM_DIR, "models", "talon1718")

# classic HUD: one colour (BGR), a thin dark outline keeps it readable on a bright sky
HUD = (60, 255, 60)
HUD_DIM = (30, 140, 30)
SHADOW = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX

# gz camera frame (+x forward, +y left, +z up) -> camera optical frame (+x right, +y down, +z forward)
GZ_TO_OPT = np.array([
    [ 0., -1.,  0.],
    [ 0.,  0., -1.],
    [ 1.,  0.,  0.]
])

# 3D bounding box corners of Talon 1718 in model frame
# Wingspan: ~1.72 m (Y: -0.86 to +0.86)
# Fuselage: ~1.10 m (X: -0.65 to +0.45)
# Height:   ~0.35 m (Z: -0.10 to +0.25)
TALON_BBOX_CORNERS = np.array([
    [ 0.45,  0.86,  0.25],
    [ 0.45,  0.86, -0.10],
    [ 0.45, -0.86,  0.25],
    [ 0.45, -0.86, -0.10],
    [-0.65,  0.86,  0.25],
    [-0.65,  0.86, -0.10],
    [-0.65, -0.86,  0.25],
    [-0.65, -0.86, -0.10],
])

# PX4 custom mode (main_mode, sub_mode) -> name, see src/modules/commander/px4_custom_mode.h
PX4_MODE_NAMES = {
    (1, 0): "MANUAL", (2, 0): "ALTITUDE", (3, 0): "POSITION", (3, 1): "ORBIT", (3, 2): "POSITION SLOW",
    (4, 1): "READY", (4, 2): "TAKEOFF", (4, 3): "HOLD", (4, 4): "MISSION", (4, 5): "RETURN",
    (4, 6): "LAND", (4, 8): "FOLLOW ME", (4, 9): "PRECISION LAND", (4, 10): "VTOL TAKEOFF",
    (5, 0): "ACRO", (6, 0): "OFFBOARD", (7, 0): "STABILIZED", (10, 0): "TERMINATION",
    (11, 0): "ALTITUDE CRUISE",
}
PX4_MODE_NAMES.update({(4, 11 + i): f"EXTERNAL {i + 1}" for i in range(8)})

GPS_FIX = {0: "NO GPS", 1: "NO FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK FLT", 6: "RTK FIX"}


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
    ])


def read_stl_bounds(path):
    try:
        with open(path, "rb") as f:
            f.seek(80)
            n = struct.unpack("<I", f.read(4))[0]
            rec = np.frombuffer(f.read(n * 50),
                                dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
        return rec["v"].reshape(-1, 3).astype(np.float64)
    except Exception:
        return np.zeros((0, 3), dtype=np.float64)


def talon_vertices():
    """Load model vertices from STL meshes or fallback to bounding box corners.

    Only the mesh vertices: the corners of the 3D box lie outside the silhouette and
    would make the image box about twice as large (same as tools/sim_detector.py)."""
    sdf_path = os.path.join(MODEL_DIR, "model.sdf")
    pts = []
    if os.path.exists(sdf_path):
        try:
            sdf = open(sdf_path).read()
            link_pose = {m.group(1): np.array([float(v) for v in m.group(2).split()[:3]])
                         for m in re.finditer(r'<link name="(\w+)">\s*<pose>([^<]+)</pose>', sdf)}
            for link in re.finditer(r'<link name="(\w+)">(.*?)</link>', sdf, re.S):
                offset = link_pose.get(link.group(1), np.zeros(3))
                for uri in re.findall(r"<uri>model://talon1718/meshes/([^<]+)</uri>", link.group(2)):
                    mesh_path = os.path.join(MODEL_DIR, "meshes", uri)
                    if os.path.exists(mesh_path):
                        v = np.unique(np.round(read_stl_bounds(mesh_path), 3), axis=0)
                        if len(v) > 0:
                            pts.append(v + offset)
        except Exception:
            pass
    return np.concatenate(pts) if pts else TALON_BBOX_CORNERS


# ---------------------------------------------------------------------------------------
# MAVLink flight data


class Telemetry(threading.Thread):
    """Reads the PX4 HUD link in the background and keeps the newest message of each type
    with its arrival time, so that stale values are not shown."""

    # message id -> rate [Hz], requested with MAV_CMD_SET_MESSAGE_INTERVAL: the PX4
    # instance runs in custom mode and streams only HEARTBEAT on its own
    STREAMS = {
        mav.MAVLINK_MSG_ID_ATTITUDE_QUATERNION: 50.0,
        mav.MAVLINK_MSG_ID_VFR_HUD: 10.0,
        mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10.0,
        mav.MAVLINK_MSG_ID_SYS_STATUS: 2.0,
        mav.MAVLINK_MSG_ID_GPS_RAW_INT: 2.0,
        mav.MAVLINK_MSG_ID_AVAILABLE_MODES: 1.0,    # sends AVAILABLE_MODES_MONITOR after a change
    }

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.lock = threading.Lock()
        self.msgs = {}          # type -> (monotonic time, message)
        self.mode_names = {}    # custom_mode -> name, from AVAILABLE_MODES (external modes)
        self.tsys, self.tcomp = None, None
        self.monitor_seq = None

    def get(self, msg_type, max_age):
        with self.lock:
            entry = self.msgs.get(msg_type)

        if entry is None or time.monotonic() - entry[0] > max_age:
            return None

        return entry[1]

    def mode_name(self, custom_mode):
        name = self.mode_names.get(custom_mode)

        if name:
            return name.upper()

        return PX4_MODE_NAMES.get(((custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF), "MODE ?")

    def run(self):
        conn = mavutil.mavlink_connection(self.url, source_system=246, source_component=mav.MAV_COMP_ID_USER2,
                                          dialect="development")
        last_request = 0.0

        while True:
            m = conn.recv_match(blocking=True, timeout=0.5)
            now = time.monotonic()

            if m is not None and m.get_type() != "BAD_DATA":
                self.handle(conn, m, now)

            # (re)request the streams on the first heartbeat and after a PX4 restart
            if self.tsys is not None and now - last_request > 2.0 and self.get("ATTITUDE_QUATERNION", 1.0) is None:
                for msg_id, rate in self.STREAMS.items():
                    conn.mav.command_long_send(self.tsys, self.tcomp, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                               msg_id, 1e6 / rate, 0, 0, 0, 0, 0)

                self.request_modes(conn)
                last_request = now

    def request_modes(self, conn):
        conn.mav.command_long_send(self.tsys, self.tcomp, mav.MAV_CMD_REQUEST_MESSAGE, 0,
                                   mav.MAVLINK_MSG_ID_AVAILABLE_MODES, 0, 0, 0, 0, 0, 0)

    def handle(self, conn, m, now):
        t = m.get_type()

        if t == "HEARTBEAT":
            if m.type == mav.MAV_TYPE_GCS or m.autopilot == mav.MAV_AUTOPILOT_INVALID:
                return

            if self.tsys is None:
                self.tsys, self.tcomp = m.get_srcSystem(), m.get_srcComponent()

        if (m.get_srcSystem(), m.get_srcComponent()) != (self.tsys, self.tcomp):
            return

        if t == "AVAILABLE_MODES":
            name = m.mode_name.rstrip("\x00") if isinstance(m.mode_name, str) else ""

            if name and not name.startswith("("):
                self.mode_names[m.custom_mode] = name

        elif t == "AVAILABLE_MODES_MONITOR":
            if self.monitor_seq is not None and m.seq != self.monitor_seq:
                self.request_modes(conn)

            self.monitor_seq = m.seq

        with self.lock:
            self.msgs[t] = (now, m)


# ---------------------------------------------------------------------------------------
# drawing


def hud_line(img, p1, p2, th=1, color=HUD):
    p1 = (int(round(p1[0])), int(round(p1[1])))
    p2 = (int(round(p2[0])), int(round(p2[1])))
    cv2.line(img, p1, p2, SHADOW, th + 2, cv2.LINE_AA)
    cv2.line(img, p1, p2, color, th, cv2.LINE_AA)


def hud_text(img, s, org, scale, align="left", color=HUD):
    (w, h), _ = cv2.getTextSize(s, FONT, scale, 1)
    x, y = int(org[0]), int(org[1])

    if align == "center":
        x -= w // 2

    elif align == "right":
        x -= w

    cv2.putText(img, s, (x, y), FONT, scale, SHADOW, 3, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), FONT, scale, color, 1, cv2.LINE_AA)
    return w, h


def hud_box(img, centre, size, text, scale):
    cx, cy = centre
    w, h = size
    pts = [(cx - w / 2, cy - h / 2), (cx + w / 2, cy - h / 2), (cx + w / 2, cy + h / 2), (cx - w / 2, cy + h / 2)]

    for i in range(4):
        hud_line(img, pts[i], pts[(i + 1) % 4])

    _, th = cv2.getTextSize(text, FONT, scale, 1)[0]
    hud_text(img, text, (cx, cy + th / 2), scale, "center")


def fmt(value, spec, valid=True):
    return format(value, spec) if valid and value is not None and math.isfinite(value) else "---"


def draw_heading_tape(img, heading, w, s):
    cx, y = w / 2, 48 * s
    half = 0.28 * w
    ppd = half / 35.0       # pixels per degree, +-35 deg visible
    hud_line(img, (cx - half, y), (cx + half, y))

    if heading is None:
        hud_text(img, "---", (cx, y - 14 * s), 0.6 * s, "center")
        return

    start = int(math.floor((heading - 35) / 5.0)) * 5

    for d in range(start, start + 75, 5):
        x = cx + (d - heading) * ppd

        if abs(x - cx) > half:
            continue

        major = d % 10 == 0
        hud_line(img, (x, y), (x, y + (12 if major else 6) * s))

        if major and abs(x - cx) > 26 * s:
            hud_text(img, f"{d % 360:03d}", (x, y + 28 * s), 0.42 * s, "center")

    # caret and boxed current heading
    hud_line(img, (cx - 6 * s, y - 8 * s), (cx, y))
    hud_line(img, (cx + 6 * s, y - 8 * s), (cx, y))
    hud_box(img, (cx, y - 22 * s), (54 * s, 22 * s), f"{int(round(heading)) % 360:03d}", 0.55 * s)


def rotate(cx, cy, dx, dy, roll):
    """Screen point of (dx, dy) around the boresight, the horizon turns against the roll."""
    c, sn = math.cos(roll), math.sin(roll)
    return cx + dx * c + dy * sn, cy - dx * sn + dy * c


def draw_pitch_ladder(img, roll_deg, pitch_deg, w, h, s):
    cx, cy = w / 2, h / 2
    roll = math.radians(roll_deg)
    ppd = 7.0 * s           # pixels per degree of pitch
    gap, half = 45 * s, 120 * s

    for p in range(-90, 91, 10):
        if abs(p - pitch_deg) > 22:
            continue

        dy = (pitch_deg - p) * ppd

        if p == 0:
            # horizon: long and solid
            for side in (-1, 1):
                hud_line(img, rotate(cx, cy, side * gap, dy, roll), rotate(cx, cy, side * (half + 60 * s), dy, roll))
            continue

        tick = 8 * s if p < 0 else -8 * s    # end ticks point to the horizon

        for side in (-1, 1):
            if p > 0:
                hud_line(img, rotate(cx, cy, side * gap, dy, roll), rotate(cx, cy, side * half, dy, roll))

            else:
                # negative pitch: dashed
                n = 5
                for i in range(n):
                    a = gap + (half - gap) * i / n
                    b = gap + (half - gap) * (i + 0.6) / n
                    hud_line(img, rotate(cx, cy, side * a, dy, roll), rotate(cx, cy, side * b, dy, roll))

            hud_line(img, rotate(cx, cy, side * half, dy, roll), rotate(cx, cy, side * half, dy - tick, roll))
            lx, ly = rotate(cx, cy, side * (half + 22 * s), dy, roll)
            hud_text(img, f"{abs(p)}", (lx, ly + 5 * s), 0.42 * s, "center")


def draw_bank_scale(img, roll_deg, w, h, s):
    cx, cy = w / 2, h / 2
    r = 0.36 * h

    for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
        rad = math.radians(a)
        length = (16 if a in (0, -30, 30, -60, 60) else 9) * s
        p1 = (cx + r * math.sin(rad), cy + r * math.cos(rad))
        p2 = (cx + (r + length) * math.sin(rad), cy + (r + length) * math.cos(rad))
        hud_line(img, p1, p2)

    if roll_deg is None:
        return

    # pointer fixed to the horizon, moves along the scale
    rad = math.radians(max(-70.0, min(70.0, roll_deg)))
    tip = (cx + (r - 2 * s) * math.sin(rad), cy + (r - 2 * s) * math.cos(rad))
    base = r - 16 * s
    left = (cx + base * math.sin(rad) - 7 * s * math.cos(rad), cy + base * math.cos(rad) + 7 * s * math.sin(rad))
    right = (cx + base * math.sin(rad) + 7 * s * math.cos(rad), cy + base * math.cos(rad) - 7 * s * math.sin(rad))
    hud_line(img, tip, left)
    hud_line(img, left, right)
    hud_line(img, right, tip)


def draw_flight_data(img, tel, sim_t, w, h, gmb_data=None, fps=None):
    s = h / 720.0
    hb = tel.get("HEARTBEAT", 3.0)
    att = tel.get("ATTITUDE_QUATERNION", 0.5)
    vfr = tel.get("VFR_HUD", 1.0)
    gpi = tel.get("GLOBAL_POSITION_INT", 1.0)
    sys_status = tel.get("SYS_STATUS", 3.0)
    gps = tel.get("GPS_RAW_INT", 3.0)

    roll = pitch = yaw = tilt = None

    if att is not None:
        qw, qx, qy, qz = att.q1, att.q2, att.q3, att.q4
        roll = math.degrees(math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))))
        yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))) % 360
        # thrust axis (body z) from the vertical
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy)))))

    draw_heading_tape(img, yaw, w, s)
    draw_bank_scale(img, roll, w, h, s)

    if roll is not None:
        draw_pitch_ladder(img, roll, pitch, w, h, s)

    # speed (left) and altitude (right) boxes
    xl, xr, yc = w / 2 - 0.34 * w, w / 2 + 0.34 * w, h / 2
    hud_text(img, "GS", (xl, yc - 22 * s), 0.45 * s, "center")
    hud_box(img, (xl, yc), (84 * s, 28 * s), fmt(vfr.groundspeed if vfr else None, ".1f"), 0.65 * s)
    hud_text(img, "m/s", (xl, yc + 34 * s), 0.4 * s, "center")
    hud_text(img, "ALT", (xr, yc - 22 * s), 0.45 * s, "center")
    hud_box(img, (xr, yc), (84 * s, 28 * s), fmt(gpi.relative_alt / 1000.0 if gpi else None, ".1f"), 0.65 * s)
    hud_text(img, "VS " + fmt(-gpi.vz / 100.0 if gpi else None, "+.1f"), (xr, yc + 34 * s), 0.45 * s, "center")

    # attitude in degrees (bottom left)
    x0, y0, dy = 24 * s, h - 78 * s, 24 * s
    hud_text(img, "ROLL  " + fmt(roll, "+6.1f"), (x0, y0), 0.55 * s)
    hud_text(img, "PITCH " + fmt(pitch, "+6.1f"), (x0, y0 + dy), 0.55 * s)
    hud_text(img, "TILT  " + fmt(tilt, "6.1f"), (x0, y0 + 2 * dy), 0.55 * s)

    # vehicle state (bottom right)
    x1 = w - 24 * s

    if hb is None:
        hud_text(img, "NO MAVLINK", (w / 2, h * 0.30), 0.8 * s, "center")
        mode, armed = "---", "---"

    else:
        mode = tel.mode_name(hb.custom_mode)
        armed = "ARMED" if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"

    bat = "BAT ---"

    if sys_status is not None and sys_status.voltage_battery != 65535:
        bat = f"BAT {sys_status.voltage_battery / 1000.0:.1f}V"

        if sys_status.battery_remaining >= 0:
            bat += f" {sys_status.battery_remaining}%"

    gps_s = f"GPS {GPS_FIX.get(gps.fix_type, '?')} {gps.satellites_visible}" if gps else "GPS ---"
    hud_text(img, mode, (x1, y0), 0.55 * s, "right")
    hud_text(img, armed, (x1, y0 + dy), 0.55 * s, "right")
    hud_text(img, f"{bat}   {gps_s}", (x1, y0 + 2 * dy), 0.5 * s, "right")

    # sim time & video fps (top left)
    hud_text(img, f"T {sim_t:7.1f}", (24 * s, 30 * s), 0.5 * s)
    fps_str = f"FPS {fps:5.1f}" if fps is not None and fps > 0.1 else "FPS   ---"
    hud_text(img, fps_str, (24 * s, 54 * s), 0.5 * s)

    # gimbal angles relative to forward boresight (top right, 2 lines like bottom left)
    gmb_yaw = gmb_pitch = None
    if gmb_data is not None:
        gmb_yaw = gmb_data.get("yaw")
        gmb_pitch = gmb_data.get("pitch")

    x_tr = w - 24 * s
    y_tr = 30 * s
    dy_tr = 24 * s
    hud_text(img, "GMB YAW   " + fmt(gmb_yaw, "+6.1f"), (x_tr, y_tr), 0.55 * s, "right")
    hud_text(img, "GMB PITCH " + fmt(gmb_pitch, "+6.1f"), (x_tr, y_tr + dy_tr), 0.55 * s, "right")


def draw_hud(img, tracking_data, width, height):
    """Boresight reticle, nose indicator and target bounding box."""
    cx, cy = width // 2, height // 2

    # Draw center boresight reticle
    hud_line(img, (cx - 25, cy), (cx - 8, cy))
    hud_line(img, (cx + 8, cy), (cx + 25, cy))
    hud_line(img, (cx, cy - 25), (cx, cy - 8))
    hud_line(img, (cx, cy + 8), (cx, cy + 25))
    cv2.circle(img, (cx, cy), 15, SHADOW, 3, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 15, HUD, 1, cv2.LINE_AA)

    # Fuselage nose indicator (aircraft waterline symbol where the drone body points)
    if tracking_data and "boresight" in tracking_data:
        ub, vb = tracking_data["boresight"]
        if 0 <= ub < width and 0 <= vb < height:
            hud_line(img, (ub - 18, vb), (ub - 6, vb))
            hud_line(img, (ub + 6, vb), (ub + 18, vb))
            hud_line(img, (ub - 6, vb), (ub, vb - 6))
            hud_line(img, (ub, vb - 6), (ub + 6, vb))

    if not tracking_data or not tracking_data.get("in_front"):
        return

    range_m = tracking_data.get("range_m")
    if range_m is None or range_m > 50.0:
        return

    u_c = int(tracking_data["u_centre"])
    v_c = int(tracking_data["v_centre"])
    in_fov = tracking_data["in_fov"]

    if in_fov:
        # Faint line-of-sight line from boresight to target
        cv2.line(img, (cx, cy), (u_c, v_c), HUD_DIM, 1, cv2.LINE_AA)

        # Bounding Box around airplane
        bbox = tracking_data.get("bbox")
        if bbox:
            u_min, v_min, u_max, v_max = bbox
            w_box = u_max - u_min
            h_box = v_max - v_min

            # Main bounding box with shadow for contrast
            cv2.rectangle(img, (u_min, v_min), (u_max, v_max), SHADOW, 2, cv2.LINE_AA)
            cv2.rectangle(img, (u_min, v_min), (u_max, v_max), HUD, 1, cv2.LINE_AA)

            # Subtle corner brackets
            corner_len = max(4, min(12, min(w_box, h_box) // 3))
            for p1, p2 in [
                ((u_min, v_min), (u_min + corner_len, v_min)),
                ((u_min, v_min), (u_min, v_min + corner_len)),
                ((u_max, v_min), (u_max - corner_len, v_min)),
                ((u_max, v_min), (u_max, v_min + corner_len)),
                ((u_min, v_max), (u_min + corner_len, v_max)),
                ((u_min, v_max), (u_min, v_max - corner_len)),
                ((u_max, v_max), (u_max - corner_len, v_max)),
                ((u_max, v_max), (u_max, v_max - corner_len)),
            ]:
                cv2.line(img, p1, p2, SHADOW, 3, cv2.LINE_AA)
                cv2.line(img, p1, p2, HUD, 1, cv2.LINE_AA)

            # Target distance under bbox (centered horizontally on the box)
            u_box_c = (u_min + u_max) // 2
            hud_text(img, f"{range_m:.1f}m", (u_box_c, v_max + 14), 0.45, "center", color=HUD)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="ankara")
    ap.add_argument("--interceptor", default="interceptor_0")
    ap.add_argument("--target", default="talon1718_1")
    ap.add_argument("--max-range", type=float, default=50.0)
    ap.add_argument("--mavlink", default="udpin:0.0.0.0:14551", help="PX4 HUD link (see the airframe .post)")
    ap.add_argument("extra_args", nargs="*", help="Optional extra arguments (e.g. port)")
    a = ap.parse_args()

    import yaml
    with open(os.path.join(SIM_DIR, "interceptor.yaml")) as f:
        cfg = yaml.safe_load(f)
    cam = cfg.get("camera", {})
    width = cam.get("width", 1280)
    height = cam.get("height", 720)
    hfov_deg = cam.get("hfov_deg", 60.0)
    hfov_rad = math.radians(hfov_deg)
    fx = width / 2 / math.tan(hfov_rad / 2)
    fy = fx
    cx, cy = width / 2, height / 2

    # Camera mount offset
    fus_len = cfg["fuselage"]["length"]
    components = cfg.get("components", {})
    total_mass = cfg["fuselage"]["mass"]
    cg_num = 0.0
    for name, comp in components.items():
        total_mass += comp["mass"]
        cg_num += comp["mass"] * comp["z"]
    cg_num += cfg["fuselage"]["mass"] * fus_len / 2
    cg = cg_num / total_mass
    cam_offset_body = np.array([0.0, 0.0, fus_len - cg])

    # Rotation: Body FLU to Gazebo camera frame
    # Camera sensor looks along body +z (thrust/nose direction)
    R_cam_gz_body = np.array([
        [ 0., 0., 1.],
        [ 0., 1., 0.],
        [-1., 0., 0.]
    ])

    talon_pts = talon_vertices()

    tel = Telemetry(a.mavlink)
    tel.start()

    node = Node()
    state = {"poses": {}, "t": 0.0, "last_frame": None, "new_frame": False,
             "gmb_yaw": 0.0, "gmb_pitch": 0.0, "det": None, "frame_times": deque(maxlen=30),
             "fps": 0.0, "last_frame_time": 0.0}

    def on_clock(msg):
        state["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        for p in msg.pose:
            if p.name in (a.interceptor, a.target):
                state["poses"][p.name] = p

    def on_gmb_yaw(msg):
        state["gmb_yaw"] = msg.data

    def on_gmb_pitch(msg):
        state["gmb_pitch"] = msg.data

    def on_detection(msg):
        if len(msg.data) >= 6:
            state["det"] = {
                "cx": float(msg.data[0]),
                "cy": float(msg.data[1]),
                "w": float(msg.data[2]),
                "h": float(msg.data[3]),
                "range_m": float(msg.data[4]),
                "visible": bool(msg.data[5] > 0.5),
                "time": time.time(),
            }

    def on_image(msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            # Gazebo outputs RGB, OpenCV uses BGR
            state["last_frame"] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            state["new_frame"] = True

            now = time.time()
            state["last_frame_time"] = now
            ft = state["frame_times"]
            ft.append(now)
            if len(ft) >= 2:
                dt = ft[-1] - ft[0]
                if dt > 0.005:
                    state["fps"] = (len(ft) - 1) / dt
        except Exception:
            pass

    node.subscribe(Clock, f"/world/{a.world}/clock", on_clock)
    node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)
    node.subscribe(Double, f"/model/{a.interceptor}/command/seeker_yaw", on_gmb_yaw)
    node.subscribe(Double, f"/model/{a.interceptor}/command/seeker_pitch", on_gmb_pitch)
    node.subscribe(Double_V, f"/model/{a.interceptor}/detection", on_detection)

    # Discover camera topic (on gimbal_pitch_link or base_link)
    cam_topic = f"/world/{a.world}/model/{a.interceptor}/link/gimbal_pitch_link/sensor/nose_camera/image"
    node.subscribe(ImageMsg, cam_topic, on_image)

    # Also scan active topics to catch any alternate link name
    for t in node.topic_list():
        if "nose_camera" in t and t.endswith("/image"):
            if t != cam_topic:
                cam_topic = t
                node.subscribe(ImageMsg, cam_topic, on_image)
            break

    window_name = "Interceptor Nose Camera [HUD]"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, width, height)

    print(f"view_hud: connecting to {cam_topic}, flight data from {a.mavlink}...")
    show_flight_data = True

    while True:
        # Process GUI events
        key = cv2.waitKey(15) & 0xFF
        if key == 27 or key == ord('q'):
            break

        if key == ord('h'):
            show_flight_data = not show_flight_data

        frame = state["last_frame"]
        if frame is None:
            # Show waiting screen
            splash = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.circle(splash, (width // 2, height // 2), 20, HUD, 1, cv2.LINE_AA)
            hud_text(splash, "NO VIDEO", (width / 2, height / 2 + 60), 0.7, "center")
            cv2.imshow(window_name, splash)
            continue

        # draw on a copy: the same frame is shown again until the next one arrives
        frame = frame.copy()
        if time.time() - state.get("last_frame_time", 0.0) > 1.0:
            state["fps"] = 0.0
        pi = state["poses"].get(a.interceptor)
        pt = state["poses"].get(a.target)

        tracking_data = {"in_front": False}
        gmb_data = None

        if pi is not None:
            R_int = quat_to_rot(pi.orientation.w, pi.orientation.x, pi.orientation.y, pi.orientation.z)
            p_int = np.array([pi.position.x, pi.position.y, pi.position.z])

            # Camera position is always at the interceptor nose in world frame
            p_cam_world = p_int + R_int @ cam_offset_body

            # Gimbal camera orientation from commanded seeker pitch and yaw angles
            pitch_rad = float(state.get("gmb_pitch", 0.0))
            yaw_rad = float(state.get("gmb_yaw", 0.0))
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)

            gmb_data = {
                "yaw": yaw_deg,
                "pitch": pitch_deg,
            }

            cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            # Relative gimbal rotation in body FLU: yaw around +Z, pitch around transverse +Y
            R_rel = np.array([
                [ cy * cp, -sy,  cy * sp],
                [ sy * cp,  cy,  sy * sp],
                [    -sp,   0.,      cp ]
            ])

            # Camera orientation in world frame = vehicle body attitude * gimbal relative rotation
            R_cam = R_int @ R_rel

            # Camera optical frame in world (takes world vectors into camera optical coords)
            R_cam_opt_world = GZ_TO_OPT @ R_cam_gz_body @ R_cam.T

            # Project vehicle fuselage centerline (nose +z in body FLU) into camera optical frame
            n_body_world = R_int @ np.array([0., 0., 1.])
            n_body_cam = R_cam_opt_world @ n_body_world
            if n_body_cam[2] > 0.05:
                u_bore = fx * n_body_cam[0] / n_body_cam[2] + cx
                v_bore = fy * n_body_cam[1] / n_body_cam[2] + cy
                tracking_data["boresight"] = (int(u_bore), int(v_bore))

            det = state.get("det")
            now = time.time()
            det_active = (det is not None and (now - det.get("time", 0.0) < 0.6) and det.get("visible"))

            if det_active:
                if det["range_m"] <= a.max_range:
                    cx_px = int(det["cx"] * width)
                    cy_px = int(det["cy"] * height)
                    w_px = max(10, int(det["w"] * width))
                    h_px = max(8, int(det["h"] * height))
                    u_min = int(np.clip(cx_px - w_px // 2, 0, width - 1))
                    u_max = int(np.clip(cx_px + w_px // 2, 0, width - 1))
                    v_min = int(np.clip(cy_px - h_px // 2, 0, height - 1))
                    v_max = int(np.clip(cy_px + h_px // 2, 0, height - 1))

                    tracking_data["in_front"] = True
                    tracking_data["range_m"] = det["range_m"]
                    tracking_data["u_centre"] = cx_px
                    tracking_data["v_centre"] = cy_px
                    tracking_data["in_fov"] = (0 <= cx_px < width and 0 <= cy_px < height)
                    tracking_data["locked"] = True
                    tracking_data["bbox"] = (u_min, v_min, u_max, v_max)
            elif pt is not None:
                # Geometric projection fallback from ground truth poses
                R_tgt = quat_to_rot(pt.orientation.w, pt.orientation.x, pt.orientation.y, pt.orientation.z)
                p_tgt = np.array([pt.position.x, pt.position.y, pt.position.z])

                d_world = p_tgt - p_cam_world
                d_cam_gz = R_cam_gz_body @ (R_cam.T @ d_world)
                d_opt = GZ_TO_OPT @ d_cam_gz

                range_m = float(np.linalg.norm(d_opt))

                if d_opt[2] > 0.1 and range_m <= a.max_range:
                    tracking_data["in_front"] = True
                    tracking_data["range_m"] = range_m
                    los = d_opt / range_m
                    tracking_data["los"] = los

                    u_centre = fx * los[0] / los[2] + cx
                    v_centre = fy * los[1] / los[2] + cy
                    tracking_data["u_centre"] = u_centre
                    tracking_data["v_centre"] = v_centre

                    in_fov = (0 <= u_centre < width and 0 <= v_centre < height)
                    tracking_data["in_fov"] = in_fov
                    tracking_data["locked"] = (range_m <= a.max_range and in_fov)

                    # Bounding box from vertices relative to camera
                    if talon_pts is not None:
                        pts_world = (R_tgt @ talon_pts.T).T + p_tgt
                        pts_rel_world = pts_world - p_cam_world
                        pts_cam = (R_cam_opt_world @ pts_rel_world.T).T

                        in_front_pts = pts_cam[:, 2] > 0.1
                        if np.any(in_front_pts):
                            pts_f = pts_cam[in_front_pts]
                            u_pts = fx * pts_f[:, 0] / pts_f[:, 2] + cx
                            v_pts = fy * pts_f[:, 1] / pts_f[:, 2] + cy
                            u_raw_min = np.floor(u_pts.min() - 2)
                            u_raw_max = np.ceil(u_pts.max() + 2)
                            v_raw_min = np.floor(v_pts.min() - 2)
                            v_raw_max = np.ceil(v_pts.max() + 2)
                            w_box = max(10, int(u_raw_max - u_raw_min))
                            h_box = max(8, int(v_raw_max - v_raw_min))
                            u_c = int((u_raw_min + u_raw_max) / 2)
                            v_c = int((v_raw_min + v_raw_max) / 2)
                            u_min = int(np.clip(u_c - w_box // 2, 0, width - 1))
                            u_max = int(np.clip(u_c + w_box // 2, 0, width - 1))
                            v_min = int(np.clip(v_c - h_box // 2, 0, height - 1))
                            v_max = int(np.clip(v_c + h_box // 2, 0, height - 1))
                            tracking_data["bbox"] = (u_min, v_min, u_max, v_max)

                    # Fallback bounding box if mesh vertices not available
                    if "bbox" not in tracking_data and in_fov and range_m > 0.1:
                        w_px = max(10, int(fx * 1.72 / range_m))
                        h_px = max(8, int(fy * 0.80 / range_m))
                        u_min = int(np.clip(u_centre - w_px // 2, 0, width - 1))
                        u_max = int(np.clip(u_centre + w_px // 2, 0, width - 1))
                        v_min = int(np.clip(v_centre - h_px // 2, 0, height - 1))
                        v_max = int(np.clip(v_centre + h_px // 2, 0, height - 1))
                        tracking_data["bbox"] = (u_min, v_min, u_max, v_max)

        # Render HUD onto frame
        h_img, w_img = frame.shape[:2]

        if show_flight_data:
            draw_flight_data(frame, tel, state["t"], w_img, h_img, gmb_data, state.get("fps", 0.0))

        draw_hud(frame, tracking_data, width, height)
        cv2.imshow(window_name, frame)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

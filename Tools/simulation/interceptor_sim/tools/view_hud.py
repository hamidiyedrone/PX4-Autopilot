#!/usr/bin/env python3
"""
HUD Camera Viewer with Target Bounding Box and 3D Pose Overlay (Minimal, No Text).

Subscribes directly to Gazebo camera image topic and dynamic poses.
Renders:
  - Pixel-perfect 2D Bounding Box around target plane
  - Thin 3D Orientation Arrows on the target (Red: Nose, Green: Right Wing, Blue: Down)
  - Center Boresight Reticle & LOS line
  - Zero text clutter

Usage:
  python3 tools/view_hud.py [--world ankara] [--max-range 100]
"""

import argparse
import math
import os
import re
import struct
import sys
import time

import cv2
import numpy as np

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image as ImageMsg
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_DIR = os.path.join(SIM_DIR, "models", "talon1718")

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
    """Load model vertices from STL meshes or fallback to bounding box corners."""
    sdf_path = os.path.join(MODEL_DIR, "model.sdf")
    pts = [TALON_BBOX_CORNERS]
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
    return np.concatenate(pts)


def draw_hud(img, tracking_data, width, height):
    """Draw clean tactical HUD with Bounding Box and thin 3D Pose arrows (no text)."""
    cx, cy = width // 2, height // 2

    # Draw center boresight reticle
    reticle_color = (0, 200, 255)  # Amber
    cv2.circle(img, (cx, cy), 15, reticle_color, 1, cv2.LINE_AA)
    cv2.line(img, (cx - 25, cy), (cx - 8, cy), reticle_color, 1, cv2.LINE_AA)
    cv2.line(img, (cx + 8, cy), (cx + 25, cy), reticle_color, 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy - 25), (cx, cy - 8), reticle_color, 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy + 8), (cx, cy + 25), reticle_color, 1, cv2.LINE_AA)

    if not tracking_data or not tracking_data.get("in_front"):
        return

    u_c = int(tracking_data["u_centre"])
    v_c = int(tracking_data["v_centre"])
    in_fov = tracking_data["in_fov"]
    locked = tracking_data["locked"]

    # Target color: Green if locked (range <= max_range), Yellow if tracked further out
    tgt_color = (0, 255, 0) if locked else (0, 255, 255)

    if in_fov:
        # Faint line-of-sight line from boresight to target
        cv2.line(img, (cx, cy), (u_c, v_c), (80, 80, 80), 1, cv2.LINE_AA)

        # 1. Bounding Box around airplane
        bbox = tracking_data.get("bbox")
        if bbox:
            u_min, v_min, u_max, v_max = bbox
            w_box = u_max - u_min
            h_box = v_max - v_min

            # Main bounding box
            cv2.rectangle(img, (u_min, v_min), (u_max, v_max), tgt_color, 1, cv2.LINE_AA)

            # Subtle corner brackets
            corner_len = max(4, min(10, min(w_box, h_box) // 3))
            # Top-left
            cv2.line(img, (u_min, v_min), (u_min + corner_len, v_min), tgt_color, 2, cv2.LINE_AA)
            cv2.line(img, (u_min, v_min), (u_min, v_min + corner_len), tgt_color, 2, cv2.LINE_AA)
            # Top-right
            cv2.line(img, (u_max, v_min), (u_max - corner_len, v_min), tgt_color, 2, cv2.LINE_AA)
            cv2.line(img, (u_max, v_min), (u_max, v_min + corner_len), tgt_color, 2, cv2.LINE_AA)
            # Bottom-left
            cv2.line(img, (u_min, v_max), (u_min + corner_len, v_max), tgt_color, 2, cv2.LINE_AA)
            cv2.line(img, (u_min, v_max), (u_min, v_max - corner_len), tgt_color, 2, cv2.LINE_AA)
            # Bottom-right
            cv2.line(img, (u_max, v_max), (u_max - corner_len, v_max), tgt_color, 2, cv2.LINE_AA)
            cv2.line(img, (u_max, v_max), (u_max - corner_len, v_max), tgt_color, 2, cv2.LINE_AA)

        # 2. Thin 3D Pose Arrows directly on the airplane
        axes_2d = tracking_data.get("axes_2d")
        if axes_2d:
            origin_pt = (u_c, v_c)
            # Red: Nose forward (X)
            cv2.arrowedLine(img, origin_pt, axes_2d["nose"], (0, 0, 255), 1, tipLength=0.2, line_type=cv2.LINE_AA)
            # Green: Right wing (Y)
            cv2.arrowedLine(img, origin_pt, axes_2d["right"], (0, 255, 0), 1, tipLength=0.2, line_type=cv2.LINE_AA)
            # Blue: Down/belly (Z)
            cv2.arrowedLine(img, origin_pt, axes_2d["down"], (255, 120, 0), 1, tipLength=0.2, line_type=cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--world", default="ankara")
    ap.add_argument("--interceptor", default="interceptor_0")
    ap.add_argument("--target", default="talon1718_1")
    ap.add_argument("--max-range", type=float, default=100.0)
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

    node = Node()
    state = {"poses": {}, "t": 0.0, "last_frame": None, "new_frame": False}

    def on_clock(msg):
        state["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        for p in msg.pose:
            if p.name in (a.interceptor, a.target):
                state["poses"][p.name] = p

    def on_image(msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            # Gazebo outputs RGB, OpenCV uses BGR
            state["last_frame"] = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            state["new_frame"] = True
        except Exception:
            pass

    node.subscribe(Clock, f"/world/{a.world}/clock", on_clock)
    node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)

    # Discover camera topic
    cam_topic = f"/world/{a.world}/model/{a.interceptor}/link/base_link/sensor/nose_camera/image"
    node.subscribe(ImageMsg, cam_topic, on_image)

    # Also scan active topics to catch any alternate link name
    for t in node.topic_list():
        if "nose_camera" in t and t.endswith("/image"):
            if t != cam_topic:
                cam_topic = t
                node.subscribe(ImageMsg, cam_topic, on_image)
            break

    window_name = "Interceptor Tactical Nose Camera [HUD]"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, width, height)

    print(f"view_hud: connecting to {cam_topic}...")

    while True:
        # Process GUI events
        key = cv2.waitKey(15) & 0xFF
        if key == 27 or key == ord('q'):
            break

        frame = state["last_frame"]
        if frame is None:
            # Show waiting screen
            splash = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.circle(splash, (width // 2, height // 2), 20, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.imshow(window_name, splash)
            continue

        pi = state["poses"].get(a.interceptor)
        pt = state["poses"].get(a.target)

        tracking_data = {"in_front": False}

        if pi is not None and pt is not None:
            R_int = quat_to_rot(pi.orientation.w, pi.orientation.x, pi.orientation.y, pi.orientation.z)
            p_int = np.array([pi.position.x, pi.position.y, pi.position.z])

            R_tgt = quat_to_rot(pt.orientation.w, pt.orientation.x, pt.orientation.y, pt.orientation.z)
            p_tgt = np.array([pt.position.x, pt.position.y, pt.position.z])

            p_cam_world = p_int + R_int @ cam_offset_body
            d_world = p_tgt - p_cam_world
            d_cam_gz = R_cam_gz_body @ (R_int.T @ d_world)
            d_opt = GZ_TO_OPT @ d_cam_gz

            range_m = float(np.linalg.norm(d_opt))

            if d_opt[2] > 0.1:
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

                # Camera optical frame in world
                R_cam_opt_world = GZ_TO_OPT @ R_cam_gz_body @ R_int.T

                # Target FRD (x nose, y right wing, z down) in optical frame
                R_flu_to_frd = np.diag([1.0, -1.0, -1.0])
                R_tgt_frd = R_tgt @ R_flu_to_frd
                R_target_in_cam = R_cam_opt_world @ R_tgt_frd

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
                        u_min = int(np.clip(u_pts.min() - 2, 0, width - 1))
                        u_max = int(np.clip(u_pts.max() + 2, 0, width - 1))
                        v_min = int(np.clip(v_pts.min() - 2, 0, height - 1))
                        v_max = int(np.clip(v_pts.max() + 2, 0, height - 1))
                        if u_max - u_min > 2 and v_max - v_min > 2:
                            tracking_data["bbox"] = (u_min, v_min, u_max, v_max)

                # Thin 3D Pose Arrows directly on the airplane
                # 3D axis length: ~1.2m
                axis_len_3d = 1.2
                axes_endpoints = {
                    "nose":  d_opt + R_target_in_cam[:, 0] * axis_len_3d,
                    "right": d_opt + R_target_in_cam[:, 1] * axis_len_3d,
                    "down":  d_opt + R_target_in_cam[:, 2] * (axis_len_3d * 0.6),
                }
                axes_2d = {}
                for axis_name, pt_3d in axes_endpoints.items():
                    if pt_3d[2] > 0.1:
                        u_ax = int(fx * pt_3d[0] / pt_3d[2] + cx)
                        v_ax = int(fy * pt_3d[1] / pt_3d[2] + cy)
                        # Ensure visible minimum length (20 px)
                        dx_ax = u_ax - u_centre
                        dy_ax = v_ax - v_centre
                        dist_px = math.hypot(dx_ax, dy_ax)
                        if 0 < dist_px < 20.0:
                            s = 20.0 / dist_px
                            u_ax = int(u_centre + dx_ax * s)
                            v_ax = int(v_centre + dy_ax * s)
                        axes_2d[axis_name] = (u_ax, v_ax)

                if len(axes_2d) == 3:
                    tracking_data["axes_2d"] = axes_2d

        # Render HUD onto frame
        draw_hud(frame, tracking_data, width, height)
        cv2.imshow(window_name, frame)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

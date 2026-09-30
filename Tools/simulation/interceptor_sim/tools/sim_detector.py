#!/usr/bin/env python3
"""
Ground-truth target detector for SITL.

Subscribes to Gazebo poses (dynamic_pose/info, 250 Hz) of the interceptor
and the Talon.  Each tick:

1. Compute the Talon position relative to the interceptor's nose camera.
2. If the Talon is inside the camera FOV and closer than --max-range,
   project it to get a bounding box and compute line-of-sight, range,
   and target attitude — all in the camera optical frame (x right, y down,
   z along the optical axis).
3. Pack a binary target_vision packet (see target_vision_protocol.h) and
   send it over UDP to the target_vision driver at --rate Hz.

No noise or detection latency is added; this is a perfect detector for
algorithm development.  Robustness tests add noise later (step 8).

    python3 tools/sim_detector.py                        # defaults
    python3 tools/sim_detector.py --rate 30 --port 15600

Mirrored wire format: src/drivers/target_vision/target_vision_protocol.h
"""

import argparse
import math
import os
import re
import socket
import struct
import time

import numpy as np
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_DIR = os.path.join(SIM_DIR, "models", "talon1718")

# ----------------------------------------------------------------- protocol ---

MAGIC = b"TV"
VERSION = 1
PAYLOAD_SIZE = 64
PACKET_SIZE = 70
CRC_OFFSET = 2
CRC_LENGTH = 66  # from version to end of bbox (66 bytes)

FLAG_DETECTED = 1 << 0
FLAG_RANGE    = 1 << 1
FLAG_ATTITUDE = 1 << 2


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT (poly 0x1021) matching target_vision_crc16 in the driver."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_packet(frame_id, latency_us, flags, confidence,
                los, range_m, range_sigma_m, q, bbox):
    """Build a 70-byte target_vision packet."""
    # header: magic (2), version (1), payload_len (1)
    hdr = MAGIC + struct.pack("<BB", VERSION, PAYLOAD_SIZE)
    # payload: 64 bytes
    payload = struct.pack("<II", frame_id, latency_us)
    payload += struct.pack("<BB", flags, confidence)
    payload += struct.pack("<H", 0)  # reserved
    payload += struct.pack("<3f", *los)
    payload += struct.pack("<f", range_m)
    payload += struct.pack("<f", range_sigma_m)
    payload += struct.pack("<4f", *q)
    payload += struct.pack("<4f", *bbox)
    assert len(payload) == PAYLOAD_SIZE
    body = hdr + payload  # 68 bytes before CRC
    crc = crc16_ccitt(body[CRC_OFFSET:])  # version through bbox
    return body + struct.pack("<H", crc)


# --------------------------------------------------------------- geometry ---

def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])


def rot_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


# gz camera frame (+x forward, +y left, +z up) -> camera optical frame (+x right, +y down, +z forward)
GZ_TO_OPT = np.array([
    [ 0., -1.,  0.],
    [ 0.,  0., -1.],
    [ 1.,  0.,  0.]
])


def read_stl_bounds(path):
    """Read an STL and return all unique vertices."""
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.frombuffer(f.read(n * 50),
                            dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return rec["v"].reshape(-1, 3).astype(np.float64)


def talon_vertices():
    """All visual vertices of the Talon model, in its body frame."""
    sdf = open(os.path.join(MODEL_DIR, "model.sdf")).read()
    link_pose = {m.group(1): np.array([float(v) for v in m.group(2).split()[:3]])
                 for m in re.finditer(r'<link name="(\w+)">\s*<pose>([^<]+)</pose>', sdf)}
    pts = []
    for link in re.finditer(r'<link name="(\w+)">(.*?)</link>', sdf, re.S):
        offset = link_pose.get(link.group(1), np.zeros(3))
        for uri in re.findall(r"<uri>model://talon1718/meshes/([^<]+)</uri>", link.group(2)):
            v = np.unique(np.round(read_stl_bounds(os.path.join(MODEL_DIR, "meshes", uri)), 3), axis=0)
            pts.append(v + offset)
    return np.concatenate(pts)


# -------------------------------------------------------------------- main ---

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="ankara")
    ap.add_argument("--interceptor", default="interceptor_0",
                    help="interceptor model name in Gazebo")
    ap.add_argument("--target", default="talon1718_1",
                    help="target model name in Gazebo")
    ap.add_argument("--port", type=int, default=15600,
                    help="UDP port of the target_vision driver")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="detection rate [Hz]")
    ap.add_argument("--max-range", type=float, default=500.0,
                    help="max detection range [m]")
    ap.add_argument("--hfov", type=float, default=None,
                    help="camera HFOV [deg], default: from interceptor.yaml")
    ap.add_argument("--width", type=int, default=None,
                    help="image width [px], default: from interceptor.yaml")
    ap.add_argument("--height", type=int, default=None,
                    help="image height [px], default: from interceptor.yaml")
    a = ap.parse_args()

    # camera intrinsics: from interceptor.yaml unless overridden
    import yaml
    with open(os.path.join(SIM_DIR, "interceptor.yaml")) as f:
        cfg = yaml.safe_load(f)
    cam = cfg.get("camera", {})
    width  = a.width  or cam.get("width", 1280)
    height = a.height or cam.get("height", 720)
    hfov   = a.hfov   or cam.get("hfov_deg", 60.0)
    hfov_rad = math.radians(hfov)
    vfov_rad = 2 * math.atan(math.tan(hfov_rad / 2) * height / width)
    fx = width  / 2 / math.tan(hfov_rad / 2)
    fy = fx  # square pixels
    cx, cy = width / 2, height / 2

    # Talon vertices for bounding box projection
    try:
        talon_pts = talon_vertices()
        print(f"Talon model: {len(talon_pts)} vertices")
    except Exception as e:
        print(f"Warning: could not load Talon model ({e}), using point-source bbox")
        talon_pts = None

    # gz camera sensor pose relative to the interceptor body:
    # from build_interceptor.py: <pose>0 0 {nose} 0 -1.5708 0</pose>
    # This means the sensor is at the nose tip, pitched -90° so its +x axis
    # (gz camera forward) aligns with the interceptor's +z (thrust/nose direction).
    # We need R_body_cam_gz to go from body frame to gz-camera frame.
    cam_pitch = -math.pi / 2
    cp, sp = math.cos(cam_pitch), math.sin(cam_pitch)
    R_cam_gz_body = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp]
    ])  # rotation that takes body-frame vectors into gz-camera-frame vectors

    # camera offset in the body frame (approximately at the nose)
    fus_len = cfg["fuselage"]["length"]
    # CG is computed in build_interceptor.py; approximate it from the components
    components = cfg.get("components", {})
    total_mass = cfg["fuselage"]["mass"]
    cg_num = 0.0
    for name, comp in components.items():
        total_mass += comp["mass"]
        cg_num += comp["mass"] * comp["z"]
    cg_num += cfg["fuselage"]["mass"] * fus_len / 2
    cg = cg_num / total_mass
    cam_offset_body = np.array([0.0, 0.0, fus_len - cg])  # nose in body FLU (z up)

    # gz transport
    node = Node()
    state = {"poses": {}, "t": 0.0}

    def on_clock(msg):
        state["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        for p in msg.pose:
            if p.name in (a.interceptor, a.target):
                state["poses"][p.name] = p

    node.subscribe(Clock, f"/world/{a.world}/clock", on_clock)
    node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", a.port)

    print(f"sim_detector: {a.interceptor} camera → {a.target}, "
          f"udp {a.port}, {a.rate:.0f} Hz, max range {a.max_range:.0f} m, "
          f"FOV {hfov:.0f}°×{math.degrees(vfov_rad):.0f}°, "
          f"{width}×{height}")

    # wait for both models
    while a.interceptor not in state["poses"] or a.target not in state["poses"]:
        time.sleep(0.1)
    print("Both models found, starting detection loop", flush=True)

    frame_id = 0
    dt = 1.0 / a.rate
    last_send = 0.0
    last_print = 0.0

    while True:
        time.sleep(0.002)
        t_sim = state["t"]

        if t_sim - last_send < dt:
            continue

        last_send = t_sim
        frame_id += 1

        # get poses
        pi = state["poses"].get(a.interceptor)
        pt = state["poses"].get(a.target)

        if pi is None or pt is None:
            continue

        # interceptor pose in world
        R_int = quat_to_rot(pi.orientation.w, pi.orientation.x,
                            pi.orientation.y, pi.orientation.z)
        p_int = np.array([pi.position.x, pi.position.y, pi.position.z])

        # target pose in world
        R_tgt = quat_to_rot(pt.orientation.w, pt.orientation.x,
                            pt.orientation.y, pt.orientation.z)
        p_tgt = np.array([pt.position.x, pt.position.y, pt.position.z])

        # camera position in world
        p_cam_world = p_int + R_int @ cam_offset_body

        # target relative to camera, in gz-camera frame
        d_world = p_tgt - p_cam_world
        d_cam_gz = R_cam_gz_body @ (R_int.T @ d_world)

        # convert to camera optical frame
        d_opt = GZ_TO_OPT @ d_cam_gz  # [x_right, y_down, z_forward]

        range_m = float(np.linalg.norm(d_opt))

        # check: target must be in front of the camera and within range
        if d_opt[2] <= 0 or range_m > a.max_range or range_m < 0.1:
            pkt = pack_packet(frame_id, 0, 0, 0,
                              [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
            sock.sendto(pkt, dest)
            continue

        # line of sight: unit vector camera → target in optical frame
        los = d_opt / range_m

        # check FOV: project the centre of the target
        u_centre = fx * los[0] / los[2] + cx
        v_centre = fy * los[1] / los[2] + cy

        # generous FOV check (target centre within 1.2× the image)
        margin = 1.2
        in_fov = (-width * (margin - 1) / 2 < u_centre < width * margin
                  and -height * (margin - 1) / 2 < v_centre < height * margin)

        if not in_fov:
            pkt = pack_packet(frame_id, 0, 0, 0,
                              [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
            sock.sendto(pkt, dest)
            continue

        # --- target is visible ---

        # target attitude in camera optical frame
        # R_tgt is world→target-body (gz: x forward, y left, z up = FLU)
        # We want target FRD (x nose, y right wing, z down) in optical frame
        # Talon body in gz: x nose, y left, z up (FLU)
        # FLU → FRD: flip y and z
        R_flu_to_frd = np.diag([1.0, -1.0, -1.0])
        R_tgt_frd = R_tgt @ R_flu_to_frd  # columns of R_tgt_frd are FRD axes in world

        # camera optical frame axes in world:
        # R_cam_opt_world takes world vectors into optical frame
        R_cam_opt_world = GZ_TO_OPT @ R_cam_gz_body @ R_int.T

        # target attitude in camera optical frame
        R_target_in_cam = R_cam_opt_world @ R_tgt_frd
        q = rot_to_quat(R_target_in_cam)

        # bounding box from projected vertices
        bbox = [0.0, 0.0, 0.0, 0.0]
        if talon_pts is not None:
            # transform Talon vertices to camera optical frame
            pts_world = (R_tgt @ talon_pts.T).T + p_tgt
            pts_cam = (R_cam_opt_world @ pts_world.T).T
            # project only points in front of the camera
            in_front = pts_cam[:, 2] > 0.1
            if np.any(in_front):
                pts_f = pts_cam[in_front]
                u = fx * pts_f[:, 0] / pts_f[:, 2] + cx
                v = fy * pts_f[:, 1] / pts_f[:, 2] + cy
                # clip to image
                u = np.clip(u, 0, width)
                v = np.clip(v, 0, height)
                u_min, u_max = u.min(), u.max()
                v_min, v_max = v.min(), v.max()
                bw, bh = u_max - u_min, v_max - v_min
                if bw > 0.5 and bh > 0.5:
                    bbox = [
                        float((u_min + u_max) / 2 / width),   # cx normalised
                        float((v_min + v_max) / 2 / height),  # cy normalised
                        float(bw / width),                     # w normalised
                        float(bh / height),                    # h normalised
                    ]

        flags = FLAG_DETECTED | FLAG_RANGE | FLAG_ATTITUDE
        confidence = 255  # perfect detection
        range_sigma = 0.1  # negligible uncertainty for ground truth

        pkt = pack_packet(
            frame_id, 0, flags, confidence,
            [float(los[0]), float(los[1]), float(los[2])],
            float(range_m), range_sigma,
            [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            bbox
        )
        sock.sendto(pkt, dest)

        # periodic status
        if t_sim - last_print >= 5.0:
            print(f"t={t_sim:7.1f}  range {range_m:6.1f} m  "
                  f"los [{los[0]:+.2f} {los[1]:+.2f} {los[2]:+.2f}]  "
                  f"bbox [{bbox[0]:.2f} {bbox[1]:.2f} {bbox[2]:.3f} {bbox[3]:.3f}]  "
                  f"frames {frame_id}", flush=True)
            last_print = t_sim


if __name__ == "__main__":
    main()

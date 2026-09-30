#!/usr/bin/env python3
"""
Send fake target_vision packets over UDP to test the driver without Gazebo.

Run PX4 SITL normally (any vehicle, e.g. make px4_sitl gz_x500), then:

    # in a second terminal:
    python3 tools/test_target_vision.py

    # in the PX4 shell (pxh>):
    target_vision start -u 15600
    target_vision status          # packet / detection counters
    listener target_detection     # live uORB messages

The script sends a detection at 10 Hz with a target straight ahead at 80 m,
closing at 1 m/s, for 30 seconds. The line of sight, range, attitude and
bounding box change each tick so the output is easy to follow.
"""

import math
import socket
import struct
import time

# ---- protocol (mirrors target_vision_protocol.h) ----

MAGIC = b"TV"
VERSION = 1
PAYLOAD_SIZE = 64
FLAG_DETECTED = 1 << 0
FLAG_RANGE    = 1 << 1
FLAG_ATTITUDE = 1 << 2


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def pack_packet(frame_id, flags, confidence, los, range_m, range_sigma,
                q, bbox, latency_us=0):
    hdr = MAGIC + struct.pack("<BB", VERSION, PAYLOAD_SIZE)
    payload = struct.pack("<II", frame_id, latency_us)
    payload += struct.pack("<BBH", flags, confidence, 0)
    payload += struct.pack("<3f", *los)
    payload += struct.pack("<2f", range_m, range_sigma)
    payload += struct.pack("<4f", *q)
    payload += struct.pack("<4f", *bbox)
    body = hdr + payload
    return body + struct.pack("<H", crc16_ccitt(body[2:]))


# ---- scenario ----

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", 15600)
    rate = 10  # Hz
    duration = 30  # seconds

    print(f"Sending test packets to udp {dest[1]} at {rate} Hz for {duration} s")
    print()
    print("In the PX4 shell (pxh>):")
    print("  target_vision start -u 15600")
    print("  target_vision status")
    print("  listener target_detection")
    print()

    frame = 0
    t0 = time.monotonic()

    while True:
        t = time.monotonic() - t0
        if t > duration:
            break

        frame += 1
        range_m = max(5.0, 80.0 - t * 1.0)  # closing from 80 m

        # target slightly off-centre, oscillating left-right
        az = math.radians(5.0 * math.sin(t * 0.5))
        el = math.radians(-3.0)  # slightly below
        los = [math.sin(az) * math.cos(el),
               -math.sin(el),
               math.cos(az) * math.cos(el)]

        # identity attitude (target nose aligned with camera forward)
        q = [1.0, 0.0, 0.0, 0.0]

        # bounding box growing as range decreases (Talon ~2 m wingspan)
        angular = 2.0 / range_m  # radians
        fx = 1280 / 2 / math.tan(math.radians(30))  # 60° hfov
        box_px = angular * fx
        bbox = [0.5 + los[0] * 0.3,
                0.5 + los[1] * 0.3,
                min(0.8, box_px / 1280),
                min(0.6, box_px / 720)]

        pkt = pack_packet(
            frame, FLAG_DETECTED | FLAG_RANGE | FLAG_ATTITUDE, 230,
            los, range_m, 0.5, q, bbox
        )
        sock.sendto(pkt, dest)

        if frame % (rate * 2) == 1:
            print(f"  t={t:5.1f}s  frame={frame:4d}  range={range_m:5.1f} m  "
                  f"bbox_w={bbox[2]:.3f}", flush=True)

        time.sleep(1.0 / rate)

    # send a few "no detection" packets
    for i in range(5):
        frame += 1
        pkt = pack_packet(frame, 0, 0, [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
        sock.sendto(pkt, dest)
        time.sleep(0.1)

    print(f"\nDone: {frame} packets sent (last 5 = no detection)")
    print("Check: target_vision status")


if __name__ == "__main__":
    main()

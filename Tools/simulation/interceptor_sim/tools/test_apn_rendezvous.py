#!/usr/bin/env python3
"""
Automated 3D Blended APN + Capture Basket Rendezvous Flight Test Runner
with 50 Hz Synchronized Multi-Channel Telemetry Recording & Analysis.

Executes:
1. Clean environment reset.
2. Headless Gazebo Harmonic + Talon target_sim + ground_relay + PX4 SITL (interceptor).
3. 50 Hz high-frequency multi-channel telemetry recording:
   - Interceptor 6-DOF true states (NED pos, vel, acc, roll, pitch, yaw, total tilt)
   - Target 6-DOF true states (NED pos, vel, acc, roll, pitch, yaw)
   - Commanded trajectory setpoints (accel cmd, vel cmd, yaw cmd)
   - Gimbal seeker angles (commanded & actual link roll, pitch, yaw)
   - Net launcher barrel angles (commanded & actual pitch)
   - Relative geometry (R_t, R_v, V_c, guidance phase, target source)
4. Arming, Takeoff, and Intercept Mode Engagement ('ext1').
5. High-resolution live telemetry dashboard & firing envelope detection.
6. Post-flight analytical evaluation (pitch oscillation analysis, tracking accuracy, capture verification).
7. Exports full CSV & JSONL logs to build/telemetry/.
"""

import os
import sys

# Force pure-python protobuf before importing gz or google.protobuf
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import time
import math
import json
import csv
import urllib.request
import subprocess
import threading
from datetime import datetime

import numpy as np

try:
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.double_v_pb2 import Double_V
except ImportError:
    pass

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "../../.."))
BUILD_DIR = os.path.join(PX4_DIR, "build/px4_sitl_default")
LOG_FILE = os.path.join(SIM_DIR, "build/sim_test.log")
TELEM_DIR = os.path.join(SIM_DIR, "build/telemetry")
os.makedirs(TELEM_DIR, exist_ok=True)


def quat_to_rpy(qw, qx, qy, qz):
    """Convert quaternion [w, x, y, z] to Roll, Pitch, Yaw in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def run_px4_cmd(*args):
    px4cmd = os.path.join(SCRIPT_DIR, "px4cmd.sh")
    cmd = [px4cmd, "0"] + list(args)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return res.stdout.strip()
    except Exception as e:
        return f"Error: {e}"


def check_target_http(timeout=0.5):
    try:
        with urllib.request.urlopen("http://127.0.0.1:8000/target", timeout=timeout) as r:
            if r.status == 200:
                return json.loads(r.read().decode())
    except Exception:
        pass
    return None


class HighRateTelemetryRecorder:
    def __init__(self, out_dir):
        self.out_dir = out_dir
        self.lock = threading.Lock()
        self.running = False
        self.records = []
        self.sim_time = 0.0

        # Poses & previous states for numerical differentiation
        self.poses = {}
        self.prev_poses = {}
        self.prev_times = {}

        # Actuator commands
        self.cmd_gmb_yaw = 0.0
        self.cmd_gmb_pitch = 0.0
        self.cmd_gmb_roll = 0.0
        self.cmd_barrel_pitch = 0.0

        # Trajectory setpoints from PX4
        self.cmd_acc = np.zeros(3)
        self.cmd_vel = np.zeros(3)
        self.cmd_yaw = 0.0

        # Intercept module status
        self.phase_str = "STANDBY"
        self.target_source = "NONE"
        self.basket_rv = 999.0
        self.target_rt = 999.0
        self.closing_vc = 0.0
        self.net_fired = False
        self.firing_counter = 0

        # Gazebo node
        self.node = Node()
        self.node.subscribe(Clock, "/world/ankara/clock", self._on_clock)
        self.node.subscribe(Pose_V, "/world/ankara/dynamic_pose/info", self._on_poses)
        self.node.subscribe(Double, "/model/interceptor_0/command/seeker_yaw", self._on_gmb_yaw)
        self.node.subscribe(Double, "/model/interceptor_0/command/seeker_pitch", self._on_gmb_pitch)
        self.node.subscribe(Double, "/model/interceptor_0/command/seeker_roll", self._on_gmb_roll)
        self.node.subscribe(Double, "/model/interceptor_0/command/net_launcher_pitch", self._on_barrel_pitch)

        # MAVLink listener thread
        self.mav_thread = None
        self.record_thread = None

    def _on_clock(self, msg):
        self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_gmb_yaw(self, msg):
        with self.lock:
            self.cmd_gmb_yaw = msg.data

    def _on_gmb_pitch(self, msg):
        with self.lock:
            self.cmd_gmb_pitch = msg.data

    def _on_gmb_roll(self, msg):
        with self.lock:
            self.cmd_gmb_roll = msg.data

    def _on_barrel_pitch(self, msg):
        with self.lock:
            self.cmd_barrel_pitch = msg.data

    def _on_poses(self, msg):
        now = self.sim_time
        with self.lock:
            for p in msg.pose:
                name = p.name
                if name in ("interceptor_0", "talon1718_1") or "gimbal_pitch_link" in name or "net_launcher_barrel_link" in name:
                    pos = np.array([p.position.x, p.position.y, p.position.z])
                    ori = np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])

                    vel = np.zeros(3)
                    acc = np.zeros(3)
                    if name in self.prev_poses and now > self.prev_times.get(name, 0.0):
                        dt = max(0.001, now - self.prev_times[name])
                        prev_pos, prev_ori, prev_vel = self.prev_poses[name]
                        vel = (pos - prev_pos) / dt
                        acc = (vel - prev_vel) / dt

                    self.poses[name] = {
                        "pos": pos,
                        "ori": ori,
                        "vel": vel,
                        "acc": acc,
                        "time": now
                    }
                    self.prev_poses[name] = (pos, ori, vel)
                    self.prev_times[name] = now

    def update_from_intercept_status(self, status_text):
        with self.lock:
            for line in status_text.splitlines():
                if "Guidance Phase:" in line:
                    self.phase_str = line.split("Guidance Phase:")[-1].strip()
                if "Target Source:" in line:
                    parts = line.split(",")
                    self.target_source = parts[0].split(":")[-1].strip()
                if "Target Distance:" in line:
                    # Target Distance: 45.2 m (age: 0.05s), Basket Dist R_v: 42.80 m, V_c: 12.4 m/s
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            self.target_rt = float(parts[0].split(":")[-1].split("m")[0].strip())
                            self.basket_rv = float(parts[1].split(":")[-1].split("m")[0].strip())
                            self.closing_vc = float(parts[2].split(":")[-1].split("m")[0].strip())
                        except ValueError:
                            pass
                if "Net Gun Status:" in line:
                    self.net_fired = "FIRED" in line
                    if "Envelope Counter:" in line:
                        try:
                            cnt_str = line.split("Envelope Counter:")[-1].split("/")[0].strip()
                            self.firing_counter = int(cnt_str)
                        except ValueError:
                            pass
                if "Command Accel:" in line:
                    # Command Accel: [2.50, -1.20, 0.30] m/s^2, Yaw Cmd: 45.2 deg
                    try:
                        acc_part = line.split("Command Accel: [")[1].split("]")[0]
                        acc_vals = [float(x.strip()) for x in acc_part.split(",")]
                        self.cmd_acc = np.array(acc_vals)
                        yaw_part = line.split("Yaw Cmd:")[-1].split("deg")[0].strip()
                        self.cmd_yaw = float(yaw_part)
                    except Exception:
                        pass

    def start(self):
        self.running = True
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()

    def stop(self):
        self.running = False
        if self.record_thread:
            self.record_thread.join(timeout=2.0)

    def _record_loop(self):
        dt = 0.020  # 50 Hz
        while self.running:
            time.sleep(dt)
            with self.lock:
                t = self.sim_time
                p_int = self.poses.get("interceptor_0")
                p_tgt = self.poses.get("talon1718_1")
                p_gmb = self.poses.get("gimbal_pitch_link")
                p_bar = self.poses.get("net_launcher_barrel_link")

                if p_int is None or p_tgt is None:
                    continue

                # Relative distance and virtual basket geometry from Gazebo ground truth:
                pos_int = p_int["pos"]
                vel_int = p_int["vel"]
                pos_tgt = p_tgt["pos"]
                vel_tgt = p_tgt["vel"]

                rel_pos = pos_tgt - pos_int
                target_dist_rt = float(np.linalg.norm(rel_pos))

                tgt_speed = float(np.linalg.norm(vel_tgt))
                u_tgt = (vel_tgt / tgt_speed) if tgt_speed > 1.0 else np.array([1.0, 0.0, 0.0])

                # Virtual basket: 2.5m rear of Talon
                p_basket = pos_tgt - u_tgt * 2.5
                r_basket = p_basket - pos_int
                basket_dist_rv = float(np.linalg.norm(r_basket))

                v_rel = vel_tgt - vel_int
                closing_speed_vc = float(-np.dot(r_basket, v_rel) / max(basket_dist_rv, 0.01))

                self.target_rt = target_dist_rt
                self.basket_rv = basket_dist_rv
                self.closing_vc = closing_speed_vc

                # Euler angles for interceptor
                r_int, p_int_pitch, y_int = quat_to_rpy(*p_int["ori"])
                r_tgt, p_tgt_pitch, y_tgt = quat_to_rpy(*p_tgt["ori"])

                # Interceptor total tilt angle from vertical (Z up in ENU)
                tilt_rad = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (p_int["ori"][1]**2 + p_int["ori"][2]**2))))
                tilt_deg = math.degrees(tilt_rad)

                # Actual gimbal link Euler angles
                act_gmb_pitch = 0.0
                act_gmb_yaw = 0.0
                if p_gmb is not None:
                    _, gp, gy = quat_to_rpy(*p_gmb["ori"])
                    act_gmb_pitch = math.degrees(gp)
                    act_gmb_yaw = math.degrees(gy)

                # Actual barrel link pitch
                act_bar_pitch = 0.0
                if p_bar is not None:
                    _, bp, _ = quat_to_rpy(*p_bar["ori"])
                    act_bar_pitch = math.degrees(bp)

                row = {
                    "sim_time": round(t, 3),
                    "phase": self.phase_str,
                    "target_source": self.target_source,

                    # Target True Kinematics (ENU to NED: X=E, Y=N, Z=U -> N=Y, E=X, D=-Z)
                    "tgt_pos_n": round(float(p_tgt["pos"][1]), 3),
                    "tgt_pos_e": round(float(p_tgt["pos"][0]), 3),
                    "tgt_pos_d": round(float(-p_tgt["pos"][2]), 3),
                    "tgt_vel_n": round(float(p_tgt["vel"][1]), 3),
                    "tgt_vel_e": round(float(p_tgt["vel"][0]), 3),
                    "tgt_vel_d": round(float(-p_tgt["vel"][2]), 3),
                    "tgt_speed": round(float(np.linalg.norm(p_tgt["vel"])), 2),
                    "tgt_acc_n": round(float(p_tgt["acc"][1]), 3),
                    "tgt_acc_e": round(float(p_tgt["acc"][0]), 3),
                    "tgt_acc_d": round(float(-p_tgt["acc"][2]), 3),
                    "tgt_roll_deg": round(math.degrees(r_tgt), 1),
                    "tgt_pitch_deg": round(math.degrees(p_tgt_pitch), 1),
                    "tgt_yaw_deg": round(math.degrees(y_tgt), 1),

                    # Interceptor True Kinematics (NED)
                    "int_pos_n": round(float(p_int["pos"][1]), 3),
                    "int_pos_e": round(float(p_int["pos"][0]), 3),
                    "int_pos_d": round(float(-p_int["pos"][2]), 3),
                    "int_vel_n": round(float(p_int["vel"][1]), 3),
                    "int_vel_e": round(float(p_int["vel"][0]), 3),
                    "int_vel_d": round(float(-p_int["vel"][2]), 3),
                    "int_speed": round(float(np.linalg.norm(p_int["vel"])), 2),
                    "int_acc_n": round(float(p_int["acc"][1]), 3),
                    "int_acc_e": round(float(p_int["acc"][0]), 3),
                    "int_acc_d": round(float(-p_int["acc"][2]), 3),
                    "int_roll_deg": round(math.degrees(r_int), 1),
                    "int_pitch_deg": round(math.degrees(p_int_pitch), 1),
                    "int_yaw_deg": round(math.degrees(y_int), 1),
                    "int_tilt_deg": round(tilt_deg, 1),

                    # Commanded Setpoints
                    "cmd_acc_n": round(float(self.cmd_acc[0]), 3),
                    "cmd_acc_e": round(float(self.cmd_acc[1]), 3),
                    "cmd_acc_d": round(float(self.cmd_acc[2]), 3),
                    "cmd_acc_norm": round(float(np.linalg.norm(self.cmd_acc)), 2),
                    "cmd_yaw_deg": round(float(self.cmd_yaw), 1),

                    # Gimbal & Barrel Actuation
                    "cmd_gmb_yaw_deg": round(math.degrees(self.cmd_gmb_yaw), 1),
                    "cmd_gmb_pitch_deg": round(math.degrees(self.cmd_gmb_pitch), 1),
                    "cmd_gmb_roll_deg": round(math.degrees(self.cmd_gmb_roll), 1),
                    "cmd_barrel_pitch_deg": round(math.degrees(self.cmd_barrel_pitch), 1),
                    "act_gmb_pitch_deg": round(act_gmb_pitch, 1),
                    "act_gmb_yaw_deg": round(act_gmb_yaw, 1),
                    "act_barrel_pitch_deg": round(act_bar_pitch, 1),

                    # Relative Geometry & Capture Envelope
                    "target_dist_rt": round(self.target_rt, 2),
                    "basket_dist_rv": round(self.basket_rv, 2),
                    "closing_speed_vc": round(self.closing_vc, 2),
                    "firing_counter": self.firing_counter,
                    "net_fired": 1 if self.net_fired else 0
                }
                self.records.append(row)

    def save_logs(self):
        if not self.records:
            print("Kaydedilecek telemetri kaydı bulunamadı.")
            return None, None

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.out_dir, f"flight_telemetry_50hz_{ts_str}.csv")
        jsonl_path = os.path.join(self.out_dir, f"flight_telemetry_50hz_{ts_str}.jsonl")

        # Write CSV
        fieldnames = list(self.records[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        # Write JSONL
        with open(jsonl_path, "w") as f:
            for r in self.records:
                f.write(json.dumps(r) + "\n")

        return csv_path, jsonl_path

    def analyze_flight(self):
        """Analyzes pitch stability, gimbal tracking, and capture envelope."""
        if not self.records:
            return {}

        rec = self.records
        # Filter active tracking records (after standby)
        active_rec = [r for r in rec if r["phase"] not in ("STANDBY", "")]
        if not active_rec:
            active_rec = rec

        int_tilts = [r["int_tilt_deg"] for r in active_rec]
        int_pitches = [r["int_pitch_deg"] for r in active_rec]
        cmd_accs = [r["cmd_acc_norm"] for r in active_rec]
        basket_rvs = [r["basket_dist_rv"] for r in active_rec if r["basket_dist_rv"] < 900]

        # Pitch oscillations: check rate of zero-crossings or variance
        pitch_diffs = np.diff(int_pitches)
        pitch_reversals = sum(1 for i in range(1, len(pitch_diffs)) if pitch_diffs[i] * pitch_diffs[i-1] < -0.2)
        duration_s = max(1.0, active_rec[-1]["sim_time"] - active_rec[0]["sim_time"])
        reversal_rate_hz = pitch_reversals / duration_s

        min_rv = min(basket_rvs) if basket_rvs else 999.0
        max_tilt = max(int_tilts) if int_tilts else 0.0
        mean_tilt = float(np.mean(int_tilts)) if int_tilts else 0.0
        std_tilt = float(np.std(int_tilts)) if int_tilts else 0.0

        return {
            "duration_s": round(duration_s, 1),
            "sample_count": len(rec),
            "min_basket_rv_m": round(min_rv, 2),
            "mean_tilt_deg": round(mean_tilt, 1),
            "max_tilt_deg": round(max_tilt, 1),
            "std_tilt_deg": round(std_tilt, 1),
            "pitch_reversal_rate_hz": round(reversal_rate_hz, 2),
            "smooth_flight_verified": (reversal_rate_hz < 1.2 and std_tilt < 12.0),
            "net_fired": any(r["net_fired"] == 1 for r in rec)
        }


def main():
    print("=" * 80)
    print("  OCTOPUS INTERCEPTOR - 3D BLENDED APN & RENDEZVOUS FLIGHT TEST")
    print("      (50 Hz High-Rate Telemetry Recorder & Pitch Stability Analyzer)")
    print("=" * 80, flush=True)

    # 1. Clean up lingering simulation processes
    print("[1/6] Önceki simülasyon süreçleri temizleniyor...")
    subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)
    time.sleep(1.0)
    os.makedirs(os.path.join(SIM_DIR, "build"), exist_ok=True)

    # 2. Launch simulation headless
    print("[2/6] Gazebo Harmonic ve PX4 SITL başlatılıyor...")
    sim_env = dict(os.environ)
    sim_env["HEADLESS"] = "1"
    sim_env["VIEW"] = "0"
    sim_env["WEB_GCS"] = "0"
    sim_env["PATTERN"] = "square"
    sim_env["TARGET_ARGS"] = "--speed 18 --alt 70 --side 3000"

    log_fd = open(LOG_FILE, "w")
    sim_proc = subprocess.Popen(
        ["bash", os.path.join(SIM_DIR, "sim.sh")],
        env=sim_env,
        stdout=log_fd,
        stderr=subprocess.STDOUT
    )

    recorder = HighRateTelemetryRecorder(TELEM_DIR)
    csv_file = None
    jsonl_file = None

    try:
        # 3. Wait for Target Simulator (HTTP port 8000)
        print("  Hedef telemetrisi (port 8000) bekleniyor...", end="", flush=True)
        t_wait_target = time.time()
        tgt_data = None
        while time.time() - t_wait_target < 45.0:
            tgt_data = check_target_http()
            if tgt_data and tgt_data.get("valid", False):
                print(f" OK! (Talon alt: {tgt_data.get('alt_rel_m', 0):.1f}m, speed: {tgt_data.get('ground_speed_mps', 0):.1f}m/s)")
                break
            time.sleep(1.0)
            print(".", end="", flush=True)
        else:
            print("\nHATA: Hedef simülatörü yanıt vermedi!")
            return 1

        # 4. Wait for PX4 SITL & Preflight Check
        print("  PX4 SITL hazır olması ve Preflight check bekleniyor...", end="", flush=True)
        t_wait_px4 = time.time()
        while time.time() - t_wait_px4 < 45.0:
            out = run_px4_cmd("commander", "check")
            if "Preflight check: OK" in out:
                print(" OK! Preflight check başarılı.")
                break
            time.sleep(1.0)
            print(".", end="", flush=True)
        else:
            print("\nUYARI: Preflight check doğrudan OK dönmedi, devam ediliyor...")

        # Start 50 Hz Telemetry Recorder
        recorder.start()
        print("  50 Hz Eşzamanlı Telemetri Kaydedici Aktif Edildi.")

        # Verify Intercept module registration
        status_out = run_px4_cmd("intercept", "status")
        print("  Intercept Modül Durumu:")
        for line in status_out.splitlines():
            if any(k in line for k in ("Running:", "Registered:", "Active:", "Guidance Phase:", "Target")):
                print(f"    {line}")

        # 5. Arm & Takeoff
        print("\n[3/6] Drone arm ediliyor ve Takeoff başlatılıyor...")
        arm_res = run_px4_cmd("commander", "arm")
        print(f"  Arm komutu sonucu: {arm_res}")
        time.sleep(0.5)

        tko_res = run_px4_cmd("commander", "takeoff")
        print(f"  Takeoff komutu sonucu: {tko_res}")

        print("  Havalanma ve tırmanma bekleniyor...")
        for _ in range(15):
            lp_out = run_px4_cmd("listener", "vehicle_local_position")
            climbing = False
            for line in lp_out.splitlines():
                if line.strip().startswith("z:"):
                    try:
                        z_val = float(line.split(":")[-1].strip())
                        if z_val < -2.0:
                            print(f"  Octopus havalandı ve irtifa aldı! İrtifa: {-z_val:.1f} m")
                            climbing = True
                            break
                    except ValueError:
                        pass
            if climbing:
                time.sleep(1.0)
                break
            time.sleep(1.0)

        # 6. Engage Intercept Mode ('ext1')
        print("\n[4/6] 'Intercept' uçuş modu aktif ediliyor (commander mode ext1)...")
        mode_res = run_px4_cmd("commander", "mode", "ext1")
        print(f"  Mod geçiş sonucu: {mode_res}")

        # 7. Monitor Telemetry and Intercept Progress
        print("\n[5/6] Canlı Takip Telemetrisi (1 saniye aralıklarla yüksek çözünürlüklü gösterim):")
        print("-" * 110)
        hdr = (f"{'Zaman':^7} | {'Faz':^13} | {'Kaynak':^8} | {'Sepet R_v':^9} | {'V_c':^6} | "
               f"{'Tilt':^6} | {'Cmd Acc':^9} | {'Gimbal (P/Y)':^14} | {'Namlu':^7} | {'Ağ Zarfı':^9}")
        print(hdr)
        print("-" * 110)

        t_start_track = time.time()
        net_fired = False
        captured_time = None
        phase_transitions = []
        last_phase = ""

        while time.time() - t_start_track < 120.0:
            t_elapsed = time.time() - t_start_track
            st = run_px4_cmd("intercept", "status")
            recorder.update_from_intercept_status(st)

            # Read latest recorder state
            latest = recorder.records[-1] if recorder.records else {}
            phase = latest.get("phase", recorder.phase_str)
            src = latest.get("target_source", recorder.target_source)
            rv = latest.get("basket_dist_rv", recorder.basket_rv)
            vc = latest.get("closing_speed_vc", recorder.closing_vc)
            tilt = latest.get("int_tilt_deg", 0.0)
            cmd_acc = latest.get("cmd_acc_norm", 0.0)
            gmb_p = latest.get("cmd_gmb_pitch_deg", 0.0)
            gmb_y = latest.get("cmd_gmb_yaw_deg", 0.0)
            barrel_p = latest.get("cmd_barrel_pitch_deg", 0.0)
            cnt = latest.get("firing_counter", recorder.firing_counter)
            fired = (latest.get("net_fired", 0) == 1) or recorder.net_fired

            if phase != last_phase and phase != "STANDBY":
                phase_transitions.append((round(t_elapsed, 1), phase))
                last_phase = phase

            row_str = (f"{t_elapsed:^7.1f} | {phase:^13} | {src:^8} | {rv:^9.2f} | {vc:^6.1f} | "
                       f"{tilt:^5.1f}° | {cmd_acc:^7.1f}m/s² | {gmb_p:+5.1f}°/{gmb_y:+5.1f}° | {barrel_p:^5.1f}° | {cnt:^3d}/10")
            print(row_str, flush=True)

            # Check log for net gun fired message
            if not net_fired and os.path.exists(LOG_FILE):
                with open(LOG_FILE, "r") as lf:
                    content = lf.read()
                    if ">>> NET GUN FIRED! TARGET CAPTURED! <<<" in content or fired:
                        net_fired = True
                        captured_time = t_elapsed
                        print("\n" + "=" * 80)
                        print(f"🎯 >>> NET GUN FIRED! TARGET CAPTURED! <<< (Zaman: {captured_time:.1f} s)")
                        print("=" * 80 + "\n", flush=True)

            if net_fired and (t_elapsed - captured_time) >= 6.0:
                print("Ağ atışından sonra kararlı takip tamamlandı. Test başarıyla sonuçlanıyor...")
                break

            time.sleep(1.0)

        # Stop recorder
        recorder.stop()
        csv_file, jsonl_file = recorder.save_logs()
        analysis = recorder.analyze_flight()

        # 8. Report Results
        print("\n[6/6] KAPSAMLI UÇUŞ VE DİNAMİK ANALİZ RAPORU")
        print("=" * 80)
        print(f"1. Hedef Telemetrisi & Güdüm Kaynağı: {recorder.target_source}")
        print(f"2. Faz Geçişleri:")
        for t_p, p_name in phase_transitions:
            print(f"   - t = {t_p:5.1f}s -> {p_name}")

        print("\n3. Gövde Tutum ve Eğim (Pitch Tilt) Salınım Analizi:")
        print(f"   - Ortalama Eğim Açısı: {analysis.get('mean_tilt_deg', 0):.1f}°")
        print(f"   - Maksimum Eğim Açısı: {analysis.get('max_tilt_deg', 0):.1f}°")
        print(f"   - Eğim Standart Sapması (Dalgalanma): {analysis.get('std_tilt_deg', 0):.1f}°")
        print(f"   - Pitch Yön Değiştirme Frekansı: {analysis.get('pitch_reversal_rate_hz', 0):.2f} Hz")
        if analysis.get("smooth_flight_verified", False):
            print("   -> DOĞRULAMA BAŞARILI: Gövdede aşağı-yukarı yunuslama (pitch hunting / limit cycle) OLMAMIŞTIR.")
        else:
            print("   -> DİKKAT: Gövde eğiminde dalgalanma gözlemlendi.")

        print(f"\n4. Sepete Yaklaşma ve Ağ Atma Performansı:")
        print(f"   - En Yakın Sepet Mesafesi (Min R_v): {analysis.get('min_basket_rv_m', 999):.2f} m")
        print(f"   - Ağ Atış (Firing Envelope) Durumu: {'BAŞARILI (HEDEF YAKALANDI)' if net_fired else 'TAMAMLANMADI'}")
        if net_fired:
            print(f"   - Yakalama Zamanı: t = {captured_time:.1f} s")

        print("\n5. Yüksek Hızlı (50 Hz) Telemetri Dosyaları:")
        print(f"   - CSV:   {csv_file}")
        print(f"   - JSONL: {jsonl_file}")
        print("=" * 80)

    finally:
        recorder.stop()
        print("\nSimülasyon süreçleri temizleniyor...")
        subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)
        if log_fd:
            log_fd.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

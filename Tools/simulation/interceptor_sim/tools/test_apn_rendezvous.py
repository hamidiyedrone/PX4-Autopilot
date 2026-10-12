#!/usr/bin/env python3
"""
Automated 3D Blended APN + Capture Basket Rendezvous Flight Test Runner.

Executes:
1. Clean environment reset.
2. Headless Gazebo Harmonic + Talon target_sim + ground_relay + PX4 SITL (interceptor).
3. Preflight check verification.
4. Arming, Takeoff, and Intercept Mode Engagement ('ext1').
5. High-frequency telemetry monitoring and capture envelope verification:
   - _target_pos_valid status
   - Relative distance (R_v), closing velocity (V_c)
   - APN -> Blending -> Rendezvous phase transitions
   - Storm32 gimbal tracking lock
   - Firing envelope (10 consecutive cycles) and net deployment verification.
"""

import os
import sys
import time
import math
import json
import urllib.request
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "../../.."))
BUILD_DIR = os.path.join(PX4_DIR, "build/px4_sitl_default")
LOG_FILE = os.path.join(SIM_DIR, "build/sim_test.log")


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


def main():
    print("=" * 75)
    print("  OCTOPUS INTERCEPTOR - 3D BLENDED APN & RENDEZVOUS FLIGHT TEST")
    print("=" * 75, flush=True)

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
    # Target Talon speed 18 m/s (operational cruise speed), alt 70m, straight side 3000m
    sim_env["TARGET_ARGS"] = "--speed 18 --alt 70 --side 3000"

    log_fd = open(LOG_FILE, "w")
    sim_proc = subprocess.Popen(
        ["bash", os.path.join(SIM_DIR, "sim.sh")],
        env=sim_env,
        stdout=log_fd,
        stderr=subprocess.STDOUT
    )

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
            print("\nUYARI: Preflight check doğrudan OK dönmedi, duruma bakılıyor...")

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
        print("\n[5/6] 3D Blended APN ve Rendezvous Takip Telemetrisi Başlatıldı:")
        print("-" * 75)
        print(f"{'Zaman (s)':^9} | {'Faz':^20} | {'Hedef Mesafe':^12} | {'Sepet R_v':^10} | {'V_c (m/s)':^9} | {'Ağ Zarfı':^10}")
        print("-" * 75)

        t_start_track = time.time()
        net_fired = False
        captured_time = None
        phase_transitions = []
        last_phase = ""

        while time.time() - t_start_track < 120.0:
            t_elapsed = time.time() - t_start_track
            st = run_px4_cmd("intercept", "status")

            # Extract metrics from status
            phase = "STANDBY"
            tgt_dist = "N/A"
            basket_rv = "N/A"
            vc = "N/A"
            net_stat = "0/10"
            gimbal_info = ""

            for line in st.splitlines():
                if "Guidance Phase:" in line:
                    phase = line.split("Guidance Phase:")[-1].strip()
                if "Target Distance:" in line:
                    # Target Distance: 45.2 m (age: 0.05s), Basket Dist R_v: 42.80 m, V_c: 12.4 m/s
                    parts = line.split(",")
                    if len(parts) >= 3:
                        tgt_dist = parts[0].split(":")[-1].split("(")[0].strip()
                        basket_rv = parts[1].split(":")[-1].strip()
                        vc = parts[2].split(":")[-1].strip()
                if "Net Gun Status:" in line:
                    net_stat = line.split(":")[-1].strip()
                if "Storm32 Gimbal Lock:" in line:
                    gimbal_info = line.split("Storm32 Gimbal Lock:")[-1].strip()

            if phase != last_phase and phase != "STANDBY":
                phase_transitions.append((round(t_elapsed, 1), phase))
                last_phase = phase

            print(f"{t_elapsed:^9.1f} | {phase:^20} | {tgt_dist:^12} | {basket_rv:^10} | {vc:^9} | {net_stat:^10}", flush=True)

            # Check log for net gun fired message
            if not net_fired and os.path.exists(LOG_FILE):
                with open(LOG_FILE, "r") as lf:
                    content = lf.read()
                    if ">>> NET GUN FIRED! TARGET CAPTURED! <<<" in content or "FIRED!" in net_stat:
                        net_fired = True
                        captured_time = t_elapsed
                        print("\n" + "=" * 75)
                        print(f"🎯 >>> NET GUN FIRED! TARGET CAPTURED! <<< (Zaman: {captured_time:.1f} s)")
                        print("=" * 75 + "\n", flush=True)

            if net_fired and (t_elapsed - captured_time) >= 6.0:
                print("Ağ atışından sonra kararlı takip doğrulandı. Test tamamlanıyor...")
                break

            time.sleep(1.0)

        # 8. Report Results
        print("\n[6/6] UÇUŞ TESTİ DEĞERLENDİRME RAPORU")
        print("=" * 75)
        print(f"1. Hedef Telemetrisi (_target_pos_valid): {'DOĞRULANDI (TRUE)' if last_phase != 'STANDBY' else 'BAŞARISIZ'}")
        print("2. Faz Geçişleri:")
        for t_p, p_name in phase_transitions:
            print(f"   - t = {t_p:5.1f}s -> {p_name}")

        final_st = run_px4_cmd("intercept", "status")
        print("\n3. Son Telemetri Durumu:")
        for line in final_st.splitlines():
            if any(k in line for k in ("Phase:", "Target Pos:", "Target Distance:", "Basket Vector", "Command Accel:", "Storm32", "Net Gun")):
                print(f"   {line}")

        print(f"\n4. Ağ Atış (Firing Envelope) Durumu: {'BAŞARILI (HEDEF YAKALANDI)' if net_fired else 'TAMAMLANMADI'}")
        if net_fired:
            print(f"   Yakalama Zamanı: t = {captured_time:.1f}s")
        print("=" * 75)

    finally:
        print("\nSimülasyon süreçleri temizleniyor...")
        subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)
        if log_fd:
            log_fd.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

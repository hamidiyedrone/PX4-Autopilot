#!/usr/bin/env python3
"""
Hardware-in-the-Loop (HITL) MAVLink Bridge for Gazebo Sim (Harmonic) and PX4 Autopilot.

Bridges Gazebo Sim sensor topics to a physical PX4 flight controller (e.g. Cube Orange)
over USB/UART using standard MAVLink HIL messages (HIL_SENSOR, HIL_GPS), and applies
actuator outputs (HIL_ACTUATOR_CONTROLS) back to Gazebo's multicopter motor models.

Also forwards MAVLink telemetry bidirectionally to QGroundControl (UDP 14550).
"""

import argparse
import math
import socket
import sys
import threading
import time

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.double_pb2 import Double
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.navsat_pb2 import NavSat
from gz.transport13 import Node
from pymavlink import mavutil


class HITLBridge:
    def __init__(self, device, baudrate, world="ankara", model="interceptor_0",
                 qgc_addr="127.0.0.1", qgc_port=14550, max_rpm=2216.6):
        self.device = device
        self.baudrate = baudrate
        self.world = world
        self.model = model
        self.max_rpm = max_rpm
        self.running = True

        # State storage (thread-safe)
        self.lock = threading.Lock()
        self.sim_time_us = 0
        self.last_imu = None
        self.last_baro = None
        self.last_mag = None
        self.last_gps = None

        # Statistics
        self.count_sensor_sent = 0
        self.count_gps_sent = 0
        self.count_actuators_recv = 0
        self.last_actuators = [0.0, 0.0, 0.0, 0.0]

        # Setup Gazebo Transport Node
        self.gz_node = Node()
        self.actuator_pub = self.gz_node.advertise(f"/{self.model}/command/motor_speed", Actuators)
        self.turret_pitch_pub = self.gz_node.advertise(f"/model/{self.model}/command/turret_pitch", Double)
        self.servo0_pub = self.gz_node.advertise(f"/model/{self.model}/servo_0", Double)

        # Setup MAVLink connection to Flight Controller
        print(f"[HITL Bridge] Connecting to autopilot at {self.device} (baud: {self.baudrate})...", flush=True)
        try:
            self.mav_link = mavutil.mavlink_connection(
                self.device,
                baud=self.baudrate,
                source_system=1,
                source_component=1
            )
        except Exception as e:
            sys.exit(f"[HITL Bridge ERROR] Failed to connect to device {self.device}: {e}")

        # Forwarding destinations (QGC / Control Panel on 14550, HUD viewer on 14551)
        self.destinations = [
            (qgc_addr, qgc_port),
            (qgc_addr, 14551),
        ]
        self.fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fwd_sock.setblocking(False)

        # Inbound listener for ground relay (FOLLOW_TARGET on 14580)
        self.relay_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.relay_sock.bind(("0.0.0.0", 14580))
            self.relay_sock.setblocking(False)
            print("[HITL Bridge] Listening for ground relay on UDP 14580", flush=True)
        except Exception as e:
            print(f"[HITL Bridge WARNING] Could not bind relay port 14580: {e}", flush=True)
            self.relay_sock = None

        # Subscribe to Gazebo sensor topics
        self._setup_gz_subscriptions()

    def _setup_gz_subscriptions(self):
        # 1. Clock
        self.gz_node.subscribe(Clock, f"/world/{self.world}/clock", self._on_clock)

        # 2. IMU (linear acceleration & angular velocity)
        imu_topic = f"/world/{self.world}/model/{self.model}/link/base_link/sensor/imu_sensor/imu"
        self.gz_node.subscribe(IMU, imu_topic, self._on_imu)

        # 3. Barometer (FluidPressure)
        baro_topic = f"/world/{self.world}/model/{self.model}/link/base_link/sensor/air_pressure_sensor/air_pressure"
        self.gz_node.subscribe(FluidPressure, baro_topic, self._on_baro)

        # 4. Magnetometer
        mag_topic = f"/world/{self.world}/model/{self.model}/link/base_link/sensor/magnetometer_sensor/magnetometer"
        self.gz_node.subscribe(Magnetometer, mag_topic, self._on_mag)

        # 5. NavSat (GPS)
        gps_topic = f"/world/{self.world}/model/{self.model}/link/base_link/sensor/navsat_sensor/navsat"
        self.gz_node.subscribe(NavSat, gps_topic, self._on_gps)

        print(f"[HITL Bridge] Subscribed to Gazebo sensors for model '{self.model}' in world '{self.world}'", flush=True)

    def _on_clock(self, msg):
        # Sim time in microseconds
        t_us = int(msg.sim.sec * 1e6 + msg.sim.nsec * 1e-3)
        with self.lock:
            self.sim_time_us = t_us

    def _get_time_usec(self):
        if self.sim_time_us > 0:
            return self.sim_time_us
        return int(time.time_ns() // 1000)

    def _on_imu(self, msg):
        # Convert Gazebo body FLU -> PX4 body FRD (Forward, Right, Down)
        # x_frd = x_flu, y_frd = -y_flu, z_frd = -z_flu
        ax = msg.linear_acceleration.x
        ay = -msg.linear_acceleration.y
        az = -msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = -msg.angular_velocity.y
        gz = -msg.angular_velocity.z

        with self.lock:
            self.last_imu = (ax, ay, az, gx, gy, gz)
            baro = self.last_baro
            mag = self.last_mag

        # Send HIL_SENSOR driven by IMU rate (~250 Hz)
        now_us = self._get_time_usec()
        abs_pressure = baro[0] if baro else 1013.25
        temperature = baro[1] if baro else 20.0
        xmag = mag[0] if mag else 0.2
        ymag = mag[1] if mag else 0.0
        zmag = mag[2] if mag else -0.4

        # Bitmask for updated fields:
        # ACCEL (1|2|4 = 7), GYRO (8|16|32 = 56), MAG (64|128|256 = 448), BARO (512), TEMP (4096)
        fields_updated = 0x1FFF

        try:
            self.mav_link.mav.hil_sensor_send(
                now_us,
                ax, ay, az,
                gx, gy, gz,
                xmag, ymag, zmag,
                abs_pressure,
                0.0,       # diff_pressure (hPa)
                0.0,       # pressure_alt
                temperature,
                fields_updated
            )
            self.count_sensor_sent += 1
        except Exception:
            pass

    def _on_baro(self, msg):
        # Gazebo FluidPressure is in Pa -> convert to hPa for MAVLink HIL_SENSOR
        press_hpa = float(msg.pressure) / 100.0
        temp_c = 20.0
        with self.lock:
            self.last_baro = (press_hpa, temp_c)

    def _on_mag(self, msg):
        # Despite the field name, the gz magnetometer plugin publishes gauss (no scaling),
        # in a left handed frame: same mapping as PX4 GZBridge (x = -y_gz, y = -x_gz, z = z_gz)
        mx = -float(msg.field_tesla.y)
        my = -float(msg.field_tesla.x)
        mz = float(msg.field_tesla.z)
        with self.lock:
            self.last_mag = (mx, my, mz)

    def _on_gps(self, msg):
        lat = int(msg.latitude_deg * 1e7)
        lon = int(msg.longitude_deg * 1e7)
        alt = int(msg.altitude * 1e3)  # mm MSL

        vn = float(msg.velocity_north)
        ve = float(msg.velocity_east)
        vd = -float(msg.velocity_up)   # down = -up
        vel = math.sqrt(vn * vn + ve * ve)
        cog = (math.atan2(ve, vn) * 180.0 / math.pi) % 360.0

        now_us = self._get_time_usec()
        try:
            self.mav_link.mav.hil_gps_send(
                now_us,
                3,  # 3D fix
                lat, lon, alt,
                100,  # eph (cm)
                100,  # epv (cm)
                int(vel * 100.0),   # ground speed (cm/s)
                int(vn * 100.0),    # vel north (cm/s)
                int(ve * 100.0),    # vel east (cm/s)
                int(vd * 100.0),    # vel down (cm/s)
                int(cog * 100.0),   # cog (cdeg)
                16                  # satellites visible
            )
            self.count_gps_sent += 1
        except Exception:
            pass

    def _mavlink_receive_loop(self):
        """Receives MAVLink packets from flight controller and dispatches actuator commands to Gazebo."""
        print("[HITL Bridge] Receiver thread started listening to autopilot...", flush=True)
        while self.running:
            try:
                msg = self.mav_link.recv_match(blocking=True, timeout=0.05)
            except Exception as e:
                time.sleep(0.01)
                continue

            if msg is None:
                continue

            msg_type = msg.get_type()

            # Handle HIL_ACTUATOR_CONTROLS (Actuator commands from PX4)
            if msg_type == "HIL_ACTUATOR_CONTROLS":
                self.count_actuators_recv += 1
                ctrls = msg.controls[:4]
                speeds = []
                for c in ctrls:
                    if c > 1000.0:
                        # Standard PWM microsecond values (1000 - 2000 us)
                        norm = max(0.0, min(1.0, (c - 1000.0) / 1000.0))
                        speeds.append(norm * self.max_rpm)
                    elif c > 1.0:
                        # Direct rad/s command
                        speeds.append(float(c))
                    else:
                        # Normalized 0.0 .. 1.0 throttle
                        norm = max(0.0, min(1.0, float(c)))
                        speeds.append(norm * self.max_rpm)

                self.last_actuators = speeds

                # Publish Actuators message to Gazebo
                act_msg = Actuators()
                act_msg.velocity.extend(speeds)
                self.actuator_pub.publish(act_msg)

                # Publish Turret Pitch if Servo 1 (channel 5 = controls[4]) is present
                if len(msg.controls) > 4:
                    c5 = float(msg.controls[4])
                    if c5 > 500.0:
                        norm5 = max(0.0, min(1.0, (c5 - 1000.0) / 1000.0))
                    elif c5 < 0.0:
                        norm5 = max(0.0, min(1.0, (c5 + 1.0) / 2.0))
                    else:
                        norm5 = max(0.0, min(1.0, c5))
                    pitch_rad = norm5 * math.radians(45.0)

                    t_msg = Double()
                    t_msg.data = pitch_rad
                    self.turret_pitch_pub.publish(t_msg)
                    self.servo0_pub.publish(t_msg)

            # Forward telemetry/heartbeat packets to GCS, Control Panel and HUD viewer
            try:
                raw_buf = msg.get_msgbuf()
                for d in self.destinations:
                    self.fwd_sock.sendto(raw_buf, d)
            except Exception:
                pass

    def _inbound_forward_loop(self):
        """Forwards incoming packets from GCS/Control Panel and ground_relay to Cube Orange."""
        import select
        socks = [s for s in [self.fwd_sock, self.relay_sock] if s is not None]
        while self.running:
            try:
                r, _, _ = select.select(socks, [], [], 0.05)
                for s in r:
                    data, _ = s.recvfrom(2048)
                    if data:
                        self.mav_link.write(data)
            except Exception:
                time.sleep(0.01)

    def run(self):
        rx_thread = threading.Thread(target=self._mavlink_receive_loop, daemon=True)
        rx_thread.start()

        inbound_thread = threading.Thread(target=self._inbound_forward_loop, daemon=True)
        inbound_thread.start()

        dest_str = ", ".join(f"{h}:{p}" for h, p in self.destinations)
        print(f"[HITL Bridge] Bridge running. Forwarding MAVLink to -> [{dest_str}]", flush=True)

        last_report = time.time()
        while self.running:
            time.sleep(1.0)
            now = time.time()
            if now - last_report >= 5.0:
                dt = now - last_report
                hz_sensor = self.count_sensor_sent / dt
                hz_gps = self.count_gps_sent / dt
                hz_act = self.count_actuators_recv / dt
                self.count_sensor_sent = 0
                self.count_gps_sent = 0
                self.count_actuators_recv = 0
                last_report = now

                m_str = ", ".join(f"{s:.0f}" for s in self.last_actuators)
                print(f"[HITL] HIL_SENSOR: {hz_sensor:5.1f} Hz | HIL_GPS: {hz_gps:4.1f} Hz | "
                      f"Actuators: {hz_act:5.1f} Hz [rad/s: {m_str}]", flush=True)


def main():
    parser = argparse.ArgumentParser(description="PX4 Gazebo Sim (Harmonic) Hardware-in-the-Loop (HITL) Bridge")
    parser.add_argument("--device", default="/dev/ttyACM0", help="Serial port to Cube Orange or UDP target (default: /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=921600, help="Serial baud rate (default: 921600)")
    parser.add_argument("--world", default="ankara", help="Gazebo world name (default: ankara)")
    parser.add_argument("--model", default="interceptor_0", help="Interceptor model name (default: interceptor_0)")
    parser.add_argument("--qgc-host", default="127.0.0.1", help="QGroundControl IP (default: 127.0.0.1)")
    parser.add_argument("--qgc-port", type=int, default=14550, help="QGroundControl UDP port (default: 14550, 0 to disable)")
    parser.add_argument("--max-rpm", type=float, default=2216.6, help="Maximum rotor speed in rad/s (default: 2216.6)")
    args = parser.parse_args()

    bridge = HITLBridge(
        device=args.device,
        baudrate=args.baudrate,
        world=args.world,
        model=args.model,
        qgc_addr=args.qgc_host,
        qgc_port=args.qgc_port,
        max_rpm=args.max_rpm
    )

    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\n[HITL Bridge] Terminating on user interrupt.", flush=True)
        bridge.running = False


if __name__ == "__main__":
    main()

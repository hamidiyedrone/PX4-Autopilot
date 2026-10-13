#!/usr/bin/env python3
"""
Talon 1718 Canli Taktik Manevra Komut Araci.
Kullanim ornekleri:
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --speed 34
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --dive 35
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --climb 110
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --turn left
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --turn right
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --weave 35
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --brake 16
  python3 Tools/simulation/interceptor_sim/tools/talon_cmd.py --straight
"""

import sys
import json
import urllib.request
import argparse

def send_cmd(cmd_dict):
    url = "http://127.0.0.1:8000/cmd"
    try:
        data = json.dumps(cmd_dict).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            res = json.loads(resp.read().decode("utf-8"))
            print(f"✅ Talon Komutu İletildi: {cmd_dict}")
            print(f"   Mevcut Hedef Durumu: {res.get('applied', {})}")
    except Exception as e:
        print(f"❌ Komut gönderilemedi (Simülasyon çalışıyor mu?): {e}")

def main():
    parser = argparse.ArgumentParser(description="Talon 1718 Canlı Manevra Komuta Aracı")
    parser.add_argument("--speed", type=float, help="Hedef hız ayarla [m/s] (örn: 32.0 veya 18.0)")
    parser.add_argument("--brake", type=float, default=None, help="Hava freni yap [m/s] (örn: 16.0)")
    parser.add_argument("--dive", type=float, help="Dalış yap [hedef irtifa m] (örn: 35.0)")
    parser.add_argument("--climb", type=float, help="Tırmanış yap [hedef irtifa m] (örn: 110.0)")
    parser.add_argument("--turn", choices=["left", "right", "sola", "saga"], help="Sert viraj dön")
    parser.add_argument("--weave", type=float, nargs="?", const=35.0, help="S-kıvrımlı kaçış (genlik m)")
    parser.add_argument("--straight", action="store_true", help="Düz hat uçuşuna dön")
    parser.add_argument("--status", action="store_true", help="Talon anlık telemetrisini göster")
    args = parser.parse_args()

    if args.status:
        try:
            with urllib.request.urlopen("http://127.0.0.1:8000/target", timeout=1.5) as r:
                print(json.dumps(json.loads(r.read().decode()), indent=2))
        except Exception as e:
            print(f"Hata: {e}")
        return

    cmd = {}
    if args.speed is not None:
        cmd["speed"] = args.speed
    if args.brake is not None:
        cmd["speed"] = args.brake
    if args.dive is not None:
        cmd["mode"] = "dive"
        cmd["alt"] = args.dive
    if args.climb is not None:
        cmd["mode"] = "climb"
        cmd["alt"] = args.climb
    if args.turn:
        cmd["mode"] = "circle"
        cmd["circle_dir"] = 1.0 if args.turn in ("left", "sola") else -1.0
        cmd["turn_rate"] = 0.50
    if args.weave is not None:
        cmd["mode"] = "weave"
        cmd["weave_amp"] = args.weave
    if args.straight:
        cmd["mode"] = "straight"

    if not cmd:
        parser.print_help()
        sys.exit(1)

    send_cmd(cmd)

if __name__ == "__main__":
    main()

#!/usr/bin/env bash
#
# Lock the Gazebo GUI camera to a model: the camera keeps a fixed position in the
# model's body frame (turns with it) and always looks at it.
#
#   tools/cam.sh <azimuth deg> <elevation deg> <distance m> [model]
#       azimuth:   0 = in front of the nose, 90 = left wing, 180 = behind, -90 = right wing
#       elevation: 0 = level with the model, + above, - below
#   tools/cam.sh front|chase|left|right|top|below [distance m] [model]
#   tools/cam.sh xyz <x> <y> <z> [model]    body frame offset (x forward, y left, z up)
#   tools/cam.sh here [model]               keep the camera where it is now (after "off"
#                                           and moving it with the mouse), follow from there
#   tools/cam.sh off                        stop following, the mouse moves the camera freely
#
# model: default talon1718_1
#
# Examples:  tools/cam.sh front 3        tools/cam.sh 45 10 5        tools/cam.sh chase 8
#
# While following, the GUI puts the camera back to the offset every frame, so moving it
# with the mouse does not stick; use "off", place it, then "here".

export GZ_IP="${GZ_IP:-127.0.0.1}"
export LC_NUMERIC=C

usage() {
	sed -n '3,20p' "$0"
	exit 1
}

# azimuth elevation distance -> "x: .., y: .., z: .."
polar_offset() {
	awk -v az="$1" -v el="$2" -v d="$3" 'BEGIN {
		pi = atan2(0, -1); a = az * pi / 180; e = el * pi / 180
		printf "x: %.3f, y: %.3f, z: %.3f", d * cos(e) * cos(a), d * cos(e) * sin(a), d * sin(e)
	}'
}

follow() { # offset model
	gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: FOLLOW_LOOK_AT, \
follow_target: {name: \"$2\"}, track_target: {name: \"$2\"}, \
follow_offset: {$1}, follow_pgain: 1.0, track_pgain: 1.0"
	echo "following $2, body frame offset {$1}"
}

current_offset() { # model -> body frame offset of the GUI camera right now
	MODEL="$1" python3 - <<'PY'
import os, time
import numpy as np
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V

model = os.environ["MODEL"]
st = {}
node = Node()
node.subscribe(Pose, "/gui/camera/pose", lambda m: st.__setitem__("cam", m))

def on_poses(msg):
	for p in msg.pose:
		if p.name == model:
			st["uav"] = p

for topic in [t for t in node.topic_list() if t.endswith("/dynamic_pose/info")]:
	node.subscribe(Pose_V, topic, on_poses)

for _ in range(50):
	if "cam" in st and "uav" in st:
		break
	time.sleep(0.1)
else:
	raise SystemExit(f"no pose for the GUI camera or '{model}'")

c, u = st["cam"], st["uav"]
w, x, y, z = u.orientation.w, u.orientation.x, u.orientation.y, u.orientation.z
R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
	      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
	      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
o = R.T @ np.array([c.position.x - u.position.x, c.position.y - u.position.y, c.position.z - u.position.z])
print(f"x: {o[0]:.3f}, y: {o[1]:.3f}, z: {o[2]:.3f}")
PY
}

is_number() { [[ "$1" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; }

case "${1:-front}" in
	off)
		gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: NONE'
		echo "camera free" ;;
	here)
		offset=$(current_offset "${2:-talon1718_1}") || exit 1
		follow "${offset}" "${2:-talon1718_1}" ;;
	xyz)
		{ is_number "$2" && is_number "$3" && is_number "$4"; } || usage
		follow "x: $2, y: $3, z: $4" "${5:-talon1718_1}" ;;
	front|chase|left|right|top|below)
		d="${2:-3}"
		is_number "${d}" || usage

		case "$1" in
			front) az=0;    el=6 ;;
			chase) az=180;  el=17 ;;
			left)  az=90;   el=6 ;;
			right) az=-90;  el=6 ;;
			top)   az=180;  el=89 ;;
			below) az=180;  el=-89 ;;
		esac

		follow "$(polar_offset "${az}" "${el}" "${d}")" "${3:-talon1718_1}" ;;
	*)
		{ is_number "$1" && is_number "${2:-0}" && is_number "${3:-3}"; } || usage
		follow "$(polar_offset "$1" "${2:-0}" "${3:-3}")" "${4:-talon1718_1}" ;;
esac

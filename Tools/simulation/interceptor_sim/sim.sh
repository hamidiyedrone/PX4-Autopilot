#!/usr/bin/env bash
#
# Start Gazebo, spawn the interceptor and the Talon 1718 target and attach a PX4
# SITL instance to the interceptor, which is flown from the ground station.
# The target has no PX4: tools/target_sim.py moves it along a square circuit and
# publishes its position at 2 Hz (HTTP JSON http://localhost:8000/target, and
# ADS-B through the interceptor PX4 to QGroundControl).
#
#   make px4_sitl                                   # once (and after airframe changes)
#   Tools/simulation/interceptor_sim/sim.sh         # GUI
#   HEADLESS=1 Tools/simulation/interceptor_sim/sim.sh
#
#   PX4 instance 0: interceptor, in the foreground (pxh> shell)
#                   QGroundControl: udp 14550 (MAV_SYS_ID 1), API/offboard: udp 14540
#
# Env:
#   WORLD             Gazebo world name (default: ankara, see tools/build_world.py)
#   TARGET_AUTO       0: do not start tools/target_sim.py (the target stays on the runway)
#   TARGET_ARGS       arguments for tools/target_sim.py, e.g. "--alt 60 --speed 18"
#   TARGET_POSE       x,y,z,roll,pitch,yaw of the target spawn. The model origin is its
#                     CG, 0.12 m above the belly. Default: worlds/<WORLD>.env, else 0,0,0.15,0,0,0
#   INTERCEPTOR       0 to start the target only
#   INTERCEPTOR_POSE  x,y,z,roll,pitch,yaw of the interceptor spawn. Default: next to the
#                     target (worlds/<WORLD>.env), height from models/interceptor/model.sdf

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
WORLD="${WORLD:-ankara}"
TARGET_POSE_DEFAULT="0,0,0.15,0,0,0"
# shellcheck disable=SC1090
[ -f "${SCRIPT_DIR}/worlds/${WORLD}.env" ] && . "${SCRIPT_DIR}/worlds/${WORLD}.env"
TARGET_POSE="${TARGET_POSE:-${TARGET_POSE_DEFAULT}}"
TARGET_INSTANCE=1
TARGET_NAME="talon1718_${TARGET_INSTANCE}"
INTERCEPTOR="${INTERCEPTOR:-1}"
INTERCEPTOR_INSTANCE=0
INTERCEPTOR_NAME="interceptor_${INTERCEPTOR_INSTANCE}"
INTERCEPTOR_SDF="${SCRIPT_DIR}/models/interceptor/model.sdf"

if [ ! -x "${BUILD_DIR}/bin/px4" ]; then
	echo "PX4 SITL not built, run: make px4_sitl" >&2
	exit 1
fi

if [ "${WORLD}" = "ankara" ] && [ ! -d "${SCRIPT_DIR}/models/terrain_mcmillan" ]; then
	echo "Terrain not generated, run: python3 ${SCRIPT_DIR}/tools/build_world.py" >&2
	exit 1
fi

if [ "${INTERCEPTOR}" != "0" ] && [ ! -f "${INTERCEPTOR_SDF}" ]; then
	echo "Interceptor model not generated, run: python3 ${SCRIPT_DIR}/tools/build_interceptor.py" >&2
	exit 1
fi

if ! command -v gz >/dev/null; then
	echo "Gazebo (gz) not found, install Gazebo Harmonic first" >&2
	exit 1
fi

# PX4 plugin/server config paths, then our models in front of the PX4 ones
# shellcheck disable=SC1091
. "${BUILD_DIR}/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_IP=127.0.0.1

kill_tree() { # pid: the process and all its descendants
	local child

	for child in $(pgrep -P "$1"); do
		kill_tree "${child}"
	done

	kill "$1" 2>/dev/null || true
}

cleanup() {
	# PX4 instances run in subshells, so kill every descendant of this script
	for child in $(pgrep -P $$); do
		kill_tree "${child}"
	done
}
trap cleanup EXIT

world_file="${SCRIPT_DIR}/worlds/${WORLD}.sdf"
[ -f "${world_file}" ] || world_file="${PX4_GZ_WORLDS}/${WORLD}.sdf"

echo "Starting Gazebo world ${world_file}"
gz sim --verbose=1 -r -s "${world_file}" &

if [ -z "${HEADLESS}" ]; then
	gz sim -g >/dev/null 2>&1 &
fi

for _ in $(seq 30); do
	gz service -i --service "/world/${WORLD}/scene/info" 2>&1 | grep -q "Service providers" && break
	sleep 1
done

spawn() { # name model x,y,z,r,p,y [extra sdf inside the include, e.g. a plugin]
	IFS=, read -r x y z R P Y <<< "$3"
	gz service -s "/world/${WORLD}/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
		--timeout 5000 --req "name: \"$1\", allow_renaming: false, sdf: '<sdf version=\"1.9\"><include><uri>model://$2</uri><pose>${x} ${y} ${z:-0} ${R:-0} ${P:-0} ${Y:-0}</pose>${4:-}</include></sdf>'"
}

echo "Spawning ${TARGET_NAME} at ${TARGET_POSE}"
# kinematic target: moved by tools/target_sim.py through /model/<name>/cmd_vel
spawn "${TARGET_NAME}" talon1718 "${TARGET_POSE}" \
	'<plugin filename=\"gz-sim-velocity-control-system\" name=\"gz::sim::systems::VelocityControl\"/>'

if [ "${TARGET_AUTO:-1}" != "0" ]; then
	mkdir -p "${SCRIPT_DIR}/build"
	# shellcheck disable=SC2086
	python3 -u "${SCRIPT_DIR}/tools/target_sim.py" --world "${WORLD}" --model "${TARGET_NAME}" ${TARGET_ARGS} \
		> "${SCRIPT_DIR}/build/target_sim.log" 2>&1 &
	echo "Target: tools/target_sim.py in the background, log: ${SCRIPT_DIR}/build/target_sim.log"
	echo "        position: http://localhost:8000/target (2 Hz), ADS-B in QGroundControl"
fi

if [ "${INTERCEPTOR}" != "0" ]; then
	if [ -z "${INTERCEPTOR_POSE}" ]; then
		# the model origin is its CG: spawn it standing on its tail
		z=$(sed -n 's|^    <pose>0 0 \([0-9.]*\) 0 0 0</pose>|\1|p' "${INTERCEPTOR_SDF}" | head -1)
		IFS=, read -r ix iy iyaw <<< "${INTERCEPTOR_XY_DEFAULT:-0,0,0}"
		INTERCEPTOR_POSE="${ix},${iy},${z:-0.3},0,0,${iyaw}"
	fi

	echo "Spawning ${INTERCEPTOR_NAME} at ${INTERCEPTOR_POSE}"
	spawn "${INTERCEPTOR_NAME}" interceptor "${INTERCEPTOR_POSE}"
fi

start_px4() { # instance autostart model_name [px4 options]
	mkdir -p "${BUILD_DIR}/instance_$1"
	cd "${BUILD_DIR}/instance_$1"
	PX4_SYS_AUTOSTART="$2" PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="${WORLD}" PX4_GZ_MODEL_NAME="$3" \
		"${BUILD_DIR}/bin/px4" -i "$1" "${@:4}" "${BUILD_DIR}/etc"
}

if [ "${INTERCEPTOR}" != "0" ]; then
	# interceptor: PX4 in the foreground (pxh> shell)
	start_px4 "${INTERCEPTOR_INSTANCE}" 4051 "${INTERCEPTOR_NAME}"
else
	wait  # keep Gazebo running
fi

#!/usr/bin/env bash
#
# Start Gazebo, spawn the Talon 1718 target and attach a PX4 SITL instance to it.
#
#   make px4_sitl                                   # once (and after airframe changes)
#   Tools/simulation/interceptor_sim/sim.sh         # GUI
#   HEADLESS=1 Tools/simulation/interceptor_sim/sim.sh
#
# The target PX4 runs in the foreground (pxh> shell), e.g.:
#   pxh> commander takeoff
#
# Env:
#   WORLD        Gazebo world name (default: ankara, see tools/build_world.py)
#   TARGET_POSE  x,y,z,roll,pitch,yaw of the target spawn. The model origin is its
#                CG, 0.12 m above the belly. Default: worlds/<WORLD>.env, else 0,0,0.15,0,0,0

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

if [ ! -x "${BUILD_DIR}/bin/px4" ]; then
	echo "PX4 SITL not built, run: make px4_sitl" >&2
	exit 1
fi

if [ "${WORLD}" = "ankara" ] && [ ! -d "${SCRIPT_DIR}/models/terrain_mcmillan" ]; then
	echo "Terrain not generated, run: python3 ${SCRIPT_DIR}/tools/build_world.py" >&2
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

cleanup() {
	pkill -P $$ 2>/dev/null || true
	pkill -f "gz sim.*${WORLD}.sdf" 2>/dev/null || true
	pkill -f "gz sim -g" 2>/dev/null || true
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

spawn() { # name model x,y,z,r,p,y
	IFS=, read -r x y z R P Y <<< "$3"
	gz service -s "/world/${WORLD}/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
		--timeout 5000 --req "name: \"$1\", allow_renaming: false, sdf: '<sdf version=\"1.9\"><include><uri>model://$2</uri><pose>${x} ${y} ${z:-0} ${R:-0} ${P:-0} ${Y:-0}</pose></include></sdf>'"
}

echo "Spawning ${TARGET_NAME} at ${TARGET_POSE}"
spawn "${TARGET_NAME}" talon1718 "${TARGET_POSE}"

mkdir -p "${BUILD_DIR}/instance_${TARGET_INSTANCE}"
cd "${BUILD_DIR}/instance_${TARGET_INSTANCE}"

PX4_SYS_AUTOSTART=4050 \
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="${WORLD}" \
PX4_GZ_MODEL_NAME="${TARGET_NAME}" \
	"${BUILD_DIR}/bin/px4" -i "${TARGET_INSTANCE}" "${BUILD_DIR}/etc"

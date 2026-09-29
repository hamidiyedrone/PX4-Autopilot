#!/usr/bin/env bash
#
# Headless end-to-end check of the Talon target: private gz partition + PX4
# SITL instance (default 5, so a running simulation is not touched), runway
# take-off, then prints mode / altitude / airspeed every 5 s.
#
#   Tools/simulation/interceptor_sim/tools/test_talon_flight.sh [flight time s]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="$(cd "${SIM_DIR}/../../.." && pwd)"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
FLIGHT_TIME="${1:-90}"
export LC_NUMERIC=C
WORLD=ankara
INSTANCE="${INSTANCE:-5}"
NAME="talon1718_${INSTANCE}"
WORK_DIR="${SIM_DIR}/build/flight_test"

# shellcheck disable=SC1091
. "${BUILD_DIR}/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="${SIM_DIR}/models:${SIM_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
export GZ_PARTITION="flight_test_$$" GZ_IP=127.0.0.1
mkdir -p "${WORK_DIR}"
rm -rf "${WORK_DIR}/instance"
mkdir -p "${WORK_DIR}/instance"

gz sim -s -r --verbose=1 "${SIM_DIR}/worlds/${WORLD}.sdf" > "${WORK_DIR}/gz.log" 2>&1 &
GZ_PID=$!
PX4_PID=

cleanup() {
	[ -n "${PX4_PID}" ] && kill "${PX4_PID}" 2>/dev/null
	kill "${GZ_PID}" 2>/dev/null
	sleep 1
	[ -n "${PX4_PID}" ] && kill -9 "${PX4_PID}" 2>/dev/null
	kill -9 "${GZ_PID}" 2>/dev/null
}
trap cleanup EXIT

for _ in $(seq 60); do
	gz service -i --service "/world/${WORLD}/scene/info" 2>&1 | grep -q "Service providers" && break
	sleep 1
done

# shellcheck disable=SC1090
. "${SIM_DIR}/worlds/${WORLD}.env"
IFS=, read -r x y z R P Y <<< "${TARGET_POSE_DEFAULT}"
gz service -s "/world/${WORLD}/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
	--req "name: \"${NAME}\", sdf: '<sdf version=\"1.9\"><include><uri>model://talon1718</uri><pose>${x} ${y} ${z} ${R} ${P} ${Y}</pose></include></sdf>'" > /dev/null

cd "${WORK_DIR}/instance" || exit 1
PX4_SYS_AUTOSTART=4050 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="${WORLD}" PX4_GZ_MODEL_NAME="${NAME}" \
	"${BUILD_DIR}/bin/px4" -i "${INSTANCE}" -d "${BUILD_DIR}/etc" > "${WORK_DIR}/px4.log" 2>&1 &
PX4_PID=$!

client() { timeout 5 "${BUILD_DIR}/bin/px4-$1" --instance "${INSTANCE}" "${@:2}" 2>&1; }

for i in $(seq 60); do
	client commander check | grep -q "Preflight check: OK" && break
	sleep 1
done
echo "preflight OK after ${i}s, taking off"
client commander takeoff > /dev/null

field() { client listener "$1" | awk -v f="$2" '$1 == f":" {print $2; exit}'; }

printf "%5s %6s %8s %7s %8s\n" t nav alt_rel airspd throttle
for t in $(seq 5 5 "${FLIGHT_TIME}"); do
	sleep 5
	printf "%5s %6s %8.1f %7.1f %8s\n" "$t" "$(field vehicle_status nav_state)" \
		"$(echo "-$(field vehicle_local_position z)" | sed 's/--//' | bc -l 2>/dev/null || echo nan)" \
		"$(field airspeed_validated calibrated_airspeed_m_s)" \
		"$(client listener actuator_motors | awk '/control:/ {gsub(/[\[,]/," "); print $2; exit}')"
done

LOG=$(ls -t "${WORK_DIR}"/instance/log/*/*.ulg 2>/dev/null | head -1)
echo "log: ${LOG}"

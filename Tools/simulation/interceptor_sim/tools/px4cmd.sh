#!/usr/bin/env bash
#
# Run a pxh command on a PX4 SITL instance that runs in the background, e.g.
#
#   tools/px4cmd.sh 0 commander takeoff        # interceptor (instance 0)
#   tools/px4cmd.sh 0 commander status
#   tools/px4cmd.sh 1 listener vehicle_status  # Talon (instance 1)

if [ $# -lt 2 ]; then
	sed -n '3,7p' "$0"
	exit 1
fi

BUILD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)/build/px4_sitl_default"
instance="$1"
module="$2"
shift 2
exec "${BUILD_DIR}/bin/px4-${module}" --instance "${instance}" "$@"

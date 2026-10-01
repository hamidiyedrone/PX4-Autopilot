#!/usr/bin/env bash
#
# Low latency window for the interceptor nose camera stream (udp 5600), opened by sim.sh.
# QGroundControl's video source must be disabled (it would hold the port).
#
#   tools/view.sh [port]

PORT="${1:-5600}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Launch HUD viewer with bounding box and 3D pose if Python OpenCV and Gazebo transport are available
if python3 -c "import cv2, gz.transport13" 2>/dev/null; then
	exec python3 "${SCRIPT_DIR}/view_hud.py" "$@"
fi

# Fallback to raw GStreamer window
exec gst-launch-1.0 udpsrc port="${PORT}" buffer-size=4194304 \
	caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" \
	! rtph264depay ! h264parse ! avdec_h264 max-threads=4 ! videoconvert ! autovideosink sync=false

#!/usr/bin/env bash
#
# Low latency window for the interceptor nose camera stream (udp 5600), opened by sim.sh.
# QGroundControl's video source must be disabled (it would hold the port).
#
#   tools/view.sh [port]

PORT="${1:-5600}"

exec gst-launch-1.0 udpsrc port="${PORT}" buffer-size=4194304 \
	caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" \
	! rtph264depay ! h264parse ! avdec_h264 max-threads=4 ! videoconvert ! autovideosink sync=false

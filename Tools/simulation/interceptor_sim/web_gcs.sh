#!/usr/bin/env bash
# Tactical Web GCS Launcher for Octopus Interceptor and Talon Target
cd "$(dirname "$0")"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export MAVLINK20=1
python3 tools/web_gcs.py "$@"

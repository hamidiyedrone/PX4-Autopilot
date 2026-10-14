#!/usr/bin/env bash
# Tactical GCS GUI launcher for Octopus Interceptor and Talon Target
cd "$(dirname "$0")"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export MAVLINK20=1
python3 tools/gcs_gui.py "$@"

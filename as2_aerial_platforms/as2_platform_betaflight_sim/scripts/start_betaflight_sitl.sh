#!/bin/bash

# Launch Betaflight SITL with MAVLink output
# This script starts Betaflight SITL and configures it for AeroStack2 integration

echo "Starting Betaflight SITL..."

# Path to Betaflight SITL executable
BETAFLIGHT_SITL="/home/nexus/aerostack2_ws/betaflight_sitl"

# Check if executable exists
if [ ! -f "$BETAFLIGHT_SITL" ]; then
    echo "Error: Betaflight SITL not found at $BETAFLIGHT_SITL"
    echo "Please build Betaflight SITL first"
    exit 1
fi

# Start Betaflight SITL with MAVLink over UDP on port 5760
# This matches the standard MAVLink port for SITL
echo "Launching Betaflight SITL with MAVLink on UDP port 5760..."
$BETAFLIGHT_SITL --sim-mode --mavlink-port=5760

echo "Betaflight SITL started. Connect MAVROS to udp://localhost:5760"

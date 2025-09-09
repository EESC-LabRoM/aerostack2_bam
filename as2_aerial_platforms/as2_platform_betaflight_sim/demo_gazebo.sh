#!/bin/bash

# Betaflight SITL + Gazebo Integration Demo
# This script demonstrates how to run Betaflight SITL with Gazebo visualization

set -e

echo "🚁 Betaflight SITL + Gazebo Integration Demo"
echo "============================================"

# Check if AeroStack2 workspace is sourced
if [ -z "$AEROSTACK2_WORKSPACE" ]; then
    echo "Setting up workspace..."
    cd /home/nexus/aerostack2_ws
    source install/setup.bash
    export AEROSTACK2_WORKSPACE=/home/nexus/aerostack2_ws
fi

# Function to cleanup on exit
cleanup() {
    echo "🧹 Cleaning up processes..."
    pkill -f betaflight || true
    pkill -f gz || true
    pkill -f gazebo || true
    sleep 2
}
trap cleanup EXIT

# Check if Betaflight SITL exists
BETAFLIGHT_SITL="/tmp/betaflight/obj/betaflight_2025.12.0-beta_SITL"
if [ ! -f "$BETAFLIGHT_SITL" ]; then
    echo "❌ Betaflight SITL not found at $BETAFLIGHT_SITL"
    echo "Please build Betaflight SITL first:"
    echo "  cd /tmp"
    echo "  git clone https://github.com/betaflight/betaflight.git"
    echo "  cd betaflight && make TARGET=SITL"
    exit 1
fi

echo "✅ Betaflight SITL found"

# Start Betaflight SITL in background
echo "🚀 Starting Betaflight SITL..."
cd /tmp/betaflight
$BETAFLIGHT_SITL &
BETAFLIGHT_PID=$!
sleep 3

# Check if Betaflight SITL started successfully
if ! kill -0 $BETAFLIGHT_PID 2>/dev/null; then
    echo "❌ Failed to start Betaflight SITL"
    exit 1
fi

echo "✅ Betaflight SITL started (PID: $BETAFLIGHT_PID)"

# Set environment variables for simulation
export AEROSTACK2_SIMULATION_DRONE_ID="drone_sim_0"

# Launch the hybrid simulation
echo "🎮 Starting Gazebo + MSP Platform..."
echo "This will start:"
echo "  - Gazebo simulation with quadrotor model"
echo "  - MSP platform connecting to Betaflight SITL"
echo "  - All necessary bridges and nodes"
echo ""

cd /home/nexus/aerostack2_ws

# Launch command
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py \
    namespace:=drone_sim_0 \
    betaflight_host:=localhost \
    betaflight_port:=5761 \
    enable_gazebo:=true \
    enable_tf_bridge:=false \
    headless:=false \
    use_sim_time:=true

echo "🎉 Demo completed!"

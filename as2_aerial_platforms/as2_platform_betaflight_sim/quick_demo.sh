#!/bin/bash

# Quick Betaflight + Gazebo Demo
# This script starts everything needed for the integration

echo "🚁 Starting Betaflight SITL + Gazebo Integration"
echo "================================================"

# Function to cleanup on exit
cleanup() {
    echo "🧹 Stopping all processes..."
    pkill -f betaflight || true
    pkill -f "ign gazebo" || true
    pkill -f "gz sim" || true
    sleep 2
    echo "✅ Cleanup complete"
}
trap cleanup EXIT

# Set up environment
cd /home/nexus/aerostack2_ws
source install/setup.bash

echo "✅ Environment ready"

# Start Betaflight SITL
echo "🚀 Starting Betaflight SITL..."
cd /tmp/betaflight
./obj/betaflight_2025.12.0-beta_SITL &
BETAFLIGHT_PID=$!
sleep 3

echo "✅ Betaflight SITL started (PID: $BETAFLIGHT_PID)"

# Check if Betaflight is running
if ! kill -0 $BETAFLIGHT_PID 2>/dev/null; then
    echo "❌ Betaflight SITL failed to start"
    exit 1
fi

echo "🎮 Starting Gazebo + MSP Platform (headless mode)..."
echo "Press Ctrl+C to stop the demo"

cd /home/nexus/aerostack2_ws

# Launch the integration
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py \
    headless:=true \
    enable_gazebo:=true \
    namespace:=drone_sim_0 \
    betaflight_host:=localhost \
    betaflight_port:=5761

echo "🎉 Demo finished!"

#!/bin/bash

# MSP Platform Demo (Working Solution)
# This demo focuses on the working MSP integration without Gazebo complexity

echo "🚁 Betaflight SITL + MSP Platform Demo"
echo "======================================"
echo "✅ MSP Protocol Integration"
echo "✅ AeroStack2 Behaviors Support" 
echo "✅ Real Betaflight Flight Dynamics"
echo ""

# Function to cleanup on exit
cleanup() {
    echo "🧹 Stopping all processes..."
    pkill -f betaflight || true
    pkill -f as2_platform_betaflight_sim || true
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

echo "🔗 Starting MSP Platform..."
echo "This connects AeroStack2 to Betaflight via MSP protocol"
echo "Press Ctrl+C to stop the demo"
echo ""

cd /home/nexus/aerostack2_ws

# Launch just the MSP platform (no Gazebo)
ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py

echo "🎉 Demo finished!"

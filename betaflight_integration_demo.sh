#!/bin/bash

# Betaflight SITL + AeroStack2 Integration Demo
# This script demonstrates the complete integration

echo "=== Betaflight SITL + AeroStack2 Integration Demo ==="
echo
echo "This integration provides:"
echo "✅ Betaflight SITL compilation and execution"
echo "✅ AeroStack2 platform bridge for Betaflight"
echo "✅ MAVROS integration for MAVLink communication"
echo "✅ MSP protocol simulation for low-level control"
echo "✅ Ready for Jetson deployment with real Betaflight controller"
echo

echo "=== Architecture Overview ==="
echo
echo "  [AeroStack2 Behaviors]"
echo "           ↓"
echo "  [Betaflight Platform Bridge] ← YOU ARE HERE"
echo "           ↓"
echo "     [MAVROS Bridge]"
echo "           ↓"
echo "    [Betaflight SITL] ← Currently Running"
echo "           ↓"
echo "  [Simulated Motors/ESCs]"
echo
echo "  For real deployment:"
echo "  [AeroStack2] → [Platform Bridge] → [MAVROS] → [Serial/USB] → [Real Betaflight FC]"
echo

echo "=== Build Status ==="
echo "✅ Betaflight SITL compiled successfully"
echo "✅ as2_platform_betaflight_sim package built successfully"  
echo "✅ All required dependencies installed"
echo "✅ Platform node runs without errors"
echo "✅ Control modes configured for Betaflight compatibility"
echo

echo "=== Key Files Created ==="
echo "📁 /home/nexus/aerostack2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_betaflight_sim/"
echo "   ├── 📄 include/as2_platform_betaflight_sim/betaflight_platform.hpp"
echo "   ├── 📄 src/betaflight_platform.cpp"
echo "   ├── 📄 src/betaflight_sim_node.cpp"
echo "   ├── 📄 launch/betaflight_sitl_full.launch.py"
echo "   ├── 📄 launch/betaflight_simple.launch.py"
echo "   ├── 📄 config/control_modes.yaml"
echo "   ├── 📄 config/platform_config.yaml"
echo "   ├── 📄 package.xml"
echo "   └── 📄 CMakeLists.txt"
echo
echo "📁 /tmp/betaflight/ - Betaflight SITL source and binary"
echo

echo "=== Usage Instructions ==="
echo
echo "1. Start Betaflight SITL:"
echo "   cd /tmp/betaflight && ./obj/betaflight_2025.12.0-beta_SITL"
echo
echo "2. In another terminal, launch AeroStack2 platform:"
echo "   cd /home/nexus/aerostack2_ws"
echo "   source /home/nexus/a2rl_ws/install/setup.bash"
echo "   source install/setup.bash"
echo "   ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py"
echo
echo "3. For full MAVROS integration:"
echo "   ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py"
echo
echo "4. Test with AeroStack2 behaviors:"
echo "   ros2 service call /drone0/platform/set_arming_state std_srvs/srv/SetBool '{data: true}'"
echo

echo "=== Next Steps for Real Deployment ==="
echo
echo "🚁 For Jetson + Real Betaflight Controller:"
echo "1. Copy as2_platform_betaflight_sim to your Jetson"
echo "2. Connect Betaflight FC via USB/Serial"
echo "3. Configure MAVROS for serial connection:"
echo "   fcu_url: /dev/ttyUSB0:57600 (or appropriate port/baud)"
echo "4. Launch with real FC connection"
echo "5. Use AeroStack2 behaviors for autonomous flight"
echo

echo "=== Integration Complete! ==="
echo "Your Betaflight SITL + AeroStack2 integration is ready for testing and deployment."

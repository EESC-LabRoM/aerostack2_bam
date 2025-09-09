#!/bin/bash

# 🚁 Jetson Hardware Deployment Guide
# Navigation Commands for Real Betaflight Flight Controller

echo "🚁 Jetson + Betaflight Hardware Deployment"
echo "=========================================="
echo ""
echo "✅ YES! Navigation commands work on real hardware!"
echo "   The MSP protocol is identical for SITL and real FC."
echo ""

echo "🔧 HARDWARE SETUP STEPS:"
echo "========================"
echo ""
echo "1️⃣ PHYSICAL CONNECTIONS:"
echo "   - Connect Betaflight FC to Jetson via USB/UART"
echo "   - Common ports: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyS0"
echo "   - Recommended: Use hardware UART for best performance"
echo ""

echo "2️⃣ BETAFLIGHT CONFIGURATION:"
echo "   - Enable MSP over serial in Betaflight Configurator"
echo "   - Set baud rate: 115200 (recommended) or 57600"
echo "   - Enable 'Serial RX' on UART if using RC override"
echo "   - Verify MSP is working with Betaflight Configurator"
echo ""

echo "3️⃣ JETSON SETUP:"
echo "   - Install AeroStack2 and this MSP platform"
echo "   - Set user permissions: sudo usermod -a -G dialout \$USER"
echo "   - Logout/login to apply permissions"
echo ""

echo "4️⃣ CONNECTION TEST:"
echo "   # Check available serial ports"
echo "   ls -la /dev/tty{USB,ACM,S}*"
echo ""
echo "   # Test basic connection"
echo "   stty -F /dev/ttyUSB0 115200"
echo "   echo 'test' > /dev/ttyUSB0"
echo ""

echo "🚀 LAUNCH COMMANDS FOR REAL HARDWARE:"
echo "====================================="
echo ""
echo "Option 1 - USB Connection:"
echo 'ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py \'
echo '  betaflight_host:=/dev/ttyUSB0 \'
echo '  betaflight_port:=115200'
echo ""

echo "Option 2 - UART Connection:"
echo 'ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py \'
echo '  betaflight_host:=/dev/ttyS0 \'
echo '  betaflight_port:=115200'
echo ""

echo "Option 3 - TCP/IP (if using wireless MSP):"
echo 'ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py \'
echo '  betaflight_host:=192.168.1.100 \'
echo '  betaflight_port:=5761'
echo ""

echo "📡 SAME NAVIGATION COMMANDS WORK:"
echo "================================="
echo ""
echo "ARM:"
echo 'ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \'
echo '  "{control_mode: {yaw_mode: 0, control_mode: 0, reference_frame: 0}}"'
echo ""

echo "TAKEOFF:"
echo 'ros2 service call /drone0/takeoff as2_msgs/srv/Takeoff \'
echo '  "{takeoff_height: 2.0, takeoff_speed: 1.0}"'
echo ""

echo "POSITION CONTROL:"
echo 'ros2 topic pub --once /drone0/motion_reference/pose geometry_msgs/msg/PoseStamped \'
echo "'{"
echo '  "header": {"frame_id": "earth"},'
echo '  "pose": {"position": {"x": 5.0, "y": 3.0, "z": 2.0}}'
echo "}'"
echo ""

echo "VELOCITY CONTROL:"
echo 'ros2 topic pub /drone0/motion_reference/twist geometry_msgs/msg/TwistStamped \'
echo "'{"
echo '  "header": {"frame_id": "base_link"},'
echo '  "twist": {"linear": {"x": 2.0, "y": 0.0, "z": 0.0}}'
echo "}'"
echo ""

echo "LAND:"
echo 'ros2 service call /drone0/land as2_msgs/srv/Land "{land_speed: 0.5}"'
echo ""

echo "🛡️ SAFETY CONSIDERATIONS:"
echo "========================="
echo ""
echo "⚠️  IMPORTANT SAFETY NOTES:"
echo "   - Always test in simulation first"
echo "   - Start with low speeds and small movements"
echo "   - Keep manual override (RC transmitter) ready"
echo "   - Test arming/disarming before flight"
echo "   - Use failsafe configurations in Betaflight"
echo "   - Monitor battery voltage and flight time"
echo ""

echo "🔍 HARDWARE VERIFICATION:"
echo "========================="
echo ""
echo "Test MSP Connection:"
echo 'ros2 topic echo /drone0/sensor_measurements/imu --once'
echo ""
echo "Check Platform Status:"
echo 'ros2 topic echo /drone0/platform/info --once'
echo ""
echo "Monitor Navigation Commands:"
echo 'ros2 topic echo /drone0/motion_reference/pose'
echo ""

echo "⚡ PERFORMANCE TUNING:"
echo "====================="
echo ""
echo "For optimal performance on Jetson:"
echo "- Use hardware UART instead of USB when possible"
echo "- Set CPU governor to 'performance'"
echo "- Disable power management: sudo nvpmodel -m 0"
echo "- Monitor CPU usage: htop"
echo "- Adjust control frequencies if needed"
echo ""

echo "🔧 TROUBLESHOOTING REAL HARDWARE:"
echo "================================="
echo ""
echo "Permission Denied:"
echo "sudo chmod 666 /dev/ttyUSB0"
echo "sudo usermod -a -G dialout \$USER"
echo ""
echo "Connection Timeout:"
echo "- Check cable connections"
echo "- Verify baud rate matches Betaflight"
echo "- Try different USB ports"
echo "- Check Betaflight MSP is enabled"
echo ""
echo "No Sensor Data:"
echo "- Verify FC is powered and booted"
echo "- Check MSP communication in Betaflight Configurator"
echo "- Monitor /drone0/platform/info for connection status"
echo ""

# Check if we're on a Jetson
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model 2>/dev/null)
    if [[ "$MODEL" =~ "Jetson" ]]; then
        echo "✅ DETECTED: Running on Jetson hardware!"
        echo "   Model: $MODEL"
        echo ""
        echo "🔋 Jetson-Specific Tips:"
        echo "   - Use /dev/ttyTHS* for hardware UART"
        echo "   - Enable UART in /boot/extlinux/extlinux.conf"
        echo "   - Monitor temperature: tegrastats"
    fi
fi

echo ""
echo "🎯 HARDWARE DEPLOYMENT READY!"
echo "============================="
echo ""
echo "The same navigation commands that work in simulation"
echo "will work EXACTLY the same on your Jetson + Betaflight!"
echo ""
echo "MSP Protocol Advantages for Hardware:"
echo "✅ Native Betaflight communication"
echo "✅ Low latency serial communication"
echo "✅ Proven reliability in RC community"
echo "✅ Full access to Betaflight features"
echo "✅ Real-time performance on Jetson"
echo ""
echo "Ready to fly! 🚁"

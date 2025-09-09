#!/bin/bash

# 🚁 Complete Navigation Guide for Betaflight MSP Platform
# This answers your question: "How can I use navigation instructions via MSP?"

echo "🚁 Betaflight MSP Navigation Guide"
echo "=================================="
echo ""
echo "✅ Answer: YES! You can send navigation instructions via MSP!"
echo "   The MSP platform exposes standard AeroStack2 navigation topics."
echo ""

echo "📋 NAVIGATION TOPICS YOU CAN PUBLISH TO:"
echo ""
echo "🎯 POSITION CONTROL:"
echo "   Topic: /drone0/motion_reference/pose"
echo "   Type:  geometry_msgs/msg/PoseStamped"
echo "   Use:   Send target position (x,y,z) and orientation"
echo ""
echo "🚀 VELOCITY CONTROL:"
echo "   Topic: /drone0/motion_reference/twist"
echo "   Type:  geometry_msgs/msg/TwistStamped"
echo "   Use:   Send target velocity (linear & angular)"
echo ""
echo "⚡ THRUST CONTROL:"
echo "   Topic: /drone0/actuator_command/thrust"
echo "   Type:  as2_msgs/msg/Thrust"
echo "   Use:   Direct thrust values for each motor"
echo ""

echo "🛠️ CONTROL SERVICES:"
echo ""
echo "🔧 ARM/DISARM:"
echo "   Service: /drone0/arm"
echo "   Type:    as2_msgs/srv/SetControlMode"
echo ""
echo "🛫 TAKEOFF:"
echo "   Service: /drone0/takeoff"
echo "   Type:    as2_msgs/srv/Takeoff"
echo ""
echo "🛬 LANDING:"
echo "   Service: /drone0/land"
echo "   Type:    as2_msgs/srv/Land"
echo ""

echo "📊 SENSOR DATA (READ-ONLY):"
echo ""
echo "📡 IMU DATA:"
echo "   Topic: /drone0/sensor_measurements/imu"
echo "   Type:  sensor_msgs/msg/Imu"
echo ""
echo "🗺️ POSITION:"
echo "   Topic: /drone0/self_localization/pose"
echo "   Type:  geometry_msgs/msg/PoseWithCovarianceStamped"
echo ""

echo ""
echo "🚀 PRACTICAL EXAMPLES:"
echo "======================"
echo ""

echo "1️⃣ ARM THE DRONE:"
echo 'ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \\'
echo '  "{control_mode: {yaw_mode: 0, control_mode: 0, reference_frame: 0}}"'
echo ""

echo "2️⃣ TAKEOFF TO 2 METERS:"
echo 'ros2 service call /drone0/takeoff as2_msgs/srv/Takeoff \\'
echo '  "{takeoff_height: 2.0, takeoff_speed: 1.0}"'
echo ""

echo "3️⃣ FLY TO POSITION (5,3,2):"
cat << 'EOF'
ros2 topic pub --once /drone0/motion_reference/pose geometry_msgs/msg/PoseStamped '
{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": "earth"
  },
  "pose": {
    "position": {"x": 5.0, "y": 3.0, "z": 2.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}'
EOF
echo ""

echo "4️⃣ SET FORWARD VELOCITY (2 m/s):"
cat << 'EOF'
ros2 topic pub --once /drone0/motion_reference/twist geometry_msgs/msg/TwistStamped '
{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": "base_link"
  },
  "twist": {
    "linear": {"x": 2.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}'
EOF
echo ""

echo "5️⃣ MONITOR DRONE STATE:"
echo "ros2 topic echo /drone0/self_localization/pose"
echo ""

echo "6️⃣ LAND THE DRONE:"
echo 'ros2 service call /drone0/land as2_msgs/srv/Land "{land_speed: 0.5}"'
echo ""

echo ""
echo "🔧 SETUP STEPS:"
echo "==============="
echo ""
echo "Terminal 1 - Start Betaflight SITL:"
echo "cd /tmp/betaflight && ./obj/betaflight_2025.12.0-beta_SITL"
echo ""
echo "Terminal 2 - Start MSP Platform:"
echo "cd /home/nexus/aerostack2_ws"
echo "source install/setup.bash"
echo "ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py"
echo ""
echo "Terminal 3 - Send Navigation Commands:"
echo "source install/setup.bash"
echo "# Use any of the commands above!"
echo ""

echo ""
echo "💡 DEBUGGING TIPS:"
echo "=================="
echo ""
echo "Check available topics:"
echo "ros2 topic list | grep drone0"
echo ""
echo "Monitor specific topic:"
echo "ros2 topic echo /drone0/motion_reference/pose"
echo ""
echo "Check topic message format:"
echo "ros2 interface show geometry_msgs/msg/PoseStamped"
echo ""

# Check if system is ready
echo ""
echo "🔍 CURRENT STATUS:"
echo "=================="

# Check if Betaflight SITL is running
if pgrep -f "betaflight.*SITL" > /dev/null; then
    echo "✅ Betaflight SITL: Running"
else
    echo "❌ Betaflight SITL: Not running"
fi

# Check if MSP platform is running
if pgrep -f "as2_platform_betaflight_sim_node" > /dev/null; then
    echo "✅ MSP Platform: Running"
    echo ""
    echo "📋 Available Navigation Topics:"
    ros2 topic list 2>/dev/null | grep -E "(drone0/motion_reference|drone0/actuator_command)" | head -5
else
    echo "❌ MSP Platform: Not running"
fi

echo ""
echo "🎯 Ready to navigate with MSP! 🚁"

#!/bin/bash

# 🚁 Live Navigation Demo for Betaflight MSP Platform
# This script demonstrates real navigation commands

echo "🚁 Live MSP Navigation Demo"
echo "==========================="
echo ""

# Check if everything is running
if ! pgrep -f "betaflight.*SITL" > /dev/null; then
    echo "❌ Betaflight SITL not running. Start it first:"
    echo "   cd /tmp/betaflight && ./obj/betaflight_2025.12.0-beta_SITL"
    exit 1
fi

if ! pgrep -f "as2_platform_betaflight_sim_node" > /dev/null; then
    echo "❌ MSP Platform not running. Start it first:"
    echo "   ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py"
    exit 1
fi

echo "✅ Both Betaflight SITL and MSP Platform are running!"
echo ""

# Source ROS2 environment
source /home/nexus/aerostack2_ws/install/setup.bash

echo "🔍 Available Navigation Topics:"
echo "$(ros2 topic list | grep -E '(motion_reference|actuator_command)' | head -5)"
echo ""

echo "🚀 Demo Commands:"
echo "1. First ARM the drone"
echo "2. Then TAKEOFF"
echo "3. Send position/velocity commands"
echo "4. Finally LAND and DISARM"
echo ""

read -p "Press Enter to start the demo..."

echo ""
echo "1️⃣ Arming the drone..."
ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \
  "{control_mode: {yaw_mode: 0, control_mode: 0, reference_frame: 0}}" \
  --wait 2

if [ $? -eq 0 ]; then
    echo "✅ Drone armed successfully!"
else
    echo "❌ Failed to arm drone"
fi

sleep 2

echo ""
echo "2️⃣ Taking off to 2 meters..."
ros2 service call /drone0/takeoff as2_msgs/srv/Takeoff \
  "{takeoff_height: 2.0, takeoff_speed: 1.0}" \
  --wait 5

if [ $? -eq 0 ]; then
    echo "✅ Takeoff command sent!"
else
    echo "❌ Failed to takeoff"
fi

sleep 3

echo ""
echo "3️⃣ Sending position command (move to x=2, y=1, z=2)..."
ros2 topic pub --once /drone0/motion_reference/pose geometry_msgs/msg/PoseStamped \
'{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": "earth"
  },
  "pose": {
    "position": {"x": 2.0, "y": 1.0, "z": 2.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}'

echo "✅ Position command sent!"
sleep 3

echo ""
echo "4️⃣ Sending velocity command (forward 1 m/s for 2 seconds)..."
ros2 topic pub /drone0/motion_reference/twist geometry_msgs/msg/TwistStamped \
'{
  "header": {
    "stamp": {"sec": 0, "nanosec": 0},
    "frame_id": "base_link"
  },
  "twist": {
    "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}' --times 3 &

echo "✅ Velocity command sent!"
sleep 4

echo ""
echo "5️⃣ Landing the drone..."
ros2 service call /drone0/land as2_msgs/srv/Land \
  "{land_speed: 0.5}" \
  --wait 5

if [ $? -eq 0 ]; then
    echo "✅ Landing command sent!"
else
    echo "❌ Failed to land"
fi

sleep 3

echo ""
echo "6️⃣ Disarming the drone..."
ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \
  "{control_mode: {yaw_mode: 0, control_mode: 1, reference_frame: 0}}" \
  --wait 2

if [ $? -eq 0 ]; then
    echo "✅ Drone disarmed successfully!"
else
    echo "❌ Failed to disarm drone"
fi

echo ""
echo "🎉 Demo completed!"
echo ""
echo "💡 You can monitor the drone state with:"
echo "   ros2 topic echo /drone0/self_localization/pose"
echo ""
echo "💡 You can send your own commands using the examples in navigation_guide.sh"

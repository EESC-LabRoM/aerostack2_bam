# Betaflight SITL + AeroStack2 MSP Integration

This package provides a complete integration between Betaflight flight controller firmware and AeroStack2 autonomous drone framework, specifically designed for deployment on Jetson computers with real Betaflight controllers using the **MSP (Multiwii Serial Protocol)** instead of MAVROS. This approach provides native Betaflight communication and works with both SITL simulation and real hardware.

## Overview

This integration enables you to:
- **Run Betaflight SITL** (Software-in-the-Loop) for simulation
- **Bridge AeroStack2 with Betaflight** using a custom platform
- **Use MSP** for MAVLink protocol communication
- **Simulate MSP** (Multiwii Serial Protocol) commands
- **Deploy on real hardware** (Jetson + Betaflight FC)

## Architecture

```
[AeroStack2 Behaviors]
         ↓
[Betaflight Platform Bridge] ← This Package
         ↓
   [MSP Helper]
         ↓
[Betaflight SITL/FC]
         ↓
[Motors/ESCs/Simulation]
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- AeroStack2 framework
- MSP package
- Build tools (cmake, make, gcc)

## Installation

### 1. Clone and Build Betaflight SITL

```bash
# Clone Betaflight
cd /tmp
git clone https://github.com/betaflight/betaflight.git
cd betaflight

# Install dependencies
sudo apt update
sudo apt install -y build-essential cmake git

# Build SITL target
make TARGET=SITL
```

### 2. Install MSP

```bash
sudo apt install -y ros-humble-msp ros-humble-msp-extras
sudo geographiclib-get-geoids egm96-5
```

### 3. Build the Platform Package

```bash
# Navigate to your AeroStack2 workspace
cd /path/to/your/aerostack2_ws

# Build the Betaflight platform
source /path/to/your/ros2/install/setup.bash
colcon build --packages-select as2_platform_betaflight_sim
```

## Usage

### MSP Platform Integration (✅ WORKING)

**Core functionality for navigation commands via Jetson to Betaflight:**

1. **Quick Demo (Recommended):**
```bash
# Run the MSP platform demo
cd /home/nexus/aerostack2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_betaflight_sim
./msp_demo.sh
```

2. **Test Navigation Commands:**
```bash
# In another terminal, test navigation capabilities
./test_navigation.sh
```

3. **Manual Setup:**
```bash
# Terminal 1: Start Betaflight SITL
cd /tmp/betaflight
./obj/betaflight_2025.12.0-beta_SITL

# Terminal 2: Start MSP Platform
cd /home/nexus/aerostack2_ws
source install/setup.bash
ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py
```

### Navigation Command Examples

Once the platform is running, you can send navigation commands:

```bash
# ARM the drone
ros2 service call /drone0/platform/set_arming_state std_srvs/srv/SetBool '{data: true}'

# Send velocity commands (forward 1 m/s)
ros2 topic pub /drone0/motion_reference_handlers/speed_motion geometry_msgs/msg/TwistStamped "
header:
  frame_id: 'base_link'
twist:
  linear: {x: 1.0, y: 0.0, z: 0.0}
  angular: {z: 0.0}"

# Stop motion
ros2 topic pub /drone0/motion_reference_handlers/speed_motion geometry_msgs/msg/TwistStamped "
header:
  frame_id: 'base_link'
twist:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {z: 0.0}"
```

### Gazebo Integration (🔧 OPTIONAL)

**Note:** Gazebo integration is available but may require additional setup depending on your Gazebo version.

The MSP platform works independently of Gazebo - you can send navigation commands without visualization.

For Gazebo integration:
```bash
# Check your Gazebo version
ign gazebo --version

# If you have Ignition Gazebo 6.x, try:
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py headless:=true
```

**Important:** The core navigation functionality works perfectly without Gazebo!

### Gazebo Configuration

The Gazebo integration provides:
- **3D Visualization** of drone movement
- **Sensor Simulation** (cameras, lidar, GPS, IMU)
- **World Physics** and collision detection
- **Optional Position Sync** with Betaflight SITL

Configure your Gazebo simulation in `config/gazebo_simulation.json`:
```json
{
    "world_name": "empty",
    "drones": [
        {
            "model_type": "quadrotor_base",
            "model_name": "betaflight_drone",
            "xyz": [0.0, 0.0, 0.2],
            "rpy": [0, 0, 1.57],
            "payload": [
                {
                    "model_name": "front_camera",
                    "model_type": "hd_camera",
                    "xyz": [0.1, 0.0, 0.0]
                }
            ]
        }
    ]
}
```

### Real Hardware Deployment

For Jetson + Real Betaflight Controller:

1. **Connect Betaflight FC** via USB/Serial to Jetson

2. **Configure MSP** for serial connection:
```bash
ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py fcu_url:=/dev/ttyUSB0:115200
```

3. **Launch AeroStack2 behaviors** for autonomous flight

## Package Structure

```
as2_platform_betaflight_sim/
├── include/as2_platform_betaflight_sim/
│   └── betaflight_platform.hpp          # Platform interface
├── src/
│   ├── betaflight_platform.cpp          # Main platform implementation
│   └── betaflight_sim_node.cpp          # ROS2 node wrapper
├── launch/
│   ├── betaflight_simple.launch.py      # Platform only
│   └── betaflight_sitl_full.launch.py   # Platform + MSP
├── config/
│   ├── control_modes.yaml               # Supported control modes
│   └── platform_config.yaml             # Platform configuration
├── package.xml                          # ROS2 package manifest
└── CMakeLists.txt                       # Build configuration
```

## Configuration

### Control Modes

The platform supports the following AeroStack2 control modes:
- `UNSET` - No control
- `HOVER` - Hover in place
- `ATTITUDE` - Attitude control (with yaw angle/speed)
- `SPEED` - Velocity control (body/global frame)

### Platform Parameters

Key parameters in `platform_config.yaml`:
- `msp.connection_url` - MSP connection string
- `betaflight.msp_port` - MSP communication port
- `betaflight.control_frequency` - Control loop frequency
- `sensors.*` - Sensor configuration

## Testing

### Basic Platform Test

```bash
# Test platform startup
ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py

# Check topics
ros2 topic list | grep drone0

# Test arming service
ros2 service call /drone0/platform/set_arming_state std_srvs/srv/SetBool '{data: true}'
```

### Integration Test with Behaviors

```bash
# Launch full stack
ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py

# Test takeoff behavior (example)
ros2 run as2_behaviors_motion takeoff_behavior --ros-args -r __ns:=/drone0
```

## Navigation Instructions via MSP

### Overview

**YES! You can send navigation instructions via MSP to Betaflight!** The MSP platform exposes standard AeroStack2 navigation topics that translate directly to native Betaflight commands.

### Available Navigation Topics

#### 🎯 Position Control
- **Topic:** `/drone0/motion_reference/pose`
- **Type:** `geometry_msgs/msg/PoseStamped`
- **Use:** Send target position (x,y,z) and orientation

#### 🚀 Velocity Control
- **Topic:** `/drone0/motion_reference/twist`
- **Type:** `geometry_msgs/msg/TwistStamped`
- **Use:** Send target velocity (linear & angular)

#### ⚡ Direct Thrust Control
- **Topic:** `/drone0/actuator_command/thrust`
- **Type:** `as2_msgs/msg/Thrust`
- **Use:** Direct thrust values for each motor

### Control Services

#### 🔧 Arm/Disarm
- **Service:** `/drone0/arm`
- **Type:** `as2_msgs/srv/SetControlMode`

#### 🛫 Takeoff
- **Service:** `/drone0/takeoff`
- **Type:** `as2_msgs/srv/Takeoff`

#### 🛬 Landing
- **Service:** `/drone0/land`
- **Type:** `as2_msgs/srv/Land`

### Sensor Data Topics (Read-Only)

#### 📡 IMU Data
- **Topic:** `/drone0/sensor_measurements/imu`
- **Type:** `sensor_msgs/msg/Imu`

#### 🗺️ Position Data
- **Topic:** `/drone0/self_localization/pose`
- **Type:** `geometry_msgs/msg/PoseWithCovarianceStamped`

### Navigation Examples

#### Complete Flight Sequence

```bash
# 1. ARM THE DRONE
ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \
  "{control_mode: {yaw_mode: 0, control_mode: 0, reference_frame: 0}}"

# 2. TAKEOFF TO 2 METERS
ros2 service call /drone0/takeoff as2_msgs/srv/Takeoff \
  "{takeoff_height: 2.0, takeoff_speed: 1.0}"

# 3. FLY TO POSITION (x=5, y=3, z=2 meters)
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

# 4. SET FORWARD VELOCITY (2 m/s)
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

# 5. LAND THE DRONE
ros2 service call /drone0/land as2_msgs/srv/Land "{land_speed: 0.5}"

# 6. DISARM THE DRONE
ros2 service call /drone0/arm as2_msgs/srv/SetControlMode \
  "{control_mode: {yaw_mode: 0, control_mode: 1, reference_frame: 0}}"
```

#### Monitor Drone State

```bash
# Monitor current position
ros2 topic echo /drone0/self_localization/pose

# Check available topics
ros2 topic list | grep drone0

# Monitor IMU data
ros2 topic echo /drone0/sensor_measurements/imu
```

### Quick Start Navigation

#### 3-Terminal Setup

**Terminal 1 - Start Betaflight SITL:**
```bash
cd /tmp/betaflight && ./obj/betaflight_2025.12.0-beta_SITL
```

**Terminal 2 - Start MSP Platform:**
```bash
cd /home/nexus/aerostack2_ws
source install/setup.bash
ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py
```

**Terminal 3 - Send Navigation Commands:**
```bash
source install/setup.bash
# Use any of the navigation commands above!
```

### Ready-to-Use Demo Scripts

The package includes several demo scripts for testing navigation:

```bash
# Complete navigation reference guide
./navigation_guide.sh

# Interactive live navigation demo
./live_navigation_demo.sh

# Basic MSP platform test
./msp_demo.sh

# Gazebo integration with visualization
./demo_gazebo.sh
```

### Debugging Navigation

#### Check Message Formats
```bash
# Check topic message structure
ros2 interface show geometry_msgs/msg/PoseStamped
ros2 interface show geometry_msgs/msg/TwistStamped
ros2 interface show as2_msgs/srv/Takeoff
```

#### Monitor Command Reception
```bash
# Monitor if commands are being received
ros2 topic echo /drone0/motion_reference/pose
ros2 topic echo /drone0/motion_reference/twist
```

#### Service Availability
```bash
# Check available services
ros2 service list | grep drone0
ros2 service type /drone0/arm
ros2 service type /drone0/takeoff
```

### Jetson Hardware Navigation

When deploying on Jetson with real Betaflight hardware:

1. **Serial Connection:**
   ```bash
   # Update fcu_url in launch file
   ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py fcu_url:=/dev/ttyUSB0:115200
   ```

2. **Performance Tuning:**
   - Adjust control frequencies for real-time performance
   - Use hardware serial ports when possible
   - Monitor CPU usage during navigation

3. **Safety Considerations:**
   - Always test in simulation first
   - Use manual override capability
   - Start with low-speed navigation commands

## Troubleshooting

### Common Issues

1. **Betaflight SITL not starting:**
   - Check if port 5761 is available
   - Verify build was successful

2. **MSP connection failed:**
   - Ensure Betaflight SITL is running first
   - Check firewall settings for UDP port 5761

3. **Platform node crashes:**
   - Verify control_modes.yaml exists
   - Check ROS2 environment sourcing

4. **No sensor data:**
   - Verify MSP topics are published
   - Check topic remapping in launch files

### Debug Commands

```bash
# Check MSP connection
ros2 topic echo /drone0/msp/state

# Monitor platform status
ros2 topic echo /drone0/platform/info

# Check available services
ros2 service list | grep drone0
```

## Real Hardware Notes

### Jetson Deployment

1. **Serial Configuration:**
   ```bash
   # Check available ports
   ls /dev/ttyUSB* /dev/ttyACM*
   
   # Set permissions
   sudo usermod -a -G dialout $USER
   ```

2. **Betaflight Configuration:**
   - Enable MSP over serial
   - Set appropriate baud rate (57600/115200)
   - Configure MAVLink output if needed

3. **Performance Optimization:**
   - Use hardware serial ports when possible
   - Adjust control frequencies for real-time performance
   - Monitor CPU usage on Jetson

## Contributing

This integration was developed as a bridge between AeroStack2's high-level autonomy capabilities and Betaflight's proven flight control performance. 

### Development Notes

- Platform follows AeroStack2 aerial platform interface
- MSP handles MAVLink protocol translation
- MSP simulation provides Betaflight-specific commands
- Designed for easy extension and customization

## License

This package follows the same license as AeroStack2. Please refer to the main AeroStack2 repository for license details.

## Support

For issues related to:
- **AeroStack2:** Check AeroStack2 documentation and GitHub issues
- **Betaflight:** Refer to Betaflight documentation
- **MSP:** See MSP GitHub repository
- **This integration:** Create an issue with detailed logs and configuration

---

**Happy Flying with Betaflight + AeroStack2! 🚁**

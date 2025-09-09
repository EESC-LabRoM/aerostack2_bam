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

### Simulation Mode (SITL)

1. **Start Betaflight SITL:**
```bash
cd /tmp/betaflight
./obj/betaflight_2025.12.0-beta_SITL
```

2. **Launch Platform Only:**
```bash
cd /path/to/aerostack2_ws
source install/setup.bash
ros2 launch as2_platform_betaflight_sim betaflight_simple.launch.py
```

3. **Launch with MSP:**
```bash
ros2 launch as2_platform_betaflight_sim betaflight_sitl_full.launch.py
```

### Gazebo Integration (3D Visualization)

For full 3D visualization combining Betaflight SITL with Gazebo:

1. **Quick Start (Easiest):**
```bash
# Run the automated demo script
cd /home/nexus/aerostack2_ws/src/aerostack2/as2_aerial_platforms/as2_platform_betaflight_sim
./quick_demo.sh
```

2. **Manual Setup:**
```bash
# Terminal 1: Start Betaflight SITL
cd /tmp/betaflight
./obj/betaflight_2025.12.0-beta_SITL

# Terminal 2: Start Gazebo + MSP Platform
cd /home/nexus/aerostack2_ws
source install/setup.bash
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py headless:=true
```

3. **With GUI (if display available):**
```bash
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py headless:=false
```

4. **Custom Configuration:**
```bash
# Custom Betaflight connection
ros2 launch as2_platform_betaflight_sim betaflight_gazebo_launch.py \
    betaflight_host:=192.168.1.100 \
    betaflight_port:=5761 \
    namespace:=my_drone
```

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

#include "as2_platform_betaflight_sim/betaflight_platform.hpp"
#include "rclcpp/rclcpp.hpp"

namespace betaflight_sim {

BetaflightPlatform::BetaflightPlatform() : as2::AerialPlatform() {
  armed_ = false;
  offboard_mode_ = false;
  
  // Create publishers for MAVROS topics
  attitude_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "/mavros/setpoint_raw/attitude", 10);
    
  rc_pub_ = this->create_publisher<mavros_msgs::msg::RCIn>(
    "/mavros/rc/in", 10);
  
  // Subscribe to MAVROS IMU data
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/mavros/imu/data", 10,
    std::bind(&BetaflightPlatform::imuCallback, this, std::placeholders::_1));
  
  // Create timer for periodic MSP simulation
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), // 50Hz
    std::bind(&BetaflightPlatform::timerCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "Betaflight SITL Platform initialized");
}

void BetaflightPlatform::configureSensors() {
  // Configure platform sensors - basic setup for SITL
  RCLCPP_INFO(this->get_logger(), "Configuring sensors for Betaflight SITL");
}

bool BetaflightPlatform::ownSendCommand() {
  // Handle sending commands to Betaflight SITL via MAVROS
  sendAttitudeCommand();
  return true;
}

bool BetaflightPlatform::ownSetArmingState(bool state) {
  RCLCPP_INFO(this->get_logger(), "Setting arm state: %s", state ? "ARMED" : "DISARMED");
  armed_ = state;
  return true;
}

bool BetaflightPlatform::ownSetOffboardControl(bool offboard) {
  RCLCPP_INFO(this->get_logger(), "Setting offboard control: %s", 
              offboard ? "ENABLED" : "DISABLED");
  offboard_mode_ = offboard;
  return true;
}

bool BetaflightPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) {
  RCLCPP_INFO(this->get_logger(), "Setting platform control mode: %d", msg.control_mode);
  return true;
}

void BetaflightPlatform::ownKillSwitch() {
  RCLCPP_WARN(this->get_logger(), "EMERGENCY KILL SWITCH ACTIVATED!");
  armed_ = false;
  offboard_mode_ = false;
}

void BetaflightPlatform::ownStopPlatform() {
  RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP - Platform will hover");
  // In real implementation, this would send hover commands
}

void BetaflightPlatform::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Process IMU data from MAVROS (Betaflight SITL)
  // This would typically update internal state estimation
}

void BetaflightPlatform::timerCallback() {
  simulateMSPCommands();
}

void BetaflightPlatform::simulateMSPCommands() {
  // Simulate MSP protocol commands that would be sent to Betaflight
  // In real implementation, this might send RC commands or status requests
  
  if (armed_ && offboard_mode_) {
    // Send periodic RC commands to maintain control
    auto rc_msg = mavros_msgs::msg::RCIn();
    rc_msg.header.stamp = this->now();
    rc_msg.channels = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000}; // Neutral values
    rc_pub_->publish(rc_msg);
  }
}

void BetaflightPlatform::sendAttitudeCommand() {
  if (!armed_ || !offboard_mode_) {
    return;
  }
  
  // Get current command from AerialPlatform base class
  auto cmd = command_twist_msg_.twist;
  
  // Convert twist command to attitude target for Betaflight
  auto attitude_msg = mavros_msgs::msg::AttitudeTarget();
  attitude_msg.header.stamp = this->now();
  attitude_msg.header.frame_id = "base_link";
  
  // Map twist commands to attitude setpoints
  // This is a simplified mapping - real implementation would be more sophisticated
  attitude_msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                          mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                          mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  
  // Simple mapping: linear.x -> pitch, linear.y -> roll, angular.z -> yaw, linear.z -> thrust
  float roll_angle = -cmd.linear.y * 0.2;  // Negative for correct direction
  float pitch_angle = cmd.linear.x * 0.2;
  float yaw_rate = cmd.angular.z;
  float thrust = (cmd.linear.z + 1.0) / 2.0; // Map from [-1,1] to [0,1]
  
  // Convert to quaternion (simplified - assumes small angles)
  attitude_msg.orientation.w = 1.0;
  attitude_msg.orientation.x = roll_angle / 2.0;
  attitude_msg.orientation.y = pitch_angle / 2.0;
  attitude_msg.orientation.z = 0.0;
  
  attitude_msg.body_rate.z = yaw_rate;
  attitude_msg.thrust = thrust;
  
  attitude_pub_->publish(attitude_msg);
}

} // namespace betaflight_sim

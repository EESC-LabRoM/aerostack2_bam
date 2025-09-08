#include "as2_platform_betaflight_sim/betaflight_platform.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace betaflight_sim {

BetaflightPlatform::BetaflightPlatform() : as2::AerialPlatform() {
  armed_ = false;
  offboard_mode_ = false;
  current_altitude_ = 0.0f;
  current_voltage_ = 12.0f;
  
  // Get MSP connection parameters from ROS parameters
  this->declare_parameter("msp_connection", "udp:localhost:5761");
  this->declare_parameter("msp_baud_rate", 115200);
  
  msp_connection_string_ = this->get_parameter("msp_connection").as_string();
  msp_baud_rate_ = this->get_parameter("msp_baud_rate").as_int();
  
  // Initialize MSP helper
  msp_helper_ = std::make_unique<MSPHelper>(msp_connection_string_, msp_baud_rate_);
  
  // Connect to Betaflight
  if (!msp_helper_->connect()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to Betaflight via MSP: %s", 
                 msp_connection_string_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to Betaflight via MSP: %s", 
                msp_connection_string_.c_str());
  }
  
  // Create timer for periodic MSP communication (50Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), 
    std::bind(&BetaflightPlatform::timerCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "Betaflight MSP Platform initialized");
}

void BetaflightPlatform::configureSensors() {
  // Configure platform sensors - MSP will provide telemetry data
  RCLCPP_INFO(this->get_logger(), "Configuring sensors for Betaflight MSP");
}

bool BetaflightPlatform::ownSendCommand() {
  // Send commands to Betaflight via MSP
  sendRCCommands();
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
  
  // Send neutral RC commands to stop the drone
  if (msp_helper_ && msp_helper_->isConnected()) {
    msp_helper_->sendRCCommand(
      MSPHelper::RC_NEUTRAL, // Roll
      MSPHelper::RC_NEUTRAL, // Pitch
      MSPHelper::RC_NEUTRAL, // Yaw
      MSPHelper::RC_MIN      // Throttle (minimum)
    );
  }
}

void BetaflightPlatform::ownStopPlatform() {
  RCLCPP_WARN(this->get_logger(), "EMERGENCY STOP - Platform will hover");
  
  // Send hover commands (neutral stick positions with current throttle)
  if (msp_helper_ && msp_helper_->isConnected()) {
    msp_helper_->sendRCCommand(
      MSPHelper::RC_NEUTRAL, // Roll
      MSPHelper::RC_NEUTRAL, // Pitch  
      MSPHelper::RC_NEUTRAL, // Yaw
      MSPHelper::RC_NEUTRAL  // Throttle (hover)
    );
  }
}

void BetaflightPlatform::timerCallback() {
  // Update telemetry from Betaflight
  updateTelemetry();
}

void BetaflightPlatform::updateTelemetry() {
  if (!msp_helper_ || !msp_helper_->isConnected()) {
    return;
  }
  
  // Read altitude
  float altitude;
  if (msp_helper_->readAltitude(altitude)) {
    current_altitude_ = altitude;
  }
  
  // Read analog values (battery, etc.)
  float voltage, current;
  uint16_t rssi;
  if (msp_helper_->readAnalog(voltage, current, rssi)) {
    current_voltage_ = voltage;
  }
  
  // Read current RC values
  std::vector<uint16_t> rc_values;
  if (msp_helper_->readRCValues(rc_values)) {
    current_rc_values_ = rc_values;
  }
}

void BetaflightPlatform::sendRCCommands() {
  if (!msp_helper_ || !msp_helper_->isConnected() || !armed_ || !offboard_mode_) {
    return;
  }
  
  // Convert AeroStack2 twist command to RC values
  uint16_t roll, pitch, yaw, throttle;
  convertTwistToRC(command_twist_msg_, roll, pitch, yaw, throttle);
  
  // Send RC command via MSP
  bool success = msp_helper_->sendRCCommand(roll, pitch, yaw, throttle);
  
  if (!success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                         "Failed to send RC command via MSP");
  }
}

void BetaflightPlatform::convertTwistToRC(const geometry_msgs::msg::TwistStamped& twist,
                                         uint16_t& roll, uint16_t& pitch, uint16_t& yaw, uint16_t& throttle) {
  // Get twist command values
  double linear_x = twist.twist.linear.x;   // Forward/backward (pitch)
  double linear_y = twist.twist.linear.y;   // Left/right (roll) 
  double linear_z = twist.twist.linear.z;   // Up/down (throttle)
  double angular_z = twist.twist.angular.z; // Yaw rate
  
  // Convert to RC values (1000-2000 range, 1500 neutral)
  // Scale factors can be adjusted based on desired responsiveness
  const double scale_angle = 200.0;  // ±200 from neutral for angles
  const double scale_throttle = 500.0; // ±500 from neutral for throttle
  const double scale_yaw_rate = 200.0; // ±200 from neutral for yaw rate
  
  // Roll: negative linear_y for correct direction
  roll = static_cast<uint16_t>(MSPHelper::RC_NEUTRAL - linear_y * scale_angle);
  
  // Pitch: positive linear_x for forward
  pitch = static_cast<uint16_t>(MSPHelper::RC_NEUTRAL + linear_x * scale_angle);
  
  // Yaw rate: positive angular_z for clockwise
  yaw = static_cast<uint16_t>(MSPHelper::RC_NEUTRAL + angular_z * scale_yaw_rate);
  
  // Throttle: linear_z range [-1,1] mapped to [1000,2000]
  throttle = static_cast<uint16_t>(MSPHelper::RC_NEUTRAL + linear_z * scale_throttle);
  
  // Clamp values to valid RC range
  roll = std::max(MSPHelper::RC_MIN, std::min(MSPHelper::RC_MAX, roll));
  pitch = std::max(MSPHelper::RC_MIN, std::min(MSPHelper::RC_MAX, pitch));
  yaw = std::max(MSPHelper::RC_MIN, std::min(MSPHelper::RC_MAX, yaw));
  throttle = std::max(MSPHelper::RC_MIN, std::min(MSPHelper::RC_MAX, throttle));
}

} // namespace betaflight_sim

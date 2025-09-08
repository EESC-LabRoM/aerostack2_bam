#pragma once

#include "as2_core/aerial_platform.hpp"
#include "as2_msgs/msg/control_mode.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "as2_platform_betaflight_sim/msp_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace betaflight_sim {

class BetaflightPlatform : public as2::AerialPlatform {
public:
  BetaflightPlatform();
  
private:
  bool armed_;
  bool offboard_mode_;
  
  // MSP communication
  std::unique_ptr<MSPHelper> msp_helper_;
  std::string msp_connection_string_;
  int msp_baud_rate_;
  
  // Timer for periodic operations
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Telemetry data
  float current_altitude_;
  float current_voltage_;
  std::vector<uint16_t> current_rc_values_;
  
  // Core platform functions (inherited from AerialPlatform)
  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;
  
  // MSP communication functions
  void timerCallback();
  void updateTelemetry();
  void sendRCCommands();
  
  // Convert AeroStack2 commands to RC values
  void convertTwistToRC(const geometry_msgs::msg::TwistStamped& twist,
                       uint16_t& roll, uint16_t& pitch, uint16_t& yaw, uint16_t& throttle);
};

} // namespace betaflight_sim

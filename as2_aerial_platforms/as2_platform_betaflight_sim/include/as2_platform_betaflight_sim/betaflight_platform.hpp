#pragma once

#include "as2_core/aerial_platform.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"

namespace betaflight_sim {

class BetaflightPlatform : public as2::AerialPlatform {
public:
  BetaflightPlatform();
  ~BetaflightPlatform() = default;

  // Override virtual methods from AerialPlatform
  void configureSensors() override;
  bool ownSendCommand() override;
  bool ownSetArmingState(bool state) override;
  bool ownSetOffboardControl(bool offboard) override;
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode& msg) override;
  void ownKillSwitch() override;
  void ownStopPlatform() override;

private:
  // Publishers for simulated MAVLink/MSP communication
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_pub_;
  rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub_;
  
  // Subscribers for sensor simulation
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // Timer for periodic updates
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Internal state
  bool armed_;
  bool offboard_mode_;
  
  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timerCallback();
  
  // MSP simulation methods
  void simulateMSPCommands();
  void sendAttitudeCommand();
};

} // namespace betaflight_sim

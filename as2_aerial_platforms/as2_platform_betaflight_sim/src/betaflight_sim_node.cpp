#include "as2_platform_betaflight_sim/betaflight_platform.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<betaflight_sim::BetaflightPlatform>();
  as2::spinLoop(node);
  rclcpp::shutdown();
  return 0;
}

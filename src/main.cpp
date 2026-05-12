#include "rcm_modulation_demo/rcm_tracking_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rcm_modulation_demo::RcmTrackingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp/rclcpp.hpp"
#include "ryzen_ai_cvml/depth_estimation_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ryzen_ai_cvml::DepthEstimationNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
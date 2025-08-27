#include "etsi_its_conversion/Converter.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<etsi_its_conversion::Converter>(rclcpp::NodeOptions{});
  // Use a MultiThreadedExecutor so callbacks within the converter's reentrant
  // callback group can run in parallel. The default `rclcpp::spin` uses a
  // SingleThreadedExecutor and would process callbacks sequentially.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

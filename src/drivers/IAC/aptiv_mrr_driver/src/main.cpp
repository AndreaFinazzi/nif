#include <rclcpp/rclcpp.hpp>
#include "aptiv_mrr_driver/driver_node.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  const auto options = rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<aptiv_mrr_driver::DriverNode>(options));
  rclcpp::shutdown();
  return 0;
}

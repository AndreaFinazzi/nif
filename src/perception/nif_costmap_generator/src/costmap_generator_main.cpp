#include <costmap_generator/costmap_generator.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CostmapGenerator>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

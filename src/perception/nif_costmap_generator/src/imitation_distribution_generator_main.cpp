#include <costmap_generator/imitation_output_to_distribution.h>

using namespace nif::perception::costmap;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CostmapGeneratorV2>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

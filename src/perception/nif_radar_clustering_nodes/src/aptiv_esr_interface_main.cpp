#include <nif_radar_clustering_nodes/aptiv_esr_interface_node.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nif::perception::AptiveESRInterfaceNode>("aptiv_esr_interface_node");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
#include <nif_points_clustering_nodes/lidar_euclidean_clustering_node.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PointsClustering>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
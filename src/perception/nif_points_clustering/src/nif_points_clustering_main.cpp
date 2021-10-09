#include <nif_points_clustering/lidar_euclidean_cluster_detect.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PointsClustering>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
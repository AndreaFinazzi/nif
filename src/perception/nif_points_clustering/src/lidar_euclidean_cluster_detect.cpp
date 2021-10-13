#include <nif_points_clustering/lidar_euclidean_cluster_detect.h>

#ifdef GPU_CLUSTERING

#include "nif_points_clustering/gpu_euclidean_clustering.h"

#endif

#define __APP_NAME__ "euclidean_clustering"

// Constructor
PointsClustering::PointsClustering()
    : Node("nif_points_clustering")
{
  this->declare_parameter<int>("cluster_size_min", 10);
  this->declare_parameter<int>("cluster_size_max", 2000);
  this->declare_parameter<double>("max_cluster_distance", 0.5);
  this->declare_parameter<double>("height_filter_thres", -0.2);

  this->m_cluster_size_min = this->get_parameter("cluster_size_min").as_int();
  this->m_cluster_size_max = this->get_parameter("cluster_size_max").as_int();
  this->m_max_cluster_distance = this->get_parameter("max_cluster_distance").as_double();
  this->m_height_filter_thres =
      this->get_parameter("height_filter_thres").as_double();

  pubClusterPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_clustered_points", nif::common::constants::QOS_SENSOR_DATA);
  pubSimpleheightMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_simple_heightmap_points", nif::common::constants::QOS_SENSOR_DATA);
  pubClusteredArray = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "out_clustered_array", nif::common::constants::QOS_SENSOR_DATA);
  subInputPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_lidar_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsClustering::PointsCallback, this,
                std::placeholders::_1));

  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(
      30ms, std::bind(&PointsClustering::timer_callback, this));
}

PointsClustering::~PointsClustering() {}

void PointsClustering::timer_callback() {

  std::lock_guard<std::mutex> sensor_lock(sensor_mtx);

  if(!bPoints)
    return;

  //simple height map filter
  m_simpleHeightmapPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto point : m_inPoints->points)
  {
    if (point.z > m_height_filter_thres && point.z < 2.0 && fabs(point.y) < 20.0) {
      m_simpleHeightmapPoints->points.push_back(point);
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_simple_heightmap_msg;
  pcl::toROSMsg(*m_simpleHeightmapPoints, cloud_simple_heightmap_msg);
  cloud_simple_heightmap_msg.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  cloud_simple_heightmap_msg.header.stamp = this->now();
  pubSimpleheightMap->publish(cloud_simple_heightmap_msg);

  
  pcl::PointCloud<pcl::PointXYZI>::Ptr registeredPoints(
      new pcl::PointCloud<pcl::PointXYZI>);

  visualization_msgs::msg::MarkerArray clustered_array;
  clusterAndColorGpu(m_simpleHeightmapPoints, registeredPoints,
                     clustered_array, m_max_cluster_distance);

  sensor_msgs::msg::PointCloud2 cloud_clustered_msg;
  pcl::toROSMsg(*registeredPoints, cloud_clustered_msg);
  cloud_clustered_msg.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  cloud_clustered_msg.header.stamp = this->now();
  pubClusterPoints->publish(cloud_clustered_msg);

  pubClusteredArray->publish(clustered_array);
}

void PointsClustering::PointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx);

  m_inPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr originPoints(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *originPoints);
  m_inPoints = downsample(originPoints, 0.05);

  bPoints = true;

  // std::cout << "callback" << std::endl;

}

void PointsClustering::clusterAndColorGpu(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    visualization_msgs::msg::MarkerArray& out_clustered_array,
    double in_max_cluster_distance) {

#ifdef GPU_CLUSTERING

  int size = in_cloud_ptr->points.size();

  // if (size == 0)
  //   return clusters;

  float *tmp_x, *tmp_y, *tmp_z;

  tmp_x = (float *)malloc(sizeof(float) * size);
  tmp_y = (float *)malloc(sizeof(float) * size);
  tmp_z = (float *)malloc(sizeof(float) * size);

  for (int i = 0; i < size; i++) {
    pcl::PointXYZI tmp_point = in_cloud_ptr->at(i);

    tmp_x[i] = tmp_point.x;
    tmp_y[i] = tmp_point.y;
    tmp_z[i] = tmp_point.z;
  }

  GpuEuclideanCluster gecl_cluster;

  gecl_cluster.setInputPoints(tmp_x, tmp_y, tmp_z, size);
  gecl_cluster.setThreshold(in_max_cluster_distance);
  gecl_cluster.setMinClusterPts(m_cluster_size_min);
  gecl_cluster.setMaxClusterPts(m_cluster_size_max);
  gecl_cluster.extractClusters();
  std::vector<GpuEuclideanCluster::GClusterIndex> cluster_indices =
      gecl_cluster.getOutput();

  unsigned int k = 0;
  
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr bufPoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    SetCloud(in_cloud_ptr, bufPoints, it->points_in_cluster,
             out_clustered_array, k);
    //   clusters.push_back(cluster);
    *out_cloud_ptr += *bufPoints;
    k++;
    // std::cout << "k: " << k << ", " << it->points_in_cluster.size() << std::endl;
  }

  // std::cout << "working" << std::endl;

  free(tmp_x);
  free(tmp_y);
  free(tmp_z);

#endif
  // return clusters;
}

void PointsClustering::SetCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud_ptr,
    const std::vector<int> &in_cluster_indices,
    visualization_msgs::msg::MarkerArray &out_clustered_array, int ind) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr bufPoints(new pcl::PointCloud<pcl::PointXYZI>);
  // extract pointcloud using the indices
  // calculate min and max points
  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  float average_x = 0, average_y = 0, average_z = 0;

  for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end();
       ++pit) {
    // fill new colored cluster point by point
    pcl::PointXYZI p;
    p.x = in_origin_cloud_ptr->points[*pit].x;
    p.y = in_origin_cloud_ptr->points[*pit].y;
    p.z = in_origin_cloud_ptr->points[*pit].z;
    p.intensity = ind;

    average_x += p.x;
    average_y += p.y;
    average_z += p.z;
    bufPoints->points.push_back(p);

    if (p.x < min_x)
      min_x = p.x;
    if (p.y < min_y)
      min_y = p.y;
    if (p.z < min_z)
      min_z = p.z;
    if (p.x > max_x)
      max_x = p.x;
    if (p.y > max_y)
      max_y = p.y;
    if (p.z > max_z)
      max_z = p.z;
  }
  double length = max_x - min_x;
  double width = max_y - min_y;
  double height = max_z - min_z;

  visualization_msgs::msg::Marker marker_buf;
  marker_buf.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  marker_buf.header.stamp = this->now();
  marker_buf.id = ind;
  marker_buf.type = visualization_msgs::msg::Marker::CUBE;
  marker_buf.action = visualization_msgs::msg::Marker::ADD;
  marker_buf.lifetime = rclcpp::Duration(0, 5);
  marker_buf.pose.position.x = min_x + length / 2;
  marker_buf.pose.position.y = min_y + width / 2;
  marker_buf.pose.position.z = min_z + height / 2;
  marker_buf.pose.orientation.w = 1.0;
  marker_buf.scale.x = length;
  marker_buf.scale.y = width;
  marker_buf.scale.z = height;
  marker_buf.color.r = 1.0;
  marker_buf.color.g = 1.0;
  marker_buf.color.b = 1.0;
  marker_buf.color.a = 1.0;

  if (length < 7.0 && width < 7.0 && height > 0.5) {
    *register_cloud_ptr = *bufPoints;
    out_clustered_array.markers.push_back(marker_buf);
    }

  // bounding_box_.header = in_ros_header;

  // bounding_box_.pose.position.x = min_point_.x + length_ / 2;
  // bounding_box_.pose.position.y = min_point_.y + width_ / 2;
  // bounding_box_.pose.position.z = min_point_.z + height_ / 2;

  // // Get EigenValues, eigenvectors
  // if (current_cluster->points.size() > 3) {
  //   pcl::PCA<pcl::PointXYZI> current_cluster_pca;
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster_mono(
  //       new pcl::PointCloud<pcl::PointXYZI>);

  //   pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*current_cluster,
  //                                                        *current_cluster_mono);

  //   current_cluster_pca.setInputCloud(current_cluster_mono);
  //   eigen_vectors_ = current_cluster_pca.getEigenVectors();
  //   eigen_values_ = current_cluster_pca.getEigenValues();
  // }

  // valid_cluster_ = true;
  // pointcloud_ = current_cluster;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
PointsClustering::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

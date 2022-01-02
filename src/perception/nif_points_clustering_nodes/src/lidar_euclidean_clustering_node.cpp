#include <nif_points_clustering_nodes/lidar_euclidean_clustering_node.h>

#ifdef GPU_CLUSTERING

#include "nif_gpu_euclidean_clustering/gpu_euclidean_clustering.h"

#endif

#define __APP_NAME__ "euclidean_clustering"

using namespace nif::common::frame_id::localization;

// Constructor
PointsClustering::PointsClustering()
    : Node("lidar_clustering_node")
{
  this->declare_parameter<double>("left_lower_distance", double(0.5));
  this->declare_parameter<double>("right_lower_distance", double(0.5));
  this->declare_parameter<double>("rear_lower_distance", double(2.2));
  this->declare_parameter<double>("front_lower_distance", double(1.5));

  this->declare_parameter<double>("left_upper_distance", double(20.0));
  this->declare_parameter<double>("right_upper_distance", double(20.0));
  this->declare_parameter<double>("rear_upper_distance", double(50.0));
  this->declare_parameter<double>("front_upper_distance", double(50.0));

  this->declare_parameter<double>("height_lower_distance", double(-0.5));
  this->declare_parameter<double>("height_upper_distance", double(1.0));
  this->declare_parameter<double>("resolution", double(0.25));
  this->declare_parameter<double>("count_threshold", double(3.));

  this->declare_parameter<int>("cluster_size_min", 10);
  this->declare_parameter<int>("cluster_size_max", 2000);
  this->declare_parameter<double>("max_cluster_distance", 0.5);
  this->declare_parameter<double>("height_filter_thres", -0.2);

  this->declare_parameter<bool>("use_inverse_map", false);

  this->m_cluster_size_min = this->get_parameter("cluster_size_min").as_int();
  this->m_cluster_size_max = this->get_parameter("cluster_size_max").as_int();
  this->m_max_cluster_distance = this->get_parameter("max_cluster_distance").as_double();
  this->m_height_filter_thres =
      this->get_parameter("height_filter_thres").as_double();

  this->m_use_inverse_map = this->get_parameter("use_inverse_map").as_bool();

  this->get_parameter("left_lower_distance", left_lower_distance_);
  this->get_parameter("right_lower_distance", right_lower_distance_);
  this->get_parameter("rear_lower_distance", rear_lower_distance_);
  this->get_parameter("front_lower_distance", front_lower_distance_);

  this->get_parameter("left_upper_distance", left_upper_distance_);
  this->get_parameter("right_upper_distance", right_upper_distance_);
  this->get_parameter("rear_upper_distance", rear_upper_distance_);
  this->get_parameter("front_upper_distance", front_upper_distance_);

  this->get_parameter("height_lower_distance", height_lower_distance_);
  this->get_parameter("height_upper_distance", height_upper_distance_);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("count_threshold", count_threshold_);


  pubClusterPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_clustered_points", nif::common::constants::QOS_SENSOR_DATA);
  pubSimpleheightMap = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_simple_heightmap_points", nif::common::constants::QOS_SENSOR_DATA);
  pubInflationPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_inflation_points", nif::common::constants::QOS_SENSOR_DATA);
  pubClusteredArray =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "out_clustered_array", nif::common::constants::QOS_SENSOR_DATA);
  pubClusteredCenterPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "out_clustered_center_points", nif::common::constants::QOS_SENSOR_DATA);
  pubPerceptionList = this->create_publisher<nif::common::msgs::PerceptionResultList>(
      "out_perception_list", nif::common::constants::QOS_SENSOR_DATA);
  pubInverseMappedPoints =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "out_inverse_mapped_points",
          nif::common::constants::QOS_SENSOR_DATA);


  subInputPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_lidar_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsClustering::PointsCallback, this,
                std::placeholders::_1));
  subLeftPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_left_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsClustering::LeftPointsCallback, this,
                std::placeholders::_1));
  subRightPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_right_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsClustering::RightPointsCallback, this,
                std::placeholders::_1));

  // if (this->m_use_inverse_map) {
  //   subInverseMapPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "in_inverse_map_points", nif::common::constants::QOS_SENSOR_DATA,
  //     std::bind(&PointsClustering::inverseMapPointsCallback, this,
  //               std::placeholders::_1));
  // }

  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(
      50ms, std::bind(&PointsClustering::timer_callback, this));

  this->parameters_callback_handle = this->add_on_set_parameters_callback(
      std::bind(&PointsClustering::parametersCallback, this, std::placeholders::_1));

}

PointsClustering::~PointsClustering() {}

void PointsClustering::timer_callback() {
  auto now = this->now();

  bool has_front_points = this->bFrontPoints; // && 
    // ((now - this->m_front_points_last_update) < rclcpp::Duration(1, 0));
  bool has_right_points = this->bRightPoints; // && 
    // ((now - this->m_right_points_last_update) < rclcpp::Duration(1, 0));
  bool has_left_points = this->bLeftPoints; // && 
    // ((now - this->m_left_points_last_update) < rclcpp::Duration(1, 0));
  bool has_inverse_map_Points = this->bInverseMapPoints; // && 
    // ((now - this->m_inverse_map_points_last_update) < rclcpp::Duration(1, 0));

  pcl::PointCloud<pcl::PointXYZI>::Ptr target_pcl;

  if (!this->m_use_inverse_map) {
    //simple height map filter
    m_simpleHeightmapPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    
    { // filter front points
    std::lock_guard<std::mutex> sensor_lock(sensor_mtx_f);
    if(has_front_points && m_inPoints && !m_inPoints->points.empty()) {

      for (auto point : m_inPoints->points)
      {
        pcl::PointXYZI point_buf;
        point_buf = point;
        point_buf.x = point.x + 0.9214; // ???
        point_buf.z = point.z + 0.212; // ???
    
        if (point.z > m_height_filter_thres && point.z < 2.0 && fabs(point.y) < 20.0) {
          m_simpleHeightmapPoints->points.push_back(point_buf);
        }
      }
    }
    }

    { // filter right points
    std::lock_guard<std::mutex> sensor_lock(sensor_mtx_r);
    if(has_right_points && m_RightPoints && !m_RightPoints->points.empty())
    {
      
      for (auto point : m_RightPoints->points) {
        if (point.z > m_height_filter_thres && point.z < 2.0 && 
            fabs(point.y) > 0.5 && fabs(point.y) < 20.0 && 
            point.x > -2.0 && point.x < -0.5) {
          m_simpleHeightmapPoints->points.push_back(point);
        }
      }
    }
    }

    { // filter left points
    std::lock_guard<std::mutex> sensor_lock(sensor_mtx_l);
    if (has_left_points && m_LeftPoints && !m_LeftPoints->points.empty()) {

      for (auto point : m_LeftPoints->points) {
        if (point.z > m_height_filter_thres && point.z < 2.0 && 
            fabs(point.y) < 20.0 && 
            point.x > -5.0 && point.x < 0.5) {
          m_simpleHeightmapPoints->points.push_back(point);
        }
      }
    }
    }

    sensor_msgs::msg::PointCloud2 cloud_simple_heightmap_msg;
    pcl::toROSMsg(*m_simpleHeightmapPoints, cloud_simple_heightmap_msg);
    cloud_simple_heightmap_msg.header.frame_id =
        nif::common::frame_id::localization::BASE_LINK;
    cloud_simple_heightmap_msg.header.stamp = this->now();
    pubSimpleheightMap->publish(cloud_simple_heightmap_msg);

    target_pcl = m_simpleHeightmapPoints;
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr concatPoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    { // concat front points
      std::lock_guard<std::mutex> sensor_lock(sensor_mtx_f);
      if(has_front_points && m_inPoints && !m_inPoints->points.empty()) {
        
        *concatPoints += *m_inPoints;
      }
    }

    { // concat left points
      std::lock_guard<std::mutex> sensor_lock(sensor_mtx_r);
      if(has_right_points && m_RightPoints && !m_RightPoints->points.empty())
      {
        
        *concatPoints += *m_RightPoints;
      }
    }

    { // filter left points
      std::lock_guard<std::mutex> sensor_lock(sensor_mtx_l);
      if (has_left_points && m_LeftPoints && !m_LeftPoints->points.empty()) {

        *concatPoints += *m_LeftPoints;
      }
    }

    float min_x = -(front_upper_distance_ + rear_upper_distance_) / 2;
    float min_y = -(left_upper_distance_ + right_upper_distance_) / 2;


    pcl::PointCloud<pcl::PointXYZI>::Ptr egoShapeFilteredPoints(
        new pcl::PointCloud<pcl::PointXYZI>);

    EgoShape(concatPoints, egoShapeFilteredPoints, 
      left_lower_distance_, right_lower_distance_, front_lower_distance_, rear_lower_distance_,
      left_upper_distance_, right_upper_distance_, front_upper_distance_, rear_upper_distance_, 
      height_lower_distance_, height_upper_distance_);

    /* REGISTER POINTS TO GRID 
    1. Register points on the 2-d grid map for ground-filtering
    2. Visualize the occupancy grid map
      - input : ego-shape & voxelized points, grid resolution, origin point of grid
      - output : 2-D grid map
    */
    RegisterPointToGrid(egoShapeFilteredPoints, resolution_, min_x, min_y);

    /* INVERSE MAP
    1. Find the ground filtered points
    2. Find the left/right filtered points
      - input : ego-shape & voxelized points, grid resolution, origin point of grid
      - output : Inverse mapped filtered points (Both, Left, Right)
    */
    m_inverseMapPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    InverseMap( egoShapeFilteredPoints, m_inverseMapPoints, 
                min_x, min_y, resolution_);

    sensor_msgs::msg::PointCloud2 cloud_inverse_msg;
    pcl::toROSMsg(*m_inverseMapPoints, cloud_inverse_msg);
    cloud_inverse_msg.header.frame_id = BASE_LINK;
    cloud_inverse_msg.header.stamp = this->now();
    pubInverseMappedPoints->publish(cloud_inverse_msg);

    target_pcl = m_inverseMapPoints;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr registeredPoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr inflatedPoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr centerPoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  nif::common::msgs::PerceptionResultList perception_msg{};

  visualization_msgs::msg::MarkerArray clustered_array;
  clusterAndColorGpu(target_pcl, registeredPoints,
                    clustered_array, m_max_cluster_distance);

  int i = 0;
  perception_msg.perception_list.resize(clustered_array.markers.size());
  for (auto marker : clustered_array.markers)
  {
    pcl::PointXYZI point_buf;
    point_buf.x = marker.pose.position.x;
    point_buf.y = marker.pose.position.y;
    centerPoints->points.push_back(point_buf);

    perception_msg.perception_list[i].header = marker.header;
    perception_msg.perception_list[i].detection_result_3d.center = marker.pose;

    i += 1;
  }

  createGaussianWorld(clustered_array, 7.0, 3.0, inflatedPoints);

  sensor_msgs::msg::PointCloud2 cloud_clustered_msg;
  pcl::toROSMsg(*registeredPoints, cloud_clustered_msg);
  cloud_clustered_msg.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  cloud_clustered_msg.header.stamp = this->now();
  pubClusterPoints->publish(cloud_clustered_msg);

  sensor_msgs::msg::PointCloud2 cloud_inflated_msg;
  pcl::toROSMsg(*inflatedPoints, cloud_inflated_msg);
  cloud_inflated_msg.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  cloud_inflated_msg.header.stamp = this->now();
  pubInflationPoints->publish(cloud_inflated_msg);

  sensor_msgs::msg::PointCloud2 cloud_cluster_center_msg;
  pcl::toROSMsg(*centerPoints, cloud_cluster_center_msg);
  cloud_cluster_center_msg.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  cloud_cluster_center_msg.header.stamp = this->now();
  pubClusteredCenterPoints->publish(cloud_cluster_center_msg);

  pubClusteredArray->publish(clustered_array);

  perception_msg.header = cloud_cluster_center_msg.header;
  pubPerceptionList->publish(perception_msg);
}

void PointsClustering::PointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  std::lock_guard<std::mutex> sensor_lock(sensor_mtx_f);
  m_inPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  this->m_front_points_last_update = this->now();
  // pcl::PointCloud<pcl::PointXYZI>::Ptr originPoints(
  //     new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *m_inPoints);
  // m_inPoints = downsample(m_inPoints, resolution_);
  downsample(m_inPoints, resolution_);

  nif::perception::tools::transformLuminarPointCloudCustom(
    *m_inPoints, *m_inPoints, transfrom_list[0]);

  bFrontPoints = true;

  // std::cout << "callback" << std::endl;
}

void PointsClustering::LeftPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx_l);

  m_LeftPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  this->m_left_points_last_update = this->now();
  pcl::fromROSMsg(*msg, *m_LeftPoints);

  downsample(m_LeftPoints, resolution_);

  nif::perception::tools::transformLuminarPointCloudCustom(
    *m_LeftPoints, *m_LeftPoints, transfrom_list[1]);

  bLeftPoints = true;
}

void PointsClustering::RightPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx_r);

  m_RightPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  this->m_right_points_last_update = this->now();
  pcl::fromROSMsg(*msg, *m_RightPoints);

  downsample(m_RightPoints, resolution_);

  nif::perception::tools::transformLuminarPointCloudCustom(
    *m_RightPoints, *m_RightPoints, transfrom_list[2]);

  bRightPoints = true;
}

void PointsClustering::inverseMapPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx_im);

  m_inverseMapPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  this->m_inverse_map_points_last_update = this->now();
  pcl::fromROSMsg(*msg, *m_inverseMapPoints);

  bInverseMapPoints = true;
}

void PointsClustering::clusterAndColorGpu(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    visualization_msgs::msg::MarkerArray& out_clustered_array,
    double in_max_cluster_distance) {

#ifdef GPU_CLUSTERING
  if (in_cloud_ptr && !in_cloud_ptr->points.empty())
  {
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
  }

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

  if (length < 7.0 && width < 7.0 && height > 0.1) {
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

void
PointsClustering::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                             double resolution) {
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
  //     new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, 0.02);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*cloud);
}

// Calculate gaussian interpolation.
void PointsClustering::createGaussianWorld(
    visualization_msgs::msg::MarkerArray& marker_array_in, double inflation_x,
    double inflation_y, pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out) {

  struct Gaussian {
    double x0, y0;
    double varX, varY;
    double s;
  };

  AnalyticalFunctions func;
  std::vector<std::pair<double, double>> vars;
  std::vector<std::pair<double, double>> means;
  std::vector<double> scales;
  std::vector<Gaussian> g;

  for (auto marker : marker_array_in.markers) {
    Gaussian gaussian_tmp;
    gaussian_tmp.x0 = marker.pose.position.x;
    gaussian_tmp.y0 = marker.pose.position.y;
    gaussian_tmp.varX = inflation_x;
    gaussian_tmp.varY = inflation_y;
    gaussian_tmp.s = 1 / (inflation_x + 0.00001);
    g.push_back(gaussian_tmp);
  }

  func.f_ = [g](double x, double y) {
    double value = 0.0;
    for (int i = 0; i < g.size(); ++i) {
      const double x0 = g.at(i).x0;
      const double y0 = g.at(i).y0;
      const double varX = g.at(i).varX;
      const double varY = g.at(i).varY;
      const double s = g.at(i).s;
      value += s * std::exp(-(x - x0) * (x - x0) / (2.0 * varX + 0.00001) -
                            (y - y0) * (y - y0) / (2.0 * varY + 0.00001));
    }
    return value;
  };

  for (auto marker : marker_array_in.markers) {
    for (double i = -inflation_x - marker.scale.x / 2;
         i < inflation_x + marker.scale.x / 2; i = i + 0.5) {
      for (double j = -inflation_y - marker.scale.y / 2;
           j < inflation_y + marker.scale.y / 2; j = j + 0.5) {
        pcl::PointXYZI point_buf;
        point_buf.x = marker.pose.position.x + i;
        point_buf.y = marker.pose.position.y + j;
        point_buf.intensity = func.f_(point_buf.x, point_buf.y);
        if(point_buf.intensity < fabs(1.0))      
          points_out->points.push_back(point_buf);
      }
    }
  }

  // return output;
}


void PointsClustering::RegisterPointToGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
    double in_resolution, float min_x, float min_y) {
  // nav_msgs::msg::OccupancyGrid oc_grid_msg;
  // oc_grid_msg.header.frame_id = BASE_LINK;
  // oc_grid_msg.header.stamp = this->now();
  // oc_grid_msg.info.origin.position.x = min_x;
  // oc_grid_msg.info.origin.position.y = min_y;
  // oc_grid_msg.info.origin.position.z = 0.;
  // oc_grid_msg.info.origin.orientation.x = 0.;
  // oc_grid_msg.info.origin.orientation.y = 0.;
  // oc_grid_msg.info.origin.orientation.z = 0.;
  // oc_grid_msg.info.origin.orientation.w = 1.;
  // oc_grid_msg.info.width = MAP_WIDTH;
  // oc_grid_msg.info.height = MAP_HEIGHT;
  // oc_grid_msg.info.resolution = in_resolution;

  for (int i = 0; i < MAP_HEIGHT + 1; i++) {
    // this->map[i].fill(0.0);
    this->count_map[i].fill(0.0);
    // this->mean_map[i].fill(0.0);
    // this->cov_map[i].fill(0.0);
    // this->points_map[i].fill({});
  }

  int x, y;
  for (auto point_buf : in_cloud_ptr->points) {
    x = std::floor((point_buf.x - min_x) / in_resolution);
    y = std::floor((point_buf.y - min_y) / in_resolution);

    if (x > MAP_WIDTH || x < 0 || y > MAP_HEIGHT || y < 0) {
      // std::cout << x << ", " << y << std::endl;
      continue;
    }
    // count_map[y][x] = count_map[y][x] + 1.f; // count hit

    // std::vector<double> points_map_buf = points_map[y][x];
    // points_map_buf.push_back(point_buf.z);
    count_map[y][x] = count_map[y][x] + 1.f; // count hit
    // points_map[y][x] = points_map_buf;
    // map[y][x] = map[y][x] + point_buf.z + 0.5; // accumulate height
    // double sum_of_elems =
    //     std::accumulate(points_map_buf.begin(), points_map_buf.end(),
    //                     decltype(points_map_buf)::value_type(0));

    // if(count_map[y][x] != 0.)
    // {
    //   mean_map[y][x] = sum_of_elems / count_map[y][x]; // calculate mean map
    //   for(auto z : points_map_buf)
    //   {
    //     cov_map[y][x] = cov_map[y][x] * count_map[y][x]; 
    //     cov_map[y][x] = (cov_map[y][x] + pow((z - mean_map[y][x]),2)) / count_map[y][x];
    //   }
    // }
  }

  // for (int i = 0; i < MAP_HEIGHT; i++) {
  //   for (int j = 0; j < MAP_WIDTH; j++) {
  //     if (count_map[i][j] > count_threshold_) {
  //       // oc_grid_msg.data.push_back((int8_t)(map[i][j]));
  //       oc_grid_msg.data.push_back((int8_t)(80.));
  //     } else {
  //       oc_grid_msg.data.push_back((int8_t)(0.0));
  //     }
  //   }
  // }
  // pub_oc_grid->publish(oc_grid_msg);
}

void PointsClustering::InverseMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut,
    float min_x, float min_y, float in_resolution) {
  int x, y;
  for (auto&& point_buf : cloudIn->points) {
    
    x = std::floor((point_buf.x - min_x) / in_resolution);
    y = std::floor((point_buf.y - min_y) / in_resolution);
    
    if (count_map[y][x] > count_threshold_) {
      cloudOut->points.push_back(point_buf);
    }
  }
}

void PointsClustering::EgoShape(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    double in_left_lower_threshold, 
    double in_right_lower_threshold,
    double in_front_lower_threshold,
    double in_rear_lower_threshold, // lower limit
    double in_left_upper_threshold, 
    double in_right_upper_threshold,
    double in_front_upper_threshold,
    double in_rear_upper_threshold, // upper limit
    double in_height_lower_threshold, 
    double in_height_upper_threshold) {
  
  pcl::PointIndices::Ptr out_indices_list(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    auto& x = in_cloud_ptr->points[i].x;
    auto& y = in_cloud_ptr->points[i].y;
    auto& z = in_cloud_ptr->points[i].z;

    // outer side boundaries
    if (y > in_left_upper_threshold ||
        y < -in_right_upper_threshold) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    // outer front/rear boundaries
    if (x > in_front_upper_threshold ||
        x < -in_rear_upper_threshold) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    if (z < (in_height_lower_threshold)) {
      out_indices_list->indices.push_back(i);
      continue;
    }
    if (z > (in_height_upper_threshold)) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    if ( y < in_left_lower_threshold  && y > -1.0 * in_right_lower_threshold &&
         x < in_front_lower_threshold && x > -1.0 * in_rear_lower_threshold )
      {
        out_indices_list->indices.push_back(i);
        continue;
      }
      
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(out_indices_list);
  extract.setNegative(
      true); // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

rcl_interfaces::msg::SetParametersResult
PointsClustering::parametersCallback(const std::vector<rclcpp::Parameter> &vector) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto &param : vector) {
    if (param.get_name() == "cluster_size_min") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        // TODO // @DEBUG implement constraints
        if (true) {
          this->m_cluster_size_min = param.as_int();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "cluster_size_max") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        // TODO // @DEBUG implement constraints
        if (true) {
          this->m_cluster_size_max = param.as_int();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "max_cluster_distance") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        // TODO // @DEBUG implement constraints
        if (true) {
          this->m_max_cluster_distance = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "height_filter_thres") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        // TODO // @DEBUG implement constraints
        if (true) {
          this->m_height_filter_thres = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "use_inverse_map") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        // TODO // @DEBUG implement constraints
        if (true) {
          this->m_use_inverse_map = param.as_bool();
          result.successful = true;
        }
      }
    } 
  }
  return result;
}
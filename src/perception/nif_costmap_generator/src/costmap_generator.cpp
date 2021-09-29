/*
 * costmap_generator.cpp
 *
 *  Created on: Aug 9, 2021
 *      Author: Daegyu Lee
 */

#include <costmap_generator/costmap_generator.h>

using namespace message_filters;
using namespace std::placeholders;
using namespace nif::perception::costmap;
using namespace nif::common::frame_id::localization;

// Constructor
CostmapGenerator::CostmapGenerator()
    : Node("nif_costmap_generator_node"),
      SENSOR_POINTS_COSTMAP_LAYER_("sensor_points"),
      COMBINED_COSTMAP_LAYER_("costmap"), INFLATION_COSTMAP_LAYER_("inflation")
{
  //   : nh_(nh), private_nh_(private_nh), has_subscribed_wayarea_(false),
  //     LANE_POINTS_COSTMAP_LAYER_("lane"), LASER_2D_COSTMAP_LAYER_("laser_scan"),
  //     BOUNDING_BOX_COSTMAP_LAYER_("bounding_boxes"),
  //     VISUAL_COSTMAP_LAYER_("visual"),

  this->declare_parameter<std::string>("lidar_frame", "center_of_gravity");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<double>("grid_min_value", 0.0);
  this->declare_parameter<double>("grid_max_value", 1.0);
  this->declare_parameter<double>("grid_resolution", 0.2);
  this->declare_parameter<double>("grid_length_x", 50);
  this->declare_parameter<double>("grid_length_y", 30);
  this->declare_parameter<double>("grid_position_x", 0);
  this->declare_parameter<double>("grid_position_y", 0);
  this->declare_parameter<double>("maximum_lidar_height_thres", 0.3);
  this->declare_parameter<double>("minimum_lidar_height_thres", -0.5);
  this->declare_parameter<double>("maximum_laserscan_distance_thres", 50);
  this->declare_parameter<double>("minimum_laserscan_distance_thres", 0.1);
  this->declare_parameter<bool>("use_points", true);
  this->declare_parameter<bool>("enable_potential", false);

  this->lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  this->map_frame_ = this->get_parameter("map_frame").as_string();
  this->grid_min_value_ = this->get_parameter("grid_min_value").as_double();
  this->grid_max_value_ = this->get_parameter("grid_max_value").as_double();
  this->grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  this->grid_length_x_ = this->get_parameter("grid_length_x").as_double();
  this->grid_length_y_ = this->get_parameter("grid_length_y").as_double();
  this->grid_position_x_ = this->get_parameter("grid_position_x").as_double();
  this->grid_position_y_ = this->get_parameter("grid_position_y").as_double();
  this->maximum_lidar_height_thres_ = this->get_parameter("maximum_lidar_height_thres").as_double();
  this->minimum_lidar_height_thres_ = this->get_parameter("minimum_lidar_height_thres").as_double();
  this->maximum_laserscan_distance_thres_ = this->get_parameter("maximum_laserscan_distance_thres").as_double();
  this->minimum_laserscan_distance_thres_ = this->get_parameter("minimum_laserscan_distance_thres").as_double();
  this->use_points_ = this->get_parameter("use_points").as_bool();
  this->bEnablePotential_ = this->get_parameter("enable_potential").as_bool();

  pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "out_map_topic_name", nif::common::constants::QOS_SENSOR_DATA);
  pub_points_on_global_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points_on_global", nif::common::constants::QOS_SENSOR_DATA);
  pub_points_on_track_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points_on_track", nif::common::constants::QOS_SENSOR_DATA);

  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(10ms, std::bind(&CostmapGenerator::timer_callback, this));
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "weaker_thres_inverse_mapped_points",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&CostmapGenerator::sensorPointsCallback, this,
                std::placeholders::_1));

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "in_odometry_ekf_estimated", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&CostmapGenerator::OdometryCallback, this,
                std::placeholders::_1));

  sub_closest_geofence_idx_ = this->create_subscription<std_msgs::msg::Int32>(
      "/int/closest_geofence_idx", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&CostmapGenerator::ClosestGeofenceIndexCallback, this,
                std::placeholders::_1));

  auto rmw_qos_profile =
      nif::common::constants::QOS_EGO_ODOMETRY.get_rmw_qos_profile();

  sub_InnerGeofence.subscribe(this, "/geofence_inner", rmw_qos_profile);
  sub_OuterGeofence.subscribe(this, "/geofence_outer", rmw_qos_profile);

  m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_InnerGeofence, sub_OuterGeofence);

  m_sync->registerCallback(std::bind(
      &CostmapGenerator::MessegefilteringCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  costmap_ = initGridmap();
  // RoadBoundarycostmap_ = initBoundaryGridmap();
}

CostmapGenerator::~CostmapGenerator(){}

void CostmapGenerator::run() {

  if (bGeoFence && bPoints && bOdometry)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointsOnGlobal(
        new pcl::PointCloud<pcl::PointXYZI>);
    TransformPointsToGlobal(m_in_sensor_points, PointsOnGlobal, m_veh_x, m_veh_y, m_veh_yaw);

    sensor_msgs::msg::PointCloud2 points_on_global_msg;
    pcl::toROSMsg(*PointsOnGlobal, points_on_global_msg);
    points_on_global_msg.header.stamp = this->now();
    points_on_global_msg.header.frame_id = ODOM;
    pub_points_on_global_->publish(points_on_global_msg);
    std::cout << "points size : " << PointsOnGlobal->points.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointsOnTrackGlobal(
        new pcl::PointCloud<pcl::PointXYZI>);
    SearchPointsOntrack(m_InnerGeoFence, m_OuterGeoFence,
                        m_closestGeofenceIndex, PointsOnGlobal, PointsOnTrackGlobal);
    std::cout << "tarck points size : " << PointsOnTrackGlobal->points.size() << std::endl;

    std::cout << "m_InnerGeoFence: " << m_InnerGeoFence.size() << std::endl; 
    std::cout << "m_OuterGeoFence: " << m_OuterGeoFence.size() << std::endl;

    sensor_msgs::msg::PointCloud2 points_on_track_msg;
    pcl::toROSMsg(*PointsOnTrackGlobal, points_on_track_msg);
    points_on_track_msg.header.stamp = this->now();
    points_on_track_msg.header.frame_id = ODOM;
    pub_points_on_track_->publish(points_on_track_msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr PointsOnTrackBody(
        new pcl::PointCloud<pcl::PointXYZI>);
    TransformPointsToBody(PointsOnTrackGlobal, PointsOnTrackBody, 
                          m_veh_x, m_veh_y, m_veh_yaw);
  }

  if(bEnablePotential_)
    MakeInflationWithPoints();

  generateCombinedCostmap();
  publishRosMsg(&costmap_);
  // publishRoadBoundaryMsg(&RoadBoundarycostmap_);
}

void CostmapGenerator::timer_callback() {
  run();
}

void CostmapGenerator::sensorPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!use_points_) {
    return;
  }
  m_in_sensor_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::fromROSMsg(*msg, *m_in_sensor_points);

  costmap_[SENSOR_POINTS_COSTMAP_LAYER_] =
      generateSensorPointsCostmap(m_in_sensor_points);
  m_in_header = msg->header;
  bPoints = true;

  std::cout << "points received" << std::endl;
}

void CostmapGenerator::OdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
  m_veh_x = msg->pose.pose.position.x;
  m_veh_y = msg->pose.pose.position.y;

  tf2::Quaternion tf_quat;
  tf2::convert(msg->pose.pose.orientation, tf_quat);
  tf2::Matrix3x3 mat(tf_quat);
  mat.getRPY(m_veh_roll, m_veh_pitch, m_veh_yaw);
  bOdometry = true;

  std::cout << "odometry received" << std::endl;
}

void CostmapGenerator::MessegefilteringCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inner_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &outer_msg)
{
  m_inner_geofence_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_outer_geofence_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::fromROSMsg(*inner_msg, *m_inner_geofence_points);
  pcl::fromROSMsg(*outer_msg, *m_outer_geofence_points);

  if (bGeoFence)
    return;

  m_InnerGeoFence.clear();
  m_OuterGeoFence.clear();
  for (auto point : m_inner_geofence_points->points) {
    std::pair<double, double> xy_buf;
    xy_buf.first = point.x;
    xy_buf.second = point.y;
    m_InnerGeoFence.push_back(xy_buf);
  }
  std::pair<double, double> first_point_inner;
  first_point_inner.first = m_inner_geofence_points->points[0].x;
  first_point_inner.second = m_inner_geofence_points->points[0].y;
  m_InnerGeoFence.push_back(first_point_inner);

  for (auto point : m_outer_geofence_points->points) {
    std::pair<double, double> xy_buf;
    xy_buf.first = point.x;
    xy_buf.second = point.y;
    m_OuterGeoFence.push_back(xy_buf);
  }
  std::pair<double, double> first_point_outer;
  first_point_outer.first = m_outer_geofence_points->points[0].x;
  first_point_outer.second = m_outer_geofence_points->points[0].y;
  m_OuterGeoFence.push_back(first_point_outer);
  bGeoFence = true;

  std::cout << "geofence received" << std::endl;
}

void CostmapGenerator::ClosestGeofenceIndexCallback(
    const std_msgs::msg::Int32::SharedPtr msg) {
  m_closestGeofenceIndex = msg->data;
}

void CostmapGenerator::TransformPointsToGlobal(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut, 
    const double &veh_x_, const double &veh_y_, const double &veh_yaw_) {

    for (auto point : CloudIn->points) {
    pcl::PointXYZI pointOnGlobal;
    pointOnGlobal.x = point.x * cos(veh_yaw_) - point.y * sin(veh_yaw_) + veh_x_;
    pointOnGlobal.y = point.x * sin(veh_yaw_) + point.y * cos(veh_yaw_) + veh_y_;
    pointOnGlobal.z = point.z;
    double dist =
        sqrt(pow(point.x, 2) + pow(point.y, 2));
    pointOnGlobal.intensity = dist;

    if(dist < 70.0)
    {
      CloudOut->points.push_back(pointOnGlobal);
    }
  }
}

void CostmapGenerator::TransformPointsToBody(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut, const double &veh_x_,
    const double &veh_y_, const double &veh_yaw_) {

  for (auto point : CloudIn->points) {
    pcl::PointXYZI pointOnGlobal;
    pointOnGlobal.x = (point.x - veh_x_) * cos(veh_yaw_) + (point.y -veh_y_) * sin(veh_yaw_);
    pointOnGlobal.y = -(point.x - veh_x_) * sin(veh_yaw_) + (point.y -  veh_y_) * cos(veh_yaw_);
    pointOnGlobal.z = point.z;
    CloudOut->points.push_back(pointOnGlobal);
  }
}

void CostmapGenerator::MakeInflationWithPoints()
{
  if(!bPoints)
    return;

  costmap_[INFLATION_COSTMAP_LAYER_].setConstant(grid_min_value_);
  obstacleArray.clear();
  int idx = 0;
  for (grid_map::GridMapIterator iterator(costmap_);
  !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    costmap_.getPosition(index, pos);
    if(costmap_[SENSOR_POINTS_COSTMAP_LAYER_](index(0), index(1)) != 0 )
    {
      std::pair<double, double> pointBuf;
      pointBuf.first = pos.x();
      pointBuf.second = pos.y();
      obstacleArray.push_back(pointBuf);
    }
    idx ++;
  }
  costmap_[INFLATION_COSTMAP_LAYER_] = createGaussianWorld(&costmap_,
  INFLATION_COSTMAP_LAYER_, 3.0, 3.0, obstacleArray);

}

    // void CostmapGenerator::laneBoundaryCallback(
    //     const sensor_msgs::PointCloud2::ConstPtr
    //     &in_lane_points_on_global_msg) {
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr in_lane_points_on_global(
    //       new pcl::PointCloud<pcl::PointXYZI>);
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr in_lane_points_on_body(
    //       new pcl::PointCloud<pcl::PointXYZI>);
    //   pcl::fromROSMsg(*in_lane_points_on_global_msg,
    //   *in_lane_points_on_global);

    //   double roll, pitch, yaw;
    //   tf::Quaternion q;
    //   tf::quaternionMsgToTF(m_odom.pose.pose.orientation, q);
    //   tf::Matrix3x3 m(q);
    //   m.getRPY(roll, pitch, yaw);

    //   for (auto point : in_lane_points_on_global->points) {
    //     double local_x = (point.x - m_odom.pose.pose.position.x) * cos(yaw) +
    //                      (point.y - m_odom.pose.pose.position.y) * sin(yaw);
    //     double local_y = -(point.x - m_odom.pose.pose.position.x) * sin(yaw)
    //     +
    //                      (point.y - m_odom.pose.pose.position.y) * cos(yaw);

    //     pcl::PointXYZI current_point;
    //     current_point.x = local_x;
    //     current_point.y = local_y;
    //     in_lane_points_on_body->points.push_back(current_point);
    //   }

    //   RoadBoundarycostmap_[LANE_POINTS_COSTMAP_LAYER_] =
    //       lane2costmap_.makeCostmapFromSensorPoints(
    //           1, -1, grid_min_value_, grid_max_value_, costmap_,
    //           LANE_POINTS_COSTMAP_LAYER_, in_lane_points_on_body);

    //   bRoadBoundary = true;
    // }

    // void CostmapGenerator::BoundingBoxesCallback(
    //     const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &in_boxes) {

    //   costmap_[BOUNDING_BOX_COSTMAP_LAYER_] =
    //   generateBBoxesCostmap(in_boxes);
    //   // bBoundingBox = true;
    //   bBoundingBox = true;
    // }

    // void CostmapGenerator::VisualCallback(const
    // detection_msgs::BoundingBoxArrayConstPtr& msg)
    // {
    //   costmap_[VISUAL_COSTMAP_LAYER_] = generateVisualCostmap(msg);
    //   bVisual = true;
    // }

    // void CostmapGenerator::OdometryCallback(
    //     const nav_msgs::Odometry::ConstPtr &msg) {
    //   m_odom = *msg;
    // }

    // void CostmapGenerator::LocalWaypointCallback(
    //     const nav_msgs::PathConstPtr &msg) {
    //   m_LocalPathOnBody = *msg;
    // }

    // // Create the map using length, resolution, pose.
    grid_map::GridMap CostmapGenerator::initGridmap() {
  grid_map::GridMap map;

  map.setFrameId(lidar_frame_);
  map.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
                  grid_resolution_,
                  grid_map::Position(grid_position_x_, grid_position_y_));

  map.add(SENSOR_POINTS_COSTMAP_LAYER_, grid_min_value_);
  map.add(INFLATION_COSTMAP_LAYER_, grid_min_value_);
  // map.add(LASER_2D_COSTMAP_LAYER_, grid_min_value_);
  // map.add(BOUNDING_BOX_COSTMAP_LAYER_, grid_min_value_);
  // map.add(VISUAL_COSTMAP_LAYER_, grid_min_value_);

  map.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);

  return map;
}

grid_map::Matrix CostmapGenerator::generateSensorPointsCostmap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_points) {

  grid_map::Matrix sensor_points_costmap =
      points2costmap_.makeCostmapFromSensorPoints(
          maximum_lidar_height_thres_, minimum_lidar_height_thres_,
          grid_min_value_, grid_max_value_, costmap_,
          SENSOR_POINTS_COSTMAP_LAYER_, in_sensor_points);
  return sensor_points_costmap;
}


// grid_map::Matrix CostmapGenerator::generateBBoxesCostmap(
//     const jsk_recognition_msgs::BoundingBoxArrayConstPtr &in_boxes) {
//   grid_map::Matrix bboxes_costmap = bboxes2costmap_.makeCostmapFromBBoxes(
//       costmap_, expand_polygon_size_, size_of_expansion_kernel_, in_boxes);
//   return bboxes_costmap;
// }


void CostmapGenerator::generateCombinedCostmap() {
  //   // assuming combined_costmap is calculated by element wise max operation

  costmap_[COMBINED_COSTMAP_LAYER_].setConstant(grid_min_value_);
  if (bPoints) {
    costmap_[COMBINED_COSTMAP_LAYER_] =
        costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
            costmap_[SENSOR_POINTS_COSTMAP_LAYER_]);
    costmap_[COMBINED_COSTMAP_LAYER_] =
        costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
            costmap_[INFLATION_COSTMAP_LAYER_]);
  }
  // if (bBoundingBox) {
  //   costmap_[COMBINED_COSTMAP_LAYER_] =
  //       costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
  //           costmap_[BOUNDING_BOX_COSTMAP_LAYER_]);
  // }

  // if(bVisual){
  //   costmap_[COMBINED_COSTMAP_LAYER_] =
  //       costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
  //           costmap_[VISUAL_COSTMAP_LAYER_]);

}

pcl::PointCloud<pcl::PointXYZI>::Ptr
CostmapGenerator::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

void CostmapGenerator::publishRosMsg(grid_map::GridMap *map) {
  nav_msgs::msg::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
      *map, COMBINED_COSTMAP_LAYER_, grid_min_value_, grid_max_value_,
      out_occupancy_grid);
  out_occupancy_grid.header = m_in_header;
  out_occupancy_grid.header.frame_id = lidar_frame_;
  pub_occupancy_grid_->publish(out_occupancy_grid);
}

// void CostmapGenerator::publishRoadBoundaryMsg(grid_map::GridMap *map) {
//   nav_msgs::OccupancyGrid out_occupancy_grid;
//   grid_map::GridMapRosConverter::toOccupancyGrid(
//       *map, COMBINED_COSTMAP_LAYER_, grid_min_value_, grid_max_value_,
//       out_occupancy_grid);
//   out_occupancy_grid.header = m_in_header;
//   out_occupancy_grid.header.frame_id = lidar_frame_;
//   pub_road_occupancy_grid_.publish(out_occupancy_grid);
// }


//Calculate gaussian interpolation.
grid_map::Matrix CostmapGenerator::createGaussianWorld(grid_map::GridMap *map, const std::string layer_name, 
                                                         double inflation_x, double inflation_y,
                                                         const std::vector<std::pair<double, double>>& pointArray)
{
  struct Gaussian
  {
    double x0, y0;
    double varX, varY;
    double s;
  };

  AnalyticalFunctions func;
  std::vector<std::pair<double, double>> vars;
  std::vector<std::pair<double, double>> means;
  std::vector<double> scales;
  std::vector<Gaussian> g;

  for(auto point : pointArray)
  {
    Gaussian gaussian_tmp;
    gaussian_tmp.x0 = point.first;
    gaussian_tmp.y0 = point.second;
    gaussian_tmp.varX = inflation_x; 
    gaussian_tmp.varY = inflation_y;
    gaussian_tmp.s =  1 / inflation_x;
    g.push_back(gaussian_tmp);
  }

  func.f_ = [g](double x,double y) {
    double value = 0.0;
    for (int i = 0; i < g.size(); ++i) {
      const double x0 = g.at(i).x0;
      const double y0 = g.at(i).y0;
      const double varX = g.at(i).varX;
      const double varY = g.at(i).varY;
      const double s = g.at(i).s;
      value += s * std::exp(-(x-x0)*(x-x0) / (2.0 * varX) - (y-y0)*(y-y0) / (2.0 * varY));
    }
    return value;
  };

  return fillGridMap(map, layer_name, func);
  
  // return output;
}

grid_map::Matrix CostmapGenerator::fillGridMap(grid_map::GridMap *map, const std::string layer_name, 
                                              const AnalyticalFunctions &functions)
{
  grid_map::Matrix& data = (*map)[layer_name];
  double max = 0;
  for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    grid_map::Position pos;
    map->getPosition(index, pos);
    data(index(0), index(1)) =  functions.f_(pos.x(), pos.y()) * grid_resolution_ / 10;
    if(max < data(index(0), index(1)))
      max = data(index(0), index(1));
  }
  // std::cout << max << std::endl;
  return data;
}

void CostmapGenerator::SearchPointsOntrack(
    const std::vector<std::pair<double, double>> &inner_array_in,
    const std::vector<std::pair<double, double>> &outer_array_in,
    const int& closest_idx,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut) {

  bool first_point_for_inner = true;
  bool first_point_for_outer = true;

  int size = inner_array_in.size();

  std::cout << "inner_array_in: " << inner_array_in.size() << std::endl;
  std::cout << "closest_idx: "<< closest_idx << std::endl;

  double prev_geofence_x, prev_geofence_y;
  pcl::PointCloud<pcl::PointXYZI>::Ptr inner_boundary_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);

  // Sequentially find the points located on the racing track
  // Firstly, search the points from inner boundary condition
  // INPUT : Origin points
  // OUTPUT :  inner_boudnary filtered points
  for (int i = -1; i < 2; i++) {
    int idx = closest_idx + i + size;
    idx = idx % size;
    std::cout << idx << "  ";
  }
  int cnt = 0;
  
  for (auto point_buf : CloudIn->points) {
    bool finish_loop = false;

    for (int i = -1; i < 2; i++) {
      int idx = (closest_idx + i + size) % size;
      int idx_prev = ((closest_idx + i +size) - 1) % size;

      // std::cout << idx << ", " << idx_prev << std::endl;

      auto geofence_xy = inner_array_in[idx];
      auto prev_geofence_xy = inner_array_in[idx_prev];

      double slope;
      if (geofence_xy.first - prev_geofence_xy.first == 0) {
        slope = 0;
      } else {
        slope = (geofence_xy.second - prev_geofence_xy.second) /
                (geofence_xy.first - prev_geofence_xy.first);
      }
      double bias = geofence_xy.second - slope * geofence_xy.first;
      double normal_distance = fabs(slope * point_buf.x - point_buf.y + bias) / sqrt(pow(slope, 2) + 1);

      double prod1 = (point_buf.x - prev_geofence_xy.first) * (geofence_xy.first - prev_geofence_xy.first) +
                      (point_buf.y - prev_geofence_xy.second) * (geofence_xy.second - prev_geofence_xy.second);
      double prod2 = (point_buf.x - geofence_xy.first) * (prev_geofence_xy.first - geofence_xy.first) +
                      (point_buf.y - geofence_xy.second) * (prev_geofence_xy.second - geofence_xy.second);

      double cross_prod_z = ((prev_geofence_xy.first - point_buf.x) *
                                  (geofence_xy.second - point_buf.y) -
                              (prev_geofence_xy.second - point_buf.y) *
                                  (geofence_xy.first - point_buf.x));

      // std::cout << geofence_xy.first << ", "
      //           << geofence_xy.second << ", "
      //           << prev_geofence_xy.first << ", "
      //           << prev_geofence_xy.second << std::endl;

      // put the points which are posed on the right from inner boundary
      if (cross_prod_z < 0. && prod1 > 0 && prod2 > 0 && fabs(normal_distance) > 1.5) {
        inner_boundary_filtered->points.push_back(point_buf);
        finish_loop = true;
        continue;
      }
      }
      
      if (finish_loop)
      {
        cnt = cnt + 1;
        continue;
      }
  }
  
  // std::cout << "cnt : " << cnt << ", not counted : "
  //           << CloudIn->points.size() - cnt << std::endl;

  // Secondely, search the points from outer boundary condition
  // INPUT : inner_boudnary filtered points
  // OUTPUT : both boundary filtered points = CloudOut
  for (auto point_buf : inner_boundary_filtered->points) {

    bool finish_loop = false;

    for (int i = -1; i < 2; i++) {
      int idx = (closest_idx + i + size) % size;
      int idx_prev = ((closest_idx + i + size) - 1) % size;

      // std::cout << idx << ", " << idx_prev << std::endl;

      auto geofence_xy = outer_array_in[idx];
      auto prev_geofence_xy = outer_array_in[idx_prev];

      double slope;
      if (geofence_xy.first - prev_geofence_xy.first == 0) {
        slope = 0;
      } else {
        slope = (geofence_xy.second - prev_geofence_xy.second) /
                (geofence_xy.first - prev_geofence_xy.first);
      }
      double bias = geofence_xy.second - slope * geofence_xy.first;
      double normal_distance = fabs(slope * point_buf.x - point_buf.y + bias) /
                               sqrt(pow(slope, 2) + 1);

      double prod1 = (point_buf.x - prev_geofence_xy.first) *
                         (geofence_xy.first - prev_geofence_xy.first) +
                     (point_buf.y - prev_geofence_xy.second) *
                         (geofence_xy.second - prev_geofence_xy.second);
      double prod2 = (point_buf.x - geofence_xy.first) *
                         (prev_geofence_xy.first - geofence_xy.first) +
                     (point_buf.y - geofence_xy.second) *
                         (prev_geofence_xy.second - geofence_xy.second);

      double cross_prod_z = ((prev_geofence_xy.first - point_buf.x) *
                                 (geofence_xy.second - point_buf.y) -
                             (prev_geofence_xy.second - point_buf.y) *
                                 (geofence_xy.first - point_buf.x));

      // put the points which are posed on the right from inner boundary
      if (cross_prod_z > 0. && prod1 > 0 && prod2 > 0 &&
          fabs(normal_distance) > 1.5) {
        CloudOut->points.push_back(point_buf);
        finish_loop = true;
        continue;
      }
      }
      if (finish_loop)
        continue;
    }
 
}
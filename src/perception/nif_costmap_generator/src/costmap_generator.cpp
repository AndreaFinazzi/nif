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

// Constructor
CostmapGenerator::CostmapGenerator() 
: Node("indy_costmap_generator"), bPoints(false), bBoundingBox(false), bRoadBoundary(false), bVisual(false),
  SENSOR_POINTS_COSTMAP_LAYER_("sensor_points"), COMBINED_COSTMAP_LAYER_("costmap")
{
  //   : nh_(nh), private_nh_(private_nh), has_subscribed_wayarea_(false),
    
      // INFLATION_COSTMAP_LAYER_("inflation"),
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
  this->declare_parameter<std::string>("points_topic_name",
                                       "/inverse_mapped_points");
  this->declare_parameter<std::string>("map_topic_name", "/semantics/costmap_generator/occupancy_grid");
  
  respond();

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.best_effort();

  // pub_costmap_ = nh_.advertise<grid_map_msgs::GridMap>("semantics/costmap", 1);
  pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_map_name_, 10);
  // pub_road_occupancy_grid_ =
  //     nh_.advertise<nav_msgs::OccupancyGrid>("/road_occupancy_grid", 1);

  // sub_points_ = nh_.subscribe(points_topic_name_, 10,
  //                             &CostmapGenerator::sensorPointsCallback, this);
  // // sub_lane_boundary_ =
  // //     nh_.subscribe("/hdmap/lane_boundary", 10,
  // //                   &CostmapGenerator::laneBoundaryCallback, this);
  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(10ms, std::bind(&CostmapGenerator::timer_callback, this));
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic_name_, 10,
                          std::bind(&CostmapGenerator::sensorPointsCallback, this, std::placeholders::_1));
  // sub_laserscan_ =
  //     nh_.subscribe(scan_topic_, 1, &CostmapGenerator::LaserScanCallback, this);

  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "out_odometry_ekf_estimated", 10,
      std::bind(&CostmapGenerator::OdometryCallback, this,
                std::placeholders::_1));

  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  sub_InnerGeofence.subscribe(this, "/geofence_inner", rmw_qos_profile);
  sub_OuterGeofence.subscribe(this, "/geofence_outer", rmw_qos_profile);

  m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_InnerGeofence, sub_OuterGeofence);

  m_sync->registerCallback(
      std::bind(&CostmapGenerator::MessegefilteringCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  costmap_ = initGridmap();
  // RoadBoundarycostmap_ = initBoundaryGridmap();
}

CostmapGenerator::~CostmapGenerator(){}

void CostmapGenerator::respond()
{
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("grid_min_value", grid_min_value_);
  this->get_parameter("grid_max_value", grid_max_value_);
  this->get_parameter("grid_resolution", grid_resolution_);
  this->get_parameter("grid_length_x", grid_length_x_);
  this->get_parameter("grid_length_y", grid_length_y_);
  this->get_parameter("grid_position_x", grid_position_x_);
  this->get_parameter("grid_position_y", grid_position_y_);
  this->get_parameter("maximum_lidar_height_thres", maximum_lidar_height_thres_);
  this->get_parameter("minimum_lidar_height_thres", minimum_lidar_height_thres_);
  this->get_parameter("maximum_laserscan_distance_thres", maximum_laserscan_distance_thres_);
  this->get_parameter("minimum_laserscan_distance_thres", minimum_laserscan_distance_thres_);

  this->get_parameter("use_points", use_points_);
  this->get_parameter("points_topic_name", points_topic_name_);
  this->get_parameter("map_topic_name", output_map_name_);
}

void CostmapGenerator::run() {

  // if(bEnablePotential_)
  //   MakeInflationWithPoints();
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
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_sensor_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered_points(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *in_sensor_points);

  // voxel_filtered_points = downsample(in_sensor_points, grid_resolution_);

  costmap_[SENSOR_POINTS_COSTMAP_LAYER_] =
      generateSensorPointsCostmap(in_sensor_points);
  m_in_header = msg->header;
  bPoints = true;
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
}

void CostmapGenerator::MessegefilteringCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inner_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &outer_msg)
{
  m_inner_geofence_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_outer_geofence_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::fromROSMsg(*inner_msg, *m_inner_geofence_points);
  pcl::fromROSMsg(*outer_msg, *m_outer_geofence_points);

  bGeoFence = true;
}

void CostmapGenerator::TransformPointsToGlobal(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut)
    {
      
    }

    // void CostmapGenerator::MakeInflationWithPoints()
    // {
    //   if(!bPoints)
    //     return;

    //   costmap_[INFLATION_COSTMAP_LAYER_].setConstant(grid_min_value_);
    //   obstacleArray.clear();
    //   int idx = 0;
    //   for (grid_map::GridMapIterator iterator(costmap_);
    //   !iterator.isPastEnd(); ++iterator) {
    //     const grid_map::Index index(*iterator);
    //     grid_map::Position pos;
    //     costmap_.getPosition(index, pos);
    //     if(costmap_[SENSOR_POINTS_COSTMAP_LAYER_](index(0), index(1)) != 0 )
    //     {
    //       std::pair<double, double> pointBuf;
    //       pointBuf.first = pos.x();
    //       pointBuf.second = pos.y();
    //       obstacleArray.push_back(pointBuf);
    //     }
    //     idx ++;
    //   }
    //   costmap_[INFLATION_COSTMAP_LAYER_] = createGaussianWorld(&costmap_,
    //   INFLATION_COSTMAP_LAYER_, 0.2, 0.2, obstacleArray);

    // }

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

    // void CostmapGenerator::LaserScanCallback(
    //     const sensor_msgs::LaserScanConstPtr &in_laser_scan_msg) {
    //   if (!use_laserscan_) {
    //     return;
    //   }
    //   costmap_[LASER_2D_COSTMAP_LAYER_] =
    //       generateLaserScanCostmap(in_laser_scan_msg);
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
  // map.add(LASER_2D_COSTMAP_LAYER_, grid_min_value_);
  // map.add(BOUNDING_BOX_COSTMAP_LAYER_, grid_min_value_);
  // map.add(VISUAL_COSTMAP_LAYER_, grid_min_value_);
  // map.add(INFLATION_COSTMAP_LAYER_, grid_min_value_);
  map.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);

  return map;
}

// void CostmapGenerator::RollPitchYawCallback(
//     const geometry_msgs::Vector3::ConstPtr &msg) {
//   veh_roll_rad_ = msg->x;
//   veh_pitch_rad_ = msg->y;
//   veh_yaw_rad_ = msg->z;
// }

// // Create the map using length, resolution, pose.
// grid_map::GridMap CostmapGenerator::initBoundaryGridmap() {
//   grid_map::GridMap map;

//   map.setFrameId(lidar_frame_);
//   map.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
//                   grid_resolution_,
//                   grid_map::Position(grid_position_x_, grid_position_y_));

//   map.add(LANE_POINTS_COSTMAP_LAYER_, grid_min_value_);
//   map.add(COMBINED_COSTMAP_LAYER_, grid_min_value_);

//   return map;
// }

grid_map::Matrix CostmapGenerator::generateSensorPointsCostmap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_points) {

  grid_map::Matrix sensor_points_costmap =
      points2costmap_.makeCostmapFromSensorPoints(
          maximum_lidar_height_thres_, minimum_lidar_height_thres_,
          grid_min_value_, grid_max_value_, costmap_,
          SENSOR_POINTS_COSTMAP_LAYER_, in_sensor_points);
  return sensor_points_costmap;
}

// grid_map::Matrix CostmapGenerator::generateLaserScanCostmap(
//     const sensor_msgs::LaserScanConstPtr &in_laser_scan) {
//   grid_map::Matrix laser_scan_costmap =
//       laserscan2costmap_.makeCostmapFromLaserScan(
//           maximum_laserscan_distance_thres_, minimum_laserscan_distance_thres_,
//           grid_min_value_, grid_max_value_, costmap_, LASER_2D_COSTMAP_LAYER_,
//           in_laser_scan);
//   return laser_scan_costmap;
// }

// grid_map::Matrix CostmapGenerator::generateBBoxesCostmap(
//     const jsk_recognition_msgs::BoundingBoxArrayConstPtr &in_boxes) {
//   grid_map::Matrix bboxes_costmap = bboxes2costmap_.makeCostmapFromBBoxes(
//       costmap_, expand_polygon_size_, size_of_expansion_kernel_, in_boxes);
//   return bboxes_costmap;
// }

// grid_map::Matrix CostmapGenerator::generateVisualCostmap(
//     const detection_msgs::BoundingBoxArrayConstPtr &msg) {
//   grid_map::Matrix visual_costmap = visual2costmap_.makeCostmapFromBBoxes(
//       costmap_, visual_expand_size_, size_of_expansion_kernel_, msg);
//   return visual_costmap;
// }

void CostmapGenerator::generateCombinedCostmap() {
  //   // assuming combined_costmap is calculated by element wise max operation

  costmap_[COMBINED_COSTMAP_LAYER_].setConstant(grid_min_value_);
  if (bPoints) {
    costmap_[COMBINED_COSTMAP_LAYER_] =
        costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
            costmap_[SENSOR_POINTS_COSTMAP_LAYER_]);
    // costmap_[COMBINED_COSTMAP_LAYER_] =
    //     costmap_[COMBINED_COSTMAP_LAYER_].cwiseMax(
    //         costmap_[INFLATION_COSTMAP_LAYER_]);
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


// //Calculate gaussian interpolation.
// grid_map::Matrix CostmapGenerator::createGaussianWorld(grid_map::GridMap *map, const std::string layer_name, 
//                                                          double inflation_x, double inflation_y,
//                                                          const std::vector<std::pair<double, double>>& pointArray)
// {
//   struct Gaussian
//   {
//     double x0, y0;
//     double varX, varY;
//     double s;
//   };

//   AnalyticalFunctions func;
//   std::vector<std::pair<double, double>> vars;
//   std::vector<std::pair<double, double>> means;
//   std::vector<double> scales;
//   std::vector<Gaussian> g;

//   for(auto point : pointArray)
//   {
//     Gaussian gaussian_tmp;
//     gaussian_tmp.x0 = point.first;
//     gaussian_tmp.y0 = point.second;
//     gaussian_tmp.varX = inflation_x; 
//     gaussian_tmp.varY = inflation_y;
//     gaussian_tmp.s =  1 / inflation_x;
//     g.push_back(gaussian_tmp);
//   }

//   func.f_ = [g](double x,double y) {
//     double value = 0.0;
//     for (int i = 0; i < g.size(); ++i) {
//       const double x0 = g.at(i).x0;
//       const double y0 = g.at(i).y0;
//       const double varX = g.at(i).varX;
//       const double varY = g.at(i).varY;
//       const double s = g.at(i).s;
//       value += s * std::exp(-(x-x0)*(x-x0) / (2.0 * varX) - (y-y0)*(y-y0) / (2.0 * varY));
//     }
//     return value;
//   };

//   return fillGridMap(map, layer_name, func);
  
//   // return output;
// }

// grid_map::Matrix CostmapGenerator::fillGridMap(grid_map::GridMap *map, const std::string layer_name, 
//                                               const AnalyticalFunctions &functions)
// {
//   grid_map::Matrix& data = (*map)[layer_name];
//   double max = 0;
//   for (grid_map::GridMapIterator iterator(*map); !iterator.isPastEnd(); ++iterator) {
//     const grid_map::Index index(*iterator);
//     grid_map::Position pos;
//     map->getPosition(index, pos);
//     data(index(0), index(1)) =  functions.f_(pos.x(), pos.y()) * grid_resolution_ / 10;
//     if(max < data(index(0), index(1)))
//       max = data(index(0), index(1));
//   }
//   // std::cout << max << std::endl;
//   return data;
// }
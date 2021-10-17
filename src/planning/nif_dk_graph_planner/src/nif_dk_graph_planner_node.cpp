/*
 * nif_dk_graph_planner_node.cpp
 *
 *  Created on: Oct 9, 2021
 *      Author: Daegyu Lee
 */
#include "nif_dk_graph_planner/nif_dk_graph_planner_node.h"

#include "nif_frame_id/frame_id.h"
#include <assert.h>
#include <string>

#define INF 9999999

using namespace nif::planning;
using namespace nif::common::frame_id::localization;

DKGraphPlannerNode::DKGraphPlannerNode(const std::string &node_name_)
    : Node(node_name_)
// nh_(nh), private_nh_(private_nh), bParcingComplete(false),
//   bCSVGoalList(false), bInitPose(false), bGoalPose(false),
//   bReplanRoute(false)
{
  this->declare_parameter<std::string>("osm_name", std::string(""));
  this->declare_parameter<std::string>("racing_trajectory", std::string(""));

  this->declare_parameter<double>("origin_lat", double(39.809786));
  this->declare_parameter<double>("origin_lon", double(-86.235148));

  this->declare_parameter<double>("ref_gain",       double(1.0));
  this->declare_parameter<double>("collision_gain", double(15.0));
  this->declare_parameter<double>("curvature_gain", double(2.0));
  this->declare_parameter<double>("transient_gain", double(2.0));

  this->m_OsmFileName = this->get_parameter("osm_name").as_string();
  this->m_RacingTrajectory =
      this->get_parameter("racing_trajectory").as_string();
  this->m_originLat = this->get_parameter("origin_lat").as_double();
  this->m_originLon = this->get_parameter("origin_lon").as_double();

this->m_ref_gain = this->get_parameter("ref_gain").as_double();
this->m_collision_gain = this->get_parameter("collision_gain").as_double();
this->m_curvature_gain = this->get_parameter("curvature_gain").as_double();
this->m_transient_gain = this->get_parameter("transient_gain").as_double();

  // Set the ltp reference point
  nif::localization::utils::GeodeticConverter::GeoRef ref;
  ref.latitude = this->m_originLat;
  ref.longitude = this->m_originLon;
  ref.altitude = 0.;
  conv_.initializeReference(ref);

  using namespace std::chrono_literals; // NOLINT
  sub_timer_ = this->create_wall_timer(
      10ms, std::bind(&DKGraphPlannerNode::timer_callback, this));

  pubOsmParcer =
      this->create_publisher<nif_dk_graph_planner_msgs::msg::OsmParcer>(
          "/graph_planner/OsmParcer", nif::common::constants::QOS_PLANNING);
  pubFullNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/full_nodes", nif::common::constants::QOS_PLANNING);
  pubRacingLineRefPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/racing_line", nif::common::constants::QOS_PLANNING);
  pubFirstNodeContainPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/graph_planner/first_node_info", nif::common::constants::QOS_PLANNING);
  pubFinalPathPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/final_path_points", nif::common::constants::QOS_PLANNING);
  pubCandidatesNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/cadidates_points", nif::common::constants::QOS_PLANNING);
  pubCostPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/cost", nif::common::constants::QOS_PLANNING);
  pubFinalPath = this->create_publisher<nav_msgs::msg::Path>(
      "/graph_planner/final_path", nif::common::constants::QOS_PLANNING);
  pubWallInflatedPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/graph_planner/wall_inflated", nif::common::constants::QOS_PLANNING);
  pubFinalPathOnBody = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/on_body_final_points", nif::common::constants::QOS_PLANNING);
  pubBestLayerXYPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/best_layer_xy", nif::common::constants::QOS_PLANNING);

  SubOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
          "in_ekf_odometry", nif::common::constants::QOS_EGO_ODOMETRY,
          std::bind(&DKGraphPlannerNode::CallbackOdometry, this,
                    std::placeholders::_1));
  SubOccupancyGrid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/semantics/costmap_generator/occupancy_grid",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&DKGraphPlannerNode::CallbackOccupancyGrid, this,
                std::placeholders::_1));
  SubCostPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_inflated_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&DKGraphPlannerNode::CallbackCostPoints, this,
                std::placeholders::_1));
  SubClusterCenterPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_cluster_center_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&DKGraphPlannerNode::CallbackClusterCenterPoints, this,
                std::placeholders::_1));
  SubWallPoints =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "in_wall_points", nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&DKGraphPlannerNode::CallbackWallPoints, this,
                    std::placeholders::_1));

  OsmParcing();
  RacingLineParcing();
  BuildGraph();
}

DKGraphPlannerNode::~DKGraphPlannerNode() {}

void DKGraphPlannerNode::OsmParcing() {
  RCLCPP_INFO(this->get_logger(), "opening the osm map file..");
  // Open the file
  m_FullFilePath = m_OsmFileName.c_str();
  pugi::xml_document doc;
  pugi::xml_parse_result parcer = doc.load_file(m_FullFilePath.c_str());

  // file opening result
  if (!parcer) {
    RCLCPP_ERROR(this->get_logger(), "Invalid file path.. ");
    std::cout << "Parse error: " << parcer.description()
              << ", character pos= " << parcer.offset << std::endl;
    std::cout << "Tried to open .. \n" << m_FullFilePath.c_str() << std::endl;
  } else {
    std::cout << "Parse result: " << parcer.description()
              << ", character pos= " << parcer.offset << std::endl;
    std::cout << "Tried to open .. \n" << m_FullFilePath.c_str() << std::endl;
    RCLCPP_INFO(this->get_logger(), "Valid file!");
  }


  // Node data
  for (pugi::xml_node node : doc.child("osm").children("node")) {
    // std::cout << node.attribute("id").value() << ", lat: " <<
    // node.attribute("lat").value() << ", lon: " <<
    // node.attribute("lon").value()<< std::endl;

    nif_dk_graph_planner_msgs::msg::Node nodeTmp;
    nodeTmp.id = node.attribute("id").as_int();
    nodeTmp.lat = node.attribute("lat").as_double();
    nodeTmp.lon = node.attribute("lon").as_double();

    nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
    currentGPS.latitude = (double)nodeTmp.lat;  
    currentGPS.longitude = (double)nodeTmp.lon; 
    // Currently ignore altitude for the most part and just track x/y
    currentGPS.altitude = 0.;
    nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
    conv_.geodetic2Ned(currentGPS, ltp_pt);

    nodeTmp.x = ltp_pt.x;
    nodeTmp.y = -ltp_pt.y;

    // Tags
    for (pugi::xml_node tag : node.children("tag")) {
      std::string start_layer = "start_layer";
      if (tag.attribute("k").as_string() == start_layer) {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.start_layer = tag.attribute("v").as_int();
      }
      std::string end_layer = "end_layer";
      if (tag.attribute("k").as_string() == end_layer) {
        // std::cout << "yaw: " << tag.attribute("v").as_double() << std::endl;
        nodeTmp.end_layer = tag.attribute("v").as_int();
      }
      std::string start_node = "start_node";
      if (tag.attribute("k").as_string() == start_node) {
        // std::cout << "start_node: " << tag.attribute("v").as_double() <<
        // std::endl;
        nodeTmp.start_node = tag.attribute("v").as_int();
      }
      std::string psi = "psi";
      if (tag.attribute("k").as_string() == psi) {
        // std::cout << "psi: " << tag.attribute("v").as_double() <<
        // std::endl;
        nodeTmp.psi = tag.attribute("v").as_double();
      }
      std::string kappa = "kappa";
      if (tag.attribute("k").as_string() == kappa) {
        // std::cout << "kappa: " << tag.attribute("v").as_double() <<
        // std::endl;
        nodeTmp.kappa = tag.attribute("v").as_double();
      }
    }
    m_OsmParcer.nodes.push_back(nodeTmp);
  }

  // // Way data
  nif_dk_graph_planner_msgs::msg::OsmParcer WayParcer;
  for (pugi::xml_node way : doc.child("osm").children("way")) {
    // Way id
    // std::cout << "---------------" << std::endl;
    // std::cout << "way id: " << way.attribute("id").value() << std::endl;
    nif_dk_graph_planner_msgs::msg::Way wayTmp;
    wayTmp.id = way.attribute("id").as_int();
    double kappa_sum = 0.;
    double psi_sum = 0.;
    // attribute of way data : Node IDs
    for (pugi::xml_node way_data : way.children("nd")) {
      nif_dk_graph_planner_msgs::msg::Node nodeRef; // Node Ids in the each way
      nodeRef.id = way_data.attribute("ref").as_int();
      for (auto node : m_OsmParcer.nodes) {
        if (node.id == nodeRef.id) {
          nodeRef = node;
          kappa_sum += fabs(node.kappa);
          psi_sum += fabs(node.psi);
        }
      }
      wayTmp.nodes.push_back(nodeRef);
    }
    wayTmp.first_node_id = wayTmp.nodes[0].id;
    wayTmp.last_node_id = wayTmp.nodes[wayTmp.nodes.size() - 1].id;
    wayTmp.cost = INF;
    wayTmp.kappa = kappa_sum;
    wayTmp.psi = psi_sum;

    // std::cout << psi_sum << ", " << kappa_sum << std::endl;

    // Tags
    for (pugi::xml_node tag : way.children("tag")) {
      // start_layer
      std::string start_layer = "start_layer";
      // end_layer
      std::string end_layer = "end_layer";
      // start_node
      std::string start_node = "start_node";
      // end_node
      std::string end_node = "end_node";

      if (tag.attribute("k").as_string() == start_layer) {
        wayTmp.start_layer = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == end_layer) {
        wayTmp.end_layer = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == start_node) {
        wayTmp.start_node = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == end_node) {
        wayTmp.end_node = tag.attribute("v").as_int();
      }
    }
    m_OsmParcer.ways.push_back(wayTmp);
  }

  m_FullIndexedPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  m_FullNodeIDPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto node : m_OsmParcer.nodes) {
    pcl::PointXYZI current_point;
    current_point.x = node.x;
    current_point.y = node.y;
    current_point.intensity = node.start_layer;
    m_FullIndexedPoints->points.push_back(current_point);

    pcl::PointXYZI node_id_point;
    node_id_point.x = node.x;
    node_id_point.y = node.y;
    node_id_point.intensity = node.id;
    m_FullNodeIDPoints->points.push_back(node_id_point);
  }

  m_FirstNodeContainPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto wayTmp : m_OsmParcer.ways) {
    for (auto node : wayTmp.nodes)
    {
      pcl::PointXYZI node_id_point;
      node_id_point.x = node.x;
      node_id_point.y = node.y;
      node_id_point.intensity = wayTmp.first_node_id;
      m_FirstNodeContainPoints->points.push_back(node_id_point);
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Converting OSM file to the ros system is completed!");
  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(), "node size: %d" , m_OsmParcer.nodes.size());
  RCLCPP_INFO(this->get_logger(), "way size: %d" , m_OsmParcer.ways.size());

  bParcingComplete = true;
}

void DKGraphPlannerNode::RacingLineParcing()
{
  m_racingLineRefPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<std::pair<std::string, std::vector<double>>> result = read_csv(m_RacingTrajectory);

  int start_layer, end_layer, start_node, end_node;
  double distance;

  int layer_start = -1;
  for (int i = 0; i < result[0].second.size()-1; i++) {
    pcl::PointXYZI point_buf;
    point_buf.x = result[1].second[i];
    point_buf.y = result[2].second[i];        
    point_buf.intensity = i;
    m_racingLineRefPoints->points.push_back(point_buf);

    // if (distance < 0.2)
    // {
    //   std::pair<int, std::pair<int, int>> layer_node; //layer - (start_node , end_node)
    //   layer_node.first = start_layer; 

    //   std::pair<int, int> node_pair;
    //   node_pair.first = start_node;
    //   node_pair.second = end_node;
    //   layer_node.second = node_pair;  

    //   if(layer_start != start_layer)
    //   {
    //     m_RacingLineGraphArray.push_back(layer_node);
    //     layer_start = start_layer;
    //   }
    // }
  }
  double max_cost = 0.0;
  for(int i = 0; i < m_OsmParcer.ways.size(); i++)
  {
    auto way = m_OsmParcer.ways[i];
    double cost_sum = 0.0;
    for(auto node : way.nodes)
    {
      double dist = getClosestDistance(node.x, node.y, m_racingLineRefPoints);
      cost_sum += dist;
    }
    // size_t node_size = way.nodes.size(); 
    // double first_node_dist = getClosestDistance(way.nodes[0].x, way.nodes[0].y, m_racingLineRefPoints);
    // double last_node_dist = getClosestDistance(way.nodes[node_size-1].x, way.nodes[node_size-1].y,
    //                                             m_racingLineRefPoints);

    // cost_sum = first_node_dist + last_node_dist;
    m_OsmParcer.ways[i].cost = cost_sum;
    if (max_cost < cost_sum)
      max_cost = cost_sum;
  }
  // normalize cost : distance from the racing line
  for (int i = 0; i < m_OsmParcer.ways.size(); i++) {
    m_OsmParcer.ways[i].cost = m_OsmParcer.ways[i].cost / max_cost;
    // std::cout << m_OsmParcer.ways[i].cost << ", " << max_cost << std::endl;
  }
  // std::cout << cost_sum << std::endl;

  m_WaysResister.clear();
  m_FirstNodeBasedResister.clear();
  for (auto wayRegister : m_OsmParcer.ways) {
    m_WaysResister[wayRegister.start_layer].push_back(wayRegister);
    m_FirstNodeBasedResister[wayRegister.first_node_id].push_back(wayRegister);
  }

  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(),
              "Racing line is loaded!");
  RCLCPP_INFO(this->get_logger(), "racing line size: %d" , m_racingLineRefPoints->points.size());
  RCLCPP_INFO(this->get_logger(), "layer size: %d", m_WaysResister.size());
  m_LayerSize = m_WaysResister.size();

  bRacingLine = true;
}

void DKGraphPlannerNode::CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
  std::lock_guard<std::mutex> lock(mtx);
  if (bRacingLine && bParcingComplete && bBuildGraph) {

    m_veh_x = msg->pose.pose.position.x;
    m_veh_y = msg->pose.pose.position.y;
    tf2::Quaternion tf_quat;
    tf2::convert(msg->pose.pose.orientation, tf_quat);
    tf2::Matrix3x3 mat(tf_quat);
    mat.getRPY(m_veh_roll, m_veh_pitch, m_veh_yaw);

    double m_closest_x_in_racing_line, m_closest_y_in_racing_line;
  
    // // get target index from reference racing points
    int current_idx;
    GetIntensityInfo(m_veh_x, m_veh_y, m_racingLineRefPoints,
                     m_closest_x_in_racing_line,
                     m_closest_y_in_racing_line, 
                     current_idx);
    int current_node_id = 0;
    // get node id for planing
    // Path from vehicle

    int current_layer_for_starting_node;
    GetIntensityInfo(m_veh_x, m_veh_y, m_FullIndexedPoints,
                     current_layer_for_starting_node); // to get current layer


    int prev_layer_for_starting_node = current_layer_for_starting_node -1;
    if(prev_layer_for_starting_node < 0)
      prev_layer_for_starting_node = prev_layer_for_starting_node + m_LayerSize;

    prev_layer_for_starting_node = prev_layer_for_starting_node % m_LayerSize;


    for (auto way : m_WaysResister[prev_layer_for_starting_node]) {
      if (way.start_node == 2)
        current_node_id = way.first_node_id;

      if(current_node_id == 0)
      {
        current_node_id = way.first_node_id;
        break;
      }
    }

    GetIntensityInfo(m_veh_x, m_veh_y, m_FirstNodeContainPoints,
                     m_ClosestFirstNodeId); // to get current first node id
    if(current_node_id == 0)
    {
      current_node_id = m_ClosestFirstNodeId;
    }
    // std::cout << m_ClosestFirstNodeId << std::endl;

    
      // Path from racing line
      // GetIntensityInfo(m_closest_x_in_racing_line,
      // m_closest_y_in_racing_line,
      //                  m_FirstNodeContainPoints, current_node_id);

      // if(m_prevStartFirstNodeId != -1)
      //   current_node_id = m_prevStartFirstNodeId;

    m_StartNode = current_node_id;

    double target_x_in_racing_line, target_y_in_racing_line;
    int target_idx = current_idx + 100;
    int target_node_id;
    size_t racingLineRefSize = m_racingLineRefPoints->points.size();
    target_idx = target_idx % racingLineRefSize;

    target_x_in_racing_line = m_racingLineRefPoints->points[target_idx].x;
    target_y_in_racing_line = m_racingLineRefPoints->points[target_idx].y;
    GetIntensityInfo(target_x_in_racing_line, target_y_in_racing_line, m_FirstNodeContainPoints, target_node_id);
    m_closestGoalNode = target_node_id;

    m_nearbyFirstNodes.clear();
    int search_num_idx = 20;
    int current_layer;
    getNearbyNodesFromLayer(
        m_veh_x, m_veh_y, search_num_idx, m_FullIndexedPoints, current_layer,
        m_nearbyFirstNodes);

    m_currentLayer = current_layer;
    
    bOdometry = true;
  }
}

void DKGraphPlannerNode::CallbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
  m_OccupancyGrid = *msg;
  bOccupancyGrid = true;
}

void DKGraphPlannerNode::CallbackCostPoints(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  m_InflatedCostPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_InflatedCostPoints);
  bInflatedPoints = true;
}

void DKGraphPlannerNode::CallbackClusterCenterPoints(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_ClusterCenterPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_ClusterCenterPoints);
  bCenteredPoints = true;
}

void DKGraphPlannerNode::CallbackWallPoints(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  m_WallPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_points(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *in_points);
  for(auto point : in_points->points)
  {
    pcl::PointXYZI point_buf;
    point_buf.x = point.x;
    point_buf.y = point.y;
    projected_points->points.push_back(point_buf);
  }
  m_WallPoints = downsample(projected_points, 1.0);

  bWallPoints = true;
}

void DKGraphPlannerNode::timer_callback() {
  if (bRacingLine && bParcingComplete && bBuildGraph) {

    // if (bWallPoints) {
    //   m_WallInflatedCostPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
    //   createGaussianWorld(m_WallPoints, 5.0, 5.0, m_WallInflatedCostPoints);
    //   bWallInflated = true;
    // }

    MessagePublisher();
    Planning();
    ToPathMsg();
  }
}

void DKGraphPlannerNode::MessagePublisher() {
  // origin node publisher  
  sensor_msgs::msg::PointCloud2 FullNodeCloudMsg;
  pcl::toROSMsg(*m_FullIndexedPoints, FullNodeCloudMsg);
  FullNodeCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FullNodeCloudMsg.header.stamp = this->now();
  pubFullNodePoints->publish(FullNodeCloudMsg);

  // racing line publisher
  sensor_msgs::msg::PointCloud2 RacingLineCloudMsg;
  pcl::toROSMsg(*m_racingLineRefPoints, RacingLineCloudMsg);
  RacingLineCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  RacingLineCloudMsg.header.stamp = this->now();
  pubRacingLineRefPoints->publish(RacingLineCloudMsg);

  // first node info publisher
  sensor_msgs::msg::PointCloud2 FirstNodeInfoCloudMsg;
  pcl::toROSMsg(*m_FirstNodeContainPoints, FirstNodeInfoCloudMsg);
  FirstNodeInfoCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FirstNodeInfoCloudMsg.header.stamp = this->now();
  pubFirstNodeContainPoints->publish(FirstNodeInfoCloudMsg);

  // if (bWallInflated) {
  //   sensor_msgs::msg::PointCloud2 WallInflatedCloudMsg;
  //   pcl::toROSMsg(*m_WallInflatedCostPoints, WallInflatedCloudMsg);
  //   WallInflatedCloudMsg.header.frame_id =
  //       nif::common::frame_id::localization::BASE_LINK;
  //   WallInflatedCloudMsg.header.stamp = this->now();
  //   pubWallInflatedPoints->publish(WallInflatedCloudMsg);
  // }
}

void DKGraphPlannerNode::GetIntensityInfo(
    const double &x_in, const double &y_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_points, int &intensity_out) {
  if (in_points->points.empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(in_points);

  pcl::PointXYZI searchPoint;
  searchPoint.x = x_in;
  searchPoint.y = y_in;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  double nearest_x_in_points, nearest_y_in_points;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      nearest_x_in_points = in_points->points[pointIdxNKNSearch[i]].x;
      nearest_y_in_points = in_points->points[pointIdxNKNSearch[i]].y;
      intensity_out = in_points->points[pointIdxNKNSearch[i]].intensity;
    }
  }
}

void DKGraphPlannerNode::GetIntensityInfo(
    const double &x_in, const double &y_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_points, double &intensity_out) {
  if(in_points->points.empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(in_points);

  pcl::PointXYZI searchPoint;
  searchPoint.x = x_in;
  searchPoint.y = y_in;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  double nearest_x_in_points, nearest_y_in_points;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      nearest_x_in_points = in_points->points[pointIdxNKNSearch[i]].x;
      nearest_y_in_points = in_points->points[pointIdxNKNSearch[i]].y;
      intensity_out = in_points->points[pointIdxNKNSearch[i]].intensity;
    }
  }
}

void DKGraphPlannerNode::GetIntensityInfo(
    const double &x_in, const double &y_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_points,
    double &nearest_x_in_points_out, double &nearest_y_in_points_out,
    int &intensity_out) {
  if (in_points->points.empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(in_points);

  pcl::PointXYZI searchPoint;
  searchPoint.x = x_in;
  searchPoint.y = y_in;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  double nearest_x_in_points, nearest_y_in_points;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      nearest_x_in_points = in_points->points[pointIdxNKNSearch[i]].x;
      nearest_y_in_points = in_points->points[pointIdxNKNSearch[i]].y;
      intensity_out = in_points->points[pointIdxNKNSearch[i]].intensity;
    }
  }
  nearest_x_in_points_out = nearest_x_in_points;
  nearest_y_in_points_out = nearest_y_in_points;
}

double DKGraphPlannerNode::getClosestDistance(
    const double &x_in, const double &y_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_in) {

  if (points_in->points.empty())
    return 0.;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(points_in);

  pcl::PointXYZI searchPoint;
  searchPoint.x = x_in;
  searchPoint.y = y_in;

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  double nearest_x_in_points, nearest_y_in_points;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      nearest_x_in_points = points_in->points[pointIdxNKNSearch[i]].x;
      nearest_y_in_points = points_in->points[pointIdxNKNSearch[i]].y;
    }
  }
  double distance = sqrt(pow(nearest_x_in_points - x_in, 2) +
                         pow(nearest_y_in_points - y_in, 2));
  
  return distance;
}

void DKGraphPlannerNode::getNearbyNodesFromLayer(
    const double &x_in, const double &y_in, const int &search_num_idx_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
    int& current_layer_out, std::vector<std::pair<int, int>> &nearby_indice_out) {

  if (!bParcingComplete)
    return;

  if (points_in->points.empty())
    return;

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(points_in);

  pcl::PointXYZI searchPoint;
  searchPoint.x = x_in;
  searchPoint.y = y_in;

  int K = 1;
  int current_layer;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  double nearest_x_in_points, nearest_y_in_points;
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                            pointNKNSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
      nearest_x_in_points = points_in->points[pointIdxNKNSearch[i]].x;
      nearest_y_in_points = points_in->points[pointIdxNKNSearch[i]].y;
      current_layer = points_in->points[pointIdxNKNSearch[i]].intensity;
    }
  }
  current_layer_out = current_layer;

  for(int i = 0; i < search_num_idx_in; i++)
  {
    int layer_idx = current_layer_out + i;
    if(layer_idx > m_LayerSize - 1)
      layer_idx = layer_idx % m_LayerSize;

    for (auto way : m_WaysResister[layer_idx]) {
      std::pair<int, int> first_node_and_layer_pair;
      first_node_and_layer_pair.first = way.first_node_id;
      first_node_and_layer_pair.second = way.start_layer;
      nearby_indice_out.push_back(first_node_and_layer_pair);
    }
  }
  removeDuplicated(nearby_indice_out);
}

void DKGraphPlannerNode::BuildGraph()
{
  if(!bRacingLine)
  {
    RCLCPP_WARN(this->get_logger(), "------------------------------------");
    RCLCPP_WARN(this->get_logger(), "Graph is not build!");
    return;
  } 
  int V, E;
  V = m_OsmParcer.nodes.size(); // the number of node
  E = m_OsmParcer.ways.size();  // the number of way
  m_graph.clear();

  // Build a graph.
  for (auto way : m_OsmParcer.ways) {
    Connected connected_;
    connected_.id = way.last_node_id;
    connected_.cost = way.cost;
    connected_.curvature_cost = way.kappa;
    connected_.collision_cost = 0;
    m_graph[way.first_node_id].push_back(connected_);
  }
  bBuildGraph = true;

  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(), "Graph is build!");
  RCLCPP_INFO(this->get_logger(), "graph size: %d", m_graph.size());
}

void DKGraphPlannerNode::UpdateGraph()
{
  if(!bInflatedPoints || !bCenteredPoints)
    return;

  if (!bOdometry)
    return;
  
  int count = 0;
  int nextLayer = 1 + m_currentLayer; // maximum current layer : 42 , layer size : 43
  if(nextLayer > m_LayerSize - 1)
    nextLayer = nextLayer - m_LayerSize;
  int nextnextLayer = 1 + nextLayer;
  if (nextnextLayer > m_LayerSize - 1)
    nextnextLayer = nextnextLayer - m_LayerSize;
  m_CostPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto nearbyNode : m_nearbyFirstNodes) {
    // std::cout << "----------------------\n";
    auto first_node_in_nearby = nearbyNode.first; //first_node_id in nearby way
    auto start_layer_in_nearby = nearbyNode.second; //start_layer in nearby way
    double best_layer_last_x = m_BestLayerXYArray[start_layer_in_nearby].first;
    double best_layer_last_y = m_BestLayerXYArray[start_layer_in_nearby].second;
    // std::cout << best_layer_last_x << ", " << best_layer_last_y << std::endl;
    for (int i = 0; i < m_FirstNodeBasedResister[first_node_in_nearby].size();
         i++) {
      auto way = m_FirstNodeBasedResister[first_node_in_nearby][i];
      auto way_with_layer = m_WaysResister[start_layer_in_nearby][i];
      double cost_accumulated = 0;
      double cost_transient;
      if (way.start_layer == nextnextLayer || way.start_layer == (nextLayer) ||
          way.start_layer == (m_currentLayer)) {
        cost_transient = sqrt(pow(way_with_layer.nodes[way.nodes.size()-1].x - best_layer_last_x, 2) + 
                              pow(way_with_layer.nodes[way.nodes.size()-1].y - best_layer_last_y, 2));
      }
      double cost_close_to_vehicle;
      if(m_currentLayer == start_layer_in_nearby)
      {
        cost_close_to_vehicle = abs(way.start_node - m_FirstNodeBasedResister[m_ClosestFirstNodeId][0].start_node);
      }

      for (auto node : way.nodes) {
        double pt_x_global = node.x;
        double pt_y_global = node.y;

        //global to local
        double pt_x_body = (pt_x_global - m_veh_x) * cos(m_veh_yaw) +
                           (pt_y_global - m_veh_y) * sin(m_veh_yaw);
        double pt_y_body = -(pt_x_global - m_veh_x) * sin(m_veh_yaw) +
                           (pt_y_global - m_veh_y) * cos(m_veh_yaw);

        double collision_cost =
            CorrespondingCost(pt_x_body, pt_y_body, 3.0, m_ClusterCenterPoints,
                              m_InflatedCostPoints);

        double wall_cost = 0.;
        // if (bWallInflated)
        // {
        //   wall_cost = CorrespondingCost(pt_x_body, pt_y_body, 5.0, m_WallPoints,
        //                                 m_WallInflatedCostPoints);
        // }
        // std::cout << collision_cost << std::endl;

        cost_accumulated += collision_cost; // + wall_cost;
        count++;

        pcl::PointXYZI pointbuf;
        pointbuf.x = pt_x_global;
        pointbuf.y = pt_y_global;
        pointbuf.intensity = way.cost + collision_cost + cost_transient; ; 
        m_CostPoints->points.push_back(pointbuf);
      }

      // std::cout << "cost_accumulated : " << cost_accumulated <<
      // std::endl;
      // if (cost_accumulated != 0.)
      // {
        // std::cout << "cost :" << way.cost << std::endl;
        // std::cout << "cost_accumulated : " << cost_accumulated << std::endl;
        // std::cout << "cost_transient : " << cost_transient << std::endl;
      // }
      
      int j = 0;
      for (auto graph : m_graph[first_node_in_nearby]) {
        if (graph.id == way.last_node_id) {
          m_graph[first_node_in_nearby][j].collision_cost = cost_accumulated;
          m_graph[first_node_in_nearby][j].transient_cost = cost_transient;
          m_graph[first_node_in_nearby][j].cost_close_to_vehicle =
              cost_close_to_vehicle;
        }
        j++;
      }
    }
  }

  sensor_msgs::msg::PointCloud2 CostCloudMsg;
  pcl::toROSMsg(*m_CostPoints, CostCloudMsg);
  CostCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  CostCloudMsg.header.stamp = this->now();
  pubCostPoints->publish(CostCloudMsg);

}

void DKGraphPlannerNode::ReleaseGraph()
{
  if (!bInflatedPoints || !bCenteredPoints)
    return;

  for (auto nearbyNode : m_nearbyFirstNodes) {
    auto first_node_in_nearby = nearbyNode.first;
    for (auto way : m_FirstNodeBasedResister[first_node_in_nearby])
      for (int i = 0; i < m_graph[first_node_in_nearby].size(); i++) {
        if (m_graph[first_node_in_nearby][i].id == way.last_node_id) {
          // m_graph[first_node_in_nearby][i].cost = way.cost;
          m_graph[first_node_in_nearby][i].collision_cost = 0.;
          m_graph[first_node_in_nearby][i].transient_cost = 0.;
          m_graph[first_node_in_nearby][i].cost_close_to_vehicle = 0.;
      }
    }
  }
}

void DKGraphPlannerNode::Planning() {
  std::lock_guard<std::mutex> lock(mtx);

  // std::cout << "-----Planning----- " << std::endl;
  current_time = static_cast<double>(this->now().seconds()) +
                 static_cast<double>(this->now().nanoseconds()) * 1e-9;
  // std::cout << "Time[sec] : " << current_time - prev_time << std::endl;
  // std::cout << "Received Occupancy : " << bOccupancyGrid << std::endl;
  // std::cout << "Received Oodmetry : " << bOdometry << std::endl;

  UpdateGraph();

  m_PlanningPathNodes.clear();
  // Initialize the dist array with infinite values.
  std::unordered_map<int, double> dist;
  int path_index = 0;
  for (auto node : m_OsmParcer.nodes) {
    dist[node.id] = INF;
  }

    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>>
        qu;

    qu.push({0., m_StartNode}); //Put init node in the que
    dist[m_StartNode] = 0.; // Update value of start point as 0
    // for(auto id : m_PlanningPat hNodes)
    //   std::cout << "m_PredSuccMap: " <<m_PredSuccMap[id] << std::endl;
    // std::cout << m_graph[m_StartNode].size() << std::endl;

    m_PredSuccMap.clear();
    while (!qu.empty()) {

      double cost = qu.top().first;  // cost : distance to next node
      int here = qu.top().second; // here : current node Id
      // std::cout << here << "," << cost << std::endl;

      qu.pop();

      m_PlanningPathNodes.push_back(here);

      if (here == m_closestGoalNode) {
        // std::cout << "path finding!" << std::endl;
        // m_PlanningPathNodes.pop_back();
        break;
      }

      for (auto connected : m_graph[here]) {
        int next = connected.id;
        double nextcost = connected.cost * m_ref_gain + 
                          connected.collision_cost * m_collision_gain +
                          connected.curvature_cost * m_curvature_gain +
                          connected.transient_cost * m_transient_gain;
        // connected.cost_close_to_vehicle;  

        // std::cout << "connected.cost: " << connected.cost << std::endl;
        // std::cout << "connected.collision_cost : " << connected.collision_cost << std::endl;
        // std::cout << "connected.transient_cost : " << connected.transient_cost << std::endl;

        if (dist[next] > dist[here] + nextcost) {
          dist[next] = dist[here] + nextcost;
          qu.push({dist[next], next});
          m_PredSuccMap[next] = here;
        }
      }
    }
    // if(!m_PlanningPathNodes.empty())
      // m_PlanningPathNodes.pop_front();

    // std::cout << "--------------" << std::endl;
    // for(auto id : m_PlanningPathNodes)
    //   std::cout << "m_PredSuccMap: " <<m_PredSuccMap[id] << std::endl;
    
    std::deque<int> FinalPathQue;
    auto next = m_PredSuccMap[m_closestGoalNode];

    // std::cout << "graph size: "<< m_graph.size() << std::endl;
    // std::cout << "m_StartNode: " << m_StartNode << std::endl;
    // std::cout <<"m_closestGoalNode: " << m_closestGoalNode << std::endl;
    // std::cout << "next: "<<next << std::endl;
    FinalPathQue.push_back(m_closestGoalNode);
    FinalPathQue.push_back(next);
    for (int i = 0; i < m_PlanningPathNodes.size(); i++) { // m_PredSuccMap
      next = m_PredSuccMap[next];
      FinalPathQue.push_back(next);
    }
    m_FinalNodes.clear();
    for (auto final : FinalPathQue) {
      if(final == 0)
        continue;

      m_FinalNodes.push_back(final);
    }
    std::reverse(m_FinalNodes.begin(), m_FinalNodes.end());

    // //test
    // std::cout << "m_FinalNodes : "<< m_FinalNodes.size() << std::endl;
    // for (auto final_node : m_FinalNodes)
    // {
    //   std::cout << final_node << "\n";
    // }

    ReleaseGraph();

    prev_time = current_time;
}

void DKGraphPlannerNode::ToPathMsg() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr candidates_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr finalcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);

  int count = 0;
  // Candidates Nodes
  for (auto node_id : m_PlanningPathNodes) {
    for (auto candidates : m_FirstNodeBasedResister[node_id]) {
      for(auto nd : candidates.nodes)
      {
        pcl::PointXYZI current_point;
        current_point.x = nd.x;
        current_point.y = nd.y;
        current_point.z = nd.z;
        current_point.intensity = count;
        candidates_ptr->points.push_back(current_point);
        count++;
      }
    }
  }
  sensor_msgs::msg::PointCloud2 CandidatesNodeCloudMsg;
  if(!candidates_ptr->points.empty())
    pcl::toROSMsg(*candidates_ptr, CandidatesNodeCloudMsg);
  CandidatesNodeCloudMsg.header.frame_id =
      nif::common::frame_id::localization::ODOM;
  CandidatesNodeCloudMsg.header.stamp = this->now();
  pubCandidatesNodePoints->publish(CandidatesNodeCloudMsg);

  // Final Nodes
  int cnt = 0;
  double prev_x = 0.;
  double prev_y = 0.;
  m_BestLayerArray.clear();
  m_BestLayerXYArray.clear();
  nav_msgs::msg::Path FinalPath;
  for (auto node_id : m_FinalNodes) {
    for (auto final_id : m_FirstNodeBasedResister[node_id]) {
      if (!ExistInList(final_id.last_node_id, m_FinalNodes))
        continue;
      
      if (m_prevFirstLayer != m_currentLayer)
      {
        m_prevFirstLayer = m_currentLayer;
        std::cout << "layer changed!" << std::endl;
      }

      for (auto nd : final_id.nodes) {
        pcl::PointXYZI point_buf;
        point_buf.x = nd.x;
        point_buf.y = nd.y;
        point_buf.z = nd.z;
        point_buf.intensity = cnt;
        finalcloud_ptr->points.push_back(point_buf);
        cnt++;

        geometry_msgs::msg::PoseStamped pose_buf;
        if(prev_x == 0. && prev_y == 0.)
        {
          prev_x = nd.x;
          prev_y = nd.y;
          continue;          
        }
        if(prev_x == nd.x || prev_y == nd.y)
          continue;
        
        pose_buf.pose.position.x = nd.x;
        pose_buf.pose.position.y = nd.y;
        double yaw = atan2(nd.y - prev_y, nd.x - prev_x);
        tf2::Quaternion quat_;
        geometry_msgs::msg::Quaternion quat_msg;
        quat_.setRPY(0., 0., yaw);
        quat_.normalize();
        quat_msg = tf2::toMsg(quat_);
        pose_buf.pose.orientation = quat_msg;
        FinalPath.poses.push_back(pose_buf);

        prev_x = nd.x;
        prev_y = nd.y;

      }
      // m_BestLayerArray[final_id.start_layer] = final_id.start_node;
      m_BestLayerXYArray[final_id.start_layer] =
          std::make_pair(final_id.nodes[final_id.nodes.size() - 1].x,
                         final_id.nodes[final_id.nodes.size() - 1].y);
    }
  }
  //BestLayer XY visualize(should be end of each ways)
  pcl::PointCloud<pcl::PointXYZI>::Ptr best_xy_points(new pcl::PointCloud<pcl::PointXYZI>);
  int cnt2 =0;
  for(auto xy : m_BestLayerXYArray)
  {
    pcl::PointXYZI point_buf;
    point_buf.x = xy.second.first;
    point_buf.y = xy.second.second;
    point_buf.intensity = cnt2;
    best_xy_points->points.push_back(point_buf);
    cnt2 ++;
  }
  sensor_msgs::msg::PointCloud2 BestLayerXYCloudMsg;
  pcl::toROSMsg(*best_xy_points, BestLayerXYCloudMsg);
  BestLayerXYCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  BestLayerXYCloudMsg.header.stamp = this->now();
  pubBestLayerXYPoints->publish(BestLayerXYCloudMsg);

  if (FinalPath.poses.empty()) {
    // std::cout << "planning node size : " << m_FinalNodes.size() << std::endl;
    // std::cout << "m_currentLayer : " << m_currentLayer << std::endl;
    // std::cout << "start node id : " << m_StartNode << std::endl;
    RCLCPP_WARN(this->get_logger(), "Empty poses in the fianl path.");
  }

  sensor_msgs::msg::PointCloud2 FinalPathCloudMsg;
  if(!finalcloud_ptr->points.empty())
    pcl::toROSMsg(*finalcloud_ptr, FinalPathCloudMsg);
  FinalPathCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FinalPathCloudMsg.header.stamp = this->now();
  pubFinalPathPoints->publish(FinalPathCloudMsg);

  FinalPath.header.frame_id = nif::common::frame_id::localization::ODOM;
  FinalPath.header.stamp = this->now();
  pubFinalPath->publish(FinalPath);

  pcl::PointCloud<pcl::PointXYZI>::Ptr finalOnBodyPoints(
      new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::msg::PointCloud2 FinalOnBodyCloudMsg;
  if (!finalcloud_ptr->points.empty())
  {
    TransformPointsToBody(finalcloud_ptr, finalOnBodyPoints, 
                          m_veh_x, m_veh_y, m_veh_yaw);
    pcl::toROSMsg(*finalOnBodyPoints, FinalOnBodyCloudMsg);
  }
  FinalOnBodyCloudMsg.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  FinalOnBodyCloudMsg.header.stamp = this->now();
  pubFinalPathOnBody->publish(FinalOnBodyCloudMsg);
}

bool DKGraphPlannerNode::ExistInList(int id, std::deque<int> list) {
  bool exist = false;
  for (auto scan : list) {
    if (scan == id) {
      exist = true;
      break;
    }
  }
  return exist;
}

std::vector<std::pair<std::string, std::vector<double>>>
DKGraphPlannerNode::read_csv(std::string filename) {
  // Reads a CSV file into a vector of <string, vector<int>> pairs where
  // each pair represents <column name, column values>

  // Create a vector of <string, int vector> pairs to store the result
  std::vector<std::pair<std::string, std::vector<double>>> result;

  // Create an input filestream
  std::ifstream CSVInput(filename);

  // Make sure the file is open
  if (!CSVInput.is_open())
    throw std::runtime_error("Could not open file");

  // Helper vars
  std::string line, colname;
  double val;

  // Read the column names
  if (CSVInput.good()) {
    // Extract the first line in the file
    std::getline(CSVInput, line);

    // Create a stringstream from line
    std::stringstream ss(line);

    // Extract each column name
    while (std::getline(ss, colname, ';')) {
      // Initialize and add <colname, double vector> pairs to result
      result.push_back({colname, std::vector<double>{}});
    }
  } else {
    std::cout << "Check CSV file" << std::endl;
  }

  // Read data, line by line
  while (std::getline(CSVInput, line)) {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // std::cout << line << std::endl;

    // Keep track of the current column index
    int colIdx = 0;
    std::string token;
    while (std::getline(ss, token, ';')) {
      // std::cout << token << '\n';
      // val = std::stod(token);
      if (token == "Start" || token == "Add" || token == "End")
        val = 0;
      else {
        val = std::stod(token);
      }
      result.at(colIdx).second.push_back(val);
      colIdx++;
    }
  }

  // Close file
  CSVInput.close();
  return result;
}

double DKGraphPlannerNode::CorrespondingCost(
    const double pt_x, const double pt_y,
    const nav_msgs::msg::OccupancyGrid &gridmap) {
  int pt_in_grid_x = (pt_x) / gridmap.info.resolution -
                     gridmap.info.origin.position.x / gridmap.info.resolution + 1;
  int pt_in_grid_y = (pt_y) / gridmap.info.resolution -
                     gridmap.info.origin.position.y / gridmap.info.resolution + 1;

  double cost;
  if (pt_in_grid_x > gridmap.info.width || pt_in_grid_y > gridmap.info.height) {
    cost = 0.;
    return cost;
  }

  cost = gridmap.data[pt_in_grid_y * gridmap.info.width + pt_in_grid_x];

  return cost;
}

double DKGraphPlannerNode::CorrespondingCost(
    const double pt_x, const double pt_y, double inflation,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & in_centered_points, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr& in_inflated_points) {
  
  double cost = 0.0;
  double max_cost = 0.0;
  for (auto center_point : in_centered_points->points) {
    double dist = sqrt(pow(pt_x - center_point.x, 2) + pow(pt_y - center_point.y, 2));
    if (dist > inflation)
      continue;
    else
    {
      GetIntensityInfo(pt_x, pt_y, in_inflated_points, cost);
      if (max_cost < cost)
      {
        max_cost = cost;
      }
    }
  }

  return max_cost;
}

void DKGraphPlannerNode::removeDuplicated(std::vector<std::pair<int,int>> &v) {

  std::set<std::pair<int, int>, duplication_comparator> unique;

  // Fill the set
  for (const auto &p : v) {
    unique.insert(p);
  }
  v.clear();
  
  for (const auto &p : unique) {
    // std::cout << p.first << ", " << p.second << "\n";
    v.push_back(p);
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
DKGraphPlannerNode::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

// Calculate gaussian interpolation.
void DKGraphPlannerNode::createGaussianWorld(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& points_in, double inflation_x,
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

  for (auto point : points_in->points) {
    Gaussian gaussian_tmp;
    gaussian_tmp.x0 = point.x;
    gaussian_tmp.y0 = point.y;
    gaussian_tmp.varX = inflation_x;
    gaussian_tmp.varY = inflation_y;
    gaussian_tmp.s = 1 / inflation_x;
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
      value += s * std::exp(-(x - x0) * (x - x0) / (2.0 * varX) -
                            (y - y0) * (y - y0) / (2.0 * varY));
    }
    return value;
  };

  for (auto point : points_in->points) {
    for (double i = -inflation_x;
         i < inflation_x; i = i + 2.5) {
      for (double j = -inflation_y;
           j < inflation_y; j = j + 2.5) {
        pcl::PointXYZI point_buf;
        point_buf.x = point.x + i;
        point_buf.y = point.y + j;
        point_buf.intensity = func.f_(point_buf.x, point_buf.y);
        if (point_buf.intensity < fabs(1.0))
          points_out->points.push_back(point_buf);
      }
    }
  }
  // return output;
}

void DKGraphPlannerNode::TransformPointsToBody(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOut, const double &veh_x_,
    const double &veh_y_, const double &veh_yaw_) {

  for (auto point : CloudIn->points) {
    pcl::PointXYZI pointOnBody;
    pointOnBody.x =
        (point.x - veh_x_) * cos(veh_yaw_) + (point.y - veh_y_) * sin(veh_yaw_);
    pointOnBody.y = -(point.x - veh_x_) * sin(veh_yaw_) +
                    (point.y - veh_y_) * cos(veh_yaw_);
    pointOnBody.z = point.z;
    pointOnBody.intensity = point.intensity;
    CloudOut->points.push_back(pointOnBody);
  }
}
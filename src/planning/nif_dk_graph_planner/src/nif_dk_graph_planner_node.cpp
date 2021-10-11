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

  this->m_OsmFileName = this->get_parameter("osm_name").as_string();
  this->m_RacingTrajectory =
      this->get_parameter("racing_trajectory").as_string();
  this->m_originLat = this->get_parameter("origin_lat").as_double();
  this->m_originLon = this->get_parameter("origin_lon").as_double();

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
          "/graph_planner/OsmParcer", nif::common::constants::QOS_EGO_ODOMETRY);

  pubFullNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/full_nodes", nif::common::constants::QOS_SENSOR_DATA);
  pubRacingLineRefPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/racing_line", nif::common::constants::QOS_SENSOR_DATA);
  pubFirstNodeContainPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/graph_planner/first_node_info", nif::common::constants::QOS_SENSOR_DATA);
  pubFinalPathPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/final_path_points", nif::common::constants::QOS_SENSOR_DATA);
  pubCandidatesNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/cadidates_points", nif::common::constants::QOS_SENSOR_DATA);
  pubCostPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/cost", nif::common::constants::QOS_SENSOR_DATA);

  SubOdometry =  this->create_subscription<nav_msgs::msg::Odometry>(
      "in_ekf_odometry", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&DKGraphPlannerNode::CallbackOdometry, this,
                std::placeholders::_1));
  SubOccupancyGrid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/semantics/costmap_generator/occupancy_grid",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&DKGraphPlannerNode::CallbackOccupancyGrid, this,
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
    RCLCPP_INFO(this->get_logger(), "Vaild file!");
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
    // attribute of way data : Node IDs
    for (pugi::xml_node way_data : way.children("nd")) {
      nif_dk_graph_planner_msgs::msg::Node nodeRef; // Node Ids in the each way
      nodeRef.id = way_data.attribute("ref").as_int();
      for (auto node : m_OsmParcer.nodes) {
        if (node.id == nodeRef.id) {
          nodeRef = node;
        }
      }
      wayTmp.nodes.push_back(nodeRef);
    }
    wayTmp.first_node_id = wayTmp.nodes[0].id;
    wayTmp.last_node_id = wayTmp.nodes[wayTmp.nodes.size() - 1].id;
    wayTmp.cost = INF;

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

  for(auto wayRegister : m_OsmParcer.ways) {
    m_WaysResister[wayRegister.start_layer].push_back(wayRegister);
    m_FirstNodeBasedResister[wayRegister.first_node_id].push_back(wayRegister);
  }


  RCLCPP_INFO(this->get_logger(),
              "Converting OSM file to the ros system is completed!");
  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(), "node size: %d" , m_OsmParcer.nodes.size());
  RCLCPP_INFO(this->get_logger(), "way size: %d" , m_OsmParcer.ways.size());
  RCLCPP_INFO(this->get_logger(), "layer size: %d", m_WaysResister.size());

  bParcingComplete = true;
}

void DKGraphPlannerNode::RacingLineParcing()
{
  m_racingLineRefPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<std::pair<std::string, std::vector<double>>> result = read_csv(m_RacingTrajectory);

  int start_layer, end_layer, start_node, end_node;
  double distance;

  int layer_start = -1;
  for (int i = 0; i < result[0].second.size(); i++) {
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
    m_OsmParcer.ways[i].cost = cost_sum;
    if (max_cost < cost_sum)
      max_cost = cost_sum;
  }
  // normalize cost : distance from the racing line
  for (int i = 0; i < m_OsmParcer.ways.size(); i++) {
    m_OsmParcer.ways[i].cost = m_OsmParcer.ways[i].cost / max_cost;
  }

  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(),
              "Racing line is loaded!");
  RCLCPP_INFO(this->get_logger(), "racing line size: %d" , m_racingLineRefPoints->points.size());

  bRacingLine = true;
}

void DKGraphPlannerNode::CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
  std::lock_guard<std::mutex> lock(mtx);

  if (bRacingLine && bParcingComplete) {

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
    int current_node_id;
    // get node id for planing
    // GetIntensityInfo(m_veh_x, m_veh_y, m_FirstNodeContainPoints, current_node_id);
    GetIntensityInfo(m_closest_x_in_racing_line, m_closest_y_in_racing_line,
                     m_FirstNodeContainPoints, current_node_id);
    
    // put prev best first node as closest start node
    // std::cout << "original : " << current_node_id << std::endl;
    // for (auto ways_with_same_first_node_id : m_FirstNodeBasedResister[current_node_id])
    // {
    //   int layer_id = ways_with_same_first_node_id.start_layer;
    //   for (auto ways_with_same_layer : m_WaysResister[layer_id])
    //   {
    //     if (ways_with_same_layer.end_node == m_prevBestEndNodeInLayer)
    //     {
    //       current_node_id = ways_with_same_layer.first_node_id;
    //       break;
    //     }
    //   }
    // }
    // // std::cout << "after : " << current_node_id << std::endl;
    if(m_prevStartFirstNodeId != -1)
      current_node_id = m_prevStartFirstNodeId;

    m_closestStartNode = current_node_id;

    double target_x, target_y;
    int target_idx = current_idx + 100;
    int target_node_id;
    size_t racingLineRefSize = m_racingLineRefPoints->points.size();
    target_idx = target_idx % racingLineRefSize;

    target_x = m_racingLineRefPoints->points[target_idx].x;
    target_y = m_racingLineRefPoints->points[target_idx].y;
    GetIntensityInfo(target_x, target_y, m_FirstNodeContainPoints, target_node_id);
    m_closestGoalNode = target_node_id;

    m_nearbyFirstNodes.clear();
    int search_num_idx = 10;
    int current_layer;
    getNearbyNodesFromLayer(m_veh_x, m_veh_y, search_num_idx, m_FullIndexedPoints,
                            current_layer, m_nearbyFirstNodes);

    m_currentLayer = current_layer;

    // std::cout << "------\n" ;
    // std::cout << "m_currentLayer : " << m_currentLayer << std::endl;
    // std::cout << "m_prevFirstLayer : " << m_prevFirstLayer << std::endl;
    // std::cout << "m_prevBestFirstNodeInLayer : " << m_prevBestFirstNodeInLayer << std::endl;
    // std::cout << "m_prevBestEndNodeInLayer : "<< m_prevBestEndNodeInLayer << std::endl;

    // for (auto node : m_nearbyFirstNodes)
    //   std::cout << node << std::endl;
    // std::cout << "m_nearbyFirstNodes.size() : " <<
    // m_nearbyFirstNodes.size()
    //           << std::endl;

    bOdometry = true;
  }
}

void DKGraphPlannerNode::CallbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
  m_OccupancyGrid = *msg;
  bOccupancyGrid = true;
}

void DKGraphPlannerNode::timer_callback() {
  if (bRacingLine && bParcingComplete && bBuildGraph) {
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
}

void DKGraphPlannerNode::GetIntensityInfo(
    const double &x_in, const double &y_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_points, int &intensity_out) {
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
    int& current_layer_out, std::vector<int> &nearby_indice_out) {

  if (!bParcingComplete)
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

  int layer_size = m_WaysResister.size();
  for(int i = 0; i < search_num_idx_in; i++)
  {
    int layer_idx = current_layer_out + i;
    if(layer_idx > layer_size)
      layer_idx = layer_idx % layer_size;

    for (auto way : m_WaysResister[layer_idx]) {
      nearby_indice_out.push_back(way.first_node_id);
    }
  }

  removeDuplicated(nearby_indice_out);
}

void DKGraphPlannerNode::BuildGraph()
{
  int V, E;
  V = m_OsmParcer.nodes.size(); // the number of node
  E = m_OsmParcer.ways.size();  // the number of way
  m_graph.clear();

  // Build a graph.
  for (auto way : m_OsmParcer.ways) {
    Connected connected_;
    connected_.id = way.last_node_id;
    connected_.cost = way.cost;
    connected_.additional_cost = 0;
    m_graph[way.first_node_id].push_back(connected_);
  }
  bBuildGraph = true;

  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(), "Graph is build!");
  RCLCPP_INFO(this->get_logger(), "graph size: %d", m_graph.size());
}

void DKGraphPlannerNode::UpdateGraph()
{
  if(!bOccupancyGrid)
    return;

  if (!bOdometry)
    return;

  // std::cout << "--------update graph----------\n";
  int count = 0;
  m_CostPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  for (auto nearbyNode : m_nearbyFirstNodes) {
    // std::cout << "----------------------\n";
    for (int i =0; i < m_FirstNodeBasedResister[nearbyNode].size(); i++)
    {
      auto way = m_FirstNodeBasedResister[nearbyNode][i];
      double cost_accumulated = 0;
      for (auto node : way.nodes) {
        double pt_x_global = node.x;
        double pt_y_global = node.y;

        //global to local
        double pt_x_body = (pt_x_global - m_veh_x) * cos(m_veh_yaw) +
                           (pt_y_global - m_veh_y) * sin(m_veh_yaw);
        double pt_y_body = -(pt_x_global - m_veh_x) * sin(m_veh_yaw) +
                           (pt_y_global - m_veh_y) * cos(m_veh_yaw);

        double cost = CorrespondingCost(pt_x_body, pt_y_body, m_OccupancyGrid);
        cost_accumulated += cost;
        count++;

        pcl::PointXYZI pointbuf;
        pointbuf.x = pt_x_global;
        pointbuf.y = pt_y_global;
        pointbuf.intensity = cost;
        m_CostPoints->points.push_back(pointbuf);
      }
      // std::cout << "cost_accumulated : " << cost_accumulated << std::endl;
      int j = 0;
      for (auto graph : m_graph[nearbyNode])
      {         
        if (graph.id == way.last_node_id) {
          m_graph[nearbyNode][j].additional_cost = cost_accumulated;
          if (m_graph[nearbyNode][j].additional_cost != 0.0)
          {
            // std::cout << "cost: " << m_graph[nearbyNode][j].cost
                      // << ", add : " << m_graph[nearbyNode][j].additional_cost
                      // << std::endl;
          }
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
  if (!bOccupancyGrid)
    return;

  for (auto nearbyNode : m_nearbyFirstNodes) {
    for (auto way : m_FirstNodeBasedResister[nearbyNode])
      for (int i = 0; i < m_graph[nearbyNode].size(); i++) {
        if (m_graph[nearbyNode][i].id == way.last_node_id) {
          // m_graph[nearbyNode][i].cost = way.cost;
          m_graph[nearbyNode][i].additional_cost = 0;
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

    qu.push({0., m_closestStartNode}); //Put init node in the que
    dist[m_closestStartNode] = 0.; // Update value of start point as 0
    // for(auto id : m_PlanningPat hNodes)
    //   std::cout << "m_PredSuccMap: " <<m_PredSuccMap[id] << std::endl;
    // std::cout << m_graph[m_closestStartNode].size() << std::endl;

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
        double nextcost = connected.cost + connected.additional_cost;
        // std::cout << "connected.cost: " << connected.cost << std::endl;
        // std::cout << "connected.additional_cost : " << connected.additional_cost << std::endl;

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
    // std::cout << "m_closestStartNode: " << m_closestStartNode << std::endl;
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
  for (auto node_id : m_FinalNodes) {
    for (auto final_id : m_FirstNodeBasedResister[node_id]) {
      if (!ExistInList(final_id.last_node_id, m_FinalNodes))
        continue;
      
      if (m_prevFirstLayer != m_currentLayer)
      {
        m_prevFirstLayer = m_currentLayer;
        std::cout << "layer changed!" << std::endl;
        if(m_UsePrevStartFirstNodeAfter2 < 2)
        {
          m_prevStartFirstNodeId = final_id.first_node_id;
        }
        else
        {
          m_prevStartFirstNodeId = m_FinalNodes[1];
        }

        m_UsePrevStartFirstNodeAfter2 ++;
        if (cnt == 0) {
          m_prevBestFirstNodeInLayer = final_id.start_node;
          m_prevBestEndNodeInLayer = final_id.end_node;
        }
      }

      for (auto nd : final_id.nodes) {
        pcl::PointXYZI point_buf;
        point_buf.x = nd.x;
        point_buf.y = nd.y;
        point_buf.z = nd.z;
        point_buf.intensity = cnt;
        finalcloud_ptr->points.push_back(point_buf);
        cnt++;
      }
    }
  }
  sensor_msgs::msg::PointCloud2 FinalPathCloudMsg;
  if(!finalcloud_ptr->points.empty())
    pcl::toROSMsg(*finalcloud_ptr, FinalPathCloudMsg);
  FinalPathCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FinalPathCloudMsg.header.stamp = this->now();
  pubFinalPathPoints->publish(FinalPathCloudMsg);
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

void DKGraphPlannerNode::removeDuplicated(std::vector<int> &v) {
  auto end = v.end();
  for (auto it = v.begin(); it != end; ++it) {
    end = std::remove(it + 1, end, *it);
  }

  v.erase(end, v.end());
}
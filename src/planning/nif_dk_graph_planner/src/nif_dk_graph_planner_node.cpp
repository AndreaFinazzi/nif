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
  // pubLaneArray = this->create_publisher<autoware_msgs::LaneArray>(
  //     "/graph_planner/origin_lane_waypoints_array", nif::common::constants::QOS_SENSOR_DATA);
  // pubFullLink = this->create_publisher<visualization_msgs::MarkerArray>(
  //     "/graph_planner/full_link", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalLink = this->create_publisher<visualization_msgs::MarkerArray>(
  //     "/graph_planner/final_link", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalLaneArray = this->create_publisher<autoware_msgs::LaneArray>(
  //     "/graph_planner/final_lane_waypoints_array", nif::common::constants::QOS_SENSOR_DATA);

  pubFullNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/full_nodes", nif::common::constants::QOS_SENSOR_DATA);
  pubRacingLinePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/graph_planner/racing_line", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //     "/graph_planner/final_node_points", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalPathPoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //     "/graph_planner/final_path_points", nif::common::constants::QOS_SENSOR_DATA);
  // pubCandidatesNodePoints = this->create_publisher<sensor_msgs::msg::PointCloud2>(
  //     "/graph_planner/cadidates_points", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalNodes = this->create_publisher<std_msgs::Int32MultiArray>(
  //     "/graph_planner/final_node_array", nif::common::constants::QOS_SENSOR_DATA);
  // pubFinalITSIds = this->create_publisher<nif_dk_graph_planner_msgs::msg::FinalPathIDArray>(
  //     "/graph_planner/final_path_id_array", nif::common::constants::QOS_SENSOR_DATA);

  // pubStartWay = this->create_publisher<geometry_msgs::PoseArray>(
  //     "/graph_planner/start_way", nif::common::constants::QOS_SENSOR_DATA);
  // pubGoalWay = this->create_publisher<geometry_msgs::PoseArray>(
  //     "/graph_planner/goal_way", nif::common::constants::QOS_SENSOR_DATA);

  // pubWptNodeCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/route_planner/goal_xy_nodes", 10, true);

  // SubInitPose = nh_.subscribe("/initialpose", 1,
  //                             &DKGraphPlannerNode::CallbackInitPoseRviz, this);
  // SubGoal = nh_.subscribe("/move_base_simple/goal", 1,
  //                         &DKGraphPlannerNode::CallbackGoalRviz, this);

  SubOdometry =  this->create_subscription<nav_msgs::msg::Odometry>(
      "in_ekf_odometry", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&DKGraphPlannerNode::CallbackOdometry, this,
                std::placeholders::_1));
  // SubGoalXYList = nh_.subscribe("/route_planner/goal_xy_list", 1,
  //                             &DKGraphPlannerNode::CallbackGoalXYList, this);

  OsmParcing();
  RacingLineParcing();
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
    wayTmp.cost = sqrt(
        pow(wayTmp.nodes[0].x - wayTmp.nodes[wayTmp.nodes.size() - 1].x, 2) +
        pow(wayTmp.nodes[0].y - wayTmp.nodes[wayTmp.nodes.size() - 1].y, 2));

    // Tags
    for (pugi::xml_node tag : way.children("tag")) {
      // start_layer
      std::string start_layer = "start_layer";
      // end_layer
      std::string end_layer = "end_layer";

      if (tag.attribute("k").as_string() == start_layer) {
        wayTmp.start_layer = tag.attribute("v").as_int();
      } else if (tag.attribute("k").as_string() == end_layer) {
        wayTmp.end_layer = tag.attribute("v").as_int();
      }
    }

    m_OsmParcer.ways.push_back(wayTmp);
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
  m_racingLine_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

  std::vector<std::pair<std::string, std::vector<double>>> result = read_csv(m_RacingTrajectory);

  for (int i = 0; i < result[0].second.size(); i++) {
    pcl::PointXYZI point_buf;
    point_buf.x = result[1].second[i];
    point_buf.y = result[2].second[i];
    point_buf.intensity = i;
    m_racingLine_points->points.push_back(point_buf);
  }

  RCLCPP_INFO(this->get_logger(), "------------------------------------");
  RCLCPP_INFO(this->get_logger(),
              "Racing line is loaded!");
  RCLCPP_INFO(this->get_logger(), "racing line size: %d" , m_racingLine_points->points.size());

  bRacingLine = true;
}

void DKGraphPlannerNode::CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) 
{
//   std::lock_guard<std::mutex> lock(mtx);
//   m_InitPose = msg.pose.pose;
//   m_Odometry = msg;
//   double minimumDistance = INF;
//   int closestNodeId = 0;

//   bInitPose = true;

//   if (!bReplanRoute)
//     return;

//   if (bParcingComplete) {
//     double bias, normal_distance;
//     double min_distance = INF;
//     int minWayIndex = 0;
//     int minNodeIndex = 0;
//     std::string minWayITS;
//     for (auto way : m_OsmParcer.ways) {
//       bool first_node = true;
//       double prev_x, prev_y;
//       for (auto node : way.nodes) {
//         // double cost_distance = sqrt(pow(node.x - msg.pose.pose.position.x, 2) +
//         //                             pow(node.y - msg.pose.pose.position.y, 2));

//         // if (cost_distance > 100) {
//         //   continue;
//         // }

//         if (first_node) {
//           prev_x = node.x;
//           prev_y = node.y;
//           first_node = false;
//           continue;
//         }

//         double slope;
//         if (node.x - prev_x == 0) {
//           slope = 0;
//         } else {
//           slope = (node.y - prev_y) / (node.x - prev_x);
//         }
//         bias = node.y - slope * node.x;
//         normal_distance = fabs(slope * msg.pose.pose.position.x -
//                                msg.pose.pose.position.y + bias) /
//                           sqrt(pow(slope, 2) + 1);

//         double prev_distance = sqrt(pow(prev_x - msg.pose.pose.position.x, 2) +
//                                     pow(prev_y - msg.pose.pose.position.y, 2));
//         double prev_to_node =
//             sqrt(pow(node.x - prev_x, 2) + pow(node.y - prev_y, 2));

//         double prod1 = (msg.pose.pose.position.x - prev_x) * (node.x - prev_x) +
//                        (msg.pose.pose.position.y - prev_y) * (node.y - prev_y);
//         double prod2 = (msg.pose.pose.position.x - node.x) * (prev_x - node.x) +
//                        (msg.pose.pose.position.y - node.y) * (prev_y - node.y);

//         if (prod1 < 0 || prod2 < 0)
//           continue;

//         if (normal_distance < min_distance) {
//           min_distance = normal_distance;
//           minWayIndex = way.id;
//           minWayITS = way.ITScurrentLinkID;
//           minNodeIndex = node.id;
//         }
//         prev_x = node.x;
//         prev_y = node.y;
//       }
//     }

//     m_closestWayId_start = m_Resister[minWayIndex].id;
//     if(!m_Resister[minWayIndex].nodes.empty())
//       m_closestStartNode = m_Resister[minWayIndex].nodes[0].id;

//     geometry_msgs::PoseArray StartWayPoseArray;
//     StartWayPoseArray.header = msg.header;
//     for (auto node : m_Resister[minWayIndex].nodes) {
//       geometry_msgs::Pose poseTmp;
//       poseTmp.position.x = node.x;
//       poseTmp.position.y = node.y;
//       poseTmp.position.z = 36.5;
//       poseTmp.orientation = tf::createQuaternionMsgFromRollPitchYaw(
//           0.0, 0.0, node.heading * M_PI / 180);
//       StartWayPoseArray.poses.push_back(poseTmp);
//     }
//     pubStartWay.publish(StartWayPoseArray);    
//     std::cout << "-----------Odom POSE INFO-----------" << std::endl;
//     std::cout << "minWayITS: "<< minWayITS  << std::endl;

//     bReplanRoute = false;
//   } else {
//     ROS_WARN("Map was not loaded yet. please wait for map loading...");
//   }
// }

// void DKGraphPlannerNode::CallbackInitPoseRviz(
//     const geometry_msgs::PoseWithCovarianceStamped &msg) {
//   std::lock_guard<std::mutex> lock(mtx);
//   m_InitPose = msg.pose.pose;
//   bInitPose = true;

//   RCLCPP_INFO(this->get_logger(), "New init pose is received! Searching for nearest way");
//   double minimumDistance = INF;
//   int closestNodeId = 0;

//   if (bParcingComplete) {
//     double bias, normal_distance;
//     double min_distance = INF;
//     int minWayIndex = 0;
//     int minNodeIndex = 0;
//     for (auto way : m_OsmParcer.ways) {
//       bool first_node = true;
//       double prev_x, prev_y;
//       for (auto node : way.nodes) {
//         double cost_distance = sqrt(pow(node.x - msg.pose.pose.position.x, 2) +
//                                     pow(node.y - msg.pose.pose.position.y, 2));

//         if (cost_distance > 100) {
//           continue;
//         }

//         if (first_node) {
//           prev_x = node.x;
//           prev_y = node.y;
//           first_node = false;
//           continue;
//         }

//         double slope;
//         if (node.x - prev_x == 0) {
//           slope = 0;
//         } else {
//           slope = (node.y - prev_y) / (node.x - prev_x);
//         }
//         bias = node.y - slope * node.x;
//         normal_distance = fabs(slope * msg.pose.pose.position.x -
//                                msg.pose.pose.position.y + bias) /
//                           sqrt(pow(slope, 2) + 1);

//         double prev_distance = sqrt(pow(prev_x - msg.pose.pose.position.x, 2) +
//                                     pow(prev_y - msg.pose.pose.position.y, 2));
//         double prev_to_node =
//             sqrt(pow(node.x - prev_x, 2) + pow(node.y - prev_y, 2));

//         double prod1 = (msg.pose.pose.position.x - prev_x) * (node.x - prev_x) +
//                        (msg.pose.pose.position.y - prev_y) * (node.y - prev_y);
//         double prod2 = (msg.pose.pose.position.x - node.x) * (prev_x - node.x) +
//                        (msg.pose.pose.position.y - node.y) * (prev_y - node.y);

//         if (prod1 < 0 || prod2 < 0)
//           continue;

//         if (normal_distance < min_distance) {
//           min_distance = normal_distance;
//           minWayIndex = way.id;
//           minNodeIndex = node.id;
//         }
//         prev_x = node.x;
//         prev_y = node.y;
//       }
//     }

//     m_closestWayId_start = m_Resister[minWayIndex].id;
//     if(!m_Resister[minWayIndex].nodes.empty())
//       m_closestStartNode = m_Resister[minWayIndex].nodes[0].id;


//     geometry_msgs::PoseArray StartWayPoseArray;
//     StartWayPoseArray.header = msg.header;
//     for (auto node : m_Resister[minWayIndex].nodes) {
//       geometry_msgs::Pose poseTmp;
//       poseTmp.position.x = node.x;
//       poseTmp.position.y = node.y;
//       poseTmp.position.z = 36.5;
//       poseTmp.orientation = tf::createQuaternionMsgFromRollPitchYaw(
//           0.0, 0.0, node.heading * M_PI / 180);
//       StartWayPoseArray.poses.push_back(poseTmp);
//     }
//     pubStartWay.publish(StartWayPoseArray);    
 

//     std::cout << "-----------INIT POSE INFO-----------" << std::endl;
//     std::cout << "closest node Id : " << minNodeIndex
//               << ", distance: " << min_distance << std::endl;
//     std::cout << "closest way Id : " << m_closestWayId_start << "\n"
//               << std::endl;
//   } else {
//     ROS_WARN("Map was not loaded yet. please wait for map loading...");
//   }
}

// void DKGraphPlannerNode::timerCallback(const ros::TimerEvent &event) {}

// void DKGraphPlannerNode::CallbackGoalRviz(const geometry_msgs::PoseStamped &msg) {
//   m_GoalPose = msg.pose;
//   bGoalPose = true;

//   RCLCPP_INFO(this->get_logger(), "New goal pose is received! Navigating to the goal pose.");
//   double minimumDistance = INF;
//   int closestNodeId = 0;

//   if (m_PrevGoalPose.position == msg.pose.position) {
//     return;
//   } else {
//     bReplanRoute = true;
//     RCLCPP_INFO(this->get_logger(), "New goal point!, X: %f, Y: %f", msg.pose.position.x,
//              msg.pose.position.y);
//   }
//   if (bParcingComplete) {
//     double bias, normal_distance;
//     double min_distance = INF;
//     int minWayIndex = 0;
//     int minNodeIndex = 0;
//     for (auto way : m_OsmParcer.ways) {
//       bool first_node = true;
//       double prev_x, prev_y;
//       for (auto node : way.nodes) {

//         if (first_node) {
//           prev_x = node.x;
//           prev_y = node.y;
//           first_node = false;
//           continue;
//         }

//         double slope;
//         if (node.x - prev_x == 0) {
//           slope = 0;
//         } else {
//           slope = (node.y - prev_y) / (node.x - prev_x);
//         }
//         bias = node.y - slope * node.x;
//         normal_distance =
//             fabs(slope * msg.pose.position.x - msg.pose.position.y + bias) /
//             sqrt(pow(slope, 2) + 1);

//         double prev_distance = sqrt(pow(prev_x - msg.pose.position.x, 2) +
//                                     pow(prev_y - msg.pose.position.y, 2));
//         double prev_to_node =
//             sqrt(pow(node.x - prev_x, 2) + pow(node.y - prev_y, 2));

//         double prod1 = (msg.pose.position.x - prev_x) * (node.x - prev_x) +
//                        (msg.pose.position.y - prev_y) * (node.y - prev_y);
//         double prod2 = (msg.pose.position.x - node.x) * (prev_x - node.x) +
//                        (msg.pose.position.y - node.y) * (prev_y - node.y);

//         if (prod1 < 0 || prod2 < 0)
//           continue;

//         if (normal_distance < min_distance) {
//           min_distance = normal_distance;
//           minWayIndex = way.id;
//           minNodeIndex = node.id;
//         }
//         prev_x = node.x;
//         prev_y = node.y;
//       }
//     }

//     m_closestWayId_goal = minWayIndex; //m_Resister[minWayIndex].id;
//     if(!m_Resister[minWayIndex].nodes.empty())
//       m_closestGoalNode = m_Resister[minWayIndex].nodes[m_Resister[minWayIndex].nodes.size()-1].id;

//     geometry_msgs::PoseArray GoalWayPoseArray;
//     GoalWayPoseArray.header = msg.header;
//     for (auto node : m_Resister[minWayIndex].nodes) {
//       geometry_msgs::Pose poseTmp;
//       poseTmp.position.x = node.x;
//       poseTmp.position.y = node.y;
//       poseTmp.position.z = 36.5;
//       poseTmp.orientation = tf::createQuaternionMsgFromRollPitchYaw(
//           0.0, 0.0, node.heading * M_PI / 180);
//       GoalWayPoseArray.poses.push_back(poseTmp);

//     }
//     pubGoalWay.publish(GoalWayPoseArray);

//     std::cout << "-----------GOAL POSE INFO-----------" << std::endl;
//     std::cout << "closest node Id : " << minNodeIndex
//               << ", distance: " << min_distance << std::endl;
//     std::cout << "closest way Id : " << m_closestWayId_goal << "\n"
//               << std::endl;

//     m_PrevGoalPose = msg.pose;

//   } else {
//     ROS_WARN("Map was not loaded yet. please wait for map loading...");
//   }
// }

void DKGraphPlannerNode::timer_callback() {
  if (bRacingLine && bParcingComplete)
  {

    //   m_OsmParcer.header.frame_id = "map";
    //   m_OsmParcer.header.stamp = ros::Time::now();
    //   pubOsmParcer.publish(m_OsmParcer);
    //   Planning();

    MessagePublisher();
  //   ToPathMsg();
  }
}

void DKGraphPlannerNode::MessagePublisher() {
  // origin node publisher  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  int count = 0;
  for (auto node : m_OsmParcer.nodes) {
    pcl::PointXYZI current_point;
    current_point.x = node.x;
    current_point.y = node.y;
    current_point.z = node.z;
    current_point.intensity = node.start_layer;

    cloud_ptr->points.push_back(current_point);
  }
  sensor_msgs::msg::PointCloud2 FullNodeCloudMsg;
  pcl::toROSMsg(*cloud_ptr, FullNodeCloudMsg);
  FullNodeCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  FullNodeCloudMsg.header.stamp = this->now();
  pubFullNodePoints->publish(FullNodeCloudMsg);


  // racing line publisher
  sensor_msgs::msg::PointCloud2 RacingLineCloudMsg;
  pcl::toROSMsg(*m_racingLine_points, RacingLineCloudMsg);
  RacingLineCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
  RacingLineCloudMsg.header.stamp = this->now();
  pubRacingLinePoints->publish(RacingLineCloudMsg);
}

void DKGraphPlannerNode::SearchGraph(double &x_in, const double &y_in,
                                     int &start_layer_out, int &end_layer_out,
                                     int &node_out) 
{
  
}

// bool DKGraphPlannerNode::ExistInList(int id, std::vector<int> list)
// {
//   bool exist = false;
//   for(auto scan : list)
//   {
//     if(scan == id)
//     {
//       exist = true;
//       break;
//     }
//   }
//   // bool checkWpt = ExistInList(way.id, m_GoalXY_to_way_id_List);

//   return exist;
// }

// void DKGraphPlannerNode::Planning() {
//   if (bInitPose && bGoalPose) {
//     int V, E;

//     V = m_OsmParcer.nodes.size(); // the number of node
//     E = m_OsmParcer.ways.size();  // the number of way
//     m_graph.clear();
//     m_PlanningPathNodes.clear();
//     // Build a graph.
//     for (auto way : m_OsmParcer.ways) {
      
      
//       Connected connected_;
//       connected_.id = way.lastNodeId;
//       connected_.cost = way.cost;
//       m_graph[way.firstNodeId].push_back(connected_);

//       if (way.leftWayLastNodeId != 0) {
//         Connected left;
//         left.id = way.leftWayLastNodeId;
//         left.cost = way.cost + 15;
//         m_graph[way.firstNodeId].push_back(left);
//       }

//       if (way.rightWayLastNodeId != 0) {
//         Connected right;
//         right.id = way.rightWayLastNodeId;
//         right.cost = way.cost + 15;
//         m_graph[way.firstNodeId].push_back(right);
//       }
//     }

//     // Initialize the dist array with infinite values.
//     std::unordered_map<int, double> dist;
//     int path_index = 0;
//     for (auto node : m_OsmParcer.nodes) {
//       dist[node.id] = INF;
//     }
//     std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
//                         std::greater<std::pair<int, int>>>
//         qu;

//     qu.push({0, m_closestStartNode}); //우선순위 큐에 시작점을 넣어줍니다.
//     dist[m_closestStartNode] = 0; // Update value of start point as 0

//     m_PredSuccMap.clear();
//     while (!qu.empty()) {

//       int cost = qu.top().first;  // cost : distance to next node
//       int here = qu.top().second; // here : current node Id

//       qu.pop();

//       m_PlanningPathNodes.push_back(here);

//       if (here == m_closestGoalNode) {
//         // std::cout << "path finding!" << std::endl;
//         // m_PlanningPathNodes.pop_back();
//         break;
//       }

//       for (auto connected : m_graph[here]) {
//         int next = connected.id;
//         int nextcost = connected.cost;

//         if (dist[next] > dist[here] + nextcost) {
//           dist[next] = dist[here] + nextcost;
//           qu.push({dist[next], next});
//           m_PredSuccMap[next] = here;
//         }
//       }
//     }
//     // if(!m_PlanningPathNodes.empty())
//     //   m_PlanningPathNodes.pop_front();
//     // for(auto id : m_PlanningPathNodes)
//     //   std::cout << "m_PredSuccMap: " <<m_PredSuccMap[id] << std::endl;
//     std::deque<int> FinalPathQue;
//     auto next = m_PredSuccMap[m_closestGoalNode];
//     // std::cout <<"m_closestGoalNode: " << m_closestGoalNode << std::endl;
//     // std::cout << "next: "<<next << std::endl;
//     FinalPathQue.push_back(m_closestGoalNode);
//     FinalPathQue.push_back(next);
//     for (int i = 0; i < m_PlanningPathNodes.size(); i++) { // m_PredSuccMap
//       next = m_PredSuccMap[next];
//       FinalPathQue.push_back(next);
//     }
//     m_FinalNodes.clear();
//     for (auto final : FinalPathQue) {
//       m_FinalNodes.push_back(final);
//     }
//     std::reverse(m_FinalNodes.begin(), m_FinalNodes.end());
    

//     // //test
//     // std::cout << "m_FinalNodes : "<< m_FinalNodes.size() << std::endl;
//     // bGoalPose = false;
//   }
// }

// void DKGraphPlannerNode::ToPathMsg() {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr final_node_cloud_ptr(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr candidates_ptr(
//       new pcl::PointCloud<pcl::PointXYZI>);

//   int count = 0;
//   int cnt = 0;
//   // Candidates Nodes
//   for (auto node_id : m_PlanningPathNodes) {
//     for (auto node : m_OsmParcer.nodes) {
//       if (node.id != node_id) {
//         continue;
//       }
//       pcl::PointXYZI current_point;
//       current_point.x = node.x;
//       current_point.y = node.y;
//       current_point.z = node.z;
//       current_point.intensity = count;
//       candidates_ptr->points.push_back(current_point);
//       count++;
//     }
//   }
//   sensor_msgs::msg::PointCloud2 CandidatesNodeCloudMsg;
//   if(!candidates_ptr->points.empty())
//     pcl::toROSMsg(*candidates_ptr, CandidatesNodeCloudMsg);
//   CandidatesNodeCloudMsg.header.frame_id = "map";
//   CandidatesNodeCloudMsg.header.stamp = ros::Time::now();
//   pubCandidatesNodePoints.publish(CandidatesNodeCloudMsg);

//   // Final Nodes
//   for (auto node_id : m_FinalNodes) {
//     for (auto node : m_OsmParcer.nodes) {
//       if (node.id != node_id) {
//         continue;
//       }

//       pcl::PointXYZI current_point;
//       current_point.x = node.x;
//       current_point.y = node.y;
//       current_point.z = node.z;
//       current_point.intensity = count;
//       final_node_cloud_ptr->points.push_back(current_point);
//       count++;
//     }
//   }
//   sensor_msgs::msg::PointCloud2 FinalNodeCloudMsg;
//   if(!final_node_cloud_ptr->points.empty())
//     pcl::toROSMsg(*final_node_cloud_ptr, FinalNodeCloudMsg);
//   FinalNodeCloudMsg.header.frame_id = "map";
//   FinalNodeCloudMsg.header.stamp = ros::Time::now();
//   pubFinalNodePoints.publish(FinalNodeCloudMsg);

//   std_msgs::Int32MultiArray FinalNodeArray;
//   FinalNodeArray.data.clear();
//   for (auto node : m_FinalNodes)
//     FinalNodeArray.data.push_back(node);
//   pubFinalNodes.publish(FinalNodeArray);

//   nif_dk_graph_planner_msgs::msg::FinalPathIDArray FinalPathIDArrayMsg;
//   FinalPathIDArrayMsg.ids.clear();
//   FinalPathIDArrayMsg.ITSids.clear();

//   // //For full link
//   if(m_FinalNodes.empty())
//     return;

//   visualization_msgs::MarkerArray finalLink;
//   for (int i = 0; i < m_FinalNodes.size() - 1; i++) {

//     bool NeedToChangeLane = true;
//     for (auto way : m_OsmParcer.ways) {
//       if (m_FinalNodes[i] == way.firstNodeId &&
//           m_FinalNodes[i + 1] == way.lastNodeId) {
//         visualization_msgs::Marker link;
//         link.header.frame_id = "map";
//         link.header.stamp = ros::Time::now();
//         link.id = way.id;
//         link.type = visualization_msgs::Marker::LINE_STRIP;
//         link.action = visualization_msgs::Marker::ADD;
//         link.scale.x = 0.5, link.scale.y = 0.5, link.scale.z = 0.1;
//         link.color.r = 0, link.color.g = 1, link.color.b = 0,
//         link.color.a = 0.8;
//         for (auto nodeInWay : way.nodes) {
//           geometry_msgs::Point pointTmp;
//           pointTmp.x = nodeInWay.x;
//           pointTmp.y = nodeInWay.y;
//           pointTmp.z = way.speedLimit;
//           link.points.push_back(pointTmp);

//           std_msgs::ColorRGBA colorTmp;
//           colorTmp.a = 0.8;
//           colorTmp.r = 0.0;
//           colorTmp.g = 1.0;
//           colorTmp.b = 0.0;
//           link.colors.push_back(colorTmp);

//           NeedToChangeLane = false;
//         }
//         finalLink.markers.push_back(link);

//         FinalPathIDArrayMsg.ids.push_back(way.id);
//         FinalPathIDArrayMsg.ITSids.push_back(way.ITScurrentLinkID);
//       }
//     }

//     if (NeedToChangeLane) {
//       for (auto way : m_OsmParcer.ways) {
//         // Lane Change To Left
//         if (m_FinalNodes[i] == way.firstNodeId &&
//             m_FinalNodes[i + 1] == way.leftWayLastNodeId) {
//           visualization_msgs::Marker link;
//           link.header.frame_id = "map";
//           link.header.stamp = ros::Time::now();
//           link.id = way.id;
//           link.type = visualization_msgs::Marker::LINE_STRIP;
//           link.action = visualization_msgs::Marker::ADD;
//           link.scale.x = 0.5, link.scale.y = 0.5, link.scale.z = 0.1;
//           link.color.r = 1.0, link.color.g = 0.0, link.color.b = 0.0,
//           link.color.a = 0.8;
//           link.text = std::to_string(way.speedLimit);
//           for (auto node : m_OsmParcer.nodes) {
//             if (node.id == way.firstNodeId) {
//               geometry_msgs::Point pointTmp;
//               pointTmp.x = node.x;
//               pointTmp.y = node.y;
//               pointTmp.z = way.speedLimit;
//               link.points.push_back(pointTmp);

//               std_msgs::ColorRGBA colorTmp;
//               colorTmp.a = 0.8;
//               colorTmp.r = 1.0;
//               colorTmp.g = 0.0;
//               colorTmp.b = 0.0;
//               link.colors.push_back(colorTmp);
//             } else if (node.id == way.leftWayLastNodeId) {
//               geometry_msgs::Point pointTmp;
//               pointTmp.x = node.x;
//               pointTmp.y = node.y;
//               pointTmp.z = way.speedLimit;
//               link.points.push_back(pointTmp);

//               std_msgs::ColorRGBA colorTmp;
//               colorTmp.a = 0.8;
//               colorTmp.r = 1.0;
//               colorTmp.g = 0.0;
//               colorTmp.b = 0.0;
//               link.colors.push_back(colorTmp);
//             }
//           }
//           finalLink.markers.push_back(link);

//           FinalPathIDArrayMsg.ids.push_back(way.id);
//           FinalPathIDArrayMsg.ITSids.push_back(way.ITScurrentLinkID);
//         }
//         // Lane Change To Right
//         if (m_FinalNodes[i] == way.firstNodeId &&
//             m_FinalNodes[i + 1] == way.rightWayLastNodeId) {
//           visualization_msgs::Marker link;
//           link.header.frame_id = "map";
//           link.header.stamp = ros::Time::now();
//           link.id = way.id;
//           link.type = visualization_msgs::Marker::LINE_STRIP;
//           link.action = visualization_msgs::Marker::ADD;
//           link.scale.x = 0.5, link.scale.y = 0.5, link.scale.z = 0.1;
//           link.color.r = 1, link.color.g = 0, link.color.b = 0,
//           link.color.a = 0.8;
//           for (auto node : m_OsmParcer.nodes) {
//             if (node.id == way.firstNodeId) {
//               geometry_msgs::Point pointTmp;
//               pointTmp.x = node.x;
//               pointTmp.y = node.y;
//               pointTmp.z = way.speedLimit;
//               link.points.push_back(pointTmp);

//               std_msgs::ColorRGBA colorTmp;
//               colorTmp.a = 0.8;
//               colorTmp.r = 0.0;
//               colorTmp.g = 0.0;
//               colorTmp.b = 1.0;
//               link.colors.push_back(colorTmp);
//             } else if (node.id == way.rightWayLastNodeId) {
//               geometry_msgs::Point pointTmp;
//               pointTmp.x = node.x;
//               pointTmp.y = node.y;
//               pointTmp.z = way.speedLimit;
//               link.points.push_back(pointTmp);

//               std_msgs::ColorRGBA colorTmp;
//               colorTmp.a = 0.8;
//               colorTmp.r = 0.0;
//               colorTmp.g = 0.0;
//               colorTmp.b = 1.0;
//               link.colors.push_back(colorTmp);
//             }
//           }
//           finalLink.markers.push_back(link);

//           FinalPathIDArrayMsg.ids.push_back(way.id);
//           FinalPathIDArrayMsg.ITSids.push_back(way.ITScurrentLinkID);
//         }
//       }
//     }
//   }
//   pubFinalLink.publish(finalLink); // MarkerArray
//   pubFinalITSIds.publish(FinalPathIDArrayMsg);

//   // Calculate Lane using nodes
//   pcl::PointCloud<pcl::PointXYZI>::Ptr final_path_cloud_ptr(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   autoware_msgs::Lane FinalLaneTmp;
//   m_FinalLaneArray.lanes.clear();
//   std::vector<std::pair<geometry_msgs::Pose, std::string>> FinalPathBuf;
//   geometry_msgs::Point prevPoint;
//   int link_index = 0;
//   for (auto link : finalLink.markers) {
//     for (int i =0; i < link.points.size(); i++) {
//       auto point = link.points[i];
//       auto color = link.colors[i];
//       bool exist = false;
//       std::pair<geometry_msgs::Pose, std::string> PoseBuf;
//       PoseBuf.first.position.x = point.x;
//       PoseBuf.first.position.y = point.y;
//       PoseBuf.first.position.z = point.z; // speed limit
//       // PoseBuf.first.orientation.w = 1;
//       double delta_x, delta_y, yaw;
//       if(link_index == 0 && link.points.size() > 1)
//       {
//         delta_x = link.points[link.points.size() -1].x - link.points[0].x;
//         delta_y = link.points[link.points.size() -1].y - link.points[0].y;
//       }
//       else
//       {
//         delta_x = point.x - prevPoint.x;
//         delta_y = point.y - prevPoint.y;
//       }
//       if(point.x - prevPoint.x == 0)
//         delta_x = 0.0001;
//       yaw = atan2(delta_y, delta_x);      
//       PoseBuf.first.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);        
//       prevPoint = point;
      
//       if(color.r == 1.0)
//         PoseBuf.second = "left";
//       else if(color.b == 1.0)
//         PoseBuf.second = "right";
//       else
//       {
//         PoseBuf.second= "";
//       }
      
//       for (auto check : FinalPathBuf) {
//         if (check.first.position.x == point.x && check.first.position.y == point.y)
//           exist = true;
//       }
//       if (!exist)
//         FinalPathBuf.push_back(PoseBuf);

//       // std::cout << PoseBuf << std::endl;
//     }
//     link_index ++;
//   }

//   if (!FinalPathBuf.empty()) {
//     FinalLaneTmp.header.frame_id = "map";
//     for (int i = 0; i < FinalPathBuf.size() - 1; i++) {
//       double BetweenPointsDist = sqrt(
//           pow(FinalPathBuf[i + 1].first.position.x - FinalPathBuf[i].first.position.x, 2) +
//           pow(FinalPathBuf[i + 1].first.position.y - FinalPathBuf[i].first.position.y, 2));
//       double step_thres = 1;
//       int step_number = BetweenPointsDist / step_thres;
//       if (BetweenPointsDist < step_thres) {
//         autoware_msgs::Waypoint WaypointTmp;
//         WaypointTmp.pose.pose.position.x = FinalPathBuf[i].first.position.x;
//         WaypointTmp.pose.pose.position.y = FinalPathBuf[i].first.position.y;
//         WaypointTmp.pose.pose.position.z = 0;
//         WaypointTmp.twist.twist.linear.x = FinalPathBuf[i].first.position.z;
//         WaypointTmp.pose.pose.orientation= FinalPathBuf[i].first.orientation;

//         if(FinalPathBuf[i+1].second == "left")
//         {
//           WaypointTmp.twist.twist.linear.y = 1;
//         }
//         else if(FinalPathBuf[i+1].second == "right")
//         {
//           WaypointTmp.twist.twist.linear.y = 2;
//         }
//         else
//         {
//           WaypointTmp.twist.twist.linear.y = 0;
//         }
        

//         // WaypointTmp.pose.pose.orientation = FinalPathBuf[i].first.orientation;
//         // WaypointTmp.twist.twist.linear.x = std::stod(link.text); //km/h
//         FinalLaneTmp.waypoints.push_back(WaypointTmp);

//         pcl::PointXYZI current_point;
//         current_point.x = WaypointTmp.pose.pose.position.x;
//         current_point.y = WaypointTmp.pose.pose.position.y;
//         current_point.z = cnt * 0.1; // 0;
//         current_point.intensity = cnt;
//         final_path_cloud_ptr->points.push_back(current_point);
//         cnt++;
//       } else {
//         for (int j = 0; j < step_number; j++) {
//           autoware_msgs::Waypoint WaypointTmp;
//           WaypointTmp.pose.pose.position.x =
//               FinalPathBuf[i].first.position.x +
//               (FinalPathBuf[i + 1].first.position.x - FinalPathBuf[i].first.position.x) *
//                   j / step_number;
//           WaypointTmp.pose.pose.position.y =
//               FinalPathBuf[i].first.position.y +
//               (FinalPathBuf[i + 1].first.position.y - FinalPathBuf[i].first.position.y) *
//                   j / step_number;
//           WaypointTmp.pose.pose.position.z = 0;
//           WaypointTmp.pose.pose.orientation = FinalPathBuf[i].first.orientation;

//           if(FinalPathBuf[i].first.position.z == FinalPathBuf[i + 1].first.position.z)
//             WaypointTmp.twist.twist.linear.x = FinalPathBuf[i].first.position.z;
          
//           if(FinalPathBuf[i+1].second == "left")
//           {
//             WaypointTmp.twist.twist.linear.y = 1;
//           }
//           else if(FinalPathBuf[i+1].second == "right")
//           {
//             WaypointTmp.twist.twist.linear.y = 2;
//           }
//           else
//           {
//             WaypointTmp.twist.twist.linear.y = 0;
//           }          
//           WaypointTmp.pose.pose.orientation = FinalPathBuf[i].first.orientation;
//           // WaypointTmp.twist.twist.linear.x = std::stod(link.text); //km/h
//           FinalLaneTmp.waypoints.push_back(WaypointTmp);

//           pcl::PointXYZI current_point;
//           current_point.x = WaypointTmp.pose.pose.position.x;
//           current_point.y = WaypointTmp.pose.pose.position.y;
//           current_point.z = cnt * 0.1; // WaypointTmp.pose.pose.position.z;
//           current_point.intensity = cnt;
//           final_path_cloud_ptr->points.push_back(current_point);
//           cnt++;
//         }
//       }
//     }
//   } else {
//     ROS_WARN("No Path");
//   }

//   if (FinalLaneTmp.waypoints.empty())
//     return;

//   m_FinalLaneArray.lanes.push_back(FinalLaneTmp);
//   pubFinalLaneArray.publish(m_FinalLaneArray);

//   sensor_msgs::msg::PointCloud2 FinalPathCloudMsg;
//   pcl::toROSMsg(*final_path_cloud_ptr, FinalPathCloudMsg);
//   FinalPathCloudMsg.header.frame_id = "map";
//   FinalPathCloudMsg.header.stamp = ros::Time::now();
//   pubFinalPathPoints.publish(FinalPathCloudMsg);
// }



// void DKGraphPlannerNode::CallbackGoalXYList(const sensor_msgs::msg::PointCloud2ConstPtr& msg)
// {
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);  
//   pcl::fromROSMsg(*msg, *cloud_ptr);

//   if(!bParcingComplete)
//     return;

//   m_GoalXY_to_way_id_List.clear();
//   m_GoalXYList.clear();

//   for(auto point : cloud_ptr->points)
//   {
//     // std::cout << point.x << ", " << point.y << std::endl;

//     double bias, normal_distance;
//     double min_distance = INF;
//     int minWayIndex = 0;
//     int minNodeIndex = 0;
//     std::string minWayITS;
//     for (auto way : m_OsmParcer.ways) {
//       bool first_node = true;
//       double prev_x, prev_y;
//       for (auto node : way.nodes) {

//         if (first_node) {
//           prev_x = node.x;
//           prev_y = node.y;
//           first_node = false;
//           continue;
//         }

//         double slope;
//         if (node.x - prev_x == 0) {
//           slope = 0;
//         } else {
//           slope = (node.y - prev_y) / (node.x - prev_x);
//         }
//         bias = node.y - slope * node.x;
//         normal_distance = fabs(slope * point.x -
//                               point.y + bias) /
//                           sqrt(pow(slope, 2) + 1);

//         double prev_distance = sqrt(pow(prev_x - point.x, 2) +
//                                     pow(prev_y - point.y, 2));
//         double prev_to_node =
//             sqrt(pow(node.x - prev_x, 2) + pow(node.y - prev_y, 2));

//         double prod1 = (point.x - prev_x) * (node.x - prev_x) +
//                       (point.y - prev_y) * (node.y - prev_y);
//         double prod2 = (point.x - node.x) * (prev_x - node.x) +
//                       (point.y - node.y) * (prev_y - node.y);

//         if (prod1 < 0 || prod2 < 0)
//           continue;

//         if (normal_distance < min_distance) {
//           min_distance = normal_distance;
//           minWayIndex = way.id;
//           minWayITS = way.ITScurrentLinkID;
//           minNodeIndex = node.id;
//         }
//         prev_x = node.x;
//         prev_y = node.y;
//       }
//     }

//     if(!ExistInList(minWayIndex, m_GoalXY_to_way_id_List))
//     {
//       geometry_msgs::Pose PoseBuf;
//       PoseBuf.position.x = point.x;
//       PoseBuf.position.y = point.y;
//       m_GoalXY_to_way_id_List.push_back(minWayIndex);
//       m_GoalXYList.push_back(PoseBuf);
//     }
//   }

//   // m_closestWayId_start = m_Resister[m_GoalXY_to_way_id_List[0]].id;
//   // if(!m_Resister[m_GoalXY_to_way_id_List[0]].nodes.empty())
//   //   m_closestStartNode = m_Resister[m_GoalXY_to_way_id_List[0]].nodes[0].id;
  
//   // int listsize = m_GoalXY_to_way_id_List.size();
//   // m_closestWayId_goal = m_Resister[m_GoalXY_to_way_id_List[listsize - 1]].id;
//   // int lastNodesize = m_Resister[m_GoalXY_to_way_id_List[listsize - 1]].nodes.size();
//   // if(!m_Resister[m_GoalXY_to_way_id_List[listsize - 1]].nodes.empty())
//   //   m_closestGoalNode = m_Resister[m_GoalXY_to_way_id_List[listsize - 1]].nodes[lastNodesize - 1].id;


//   std::vector<std::vector<int>> StackFinalNodes;
//   StackFinalNodes.clear();

//   bCSVGoalList = true;

//   m_graph.clear();
//   //  Build a graph.
//   for (auto way : m_OsmParcer.ways) {

//     bool exist = ExistInList(way.id, m_GoalXY_to_way_id_List);

//     Connected connected_;
//     connected_.id = way.lastNodeId;
//     connected_.cost = way.cost;
//     if(exist)
//     {
//       // connected_.cost = way.cost - 20;
//     }
      
//     m_graph[way.firstNodeId].push_back(connected_);
    
//     if (way.leftWayLastNodeId != 0) {
//       Connected left;
//       left.id = way.leftWayLastNodeId;
//       left.cost = way.cost + 15;
//       m_graph[way.firstNodeId].push_back(left);
//     }

//     if (way.rightWayLastNodeId != 0) {
//       Connected right;
//       right.id = way.rightWayLastNodeId;
//       right.cost = way.cost + 15;
//       m_graph[way.firstNodeId].push_back(right);
//     }
//   }

//   pcl::PointCloud<pcl::PointXYZI>::Ptr node_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);  

//   int cnt_node =0;
//   for(auto id : m_GoalXY_to_way_id_List)
//   {
//     // for(auto node : m_Resister[id].nodes)
//     // {
//     int size_ = m_Resister[id].nodes.size() - 1;
//     pcl::PointXYZI current_point;
//     current_point.x = m_Resister[id].nodes[0].x;
//     current_point.y = m_Resister[id].nodes[0].y;
//     current_point.intensity = cnt_node;
//     cnt_node++;
//     node_cloud_ptr->points.push_back(current_point);
//     // }
//   }

//   m_PlanningPathNodes.clear();
//   if(m_GoalXY_to_way_id_List.size() > 2)
//   {
//     // std::cout << "-----\n";
//     int start = m_Resister[m_GoalXY_to_way_id_List[0]].firstNodeId;
    
//     for(int i = 0; i < m_GoalXY_to_way_id_List.size()-1; i++)
//     {
//       int goal_id = m_GoalXY_to_way_id_List[i+1];
//       double ref_x = m_GoalXYList[i].position.x;
//       double ref_y = m_GoalXYList[i].position.y;

//       int current_id = m_GoalXY_to_way_id_List[i];
//       double ref_to_start = sqrt(pow(m_Resister[current_id].nodes[0].x - ref_x , 2) + 
//                                  pow(m_Resister[current_id].nodes[0].y - ref_y , 2));
//       double ref_to_goal = sqrt(pow(m_Resister[current_id].nodes[m_Resister[current_id].nodes.size()-1].x - ref_x , 2) + 
//                                 pow(m_Resister[current_id].nodes[m_Resister[current_id].nodes.size()-1].y - ref_y , 2));
//       double next;


//       pcl::PointXYZI current_point;
//       current_point.intensity = cnt_node;
//       if(ref_to_start < ref_to_goal)
//       {
//         next = m_Resister[goal_id].firstNodeId;
//         // std::cout << "first: " << ref_to_start << ", "<< ref_to_goal <<std::endl;
//         current_point.x = m_Resister[current_id].nodes[0].x;
//         current_point.y = m_Resister[current_id].nodes[0].y;
//       }
//       else
//       {
//         next = m_Resister[goal_id].lastNodeId;
//         // std::cout << "last: "  << ref_to_start << ", "<< ref_to_goal << std::endl;
//         int size_ = m_Resister[current_id].nodes.size() - 1;
//         current_point.x = m_Resister[current_id].nodes[size_].x;
//         current_point.y = m_Resister[current_id].nodes[size_].y;
//       }
      
//       if(PlanningWithCSV(start, next).size() > 10)
//       {
//         next = m_Resister[goal_id].lastNodeId;
//         // std::cout << "first: " << ref_to_start << ", "<< ref_to_goal <<std::endl;
//         current_point.x = m_Resister[current_id].nodes[0].x;
//         current_point.y = m_Resister[current_id].nodes[0].y;
//       }


//       cnt_node++;
//       node_cloud_ptr->points.push_back(current_point);

//       if(i == m_GoalXY_to_way_id_List.size()-2)
//         next = m_Resister[goal_id].lastNodeId;
 
//       // std::cout << "step: " << PlanningWithCSV(start, next).size() << std::endl;
//       // std::cout << "m_Resister[goal_id].ITScurrentLinkID: \n"; // << PlanningWithCSV(start, next).size()  << std::endl;
//       // // if(m_Resister[start_id].lastNodeId == m_Resister[goal_id].firstNodeId)
//       // //   std::cout << "yes" << std::endl;
//       // // else
//       // // {
//       //   // std::cout << "no" << std::endl;
//       StackFinalNodes.push_back(PlanningWithCSV(start, next));

//       start = next;
//       // }
//       for(auto id : m_PlanningPathNodesForCSV)
//       {
//         m_PlanningPathNodes.push_back(id);
//       }  
    
//       sensor_msgs::msg::PointCloud2 WptNodeCloudMsg;
//       pcl::toROSMsg(*node_cloud_ptr, WptNodeCloudMsg);
//       WptNodeCloudMsg.header.frame_id = "map";
//       WptNodeCloudMsg.header.stamp = ros::Time::now();
//       pubWptNodeCloud.publish(WptNodeCloudMsg);      
//     }


//     m_FinalNodes.clear();

//     for(auto FinalNodes : StackFinalNodes)
//     {
//       for(auto final : FinalNodes)
//       {
//         m_FinalNodes.push_back(final);
//       }
//     }
  
//   }

// }


// std::vector<int> DKGraphPlannerNode::PlanningWithCSV(const int start, const int goal) { 

//   std::vector<int> FinalNodesForCSV;
//   FinalNodesForCSV.clear();
//   if (bCSVGoalList) {

//     m_PlanningPathNodesForCSV.clear();
 
//     // Initialize the dist array with infinite values.
//     std::unordered_map<int, double> dist;
//     int path_index = 0;
//     for (auto node : m_OsmParcer.nodes) {
//       dist[node.id] = INF;
//     }
//     std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>,
//                         std::greater<std::pair<int, int>>>
//         qu;

//     qu.push({0, start}); //우선순위 큐에 시작점을 넣어줍니다.
//     dist[start] = 0; // Update value of start point as 0

//     m_PredSuccMap.clear();
//     while (!qu.empty()) {

//       int cost = qu.top().first;  // cost : distance to next node
//       int here = qu.top().second; // here : current node Id

//       qu.pop();

//       m_PlanningPathNodesForCSV.push_back(here);
//       // std::cout << here << std::endl;

//       if (here == goal) {
//         // std::cout << "path finding!" << std::endl;
//         // m_PlanningPathNodesForCSV.pop_back();
//         break;
//       }

//       for (auto connected : m_graph[here]) {
//         int next = connected.id;
//         int nextcost = connected.cost;

//         if (dist[next] > dist[here] + nextcost) {
//           dist[next] = dist[here] + nextcost;
//           qu.push({dist[next], next});
//           m_PredSuccMap[next] = here;
//         }
//       }
//     }

//     if(!m_PlanningPathNodesForCSV.empty())
//       m_PlanningPathNodesForCSV.pop_front();
//     // for(auto id : m_PlanningPathNodesForCSV)
//     //   std::cout << "m_PredSuccMap: " <<m_PredSuccMap[id] << std::endl;
//     std::deque<int> FinalPathQue;
//     auto next = m_PredSuccMap[goal];
    
//     // std::cout << "goal: " << goal << ", next: " << next << std::endl;
//     FinalPathQue.push_back(goal);
//     FinalPathQue.push_back(next);
//     for (int i = 0; i < m_PredSuccMap.size(); i++) { // m_PredSuccMap
//       next = m_PredSuccMap[next];
//       FinalPathQue.push_back(next);
//     }

//     // std::cout << "\n";
//     FinalNodesForCSV.clear();
//     for (auto final : FinalPathQue) {
//       if(final != 0)
//       {
//         FinalNodesForCSV.push_back(final);
//         // std::cout << final << ", ";
//       }
        
//     }
//     std::reverse(FinalNodesForCSV.begin(), FinalNodesForCSV.end());
    
//   }
//   else
//   {
//     FinalNodesForCSV.clear();
//   }
  
//   return FinalNodesForCSV;
// }

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
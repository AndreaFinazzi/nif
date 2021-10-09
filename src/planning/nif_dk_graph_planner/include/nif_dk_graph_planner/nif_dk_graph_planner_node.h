/*
 * nif_dk_graph_planner_node.h
 *
 *  Created on: Oct 9, 2021
 *      Author: Daegyu Lee
 */
#ifndef DK_GRAPH_PLANNER_H
#define DK_GRAPH_PLANNER_H

// headers in ROS
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

#include <fstream>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <utility> // std::pair

#include <pugixml.hpp>


// //User defined messages
#include "nif_dk_graph_planner_msgs/msg/osm_parcer.hpp"
#include "nif_dk_graph_planner_msgs/msg/way.hpp"
#include "nif_dk_graph_planner_msgs/msg/node.hpp"
#include "nif_dk_graph_planner_msgs/msg/final_path_id_array.hpp"

#include "utils/geodetic_conv.h"


// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef struct Connected
{
  int id;
  double cost;
} connected_t;

typedef struct Waymap {
  int start_layer;
  int end_layer;
  int start_node;
  int end_node;
} waymap_t;

namespace nif{
namespace planning{

class DKGraphPlannerNode : public rclcpp::Node {
public:
  DKGraphPlannerNode(const std::string &node_name_);
  ~DKGraphPlannerNode();

  void OsmParcing();
  void RacingLineParcing();

  void MessagePublisher();
  void Planning();
  std::vector<int> PlanningWithCSV(const int start, const int goal);
  void ToPathMsg();
  int HeuristicFunc(int _next_id, Connected connected);
  // void aStarPlanning();
  bool ExistInList(int id, std::vector<int> list);
  void timer_callback();
  void run();

  // void CallbackInitPoseRviz(const geometry_msgs::PoseWithCovarianceStamped& msg);  
  // void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);
  void CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  // void timerCallback(const ros::TimerEvent& event);
  // void CallbackGoalXYList(const sensor_msgs::msg::PointCloud2ConstPtr& msg);
  

private:
  DKGraphPlannerNode();

  void SearchGraph(const double &x_in, const double &y_in, int &start_layer_out,
                   int &end_layer_out, int &start_node_out, int &end_node_out);
  std::vector<std::pair<std::string, std::vector<double>>> read_csv(std::string filename);
  
  rclcpp::TimerBase::SharedPtr sub_timer_;
  rclcpp::Publisher<nif_dk_graph_planner_msgs::msg::OsmParcer>::SharedPtr
      pubOsmParcer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullNodePoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRacingLinePoints;
        // pubCandidatesNodePoints, pubFinalPathPoints; ros::Publisher
        // pubFullLink, pubFinalLink; ros::Publisher pubFinalNodes,
        // pubFinalITSIds; ros::Publisher pubWptNodeCloud;

            // ros::Subscriber SubInitPose;
            // ros::Subscriber SubGoal;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubOdometry;
    // ros::Subscriber SubGoalXYList;

    std::string m_OsmFileName, m_FullFilePath;
    std::string m_RacingTrajectory;

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_racingLine_points;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_FullIndexedPoints;

    nif_dk_graph_planner_msgs::msg::OsmParcer m_OsmParcer;

    // autoware_msgs::LaneArray m_LaneArray;
    // autoware_msgs::LaneArray m_FinalLaneArray;

    // geometry_msgs::Pose m_InitPose;
    // nav_msgs::Odometry m_Odometry;
    // geometry_msgs::Pose m_GoalPose;
    // geometry_msgs::Pose m_PrevGoalPose;
    // std::vector<geometry_msgs::Pose> m_GoalXYList;

    bool bParcingComplete = false;
    bool bRacingLine = false;
    // bool bInitPose;
    // bool bGoalPose;
    // bool bBothDirection, bCCW;
    // bool bReplanRoute;
    // bool bCSVGoalList;

    // int m_closestWayId_start;
    // int m_closestWayId_goal;
    // int m_closestStartNode;
    // int m_closestGoalNode;
    std::unordered_map<int, std::vector<nif_dk_graph_planner_msgs::msg::Way>> m_WaysResister;
    // // std::unordered_map<int, nif_dk_graph_planner_msgs::msg::Way>
    // m_NextWay;
    // // std::unordered_map<int, nif_dk_graph_planner_msgs::msg::Way>
    // m_PrevWay;

    nif::localization::utils::GeodeticConverter conv_;
    double m_originLat;
    double m_originLon;
    // double m_YawBias;

    // std::unordered_map<int, std::vector<Connected>> m_graph;
    // std::unordered_map<int, int> m_PredSuccMap;
    // std::deque<int> m_PlanningPathNodes, m_PlanningPathNodesForCSV;
    // std::vector<int> m_FinalNodes;
    // std::vector<int> m_GoalXY_to_way_id_List;

    std::mutex mtx;
};
} // namespace planning
} // namespace nif

#endif  // DK_GRAPH_PLANNER_H

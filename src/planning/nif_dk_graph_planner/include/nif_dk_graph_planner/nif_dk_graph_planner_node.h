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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
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
  double additional_cost;

} connected_t;

typedef struct Obstacle {
  double x;
  double y;
  double cost;
} obstacle_t;

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
  void CallbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  
  // void timerCallback(const ros::TimerEvent& event);
  // void CallbackGoalXYList(const sensor_msgs::msg::PointCloud2ConstPtr& msg);
  

private:
  DKGraphPlannerNode();

  void BuildGraph();
  void UpdateGraph();
  void ReleaseGraph();

  void GetIntensityInfo(const double &x_in, const double &y_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr in_points,
                        int &intensity_out);
  double getClosestDistance(const double &x_in, const double &y_in,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr points_in);
  double CorrespondingCost(const double pt_x, const double pt_y,
                            const nav_msgs::msg::OccupancyGrid &gridmap);
  void getNearbyNodesFromLayer(const double &x_in, const double &y_in,
                      const int &search_num_idx_in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                      int &current_layer_out,
                      std::vector<int> &nearby_indice_out);

  std::vector<std::pair<std::string, std::vector<double>>> read_csv(
      std::string filename);
  void removeDuplicated(std::vector<int> &v);

  rclcpp::TimerBase::SharedPtr sub_timer_;
  rclcpp::Publisher<nif_dk_graph_planner_msgs::msg::OsmParcer>::SharedPtr
      pubOsmParcer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullNodePoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubRacingLineRefPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubFirstNodeContainPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubCandidatesNodePoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubFinalPathPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCostPoints;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubOdometry;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      SubOccupancyGrid;

  std::string m_OsmFileName, m_FullFilePath;
  std::string m_RacingTrajectory;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_racingLineRefPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FullIndexedPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FullNodeIDPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FirstNodeContainPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_CostPoints;

  nif_dk_graph_planner_msgs::msg::OsmParcer m_OsmParcer;

  bool bParcingComplete = false;
  bool bRacingLine = false;
  bool bBuildGraph = false;
  bool bOccupancyGrid = false;
  bool bOdometry = false;

  double m_veh_x;
  double m_veh_y;
  double m_veh_roll, m_veh_pitch, m_veh_yaw;

  int m_closestStartNode;
  int m_closestGoalNode;
  int m_currentLayer;
  std::unordered_map<int, std::vector<nif_dk_graph_planner_msgs::msg::Way>>
      m_WaysResister;
  std::unordered_map<int, std::vector<nif_dk_graph_planner_msgs::msg::Way>>
      m_FirstNodeBasedResister;

  std::vector<std::pair<int, std::pair<int, int>>>
      m_RacingLineGraphArray; // pair layer, node

  std::vector<Obstacle> m_obstacle;
  std::vector<int> m_nearbyFirstNodes;
  nav_msgs::msg::OccupancyGrid m_OccupancyGrid;

  nif::localization::utils::GeodeticConverter conv_;
  double m_originLat;
  double m_originLon;
  // double m_YawBias;

  std::unordered_map<int, std::vector<Connected>> m_graph;
  std::unordered_map<int, int> m_PredSuccMap;
  std::deque<int> m_PlanningPathNodes;
  std::vector<int> m_FinalNodes;

  std::mutex mtx;
  };
} // namespace planning
} // namespace nif

#endif  // DK_GRAPH_PLANNER_H

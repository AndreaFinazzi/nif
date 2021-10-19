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
  double collision_cost;
  int best_node;
  double transient_cost;
  double curvature_cost;
  double cost_close_to_vehicle;

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

struct AnalyticalFunctions {
  std::function<double(double, double)> f_;
};

struct duplication_comparator {
  bool operator()(const std::pair<int, int> &a,
                  const std::pair<int, int> &b) const {
    return a.first != b.first;
  }
};

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
  bool ExistInList(int id, std::deque<int> list);
  void timer_callback();
  void run();

  // void CallbackInitPoseRviz(const geometry_msgs::PoseWithCovarianceStamped& msg);  
  // void CallbackGoalRviz(const geometry_msgs::PoseStamped& msg);
  void CallbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CallbackOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void CallbackCostPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void CallbackClusterCenterPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void CallbackWallPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
private:
  DKGraphPlannerNode();

  void BuildGraph();
  void UpdateGraph();
  void ReleaseGraph();
  void FinalizePath(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &path_points_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &path_points_on_body_in,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &obs_center_in,
      const double &speed_mps_in, const double &dt_in, double &odometry_in,
      const double &obs_radius_in, const double &desired_update_dist_in,
      double &dist_to_obs_out,
      pcl::PointCloud<pcl::PointXYZI>::Ptr &path_points_out,
      bool &path_updated_out);
  void GetIntensityInfo(const double &x_in, const double &y_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr in_points,
                        int &intensity_out);
  void GetIntensityInfo(const double &x_in, const double &y_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr in_points,
                        double &intensity_out);
  void GetIntensityInfo(const double &x_in, const double &y_in,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr in_points, 
                        double &nearest_x_in_points_out, double &nearest_y_in_points_out,
                        int &intensity_out);
  double getClosestDistance(const double &x_in, const double &y_in,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr points_in);
  double CorrespondingCost(const double pt_x, const double pt_y,
                            const nav_msgs::msg::OccupancyGrid &gridmap);
  double
  CorrespondingCost(const double pt_x, const double pt_y, double inflation,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &in_centered_points,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &in_inflated_points);

  void getNearbyNodesFromLayer(const double &x_in, const double &y_in,
                      const int &search_num_idx_in,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr points_in,
                      int &current_layer_out,
                      std::vector<std::pair<int, int>> &nearby_indice_out);

  void TransformPointsToBody(const pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOut, 
                             const double &veh_x_, const double &veh_y_, const double &veh_yaw_);

  std::vector<std::pair<std::string, std::vector<double>>> read_csv(
          std::string filename);
  void removeDuplicated(std::vector<std::pair<int, int>> &v);
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  void createGaussianWorld(
      pcl::PointCloud<pcl::PointXYZI>::Ptr &points_in, double inflation_x,
      double inflation_y, pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out);
  
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubFinalPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFinalPathOnBody;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubWallInflatedPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBestLayerXYPoints;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr SubOdometry;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      SubOccupancyGrid;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubCostPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubClusterCenterPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr SubWallPoints;

  std::string m_OsmFileName, m_FullFilePath;
  std::string m_RacingTrajectory;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_racingLineRefPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FullIndexedPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FullNodeIDPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FirstNodeContainPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_CostPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_InflatedCostPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_ClusterCenterPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_WallPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_WallInflatedCostPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FinalPoints;

  nif_dk_graph_planner_msgs::msg::OsmParcer m_OsmParcer;

  bool bParcingComplete = false;
  bool bRacingLine = false;
  bool bBuildGraph = false;
  bool bOccupancyGrid = false;
  bool bOdometry = false;
  bool bInflatedPoints = false;
  bool bCenteredPoints = false;
  bool bWallPoints = false;
  bool bWallInflated = false;
  bool bDebug;

  double m_dt;

  double m_veh_x;
  double m_veh_y;
  double m_veh_roll, m_veh_pitch, m_veh_yaw;
  double m_veh_speed; 
  double prev_time, current_time;

  double m_ref_gain;
  double m_collision_gain;
  double m_curvature_gain;
  double m_transient_gain;

  double m_inflation_size;
  double m_collision_radius;
  double m_final_path_update_dist;

  double m_odom_dist = 9999.;

  int m_ClosestFirstNodeId;
  int m_StartNode; 
  int m_closestGoalNode;
  int m_currentLayer;
  int m_prevBestFirstNodeInLayer = -1;
  int m_prevBestEndNodeInLayer = -1;
  int m_prevFirstLayer;
  int m_LayerSize;
  int m_prevStartFirstNodeId = -1;
  int m_UsePrevStartFirstNodeAfter2 = 0;

  std::unordered_map<int, int> m_BestLayerArray;  //first node id , layer's way
  std::unordered_map<int, std::pair<double, double>> m_BestLayerXYArray;
  std::unordered_map<int, std::vector<nif_dk_graph_planner_msgs::msg::Way>>
          m_WaysResister;
  std::unordered_map<int, std::vector<nif_dk_graph_planner_msgs::msg::Way>>
      m_FirstNodeBasedResister;

  std::vector<std::pair<int, std::pair<int, int>>>
      m_RacingLineGraphArray; // pair layer, node

  std::vector<Obstacle> m_obstacle;
  std::vector<std::pair<int, int>> m_nearbyFirstNodes;
  nav_msgs::msg::OccupancyGrid m_OccupancyGrid;


  nif::localization::utils::GeodeticConverter conv_;
  double m_originLat;
  double m_originLon;
  // double m_YawBias;

  std::unordered_map<int, std::vector<Connected>> m_graph;
  std::unordered_map<int, int> m_PredSuccMap;
  std::deque<int> m_PlanningPathNodes;
  std::deque<int> m_FinalNodes;

  std::mutex mtx;
  };
} // namespace planning
} // namespace nif

#endif  // DK_GRAPH_PLANNER_H

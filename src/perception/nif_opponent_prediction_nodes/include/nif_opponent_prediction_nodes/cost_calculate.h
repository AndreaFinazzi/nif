#ifndef COST_CALCULATOR_H
#define COST_CALCULATOR_H

// headers in ROS
#include <ros/ros.h>
// #include <tf/transform_listener.h>
// #include <tf/tf.h>
// #include <tf/transform_datatypes.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <std_msgs/Int32.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>

// // headers in local directory
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_ros/GridMapRosConverter.hpp>
// #include <grid_map_msgs/GridMap.h>

// // headers in STL
// #include <memory>
// #include <cmath>

// class CostCalculator
// {
// public:
//   CostCalculator();
//   ~CostCalculator();

//   void init();
//   void run();
//   void CallbackOccupancyGrid(const nav_msgs::OccupancyGridConstPtr& msg);
//   void CallbackMotionPrimitives(const visualization_msgs::MarkerArrayConstPtr& msg);

//   void CallbackPath(const nav_msgs::PathConstPtr& msg);
//   void CallbackLaneArray(const autoware_msgs::LaneArrayConstPtr& msg);
//   void CallbackOdometry(const nav_msgs::OdometryConstPtr& msg);
//   void CallbackClosestIndex(const std_msgs::Int32ConstPtr& msg);
//   void CallbackBaseWaypoints(const autoware_msgs::LaneConstPtr & msg);
//   void CalculateCost(const nav_msgs::OccupancyGrid& Grid, const std::vector<std::vector<PlannerHNS::WayPoint> >& RollOut);

// private:

//   ros::NodeHandle nh_;
//   ros::NodeHandle private_nh_;

//   ros::Publisher pub_obstacles_;
//   ros::Publisher pub_occupied_grid_;
//   ros::Publisher pub_TrajectoryCost_;
//   ros::Publisher pub_LocalWeightedTrajectoriesRviz_;
//   ros::Publisher pub_LocalWeightedTrajectories_;

//   ros::Subscriber subMotionPrimitives;
//   ros::Subscriber subOccupancyGridMap;
//   ros::Subscriber sub_path_;
//   ros::Subscriber sub_lane_array_;
//   ros::Subscriber sub_odom_;
//   ros::Subscriber sub_closest_index_;
//   ros::Subscriber sub_base_waypoints_;

//   vector_map::VectorMap m_vmap;
//   grid_map::GridMap m_costmap;
//   tf::TransformListener m_tf_listener;
//   geometry_msgs::PoseStamped m_current_pose_local;
//   visualization_msgs::MarkerArray m_obstacle_markers;
//   visualization_msgs::MarkerArray m_occupied_grid_markers;

//   std::vector<std::vector<PlannerHNS::WayPoint>> m_ConvertedLocalPaths;
//   std::vector<geometry_msgs::Pose> m_Obstacles;

//   std::shared_ptr<autoware_msgs::LaneArray> m_LaneArray_ptr;
//   std::shared_ptr<nav_msgs::OccupancyGrid> m_OccupancyGrid_ptr;
//   std::shared_ptr<nav_msgs::Odometry> m_Odometry_ptr;
//   std::shared_ptr<autoware_msgs::Lane> m_BaseWaypoints_ptr;

//   std::string m_base_frame_id;

//   bool bOccupancyGrid;
//   bool bPathCandidates;
//   bool bOdometry;
//   bool bClosestIndex;
//   bool bBaseWaypoints;

//   double* m_RollOutCost_ptr;
//   double* m_CurvatureCost_ptr;
//   double* m_TrasitionCost_ptr;
//   double* m_ConsistancyCost_ptr;
//   double* m_CollisionCost_ptr;
//   bool* m_PathBlocked_ptr;

//   //Cost Calculation
//   double m_IntegCurvatureCost;
//   double m_IntegTrasitionCost;
//   double m_WeightCurvature;
//   double m_WeightTrasition;
//   double m_WeightConsistancy;
//   double m_WeightCollision;
//   double m_ClosestIndex;
//   double m_obstacle_radius;
//   double m_collision_cost; 
//   double m_curvature_cost;
//   double m_trasition_cost;
//   double m_rollOutDensity;
//   double m_rollOutNumber;
//   double m_PathBlockedThres;

//   //Best Path Selection
//   int m_current_rollout_index;
//   int m_best_rollout_index;
//   double m_minimum_distance_to_obstacle;
//   double m_minimum_cost;
//   bool m_BestPathBlocked;
//   bool m_AllPathBlocked;

//   void initRollOutCost(const autoware_msgs::LaneArray &LaneArray);
//   /* Path candidates translation from Global to Local coordinates */
//   void GlobalCoordToLocalCoord(const autoware_msgs::LaneArray &LaneArray, const nav_msgs::Odometry &Odometry);
//   void CalculateTrasitionCost(double * CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut ,const int& currTrajectoryIndex);
//   void CalculateCurvatureCost(double* CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut);
//   void CalculateConsistancyCost(double* CostArray, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut, const int& MinimumCostIndex);
//   void NormalizeCost(double* CollisionCost, double* CurvatureCost, double* TrasitionCost, double* ConsistancyCost, const std::vector<std::vector<PlannerHNS::WayPoint>>& RollOut);
//   void BestPath(const autoware_msgs::LaneArray &GlobalPathCandidates, double* CostArray, bool* BlockedArray, std::vector<std::vector<PlannerHNS::WayPoint>> LocalPathCandidates);

//   /* Utility */  
//   tf::Transform getTransform(const std::string& from, const std::string& to);
//   double getLookAheadDistance(const nav_msgs::Odometry& CurrentPose, const std::vector<PlannerHNS::WayPoint>& SinglePath, const int& look_ahead_index);
//   double getLookAheadAngleconst(const nav_msgs::Odometry& CurrentPose,const std::vector<PlannerHNS::WayPoint>& SinglePath, const int& look_ahead_index);
//   void TrajectoriesToColoredMarkers(
//     const std::vector<std::vector<PlannerHNS::WayPoint> >& paths, double* CostArray, bool* BlockedArray,
//     visualization_msgs::MarkerArray& markerArray);

//   inline geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, const tf::Transform& tf)
//   {
//     // Convert ROS pose to TF pose
//     tf::Pose tf_pose;
//     tf::poseMsgToTF(pose, tf_pose);

//     // Transform pose
//     tf_pose = tf * tf_pose;

//     // Convert TF pose to ROS pose
//     geometry_msgs::Pose ros_pose;
//     tf::poseTFToMsg(tf_pose, ros_pose);

//     return ros_pose;
//   }
// };

#endif  // COST_CALCULATOR_H

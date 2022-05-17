/*
 * costmap_generator.h
 *
 *  Created on: Aug 9, 2021
 *      Author: Daegyu Lee
 */

#ifndef COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_H

// headers in ROS

// ROS
#include "rclcpp/clock.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_frame_id/frame_id.h"
#include "nif_localization_minimal/localization_minimal.h"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// PCL library
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// headers in STL
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

// // headers in Autoware
// #include <costmap_generator/bboxes_to_costmap.h>
// #include <costmap_generator/lane_to_costmap.h>
// #include <costmap_generator/laserscan_to_costmap.h>
#include <costmap_generator/points_to_costmap.h>
// #include <costmap_generator/visual_to_costmap.h>

// #include <pcl_ros/point_cloud.h>

// #include <pcl/segmentation/conditional_euclidean_clustering.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>

// struct by_value {
//   bool operator()(geometry_msgs::Pose const &left,
//                   geometry_msgs::Pose const &right) {
//     return left.position.x < right.position.x;
//   }
// };

namespace nif
{
  namespace perception
  {
    namespace costmap
    {

      class CostmapGeneratorV2 : public rclcpp::Node
      {
      public:
        CostmapGeneratorV2();
        ~CostmapGeneratorV2();

        void init();
        void run();
        void param();

      private:
        bool use_points_;
        bool use_wayarea_;
        bool use_laserscan_;
        bool use_dynamic_trajectories_;

        void timer_callback();
        void wallPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void objectPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void groundFilteredCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void fakeObstacleCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void imitationOutputCallback(const nav_msgs::msg::Path::SharedPtr msg);
        void imitationOutputCallbackMarkerArray(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

        void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void MessegefilteringCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inner_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &outer_msg);

        void ClosestGeofenceIndexCallback(const std_msgs::msg::Int32::SharedPtr msg);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);
        void
        TransformPointsToGlobal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut,
                                const double &veh_x_, const double &veh_y_,
                                const double &veh_yaw_);

        void
        TransformPointsToBody(const pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn,
                              pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOut,
                              const double &veh_x_, const double &veh_y_,
                              const double &veh_yaw_);

        void SearchPointsOntrack(
            const std::vector<std::pair<double, double>> &inner_array_in,
            const std::vector<std::pair<double, double>> &outer_array_in,
            const int &closest_idx, pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
            pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut);

        double grid_min_value_;
        double grid_max_value_;
        double grid_resolution_;
        double grid_length_x_;
        double grid_length_y_;
        double grid_position_x_;
        double grid_position_y_;

        double maximum_lidar_height_thres_;
        double minimum_lidar_height_thres_;
        double maximum_laserscan_distance_thres_;
        double minimum_laserscan_distance_thres_;

        double m_veh_x;
        double m_veh_y;
        double m_veh_roll, m_veh_pitch, m_veh_yaw;

        int m_closestGeofenceIndex;
        bool bEnablePotential_;
        double potential_size_;

        std::vector<std::pair<double, double>>
            m_OuterGeoFence;
        std::vector<std::pair<double, double>> m_InnerGeoFence;

        bool bGeoFence = false;
        bool bOdometry = false;
        bool bWallPoints = false;
        bool bObjectPoints = false;
        bool bGroundFilteredPoints = false;
        bool bFakeObstaclePoints = false;

        pcl::PointCloud<pcl::PointXYZI>::Ptr m_in_wall_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_in_object_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_in_fake_obstacle_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_in_ground_filtered_points;

        pcl::PointCloud<pcl::PointXYZI>::Ptr m_inner_geofence_points;
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_outer_geofence_points;

        pcl::PointCloud<pcl::PointXYZI>::Ptr m_imitation_samples_pts;

        //   double expand_polygon_size_;
        //   int size_of_expansion_kernel_;

        rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_occupancy_grid_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_on_global_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_on_track_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_wall_points_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_object_points_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fake_obs_points_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_filtered_points_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_imitation_samples_;
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_imitation_samples_marker_array_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_closest_geofence_idx_;
        rclcpp::TimerBase::SharedPtr sub_timer_;

        using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_InnerGeofence;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_OuterGeofence;

        std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> m_sync;

        //   std::vector<std::vector<geometry_msgs::Point>> area_points_;
        std::vector<std::pair<double, double>> obstacleArray;
        std::vector<std::pair<double, double>> imitation_samples_pt_vec;

        PointsToCostmap points2costmap_;
        //   LaneToCostmap lane2costmap_;
        //   LaserScanToCostmap laserscan2costmap_;
        //   BBoxesToCostmap bboxes2costmap_;
        //   VisualToCostmap visual2costmap_;
        std_msgs::msg::Header m_in_header;
        //   nav_msgs::Odometry m_odom;
        //   nav_msgs::Path m_LocalPathOnBody;
        //   geometry_msgs::PoseStamped pose_prev;

        grid_map::GridMap costmap_;

        //   pcl::PointCloud<pcl::PointXYZI>::Ptr m_points;

        const std::string SENSOR_POINTS_COSTMAP_LAYER_;
        //   const std::string LASER_2D_COSTMAP_LAYER_;
        const std::string INFLATION_COSTMAP_LAYER_;
        const std::string COMBINED_COSTMAP_LAYER_;

        const std::string IMITATION_DISTRIBUTION_LAYER_ = "imitation_distribution_gaussian";

        grid_map::GridMap initGridmap();
        void publishRosMsg(grid_map::GridMap *map);
        //   void publishRoadBoundaryMsg(grid_map::GridMap *map);

        grid_map::Matrix generateSensorPointsCostmap(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_points);

        void generateCombinedCostmap();
        //   void publishBoundaryPathMsg(nav_msgs::OccupancyGrid &gridmap);

        void MakeInflationWithPoints();
        grid_map::Matrix createGaussianWorld(grid_map::GridMap *map, const std::string layer_name,
                                             double inflation_x, double inflation_y, const std::vector<std::pair<double, double>> &pointArray);
        grid_map::Matrix createGaussianWorldV2(grid_map::GridMap *map, const std::string layer_name,
                                               std::vector<double> inflation_x, std::vector<double> inflation_y,
                                               const std::vector<std::pair<double, double>> &pointArray);
        grid_map::Matrix fillGridMap(grid_map::GridMap *map, const std::string layer_name,
                                     const AnalyticalFunctions &functions);

        //   std::string m_ndtSearchMethod;
        //   pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
      };

    } // namespace costmap
  }   // namespace perception
} // namespace nif

#endif // COSTMAP_GENERATOR_H


#ifndef LASERSCAN_TO_COSTMAP_H
#define LASERSCAN_TO_COSTMAP_H

// headers in ROS
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

// headers in PCL
#include <pcl_ros/point_cloud.h>

class LaserScanToCostmap
{
public:
  LaserScanToCostmap();
  ~LaserScanToCostmap();

  /// \brief calculate cost from laserscan
  /// \param[in] maximum_laser_distance_thres: Maximum distance threshold from robot
  /// \param[in] minimum_laser_distance_thres: Minimum distance threshold from robot
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[in] in_sensor_points: subscribed pointcloud
  /// \param[out] calculated cost in grid_map::Matrix format
  grid_map::Matrix makeCostmapFromLaserScan(const double maximum_laser_distance_thres, const double minimum_laser_distance_thres,
                                            const double grid_min_value, const double grid_max_value,
                                            const grid_map::GridMap& gridmap, const std::string& gridmap_layer_name,
                                            const sensor_msgs::LaserScanConstPtr &in_laser_scan);

private:
  friend class TestClass;

  double grid_length_x_;
  double grid_length_y_;
  double grid_resolution_;
  double grid_position_x_;
  double grid_position_y_;
  double y_cell_size_;
  double x_cell_size_;
  std::vector<geometry_msgs::Vector3> mapped_indices;

  /// \brief initialize gridmap parameters
  /// \param[in] gridmap: gridmap object to be initialized
  void initGridmapParam(const grid_map::GridMap& gridmap);

  /// \brief check if index is valid in the gridmap
  /// \param[in] grid_ind: grid index corresponding with one of pointcloud
  /// \param[out] bool: true if index is valid
  bool isValidInd(const grid_map::Index& grid_ind);

  // /// \brief Get index from one of pointcloud
  // /// \param[in] point: one of subscribed pointcloud
  // /// \param[out] index in gridmap
  // grid_map::Index fetchGridIndexFromLaserScan(const sensor_msgs::LaserScanConstPtr& in_laser_scan_);

  /// \brief Assign pointcloud to appropriate cell in gridmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] in_laser_scan_: subscribed sensor_msgs::LaserScan
  /// \param[out] grid-x-length x grid-y-length size grid 
  void assignPoints2GridCell(const grid_map::GridMap& gridmap, const sensor_msgs::LaserScanConstPtr &in_laser_scan_,
                             const double maximum_laser_distance_thres, const double minimum_laser_distance_thres);

  /// \brief calculate costmap from subscribed laserscan
  /// \param[in] maximum_laser_distance_thres: Maximum distance threshold from robot
  /// \param[in] minimum_laser_distance_thres: Minimum distance threshold from robot
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[out] caculated costmap in grid_map::Matrix format
  grid_map::Matrix calculateCostmap(const double maximum_laser_distance_thres, const double minimum_laser_distance_thres,
                                    const double grid_min_value, const double grid_max_value,
                                    const grid_map::GridMap& gridmap, const std::string& gridmap_layer_name);
};

#endif  // LASERSCAN_TO_COSTMAP_H

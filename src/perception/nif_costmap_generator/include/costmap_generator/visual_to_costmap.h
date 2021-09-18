#ifndef VISUAL_TO_COSTMAP_H
#define VISUAL_TO_COSTMAP_H

// headers in ROS
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

// headers in local directory
#include "detection_msgs/BoundingBoxArray.h"
#include "detection_msgs/BoundingBox.h"


class VisualToCostmap
{
public:
  VisualToCostmap();
  ~VisualToCostmap();

  /// \brief calculate cost from DetectedObjectArray
  /// \param[in] costmap: initialized gridmap
  /// \param[in] expand_polygon_size: expand object's costmap polygon
  /// \param[in] size_of_expansion_kernel: kernel size for blurring cost
  /// \param[in] in_objects: subscribed DetectedObjectArray
  /// \param[out] calculated cost in grid_map::Matrix format
  grid_map::Matrix makeCostmapFromBBoxes(const grid_map::GridMap& costmap,
                                          const double expand_polygon_size, const double size_of_expansion_kernel,
                                          const detection_msgs::BoundingBoxArray::ConstPtr& in_bboxes);

private:

  const int NUMBER_OF_POINTS;
  const int NUMBER_OF_DIMENSIONS;
  const std::string VISUAL_COSTMAP_LAYER_;

  /// \brief make 4 rectangle points from centroid position and orientation
  /// \param[in] in_object: subscribed one of DetectedObjectArray
  /// \param[in] expand_rectangle_size: expanding 4 points
  /// \param[out] 4 rectangle points
  Eigen::MatrixXd makeRectanglePoints(const detection_msgs::BoundingBox& in_object,
                                      const double expand_rectangle_size);

  /// \brief make polygon(grid_map::Polygon) from 4 rectangle's points
  /// \param[in] in_object: subscribed one of DetectedObjectArray
  /// \param[in] expand_rectangle_size: expanding 4 points
  /// \param[out] polygon with 4 rectangle points
  grid_map::Polygon makePolygonFromBBox(const detection_msgs::BoundingBox& in_object,
                                        const double expand_rectangle_size);

  /// \brief make expanded point from convex hull's point
  /// \param[in] in_centroid: object's centroid
  /// \param[in] in_corner_point one of convex hull points
  /// \param[in] expand_polygon_size  the param for expanding convex_hull points
  /// \param[out] expanded point
  geometry_msgs::Point makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                         const geometry_msgs::Point32& in_corner_point,
                                         const double expand_polygon_size);

  /// \brief set cost in polygon by using DetectedObject's score
  /// \param[in] polygon: 4 rectangle points in polygon format
  /// \param[in] gridmap_layer_name: target gridmap layer name for calculated cost
  /// \param[in] score: set score as a cost for costmap
  /// \param[in] objects_costmap: update cost in this objects_costmap[gridmap_layer_name]
  void setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name, const float score,
                        grid_map::GridMap& objects_costmap);
};

#endif  // VISUAL_TO_COSTMAP_H

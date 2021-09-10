// headers in standard library
#include <cmath>

// headers in ROS
#include <tf/transform_datatypes.h>

// headers in local directory
#include "costmap_generator/bboxes_to_costmap.h"

// Constructor
BBoxesToCostmap::BBoxesToCostmap() :
NUMBER_OF_POINTS(4),
NUMBER_OF_DIMENSIONS(2),
BOUNDING_BOX_COSTMAP_LAYER_("bounding_boxes")
{
}

BBoxesToCostmap::~BBoxesToCostmap()
{
}

Eigen::MatrixXd BBoxesToCostmap::makeRectanglePoints(const jsk_recognition_msgs::BoundingBox& in_bboxes,
                                                     const double expand_rectangle_size)
{
  double length = in_bboxes.dimensions.x + expand_rectangle_size;
  double width = in_bboxes.dimensions.y + expand_rectangle_size;
  Eigen::MatrixXd origin_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  origin_points << length / 2, length / 2, -length / 2, -length / 2, width / 2, -width / 2, -width / 2, width / 2;

  double yaw = tf::getYaw(in_bboxes.pose.orientation);
  Eigen::MatrixXd rotation_matrix(NUMBER_OF_DIMENSIONS, NUMBER_OF_DIMENSIONS);
  rotation_matrix << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  Eigen::MatrixXd rotated_points = rotation_matrix * origin_points;

  double dx = in_bboxes.pose.position.x;
  double dy = in_bboxes.pose.position.y;
  Eigen::MatrixXd transformed_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(1, NUMBER_OF_POINTS);
  transformed_points.row(0) = rotated_points.row(0) + dx * ones;
  transformed_points.row(1) = rotated_points.row(1) + dy * ones;
  return transformed_points;
}

grid_map::Polygon BBoxesToCostmap::makePolygonFromBBox(const jsk_recognition_msgs::BoundingBox& in_bboxes,
                                                         const double expand_rectangle_size)
{
  grid_map::Polygon polygon;
  polygon.setFrameId(in_bboxes.header.frame_id);
  Eigen::MatrixXd rectangle_points = makeRectanglePoints(in_bboxes, expand_rectangle_size);
  for (int col = 0; col < rectangle_points.cols(); col++)
  {
    polygon.addVertex(grid_map::Position(rectangle_points(0, col), rectangle_points(1, col)));
  }
  return polygon;
}

geometry_msgs::Point BBoxesToCostmap::makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                                        const geometry_msgs::Point32& in_corner_point,
                                                        const double expand_polygon_size)
{
  geometry_msgs::Point expanded_point;
  if(expand_polygon_size == 0)
  {
    expanded_point.x = in_corner_point.x;
    expanded_point.y = in_corner_point.y;
    return expanded_point;
  }
  double theta = std::atan2(in_corner_point.y - in_centroid.y, in_corner_point.x - in_centroid.x);
  double delta_x = expand_polygon_size * std::cos(theta);
  double delta_y = expand_polygon_size * std::sin(theta);
  expanded_point.x = in_corner_point.x + delta_x;
  expanded_point.y = in_corner_point.y + delta_y;
  return expanded_point;
}

void BBoxesToCostmap::setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name,
                                       const float score, grid_map::GridMap& objects_costmap)
{
  grid_map::PolygonIterator iterators(objects_costmap, polygon);
  for (grid_map::PolygonIterator iterator(objects_costmap, polygon); !iterator.isPastEnd(); ++iterator)
  {
    const float current_score = objects_costmap.at(gridmap_layer_name, *iterator);
    if (score > current_score)
    {
      objects_costmap.at(gridmap_layer_name, *iterator) = score;
    }
  }
}

grid_map::Matrix BBoxesToCostmap::makeCostmapFromBBoxes(const grid_map::GridMap& costmap,
                                                         const double expand_polygon_size,
                                                         const double size_of_expansion_kernel,
                                                         const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_bboxess)
{
  grid_map::GridMap objects_costmap = costmap;
  objects_costmap.add(BOUNDING_BOX_COSTMAP_LAYER_, 0);

  for (const auto& object : in_bboxess->boxes)
  {
    grid_map::Polygon polygon, expanded_polygon;
    expanded_polygon = makePolygonFromBBox(object, expand_polygon_size);
    
    setCostInPolygon(expanded_polygon, BOUNDING_BOX_COSTMAP_LAYER_, 100, objects_costmap);
  }
  return objects_costmap[BOUNDING_BOX_COSTMAP_LAYER_];
}

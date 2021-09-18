
#include "costmap_generator/laserscan_to_costmap.h"
#include <cmath>
#include <limits>

// Constructor
LaserScanToCostmap::LaserScanToCostmap() {}

LaserScanToCostmap::~LaserScanToCostmap() {}

void LaserScanToCostmap::initGridmapParam(const grid_map::GridMap &gridmap) {
  grid_length_x_ = gridmap.getLength().x();
  grid_length_y_ = gridmap.getLength().y();
  grid_resolution_ = gridmap.getResolution();
  grid_position_x_ = gridmap.getPosition().x();
  grid_position_y_ = gridmap.getPosition().y();
  mapped_indices.clear();
}

bool LaserScanToCostmap::isValidInd(const grid_map::Index &grid_ind) {
  bool is_valid = false;
  int x_grid_ind = grid_ind.x();
  int y_grid_ind = grid_ind.y();
  if (x_grid_ind > 0 &&
      x_grid_ind < std::ceil(grid_length_x_ * (1 / grid_resolution_)) &&
      y_grid_ind > 0 &&
      y_grid_ind < std::ceil(grid_length_y_ * (1 / grid_resolution_))) {
    is_valid = true;
  }
  return is_valid;
}

void LaserScanToCostmap::assignPoints2GridCell(const grid_map::GridMap &gridmap,const sensor_msgs::LaserScanConstPtr &in_laser_scan_,
                                               const double maximum_laser_distance_thres, const double minimum_laser_distance_thres) {

  // calculate out_grid_map position
  const double origin_x_offset = grid_length_x_ / 2.0 - grid_position_x_;
  const double origin_y_offset = grid_length_y_ / 2.0 - grid_position_y_;
  // coordinate conversion for making index. Set bottom left to the origin of
  // coordinate (0, 0) in gridmap area

  double y_cell_size = std::ceil(grid_length_y_ * (1 / grid_resolution_));
  double x_cell_size = std::ceil(grid_length_x_ * (1 / grid_resolution_));
   
  //here laser scan to xy.
  size_t scan_size = in_laser_scan_->ranges.size();
  mapped_indices.clear();
  for (int i = 0; i < scan_size; i++)
  {
    double angle, x_buf, y_buf;
    double mapped_x, mapped_y;
    
    if (!std::isinf(in_laser_scan_->ranges.at(i)))
    {
      angle = in_laser_scan_->angle_min + (i * in_laser_scan_->angle_increment);
      x_buf = in_laser_scan_->ranges.at(i) * cos(angle);
      y_buf = in_laser_scan_->ranges.at(i) * sin(angle);  
      double distance = sqrt(pow(x_buf,2)+pow(y_buf,2));
      if (distance > maximum_laser_distance_thres || distance < minimum_laser_distance_thres) {
        continue;
      }
      mapped_x =
          (grid_length_x_ - origin_x_offset - x_buf) / grid_resolution_;
      mapped_y =
          (grid_length_y_ - origin_y_offset - y_buf) / grid_resolution_;
      
      int mapped_x_ind = std::ceil(mapped_x);
      int mapped_y_ind = std::ceil(mapped_y);
      grid_map::Index grid_ind(mapped_x_ind, mapped_y_ind);
      
      if (isValidInd(grid_ind)) {
        geometry_msgs::Vector3 mapped_index_buf;
        mapped_index_buf.x = mapped_x_ind;
        mapped_index_buf.y = mapped_y_ind;
        mapped_indices.push_back(mapped_index_buf);
      }
    }
  }
}

grid_map::Matrix LaserScanToCostmap::calculateCostmap(
                                  const double maximum_laser_distance_thres, const double minimum_laser_distance_thres,
                                  const double grid_min_value, const double grid_max_value,
                                  const grid_map::GridMap& gridmap, const std::string& gridmap_layer_name) {
  grid_map::Matrix gridmap_data = gridmap[gridmap_layer_name];
  gridmap_data.setConstant(0.0);
  for (size_t i = 0; i < mapped_indices.size(); i++) {
    double x_ind = mapped_indices.at(i).x;
    double y_ind = mapped_indices.at(i).y;

    gridmap_data(x_ind, y_ind) = grid_max_value;
  }
  return gridmap_data;
}

grid_map::Matrix LaserScanToCostmap::makeCostmapFromLaserScan(
                                            const double maximum_laser_distance_thres, const double minimum_laser_distance_thres,
                                            const double grid_min_value, const double grid_max_value,
                                            const grid_map::GridMap& gridmap, const std::string& gridmap_layer_name,
                                            const sensor_msgs::LaserScanConstPtr &in_laser_scan_) {
  initGridmapParam(gridmap);
  assignPoints2GridCell(gridmap, in_laser_scan_, maximum_laser_distance_thres, minimum_laser_distance_thres);
  grid_map::Matrix costmap = calculateCostmap(
      maximum_laser_distance_thres, minimum_laser_distance_thres, grid_min_value,
      grid_max_value, gridmap, gridmap_layer_name);
  return costmap;
}
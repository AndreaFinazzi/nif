/**
 * @brief utility functions to load in a path in gps coordinates
 *  and transform to an ltp frame
 **/
#ifndef BVS_LOCALIZATION_UTILS_PATH_LOADING_H_
#define BVS_LOCALIZATION_UTILS_PATH_LOADING_H_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nif_control_joint_lqr_nodes/utils/geodetic_conv.h"

namespace bvs_localization {
namespace utils {

class PathUtil {
public:
  /**
   * @brief initializes a path loader
   **/
  PathUtil();

  /**
   * @brief sets a geodetic converter to convert to LTP
   * @param converter an initialized converter
   *
   * @note pass transform_to_ltp = false to loadPath
   *  if you want points in the GPS frame the csv is in
   **/
  void setConverter(GeodeticConverter &converter);

  /**
   * @brief loads a path from a csv and transforms to ltp frame
   * @param csv_path a path to the CSV file to load
   * @param transform_to_ltp whether or not to convert the path to ltp
   *  after reading it in
   **/
  void loadPath(std::string csv_path, std::string frame_id = "",
                bool transform_to_ltp = true);

  /**
   * @brief wraps a path such that it starts closer to a pose
   * @param pose the point to start the trajectory around
   * @param pose_count the number of points on the path to include
   *  in the resulting path (-1 = whole trajectory)
   * @return a trajectory starting from the closest point to pose
   *  with pose_count points (or the )
   **/
  nav_msgs::msg::Path wrapPathTo(geometry_msgs::msg::Pose pose,
                                 int pose_limit = -1);

  /**
   * @brief gets the raw Path
   * @return the path loaded from the csv
   * @note if transform_to_ltp = true (default) then this will be in
   *  ltp frame otherwise it will be in gps frame
   **/
  nav_msgs::msg::Path &getPath();

  /**
   * @brief performs basic path smoothing algorithm to
   *  make path more continuous.
   **/
  void smoothPath();

private:
  /**
   * @brief finds the closest point on the trajectory to pose
   * @param pose the point to compare against
   * @return the index of the closest point in the trajectory to pose
   **/
  unsigned int getClosestPoint(geometry_msgs::msg::Pose pose);

  /**
   * @brief compute a yaw value for given point in the path
   * @param idx index of the point to compute yaw for
   * @return a target yaw value in radians
   **/
  geometry_msgs::msg::Quaternion
  computePointOrientation(nav_msgs::msg::Path &path, unsigned int idx);

  //! Converter for the path
  GeodeticConverter converter_;
  //! Internal Path
  nav_msgs::msg::Path path_;

}; /* class Path */

} /* namespace utils */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_UTILS_PATH_LOADING_H_ */
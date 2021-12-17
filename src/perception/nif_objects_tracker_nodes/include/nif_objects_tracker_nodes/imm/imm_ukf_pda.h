/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H

#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <vector>

// #include <tf/transform_listener.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nif_msgs/msg/detected_object.hpp"
#include "nif_msgs/msg/detected_object_array.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_msgs/msg/perception3_d_array.hpp"
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <tf2_ros/transform_listener.h>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nif_utils/utils.h"

#include "ukf.h"
#include <yaml-cpp/yaml.h>

using namespace std;

class ImmUkfPda {
private:
  int target_id_;
  bool init_;
  rclcpp::Time timestamp_;

  std::vector<UKF> targets_;

  // probabilistic data association params
  double gating_thres_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_thres_;

  // static classification param
  double static_velocity_thres_;
  int static_num_history_thres_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // whether if benchmarking tracking result
  bool is_benchmark_;
  int frame_count_;
  std::string kitti_data_dir_;

  // for benchmark
  std::string result_file_path_;

  // prevent explode param for ukf
  double prevent_explosion_thres_;

  // for vectormap assisted tarcking
  bool use_vectormap_;
  bool has_subscribed_vectormap_;
  double lane_direction_chi_thres_;
  double nearest_lane_distance_thres_;
  std::string vectormap_frame_;
  // vector_map::VectorMap vmap_;
  // std::vector<vector_map_msgs::Lane> lanes_;

  nif_msgs::msg::DetectedObjectArray previous_loop_det_;
  nif_msgs::msg::DetectedObjectArray tracked_object_;
  bool jsk_first_callback_ = true;

  double merge_distance_threshold_;
  const double CENTROID_DISTANCE =
      0.2; // distance to consider centroids the same

  std::string input_topic_;
  std::string output_topic_;

  std::string tracking_frame_;

  // tf2_ros::TransformListener tf_listener_;
  tf2::Stamped<tf2::Transform> local2global_;
  tf2::Stamped<tf2::Transform> tracking_frame2lane_frame_;
  tf2::Stamped<tf2::Transform> lane_frame2tracking_frame_;

  nav_msgs::msg::Odometry ego_odom_;

  std_msgs::msg::Header input_header_;

  void
  transformPoseToGlobal(const nif_msgs::msg::DetectedObjectArray &input,
                        nif_msgs::msg::DetectedObjectArray &transformed_input);
  void transformPoseToLocal(
      nif_msgs::msg::DetectedObjectArray &detected_objects_output);

  void
  measurementValidation(const nif_msgs::msg::DetectedObjectArray &input,
                        UKF &target, const bool second_init,
                        const Eigen::VectorXd &max_det_z,
                        const Eigen::MatrixXd &max_det_s,
                        std::vector<nif_msgs::msg::DetectedObject> &object_vec,
                        std::vector<bool> &matching_vec);
  nif_msgs::msg::DetectedObject getNearestObject(
      UKF &target,
      const std::vector<nif_msgs::msg::DetectedObject> &object_vec);
  void updateBehaviorState(const UKF &target,
                           nif_msgs::msg::DetectedObject &object);

  void initTracker(const nif_msgs::msg::DetectedObjectArray &input,
                   const rclcpp::Time timestamp);
  void secondInit(UKF &target,
                  const std::vector<nif_msgs::msg::DetectedObject> &object_vec,
                  const rclcpp::Duration dt);

  void updateTrackingNum(
      const std::vector<nif_msgs::msg::DetectedObject> &object_vec,
      UKF &target);

  bool probabilisticDataAssociation(
      const nif_msgs::msg::DetectedObjectArray &input, const rclcpp::Duration dt,
      std::vector<bool> &matching_vec,
      std::vector<nif_msgs::msg::DetectedObject> &object_vec, UKF &target);
  void makeNewTargets(const rclcpp::Time timestamp,
                      const nif_msgs::msg::DetectedObjectArray &input,
                      const std::vector<bool> &matching_vec);

  void staticClassification();

  void makeOutput(const nif_msgs::msg::DetectedObjectArray &input,
                  const std::vector<bool> &matching_vec,
                  nif_msgs::msg::DetectedObjectArray &detected_objects_output);

  void removeUnnecessaryTarget();

  void dumpResultText(nif_msgs::msg::DetectedObjectArray &detected_objects);

  void tracker(const nif_msgs::msg::DetectedObjectArray &transformed_input,
               nif_msgs::msg::DetectedObjectArray &detected_objects_output);

  // bool updateDirection(const double smallest_nis,
  //                      const nif_msgs::msg::DetectedObject &in_object,
  //                      nif_msgs::msg::DetectedObject &out_object, UKF
  //                      &target);

  // bool storeObjectWithNearestLaneDirection(
  //     const nif_msgs::msg::DetectedObject &in_object,
  //     nif_msgs::msg::DetectedObject &out_object);

  // void checkVectormapSubscription();

  nif_msgs::msg::DetectedObjectArray removeRedundantObjects(
      const nif_msgs::msg::DetectedObjectArray &in_detected_objects,
      const std::vector<size_t> in_tracker_indices);

  nif_msgs::msg::DetectedObjectArray
  forwardNonMatchedObject(const nif_msgs::msg::DetectedObjectArray &tmp_objects,
                          const nif_msgs::msg::DetectedObjectArray &input,
                          const std::vector<bool> &matching_vec);

  bool arePointsClose(const geometry_msgs::msg::Point &in_point_a,
                      const geometry_msgs::msg::Point &in_point_b,
                      float in_radius);

  bool arePointsEqual(const geometry_msgs::msg::Point &in_point_a,
                      const geometry_msgs::msg::Point &in_point_b);

  bool isPointInPool(const std::vector<geometry_msgs::msg::Point> &in_pool,
                     const geometry_msgs::msg::Point &in_point);

  void updateTargetWithAssociatedObject(
      const std::vector<nif_msgs::msg::DetectedObject> &object_vec,
      UKF &target);

public:
  ImmUkfPda(const std::string &config_wpt_file_path_);

  void setEgoOdom(const nav_msgs::msg::Odometry &odom_);

  void setDetectionResult(const nif_msgs::msg::DetectedObjectArray &input);

  // nif_msgs::msg::DetectedObjectArray getTrackedResult();
  nif_msgs::msg::Perception3DArray
  getTrackedResult(nif_msgs::msg::DetectedObjectArray &input);
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */

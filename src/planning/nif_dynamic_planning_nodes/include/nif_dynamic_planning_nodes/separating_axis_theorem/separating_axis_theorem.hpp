#ifndef __SEPARATING_AXIS_THEOREM_H__
#define __SEPARATING_AXIS_THEOREM_H__

#include "nif_msgs/msg/dynamic_trajectory.hpp"
#include "nif_utils/utils.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

namespace nif {
namespace planning {
namespace sat {

typedef struct {
  double x;
  double y;
} Vector2;

// Returns the dot products of two vectors
double sat_dot(const Vector2 &A, const Vector2 &B) {
  return A.x * B.x + A.y * B.y;
}

std::vector<Vector2> calculate_bounds(const double x, const double y,
                                      const double yaw,
                                      const double centre_to_front,
                                      const double centre_to_rear,
                                      const double centre_to_side) {

  Vector2 front_right_point;
  front_right_point.x =
      x + centre_to_side * sin(yaw) + centre_to_front * cos(yaw);
  front_right_point.y =
      y - centre_to_side * cos(yaw) + centre_to_front * sin(yaw);

  Vector2 front_left_point;
  front_left_point.x =
      x - centre_to_side * sin(yaw) + centre_to_front * cos(yaw);
  front_left_point.y =
      y + centre_to_side * cos(yaw) + centre_to_front * sin(yaw);

  Vector2 rear_left_point;
  rear_left_point.x = x - centre_to_side * sin(yaw) - centre_to_rear * cos(yaw);
  rear_left_point.y = y + centre_to_side * cos(yaw) - centre_to_rear * sin(yaw);

  Vector2 rear_right_point;
  rear_right_point.x =
      x + centre_to_side * sin(yaw) - centre_to_rear * cos(yaw);
  rear_right_point.y =
      y - centre_to_side * cos(yaw) - centre_to_rear * sin(yaw);

  return {front_right_point, front_left_point, rear_left_point,
          rear_right_point};
}

// Linear transform to find the orthogonal vector of the edge
Vector2 calculate_normalised_projection_axis(const Vector2 &current_point,
                                             const Vector2 &next_point) {
  const double axis_x = -(next_point.y - current_point.y);
  const double axis_y = next_point.x - current_point.x;
  const double magnitude = hypot(axis_x, axis_y);

  Vector2 axis_normalised;
  axis_normalised.x = axis_x / magnitude;
  axis_normalised.y = axis_y / magnitude;

  return axis_normalised;
}

// Project the vertices of each polygon onto a axis
void compute_projections(const std::vector<Vector2> &bounds_a,
                         const std::vector<Vector2> &bounds_b,
                         const Vector2 &axis_normalised,
                         std::vector<double> &projections_a,
                         std::vector<double> &projections_b) {
  projections_a.reserve(bounds_a.size());
  projections_b.reserve(bounds_b.size());

  for (size_t j = 0; j < bounds_a.size(); j++) {
    const double projection_a = sat_dot(axis_normalised, bounds_a[j]);
    const double projection_b = sat_dot(axis_normalised, bounds_b[j]);
    projections_a.push_back(projection_a);
    projections_b.push_back(projection_b);
  }
}

// Check if the projections of two polygons overlap
bool is_overlapping(const std::vector<double> &projections_a,
                    const std::vector<double> &projections_b) {
  const double max_projection_a =
      *std::max_element(projections_a.begin(), projections_a.end());
  const double min_projection_a =
      *std::min_element(projections_a.begin(), projections_a.end());
  const double max_projection_b =
      *std::max_element(projections_b.begin(), projections_b.end());
  const double min_projection_b =
      *std::min_element(projections_b.begin(), projections_b.end());

  // Does not intersect
  if (max_projection_a < min_projection_b ||
      max_projection_b < min_projection_a) {
    return false;
  }

  // Projection overlaps but may not necessarily be intersecting yet
  else {
    return true;
  }
}

// Check if two convex polygons intersect
bool separating_axis_intersect(const std::vector<Vector2> &bounds_a,
                               const std::vector<Vector2> &bounds_b) {
  for (size_t i = 0; i < bounds_a.size(); i++) {
    Vector2 current_point;
    current_point.x = bounds_a[i].x;
    current_point.y = bounds_a[i].y;

    Vector2 next_point;
    next_point.x = bounds_a[(i + 1) % bounds_a.size()].x;
    next_point.y = bounds_a[(i + 1) % bounds_a.size()].y;

    Vector2 axis_normalised =
        calculate_normalised_projection_axis(current_point, next_point);

    std::vector<double> projections_a;
    std::vector<double> projections_b;
    compute_projections(bounds_a, bounds_b, axis_normalised, projections_a,
                        projections_b);

    if (!is_overlapping(projections_a, projections_b)) {
      return false;
    }
  }

  for (size_t i = 0; i < bounds_b.size(); i++) {
    Vector2 current_point;
    current_point.x = bounds_b[i].x;
    current_point.y = bounds_b[i].y;

    Vector2 next_point;
    next_point.x = bounds_b[(i + 1) % bounds_b.size()].x;
    next_point.y = bounds_b[(i + 1) % bounds_b.size()].y;

    Vector2 axis_normalised =
        calculate_normalised_projection_axis(current_point, next_point);

    std::vector<double> projections_a;
    std::vector<double> projections_b;
    compute_projections(bounds_a, bounds_b, axis_normalised, projections_a,
                        projections_b);

    if (!is_overlapping(projections_a, projections_b)) {
      return false;
    }
  }

  // Intersects
  return true;
}

// Check if two trajectories intersect
bool separating_axis_intersect_traj(
    const nif_msgs::msg::DynamicTrajectory &traj_a_,
    const nif_msgs::msg::DynamicTrajectory &traj_b_,
    const double time_diff_tres = 2.0) {

  bool has_collision = true;

  const double centre_to_front = 3.0;
  const double centre_to_rear = 3.0;
  const double centre_to_side = 2.0;

  for (int traj_idx_a = 0; traj_idx_a < traj_a_.trajectory_path.poses.size();
       traj_idx_a++) {
    for (int traj_idx_b = 0; traj_idx_b < traj_b_.trajectory_path.poses.size();
         traj_idx_b++) {

      // To save the computation power
      if (nif::common::utils::geometry::calEuclideanDistance(
              traj_a_.trajectory_path.poses[traj_idx_a].pose,
              traj_b_.trajectory_path.poses[traj_idx_b].pose) > 10) {
        // If two point is far from each other more than XX meter, we don't have
        // to check with SAT
        continue;
      }

      auto bound_a = calculate_bounds(
          traj_a_.trajectory_path.poses[traj_idx_a].pose.position.x,
          traj_a_.trajectory_path.poses[traj_idx_a].pose.position.y,
          nif::common::utils::coordination::quat2yaw(
              traj_a_.trajectory_path.poses[traj_idx_a].pose.orientation),
          centre_to_front, centre_to_rear, centre_to_side);

      auto bound_b = calculate_bounds(
          traj_b_.trajectory_path.poses[traj_idx_b].pose.position.x,
          traj_b_.trajectory_path.poses[traj_idx_b].pose.position.y,
          nif::common::utils::coordination::quat2yaw(
              traj_b_.trajectory_path.poses[traj_idx_b].pose.orientation),
          centre_to_front, centre_to_rear, centre_to_side);

      if (separating_axis_intersect(bound_a, bound_b) == true) {
        // Two polygons intersect
        // check the time difference
        if (abs(traj_a_.trajectory_timestamp_array[traj_idx_a] -
                traj_b_.trajectory_timestamp_array[traj_idx_b]) <
            time_diff_tres) {
          // Case :  two polygons intersect and time is closer. Consider as a
          // collision
          has_collision = true;
          return has_collision;
        } else {
          continue;
        }
      }
    }
  }

  return has_collision;
}

/** Check if two trajectories intersect
 * @param ref_traj_ Reference trajectory : ego_traj (Collsion check based on
 * this trajectory's timestamp array).
 * @param target_traj_ Target trajectory : opponent_traj (Find closest timestamp
 * within this timestamp array)
 * @return (boolean) has_collision
 * **/
bool separating_axis_intersect_traj_v2(
    const nif_msgs::msg::DynamicTrajectory &ref_traj_,
    const nif_msgs::msg::DynamicTrajectory &target_traj_,
    const double max_check_time_ = 6.0, const double time_diff_tres_ = 1.0) {

  bool has_collision = true;

  const double centre_to_front = 3.0;
  const double centre_to_rear = 3.0;
  const double centre_to_side = 1.5;

  for (int ref_traj_idx = 0;
       ref_traj_idx < ref_traj_.trajectory_timestamp_array.size();
       ref_traj_idx++) {
    auto ref_timestamp = ref_traj_.trajectory_timestamp_array[ref_traj_idx];

    if (ref_timestamp > max_check_time_) {
      continue;
    }

    auto closest_idx_on_target_traj = nif::common::utils::closestIndex(
        target_traj_.trajectory_timestamp_array, ref_timestamp);

    if (abs(ref_timestamp -
            target_traj_
                .trajectory_timestamp_array[closest_idx_on_target_traj]) >
        time_diff_tres_) {
      continue;
    }

    auto dist = sqrt(
        pow(ref_traj_.trajectory_path.poses[ref_traj_idx].pose.position.x -
                target_traj_.trajectory_path.poses[closest_idx_on_target_traj]
                    .pose.position.x,
            2) +
        pow(ref_traj_.trajectory_path.poses[ref_traj_idx].pose.position.y -
                target_traj_.trajectory_path.poses[closest_idx_on_target_traj]
                    .pose.position.y,
            2));

    if (dist > 12.0) {
      continue;
    }

    auto bound_a = calculate_bounds(
        ref_traj_.trajectory_path.poses[ref_traj_idx].pose.position.x,
        ref_traj_.trajectory_path.poses[ref_traj_idx].pose.position.y,
        nif::common::utils::coordination::quat2yaw(
            ref_traj_.trajectory_path.poses[ref_traj_idx].pose.orientation),
        centre_to_front, centre_to_rear, centre_to_side);

    auto bound_b = calculate_bounds(
        target_traj_.trajectory_path.poses[closest_idx_on_target_traj]
            .pose.position.x,
        target_traj_.trajectory_path.poses[closest_idx_on_target_traj]
            .pose.position.y,
        nif::common::utils::coordination::quat2yaw(
            target_traj_.trajectory_path.poses[closest_idx_on_target_traj]
                .pose.orientation),
        centre_to_front, centre_to_rear, centre_to_side);

    if (separating_axis_intersect(bound_a, bound_b) == true) {
      has_collision = true;
      return has_collision;
    }
  }

  return has_collision;
}

} // namespace sat

} // namespace planning

} // namespace nif

#endif // __SEPARATING_AXIS_THEOREM_H__

/////////////
// EXAMPLE //
/////////////

// #include "separating_axis.hpp"
// #include <iostream>

// std::vector<Vector2> calculate_bounds(const double x, const double y,
//                                       const double yaw,
//                                       const double centre_to_front,
//                                       const double centre_to_rear,
//                                       const double centre_to_side) {

//   Vector2 front_right_point;
//   front_right_point.x =
//       x + centre_to_side * sin(yaw) + centre_to_front * cos(yaw);
//   front_right_point.y =
//       y - centre_to_side * cos(yaw) + centre_to_front * sin(yaw);

//   Vector2 front_left_point;
//   front_left_point.x =
//       x - centre_to_side * sin(yaw) + centre_to_front * cos(yaw);
//   front_left_point.y =
//       y + centre_to_side * cos(yaw) + centre_to_front * sin(yaw);

//   Vector2 rear_left_point;
//   rear_left_point.x = x - centre_to_side * sin(yaw) - centre_to_rear *
//   cos(yaw); rear_left_point.y = y + centre_to_side * cos(yaw) -
//   centre_to_rear * sin(yaw);

//   Vector2 rear_right_point;
//   rear_right_point.x =
//       x + centre_to_side * sin(yaw) - centre_to_rear * cos(yaw);
//   rear_right_point.y =
//       y - centre_to_side * cos(yaw) - centre_to_rear * sin(yaw);

//   return {front_right_point, front_left_point, rear_left_point,
//           rear_right_point};
// }

// int main() {
//   const double x1 = 0.0;
//   const double y1 = 0.0;
//   const double yaw1 = 0.0;
//   const double centre_to_front1 = 3.0;
//   const double centre_to_rear1 = 3.0;
//   const double centre_to_side1 = 1.0;

//   const double x2 = 5.0;
//   const double y2 = 3.0;
//   const double yaw2 = 0.0;
//   const double centre_to_front2 = 3.0;
//   const double centre_to_rear2 = 3.0;
//   const double centre_to_side2 = 1.0;

//   std::vector<Vector2> A = calculate_bounds(x1, y1, yaw1, centre_to_front1,
//                                             centre_to_rear1,
//                                             centre_to_side1);
//   std::vector<Vector2> B = calculate_bounds(x2, y2, yaw2, centre_to_front2,
//                                             centre_to_rear2,
//                                             centre_to_side2);

//   std::cout << (separating_axis_intersect(A, B) == true
//                     ? "The polygons intersect."
//                     : "The polygons do not intersect.")
//             << std::endl;
// }
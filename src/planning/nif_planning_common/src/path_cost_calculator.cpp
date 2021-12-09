#include "nif_planning_common/path_cost_calculator.hpp"

using namespace nif::planning::cost_calculator;

costCalculator::costCalculator() {
  m_weight_collision = WEIGHT_COLLISION;
  m_weight_curvature = WEIGHT_CURVATURE;
  m_weight_origin = WEIGHT_ORIGIN;
  m_weight_ref = WEIGHT_REF;
  m_weight_transient = WEIGHT_TRANSIENT;

  m_calc_path_first_called = false;
  m_occupancy_map_first_called = false;
}

costCalculator::costCalculator(double weight_collision_,
                               double weight_curvature_, double weight_origin_,
                               double weight_ref_, double weight_transient_)
    : m_weight_collision{weight_collision_},
      m_weight_curvature{weight_curvature_}, m_weight_origin{weight_origin_},
      m_weight_ref{weight_ref_}, m_weight_transient{weight_transient_} {
  m_calc_path_first_called = false;
  m_occupancy_map_first_called = false;
}

void costCalculator::setReferencePath(const nav_msgs::msg::Path &ref_path) {
  m_ref_x_vec.clear();
  m_ref_y_vec.clear();
  m_ref_path = ref_path;
  for (int i = 0; i < ref_path.poses.size(); i++) {
    m_ref_x_vec.push_back(ref_path.poses[i].pose.position.x);
    m_ref_y_vec.push_back(ref_path.poses[i].pose.position.y);
  }
}

void costCalculator::calcPathCost() {
  double max_collision_cost = 0.00001;
  double max_close_to_ref_path_cost = 0.00001;
  double max_curvature_cost = 0.00001;
  double max_transient_cost = 0.00001;
  double max_origin_to_path_cost = 0.00001;
  double min_dist = DBL_MAX; // to get the minimum index

  for (int i = 0; i < m_path_cadidates_ptr_vec.size(); i++) {
    for (int j = 0; j < m_path_cadidates_ptr_vec[i]->points_x().size(); j++) {
      double pt_x = m_path_cadidates_ptr_vec[i]->points_x()[j];
      double pt_y = m_path_cadidates_ptr_vec[i]->points_y()[j];
      double map_cost = this->mapCorrespondingCost(pt_x, pt_y);
      m_cost_collision_vec[i] += map_cost;

      double dist = sqrt(pow(pt_x, 2) + pow(pt_y, 2));
      if (dist < min_dist) {
        min_dist = dist;
        m_cost_origin_vec[i] = min_dist;
      }
    }

    double last_pt_x =
        m_path_cadidates_ptr_vec[i]
            ->points_x()[m_path_cadidates_ptr_vec[i]->points_x().size() - 1];
    double last_pt_y =
        m_path_cadidates_ptr_vec[i]
            ->points_y()[m_path_cadidates_ptr_vec[i]->points_y().size() - 1];
    m_cost_curvature_vec[i] = pow(atan2(last_pt_y, last_pt_x), 2);

    // std::cout << "HERE" << std::endl;
    // std::cout << "m_calc_path_first_called : " << m_calc_path_first_called <<
    // std::endl;

    if (!m_calc_path_first_called) {
      m_cost_transient_vec[i] = 0.0;
      m_prev_path_lat_displacement = 0.0;
      m_cur_path_lat_displacement = m_prev_path_lat_displacement;
      m_cost_ref_vec[i] = this->calculateProjDist(last_pt_x, last_pt_y,
                                                  m_ref_x_vec, m_ref_y_vec);
      // std::cout << "Transient " << m_cost_ref_vec[i] << std::endl;
      // assert(false);
      double transient_cost =
          pow(m_cur_path_lat_displacement - m_prev_path_lat_displacement, 2);
      m_cost_transient_vec[i] = transient_cost;
      m_prev_path_lat_displacement = m_cur_path_lat_displacement;
    } else {
      m_cur_path_lat_displacement = this->calculateSignedProjDist(
          last_pt_x, last_pt_y, m_ref_x_vec, m_ref_y_vec);
      double transient_cost =
          pow(m_cur_path_lat_displacement - m_prev_path_lat_displacement, 2);
      m_cost_transient_vec[i] = transient_cost;
      m_cost_ref_vec[i] = abs(m_cur_path_lat_displacement);
      m_prev_path_lat_displacement = m_cur_path_lat_displacement;
    }
  }
  m_calc_path_first_called = true;

  auto max_cost_collision = *max_element(std::begin(m_cost_collision_vec),
                                         std::end(m_cost_collision_vec));
  auto max_cost_curvature = *max_element(std::begin(m_cost_curvature_vec),
                                         std::end(m_cost_curvature_vec));
  auto max_cost_origin =
      *max_element(std::begin(m_cost_origin_vec), std::end(m_cost_origin_vec));
  auto max_cost_ref =
      *max_element(std::begin(m_cost_ref_vec), std::end(m_cost_ref_vec));
  auto max_cost_transient = *max_element(std::begin(m_cost_transient_vec),
                                         std::end(m_cost_transient_vec));

  // normalization
  if (max_cost_collision < 1.0)
    max_cost_collision = 1.0;
  if (max_cost_curvature < 1.0)
    max_cost_curvature = 1.0;
  if (max_cost_origin < 1.0)
    max_cost_origin = 1.0;
  if (max_cost_ref < 1.0)
    max_cost_ref = 1.0;
  if (max_cost_transient < 1.0)
    max_cost_transient = 1.0;

  m_cost_total_vec.clear();
  // std::cout << "-------------------------------------" << std::endl;

  for (int i = 0; i < m_path_cadidates_ptr_vec.size(); i++) {
    double total_cost =
        m_weight_collision * m_cost_collision_vec[i] / max_cost_collision +
        m_weight_ref * m_cost_ref_vec[i] / max_cost_ref +
        m_weight_origin * m_cost_origin_vec[i] / max_cost_origin +
        m_weight_curvature * m_cost_curvature_vec[i] / max_cost_curvature +
        m_weight_transient * m_cost_transient_vec[i] / max_cost_transient;

    m_cost_total_vec.push_back(total_cost);
  }

  int min_cost_frenet_path_idx =
      std::min_element(m_cost_total_vec.begin(), m_cost_total_vec.end()) -
      m_cost_total_vec.begin();

  if (m_last_min_cost_idx != -1 &&
      m_last_min_cost_idx != min_cost_frenet_path_idx) {
    std::cout << "min_cost_frenet_path_idx : " << min_cost_frenet_path_idx
              << std::endl;
    std::cout << "m_cost_collision_vec : "
              << m_cost_collision_vec[min_cost_frenet_path_idx] << std::endl;
    std::cout << "m_cost_curvature_vec : "
              << m_cost_curvature_vec[min_cost_frenet_path_idx] << std::endl;
    std::cout << "m_cost_origin_vec : "
              << m_cost_origin_vec[min_cost_frenet_path_idx] << std::endl;
    std::cout << "m_cost_ref_vec : " << m_cost_ref_vec[min_cost_frenet_path_idx]
              << std::endl;
    std::cout << "m_cost_transient_vec : "
              << m_cost_transient_vec[min_cost_frenet_path_idx] << std::endl;

    std::cout << "-----------------" << std::endl;
    for (int i = 0; i < m_cost_transient_vec.size(); i++) {
      std::cout << "transient cost : " << m_cost_transient_vec[i] << std::endl;
    }

    assert(false);
  }

  std::cout << "min_cost_frenet_path_idx : " << min_cost_frenet_path_idx
            << std::endl;
  std::cout << "m_cost_collision_vec : "
            << m_cost_collision_vec[min_cost_frenet_path_idx] << std::endl;
  std::cout << "m_cost_curvature_vec : "
            << m_cost_curvature_vec[min_cost_frenet_path_idx] << std::endl;
  std::cout << "m_cost_origin_vec : "
            << m_cost_origin_vec[min_cost_frenet_path_idx] << std::endl;
  std::cout << "m_cost_ref_vec : " << m_cost_ref_vec[min_cost_frenet_path_idx]
            << std::endl;
  std::cout << "m_cost_transient_vec : "
            << m_cost_transient_vec[min_cost_frenet_path_idx] << std::endl;

  if (m_cost_total_vec.size() == 0) {
    m_last_min_cost_path = NULL;
  } else {
    m_last_min_cost_path = m_path_cadidates_ptr_vec[min_cost_frenet_path_idx];
  }

  m_last_min_cost_idx = min_cost_frenet_path_idx;
}

double costCalculator::mapCorrespondingCost(double pt_x, double pt_y) {
  //  TODO : cost map health check
  if (!m_occupancy_map_first_called) {
    return 0;
  }
  int pt_in_grid_x =
      (pt_x) / m_OccupancyGrid.info.resolution -
      m_OccupancyGrid.info.origin.position.x / m_OccupancyGrid.info.resolution +
      1;
  int pt_in_grid_y =
      (pt_y) / m_OccupancyGrid.info.resolution -
      m_OccupancyGrid.info.origin.position.y / m_OccupancyGrid.info.resolution +
      1;
  double cost;
  if (pt_in_grid_x > m_OccupancyGrid.info.width ||
      pt_in_grid_y > m_OccupancyGrid.info.height) {
    cost = 100;
    return cost;
  }
  cost = m_OccupancyGrid
             .data[pt_in_grid_y * m_OccupancyGrid.info.width + pt_in_grid_x];
  return cost;
}

void costCalculator::setFrenetPathArray(
    vector<shared_ptr<Frenet::FrenetPath>> &path_cadidates_ptr_vec) {
  m_path_cadidates_ptr_vec = path_cadidates_ptr_vec;
  m_cost_collision_vec.clear();
  m_cost_curvature_vec.clear();
  m_cost_origin_vec.clear();
  m_cost_ref_vec.clear();
  m_cost_transient_vec.clear();
  m_cost_total_vec.clear();
  int vec_size = m_path_cadidates_ptr_vec.size();
  m_cost_collision_vec.resize(vec_size, 0.0);
  m_cost_curvature_vec.resize(vec_size, 0.0);
  m_cost_origin_vec.resize(vec_size, 0.0);
  m_cost_ref_vec.resize(vec_size, 0.0);
  m_cost_transient_vec.resize(vec_size, 0.0);
  m_cost_total_vec.resize(vec_size, 0.0);
  this->calcPathCost();
}

void costCalculator::setOccupancyMap(
    const nav_msgs::msg::OccupancyGrid &occupancy_map) {
  m_OccupancyGrid = occupancy_map;

  //  TODO : last update time check
  m_occupancy_map_first_called = true;
}

double costCalculator::calculateProjDist(double &pt_x_, double &pt_y_,
                                         vector<double> &x_vec,
                                         vector<double> &y_vec) {
  bool first_node = true;
  double prev_x, prev_y, bias, normal_distance;
  double min_distance = DBL_MAX;
  for (int i = 0; i < x_vec.size(); i++) {
    if (first_node) {
      prev_x = x_vec[i];
      prev_y = y_vec[i];
      first_node = false;
      continue;
    }
    double slope;
    if (x_vec[i] - prev_x == 0) {
      slope = 0;
    } else {
      slope = (y_vec[i] - prev_y) / (x_vec[i] - prev_x);
    }
    bias = y_vec[i] - slope * x_vec[i];
    normal_distance =
        fabs(slope * pt_x_ - pt_y_ + bias) / sqrt(pow(slope, 2) + 1);

    double prod1 = (pt_x_ - prev_x) * (x_vec[i] - prev_x) +
                   (pt_y_ - prev_y) * (y_vec[i] - prev_y);
    double prod2 = (pt_x_ - x_vec[i]) * (prev_x - x_vec[i]) +
                   (pt_y_ - y_vec[i]) * (prev_y - y_vec[i]);

    if (prod1 < 0 || prod2 < 0)
      continue;

    if (normal_distance < min_distance) {
      min_distance = normal_distance;
    }
    prev_x = x_vec[i];
    prev_y = y_vec[i];
  }

  return min_distance;
}
double costCalculator::calculateSignedProjDist(double &pt_x_, double &pt_y_,
                                               vector<double> &x_vec,
                                               vector<double> &y_vec) {
  bool first_node = true;
  double prev_x, prev_y, bias, normal_distance;
  double min_distance = DBL_MAX;
  double signed_min_distance = DBL_MAX;
  for (int i = 0; i < x_vec.size(); i++) {
    if (first_node) {
      prev_x = x_vec[i];
      prev_y = y_vec[i];
      first_node = false;
      continue;
    }
    double slope;
    if (x_vec[i] - prev_x == 0) {
      slope = 0;
    } else {
      slope = (y_vec[i] - prev_y) / (x_vec[i] - prev_x);
    }
    bias = y_vec[i] - slope * x_vec[i];
    normal_distance = (slope * pt_x_ - pt_y_ + bias) / sqrt(pow(slope, 2) + 1);

    double prod1 = (pt_x_ - prev_x) * (x_vec[i] - prev_x) +
                   (pt_y_ - prev_y) * (y_vec[i] - prev_y);
    double prod2 = (pt_x_ - x_vec[i]) * (prev_x - x_vec[i]) +
                   (pt_y_ - y_vec[i]) * (prev_y - y_vec[i]);

    if (prod1 < 0 || prod2 < 0)
      continue;

    if (fabs(normal_distance) < min_distance) {
      min_distance = fabs(normal_distance);

      if (normal_distance < 0)
        signed_min_distance = -1 * min_distance;
      else {
        signed_min_distance = min_distance;
      }
    }
    prev_x = x_vec[i];
    prev_y = y_vec[i];
  }

  return signed_min_distance;
}
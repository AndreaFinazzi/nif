#include "nif_planning_common/path_cost_calculator.hpp"

using namespace nif::planning::cost_calculator;

costCalculator::costCalculator(double weight_collision_,
                             double weight_curvature_,
                             double weight_origin_,
                             double weight_ref_,
                             double weight_transient_) :
      m_weight_collision{weight_collision_},
      m_weight_curvature{weight_curvature_},
      m_weight_origin{weight_origin_},
      m_weight_ref{weight_ref_},
      m_weight_transient{weight_transient_} {
  m_calc_path_first_called = false;
}

void costCalculator::calcPathCost() {
  double max_collision_cost = 0.00001;
  double max_close_to_ref_path_cost = 0.00001;
  double max_curvature_cost = 0.00001;
  double max_transient_cost = 0.00001;
  double max_origin_to_path_cost = 0.00001;
  double min_dist = DBL_MAX; //to get the minimum index


  for(int i = 0; i < m_path_cadidates_ptr_vec.size(); i++){
    for(int j = 0; j < m_path_cadidates_ptr_vec[i]->points_x().size(); j++) {
      double pt_x = m_path_cadidates_ptr_vec[i]->points_x()[j];
      double pt_y = m_path_cadidates_ptr_vec[i]->points_y()[j];
      double map_cost = this->mapCorrespondingCost(pt_x, pt_y);
      m_cost_collision_vec[i] += map_cost;

      double dist = sqrt(pow(pt_x,2) + pow(pt_y,2));
      if(dist < min_dist) {
        min_dist = dist;
        m_cost_origin_vec[i] = min_dist;
      }
    }
    double last_pt_x = m_path_cadidates_ptr_vec[i]->points_x()[m_path_cadidates_ptr_vec[i]->points_x().size()-1];
    double last_pt_y = m_path_cadidates_ptr_vec[i]->points_x()[m_path_cadidates_ptr_vec[i]->points_x().size()-1];
    m_cost_ref_vec[i] = this->calculateProjDist(last_pt_x,last_pt_y, m_ref_x_vec, m_ref_y_vec);
    m_cost_curvature_vec[i] = pow(atan2(last_pt_y, last_pt_x), 2);

    if(!m_calc_path_first_called) {
      m_cost_transient_vec[i] = 0.0;
    } else {

    }

  }


  m_calc_path_first_called = true;
}

double costCalculator::mapCorrespondingCost(double pt_x, double pt_y) {
  //  TODO : cost map health check
  int pt_in_grid_x = (pt_x) / m_OccupancyGrid.info.resolution - m_OccupancyGrid.info.origin.position.x / m_OccupancyGrid.info.resolution + 1;
  int pt_in_grid_y = (pt_y) / m_OccupancyGrid.info.resolution - m_OccupancyGrid.info.origin.position.y / m_OccupancyGrid.info.resolution + 1;
  double cost;
  if(pt_in_grid_x > m_OccupancyGrid.info.width || pt_in_grid_y > m_OccupancyGrid.info.height)
  {
    cost = 100;
    return cost;
  }
  cost = m_OccupancyGrid.data[pt_in_grid_y * m_OccupancyGrid.info.width + pt_in_grid_x];
  return cost;
}

void costCalculator::setFrenetPathArray(vector<shared_ptr<Frenet::FrenetPath>> &path_cadidates_ptr_vec) {
  m_path_cadidates_ptr_vec = path_cadidates_ptr_vec;
  m_cost_collision_vec.clear();
  m_cost_curvature_vec.clear();
  m_cost_origin_vec.clear();
  m_cost_ref_vec.clear();
  m_cost_transient_vec.clear();
  m_cost_collision_vec.resize(m_path_cadidates_ptr_vec.size(), 0.0);
  m_cost_curvature_vec.resize(m_path_cadidates_ptr_vec.size(), 0.0);
  m_cost_origin_vec.resize(m_path_cadidates_ptr_vec.size(), 0.0);
  m_cost_ref_vec.resize(m_path_cadidates_ptr_vec.size(), 0.0);
  m_cost_transient_vec.resize(m_path_cadidates_ptr_vec.size(), 0.0);
  this->calcPathCost();
}

void costCalculator::setOccupancyMap(const nav_msgs::msg::OccupancyGrid &occupancy_map) {
  m_OccupancyGrid = occupancy_map;
}

double costCalculator::calculateProjDist(double pt_x_, double pt_y_, vector<double> x_vec, vector<double> y_vec) {
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
    normal_distance = fabs(slope * pt_x_ - pt_y_ + bias) / sqrt(pow(slope, 2) + 1);

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
double costCalculator::CalculateSignedProjDist(double pt_x_, double pt_y_,
                                               vector<double> x_vec, vector<double> y_vec)
{
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

      if(normal_distance < 0)
        signed_min_distance = -1 * min_distance;
      else
      {
        signed_min_distance = min_distance;
      }
    }
    prev_x = x_vec[i];
    prev_y = y_vec[i];
  }

  return signed_min_distance;
}
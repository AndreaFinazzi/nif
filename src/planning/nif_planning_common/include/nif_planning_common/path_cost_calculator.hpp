#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <assert.h>
#include <string>
#include "vector"
#include "nif_utils/frenet_path_generator.h"
#include "nif_common/constants.h"

#define WEIGHT_COLLISION 10
#define WEIGHT_CURVATURE 1
#define WEIGHT_ORIGIN 1
#define WEIGHT_REF 1
#define WEIGHT_TRANSIENT 5

namespace nif {
namespace planning {
namespace cost_calculator {
class costCalculator {
public:
  costCalculator(double weight_collision_,
                 double weight_curvature_,
                 double weight_origin_,
                 double weight_ref_,
                 double weight_transient_);
  void setFrenetPathArray(vector<shared_ptr<Frenet::FrenetPath>>& path_cadidates_ptr_vec);
  void setOccupancyMap(const nav_msgs::msg::OccupancyGrid &occupancy_map);
  double calculateProjDist(double pt_x_, double pt_y_,
                           vector<double> x_vec, vector<double> y_vec );
  double CalculateSignedProjDist(double pt_x_, double pt_y_,
                                 vector<double> x_vec, vector<double> y_vec);
  void calcPathCost();

  double mapCorrespondingCost(double x_,double y_);

private:
  double m_weight_collision;
  double m_weight_curvature;
  double m_weight_origin;
  double m_weight_ref;
  double m_weight_transient;

  bool m_calc_path_first_called;

  vector<shared_ptr<Frenet::FrenetPath>> m_path_cadidates_ptr_vec;
  vector<double> m_cost_collision_vec;
  vector<double> m_cost_curvature_vec;
  vector<double> m_cost_origin_vec;
  vector<double> m_cost_ref_vec;
  vector<double> m_cost_transient_vec;
  shared_ptr<Frenet::FrenetPath> m_last_min_cost_path;
  nav_msgs::msg::OccupancyGrid m_OccupancyGrid;
  nav_msgs::msg::Path m_ref_path;
  vector<double> m_ref_x_vec;
  vector<double> m_ref_y_vec;

  double m_prev_path_lat_displacement;

public:
  void setWeightCollision(double weight_collision_) {
    if (weight_collision_ > 0)
      m_weight_collision = weight_collision_;
  }
  void setWeightCurvature(double weight_curvature_) {
    if (weight_curvature_ > 0)
      m_weight_curvature = weight_curvature_;
  }
  void setWeightOrigin(double weight_origin_) {
    if (weight_origin_ > 0)
      m_weight_origin = weight_origin_;
  }
  void setWeightRef(double weight_ref_) {
    if (weight_ref_ > 0)
      m_weight_ref = weight_ref_;
  }
  void setWeightTransient(double weight_transient_) {
    if (weight_transient_ > 0)
      m_weight_transient = weight_transient_;
  }
  double getWeightCollision(){return m_weight_collision;}
  double getWeightCurvature(){return m_weight_curvature;}
  double getWeightOrigin(){return m_weight_origin;}
  double getWeightRef(){return m_weight_ref;}
  double getWeightTransient(){return m_weight_transient;}
  shared_ptr<Frenet::FrenetPath> getMincostFrenetPath(){return m_last_min_cost_path;}

  void setReferencePath(const nav_msgs::msg::Path ref_path);

};
} // namespace cost_calculator

} // namespace planning

} // namespace nif

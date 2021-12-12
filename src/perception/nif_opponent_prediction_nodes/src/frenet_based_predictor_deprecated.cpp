#include "frenet_based_predictor.h"

using namespace nif::perception;

OpponentPredictor::OpponentPredictor(std::string target_ref_file_path) {
  init_success_flg = false;
  m_target_ref_file_path = target_ref_file_path;
  obj_tmp = new c_wpt(target_ref_file_path, "target_ref", "map", false, true,
                      m_target_path_sampling_interval);
  m_target_path = obj_tmp.getWPTinNavPath(); // target path in global coordinate

  // TODO : calculate cubic spliner model

  init_success_flg = true;
}

void OpponentPredictor::setOpponentStatus(
    const nif::common::msgs::PerceptionResult::SharedPtr oppo_status,
    const nav_msgs::msg::Odometry &ego_status) {
  m_opponent_status = oppo_status;
  m_ego_status = ego_status;

  // TODO : change oppo_status from local to global (using
  // nif::common::utils::coordination::getPtBodytoGlobal)

  // TODO : calc progress
}

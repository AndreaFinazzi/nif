#include "aptiv_mrr_driver/can_serialization.hpp"

#include <rclcpp/time.hpp>

constexpr auto micro_to_ns = 1000U;

namespace aptiv_mrr_driver {
namespace can {
namespace system_status {

type from_ros(const delphi_mrr_msgs::msg::VehicleStateMsg2::SharedPtr msg) {
  type can_msg;
  can_msg.can_yaw_rate = yaw_rate_encode(msg->fsm_yaw_rate);
  can_msg.can_yaw_rate_validity = msg->fsm_yaw_rate_valid;
  can_msg.can_vehicle_speed_validity = true;
  can_msg.can_vehicle_speed = vehicle_speed_encode(msg->fsm_vehicle_velocity);
  can_msg.can_vehicle_speed_direction = VEHICLE_SPEED_DIRECTION_FOWARD;  // (1)
  return can_msg;
}
} // namespace system_status 

namespace ifv_status_compensated {

type from_ros(const delphi_mrr_msgs::msg::VehicleStateMsg2::SharedPtr msg) {
  type can_msg;

  uint8_t can_yaw_rate_calf_qf = (msg->fsm_yaw_rate_valid) ?
    YAW_RATE_CALC_QF_ACCURATE : YAW_RATE_CALC_QF_UNDEFINED;

  can_msg.can_yaw_rate_calc_qf = can_yaw_rate_calf_qf;
  can_msg.can_yaw_rate_calc = yaw_rate_calc_encode(msg->fsm_yaw_rate);
  can_msg.can_vehicle_speed_calc = vehicle_speed_calc_encode(msg->fsm_vehicle_velocity);
  can_msg.can_veh_spd_comp_factor = veh_spd_comp_factor_encode(1.0);
  return can_msg;
}
  
} // namespace ifv_status_compensated

namespace detection {

delphi_mrr_msgs::msg::Detection to_ros(const type& can_msg,
                                          uint64_t stamp, unsigned frame_id) {
  delphi_mrr_msgs::msg::Detection ros_msg;
  ros_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * micro_to_ns));
  ros_msg.detection_id = static_cast<uint8_t>(frame_id - frame_id_first + 1);
  ros_msg.confid_azimuth = can_msg.can_det_confid_azimuth_01;
  ros_msg.super_res_target = can_msg.can_det_super_res_target_01;
  ros_msg.nd_target = can_msg.can_det_nd_target_01;
  ros_msg.valid_level = can_msg.can_det_valid_level_01;
  ros_msg.host_veh_clutter = can_msg.can_det_host_veh_clutter_01;
  ros_msg.azimuth = det_azimuth_decode(can_msg.can_det_azimuth_01);
  ros_msg.range = can_msg.can_det_range_01;
  ros_msg.range_rate = can_msg.can_det_range_rate_01;
  ros_msg.index2lsb = can_msg.can_scan_index_2_lsb_01;
  ros_msg.amplitude = can_msg.can_det_amplitude_01;
  
  return ros_msg;
}

} // namespace detection

namespace header_info_detections {

delphi_mrr_msgs::msg::MrrHeaderInformationDetections to_ros(
    const type& can_msg, uint64_t stamp) {
  delphi_mrr_msgs::msg::MrrHeaderInformationDetections ros_msg;
  ros_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * micro_to_ns));
  ros_msg.can_align_uodates_done = can_msg.can_align_updates_done;
  ros_msg.can_scan_index = can_msg.can_scan_index;
  ros_msg.can_number_of_det = can_msg.can_number_of_det;
  ros_msg.can_look_id = can_msg.can_look_id;
  ros_msg.can_look_index = can_msg.can_look_index;

  return ros_msg;
}

} // namespace header_info_detections

} // namespace can 
} // namespace aptiv_mrr_driver

/*
(1) the speed direction is only used to enable/disable the radar:
    foward = enabled, reverse = disabled
*/

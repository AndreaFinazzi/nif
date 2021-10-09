#pragma once

#include <delphi_mrr_msgs/msg/detection.hpp>
#include <delphi_mrr_msgs/msg/system_status.hpp>
#include <delphi_mrr_msgs/msg/mrr_header_information_detections.hpp>
#include <delphi_mrr_msgs/msg/vehicle_state_msg2.hpp>

#include "./mrr_pcan_core_v07_02.h"

namespace aptiv_mrr_driver {
namespace can {

namespace system_status {

using type = mrr_pcan_core_v07_02_system_status_t;
static constexpr auto length = MRR_PCAN_CORE_V07_02_SYSTEM_STATUS_LENGTH;
static constexpr auto yaw_rate_encode = mrr_pcan_core_v07_02_system_status_can_yaw_rate_encode;
static constexpr auto vehicle_speed_encode = mrr_pcan_core_v07_02_system_status_can_vehicle_speed_encode;
static constexpr auto pack = mrr_pcan_core_v07_02_system_status_pack;
static constexpr auto frame_id = MRR_PCAN_CORE_V07_02_SYSTEM_STATUS_FRAME_ID;
static constexpr auto VEHICLE_SPEED_DIRECTION_FOWARD = MRR_PCAN_CORE_V07_02_SYSTEM_STATUS_CAN_VEHICLE_SPEED_DIRECTION_FORWARD_CHOICE;
  
type from_ros(const delphi_mrr_msgs::msg::VehicleStateMsg2::SharedPtr msg);

} // namespace system_status 

namespace ifv_status_compensated {

using type = mrr_pcan_core_v07_02_ifv_status_compensated_t;
static constexpr auto length = MRR_PCAN_CORE_V07_02_IFV_TRACKS_CHANGE_LENGTH;
static constexpr auto yaw_rate_calc_encode = mrr_pcan_core_v07_02_ifv_status_compensated_can_yaw_rate_calc_encode;
static constexpr auto vehicle_speed_calc_encode = mrr_pcan_core_v07_02_ifv_status_compensated_can_vehicle_speed_calc_encode;
static constexpr auto veh_spd_comp_factor_encode = mrr_pcan_core_v07_02_ifv_status_compensated_can_veh_spd_comp_factor_encode;
static constexpr auto pack = mrr_pcan_core_v07_02_ifv_status_compensated_pack;
static constexpr auto frame_id = MRR_PCAN_CORE_V07_02_IFV_STATUS_COMPENSATED_FRAME_ID;
static constexpr auto YAW_RATE_CALC_QF_ACCURATE = MRR_PCAN_CORE_V07_02_IFV_STATUS_COMPENSATED_CAN_YAW_RATE_CALC_QF_ACCURATE_CHOICE;
static constexpr auto YAW_RATE_CALC_QF_UNDEFINED = MRR_PCAN_CORE_V07_02_IFV_STATUS_COMPENSATED_CAN_YAW_RATE_CALC_QF_UNDEFINED_CHOICE;

type from_ros(const delphi_mrr_msgs::msg::VehicleStateMsg2::SharedPtr msg);

} // namespace ifv_status_compensated

namespace detection {

using type = mrr_pcan_core_v07_02_mrr_detection_001_t;
static constexpr auto length = MRR_PCAN_CORE_V07_02_MRR_DETECTION_001_LENGTH;
static constexpr auto frame_id_first = MRR_PCAN_CORE_V07_02_MRR_DETECTION_001_FRAME_ID;
static constexpr auto frame_id_last = MRR_PCAN_CORE_V07_02_MRR_DETECTION_064_FRAME_ID;
static constexpr auto unpack = mrr_pcan_core_v07_02_mrr_detection_001_unpack;
static constexpr auto det_azimuth_decode = mrr_pcan_core_v07_02_mrr_detection_001_can_det_azimuth_01_decode;
static constexpr auto det_range_decode = mrr_pcan_core_v07_02_mrr_detection_001_can_det_range_01_decode;
static constexpr auto det_range_rate_decode = mrr_pcan_core_v07_02_mrr_detection_001_can_det_range_rate_01_decode;
static constexpr auto det_amplitude_decode = mrr_pcan_core_v07_02_mrr_detection_001_can_det_amplitude_01_decode;

delphi_mrr_msgs::msg::Detection to_ros(const type& can_msg,
                                       uint64_t stamp, unsigned frame_id);
} // namespace detection

namespace header_info_detections {

using type = mrr_pcan_core_v07_02_mrr_header_information_detections_t;
static constexpr auto length = MRR_PCAN_CORE_V07_02_MRR_HEADER_INFORMATION_DETECTIONS_LENGTH;
static constexpr auto unpack = mrr_pcan_core_v07_02_mrr_header_information_detections_unpack;
static constexpr auto frame_id = MRR_PCAN_CORE_V07_02_MRR_HEADER_INFORMATION_DETECTIONS_FRAME_ID;

delphi_mrr_msgs::msg::MrrHeaderInformationDetections to_ros(
    const type& can_msg, uint64_t stamp);
  
} // namespace header_info_detections


} // namespace can 
} // namespace aptiv_mrr_driver

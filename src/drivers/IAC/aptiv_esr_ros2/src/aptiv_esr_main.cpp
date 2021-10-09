// Copyright 2021 Real Time Innovations, Inc. (RTI)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "delphi_esr_msgs/msg/esr_valid1.hpp"
#include "delphi_esr_msgs/msg/esr_valid2.hpp"
#include "delphi_esr_msgs/msg/esr_status1.hpp"
#include "delphi_esr_msgs/msg/esr_status2.hpp"
#include "delphi_esr_msgs/msg/esr_status3.hpp"
#include "delphi_esr_msgs/msg/esr_status4.hpp"
#include "delphi_esr_msgs/msg/esr_status5.hpp"
#include "delphi_esr_msgs/msg/esr_status6.hpp"
#include "delphi_esr_msgs/msg/esr_status7.hpp"
#include "delphi_esr_msgs/msg/esr_status8.hpp"
#include "delphi_esr_msgs/msg/esr_status9.hpp"
// control topics (to control the RADAR)
#include "delphi_esr_msgs/msg/esr_vehicle1.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle2.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle3.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle5.hpp"
// visualization
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class EsrDrive : public rclcpp::Node
{
  public:
    // TEST node args fromTopic, toTopic, useReliable
    EsrDrive()
    : Node("esr_driver")
    {
      // decide whether we use reliable transport or best effort
      this->declare_parameter<bool>("reliable", true);
      bool reliable;
      this->get_parameter("reliable", reliable);
      rclcpp::QoS esr_qos = rclcpp::QoS(1); 
      if(reliable) 
      {
        esr_qos.reliable();
      }
      else
      {
        esr_qos.best_effort();
      }

      // declare parameter for frame to publish in
      this->declare_parameter<std::string>("frame_id", "radar_front");
      this->get_parameter("frame_id", frame_id_);

      // all publishers
      pub_track_ = this->create_publisher<delphi_esr_msgs::msg::EsrTrack>(std::string("esr_track"), esr_qos);
      pub_esr_valid1_ = this->create_publisher<delphi_esr_msgs::msg::EsrValid1>(std::string("esr_valid1"), esr_qos);
      pub_esr_valid2_ = this->create_publisher<delphi_esr_msgs::msg::EsrValid2>(std::string("esr_valid2"), esr_qos);
      pub_esr_status1_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus1>(std::string("esr_status1"), esr_qos);
      pub_esr_status2_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus2>(std::string("esr_status2"), esr_qos);
      pub_esr_status3_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus3>(std::string("esr_status3"), esr_qos);
      pub_esr_status4_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus4>(std::string("esr_status4"), esr_qos);
      pub_esr_status5_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus5>(std::string("esr_status5"), esr_qos);
      pub_esr_status6_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus6>(std::string("esr_status6"), esr_qos);
      pub_esr_status7_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus7>(std::string("esr_status7"), esr_qos);
      pub_esr_status8_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus8>(std::string("esr_status8"), esr_qos);
      pub_esr_status9_ = this->create_publisher<delphi_esr_msgs::msg::EsrStatus9>(std::string("esr_status9"), esr_qos);
      pub_visz_static = this->create_publisher<visualization_msgs::msg::Marker>(std::string("radar_visz_static"), esr_qos);
      pub_visz_moving = this->create_publisher<visualization_msgs::msg::Marker>(std::string("radar_visz_moving"), esr_qos);

      publisher_ = this->create_publisher<can_msgs::msg::Frame>(std::string("to_can_bus"), esr_qos);

      // all subscriptions
      subscription_ = this->create_subscription<can_msgs::msg::Frame>(
        std::string("from_can_bus"), esr_qos, std::bind(&EsrDrive::topic_callback, this, _1));
      sub_esr_vehicle1_ = this->create_subscription<delphi_esr_msgs::msg::EsrVehicle1>(
        std::string("esr_vehicle1"), esr_qos, std::bind(&EsrDrive::esr_vehicle1_callback, this, _1));
      sub_esr_vehicle2_ = this->create_subscription<delphi_esr_msgs::msg::EsrVehicle2>(
        std::string("esr_vehicle2"), esr_qos, std::bind(&EsrDrive::esr_vehicle2_callback, this, _1));
      sub_esr_vehicle3_ = this->create_subscription<delphi_esr_msgs::msg::EsrVehicle3>(
        std::string("esr_vehicle3"), esr_qos, std::bind(&EsrDrive::esr_vehicle3_callback, this, _1));
      sub_esr_vehicle5_ = this->create_subscription<delphi_esr_msgs::msg::EsrVehicle5>(
        std::string("esr_vehicle5"), esr_qos, std::bind(&EsrDrive::esr_vehicle5_callback, this, _1));
    }

  private:
    // callback for received 'EsrVehicle1' topics: publish to CAN (CAN ID: 1264 / 0x4F0)
    void esr_vehicle1_callback(const delphi_esr_msgs::msg::EsrVehicle1::SharedPtr msg)
    {
      // CAN_RX_VEHICLE_SPEED : 7|11@0+ (0.0625,0) [0|127.9375] "m/s" 
      uint16_t uTmp = roundf(msg->vehicle_speed / 0.0625f);
      send_msg_.data[0] = uTmp >> 3;
      uTmp = uTmp << 11;
      // CAN_RX_VEHICLE_SPEED_DIRECTION : 12|1@0+ (1,0) [0|1] "" 
      if(msg->speed_direction) {
        uTmp |= 0x1000;
      }
      // CAN_RX_YAW_RATE : 11|12@0- (0.0625,0) [-128|127.9375] "deg/s" 
      int16_t iTmp = roundf(msg->yaw_rate / 0.0625f);
      uTmp |= iTmp & 0xfff;
      send_msg_.data[1] = uTmp >> 8;
      send_msg_.data[2] = uTmp & 0xff;

      uTmp = 0;
      // CAN_RX_YAW_RATE_VALIDITY : 31|1@0+ (1,0) [0|0] "" 
      if(msg->yaw_rate_valid) {
        uTmp |= 0x8000;
      }
      // CAN_RX_STEERING_ANGLE_RATE_SIGN : 30|1@0+ (1,0) [0|1] "" 
      if(msg->steering_angle_rate_sign) {
        uTmp |= 0x4000;
      }
      // CAN_RX_RADIUS_CURVATURE : 29|14@0- (1,0) [-8192|8191] "m" 
      uTmp |= (msg->radius_curvature & 0x3FFF);
      send_msg_.data[3] = uTmp >> 8;
      send_msg_.data[4] = uTmp & 0xff;
      uTmp = 0;

      // CAN_RX_STEERING_VALIDITY : 47|1@0+ (1,0) [0|1] "" 
      if(msg->steering_angle_valid) {
        uTmp |= 0x8000;
      }
      // CAN_RX_STEERING_ANGLE_SIGN : 46|1@0+ (1,0) [0|1] "" 
      if(msg->steering_angle_sign) {
        uTmp |= 0x4000;
      }
      // CAN_RX_STEERING_ANGLE : 45|11@0+ (1,0) [0|2047] "deg" 
      uTmp |= ((msg->steering_angle & 0x7ff) << 3);
      send_msg_.data[5] = uTmp >> 8;
      uTmp = uTmp << 8;

      // CAN_RX_STEERING_ANGLE_RATE : 50|11@0+ (1,0) [0|2047] "deg/s" 
      uTmp |= (msg->steering_angle_rate & 0x7ff);
      send_msg_.data[6] = uTmp >> 8;
      send_msg_.data[7] = uTmp & 0xff;
      
      send_msg_.id = 1264;
      send_msg_.is_error = false;
      send_msg_.is_extended = false;
      send_msg_.is_rtr = false;
      send_msg_.dlc = 8;
      send_msg_.header.frame_id = frame_id_;
      send_msg_.header.stamp = this->now();
      publisher_->publish(send_msg_);
    }

    // callback for received 'EsrVehicle2' topics: publish to CAN (CAN ID: 1265 / 0x4F1)
    void esr_vehicle2_callback(const delphi_esr_msgs::msg::EsrVehicle2::SharedPtr msg)
    {
      // CAN_RX_SCAN_INDEX_ACK : 7|16@0+ (1,0) [0|65535] "" 
      send_msg_.data[0] = msg->scan_index_ack >> 8;
      send_msg_.data[1] = msg->scan_index_ack & 0xff;

      uint16_t uTmp = 0;
      // CAN_RX_USE_ANGLE_MISALIGNMENT : 23|1@0+ (1,0) [0|0] "" 
      if(msg->use_angle_misalignment) {
        uTmp |= 0x8000;
      }
      // CAN_RX_CLEAR_FAULTS : 22|1@0- (1,0) [0|0] "" 
      if(msg->clear_faults) {
        uTmp |= 0x4000;
      }
      // CAN_RX_HIGH_YAW_ANGLE : 21|6@0- (1,0) [-32|31] "deg" 
      uTmp |= (((int16_t)msg->high_yaw_angle) & 0x3f) << 8;
      // CAN_RX_MR_ONLY_TRANSMIT : 25|1@0+ (1,0) [0|0] "" 
      if(msg->mr_only_transmit) {
        uTmp |= 0x02;
      }
      // CAN_RX_LR_ONLY_TRANSMIT : 24|1@0+ (1,0) [0|0] "" 
      if(msg->lr_only_transmit) {
        uTmp |= 0x01;
      }
      send_msg_.data[2] = uTmp >> 8;
      send_msg_.data[3] = uTmp & 0xff;

      // CAN_RX_VOLVO_SHORT_TRACK_ROC : 31|4@0- (500,0) [-4000|3500] "m" 
      // no element in topic
      
      // CAN_RX_ANGLE_MISALIGNMENT : 39|8@0- (0.0625,0) [-8|7.9375] "deg"
      int16_t iTmp = roundf(msg->angle_misalignment / 0.0625f);
      send_msg_.data[4] = iTmp & 0xff;

      // CAN_RX_LATERAL_MOUNTING_OFFSET : 47|8@0- (0.01563,0) [-2.00064|1.98501] "m" 
      iTmp = roundf(msg->lateral_mounting_offset / 0.01563);
      send_msg_.data[5] = iTmp & 0xff;

      uTmp = 0;
      // CAN_RX_RADAR_CMD_RADIATE : 55|1@0+ (1,0) [0|1] "" 
      if(msg->radar_cmd_radiate) {
        uTmp |= 0x8000;
      }
      // CAN_RX_BLOCKAGE_DISABLE : 54|1@0+ (1,0) [0|1] "" 
      if(msg->blockage_disable) {
        uTmp |= 0x4000;
      }
      // CAN_RX_MAXIMUM_TRACKS : 53|6@0+ (1,0) [0|63] "" 
      uTmp |= ((((uint16_t)msg->maximum_tracks) & 0x3f) << 8);
      // CAN_RX_TURN_SIGNAL_STATUS : 63|2@0+ (1,0) [0|3] "" 
      uTmp |= (msg->turn_signal_status & 0x3) << 6;
      // CAN_RX_VEHICLE_SPEED_VALIDITY : 61|1@0+ (1,0) [0|1] "" 
      if(msg->vehicle_speed_valid) {
        uTmp |= 0x20;
      }
      // CAN_RX_MMR_UPSIDE_DOWN : 60|1@0+ (1,0) [0|1] "" 
      if(msg->mmr_upside_down) {
        uTmp |= 0x10;
      }
      // CAN_RX_GROUPING_MODE : 59|2@0+ (1,0) [0|3] "" 
      uTmp |= (msg->grouping_mode & 0x3) << 2;
      // CAN_RX_WIPER_STATUS : 57|1@0+ (1,0) [0|1] "" 
      if(msg->wiper_status) {
        uTmp |= 0x02;
      }
      // CAN_RX_RAW_DATA_ENABLE : 56|1@0+ (1,0) [0|1] "" 
      if(msg->raw_data_enabled) {
        uTmp |= 0x01;
      }
      send_msg_.data[6] = uTmp >> 8;
      send_msg_.data[7] = uTmp & 0xff;

      send_msg_.id = 1265;
      send_msg_.is_error = false;
      send_msg_.is_extended = false;
      send_msg_.is_rtr = false;
      send_msg_.dlc = 8;
      send_msg_.header.frame_id = frame_id_;
      send_msg_.header.stamp = this->now();
      publisher_->publish(send_msg_);
    }

    // callback for received 'EsrVehicle3' topics: publish to CAN (CAN ID: 1522 / 0x5F2)
    void esr_vehicle3_callback(const delphi_esr_msgs::msg::EsrVehicle3::SharedPtr msg)
    {
      uint16_t uTmp = 0;
      // CAN_RX_LONG_ACCEL_VALIDITY : 7|1@0+ (1,0) [0|1] "" 
      if(msg->long_accel_valid) {
        uTmp |= 0x8000;
      }
      // CAN_RX_LAT_ACCEL_VALIDITY : 6|1@0+ (1,0) [0|1] "" 
      if(msg->lat_accel_valid) {
        uTmp |= 0x4000;
      }
      // CAN_RX_LAT_ACCEL : 5|9@0- (0.03125,0) [-8|7.96875] "m/s^2" 
      uTmp |= ((((int16_t)roundf(msg->lat_accel / 0.03125f)) & 0x1ff) << 5);
      send_msg_.data[0] = uTmp >> 8;
      uTmp = uTmp << 8;
      // CAN_RX_LONG_ACCEL : 12|9@0- (0.03125,0) [-8|7.96875] "m/s^2" 
      uTmp |= ((((int16_t)roundf(msg->long_accel / 0.03125f)) & 0x1ff) << 4);
      send_msg_.data[1] = uTmp >> 8;
      uTmp = uTmp << 8;

      // CAN_RX_RADAR_FOV_LR : 19|5@0+ (1,0) [0|31] "deg" 
      uTmp |= (((uint16_t)msg->radar_fov_lr & 0x1f) << 7);
      // CAN_RX_RADAR_FOV_MR : 30|7@0+ (1,0) [0|127] "deg"
      uTmp |= (msg->radar_fov_mr & 0x7f);
      send_msg_.data[2] = uTmp >> 8;
      send_msg_.data[3] = uTmp & 0xff;
      uTmp = 0;

      // CAN_RX_AUTO_ALIGN_DISABLE : 39|1@0+ (1,0) [0|1] "" 
      if(msg->auto_align_disable) {
        uTmp |= 0x8000;
      }
      // CAN_RX_RADAR_HEIGHT : 38|7@0+ (1,0) [0|127] "cm" 
      uTmp |= ((uint16_t)msg->radar_height & 0x7f) << 8;
      // CAN_RX_SERV_ALIGN_TYPE : 47|1@0+ (1,0) [0|1] "" 
      if(msg->serv_align_type) {
        uTmp |= 0x80;
      }
      // CAN_RX_AALIGN_AVG_CTR_TOTAL : 45|3@0+ (250,250) [250|2000] "" 
      uTmp |= (((msg->align_avg_ctr_total - 250) / 250) & 0x7) << 3;
      // CAN_RX_AUTO_ALIGN_CONVERGED : 42|1@0+ (1,0) [0|1] "" 
      if(msg->auto_align_converged) {
        uTmp |= 0x4;
      }
      // CAN_RX_WHEEL_SLIP : 41|2@0+ (1,0) [0|3] "" 
      uTmp |= msg->wheel_slip & 0x3;
      send_msg_.data[4] = uTmp >> 8;
      send_msg_.data[5] = uTmp & 0xff;

      // CAN_RX_SERV_ALIGN_UPDATES_NEED : 55|8@0+ (1,0) [0|255] "" 
      send_msg_.data[6] = msg->serv_align_updates_need;
      // CAN_RX_ANGLE_MOUNTING_OFFSET : 63|8@0- (0.0625,0) [-8|7.9375] "deg" 
      // NOTE: this range (-8 to 7.9475) doesn't match with the received element (int8)
      send_msg_.data[7] = msg->angle_mounting_offset;
      send_msg_.id = 1522;
      send_msg_.is_error = false;
      send_msg_.is_extended = false;
      send_msg_.is_rtr = false;
      send_msg_.dlc = 8;
      send_msg_.header.frame_id = frame_id_;
      send_msg_.header.stamp = this->now();
      publisher_->publish(send_msg_);
    }

    // callback for received 'EsrVehicle5' topics: publish to CAN (CAN ID: 1524 / 0x5F4)
    void esr_vehicle5_callback(const delphi_esr_msgs::msg::EsrVehicle5::SharedPtr msg)
    {
      // CAN_RX_OVERSTEER_UNDERSTEER : 7|8@0- (1,0) [-128|127] "%" 
      send_msg_.data[0] = msg->oversteer_understeer;

      // CAN_RX_BEAMWIDTH_VERT : 14|7@0+ (0.0625,0) [0|7.9375] "deg" 
      uint16_t uTmp = ((uint16_t)roundf(msg->beamwidth_vert / 0.0625f)) & 0x7f;
      // CAN_RX_YAW_RATE_BIAS_SHIFT : 15|1@0+ (1,0) [0|1] "" 
      if(msg->yaw_rate_bias_shift) {
        uTmp |= 0x80;
      }
      send_msg_.data[1] = uTmp & 0xff;

      // CAN_RX_FUNNEL_OFFSET_LEFT : 23|8@0- (0.1,0) [-12.8|12.7] "m" 
      int8_t iTmp = (int8_t)roundf(msg->funnel_offset_left / 0.1f);
      send_msg_.data[2] = iTmp;

      // CAN_RX_FUNNEL_OFFSET_RIGHT : 31|8@0- (0.1,0) [-12.8|12.7] "m" 
      iTmp = (int8_t)roundf(msg->funnel_offset_right / 0.1f);
      send_msg_.data[3] = iTmp;

      // CAN_RX_CW_BLOCKAGE_TRESHOLD : 39|8@0+ (0.0078125,0) [0|1.9921875] "" 
      send_msg_.data[4] = roundf(msg->cw_blockage_threshold / 0.0078125f);

      // CAN_RX_DISTANCE_REAR_AXLE : 47|8@0+ (2,200) [200|710] "cm" 
      send_msg_.data[5] = (msg->distance_rear_axle - 200) / 2;

      // CAN_RX_WHEELBASE : 55|8@0+ (2,200) [200|710] "cm" 
      send_msg_.data[6] = (msg->wheel_base - 200) / 2;

      // CAN_RX_STEERING_GEAR_RATIO : 63|8@0+ (0.125,0) [0|31.875] "" 
      send_msg_.data[7] = roundf(msg->steering_gear_ratio / 0.125f);
      send_msg_.id = 1524;
      send_msg_.is_error = false;
      send_msg_.is_extended = false;
      send_msg_.is_rtr = false;
      send_msg_.dlc = 8;
      send_msg_.header.frame_id = frame_id_;
      send_msg_.header.stamp = this->now();
      publisher_->publish(send_msg_);
    }

    // callback for received CAN data ----------------------------------------
    void topic_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
      int16_t iTmp;
      uint16_t uTmp;
      uint32_t wTmp;
      std::stringstream tmpSs;
      switch(msg->id) {
		  case 0x4E0: // 1248: ESR_Status --> EsrStatus1
        // CAN_TX_ROLLING_COUNT_1 : 6|2@1+ (1,0) [0|3]
        myStatus1_.rolling_count = (msg->data[0] >> 5) & 0x3; 
         // CAN_TX_DSP_TIMESTAMP : 5|7@0+ (2,0) [0|254] "ms" 
        myStatus1_.time_stamp = 
          ((msg->data[0] & 0x3f) << 2) | ((msg->data[1] >> 6) & 0x2);
        // CAN_TX_COMM_ERROR : 14|1@0+ (1,0) [0|0]          
        myStatus1_.comm_error = msg->data[1] & 0x40;
        // CAN_TX_RADIUS_CURVATURE_CALC : 13|14@0- (1,0) [-8192|8191]
        iTmp = (((int16_t)msg->data[1] & 0x3f) << 8) | msg->data[2];
        iTmp = iTmp >> 1;
        if(iTmp & (1<<12)) {
          iTmp -= 16384;
        }
        myStatus1_.curvature = iTmp;
        // CAN_TX_SCAN_INDEX : 31|16@0+ (1,0) [0|65535]
        myStatus1_.scan_id = ((uint16_t)msg->data[3] << 8) | msg->data[4];
        // CAN_TX_YAW_RATE_CALC : 47|12@0- (0.0625,0) [-128|127.9375] "deg/s"
        iTmp = (((int16_t)msg->data[5] << 4) | (msg->data[6] >> 4));
        if(iTmp & (1<<11)) {
          iTmp -= 4096;
        } 
        myStatus1_.yaw_rate = (float)iTmp * 0.0625f;
        // CAN_TX_VEHICLE_SPEED_CALC : 50|11@0+ (0.0625,0) [0|127.9375] "m/s" 
        iTmp = (((int16_t)msg->data[6] & 0x7) << 8) | msg->data[7];
        if(iTmp & (1<<10)) {
          iTmp -= 2048;
        }
        myStatus1_.vehicle_speed_calc = (float)iTmp * 0.0625f;
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus1_.canmsg = tmpSs.str();
        myStatus1_.header.frame_id = std::string("can");
        myStatus1_.header.stamp = this->now();
        pub_esr_status1_->publish(myStatus1_);
        break;

		  case 0x4E1: // 1249: ESR_Status2 --> EsrStatus2
        // CAN_TX_MAXIMUM_TRACKS_ACK : 7|6@0+ (1,1) [1|64] "" 
        myStatus2_.max_track_ack = (msg->data[0] >> 2) + 1;

        // CAN_TX_ROLLING_COUNT_2 : 1|2@0+ (1,0) [0|3] "" 
        myStatus2_.rolling_count2 = (msg->data[0] & 0x3);

        // CAN_TX_OVERHEAT_ERROR : 15|1@0+ (1,0) [0|1] "" 
        myStatus2_.overheat_error = msg->data[1] & 0x80;

        // CAN_TX_RANGE_PERF_ERROR : 14|1@0+ (1,0) [0|1] "" 
        myStatus2_.range_perf_error = msg->data[1] & 0x40;

        // CAN_TX_INTERNAL_ERROR : 13|1@0+ (1,0) [0|1] "" 
        myStatus2_.internal_error = msg->data[1] & 0x20;

        // CAN_TX_XCVR_OPERATIONAL : 12|1@0+ (1,0) [0|1] "" 
        myStatus2_.xcvr_operational = msg->data[1] & 0x10;

        // CAN_TX_RAW_DATA_MODE : 11|1@0+ (1,0) [0|1] "" 
        myStatus2_.raw_data_mode = msg->data[1] & 0x08;

        // CAN_TX_STEERING_ANGLE_ACK : 10|11@0+ (1,0) [0|2047] "deg" 
        myStatus2_.steer_angle_ack = ((uint16_t)msg->data[1] << 8) | msg->data[2];

        // CAN_TX_TEMPERATURE : 31|8@0- (1,0) [-128|127] "degC" 
        myStatus2_.temperature = msg->data[3];

        // CAN_TX_VEH_SPD_COMP_FACTOR : 39|6@0- (0.00195,1) [0.9376|1.06045] "" 
        iTmp = msg->data[4] >> 2;
        if(iTmp & (1<<6)) {
          iTmp -= 64;
        }
        myStatus2_.spd_comp_factor = ((float)iTmp * 0.00195f) + 1;

        // CAN_TX_GROUPING_MODE : 33|2@0+ (1,0) [0|3] "" 
        myStatus2_.grouping_mode = msg->data[4] & 0x3;

        // CAN_TX_YAW_RATE_BIAS : 47|8@0- (0.125,0) [-16|15.875] "" 
        iTmp = (int16_t)msg->data[5];
        if(iTmp & 0x80) {
          iTmp -= 256;
        }
        myStatus2_.yaw_rate_bias = (float)iTmp * 0.125;

        // CAN_TX_SW_VERSION_DSP : 55|16@0+ (1,0) [0|65535] "" 
        iTmp = (((uint16_t)msg->data[6]) << 8) | msg->data[7];
        myStatus2_.sw_version_dsp = std::to_string(iTmp);
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus2_.canmsg = tmpSs.str();
        myStatus2_.header.frame_id = std::string("can");
        myStatus2_.header.stamp = this->now();
        pub_esr_status2_->publish(myStatus2_);
        break;

		  case 0x4E2: // 1250: ESR_SW --> EsrStatus3
        // CAN_TX_INTERFACE_VERSION : 7|4@0+ (1,0) [0|15] "" 
        myStatus3_.interface_version = msg->data[0] >> 4;

        // CAN_TX_HW_VERSION : 3|4@0+ (1,0) [0|15] "" 
        myStatus3_.hw_version = msg->data[0] & 0xf;

        // CAN_TX_SW_VERSION_HOST : 15|24@0+ (1,0) [0|16777215] "" 
        wTmp = (((uint32_t)msg->data[1]) << 16) 
             | (((uint32_t)msg->data[2]) << 8)
             | msg->data[3];
        myStatus3_.sw_version_host = std::to_string(wTmp);

        // CAN_TX_SERIAL_NUM : 39|24@0+ (1,0) [0|16777215] "" 
        wTmp = (((uint32_t)msg->data[4]) << 16) 
             | (((uint32_t)msg->data[5]) << 8)
             | msg->data[6];
        myStatus3_.serial_num = std::to_string(wTmp);

        // CAN_TX_SW_VERSION_PLD : 63|8@0+ (1,0) [0|255] "" 
        myStatus3_.sw_version_pld = msg->data[7];
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus3_.canmsg = tmpSs.str();
        myStatus3_.header.frame_id = std::string("can");
        myStatus3_.header.stamp = this->now();
        pub_esr_status3_->publish(myStatus3_);
        break;

		  case 0x4E3: // 1251: ESR_Output_InPath --> EsrStatus4
        // CAN_TX_TRUCK_TARGET_DET : 7|1@0+ (1,0) [0|1] "" 
        myStatus4_.truck_target_det = msg->data[0] & (1<<7);

        // CAN_TX_LR_ONLY_GRATING_LOBE_DET : 6|1@0+ (1,0) [0|1] "" 
        myStatus4_.lr_only_grating_lobe_det = msg->data[0] & (1<<6);

        // CAN_TX_SIDELOBE_BLOCKATE : 5|1@0+ (1,0) [0|1] "" 
        myStatus4_.side_lobe_blockage = msg->data[0] & (1<<5);

        // CAN_TX_PARTIAL_BLOCKAGE : 4|1@0+ (1,0) [0|1] "" 
        myStatus4_.patial_blockage = msg->data[0] & (1<<4);

        // CAN_TX_LMR_LR_MODE : 3|2@0+ (1,0) [0|3] "" 
        myStatus4_.mrlr_mode = (msg->data[0] >> 2) & 0x3;

        // CAN_TX_ROLLING_COUNT_3 : 1|2@0+ (1,0) [0|3] "" 
        myStatus4_.rolling_count3 = msg->data[0] & 0x3;

        // CAN_TX_PATH_ID_ACC_MOVE : 15|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_acc = msg->data[1];

        // CAN_TX_PATH_ID_CMBB_MOVE : 23|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_cmmb_move = msg->data[2];

        // CAN_TX_PATH_ID_CMBB_STAT : 31|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_cmmb_stat = msg->data[3];

        // CAN_TX_PATH_ID_FCW_MOVE : 39|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_fcw_move = msg->data[4];

        // CAN_TX_PATH_ID_FCW_STAT : 47|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_fcw_stat = msg->data[5];

        // CAN_TX_AUTO_ALIGN_ANGLE : 55|8@0- (0.0625,0) [-8|7.9375] "" 
        iTmp = msg->data[6];
        if(iTmp & (1<<7)) {
          iTmp -= (1<<8);
        }
        myStatus4_.auto_algin_angle = ((float)iTmp * 0.0625f);

        // CAN_TX_PATH_ID_ACC_STAT : 63|8@0+ (1,0) [0|255] "" 
        myStatus4_.path_id_acc_stat = msg->data[7];
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus4_.canmsg = tmpSs.str();
        myStatus4_.header.frame_id = std::string("can");
        myStatus4_.header.stamp = this->now();
        pub_esr_status4_->publish(myStatus4_);
        break;

      // case 0x500--0x53F (1280 to 1343) are tracked objects
		  case 0x500: case 0x501: case 0x502: case 0x503: case 0x504: case 0x505: case 0x506: case 0x507:
      case 0x508: case 0x509: case 0x50A: case 0x50B: case 0x50C: case 0x50D: case 0x50E: case 0x50F:
      case 0x510: case 0x511: case 0x512: case 0x513: case 0x514: case 0x515: case 0x516: case 0x517:
      case 0x518: case 0x519: case 0x51A: case 0x51B: case 0x51C: case 0x51D: case 0x51E: case 0x51F:
      case 0x520: case 0x521: case 0x522: case 0x523: case 0x524: case 0x525: case 0x526: case 0x527:
      case 0x528: case 0x529: case 0x52A: case 0x52B: case 0x52C: case 0x52D: case 0x52E: case 0x52F:
      case 0x530: case 0x531: case 0x532: case 0x533: case 0x534: case 0x535: case 0x536: case 0x537:
      case 0x538: case 0x539: case 0x53A: case 0x53B: case 0x53C: case 0x53D: case 0x53E: case 0x53F:    // tracked objects
        if(msg->data[1]) {
          // field is missing in message definition
          // bool TRACK_ONCOMING = msg->data[0] & 0x1;             // 0|1@0+ (1,0) [0|1] "" 
          bool TRACK_GROUPING_CHANGED = msg->data[0] & 0x2;     // 1|1@0+ (1,0) [0|1] "" 
          iTmp = (msg->data[0] >> 2);
          float TRACK_LAT_RATE = (float)iTmp * 0.25f;           // 7|6@0- (0.25,0) [-8|7.75] "" 
          iTmp = ((((int16_t)msg->data[1] & 0x1f) << 5) 
            | (((int16_t)msg->data[2] >> 3) & 0x1f));
          if(msg->data[1] & 0x10) {
            iTmp |= 0xFC00;
          }
          float TRACK_ANGLE = (float)iTmp * -0.1f;               // 12|10@0- (0.1,0) [-51.2|51.1] "" // negative through flipped mounting on AV21
          uint8_t TRACK_STATUS = (msg->data[1] >> 5) & 0x07;             // 15|3@0+ (1,0) [0|7] "" 
          float TRACK_RANGE = 
            ((((int32_t)msg->data[2] & 0x07) << 8) + (int32_t)msg->data[3]) * 0.1f;                    // 18|11@0+ (0.1,0) [0|204.7] "m" 

          float TRACK_WIDTH = ((msg->data[4] >> 2) & 0x02) * 0.5f;     // 37|4@0+ (0.5,0) [0|7.5] "" 
          bool TRACK_ROLLING_COUNT = (bool) ((msg->data[4] >> 6) & 0x01);       // 38|1@0+ (1,0) [0|1] "" 
          bool TRACK_BRIDGE_OBJECT = (bool) ((msg->data[4] >> 7) & 0x01);       // 39|1@0+ (1,0) [0|1] "" 

          // Trace Range Accel
          iTmp = ((((int16_t)msg->data[4] & 0x03) << 8) + (int16_t)msg->data[5]);
          if(msg->data[4] & 0x02) {
              iTmp |= 0xFC00;
          }
          float TRACK_RANGE_ACCEL = (float)iTmp * 0.05f;        // 33|10@0- (0.05,0) [-25.6|25.55] "m/s^2" 

          // Track Range Rate
          iTmp = ((((int16_t) msg->data[6] & 0x3f) << 8) + (int16_t)msg->data[7]);
          if(msg->data[6] & 0x20) {
              iTmp |= 0xC000;
          }
          float TRACK_RANGE_RATE = ((float)iTmp * 0.01f);

          uint8_t TRACK_MED_RANGE_MODE = ((msg->data[6] >> 6) & 0x03);   // 55|2@0+ (1,0) [0|3] "" 
        
          // fprintf(stderr, "[%x]: %u %u LatRate:%6.2f Angle:%5.1f sts:%u Range:%4.1f RngAccel:%7.2f wid:%0.1f %u %u RngRate:%6.2f medRngMode:%u\n",
          //   msg->id,
          //   TRACK_ONCOMING,
          //   TRACK_GROUPING_CHANGED,
          //   TRACK_LAT_RATE,
          //   TRACK_ANGLE,
          //   TRACK_STATUS,
          //   TRACK_RANGE,
          //   TRACK_RANGE_ACCEL,
          //   TRACK_WIDTH,
          //   TRACK_ROLLING_COUNT,
          //   TRACK_BRIDGE_OBJECT,
          //   TRACK_RANGE_RATE,
          //   TRACK_MED_RANGE_MODE
          // );
          // publish the track

          delphi_esr_msgs::msg::EsrTrack myTrack;
          for(int i=0; i<8 ; ++i) {
            tmpSs << std::hex << (int)msg->data[i];
          }
          myTrack.canmsg = tmpSs.str();
          myTrack.track_id = (uint8_t)msg->id;    // 0 to 63
          myTrack.track_lat_rate = TRACK_LAT_RATE;
          myTrack.track_group_changed = TRACK_GROUPING_CHANGED;
          myTrack.track_lat_rate = TRACK_LAT_RATE;
          myTrack.track_angle = TRACK_ANGLE;
          myTrack.track_status = TRACK_STATUS;
          myTrack.track_range = TRACK_RANGE;
          myTrack.track_range_accel = TRACK_RANGE_ACCEL;
          myTrack.track_width = TRACK_WIDTH;
          myTrack.track_rolling_count = TRACK_ROLLING_COUNT;
          myTrack.track_bridge_object = TRACK_BRIDGE_OBJECT;
          myTrack.track_range_rate = TRACK_RANGE_RATE;
          myTrack.track_med_range_mode = TRACK_MED_RANGE_MODE;
          myTrack.header.frame_id = frame_id_;
          myTrack.header.stamp = this->now();
          pub_track_->publish(myTrack);
          // visualize the track 
          myVisz_.header.frame_id = frame_id_;
          myVisz_.header.stamp = this->now();
	        myVisz_.id = (uint8_t)msg->id;    // 0 to 63
          myVisz_.type = 3; // visualization_msgs::Marker::CYLINDER;
          // let if live for one second
	        myVisz_.lifetime = rclcpp::Duration(1, 0);
	        myVisz_.pose.position.x = TRACK_RANGE*cos(2*3.14*TRACK_ANGLE/360);
          myVisz_.pose.position.y = TRACK_RANGE*sin(2*3.14*TRACK_ANGLE/360);
	        myVisz_.scale.x = 0.5;
	        myVisz_.scale.y = 0.5;
	        myVisz_.scale.z = 3.0;
          // color according to relative velocity
          float colorcode = (std::max(std::min(myTrack.track_range_rate, 40.0f), -40.0f) / 80.0f) + 0.5f;
          if(!moving_tracks_[myTrack.track_id])
          {
            if(3 <= myTrack.track_status && 4 >= myTrack.track_status )
            {
	            myVisz_.color.g = colorcode;
              myVisz_.color.r = 1 - colorcode;
	            myVisz_.color.a = 1;
              pub_visz_moving->publish(myVisz_);
            }
          }else{
            if(3 <= myTrack.track_status && 4 >= myTrack.track_status )
            {
	            myVisz_.color.g = colorcode;
              myVisz_.color.r = 1 - colorcode;
	            myVisz_.color.a = 1;
              pub_visz_static->publish(myVisz_);
            }
          }
        }
        break;

		  case 0x540: // 1344: TrackSensor1 --> EsrTrackMotionPower (somehow)
        uTmp = msg->data[0] & 0xf; // can id group: 0 to 9 (groups 1 to 10)

        // ESR tracks are their ID + 1 to start with 1
        if(uTmp <= 8) {
          for (int i = 1; i < 8; ++i) {
            moving_tracks_[(i-1) + 7*uTmp] = (msg->data[i] >> 5) & 1;      // bits     13, 21, 29, 37, 45, 53, 61
          }
        }else{
          moving_tracks_[63] = (msg->data[1] >> 5) & 1;                    // bit 13
        }
        break;

		  case 0x5D0: // 1488: SensorValidation --> EsrValid1
        myValid1_.lr_sn = msg->data[0];                  // 7|8@0+ (1,0) [0|255]
        uTmp = ((uint16_t)msg->data[1] << 8) | msg->data[2];
        myValid1_.lr_range = (float)uTmp * 0.0078125f;   // 15|16@0+ (0.0078125,0) [0|511.9921875] "m"
        iTmp = ((int16_t)msg->data[3] << 8) | msg->data[4];
        myValid1_.lr_range_rate = (float)iTmp * 0.0078125f;  // 31|16@0- (0.0078125,0) [-256|255.9921875] "m/s"
        iTmp = ((int16_t)msg->data[5] << 8) | msg->data[6];
        myValid1_.lr_angle = (float)iTmp * 0.0625f;      // 47|16@0- (0.0625,0) [-2048|2047.9375] "deg"
        myValid1_.lr_power = msg->data[7];               // 63|8@0- (1,0) [-128|127] "db"
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myValid1_.canmsg = tmpSs.str();
        myValid1_.header.frame_id = std::string("can");
        myValid1_.header.stamp = this->now();
        pub_esr_valid1_->publish(myValid1_);
        break;

		  case 0x5D1: // 1489: SensorValidation2 --> EsrValid2
        myValid2_.mr_sn = msg->data[0];                  // 7|8@0+ (1,0) [0|255]
        uTmp = ((uint16_t)msg->data[1] << 8) | msg->data[2];
        myValid2_.mr_range = (float)uTmp * 0.0078125f;   // 15|16@0+ (0.0078125,0) [0|511.9921875] "m"
        iTmp = ((int16_t)msg->data[3] << 8) | msg->data[4];
        myValid2_.mr_range_rate = (float)iTmp * 0.0078125f;  // 31|16@0- (0.0078125,0) [-256|255.9921875] "m/s"
        iTmp = ((int16_t)msg->data[5] << 8) | msg->data[6];
        myValid2_.mr_angle = (float)iTmp * 0.0625f;      // 47|16@0- (0.0625,0) [-2048|2047.9375] "deg"
        myValid2_.mr_power = msg->data[7];               // 63|8@0- (1,0) [-128|127] "db"
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myValid2_.canmsg = tmpSs.str();
        myValid2_.header.frame_id = std::string("can");
        myValid2_.header.stamp = this->now();
        pub_esr_valid2_->publish(myValid2_);
        break;

		  case 0x5E4: // 1508: AD_Data --> EsrStatus5
        myStatus5_.swbatt_a2d = msg->data[0];
        myStatus5_.ignp_a2d = msg->data[1];
        myStatus5_.temp1_a2d = msg->data[2];
        myStatus5_.temp2_a2d = msg->data[3];
        myStatus5_.supply_5v_a = msg->data[4];
        myStatus5_.supply_5v_dx = msg->data[5];
        myStatus5_.supply_53p_3v = msg->data[6];
        myStatus5_.supply_10_v = msg->data[7];
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus5_.canmsg = tmpSs.str();
        myStatus5_.header.frame_id = std::string("can");
        myStatus5_.header.stamp = this->now();
        pub_esr_status5_->publish(myStatus5_);
        break;

		  case 0x5E5: // 1509 ... guessing that it maps to EsrStatus6
        // FIXME: don't know the mapping here
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus6_.canmsg = tmpSs.str();
        break;

		  case 0x5E6: // 1510: ESR_Active_Fault --> EsrStatus7
        myStatus7_.active_fault0 = msg->data[0];
        myStatus7_.active_fault1 = msg->data[1];
        myStatus7_.active_fault2 = msg->data[2];
        myStatus7_.active_fault3 = msg->data[3];
        myStatus7_.active_fault4 = msg->data[4];
        myStatus7_.active_fault5 = msg->data[5];
        myStatus7_.active_fault6 = msg->data[6];
        myStatus7_.active_fault7 = msg->data[7];
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus7_.canmsg = tmpSs.str();
        myStatus7_.header.frame_id = std::string("can");
        myStatus7_.header.stamp = this->now();
        pub_esr_status7_->publish(myStatus7_);
        break;

		  case 0x5E7: // 1511: ESR_History_Fault --> EsrStatus8
        // FIXME: the byte order is a best-guess here
        myStatus8_.history_fault0 = msg->data[0];
        myStatus8_.history_fault1 = msg->data[1];
        myStatus8_.history_fault2 = msg->data[2];
        myStatus8_.history_fault3 = msg->data[3];
        myStatus8_.history_fault4 = msg->data[4];
        myStatus8_.history_fault5 = msg->data[5];
        myStatus8_.history_fault6 = msg->data[6];
        myStatus8_.history_fault7 = msg->data[7];
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus8_.canmsg = tmpSs.str();
        myStatus8_.header.frame_id = std::string("can");
        myStatus8_.header.stamp = this->now();
        pub_esr_status8_->publish(myStatus8_);
        break;

		  case 0x5E8: // 1512: CIPV_Targets_Etc --> EsrStatus9
        // FIXME: do not have docs for these bits
        for(int i=0; i<8 ; ++i) {
          tmpSs << std::hex << (int)msg->data[i];
        }
        myStatus9_.canmsg = tmpSs.str();
        myStatus9_.header.frame_id = std::string("can");
        myStatus9_.header.stamp = this->now();
        pub_esr_status9_->publish(myStatus9_);
        break;

		  case 0x5E9: // 1513 
        break;
		  case 0x5EA: // 1514
        break;
		  case 0x5EB: // 1515
        break;
		  case 0x5EC: // 1516
        break;
      default:
        break;
      }
    }

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
    can_msgs::msg::Frame send_msg_;

    rclcpp::Publisher<delphi_esr_msgs::msg::EsrTrack>::SharedPtr pub_track_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrValid1>::SharedPtr pub_esr_valid1_;
    delphi_esr_msgs::msg::EsrValid1 myValid1_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrValid2>::SharedPtr pub_esr_valid2_;
    delphi_esr_msgs::msg::EsrValid2 myValid2_;

    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus1>::SharedPtr pub_esr_status1_;
    delphi_esr_msgs::msg::EsrStatus1 myStatus1_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus2>::SharedPtr pub_esr_status2_;
    delphi_esr_msgs::msg::EsrStatus2 myStatus2_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus3>::SharedPtr pub_esr_status3_;
    delphi_esr_msgs::msg::EsrStatus3 myStatus3_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus4>::SharedPtr pub_esr_status4_;
    delphi_esr_msgs::msg::EsrStatus4 myStatus4_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus5>::SharedPtr pub_esr_status5_;
    delphi_esr_msgs::msg::EsrStatus5 myStatus5_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus6>::SharedPtr pub_esr_status6_;
    delphi_esr_msgs::msg::EsrStatus6 myStatus6_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus7>::SharedPtr pub_esr_status7_;
    delphi_esr_msgs::msg::EsrStatus7 myStatus7_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus8>::SharedPtr pub_esr_status8_;
    delphi_esr_msgs::msg::EsrStatus8 myStatus8_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrStatus9>::SharedPtr pub_esr_status9_;
    delphi_esr_msgs::msg::EsrStatus9 myStatus9_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_visz_static;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_visz_moving;

    visualization_msgs::msg::Marker myVisz_;


    rclcpp::Subscription<delphi_esr_msgs::msg::EsrVehicle1>::SharedPtr sub_esr_vehicle1_;
    rclcpp::Subscription<delphi_esr_msgs::msg::EsrVehicle2>::SharedPtr sub_esr_vehicle2_;
    rclcpp::Subscription<delphi_esr_msgs::msg::EsrVehicle3>::SharedPtr sub_esr_vehicle3_;
    rclcpp::Subscription<delphi_esr_msgs::msg::EsrVehicle5>::SharedPtr sub_esr_vehicle5_;

    std::string frame_id_;
    bool moving_tracks_[64];

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EsrDrive>());
  rclcpp::shutdown();
  return 0;
}

// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RAPTOR_DBW_CAN__DISPATCH_HPP_
#define RAPTOR_DBW_CAN__DISPATCH_HPP_

#include <stdint.h>

namespace raptor_dbw_can
{
typedef enum
{
  VIN_MUX_VIN0  = 0x00,
  VIN_MUX_VIN1  = 0x01,
  VIN_MUX_VIN2  = 0x02,
} VinMux;

typedef enum
{
  WHEEL_SPEED_MUX0  = 0x00,
  WHEEL_SPEED_MUX1  = 0x01,
  WHEEL_SPEED_MUX2  = 0x02,
} WheelSpeedMux;

#undef BUILD_ASSERT

enum
{
  ID_BASE_TO_CAR_SUMMARY        = 0x04B0,
  ID_BRAKE_CMD                  = 0x2F04,
  ID_BRAKE_REPORT               = 0x1F04,
  ID_ACCELERATOR_PEDAL_CMD      = 0x2F01,
  ID_ACCEL_PEDAL_REPORT         = 0x1F02,
  ID_STEERING_CMD               = 0x2F03,
  ID_STEERING_REPORT            = 0x1F03,
  ID_GEAR_CMD                   = 0x2F05,
  ID_GEAR_REPORT                = 0x1F05,
  ID_REPORT_WHEEL_SPEED         = 0x1F0B,
  ID_REPORT_IMU                 = 0x1F0A,
  ID_REPORT_TIRE_PRESSURE       = 0x1f07,
  ID_REPORT_SURROUND            = 0x1f10,
  ID_VIN                        = 0x1F08,
  ID_REPORT_DRIVER_INPUT        = 0x1F0F,
  ID_REPORT_WHEEL_POSITION      = 0x1F06,
  ID_MISC_REPORT                = 0x1F01,
  ID_LOW_VOLTAGE_SYSTEM_REPORT  = 0x1F11,
  ID_BRAKE_2_REPORT             = 0x1F12,
  ID_STEERING_2_REPORT          = 0x1F13,
  ID_OTHER_ACTUATORS_REPORT     = 0x1F14,
  ID_FAULT_ACTION_REPORT        = 0x1F15,
  ID_HMI_GLOBAL_ENABLE_REPORT   = 0x3f01,
  ID_TEST                       = 0x0718,
  ID_WHEEL_SPEED_REPORT_DO      = 0x0514,
  ID_BRAKE_PRESSURE_REPORT_DO   = 0x0515,
  ID_ACCELERATOR_REPORT_DO      = 0x0516,
  ID_STEERING_REPORT_DO         = 0x0517,
  ID_MISC_REPORT_DO             = 0x0518,
  ID_STEERING_REPORT_EXTD       = 0x0520,
  ID_RC_TO_CT                   = 0x051B,
  ID_WHEEL_STRAIN_GAUGE         = 0x051E,
  ID_WHEEL_POTENTIOMETER        = 0x051F,
  ID_TIRE_PRESSURE_FL           = 0x0528,
  ID_TIRE_PRESSURE_FR           = 0x0529,
  ID_TIRE_PRESSURE_RL           = 0x052A,
  ID_TIRE_PRESSURE_RR           = 0x052B,
  ID_TIRE_TEMP_FL_1             = 0x052C,
  ID_TIRE_TEMP_FL_2             = 0x052D,
  ID_TIRE_TEMP_FL_3             = 0x052E,
  ID_TIRE_TEMP_FL_4             = 0x052F,
  ID_TIRE_TEMP_FR_1             = 0x0530,
  ID_TIRE_TEMP_FR_2             = 0x0531,
  ID_TIRE_TEMP_FR_3             = 0x0532,
  ID_TIRE_TEMP_FR_4             = 0x0533,
  ID_TIRE_TEMP_RL_1             = 0x0534,
  ID_TIRE_TEMP_RL_2             = 0x0535,
  ID_TIRE_TEMP_RL_3             = 0x0536,
  ID_TIRE_TEMP_RL_4             = 0x0537,
  ID_TIRE_TEMP_RR_1             = 0x0538,
  ID_TIRE_TEMP_RR_2             = 0x0539,
  ID_TIRE_TEMP_RR_3             = 0x053A,
  ID_TIRE_TEMP_RR_4             = 0x053B,
  ID_VEL_ACC_HIL                = 0x05DD,
  ID_POSITION_HEADING_HIL       = 0x05DC,
  ID_PT_REPORT_1                = 0x053C,
  ID_PT_REPORT_2                = 0x053D,
};

}  // namespace raptor_dbw_can

#endif  // RAPTOR_DBW_CAN__DISPATCH_HPP_

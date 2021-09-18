// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief Base class for vehicle drivers
#ifndef VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_
#define VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_

#include <common/types.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <vehicle_interface/visibility_control.hpp>

#include <experimental/optional>
#include <cstdint>
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <utility>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using CtReport = deep_orange_msgs::msg::CtReport;

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

enum class DbwState
{
  DISABLED = 0,
  ENABLE_REQUESTED = 1,
  ENABLE_SENT = 2,
  ENABLED = 3
};  // enum class DbwState

enum class CTState
{
  CT1_PWR_ON = 1,
  CT2_INITIALIZED = 2,
  CT3_ACT_TEST = 3,
  CT4_CRANKREADY = 4,
  CT5_CRANKING = 5,
  CT6_RACEREADY = 6,
  CT7_INIT_DRIVING = 7,
  CT8_CAUTION = 8,
  CT9_NOM_RACE = 9,
  CT10_COORD_STOP = 10,
  CT11_CNTRL_SHUTDOWN = 11,
  CT12_EMRG_SHUTDOWN =12,
  CT255_DEFAULT =255
};

enum class SysState
{
  SS1_PWR_ON = 1,
  SS2_SUBSYS_CON = 2,
  SS3_ACT_TESTING = 3,
  SS4_ACT_TEST_DONE = 4,
  SS5_CRANKREADY = 5,
  SS6_PRECRANK_CHECK = 6,
  SS7_CRANKING = 7,
  SS8_ENG_RUNNING = 8,
  SS9_DRIVING = 9,
  SS10_SHUT_ENG = 10,
  SS11_PWR_OFF = 11,
  SS13_CRANK_CHECK_INIT = 13,
  SS255_DEFAULT = 255
};

enum class TrackCondition //track condition and vehicle signal difference
{
    TC0_NULL = 0,
  TC1_RED = 1, // 120162 - start engine, perform any final checks
  TC2_ORANGE = 2, // Proceed with caution
  TC3_YELLOW = 3,  // Race
  TC4_GREEN = 4,
  TC_DEFAULT = 255
};

enum class VehicleSignal
{
    VS0_NULL = 0,
  VS1_NULL = 1,
  VS2_BLACK = 2,
  VS4_CHECK = 4,
  VS8_PURPLE = 8
};

/// \brief Class for maintaining the DBW state
class VEHICLE_INTERFACE_PUBLIC DbwStateMachine
{
public:
  /// \brief Default constructor
  /// \param[in] dbw_disabled_debounce If state = ENABLE_SENT and DBW reports DISABLED, debounce this many msgs  // NOLINT
  explicit DbwStateMachine(uint16_t dbw_disabled_debounce);
  /// Destructor
  ~DbwStateMachine() = default;

  /// \brief Returns true if state is ENABLED, ENABLE_SENT, or ENABLE_REQUESTED with conditions
  bool8_t enabled() const;

  /// \brief Returns current internal state
  /// \return A DbwState object representing the current state
  DbwState get_state() const;

  /// \brief Notifies the state machine that feedback was received from the DBW system
  /// \param[in] enabled If true, DBW system reports enabled. If false, DBW system reports disabled
  void dbw_feedback(bool8_t enabled);

  /// \brief Notifies the state machine that a control command was sent to the DBW system
  void control_cmd_sent();

  /// \brief Notifies the state machine that a state command was sent to the DBW system
  void state_cmd_sent();

  /// \brief The user has requested the DBW system to enable (true) or disable (false)
  /// \param[in] enable If true, request enable. If false, request disable
  void user_request(bool8_t enable);

////////////////////////////////////////////////////////////////////

  CtReport ct_mode_transition(
    SysState raptor_mode_input,
    TrackCondition race_flag_input = TrackCondition::TC_DEFAULT,
    float32_t vehicle_vel = 0.0,
    VehicleSignal m_rec_veh_signal = VehicleSignal::VS1_NULL,
    bool8_t emergency_joy_flag = false,
    int emergency_confirmation = 0,
    bool8_t diag_hb_failure = false
    );

/////////////////////////////////////////////////////////////////////

private:
  bool8_t m_first_control_cmd_sent;
  bool8_t m_first_state_cmd_sent;
  uint16_t m_disabled_feedback_count;
  const uint16_t DISABLED_FEEDBACK_THRESH;
  DbwState m_state;

//////////////////////////////////////////////////////////////////////
  // SysState raptor_mode;
  CTState ct_current_mode;
  CtReport ct_to_rc_data;
//////////////////////////////////////////////////////////////////////

  void disable_and_reset();
};  // class DbwStateMachine

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

#endif  // VEHICLE_INTERFACE__DBW_STATE_MACHINE_HPP_

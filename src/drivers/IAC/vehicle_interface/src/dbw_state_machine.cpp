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

#include "vehicle_interface/dbw_state_machine.hpp"

namespace autoware
{
namespace drivers
{
namespace vehicle_interface
{

DbwStateMachine::DbwStateMachine(uint16_t dbw_disabled_debounce)
: m_first_control_cmd_sent{false},
  m_first_state_cmd_sent{false},
  m_disabled_feedback_count{0},
  DISABLED_FEEDBACK_THRESH{dbw_disabled_debounce},
  m_state{DbwState::DISABLED},
  ct_current_mode{CTState::CT255_DEFAULT}
{
}

bool8_t DbwStateMachine::enabled() const
{
  return m_state == DbwState::ENABLED ||
         m_state == DbwState::ENABLE_SENT ||
         (m_state == DbwState::ENABLE_REQUESTED &&
         m_first_control_cmd_sent &&
         m_first_state_cmd_sent);
}

DbwState DbwStateMachine::get_state() const {return m_state;}

void DbwStateMachine::dbw_feedback(bool8_t enabled)
{
  if (enabled) {                             // DBW system says enabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_state = DbwState::ENABLED;
      m_disabled_feedback_count = 0;
    }
  } else {                                   // DBW system says disabled
    if (m_state == DbwState::ENABLE_SENT) {  // and state is ENABLE_SENT
      m_disabled_feedback_count++;           // Increase debounce count

      if (m_disabled_feedback_count > DISABLED_FEEDBACK_THRESH) {  // check debounce
        disable_and_reset();
      }
    } else if (m_state == DbwState::ENABLED) {  // and state is ENABLED
      disable_and_reset();
    }
  }
}

void DbwStateMachine::control_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a control command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_control_cmd_sent = true;
  }
}

void DbwStateMachine::state_cmd_sent()
{
  if (m_state == DbwState::ENABLE_REQUESTED &&
    m_first_control_cmd_sent &&
    m_first_state_cmd_sent)
  {
    // We just sent a state command with
    // enable == true so we can transition
    // to ENABLE_SENT
    m_state = DbwState::ENABLE_SENT;
  }

  if (m_state == DbwState::ENABLE_REQUESTED) {
    m_first_state_cmd_sent = true;
  }
}

void DbwStateMachine::user_request(bool8_t enable)
{
  if (enable) {                           // Enable is being requested
    if (m_state == DbwState::DISABLED) {  // Only change states if currently in DISABLED
      m_state = DbwState::ENABLE_REQUESTED;
    }
  } else {                               // Disable is being requested
    disable_and_reset();                 // Disable in any state if user requests it
  }
}

void DbwStateMachine::disable_and_reset()
{
  m_state = DbwState::DISABLED;
  m_first_control_cmd_sent = false;
  m_first_state_cmd_sent = false;
  m_disabled_feedback_count = 0;
}

////////////////////////////////////////////////////////////////////////

CtReport DbwStateMachine::ct_mode_transition(
  SysState raptor_mode_input,
  TrackCondition race_flag_input,
  float32_t vehicle_vel,
  VehicleSignal m_rec_veh_signal,
  bool8_t emergency_joy_flag,
  int emergency_confirmation,
  bool8_t diag_hb_failure)
{
  switch (ct_current_mode) {
    case CTState::CT255_DEFAULT:
      if((raptor_mode_input == SysState::SS1_PWR_ON && race_flag_input == TrackCondition::TC1_RED)) {
          ct_current_mode = CTState::CT1_PWR_ON;
      }
      if(emergency_confirmation)
      {

      }
      // else if(!rc_flag_received && (test_mode_switch == 1 && raptor_mode_input == SysState::SS1_PWR_ON)){
      //     ct_current_mode = CTState::CT1_PWR_ON;
      // }

      break;
    case CTState::CT1_PWR_ON: //GNSS topic has to be added
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else {
       ct_current_mode = CTState::CT2_INITIALIZED;
      }

      break;

    case CTState::CT2_INITIALIZED:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
     else if (raptor_mode_input == SysState::SS4_ACT_TEST_DONE &&
          race_flag_input == TrackCondition::TC1_RED)
        {
          ct_current_mode = CTState::CT3_ACT_TEST; //check rc msg if coming is there or not and what is it? Raptor mode is PreCrankChecks (R4)
        }
      // else if (raptor_mode_input == SysState::SS4_ACT_TEST_DONE && test_mode_switch == 1) {
      //     ct_current_mode = CTState::CT3_ACT_TEST;
      //   }
      // RCLCPP_DEBUG_ONCE(node.get_logger(),"#####     Turn knob to 3      #####");
      break;

    case CTState::CT3_ACT_TEST:
    if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if (raptor_mode_input == SysState::SS5_CRANKREADY &&
          race_flag_input == TrackCondition::TC1_RED)
      {
        ct_current_mode = CTState::CT4_CRANKREADY;
      }
          // else if(joy_stick_switch) ct_current_mode = DOState::JOYSTICK;
      // else if (test_mode_switch == 1 && raptor_mode_input == SysState::SS5_CRANKREADY) {
      //     ct_current_mode = CTState::CT4_CRANKREADY;
      //   }
          // else if(joy_stick_switch) ct_current_mode = DOState::JOYSTICK;
          // else if(condition for sh)
      // RCLCPP_DEBUG_ONCE(node.get_logger(),"#####     Turn knob to 4      #####");
      break;
    case CTState::CT4_CRANKREADY:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if (race_flag_input == TrackCondition::TC2_ORANGE) {
        ct_current_mode = CTState::CT5_CRANKING;
      }
        // else if(test_mode_switch == 1){
        //  ct_current_mode = CTState::CT5_CRANKING;
        // }
      break;

    case CTState::CT5_CRANKING:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if (raptor_mode_input == SysState::SS8_ENG_RUNNING) {
        ct_current_mode = CTState::CT6_RACEREADY;
      }
      // else if(test_mode_switch == 1 && 
      // raptor_mode_input == SysState::SS8_ENG_RUNNING){
      //    ct_current_mode = CTState::CT6_RACEREADY;
      // }
      break;

    case CTState::CT6_RACEREADY:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if (race_flag_input == TrackCondition::TC3_YELLOW) {
        ct_current_mode = CTState::CT7_INIT_DRIVING;
      }
      break;

    case CTState::CT7_INIT_DRIVING:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if (raptor_mode_input == SysState::SS9_DRIVING &&
        race_flag_input == TrackCondition::TC3_YELLOW)
      {
        ct_current_mode = CTState::CT8_CAUTION;
      }
      break;

    case CTState::CT8_CAUTION:
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }

      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/

      else if(m_rec_veh_signal == VehicleSignal::VS8_PURPLE)
      {
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
      }

      else if (race_flag_input == TrackCondition::TC4_GREEN
      && m_rec_veh_signal == VehicleSignal::VS1_NULL) {
        ct_current_mode = CTState::CT9_NOM_RACE;
      }

      else if(vehicle_vel == 0.0F && m_rec_veh_signal == VehicleSignal::VS4_CHECK)
        ct_current_mode = CTState::CT11_CNTRL_SHUTDOWN;


      else if (race_flag_input == TrackCondition::TC1_RED) {
        ct_current_mode = CTState::CT10_COORD_STOP;
      }
      break;

    case CTState::CT9_NOM_RACE:
      // if (race_flag_input == TrackCondition::TC3_YELLOW) {
      //   ct_current_mode = CTState::CT7_CAUTION;
      // }
      // break;
      if(diag_hb_failure == true){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"diagnostic heartbeat failure"<<std::endl;
      }
      else if(emergency_joy_flag){
          ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
          std::cout<<"joystick emergency trigger"<<std::endl;
      }
      /*else if(emergency_hb_flag){
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        std::cout<<"fluidmesh connection broken"<<std::endl;
      }*/
      else if(m_rec_veh_signal == VehicleSignal::VS8_PURPLE)
      {
        ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
      }
      else if(m_rec_veh_signal == VehicleSignal::VS4_CHECK)
        ct_current_mode = CTState::CT8_CAUTION;

      else if (race_flag_input == TrackCondition::TC1_RED) {
        ct_current_mode = CTState::CT10_COORD_STOP;
      }
      break;

    case CTState::CT10_COORD_STOP:
        if(diag_hb_failure == true){
            ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
            std::cout<<"diagnostic heartbeat failure"<<std::endl;
        }
        else if(emergency_joy_flag){
            ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
            std::cout<<"joystick emergency trigger"<<std::endl;
        }
        else if(m_rec_veh_signal == VehicleSignal::VS8_PURPLE)
        {
            ct_current_mode = CTState::CT12_EMRG_SHUTDOWN;
        }
        break;

    case CTState::CT11_CNTRL_SHUTDOWN:
      break;
    case CTState::CT12_EMRG_SHUTDOWN:
      break;

  }

  switch (ct_current_mode) {
    case CTState::CT1_PWR_ON:
      ct_to_rc_data.ct_state = 1;
      break;
    case CTState::CT2_INITIALIZED:
      ct_to_rc_data.ct_state = 2;
      break;
    case CTState::CT3_ACT_TEST:
      ct_to_rc_data.ct_state = 3;
      break;
    case CTState::CT4_CRANKREADY:
      ct_to_rc_data.ct_state = 4;
      break;
    case CTState::CT5_CRANKING:
      ct_to_rc_data.ct_state = 5;
      break;
    case CTState::CT6_RACEREADY:
      ct_to_rc_data.ct_state = 6;
      break;
    case CTState::CT7_INIT_DRIVING:
      ct_to_rc_data.ct_state = 7;
      break;
    case CTState::CT8_CAUTION:
      ct_to_rc_data.ct_state = 8;
      break;
    case CTState::CT9_NOM_RACE:
      ct_to_rc_data.ct_state = 9;
      break;
    case CTState::CT10_COORD_STOP:
      ct_to_rc_data.ct_state = 10;
      break;
    case CTState::CT11_CNTRL_SHUTDOWN:
      ct_to_rc_data.ct_state = 11;
      break;
    case CTState::CT12_EMRG_SHUTDOWN:
      ct_to_rc_data.ct_state = 12;
      break;
    default:
      ct_to_rc_data.ct_state = 255;
  }
  return ct_to_rc_data;
}

////////////////////////////////////////////////////////////////////////////

}  // namespace vehicle_interface
}  // namespace drivers
}  // namespace autoware

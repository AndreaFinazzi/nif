//
// Created by usrg on 9/2/21.
//

#ifndef ROS2MASTER_LOWLEVELLONGIMINIMAL_H
#define ROS2MASTER_LOWLEVELLONGIMINIMAL_H

#include "iostream"
#include <map>
#include <functional>
#include <memory>
#include "nif_utils/PID.hpp"
#include "nif_common/vehicle_model.h"
#include "c_gear_shifter.h"
#include <yaml-cpp/yaml.h>

namespace nif {
    namespace control {
        namespace low_level {

            class LowLvlLongiMinimal {
            public:
                explicit LowLvlLongiMinimal(std::string &default_config_file_path);

                double getDesiredVelMps() const;

                void setDesiredVelMps(double desiredVelMps);

                double getParamThrottlePGain() const;

                void setParamThrottlePGain(double paramThrottlePGain);

                double getParamThrottleIGain() const;

                void setParamThrottleIGain(double paramThrottleIGain);

                double getParamThrottleDGain() const;

                void setParamThrottleDGain(double paramThrottleDGain);

                double getParamThrottleMaxIntegratorError() const;

                void setParamThrottleMaxIntegratorError(double paramThrottleMaxIntegratorError);

                double getParamBrakePGain() const;

                void setParamBrakePGain(double paramBrakePGain);

                double getParamBrakeIGain() const;

                void setParamBrakeIGain(double paramBrakeIGain);

                double getParamBrakeDGain() const;

                void setParamBrakeDGain(double paramBrakeDGain);

                double getParamBrakeMaxIntegratorError() const;

                void setParamBrakeMaxIntegratorError(double paramBrakeMaxIntegratorError);

            private:
                double desired_vel_mps;

                //! param file path
                std::string config_file_path;

                //! throttle params
                double param_throttle_p_gain;
                double param_throttle_i_gain;
                double param_throttle_d_gain;
                double param_throttle_max_integrator_error;
                double param_throttle_cmd_max;
                double param_throttle_cmd_min;
                //! brake params
                double param_brake_p_gain;
                double param_brake_i_gain;
                double param_brake_d_gain;
                double param_brake_max_integrator_error;
                double param_vel_error_deadband; // from bvs long-control
                double param_brake_cmd_max;
                double param_brake_cmd_min;
                //! PID controller time step
                double param_ts;

                std::map<int, std::shared_ptr<GearState>> gear_states;
                std::shared_ptr<GearState> cur_gear_ptr;

                utils::PID throttle_pid;
                utils::PID brake_pid;
            };
        }
    }
}

#endif //ROS2MASTER_LOWLEVELLONGIMINIMAL_H

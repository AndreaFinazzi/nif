//
// Created by usrg on 8/30/21.
//

#ifndef ROS2MASTER_LQRCONTROL_H
#define ROS2MASTER_LQRCONTROL_H

#include "param_struct.h"
#include "string"
#include "rclcpp/rclcpp.hpp"
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "nif_common/vehicle_model.h"
#include "lqr/lateral_lqr.h"
#include "utils/lateral_lqr_ros.h"
#include "utils/pure_pursuit_tracker.h"
#include "nav_msgs/msg/path.hpp"

using namespace std;

namespace nif {
    namespace control {
        namespace lateral {
            class LqrControl {
            public:
                explicit LqrControl(string
                                    config_file_path_);

                bool loadConfig(string config_file_path_);

                const string &getConfigFilePath() const;

                void setConfigFilePath(const string &configFilePath);

                bool isConfigLoad() const;

                const LqrControlParam &getParams() const;

                void setParams(const LqrControlParam &params);

                double getSteerAngleCmd() const;

                void setGlobalRefPath(nav_msgs::msg::Path::SharedPtr path_ptr);

                void setCurOdom(nav_msgs::msg::Odometry::SharedPtr odom_ptr);


                // calc algorithm outputs here
                void solve();

                double secs(rclcpp::Time t) {
                    return static_cast<double>(t.seconds()) +
                           static_cast<double>(t.nanoseconds()) * 1e-9;
                }

                double secs(rclcpp::Duration t) {
                    return static_cast<double>(t.seconds()) +
                           static_cast<double>(t.nanoseconds()) * 1e-9;
                }

            private:
                //! param for LQR controller
                string config_file_path;
                bool config_load_success_flg;
                LqrControlParam params;
                bool invert_steering = false;

                bool path_update_good_flg;

                //! algorithm inputs and params
                double current_speed_ms;
                //! params
                double max_steering_angle_deg;
                double steering_units_multiplier;
                double pure_pursuit_min_dist_m;
                double pure_pursuit_max_dist_m;
                double pure_pursuit_k_vel_m_ms;
                double steering_max_ddeg_dt;

                nav_msgs::msg::Path ref_path_in_global;
                nav_msgs::msg::Odometry cur_odom_in_global;


                //! algorithm outputs
                double steer_angle_cmd;
                double last_steer_angle_cmd;

                //! LQR Tracking State
                unsigned int lqr_tracking_idx;

                nif::control::lateral::lqr::LateralLQR::Ptr lateral_lqr;

            };
        }
    }
}

#endif //ROS2MASTER_LQRCONTROL_H

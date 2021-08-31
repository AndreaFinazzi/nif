//
// Created by usrg on 8/30/21.
//

#ifndef ROS2MASTER_KIN_LAT_CONTROLLER_H
#define ROS2MASTER_KIN_LAT_CONTROLLER_H

#include "param_struct.h"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "nif_common/vehicle_model.h"

using namespace std;

namespace nif {
    namespace control {
        namespace lateral {

            class kinControl {
            public:
                explicit kinControl(const string& config_file_path_);

                bool loadConfig(const string& config_file_path_);

                const string &getConfigFilePath() const;

                void setConfigFilePath(const string &configFilePath);

                bool isConfigLoad() const;

                const kinControlParam &getParams() const;

                void setParams(const kinControlParam &params);

                double getSteerAngleCmd() const;

                void setBodyRefPath(const nav_msgs::msg::Path::SharedPtr& path_ptr);

                void calculateFFW();

                void calculateFB();

                void calculateSteeringCmd();

                void setCmdsToZeros();

                static int findLookaheadIndex(
                        std::vector<geometry_msgs::msg::PoseStamped> refPath,
                        double desLookaheadValue);

                // calc algorithm outputs here
                void solve();

            private:
                // param for kin controller
                string config_file_path;
                bool config_load_success_flg;
                kinControlParam params;
                bool invert_steering = false;

                // algorithm inputs
                double cur_vel;
                double lat_error;
                double look_ahead_error;
                nav_msgs::msg::Path ref_path;

                double feedforward_error;
                double feedback_error;
                double curvature;

                rclcpp::Time path_update_time;
                bool path_update_good_flg;

                // algorithm outputs
                double steer_angle_cmd;
                double last_steer_angle_cmd;
            };
        }
    }
}


#endif //ROS2MASTER_KIN_LAT_CONTROLLER_H

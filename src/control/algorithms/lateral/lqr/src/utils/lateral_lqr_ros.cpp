#include "utils/lateral_lqr_ros.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace nif {
    namespace control {
        namespace lateral {
            namespace lqr {
                namespace utils {
                    lqr::LateralLQR::StateMatrix
                    LQRState(const nav_msgs::msg::Odometry &odom) {
                        lqr::LateralLQR::StateMatrix result;
                        //! These should just be in the correct frame
                        result(0, 0) = odom.pose.pose.position.x;
                        result(1, 0) = odom.pose.pose.position.y;
                        result(2, 0) = odom.twist.twist.linear.x;
                        result(3, 0) = odom.twist.twist.linear.y;

                        //! Handle yaw
                        tf2::Quaternion q(
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z,
                                odom.pose.pose.orientation.w
                        );
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        result(4, 0) = yaw;
                        // NOTE: This is probably not being estimated
                        result(5, 0) = odom.twist.twist.angular.z;
                        return result;
                    }

                    lqr::LateralLQR::GoalMatrix
                    LQRGoal(const nav_msgs::msg::Odometry &odom) {
                        lqr::LateralLQR::GoalMatrix result;
                        //! These should just be in the correct frame
                        result(0, 0) = odom.pose.pose.position.x;
                        result(1, 0) = odom.pose.pose.position.y;

                        //! Handle yaw
                        tf2::Quaternion q(
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z,
                                odom.pose.pose.orientation.w
                        );
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        result(2, 0) = yaw;
                        return result;
                    }

                    lqr::LateralLQR::GoalMatrix
                    LQRGoal(const geometry_msgs::msg::PoseStamped &pose) {
                        lqr::LateralLQR::GoalMatrix result;
                        //! These should just be in the correct frame
                        result(0, 0) = pose.pose.position.x;
                        result(1, 0) = pose.pose.position.y;

                        //! Handle yaw
                        tf2::Quaternion q(
                                pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z,
                                pose.pose.orientation.w
                        );
                        tf2::Matrix3x3 m(q);
                        double roll, pitch, yaw;
                        m.getRPY(roll, pitch, yaw);
                        result(2, 0) = yaw;
                        return result;
                    }

                    std_msgs::msg::Float32MultiArray
                    ROSError(const lqr::LateralLQR::ErrorMatrix &error) {
                        std_msgs::msg::Float32MultiArray result;
                        result.data.push_back(error(0, 0));
                        result.data.push_back(error(1, 0));
                        result.data.push_back(error(2, 0));
                        result.data.push_back(error(3, 0));
                        return result;
                    }
                }
            }
        }
    }
}



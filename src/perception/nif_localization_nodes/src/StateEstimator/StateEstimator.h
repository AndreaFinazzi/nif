/*
* Software License Agreement (BSD License)
* Copyright (c) 2013, Georgia Institute of Technology
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**********************************************
 * @file StateEstimator.h
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date May 1, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief StateEstimator class definition
 *
 ***********************************************/

#ifndef StateEstimator_H_
#define StateEstimator_H_

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/base/timing.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>

#include <list>
#include <iostream>
#include <fstream>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// NIF dependencies
// #include <gps_msgs/msg/gps_fix.hpp>
// #include <novatel_gps_msgs/msg/inspva.hpp>
#include <novatel_gps_msgs/msg/novatel_raw_imu.hpp>
#include <novatel_oem7_msgs/msg/inspva.hpp>

#include "BlockingQueue.h"

// Temporarily replaced by copy & paste
// #include <imu_3dm_gx4/msg/filter_output.hpp>
#include <nif_msgs/msg/filter_out.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// TF2 broadcasting
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// low_pass_filter
#include "nif_utils/low_pass_filter.h"


#define PI 3.14159265358979323846264338
#define IMULPFDT 0.0083
#define IMULPFCF 40.0


namespace autorally_core
{
  // struct lpf_xyz
  // {
  //   // low_pass_filter(IMULPFDT, IMULPFCF, 0.0) x;
  //   // low_pass_filter(IMULPFDT, IMULPFCF, 0.0) y;
  //   // low_pass_filter(IMULPFDT, IMULPFCF, 0.0) z;
  //   low_pass_filter x;
  //   low_pass_filter y;
  //   low_pass_filter z;

  // };
  // struct imu_lpf
  // {
  //   lpf_xyz angular_velocity;
  //   lpf_xyz linear_acceleration;
  // };
  class StateEstimator : public rclcpp::Node
  {
  private:


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr insPub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gpsPub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasAccPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasGyroPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr timePub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr statusPub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr insSub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Subscription<novatel_gps_msgs::msg::NovatelRawImu>::SharedPtr imuNovatelSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double lastImuT_, lastImuTgps_;
    /**
     * # possible statuses
     * byte OK=0     # standard operation
     * byte WARN=1   # state estimator is not currently trustworthy
     * byte ERROR=2  # state estimator has encountered an unrecoverable error
     */
    int status_;
    double accelBiasSigma_, gyroBiasSigma_;
    double gpsSigma_;
    int maxQSize_;

    BlockingQueue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsOptQ_;
    BlockingQueue<sensor_msgs::msg::Imu::SharedPtr> imuOptQ_;
    BlockingQueue<nav_msgs::msg::Odometry::SharedPtr> odomOptQ_;

    boost::mutex optimizedStateMutex_;
    gtsam::NavState optimizedState_;
    double optimizedTime_;
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuPredictor_;
    double imuDt_;
    gtsam::imuBias::ConstantBias optimizedBias_, previousBias_;
    sensor_msgs::msg::Imu::SharedPtr lastIMU_;
    boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams_;

    std::list<sensor_msgs::msg::Imu::SharedPtr> imuMeasurements_, imuGrav_;
    nif_msgs::msg::FilterOut initialPose_;
    gtsam::Pose3 bodyPSensor_, carENUPcarNED_;
    gtsam::Pose3 imuPgps_;

    bool fixedOrigin_;
    GeographicLib::LocalCartesian enu_;   /// Object to put lat/lon coordinates into local cartesian
    double E_, N_, U_, U0_;
    std::vector<double> ego_v_;
    bool gotFirstFix_;
    sensor_msgs::msg::NavSatFix::SharedPtr first_fix_;
    bool invertx_, inverty_, invertz_;
    bool usingOdom_;
    double maxGPSError_;

    bool received_imu_;

    bool hasNewDynamicParams_;
    double correction_x_;
    double correction_y_;

    std::string map_frame_;
    std::string body_frame_;
    bool use_msg_time_;
    bool use_imu_lpf_;
    // imu_lpf imu_lpf_;
    double imu_lpf_dt_;
    double imu_lpf_cut_f_;
    double imu_lpf_weight_;

    double imu_last_angular_velocity_x_;
    double imu_last_angular_velocity_y_;
    double imu_last_angular_velocity_z_;
    double imu_last_linear_acceleration_x_;
    double imu_last_linear_acceleration_y_;
    double imu_last_linear_acceleration_z_;

    gtsam::SharedDiagonal priorNoisePose_;
    gtsam::SharedDiagonal priorNoiseVel_;
    gtsam::SharedDiagonal priorNoiseBias_;
    gtsam::Vector noiseModelBetweenBias_sigma_;
    gtsam::ISAM2 *isam_;

    nav_msgs::msg::Odometry::SharedPtr lastOdom_;

  public:
    StateEstimator();
    ~StateEstimator();
    void InsCallback(novatel_oem7_msgs::msg::INSPVA::SharedPtr ins); // TODO: check if it has to be changed into novatel_gps_msgs::msg::Inspva
    // void InsCallback(novatel_gps_msgs::msg::Inspva::SharedPtr ins); // TODO: check if it has to be changed into novatel_gps_msgs::msg::Inspva
    void GpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr fix); // TODO: check if it has to be changed into novatel_gps_msgs::msg::Inspva
    void ImuNovatelCallback(novatel_gps_msgs::msg::NovatelRawImu::SharedPtr msg);
    void ImuCallback(sensor_msgs::msg::Imu::SharedPtr imu);
    void WheelOdomCallback(nav_msgs::msg::Odometry::SharedPtr odom);
    void GpsHelper();
    void GpsHelper_1();
    void tfBroadcast(nav_msgs::msg::Odometry &msg, std::string str = "");
    gtsam::BetweenFactor<gtsam::Pose3> integrateWheelOdom(double prevTime, double stopTime, int curFactor);
    void GetAccGyro(novatel_gps_msgs::msg::NovatelRawImu::SharedPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
    void GetAccGyro(sensor_msgs::msg::Imu::SharedPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
  };
}

#endif /* StateEstimator_H_ */

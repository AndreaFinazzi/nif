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

#include "BlockingQueue.h"

// Temporarily replaced by copy & paste
// #include <imu_3dm_gx4/msg/filter_output.hpp>
#include <nif_msgs/msg/filter_out.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>

#define PI 3.14159265358979323846264338


namespace autorally_core
{
  class StateEstimator : public rclcpp::Node
  {
  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posePub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasAccPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr biasGyroPub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr timePub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr statusPub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gpsSub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

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
    bool gotFirstFix_;
    bool invertx_, inverty_, invertz_;
    bool usingOdom_;
    double maxGPSError_;

    bool received_imu_;

    bool hasNewDynamicParams_;
    double correction_x_;
    double correction_y_;

    gtsam::SharedDiagonal priorNoisePose_;
    gtsam::SharedDiagonal priorNoiseVel_;
    gtsam::SharedDiagonal priorNoiseBias_;
    gtsam::Vector noiseModelBetweenBias_sigma_;
    gtsam::ISAM2 *isam_;

    nav_msgs::msg::Odometry::SharedPtr lastOdom_;

  public:
    StateEstimator();
    ~StateEstimator();
    void GpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr fix); // TODO: check if it has to be changed into novatel_gps_msgs::msg::Inspva
    void ImuCallback(sensor_msgs::msg::Imu::SharedPtr imu);
    void WheelOdomCallback(nav_msgs::msg::Odometry::SharedPtr odom);
    void GpsHelper();
    void GpsHelper_1();
    gtsam::BetweenFactor<gtsam::Pose3> integrateWheelOdom(double prevTime, double stopTime, int curFactor);
    void GetAccGyro(sensor_msgs::msg::Imu::SharedPtr imu, gtsam::Vector3 &acc, gtsam::Vector3 &gyro);
  };
};

#endif /* StateEstimator_H_ */

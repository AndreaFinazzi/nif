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
 * @file StateEstimator.cpp
 * @author Paul Drews <pdrews3@gatech.edu>
 * @date May 1, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief ROS node to fuse information sources and create an accurate state estimation
 *
 * @details Subscribes to the GPS, IMU, and wheel odometry topics, claculates
 * an estimate of the car's current state using GTSAM, and publishes that data.
 ***********************************************/



#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <vector>
#include "StateEstimator.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>


using namespace gtsam;
// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::G; // GPS pose


// macro for getting the time stamp of a ros message
// #define TIME(msg) ( (msg)->header.stamp.toSec() )
#define TIME(msg) ( (msg)->header.stamp.sec + (msg)->header.stamp.nanosec * 1e-9 )

namespace autorally_core
{

  StateEstimator::StateEstimator() :
    Node("state_estimator_node"),
    lastImuT_(0.0),
    lastImuTgps_(0.0),
    maxQSize_(0),
    gpsOptQ_(40),
    imuOptQ_(400),
    odomOptQ_(100),
    gotFirstFix_(false),
    received_imu_(false),
    use_msg_time_(false),
    imu_lpf_dt_(0.0083),
    imu_lpf_cut_f_(40)

  {
    // temporary variables to retrieve parameters
    double accSigma, gyroSigma, initialVelNoise, initialBiasNoiseAcc, initialBiasNoiseGyro, initialRotationNoise,
        carXAngle, carYAngle, carZAngle, sensorX, sensorY, sensorZ, sensorXAngle, sensorYAngle, sensorZAngle,
        gravityMagnitude;

    this->declare_parameter<double>("InitialRotationNoise",  0.25);
    this->declare_parameter<double>("InitialVelocityNoise",  0.1);
    this->declare_parameter<double>("InitialBiasNoiseAcc",  2e-1);
    this->declare_parameter<double>("InitialBiasNoiseGyro",  2e-2);
    this->declare_parameter<double>("AccelerometerSigma",  1.0e-2);
    this->declare_parameter<double>("GyroSigma",  8.73e-5);
    this->declare_parameter<double>("AccelBiasSigma",  3.9e-4);
    this->declare_parameter<double>("GyroBiasSigma",  4.8e-05);
    this->declare_parameter<double>("GPSSigma",  0.5);
    this->declare_parameter<double>("SensorTransformX",  0.0);
    this->declare_parameter<double>("SensorTransformY",  0.0);
    this->declare_parameter<double>("SensorTransformZ",  0.0);
    this->declare_parameter<double>("SensorXAngle",  0);
    this->declare_parameter<double>("SensorYAngle",  3.14159265359);
    this->declare_parameter<double>("SensorZAngle",  1.57079632679);
    this->declare_parameter<double>("CarXAngle",  3.14159265359);
    this->declare_parameter<double>("CarYAngle",  0);
    this->declare_parameter<double>("CarZAngle",  0);
    this->declare_parameter<double>("Gravity",  9.8);
    this->declare_parameter<bool>("InvertX", false);
    this->declare_parameter<bool>("InvertY", false);
    this->declare_parameter<bool>("InvertZ", false);
    this->declare_parameter<double>("Imudt",  1.0/200.0);
    this->get_parameter("InitialRotationNoise", initialRotationNoise);
    this->get_parameter("InitialVelocityNoise", initialVelNoise);
    this->get_parameter("InitialBiasNoiseAcc", initialBiasNoiseAcc);
    this->get_parameter("InitialBiasNoiseGyro", initialBiasNoiseGyro);
    this->get_parameter("AccelerometerSigma", accSigma);
    this->get_parameter("GyroSigma", gyroSigma);
    this->get_parameter("AccelBiasSigma", accelBiasSigma_);
    this->get_parameter("GyroBiasSigma", gyroBiasSigma_);
    this->get_parameter("GPSSigma", gpsSigma_);
    this->get_parameter("SensorTransformX", sensorX);
    this->get_parameter("SensorTransformY", sensorY);
    this->get_parameter("SensorTransformZ", sensorZ);
    this->get_parameter("SensorXAngle", sensorXAngle);
    this->get_parameter("SensorYAngle", sensorYAngle);
    this->get_parameter("SensorZAngle", sensorZAngle);
    this->get_parameter("CarXAngle", carXAngle);
    this->get_parameter("CarYAngle", carYAngle);
    this->get_parameter("CarZAngle", carZAngle);
    this->get_parameter("Gravity", gravityMagnitude);
    this->get_parameter("InvertX", invertx_);
    this->get_parameter("InvertY", inverty_);
    this->get_parameter("InvertZ", invertz_);
    this->get_parameter("Imudt", imuDt_);

    double gpsx, gpsy, gpsz;
    this->declare_parameter<double>("GPSX",  -0.37);
    this->declare_parameter<double>("GPSY",  0.0);
    this->declare_parameter<double>("GPSZ",  -0.06);
    this->get_parameter("GPSX", gpsx);
    this->get_parameter("GPSY", gpsy);
    this->get_parameter("GPSZ", gpsz);


    imuPgps_ = Pose3(Rot3(), Point3(gpsx, gpsy, gpsz));
    imuPgps_.print("IMU->GPS");

    bool fixedInitialPose;
    double initialRoll, intialPitch, initialYaw;
    this->declare_parameter<bool>("FixedInitialPose", true);
    this->declare_parameter<double>("initialRoll",  0);
    this->declare_parameter<double>("intialPitch",  0);
    this->declare_parameter<double>("initialYaw",  0);
    this->get_parameter("FixedInitialPose", fixedInitialPose);
    this->get_parameter("initialRoll", initialRoll);
    this->get_parameter("intialPitch", intialPitch);
    this->get_parameter("initialYaw", initialYaw);

    double latOrigin, lonOrigin, altOrigin;
    this->declare_parameter<bool>("FixedOrigin", true);
    this->declare_parameter<double>("latOrigin",  39.81242382259918);
    this->declare_parameter<double>("lonOrigin",  -86.3393514105259);
    this->declare_parameter<double>("altOrigin",  226.99352456443012);
    this->get_parameter("FixedOrigin", fixedOrigin_);
    this->get_parameter("latOrigin", latOrigin);
    this->get_parameter("lonOrigin", lonOrigin);
    this->get_parameter("altOrigin", altOrigin);

    this->declare_parameter<bool>("UseOdom", false);
    this->declare_parameter<double>("MaxGPSError",  10000.0);
    this->declare_parameter<double>("correction_x",  0);
    this->declare_parameter<double>("correction_y",  0);
    this->get_parameter("UseOdom", usingOdom_);
    this->get_parameter("MaxGPSError", maxGPSError_);
    this->get_parameter("correction_x", correction_x_);
    this->get_parameter("correction_y", correction_y_);

    this->declare_parameter<std::string>("map_frame",  "odom");
    this->declare_parameter<std::string>("body_frame",  "base_link");
    this->declare_parameter<bool>("use_msg_time", false);
    this->declare_parameter<double>("imu_lpf_dt",  0.0083);
    this->declare_parameter<double>("imu_lpf_cut_f",  40.0);
    this->declare_parameter<bool>("use_imu_lpf", false);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("body_frame", body_frame_);
    this->get_parameter("use_msg_time", use_msg_time_);
    this->get_parameter("imu_lpf_dt", imu_lpf_dt_);
    this->get_parameter("imu_lpf_cut_f", imu_lpf_cut_f_);
    this->get_parameter("use_imu_lpf", use_imu_lpf_);


    if (fixedOrigin_)
      enu_.Reset(latOrigin, lonOrigin, altOrigin);


    std::cout << "InitialRotationNoise " << initialRotationNoise << "\n"
    << "InitialVelocityNoise " << initialVelNoise << "\n"
    << "InitialBiasNoiseAcc " << initialBiasNoiseAcc << "\n"
    << "InitialBiasNoiseGyro " << initialBiasNoiseGyro << "\n"
    << "AccelerometerSigma " << accSigma << "\n"
    << "GyroSigma " << gyroSigma << "\n"
    << "AccelBiasSigma " << accelBiasSigma_ << "\n"
    << "GyroBiasSigma " << gyroBiasSigma_ << "\n"
    << "GPSSigma " << gpsSigma_ << "\n"
    << "SensorTransformX " << sensorX << "\n"
    << "SensorTransformY " << sensorY << "\n"
    << "SensorTransformZ " << sensorZ << "\n"
    << "SensorXAngle " <<  sensorXAngle << "\n"
    << "SensorYAngle " << sensorYAngle << "\n"
    << "SensorZAngle " <<   sensorZAngle << "\n"
    << "CarXAngle " <<  carXAngle << "\n"
    << "CarYAngle " <<  carYAngle << "\n"
    << "CarZAngle " <<  carZAngle << "\n"
    << "Gravity " <<   gravityMagnitude << "\n";

    // Use an ENU frame
    preintegrationParams_ =  PreintegrationParams::MakeSharedU(gravityMagnitude);
    preintegrationParams_->accelerometerCovariance = accSigma * I_3x3;
    preintegrationParams_->gyroscopeCovariance = gyroSigma * I_3x3;
    preintegrationParams_->integrationCovariance = 1e-5 * I_3x3;

    Vector biases((Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    optimizedBias_ = imuBias::ConstantBias(biases);
    previousBias_ = imuBias::ConstantBias(biases);
    imuPredictor_ = boost::make_shared<PreintegratedImuMeasurements>(preintegrationParams_, optimizedBias_);

    optimizedTime_ = 0;

    nif_msgs::msg::FilterOut::SharedPtr ip;
    
    if (!fixedInitialPose)
    {
      // received_imu_
      while (!ip)
      {
        RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Waiting for valid initial orientation");
        // TODO: Turn below ros::topic::waitForMessage into rclcpp::WaitSet something
        // ip = ros::topic::waitForMessage<imu_3dm_gx4::FilterOut>("filter", nh_, ros::Duration(15));
      }
      initialPose_ = *ip;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Using fixed initial orientation");
      Rot3 initialRotation = Rot3::Ypr(initialYaw, intialPitch, initialRoll);
      initialPose_.orientation.w = initialRotation.quaternion()[0];
      initialPose_.orientation.x = initialRotation.quaternion()[1];
      initialPose_.orientation.y = initialRotation.quaternion()[2];
      initialPose_.orientation.z = initialRotation.quaternion()[3];
      initialPose_.bias.x = 0;
      initialPose_.bias.y = 0;
      initialPose_.bias.z = 0;
    }

    Rot3 initRot(Quaternion(initialPose_.orientation.w, initialPose_.orientation.x, initialPose_.orientation.y,
          initialPose_.orientation.z));

    bodyPSensor_ = Pose3(Rot3::RzRyRx(sensorXAngle, sensorYAngle, sensorZAngle),
        Point3(sensorX, sensorY, sensorZ));
    carENUPcarNED_ = Pose3(Rot3::RzRyRx(carXAngle, carYAngle, carZAngle), Point3(0,0,0));

    bodyPSensor_.print("Body pose\n");
    carENUPcarNED_.print("CarBodyPose\n");

    posePub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose", rclcpp::QoS(1));
    insPub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose/ins", rclcpp::QoS(1));
    gpsPub_ = this->create_publisher<nav_msgs::msg::Odometry>("pose/gps_point", rclcpp::QoS(1));
    imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/lpf", rclcpp::QoS(1));
    biasAccPub_ = this->create_publisher<geometry_msgs::msg::Point>("bias_acc", rclcpp::QoS(1));
    biasGyroPub_ = this->create_publisher<geometry_msgs::msg::Point>("bias_gyro", rclcpp::QoS(1));
    timePub_ = this->create_publisher<geometry_msgs::msg::Point>("time_delays", rclcpp::QoS(1));
    statusPub_ = this->create_publisher<std_msgs::msg::Int16>("status", rclcpp::QoS(1));

    ISAM2Params params;
    params.factorization = ISAM2Params::QR;
    isam_ = new ISAM2(params);

    // prior on the first pose
    priorNoisePose_ = noiseModel::Diagonal::Sigmas(
         (Vector(6) << initialRotationNoise, initialRotationNoise, 3*initialRotationNoise,
             gpsSigma_, gpsSigma_, gpsSigma_).finished());

    // Add velocity prior
    priorNoiseVel_ = noiseModel::Diagonal::Sigmas(
        (Vector(3) << initialVelNoise, initialVelNoise, initialVelNoise).finished());

    // Add bias prior
    priorNoiseBias_ = noiseModel::Diagonal::Sigmas(
        (Vector(6) << initialBiasNoiseAcc,
            initialBiasNoiseAcc,
            initialBiasNoiseAcc,
            initialBiasNoiseGyro,
            initialBiasNoiseGyro,
            initialBiasNoiseGyro).finished());

    std::cout<<"checkpoint"<<std::endl;

    Vector sigma_acc_bias_c(3), sigma_gyro_bias_c(3);
    sigma_acc_bias_c << accelBiasSigma_,  accelBiasSigma_,  accelBiasSigma_;
    sigma_gyro_bias_c << gyroBiasSigma_, gyroBiasSigma_, gyroBiasSigma_;
    noiseModelBetweenBias_sigma_ = (Vector(6) << sigma_acc_bias_c, sigma_gyro_bias_c).finished();
    
    double imu_lpf_cut_f_radps = imu_lpf_cut_f_ * 180.0 / PI;
    imu_lpf_weight_ = 2.0 * imu_lpf_cut_f_radps / (3.0*imu_lpf_cut_f_radps + 2.0 / imu_lpf_dt_);

    insSub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "ins", rclcpp::QoS(300), std::bind(&StateEstimator::InsCallback, this, std::placeholders::_1));
    gpsSub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "gps", rclcpp::QoS(300), std::bind(&StateEstimator::GpsCallback, this, std::placeholders::_1));
    imuNovatelSub_ = this->create_subscription<novatel_gps_msgs::msg::NovatelRawImu>(
      "imu_novatel", rclcpp::QoS(600), std::bind(&StateEstimator::ImuNovatelCallback, this, std::placeholders::_1));
    imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::QoS(600), std::bind(&StateEstimator::ImuCallback, this, std::placeholders::_1));
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "wheel_odom", rclcpp::QoS(300), std::bind(&StateEstimator::WheelOdomCallback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    boost::thread optimizer(&StateEstimator::GpsHelper,this);

  }

  StateEstimator::~StateEstimator()
  {}

  void StateEstimator::GpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr fix)
  {
    if(!use_msg_time_)
    {
      fix->header.stamp = this->get_clock()->now(); // comment this line for exact calculation. (1/3)
    }
    if(!gotFirstFix_)
    {
      first_fix_ = fix;
    }
    else
    {
      fix->altitude = first_fix_->altitude;
    }
    if (!gpsOptQ_.pushNonBlocking(fix))
      RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Dropping a GPS measurement due to full queue!!");

    double E, N, U;
    enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = fix->header.stamp;
    odom.header.frame_id = map_frame_.c_str();
    odom.child_frame_id = body_frame_.c_str();
    odom.pose.pose.position.x = E;
    odom.pose.pose.position.y = N;
    odom.pose.pose.position.z = U;

    gpsPub_->publish(odom);
    tfBroadcast(odom, "/gps");
  }

  void StateEstimator::InsCallback(novatel_oem7_msgs::msg::INSPVA::SharedPtr ins)
  {
    if(gotFirstFix_)
    {
      ins->height = first_fix_->altitude;
    }

    double E, N, U;
    enu_.Forward(ins->latitude, ins->longitude, ins->height, E, N, U);
    // https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATT.html
    // Left-handed rotation around z-axis in degrees clockwise from North.
    double heading = 90 - ins->azimuth;
    heading = heading * PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, heading);

    double ego_vx = ins->east_velocity * cos(heading) + ins->north_velocity * sin(heading);
    double ego_vy = -ins->east_velocity * sin(heading) + ins->north_velocity * cos(heading);
    double ego_vz = ins->up_velocity;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = map_frame_.c_str();
    odom.child_frame_id = body_frame_.c_str();
    odom.pose.pose.position.x = E;
    odom.pose.pose.position.y = N;
    odom.pose.pose.position.z = U;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = ego_vx;
    odom.twist.twist.linear.y = ego_vy;
    odom.twist.twist.linear.z = ego_vz;

    insPub_->publish(odom);
    tfBroadcast(odom, "/ins");

  }

  void StateEstimator::GetAccGyro(sensor_msgs::msg::Imu::SharedPtr imu, Vector3 &acc, Vector3 &gyro)
  {
    double accx, accy, accz;
    if (invertx_) accx = -imu->linear_acceleration.x;
    else accx = imu->linear_acceleration.x;
    if (inverty_) accy = -imu->linear_acceleration.y;
    else accy = imu->linear_acceleration.y;
    if (invertz_) accz = -imu->linear_acceleration.z;
    else accz = imu->linear_acceleration.z;
    acc = Vector3(accx, accy, accz);

    double gx, gy, gz;
    if (invertx_) gx = -imu->angular_velocity.x;
    else gx = imu->angular_velocity.x;
    if (inverty_) gy = -imu->angular_velocity.y;
    else gy = imu->angular_velocity.y;
    if (invertz_) gz = -imu->angular_velocity.z;
    else gz = imu->angular_velocity.z;

    gyro = Vector3(gx, gy, gz);
  }


  void StateEstimator::GpsHelper()
  {
    rclcpp::Rate loop_rate(10);
    bool gotFirstFix = false;
    double startTime;
    int odomKey = 1;
    int imuKey = 1;
    int latestGPSKey = 0;
    imuBias::ConstantBias prevBias;
    Vector3 prevVel = (Vector(3) << 0.0,0.0,0.0).finished();
    Pose3 prevPose;
    int status = 0;


    while (rclcpp::ok())
    {
      bool optimize = false;

      if (!gotFirstFix)
      {
        sensor_msgs::msg::NavSatFix::SharedPtr fix = gpsOptQ_.popBlocking();
        startTime = TIME(fix);
        if(usingOdom_) {
          lastOdom_ = odomOptQ_.popBlocking();
        }

        NonlinearFactorGraph newFactors;
        Values newVariables;
        gotFirstFix = true;
        gotFirstFix_ = gotFirstFix;

        double E, N, U;
        if (!fixedOrigin_)
        {
          // enu_.Reset(fix->latitude, fix->longitude, fix->height);
          enu_.Reset(fix->latitude, fix->longitude, fix->altitude);
          E = 0; N = 0; U = 0; // we're choosing this as the origin
        }
        else
        {
          // we are given an origin
          // enu_.Forward(fix->latitude, fix->longitude, fix->height, E, N, U);
          enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);
        }
        E_ = E;
        N_ = N;
        U_ = U;

        // Add prior factors on pose, vel and bias
        Rot3 initialOrientation = Rot3::Quaternion(initialPose_.orientation.w,
            initialPose_.orientation.x,
            initialPose_.orientation.y,
            initialPose_.orientation.z);
        std::cout << "Initial orientation" << std::endl;
        std::cout << bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation() << std::endl;
        Pose3 x0(bodyPSensor_.rotation() * initialOrientation * carENUPcarNED_.rotation(),
            Point3(E, N, U));
        prevPose = x0;
        PriorFactor<Pose3> priorPose(X(0), x0, priorNoisePose_);
        newFactors.add(priorPose);
        PriorFactor<Vector3> priorVel(V(0), Vector3(0, 0, 0), priorNoiseVel_);
        newFactors.add(priorVel);
        Vector biases((Vector(6) << 0, 0, 0, initialPose_.bias.x,
            -initialPose_.bias.y, -initialPose_.bias.z).finished());
        prevBias = imuBias::ConstantBias(biases);
        PriorFactor<imuBias::ConstantBias> priorBias(B(0), imuBias::ConstantBias(biases), priorNoiseBias_);
        newFactors.add(priorBias);

        //Factor for imu->gps translation
        BetweenFactor<Pose3> imuPgpsFactor(X(0), G(0), imuPgps_,
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
        newFactors.add(imuPgpsFactor);

        // add prior values on pose, vel and bias
        newVariables.insert(X(0), x0);
        newVariables.insert(V(0), Vector3(0, 0, 0));
        newVariables.insert(B(0), imuBias::ConstantBias(biases));
        newVariables.insert(G(0), x0.compose(imuPgps_));

        isam_->update(newFactors, newVariables);
        //Read IMU measurements up to the first GPS measurement
        lastIMU_ = imuOptQ_.popBlocking();
        //If we only pop one, we need some dt
        lastImuTgps_ = TIME(lastIMU_) - 0.005;
        while(TIME(lastIMU_) < TIME(fix))
        {
          lastImuTgps_ = TIME(lastIMU_);
          lastIMU_ = imuOptQ_.popBlocking();
        }
        loop_rate.sleep();
      }
      else
      {
        NonlinearFactorGraph newFactors;
        Values newVariables;


        // add IMU measurements
        while (imuOptQ_.size() > 0 && (TIME(imuOptQ_.back()) > (startTime + imuKey * 0.1)))
        {
          double curTime = startTime + imuKey * 0.1;
          PreintegratedImuMeasurements pre_int_data(preintegrationParams_, previousBias_);
          while(TIME(lastIMU_) < curTime)
          {
            Vector3 acc, gyro;
            GetAccGyro(lastIMU_, acc, gyro);
            double imuDT = TIME(lastIMU_) - lastImuTgps_;
            lastImuTgps_ = TIME(lastIMU_);
            pre_int_data.integrateMeasurement(acc, gyro, imuDT);
            lastIMU_ = imuOptQ_.popBlocking();
          }
          // adding the integrated IMU measurements to the factor graph
          ImuFactor imuFactor(X(imuKey-1), V(imuKey-1), X(imuKey), V(imuKey), B(imuKey-1), pre_int_data);
          newFactors.add(imuFactor);
          newFactors.add(BetweenFactor<imuBias::ConstantBias>(B(imuKey-1), B(imuKey), imuBias::ConstantBias(),
              noiseModel::Diagonal::Sigmas( sqrt(pre_int_data.deltaTij()) * noiseModelBetweenBias_sigma_)));

          // Predict forward to get an initial estimate for the pose and velocity
          NavState curNavState(prevPose, prevVel);
          NavState nextNavState = pre_int_data.predict(curNavState, prevBias);
          newVariables.insert(X(imuKey), nextNavState.pose());
          newVariables.insert(V(imuKey), nextNavState.v());
          newVariables.insert(B(imuKey), previousBias_);
          newVariables.insert(G(imuKey), nextNavState.pose().compose(imuPgps_));
          prevPose = nextNavState.pose();
          prevVel = nextNavState.v();
          ++imuKey;
          optimize = true;
        }


        // add GPS measurements that are not ahead of the imu messages
        while (optimize && gpsOptQ_.size() > 0 && TIME(gpsOptQ_.front()) < (startTime + (imuKey-1)*0.1 + 1e-2))
        {
          sensor_msgs::msg::NavSatFix::SharedPtr fix = gpsOptQ_.popBlocking();
          double timeDiff = (TIME(fix) - startTime) / 0.1;
          int key = round(timeDiff);
          if (std::abs(timeDiff - key) < 1e-1)
          {
            // this is a gps message for a factor
            latestGPSKey = key;
            double E,N,U,U0;
            enu_.Forward(fix->latitude, fix->longitude, fix->altitude, E, N, U);

            // check if the GPS message is close to our expected position
            Pose3 expectedState;
            if (newVariables.exists(X(key)))
              expectedState = (Pose3) newVariables.at<Pose3>(X(key));
            else
              expectedState = isam_->calculateEstimate<Pose3>(X(key));

            double dist = std::sqrt( std::pow(expectedState.x() - E, 2) + std::pow(expectedState.y() - N, 2) );
            if (dist < maxGPSError_ || latestGPSKey < imuKey-2)
            {
              SharedDiagonal gpsNoise = noiseModel::Diagonal::Sigmas(Vector3(gpsSigma_, gpsSigma_, 3.0 * gpsSigma_));
              GPSFactor gpsFactor(G(key), Point3(E, N, U), gpsNoise);
              newFactors.add(gpsFactor);
              BetweenFactor<Pose3> imuPgpsFactor(X(key), G(key), imuPgps_,
                  noiseModel::Diagonal::Sigmas((Vector(6) << 0.001,0.001,0.001,0.03,0.03,0.03).finished()));
              newFactors.add(imuPgpsFactor);

              if (!usingOdom_)
                odomKey = key+1;
            }
            else
            {
              RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Received bad GPS message");
              // diag_warn("Received bad GPS message");
            }
          }
        }


        // if only using odom with no GPS, then remove old messages from queue
        while (!usingOdom_ && odomOptQ_.size() > 0 && TIME(odomOptQ_.front()) < (odomKey*0.1 + startTime))
          lastOdom_ = odomOptQ_.popBlocking();

        // if available, add any odom factors that are not ahead of the imu messages
        while ((usingOdom_ && latestGPSKey < imuKey-2) && optimize && odomKey < imuKey && odomOptQ_.size() > 0
            && (TIME(odomOptQ_.back()) > (startTime + odomKey * 0.1)))
        {
          double prevTime = startTime + (odomKey-1) * 0.1;
          newFactors.add(integrateWheelOdom(prevTime, prevTime+0.1, odomKey++));
        }


        // if we processed imu - then we can optimize the state
        if (optimize)
        {
          try
          {
            isam_->update(newFactors, newVariables);
            Pose3 nextState = isam_->calculateEstimate<Pose3>(X(imuKey-1));

            prevPose = nextState;
            prevVel = isam_->calculateEstimate<Vector3>(V(imuKey-1));
            prevBias = isam_->calculateEstimate<imuBias::ConstantBias>(B(imuKey-1));

            // if we haven't added gps data for 2 message (0.2s) then change status
            if (latestGPSKey + 3 < imuKey)
            {
              status = 1;
              // diag_warn("No gps");
            }
            else
            {
              status = 0;
              // diag_ok("Still ok!");
            }

            double curTime = startTime + (imuKey-1) * 0.1;

            {
              boost::mutex::scoped_lock guard(optimizedStateMutex_);
              optimizedState_ = NavState(prevPose, prevVel);
              optimizedBias_ = prevBias;
              optimizedTime_ = curTime;
              status_ = status;
            }

            nav_msgs::msg::Odometry poseNew;
            poseNew.header.stamp = rclcpp::Time(curTime);

            geometry_msgs::msg::Point ptAcc;
            ptAcc.x = prevBias.vector()[0];
            ptAcc.y = prevBias.vector()[1];
            ptAcc.z = prevBias.vector()[2];

            geometry_msgs::msg::Point ptGyro;
            ptGyro.x = prevBias.vector()[3];
            ptGyro.y = prevBias.vector()[4];
            ptGyro.z = prevBias.vector()[5];

            biasAccPub_->publish(ptAcc);
            biasGyroPub_->publish(ptGyro);
          }
          catch(gtsam::IndeterminantLinearSystemException ex)
          {
            RCLCPP_ERROR(rclcpp::get_logger("state_estimator"), "Encountered Indeterminant System Error!");
            // ROS_ERROR("Encountered Indeterminant System Error!");
            // diag_error("State estimator has encountered indeterminant system error");
            status = 2;
            {
              boost::mutex::scoped_lock guard(optimizedStateMutex_);
              status_ = status;
            }
          }
        }
        loop_rate.sleep();
      }
    }
  }

  void StateEstimator::ImuNovatelCallback(novatel_gps_msgs::msg::NovatelRawImu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = msg->header.stamp;
    imu.linear_acceleration.x = msg->linear_acceleration.x;
    imu.linear_acceleration.y = msg->linear_acceleration.y;
    imu.linear_acceleration.z = msg->linear_acceleration.z;
    imu.angular_velocity.x = msg->angular_velocity.x;
    imu.angular_velocity.y = msg->angular_velocity.y;
    imu.angular_velocity.z = msg->angular_velocity.z;
    auto imu_ptr = std::make_shared<sensor_msgs::msg::Imu>(imu);
    ImuCallback(imu_ptr);
  }


  void StateEstimator::ImuCallback(sensor_msgs::msg::Imu::SharedPtr imu)
  {
    received_imu_ = true;
    if(!use_msg_time_)
    {
      imu->header.stamp = this->get_clock()->now(); // comment this line for exact calculation. (2/3)
    }
    if(use_imu_lpf_)
    {
      imu->angular_velocity.x = imu->angular_velocity.x * imu_lpf_weight_ + imu_last_angular_velocity_x_ * (1-imu_lpf_weight_);
      imu->angular_velocity.y = imu->angular_velocity.y * imu_lpf_weight_ + imu_last_angular_velocity_y_ * (1-imu_lpf_weight_);
      // imu->angular_velocity.z = imu->angular_velocity.z * imu_lpf_weight_ + imu_last_angular_velocity_z_ * (1-imu_lpf_weight_);
      imu->linear_acceleration.x = imu->linear_acceleration.x * imu_lpf_weight_ + imu_last_linear_acceleration_x_ * (1-imu_lpf_weight_);
      imu->linear_acceleration.y = imu->linear_acceleration.y * imu_lpf_weight_ + imu_last_linear_acceleration_y_ * (1-imu_lpf_weight_);
      imu->linear_acceleration.z = imu->linear_acceleration.z * imu_lpf_weight_ + imu_last_linear_acceleration_z_ * (1-imu_lpf_weight_);
      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header = imu->header;
      imu_msg.angular_velocity = imu->angular_velocity;
      imu_msg.linear_acceleration = imu->linear_acceleration;
      imuPub_->publish(imu_msg);
    }
    double dt;
    if (lastImuT_ == 0) dt = 0.005;
    else dt = TIME(imu) - lastImuT_;

    lastImuT_ = TIME(imu);
    //ros::Time before = ros::Time::now();

    // Push the IMU measurement to the optimization thread
    int qSize = imuOptQ_.size();
    if (qSize > maxQSize_)
      maxQSize_ = qSize;
    if (!imuOptQ_.pushNonBlocking(imu))
      RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Dropping an IMU measurement due to full queue!!");

    // Each time we get an imu measurement, calculate the incremental pose from the last GTSAM pose
    imuMeasurements_.push_back(imu);
    //Grab the most current optimized state
    double optimizedTime;
    NavState optimizedState;
    imuBias::ConstantBias optimizedBias;
    int status;
    {
      boost::mutex::scoped_lock guard(optimizedStateMutex_);
      optimizedState = optimizedState_;
      optimizedBias = optimizedBias_;
      optimizedTime = optimizedTime_;
      status = status_;
    }
    if (optimizedTime == 0) return; // haven't optimized first state yet

    bool newMeasurements = false;
    int numImuDiscarded = 0;
    double imuQPrevTime;
    Vector3 acc, gyro;
    while (!imuMeasurements_.empty() && (TIME(imuMeasurements_.front()) < optimizedTime))
    {
      imuQPrevTime = TIME(imuMeasurements_.front());
      imuMeasurements_.pop_front();
      newMeasurements = true;
      numImuDiscarded++;
    }

    if(newMeasurements)
    {
      //We need to reset integration and iterate through all our IMU measurements
      imuPredictor_->resetIntegration();
      int numMeasurements = 0;
      for (auto it=imuMeasurements_.begin(); it!=imuMeasurements_.end(); ++it)
      {
        double dt_temp =  TIME(*it) - imuQPrevTime;
        imuQPrevTime = TIME(*it);
        GetAccGyro(*it, acc, gyro);
        imuPredictor_->integrateMeasurement(acc, gyro, dt_temp);
        numMeasurements++;
        // ROS_INFO("IMU time %f, dt %f", (*it)->header.stamp.toSec(), dt_temp);
      }
      // ROS_INFO("Resetting Integration, %d measurements integrated, %d discarded", numMeasurements, numImuDiscarded);
    }
    else
    {
      //Just need to add the newest measurement, no new optimized pose
      GetAccGyro(imu, acc, gyro);
      imuPredictor_->integrateMeasurement(acc, gyro, dt);
      // ROS_INFO("Integrating %f, dt %f", m_lastImuT, dt);
    }

    // predict next state given the imu measurements
    NavState currentPose = imuPredictor_->predict(optimizedState, optimizedBias);
    nav_msgs::msg::Odometry poseNew;
    poseNew.header.stamp = imu->header.stamp;

    Vector4 q = currentPose.quaternion().coeffs();
    poseNew.pose.pose.orientation.x = q[0];
    poseNew.pose.pose.orientation.y = q[1];
    poseNew.pose.pose.orientation.z = q[2];
    poseNew.pose.pose.orientation.w = q[3];

    poseNew.pose.pose.position.x = currentPose.position().x() + correction_x_;
    poseNew.pose.pose.position.y = currentPose.position().y() + correction_y_;
    poseNew.pose.pose.position.z = currentPose.position().z();

    poseNew.twist.twist.linear.x = currentPose.velocity().x();
    poseNew.twist.twist.linear.y = currentPose.velocity().y();
    poseNew.twist.twist.linear.z = currentPose.velocity().z();
    
    poseNew.twist.twist.angular.x = gyro.x() + optimizedBias.gyroscope().x();
    poseNew.twist.twist.angular.y = gyro.y() + optimizedBias.gyroscope().y();
    poseNew.twist.twist.angular.z = gyro.z() + optimizedBias.gyroscope().z();

    poseNew.child_frame_id = body_frame_.c_str();
    poseNew.header.frame_id = map_frame_.c_str();

    posePub_->publish(poseNew);
    tfBroadcast(poseNew);

    //ros::Time after = ros::Time::now();
    geometry_msgs::msg::Point delays;
    auto current_time = this->get_clock()->now();
    delays.x = TIME(imu);
    delays.y = (this->get_clock()->now() - imu->header.stamp).nanoseconds() * 10e-9;//current_time.nanoseconds() * 10e-9 - TIME(imu);
    delays.z = TIME(imu) - optimizedTime;
    timePub_->publish(delays);

    // publish the status of the estimate - set in the gpsHelper thread
    std_msgs::msg::Int16 statusMsgs;
    // statusMsgs.header.stamp = imu->header.stamp;
    // statusMsgs.status = status;
    statusMsgs.data = status;
    statusPub_->publish(statusMsgs);
    return;
  }

  void StateEstimator::WheelOdomCallback(nav_msgs::msg::Odometry::SharedPtr odom)
  {
    if(!use_msg_time_)
    {
      odom->header.stamp = this->get_clock()->now(); // comment this line for exact calculation. (3/3)
    }
      if (!odomOptQ_.pushNonBlocking(odom))
        RCLCPP_WARN(rclcpp::get_logger("state_estimator"), "Dropping an wheel odometry measurement due to full queue!!");
  }


  BetweenFactor<Pose3> StateEstimator::integrateWheelOdom(double prevTime, double stopTime, int curKey)
  {
    double x=0, y=0, theta=0, xVar=0, yVar=0, zVar=0, thetaVariance=0, dt=0, lastTimeUsed=prevTime;

    while (lastTimeUsed != stopTime)
    {
      if (odomOptQ_.size() != 0 && TIME(odomOptQ_.front()) < stopTime)
      {
        lastOdom_ = odomOptQ_.popBlocking();
        dt = TIME(lastOdom_) - lastTimeUsed;
        lastTimeUsed = TIME(lastOdom_);
      }
      else
      {
        dt = stopTime - lastTimeUsed;
        lastTimeUsed = stopTime;
      }

      // the local frame velocities
      double vx = lastOdom_->twist.twist.linear.x;
      double vy = lastOdom_->twist.twist.linear.y;
      // update the relative position from the initial
      x += vx*dt*cos(theta) - vy*dt*sin(theta);
      y += vx*dt*sin(theta) + vy*dt*cos(theta);
      theta += dt*lastOdom_->twist.twist.angular.z;
      xVar += dt * lastOdom_->twist.covariance[0];
      yVar += dt * lastOdom_->twist.covariance[7];
      zVar += dt * lastOdom_->twist.covariance[14];
      thetaVariance += dt*lastOdom_->twist.covariance[35];
    }

    Pose3 betweenPose = Pose3(Rot3::Rz(theta), Point3(x, y, 0.0));
    return BetweenFactor<Pose3>(X(curKey-1), X(curKey), betweenPose, noiseModel::Diagonal::Sigmas(
          (Vector(6) << thetaVariance*2,thetaVariance*2,thetaVariance,xVar,yVar,zVar).finished()));
  }

  void StateEstimator::tfBroadcast(nav_msgs::msg::Odometry &msg, std::string str)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    
    transformStamped.header.stamp = msg.header.stamp;
    transformStamped.header.frame_id = map_frame_.c_str();
    std::string body_frame;
    body_frame.append(body_frame_);
    body_frame.append(str);
    transformStamped.child_frame_id = body_frame.c_str();
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;

    const geometry_msgs::msg::TransformStamped transformStampedConst = transformStamped;
    tf_broadcaster_->sendTransform(transformStampedConst);
  }


};

int main (int argc, char** argv)
{
  // ros::init(argc, argv, "StateEstimator");
  // //ros::NodeHandle n;
  // autorally_core::StateEstimator wpt;
  // ros::spin();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<autorally_core::StateEstimator>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by USRG on 9/14/21.
//

#include "ekf_localizer/nif_aw_localization_node.h"
#include "nif_frame_id/frame_id.h"

#include <random>

using namespace message_filters;
using namespace std::placeholders;

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
// #define DEBUG_INFO(...) { if (show_debug_info_) { RCLCPP_INFO(this->get_logger(),  _VA_ARGS__); } }
#define DEBUG_PRINT_MAT(X) { if (show_debug_info_) { std::cout << #X << ": " << X << std::endl; } }

// clang-format on
AWLocalizationNode::AWLocalizationNode(const std::string &node_name)
    // : IBaseNode(node_name), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
    : IBaseNode(node_name), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)

{
  this->declare_parameter<bool>("use_inspva_heading", bool(true));
  this->declare_parameter<double>("bestvel_heading_update_velocity_thres", double(2.));
  m_origin_lat = this->get_global_parameter<double>("coordinates.ecef_ref_lat");
  m_origin_lon = this->get_global_parameter<double>("coordinates.ecef_ref_lon");

  this->declare_parameter<bool>("show_debug_info", bool(false));
  this->declare_parameter<double>("predict_frequency", double(50.0));
  this->declare_parameter<double>("tf_rate", double(10.0));
  this->declare_parameter<bool>("enable_yaw_bias_estimation", bool(true));
  this->declare_parameter<int>("extend_state_step", int(50));
  this->declare_parameter<std::string>("pose_frame_id", std::string("odom"));

  /* pose measurement */
  this->declare_parameter<double>("pose_additional_delay", double(0.0));
  this->declare_parameter<double>("pose_measure_uncertainty_time", double(0.01));
  this->declare_parameter<double>("pose_rate", double(20.0));              //  used for covariance calculation
  this->declare_parameter<double>("pose_gate_dist", double(30.0)); //  Mahalanobis limit
  this->declare_parameter<double>("pose_stddev_x", double(0.05));
  this->declare_parameter<double>("pose_stddev_y", double(0.05));
  this->declare_parameter<double>("pose_stddev_yaw", double(0.035));
  this->declare_parameter<bool>("use_pose_with_covariance", bool(false));

  /* twist measurement */
  this->declare_parameter<double>("twist_additional_delay", double(0.0));
  this->declare_parameter<double>("twist_rate", double(100.0));              //  used for covariance calculation
  this->declare_parameter<double>("twist_gate_dist", double(10000.0)); //  Mahalanobis limit
  this->declare_parameter<double>("twist_stddev_vx", double(0.2));
  this->declare_parameter<double>("twist_stddev_wz", double(0.03));
  this->declare_parameter<bool>("use_twist_with_covariance", bool(false));

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_yaw_bias_c, proc_stddev_vx_c, proc_stddev_wz_c;
  this->declare_parameter<double>("proc_stddev_yaw_c", double(0.005));
  this->declare_parameter<double>("proc_stddev_yaw_bias_c", double(0.001));
  this->declare_parameter<double>("proc_stddev_vx_c", double(2.0));
  this->declare_parameter<double>("proc_stddev_wz_c", double(0.2));

  this->declare_parameter<double>("top_to_bottom_bias_x", double(0.0));
  this->declare_parameter<double>("top_to_bottom_bias_y", double(0.0));
  this->declare_parameter<bool>("use_bestvel_for_speed", bool(false));

  this->m_use_inspva_heading =
      this->get_parameter("use_inspva_heading").as_bool();
  this->get_parameter("bestvel_heading_update_velocity_thres").as_double();
  this->show_debug_info_ = this->get_parameter("show_debug_info").as_bool();
  this->ekf_rate_ = this->get_parameter("predict_frequency").as_double();
  this->tf_rate_ = this->get_parameter("tf_rate").as_double();
  this->enable_yaw_bias_estimation_ = this->get_parameter("enable_yaw_bias_estimation").as_bool();
  this->extend_state_step_ = this->get_parameter("extend_state_step").as_int();

  /* pose measurement */
  this->pose_additional_delay_ = this->get_parameter("pose_additional_delay").as_double();
  this->pose_measure_uncertainty_time_ = this->get_parameter("pose_measure_uncertainty_time").as_double();
  this->pose_rate_ = this->get_parameter("pose_rate").as_double();              //  used for covariance calculation
  this->pose_gate_dist_ = this->get_parameter("pose_gate_dist").as_double(); //  Mahalanobis limit
  this->pose_stddev_x_ = this->get_parameter("pose_stddev_x").as_double();
  this->pose_stddev_y_ = this->get_parameter("pose_stddev_y").as_double();
  this->pose_stddev_yaw_ = this->get_parameter("pose_stddev_yaw").as_double();
  this->use_pose_with_covariance_ = this->get_parameter("use_pose_with_covariance").as_bool();

  /* twist measurement */
  this->twist_additional_delay_ = this->get_parameter("twist_additional_delay").as_double();
  this->twist_rate_ = this->get_parameter("twist_rate").as_double();              //  used for covariance calculation
  this->twist_gate_dist_ = this->get_parameter("twist_gate_dist").as_double(); //  Mahalanobis limit
  this->twist_stddev_vx_ = this->get_parameter("twist_stddev_vx").as_double();
  this->twist_stddev_wz_ = this->get_parameter("twist_stddev_wz").as_double();
  this->use_twist_with_covariance_ = this->get_parameter("use_twist_with_covariance").as_bool();

  /* process noise */
  proc_stddev_yaw_c = this->get_parameter("proc_stddev_yaw_c").as_double();
  proc_stddev_yaw_bias_c = this->get_parameter("proc_stddev_yaw_bias_c").as_double();
  proc_stddev_vx_c = this->get_parameter("proc_stddev_vx_c").as_double();
  proc_stddev_wz_c = this->get_parameter("proc_stddev_wz_c").as_double();

  this->top_to_bottom_bias_x = this->get_parameter("top_to_bottom_bias_x").as_double();
  this->top_to_bottom_bias_y = this->get_parameter("top_to_bottom_bias_y").as_double();
  this->bUseBestVelForSpeed =
      this->get_parameter("use_bestvel_for_speed").as_bool();

  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  if (!enable_yaw_bias_estimation_) {
    proc_stddev_yaw_bias_c = 0.0;
  }
  RCLCPP_INFO(this->get_logger(), "ORIGIN LATITUDE : %f", this->m_origin_lat);
  RCLCPP_INFO(this->get_logger(), "ORIGIN LONGITUDE : %f", this->m_origin_lon);

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c, 2.0) * ekf_dt_;
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c, 2.0) * ekf_dt_;
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c, 2.0) * ekf_dt_;
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c, 2.0) * ekf_dt_;

  /* initialize ros system */
  using namespace std::chrono_literals; // NOLINT
  timer_control_ = this->create_wall_timer(
      10ms, std::bind(&AWLocalizationNode::timerCallback, this));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_ekf_estimated", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_yaw_bias_ = this->create_publisher<std_msgs::msg::Float64>("estimated_yaw_bias", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_bestpos_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_bestpos", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_top_bestpos_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_top_odometry_bestpos", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_mahalanobisScore = this->create_publisher<std_msgs::msg::Float64>(
      "out_localization_error", nif::common::constants::QOS_EGO_ODOMETRY);

  pub_localization_status = this->create_publisher<nif_msgs::msg::LocalizationStatus>(
      "out_localization_status", nif::common::constants::QOS_INTERNAL_STATUS);

  // POSE(X, Y)
  subBESTPOS = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_bestpos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::BESTPOSCallback, this, std::placeholders::_1));
  subTOPBESTPOS = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_top_bestpos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPBESTPOSCallback, this,
                std::placeholders::_1));

  // NOT USED
  subINSPVA = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "in_inspva", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::BOTTOMINSPVACallback, this,
                std::placeholders::_1));
  // HEADING
  subTOPINSPVA = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "in_top_inspva", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPINSPVACallback, this, std::placeholders::_1));
  // HEADING BACK-UP SOLUTION
  subBESTVEL = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
      "in_bestvel", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::BESTVELCallback, this, std::placeholders::_1));
  subINSSTDEV = this->create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
      "in_insstdev", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::INSSTDEVCallback, this,
                std::placeholders::_1));
  subTOPINSSTDEV = this->create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
      "in_top_insstdev", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::TOPINSSTDEVCallback, this,
                std::placeholders::_1));
  subIMUONLY = this->create_subscription<sensor_msgs::msg::Imu>(
      "in_imu", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::IMUCallback, this,
                std::placeholders::_1)); //use this when putting bestvel speed as vehicle velocity.
  subtest = this->create_subscription<std_msgs::msg::Float64>(
      "test_noise", 1,
      std::bind(&AWLocalizationNode::TestCallback, this,
                std::placeholders::_1)); 
  auto rmw_qos_profile =
      nif::common::constants::QOS_SENSOR_DATA.get_rmw_qos_profile();

  sub_filtered_IMU.subscribe(this, "in_imu", rmw_qos_profile);
  sub_filtered_Wheel.subscribe(this, "in_wheel_speed_report", rmw_qos_profile);

  m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_filtered_IMU, sub_filtered_Wheel);

  m_sync->registerCallback(std::bind(&AWLocalizationNode::MessegefilteringCallback,
                                     this, std::placeholders::_1,
                                     std::placeholders::_2));

  dim_x_ex_ = dim_x_ * extend_state_step_;
 //  twist_linear_x = 0;

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  initEKF();

  /* debug */
  pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("debug", 1);
  pub_measured_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);

  // Set the ltp reference point
  nif::localization::utils::GeodeticConverter::GeoRef ref;
  ref.latitude = m_origin_lat;
  ref.longitude = m_origin_lon;
  ref.altitude = 0.;
  conv_.initializeReference(ref);

  gps_timeout = rclcpp::Duration(1, 0);

  // this->setNodeStatus(nif::common::NODE_INITIALIZED);
};

AWLocalizationNode::~AWLocalizationNode(){};

/*
 * timerCallback
 */
void AWLocalizationNode::timerCallback()
{
  if (bImuFirstCall && (bBOTTOM_GPS || bTOP_GPS) && (bBOTTOMGPSHeading || bTOPGPSHeading)) {
    auto node_status = nif::common::NODE_ERROR;

    m_localization_status.stamp = this->now();

    // If the geofence data is too old, report error, but keep going.
    if ((this->now().nanoseconds() - bestpos_time_last_update.nanoseconds()) >=
            this->gps_timeout.nanoseconds() ||
        (this->now().nanoseconds() - imu_time_last_update.nanoseconds()) >=
            this->gps_timeout.nanoseconds()) {
      // Set error, but keep going
      node_status = nif::common::NODE_ERROR;
    } else {
      node_status = nif::common::NODE_OK;
    }

    predictKinematicsModel();


    VehPose_t BestCorrection;
    VehPose_t CurrentEstimated;
    CurrentEstimated.x = ekf_.getXelement(IDX::X);
    CurrentEstimated.y = ekf_.getXelement(IDX::Y);
    CurrentEstimated.yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);

    double veh_vel_x, veh_yaw_rate, correction_x, correction_y, correction_yaw;

    veh_vel_x = m_dVelolcity_X;
    veh_yaw_rate = m_dIMU_yaw_rate;
    if(bTOPGPSHeading && !bBOTTOMGPSHeading)
    {
      correction_yaw = m_dGPS_TOP_Heading;
    }
    if (bBOTTOMGPSHeading)
    {
      correction_yaw = m_dGPS_Heading;
    }

    // standard deviation from novatel bottom/top
    double bottom_noise_total =
        sqrt(BestPosBottom.lat_noise * BestPosBottom.lat_noise +
             BestPosBottom.lon_noise * BestPosBottom.lon_noise);
    double top_noise_total = sqrt(BestPosTop.lat_noise * BestPosTop.lat_noise +
                                  BestPosTop.lon_noise * BestPosTop.lon_noise);

    /**
     * @brief select best correction data
     */
    bool update_pose = false;
    if (bBOTTOM_GPS && bTOP_GPS)
    {
      update_pose = CalculateBestCorrection(BestPosBottom, BestPosTop,
                                            CurrentEstimated, BestCorrection);
      correction_x = BestCorrection.x; 
      correction_y = BestCorrection.y;
      m_localization_status.top_initialized = true;
      m_localization_status.bottom_initialized = true;

    } else if ((bBOTTOM_GPS && !bTOP_GPS) ||
               (bottom_noise_total < 2.0 && top_noise_total > 2.0)) {
      correction_x = BestPosBottom.x; 
      correction_y = BestPosBottom.y;
      update_pose = true;
      m_localization_status.status = "TOP IS NOT STARTED. ONLY BOTTOM";
      m_localization_status.top_initialized = false;
      m_localization_status.bottom_initialized = true;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM;

      Eigen::MatrixXd P_curr, P_2by2;
      ekf_.getLatestP(P_curr);
      P_2by2 = P_curr.block(0, 0, 2, 2);

      Eigen::MatrixXd y_est(2, 1);
      y_est << CurrentEstimated.x, CurrentEstimated.y;

      Eigen::MatrixXd y_bottom(2, 1);
      y_bottom << BestPosBottom.x, BestPosBottom.y;
      double bottomError = GetmahalanobisDistance(y_est, y_bottom, P_2by2);

      double bottom_yaw_error =
          fabs(BestPosBottom.yaw -
               (ekf_.getXelement(IDX::YAW) +
                ekf_.getXelement(IDX::YAWB))); // correction - ekf

      if (bottom_yaw_error > 2 * M_PI)
        bottom_yaw_error = bottom_yaw_error - 2 * M_PI;

      m_localization_status.bottom_error = bottomError;

      if (bottomError < 5.0 && bottom_yaw_error < 0.1) {
        bInitConverged = true;
      }

    } else if ((!bBOTTOM_GPS && bTOP_GPS) ||
               (bottom_noise_total > 2.0 && top_noise_total < 2.0)) {
      correction_x = BestPosTop.x;
      correction_y = BestPosTop.y;
      update_pose = true;
      m_localization_status.status = "BOTTOM IS NOT STARTED. ONLY TOP";
      m_localization_status.top_initialized = true;
      m_localization_status.bottom_initialized = false;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::ONLY_TOP;

      Eigen::MatrixXd P_curr, P_2by2;
      ekf_.getLatestP(P_curr);
      P_2by2 = P_curr.block(0, 0, 2, 2);

      Eigen::MatrixXd y_est(2, 1);
      y_est << CurrentEstimated.x, CurrentEstimated.y;

      Eigen::MatrixXd y_top(2, 1);
      y_top << BestPosTop.x, BestPosTop.y;
      double topError = GetmahalanobisDistance(y_est, y_top, P_2by2);

      double top_yaw_error =
          fabs(BestPosTop.yaw -
               (ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB)));
               
      m_localization_status.top_error = topError;

      if (top_yaw_error > 2 * M_PI)
        top_yaw_error = top_yaw_error - 2 * M_PI;

      if (topError < 5.0 && top_yaw_error < 0.1) {
        bInitConverged = true;
      }

    } else {
      // NO SENSOR INITIALIZED
      m_localization_status.status = "NO SENSOR INITIALIZED";
      m_localization_status.top_initialized = false;
      m_localization_status.bottom_initialized = false;
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::NO_SENSOR_INITIALIZED;
      node_status = nif::common::NODE_ERROR;
    }

    if (bottom_noise_total > 2.0 && top_noise_total > 2.0 && bBOTTOM_GPS &&
        bTOP_GPS) {
      m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::GPS_HIGH_ERROR;
      update_pose = false;
      node_status = nif::common::NODE_ERROR;
    } else if (bottom_noise_total > 2.0 && 
               bBOTTOM_GPS && !bTOP_GPS)
    {
      m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::GPS_HIGH_ERROR;
      update_pose = false;
      node_status = nif::common::NODE_ERROR;
    } else if (top_noise_total > 2.0 &&
               !bBOTTOM_GPS && bTOP_GPS) 
    {
      m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::GPS_HIGH_ERROR;
      update_pose = false;
      node_status = nif::common::NODE_ERROR;
    }

    if (!bInitConverged) {
        m_localization_status.status = "NO CONVERGED. WAITING FOR CONVERGENCE";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::NO_CONVERGED;
        update_pose = true;
        node_status = nif::common::NODE_ERROR;
      }

    /* pose measurement update */
    if ((bottom_gps_update == true || top_gps_update == true) && update_pose) {
      measurementUpdatePose(bestpos_time_last_update, correction_x,
                            correction_y, correction_yaw);

      // std::cout << "correction_yaw : " << correction_yaw << std::endl;
      // std::cout << "CurrentEstimated : " << CurrentEstimated.yaw << std::endl;

      bottom_gps_update = false;
      top_gps_update = false;
    }

    // if(measure_flag == true)
    // {
    measurementUpdateTwist(imu_time_last_update, veh_vel_x, veh_yaw_rate);
    //   measure_flag = false;
    // }

    /* set current pose, twist */
    setCurrentResult();

    /* publish ekf result */
    publishEstimateResult();

    pub_localization_status->publish(m_localization_status);
    this->setNodeStatus(node_status);

    // std::cout << "Run " << std::endl;

  } else {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for -[/novatel_bottom/bestpos]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/novatel_bottom/inspva]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/novatel_bottom/imu/data]");
    RCLCPP_DEBUG(this->get_logger(),
                 "            -[/raptor_dbw_interface/wheel_speed_report]");
  }
}

void AWLocalizationNode::showCurrentX()
{
  if (show_debug_info_)
  {
    Eigen::MatrixXd X(dim_x_, 1);
    ekf_.getLatestX(X);
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * setCurrentResult
 */
void AWLocalizationNode::setCurrentResult()
{
  current_ekf_pose_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;
  double roll = 0.;
  double pitch = 0.;
  double yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  q_tf.setRPY(roll, pitch, yaw);
  tf2::convert(q_tf, current_ekf_pose_.pose.orientation);

  current_ekf_twist_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

// HOW TO CORRECT HEADING IN EKF
// 1. Bottom GPS
// 2. Top GPS
// 3. BestVel
void AWLocalizationNode::BESTPOSCallback
    (const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {

  this->bestpos_time_last_update = this->now();

  // test
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator(seed);
  // double mean = m_testnoise / 10.;
  // double stdev = (int)m_testnoise % 10;
  // std::normal_distribution<double> dist(mean, stdev);

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat ; //- dist(generator) * 1e-5;
  currentGPS.longitude = (double)msg->lon; //- dist(generator) * 1e-5;
  // Currently ignore altitude for the most part and just track x/y
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry ltp_odom;

  ltp_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  ltp_odom.header.stamp = this->now();
  ltp_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;

  m_dGPS_X = ltp_pt.x;
  m_dGPS_Y = -ltp_pt.y;

  // We convert from NED to FLU
  ltp_odom.pose.pose.orientation.w = 1;
  ltp_odom.pose.pose.position.x = m_dGPS_X;
  ltp_odom.pose.pose.position.y = m_dGPS_Y;
  ltp_odom.pose.pose.position.z = 0;

  tf2::Quaternion quat_ekf;
  geometry_msgs::msg::Quaternion quat_ekf_msg;
  quat_ekf.setRPY(m_dGPS_roll, 0, m_dGPS_Heading);
  quat_ekf.normalize();
  quat_ekf_msg = tf2::toMsg(quat_ekf);
  ltp_odom.pose.pose.orientation = quat_ekf_msg;

  ltp_odom.pose.covariance.at(0) = BestPosBottom.lat_noise;  //  x - x
  ltp_odom.pose.covariance.at(1) = BestPosBottom.lat_noise;  //  x - y
  ltp_odom.pose.covariance.at(6) = BestPosBottom.lon_noise;  //  y - x
  ltp_odom.pose.covariance.at(7) = BestPosBottom.lon_noise;  //  y - y
  ltp_odom.pose.covariance.at(35) = BestPosBottom.yaw_noise; //  yaw - yaw

  pub_bestpos_odometry->publish(ltp_odom);

  bottom_gps_update = true;
  bBOTTOM_GPS = true;

  BestPosBottom.x = m_dGPS_X;
  BestPosBottom.y = m_dGPS_Y;

  ////////////////////////////////////////////////////////////////////////////////////
}

void AWLocalizationNode::TOPBESTPOSCallback(
    const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {
  // test
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  // std::default_random_engine generator(seed);
  // double mean = m_testnoise / 10.;
  // double stdev = (int)m_testnoise % 10;
  // std::normal_distribution<double> dist(mean, stdev);

  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat; //+ dist(generator) * 1e-5;
  currentGPS.longitude = (double)msg->lon; //+ dist(generator) * 1e-5;
  // Currently ignore altitude for the most part and just track x/y
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry top_best_pos_odom;

  top_best_pos_odom.header.frame_id = nif::common::frame_id::localization::ODOM;
  top_best_pos_odom.header.stamp = this->now();
  top_best_pos_odom.child_frame_id = nif::common::frame_id::localization::BASE_LINK;
    
  m_d_TOP_GPS_X = ltp_pt.x;
  m_d_TOP_GPS_Y = -ltp_pt.y;


  // TO MATCH THE TOP NOVATEL TO THE BOTTOM NOVATEL 
  tf2::Transform transform_top_to_bottom;
  transform_top_to_bottom.setOrigin(tf2::Vector3(top_to_bottom_bias_x, top_to_bottom_bias_y, 0.0));
  tf2::Quaternion quat_top_to_bottom;
  quat_top_to_bottom.setRPY(0.0, 0.0, 0.0);
  transform_top_to_bottom.setRotation(quat_top_to_bottom);

  tf2::Transform transform_top_on_global;
  transform_top_on_global.setOrigin(tf2::Vector3(m_d_TOP_GPS_X, m_d_TOP_GPS_Y, -ltp_pt.z));
  tf2::Quaternion quat_top_on_global;
  quat_top_on_global.setRPY(m_dGPS_roll, 0, m_dGPS_Heading);
  transform_top_on_global.setRotation(quat_top_on_global);

  auto transform_top_to_bottom_sync = transform_top_on_global * transform_top_to_bottom ;

  tf2::Quaternion q = transform_top_to_bottom_sync.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(), q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform_top_to_bottom_sync.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());

  top_best_pos_odom.pose.pose.position.x = transform_top_to_bottom_sync.getOrigin().x();
  top_best_pos_odom.pose.pose.position.y = transform_top_to_bottom_sync.getOrigin().y();
  top_best_pos_odom.pose.pose.position.z = transform_top_to_bottom_sync.getOrigin().z();
  top_best_pos_odom.pose.pose.orientation.x = transform_top_to_bottom_sync.getRotation().x();
  top_best_pos_odom.pose.pose.orientation.y = transform_top_to_bottom_sync.getRotation().y();
  top_best_pos_odom.pose.pose.orientation.z = transform_top_to_bottom_sync.getRotation().z();
  top_best_pos_odom.pose.pose.orientation.w = transform_top_to_bottom_sync.getRotation().w();
  top_best_pos_odom.pose.covariance.at(0) = BestPosTop.lat_noise; //  x - x
  top_best_pos_odom.pose.covariance.at(1) = BestPosTop.lat_noise; //  x - y
  top_best_pos_odom.pose.covariance.at(6) = BestPosTop.lon_noise; //  y - x
  top_best_pos_odom.pose.covariance.at(7) = BestPosTop.lon_noise; //  y - y
  top_best_pos_odom.pose.covariance.at(35) = BestPosTop.yaw_noise; //  yaw - yaw

  pub_top_bestpos_odometry->publish(top_best_pos_odom);

  ////////////////////////////////////////////////////////////////////////////////////

  BestPosTop.x = transform_top_to_bottom_sync.getOrigin().x();
  BestPosTop.y = transform_top_to_bottom_sync.getOrigin().y();

  bTOP_GPS = true;
  top_gps_update = true;
}

void AWLocalizationNode::BOTTOMINSPVACallback(
    const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg) {
  double yaw = (-msg->azimuth) * nif::common::constants::DEG2RAD;
  if (yaw != 0.0 && yaw != m_prevYaw) {
    m_inspva_heading_init = true;
  }
  if (!m_inspva_heading_init) {
    // std::cout << "INSPVA HEADING IS NOT INITIALIZED" << std::endl;
    m_prevYaw = yaw;
    return;
  }

  m_dGPS_Heading = yaw;
  m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
  bBOTTOMGPSHeading = true;
  if (m_use_inspva_heading) {
    heading_flag = true;
  }

  BestPosBottom.yaw = yaw;
}

void AWLocalizationNode::TOPINSPVACallback(
    const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg) {
  double yaw = (-msg->azimuth) * nif::common::constants::DEG2RAD; // TODO

  if (yaw != 0.0 && yaw != m_prevTOPYaw) {
    m_top_inspva_heading_init = true;
  }
  if (!m_top_inspva_heading_init) {
    // std::cout << "INSPVA HEADING IS NOT INITIALIZED" << std::endl;
    m_prevTOPYaw = yaw;
    return;
  }

  m_dGPS_TOP_Heading = yaw;
  m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
  bTOPGPSHeading = true;
  if (m_use_inspva_heading) {
    heading_flag = true;
  }

  BestPosTop.yaw = yaw;
}

void AWLocalizationNode::BESTVELCallback(
    const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg) {
  double yaw = (-msg->trk_gnd) * nif::common::constants::DEG2RAD;

  if (!m_inspva_heading_init &&
      m_use_inspva_heading) // INSPVA HEADING BACK UP SOLUTITON
  {
    m_dGPS_Heading = yaw;
    // m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
    bBOTTOMGPSHeading = true;
  }

  // When we don't use INSPVA HEADING, we use bestvel heading
  // param [use_inspva_heading]
  //       True :  Use bestvel for only back-up solution
  //       False : Use bestvel for heading estimation
  if (!m_use_inspva_heading) {
    m_dGPS_Heading = yaw;
    // m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
    bBOTTOMGPSHeading = true;
  }

  if (m_dVelolcity_X > m_bestvel_heading_update_thres) {
    heading_flag = true;
  }

  if (bUseBestVelForSpeed) {
    m_dVelolcity_X = msg->hor_speed;
  }
}

void AWLocalizationNode::INSSTDEVCallback(
    const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr msg) {
    m_stdev_latitude = msg->latitude_stdev;
    m_stdev_longitude = msg->longitude_stdev;
    m_stdev_azimuth = msg->azimuth_stdev / 180 * M_PI;

    BestPosBottom.lat_noise = msg->latitude_stdev;
    BestPosBottom.lon_noise = msg->longitude_stdev;
    BestPosBottom.yaw_noise = msg->azimuth_stdev / 180 * M_PI;
}

void AWLocalizationNode::TOPINSSTDEVCallback(
    const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr msg) {

  m_stdev_latitude = msg->latitude_stdev;
  m_stdev_longitude = msg->longitude_stdev;
  m_stdev_azimuth = msg->azimuth_stdev / 180 * M_PI;

  BestPosTop.lat_noise = msg->latitude_stdev;
  BestPosTop.lon_noise = msg->longitude_stdev;
  BestPosTop.yaw_noise = msg->azimuth_stdev / 180 * M_PI;
}

void AWLocalizationNode::MessegefilteringCallback(
    const sensor_msgs::msg::Imu ::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg) {

  this->imu_time_last_update = this->now();
  m_dIMU_yaw_rate = imu_msg->angular_velocity.z;
  m_dVelolcity_X =
      (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
      nif::common::constants::KPH2MS;

  if (!bImuFirstCall) {
    bImuFirstCall = true;
    return;
  }
}

void AWLocalizationNode::IMUCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {

  if(!bUseBestVelForSpeed)
  {
    return;
  }

  this->imu_time_last_update = this->now();
  m_dIMU_yaw_rate = msg->angular_velocity.z;

  if (!bImuFirstCall) {
    bImuFirstCall = true;
    return;
  }
}
void AWLocalizationNode::TestCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
    {
      m_testnoise = msg->data;
    }

/*
 * initEKF
 */
void AWLocalizationNode::initEKF() {
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15; //  for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                           //  for yaw
  P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;                         //  for yaw bias
  P(IDX::VX, IDX::VX) = 1000.0;                                           //  for vx
  P(IDX::WZ, IDX::WZ) = 50.0;                                             //  for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void AWLocalizationNode::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k
   *
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  Eigen::MatrixXd X_next(dim_x_, 1); //  predicted state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);

  const int d_dim_x = dim_x_ex_ - dim_x_;
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = ekf_dt_;

  /* Update for latest state */
  X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt; //  dx = v * cos(yaw)
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt; //  dy = v * sin(yaw)
  X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                   //  dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  X_next(IDX::YAW) = std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  const double dvx = std::sqrt(P_curr(IDX::VX, IDX::VX));
  const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  if (dvx < 10.0 && dyaw < 1.0)
  {
   //  auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

    /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
     dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
    Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2); //  coeff of deviation of vx & yaw
    Jp << cos(yaw), -vx * sin(yaw), sin(yaw), vx * cos(yaw);
    Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2); //  cov of vx and yaw

    Q_vx_yaw(0, 0) = P_curr(IDX::VX, IDX::VX) * dt;       //  covariance of vx - vx
    Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt;     //  covariance of yaw - yaw
    Q_vx_yaw(0, 1) = P_curr(IDX::VX, IDX::YAW) * dt;      //  covariance of vx - yaw
    Q_vx_yaw(1, 0) = P_curr(IDX::YAW, IDX::VX) * dt;      //  covariance of yaw - vx
    Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose(); //  for pos_x & pos_y
  }
  else
  {
   //  vx & vy is not converged yet, set constant value.
    Q(IDX::X, IDX::X) = 0.05;
    Q(IDX::Y, IDX::Y) = 0.05;
  }

  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;        //  for yaw
  Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_; //  for yaw bias
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;           //  for vx
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;           //  for wz

  ekf_.predictWithDelay(X_next, A, Q);

 //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void AWLocalizationNode::measurementUpdatePose(rclcpp::Time measurement_time_,
                                               const double &corr_x_,
                                               const double &corr_y_,
                                               const double &corr_yaw_) {
  //  if (msg->header.frame_id != nif::common::frame_id::localization::ODOM)
  //  {
  //    ROS_WARN_DELAYED_THROTTLE(2, "pose frame_id is %s, but pose_frame is set
  //    as %s. They must be same.",
  //                              msg->header.frame_id.c_str(),
  //                              nif::common::frame_id::localization::ODOM.c_str());
  //  }
  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3; //  pos_x, pos_y, yaw, depending on Pose output
  auto t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - measurement_time_).seconds() +
                      (t_curr - measurement_time_).nanoseconds() * 1e-9 +
                      pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    //  ROS_WARN_DELAYED_THROTTLE(1.0, "Pose time stamp is inappropriate, set
    //  delay to 0[s]. delay = %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN(this->get_logger(), 
                "Pose delay exceeds the compensation limit, ignored. delay: %f[s], "
                " limit= extend_state_step * ekf_dt : %f [s]", delay_time, extend_state_step_* ekf_dt_);
    return;
  }
  // DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  const double yaw_curr =
      ekf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
  double yaw = corr_yaw_;
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error =
      normalizeYaw(yaw - ekf_yaw); //  normalize the error not to exceed 2 pi
   
  yaw = yaw_error + ekf_yaw;
  // std::cout << yaw << ", " << yaw_error << ", "
  //           << ekf_yaw << std::endl;
                   /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  Eigen::MatrixXd y_correction(2, 1);

  y << corr_x_, corr_y_, yaw;
  y_correction << corr_x_, corr_y_;


  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    RCLCPP_WARN(this->get_logger(), "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");    
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  Eigen::MatrixXd y_prediction(2, 1);

  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
      ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;

  y_prediction << ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y);

  Eigen::MatrixXd P_curr, P_y, P_2by2;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  P_2by2 = P_curr.block(0, 0, 2, 2);

  // if(!GPSIgnoreGate(update_ignore_distance, y_ekf, y, P_y))
  // {
  //   return;
  // }
  // std::cout << "P_y : " << P_y << std::endl;

  // if (!mahalanobisGate(update_ignore_distance, y_ekf, y, P_y))
  // if (!mahalanobisGate(update_ignore_distance, y_prediction, y_correction, P_2by2) && 
  //     m_localization_status.localization_status_code !=nif_msgs::msg::LocalizationStatus::BEST_STATUS) {
  //   return;
  // }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;   //  for pos x
  C(1, IDX::Y) = 1.0;   //  for pos y
  C(2, IDX::YAW) = 1.0; //  for yaw

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_pose_with_covariance_) {
    R(0, 0) = current_pose_covariance_.at(0);  //  x - x
    R(0, 1) = current_pose_covariance_.at(1);  //  x - y
    R(0, 2) = current_pose_covariance_.at(5);  //  x - yaw
    R(1, 0) = current_pose_covariance_.at(6);  //  y - x
    R(1, 1) = current_pose_covariance_.at(7);  //  y - y
    R(1, 2) = current_pose_covariance_.at(11); //  y - yaw
    R(2, 0) = current_pose_covariance_.at(30); //  yaw - x
    R(2, 1) = current_pose_covariance_.at(31); //  yaw - y
    R(2, 2) = current_pose_covariance_.at(35); //  yaw - yaw
  } else {
    const double ekf_yaw = ekf_.getXelement(IDX::YAW);
    const double vx = ekf_.getXelement(IDX::VX);
    const double wz = ekf_.getXelement(IDX::WZ);
    const double cov_pos_x =
        std::pow(pose_measure_uncertainty_time_ * vx * cos(ekf_yaw), 2.0);
    const double cov_pos_y =
        std::pow(pose_measure_uncertainty_time_ * vx * sin(ekf_yaw), 2.0);
    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);

    R(0, 0) = m_stdev_latitude + cov_pos_x * 0.2 +  std::pow(pose_stddev_x_, 2) ; //+ cov_pos_x; //  pos_x
    R(1, 1) = m_stdev_longitude + cov_pos_y * 0.2 + std::pow(pose_stddev_y_, 2) ; //+ cov_pos_y +  pos_y
    R(2, 2) = m_stdev_azimuth + cov_yaw + std::pow(pose_stddev_yaw_, 2); // + cov_yaw; //  yaw

    if (bBOTTOM_GPS && bTOP_GPS) // if TOP AND BOTTOM are started, calculated covariance using difference between two sensors.
    {
      // if (m_localization_status.localization_status_code ==
      //         nif_msgs::msg::LocalizationStatus::BEST_STATUS)
      // {
        R(0, 0) = fabs(BestPosBottom.x - BestPosTop.x) + cov_pos_x +
                  std::pow(pose_stddev_x_, 2); 
        R(1, 1) = fabs(BestPosBottom.y - BestPosTop.y) + cov_pos_y +
                  std::pow(pose_stddev_y_, 2); 
        R(2, 2) = m_stdev_azimuth + cov_yaw +
                  std::pow(pose_stddev_yaw_, 2); 
      // } else if (m_localization_status.localization_status_code ==
      //            nif_msgs::msg::LocalizationStatus::ONLY_TOP) {
      //   R(0, 0) = BestPosTop.lat_noise + cov_pos_x +
      //             std::pow(pose_stddev_x_, 2); 
      //   R(1, 1) = BestPosTop.lon_noise + cov_pos_y +
      //             std::pow(pose_stddev_y_, 2); 
      //   R(2, 2) = m_stdev_azimuth + cov_yaw +
      //             std::pow(pose_stddev_yaw_, 2); 
      // } else if (m_localization_status.localization_status_code ==
      //            nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM) {
      //   R(0, 0) = BestPosBottom.lat_noise + cov_pos_x +
      //             std::pow(pose_stddev_x_, 2); 
      //   R(1, 1) = BestPosBottom.lon_noise + cov_pos_y +
      //             std::pow(pose_stddev_y_, 2); 
      //   R(2, 2) = m_stdev_azimuth + cov_yaw +
      //             std::pow(pose_stddev_yaw_, 2); 
      // }
    }

  }

  /* In order to avoid a large change at the time of updating, measuremeent
   * update is performed by dividing at every step. */
  R *= (ekf_rate_ / pose_rate_);

  // std::cout << "ekf_rate_: "  << ekf_rate_ << std::endl;
  // std::cout << "pose_rate_: " << pose_rate_ << std::endl;

  ekf_.updateWithDelay(y, C, R, delay_step);

  //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());

}


/*
 * measurementUpdateTwist
 */
void AWLocalizationNode::measurementUpdateTwist(rclcpp::Time measurement_time_,
                                                const double &pred_vel_x_,
                                                const double &pred_yaw_rate_) {
  //  if (msg->header.frame_id != nif::common::frame_id::localization::ODOM)
  //  {
  //    ROS_WARN_DELAYED_THROTTLE(2.0, "twist frame_id must be base_link");
  //  }

  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2; //  vx, wz
  auto t_curr = this->now();

  /* Calculate delay step */
  /* Calculate delay step */
  double delay_time = (t_curr - measurement_time_).seconds() +
                      (t_curr - measurement_time_).nanoseconds() * 1e-9 +
                      twist_additional_delay_;

  if (delay_time < 0.0) {
    // ROS_WARN_DELAYED_THROTTLE(1.0, "Twist time stamp is inappropriate (delay
    // = %f [s]), set delay to 0[s].",
    //                           delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN(this->get_logger(), 
                "Twist delay exceeds the compensation limit, ignored. delay: "
                "%f[s], limit = "
                "extend_state_step * ekf_dt : %f [s]",
                delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  // DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pred_vel_x_, pred_yaw_rate_;

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    // ROS_WARN("[EKF] twist measurement matrix includes NaN of Inf. ignore
    // update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
      ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);
  // if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
  //   // ROS_WARN_DELAYED_THROTTLE(2.0, "[EKF] Twist measurement update,
  //   // mahalanobis distance is over limit. ignore "
  //   //                                "measurement data.");
  //   return;
  // }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0; //  for vx
  C(1, IDX::WZ) = 1.0; //  for wz

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_twist_with_covariance_) {
    R(0, 0) = current_twist_covariance_.at(0);  //  vx - vx
    R(0, 1) = current_twist_covariance_.at(5);  //  vx - wz
    R(1, 0) = current_twist_covariance_.at(30); //  wz - vx
    R(1, 1) = current_twist_covariance_.at(35); //  wz - wz
  } else {
    R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * ekf_dt_; //  for vx
    R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * ekf_dt_; //  for wz
  }

  /* In order to avoid a large change by update, measurement update is performed
   * by dividing at every step. */
  R *= (ekf_rate_ / twist_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  //  debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
};

/*
 * mahalanobisGate
 */
bool AWLocalizationNode::mahalanobisGate(const double& dist_max, const Eigen::MatrixXd& x, const Eigen::MatrixXd& obj_x,
                                   const Eigen::MatrixXd& cov)
{
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov * (x - obj_x);
  // std::cout << "x : " << x << ", obj_x: " << obj_x << ", delta : " << x - obj_x << std::endl;
  // std::cout << "P_y^-1 : " << cov.inverse() << std::endl;
  // std::cout << "mahalanobis_squared : " << std::sqrt(mahalanobis_squared(0))  << std::endl;

  // RCLCPP_INFO(this->get_logger(), "measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(mahalanobis_squared(0)), dist_max);
  m_mahalanobisScore = std::sqrt(mahalanobis_squared(0));

  if (mahalanobis_squared(0) > dist_max * dist_max)
  {
    RCLCPP_WARN(this->get_logger(), "speed : %f, yaw_rate : %f", m_dVelolcity_X,
                m_dIMU_yaw_rate);
    RCLCPP_WARN(this->get_logger(),
                "[EKF] Pose measurement update, mahalanobis distance is over "
                "limit.ignore measurement data.");
    return false;
  }

  return true;
}

double AWLocalizationNode::GetmahalanobisDistance(const Eigen::MatrixXd &x,
                                                  const Eigen::MatrixXd &obj_x,
                                                  const Eigen::MatrixXd &cov)
{
  Eigen::MatrixXd mahalanobis_squared =
      (x - obj_x).transpose() * cov * (x - obj_x);
  double distance = std::sqrt(mahalanobis_squared(0));

  return distance;
}

/**
 * @brief CalculateBestCorrection selects the best correction
 *        using novatel_bottom and novatel_top.
 * @param GPS_pose  novatel_bottom/bestpos, novatel_bottom/bestpos
 * @param Current_estimation  current estimation
 * @return fused correction data.
 */
bool AWLocalizationNode::CalculateBestCorrection(
    const GPSCorrectionData_t &BottomDataIn,
    const GPSCorrectionData_t &TopDataIn, const VehPose_t &CurrentEstIn,
    VehPose_t &BestCorrectionOut) {
  Eigen::MatrixXd P_curr, P_2by2;
  ekf_.getLatestP(P_curr);
  P_2by2 = P_curr.block(0, 0, 2, 2);

  Eigen::MatrixXd y_est(2, 1);
  y_est << CurrentEstIn.x, CurrentEstIn.y;

  Eigen::MatrixXd y_bottom(2, 1);
  y_bottom << BottomDataIn.x, BottomDataIn.y;
  double bottomError = GetmahalanobisDistance(y_est, y_bottom, P_2by2);

  Eigen::MatrixXd y_top(2, 1);
  y_top << TopDataIn.x, TopDataIn.y;
  double topError = GetmahalanobisDistance(y_est, y_top, P_2by2);

  double diff_top_bottom = sqrt((BottomDataIn.x - TopDataIn.x) * (BottomDataIn.x - TopDataIn.x) +
                                (BottomDataIn.y - TopDataIn.y) * (BottomDataIn.y - TopDataIn.y));

  double bottom_yaw_error = fabs(BottomDataIn.yaw - (ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB))); //correction - ekf
  double top_yaw_error = fabs(TopDataIn.yaw - (ekf_.getXelement(IDX::YAW) +
                                                ekf_.getXelement(IDX::YAWB)));
  if (bottom_yaw_error > 2*M_PI)
    bottom_yaw_error = bottom_yaw_error - 2*M_PI;
    
  if ((bottomError < 5.0 || topError < 5.0) && bottom_yaw_error < 0.1) {
    bInitConverged = true;
  }
  // std::cout << "------------" << std::endl;
  // std::cout << "bottomError : " << bottomError << std::endl;
  // std::cout << "topError : " << topError << std::endl;
  // std::cout << "diff: " << diff_top_bottom << std::endl;

  double norm_weight_bottom, norm_weight_top;

  norm_weight_bottom = 1 - bottomError / (bottomError + topError);
  norm_weight_top = 1 - topError / (bottomError + topError);

  m_localization_status.bottom_error = bottomError;
  m_localization_status.top_error = topError;
  m_localization_status.diff = diff_top_bottom;
  m_localization_status.bottom_weight = norm_weight_bottom;
  m_localization_status.top_weight = norm_weight_top;

  double pose_gate_dist_local;
  if (!bInitConverged)
  {
    pose_gate_dist_local = DBL_MAX;
  }
  else
  {
    pose_gate_dist_local = pose_gate_dist_;
  }

    // if bottom and top is close enough, we regard bottom status is nominal.
    // output : bypass fusion. return novatel bottom directly.
    if (diff_top_bottom < 0.5) {
      BestCorrectionOut.x = BottomDataIn.x;
      BestCorrectionOut.y = BottomDataIn.y;
      m_localization_status.status = "BEST STATUS";
      m_localization_status.localization_status_code =
          nif_msgs::msg::LocalizationStatus::BEST_STATUS;
      return true;
    } else // one of sensor has drift. we should fuse the sensors.
    {
      // 1. our top priority strategy is weight-sum.
      if (bottomError < pose_gate_dist_local && topError < pose_gate_dist_local &&
          fabs(topError - bottomError) < pose_gate_dist_local) {
        BestCorrectionOut.x =
            BottomDataIn.x * norm_weight_bottom + TopDataIn.x * norm_weight_top;
        BestCorrectionOut.y =
            BottomDataIn.y * norm_weight_bottom + TopDataIn.y * norm_weight_top;
        m_localization_status.status = "SENSOR FUSION";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::SENSOR_FUSION;
        return true;
      }
      // 2. if bottom is diverged, and top is nominal, USE NOVATEL TOP
      else if (bottomError > pose_gate_dist_local && topError < pose_gate_dist_local) {
        BestCorrectionOut.x = TopDataIn.x;
        BestCorrectionOut.y = TopDataIn.y;
        m_localization_status.status = "BOTTOM ERROR, USE TOP";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::ONLY_TOP;
        return true;
      }
      // 3. if top is diverged, and bottom is nominal, USE NOVATEL BOTTOM
      else if (bottomError < pose_gate_dist_local && topError > pose_gate_dist_local) {
        BestCorrectionOut.x = BottomDataIn.x;
        BestCorrectionOut.y = BottomDataIn.y;
        m_localization_status.status = "TOP ERROR, USE BOTTOM";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::ONLY_BOTTOM;
        return true;
      }
      // 4. if all the sensors are bad, do not update measurement
      else {
        m_localization_status.status = "NO UPDATE, GPS HIGH ERROR";
        m_localization_status.localization_status_code =
            nif_msgs::msg::LocalizationStatus::GPS_HIGH_ERROR;
        return false;
      }
    }
}

/*
  * GPSIgnoreGate
  * Maximum body slip angle(beta) : 0.2479
      = rad = atan(y_delta /  x_delta)
  */
bool AWLocalizationNode::GPSIgnoreGate(
    const double &dist_max,
    const Eigen::MatrixXd &x,     // estimated
    const Eigen::MatrixXd &obj_x, // correct
    const Eigen::MatrixXd &cov) {

  double x_delta = (obj_x(0,0)- x(0,0)) * cos(x(2,0)) + (obj_x(1,0)- x(1,0)) * sin(x(2,0));
  double y_delta = -(obj_x(0,0)- x(0,0)) * sin(x(2,0)) + (obj_x(1,0)- x(1,0)) * cos(x(2,0));

  RCLCPP_INFO(this->get_logger(), "x_delta = %f, y_delta = %f", x_delta, y_delta);

  if (x_delta < 0. && atan(fabs(y_delta / x_delta)) > 0.2479) {
    RCLCPP_WARN(this->get_logger(), "Non-holonomic model cannot move like this");
    return false;
  }

  return true;
}

/*
 * publishEstimateResult
 */
void AWLocalizationNode::publishEstimateResult()
{
  auto current_time = this->now();
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  current_ekf_odom_.header.stamp = current_time;
  current_ekf_odom_.header.frame_id = nif::common::frame_id::localization::ODOM;
  current_ekf_odom_.child_frame_id = nif::common::frame_id::localization::BASE_LINK;
  current_ekf_odom_.pose.pose = current_ekf_pose_.pose;
  current_ekf_odom_.pose.covariance[0] = P(IDX::X, IDX::X);
  current_ekf_odom_.pose.covariance[1] = P(IDX::X, IDX::Y);
  current_ekf_odom_.pose.covariance[5] = P(IDX::X, IDX::YAW);
  current_ekf_odom_.pose.covariance[6] = P(IDX::Y, IDX::X);
  current_ekf_odom_.pose.covariance[7] = P(IDX::Y, IDX::Y);
  current_ekf_odom_.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  current_ekf_odom_.pose.covariance[30] = P(IDX::YAW, IDX::X);
  current_ekf_odom_.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  current_ekf_odom_.pose.covariance[35] = P(IDX::YAW, IDX::YAW);

  current_ekf_odom_.twist.twist = current_ekf_twist_.twist;
  current_ekf_odom_.twist.covariance[0] = P(IDX::VX, IDX::VX);
  current_ekf_odom_.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  current_ekf_odom_.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  current_ekf_odom_.twist.covariance[35] = P(IDX::WZ, IDX::WZ);

  pub_odom_->publish(current_ekf_odom_);

  geometry_msgs::msg::TransformStamped nav_base_tf{};
  nav_base_tf.transform.translation.x = current_ekf_odom_.pose.pose.position.x;
  nav_base_tf.transform.translation.y = current_ekf_odom_.pose.pose.position.y;
  nav_base_tf.transform.translation.z = current_ekf_odom_.pose.pose.position.z;
  nav_base_tf.transform.rotation.w = current_ekf_odom_.pose.pose.orientation.w;
  nav_base_tf.transform.rotation.x = current_ekf_odom_.pose.pose.orientation.x;
  nav_base_tf.transform.rotation.y = current_ekf_odom_.pose.pose.orientation.y;
  nav_base_tf.transform.rotation.z = current_ekf_odom_.pose.pose.orientation.z;
  nav_base_tf.header.stamp = this->now();
  nav_base_tf.header.frame_id = nif::common::frame_id::localization::ODOM;
  nav_base_tf.child_frame_id = nif::common::frame_id::localization::BASE_LINK;
  broadcaster_->sendTransform(nav_base_tf);
  
  /* publish localization score*/
  std_msgs::msg::Float64 mahalanobisScoreMsg;
  mahalanobisScoreMsg.data = m_mahalanobisScore;
  pub_mahalanobisScore->publish(mahalanobisScoreMsg);

  /* publish yaw bias */
  std_msgs::msg::Float64 yawb;
  yawb.data = X(IDX::YAWB);
  pub_yaw_bias_->publish(yawb);

  /* debug measured pose */
  if (measure_odom_ptr_ != nullptr)
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose = measure_odom_ptr_->pose.pose;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }

}


double AWLocalizationNode::normalizeYaw(const double& yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

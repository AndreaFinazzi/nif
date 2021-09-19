//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by USRG on 9/14/21.
//

#include "ekf_localizer/nif_aw_localization_node.h"
#include "nif_frame_id/frame_id.h"

using namespace message_filters;
using namespace std::placeholders;

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
// #define DEBUG_INFO(...) { if (show_debug_info_) { RCLCPP_INFO(this->get_logger(),  _VA_ARGS__); } }
#define DEBUG_PRINT_MAT(X) { if (show_debug_info_) { std::cout << #X << ": " << X << std::endl; } }

// clang-format on
AWLocalizationNode::AWLocalizationNode(const std::string &node_name)
    : IBaseNode(node_name), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */) {
  this->declare_parameter<bool>("show_debug_info", bool(false));
  this->declare_parameter<double>("predict_frequency", double(50.0));
  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  this->declare_parameter<double>("tf_rate", double(10.0));
  this->declare_parameter<bool>("enable_yaw_bias_estimation", bool(true));
  this->declare_parameter<int>("extend_state_step", int(50));
  this->declare_parameter<std::string>("pose_frame_id", std::string("odom"));

  this->declare_parameter<std::string>("measure_odom_name", std::string("/ndt_odom"));
  this->declare_parameter<std::string>("system_odom_name", std::string("/Odometry/system"));
  this->declare_parameter<std::string>("final_output_odom_name", std::string("/Odometry/ekf_estimated"));

  /* pose measurement */
  this->declare_parameter<double>("pose_additional_delay", double(0.0));
  this->declare_parameter<double>("pose_measure_uncertainty_time", double(0.01));
  this->declare_parameter<double>("pose_rate", double(10.0));              //  used for covariance calculation
  this->declare_parameter<double>("pose_gate_dist", double(10000.0)); //  Mahalanobis limit
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
  if (!enable_yaw_bias_estimation_)
  {
    proc_stddev_yaw_bias_c = 0.0;
  }


  this->show_debug_info_ = this->get_parameter("show_debug_info").as_bool();
  this->ekf_rate_ = this->get_parameter("predict_frequency").as_double();
  this->tf_rate_ = this->get_parameter("tf_rate").as_double();
  this->enable_yaw_bias_estimation_ = this->get_parameter("enable_yaw_bias_estimation").as_bool();
  this->extend_state_step_ = this->get_parameter("extend_state_step").as_int();
  this->pose_frame_id_ = this->get_parameter("pose_frame_id").as_string();

  this->measure_odom_name_ = this->get_parameter("measure_odom_name").as_string();
  this->system_odom_name_ = this->get_parameter("system_odom_name").as_string();
  this->final_output_odom_name_ = this->get_parameter("final_output_odom_name").as_string();

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


  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c, 2.0) * ekf_dt_;
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c, 2.0) * ekf_dt_;
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c, 2.0) * ekf_dt_;
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c, 2.0) * ekf_dt_;

  /* initialize ros system */
  using namespace std::chrono_literals; // NOLINT
  timer_control_ = this->create_wall_timer(
      10ms, std::bind(&AWLocalizationNode::timerCallback, this));
  timer_tf_ = this->create_wall_timer(
      10ms, std::bind(&AWLocalizationNode::timerTFCallback, this));

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(final_output_odom_name_, nif::common::constants::QOS_EGO_ODOMETRY);
  pub_yaw_bias_ = this->create_publisher<std_msgs::msg::Float64>("estimated_yaw_bias", nif::common::constants::QOS_EGO_ODOMETRY);
  pub_bestpos_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
      "out_odometry_bestpos", nif::common::constants::QOS_EGO_ODOMETRY);

  // sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&AWLocalizationNode::callbackInitialPose, this, std::placeholders::_1));
  sub_odom_measure_ = this->create_subscription<nav_msgs::msg::Odometry>(measure_odom_name_, 5, std::bind(&AWLocalizationNode::callbackOdometryMeasure, this, std::placeholders::_1));
  sub_odom_system_ = this->create_subscription<nav_msgs::msg::Odometry>(system_odom_name_, 20, std::bind(&AWLocalizationNode::callbackOdometrySystem, this, std::placeholders::_1));

  sub_gpslatlon = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_bestpos", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::GPSLATLONCallback, this, std::placeholders::_1));
  subINSPVA = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "in_inspva", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&AWLocalizationNode::GPSINSPVACallback, this, std::placeholders::_1));

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

  initEKF();

  /* debug */
  pub_debug_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("debug", 1);
  pub_measured_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);
};

AWLocalizationNode::~AWLocalizationNode(){};

/*
 * timerCallback
 */
void AWLocalizationNode::timerCallback()
{

  // DEBUG_INFO("========================= timer called
  // =========================");

  /* predict model in EKF */
  auto start = std::chrono::system_clock::now();
  // DEBUG_INFO("------------------------- start prediction
  // -------------------------");
  predictKinematicsModel();
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now() - start)
                       .count();
  // DEBUG_INFO("[EKF] predictKinematicsModel calculation time = %f [ms]",
  // elapsed * 1.0e-6); DEBUG_INFO("------------------------- end prediction
  // -------------------------\n");

  /* pose measurement update */
  if (measure_odom_ptr_ != nullptr) {
    // DEBUG_INFO("------------------------- start Pose
    // -------------------------");
    start = std::chrono::system_clock::now();
    measurementUpdatePose(measure_odom_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now() - start)
                  .count();
    // DEBUG_INFO("[EKF] measurementUpdatePose calculation time = %f [ms]",
    // elapsed * 1.0e-6); DEBUG_INFO("------------------------- end Pose
    // -------------------------\n");
  }

  /* twist measurement update */
  if (system_odom_ptr_ != nullptr) {
    // DEBUG_INFO("------------------------- start twist
    // -------------------------");
    start = std::chrono::system_clock::now();
    measurementUpdateTwist(system_odom_ptr_);
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::system_clock::now() - start)
                  .count();
    // DEBUG_INFO("[EKF] measurementUpdateTwist calculation time = %f [ms]",
    // elapsed * 1.0e-6); DEBUG_INFO("------------------------- end twist
    // -------------------------\n");
  }

  /* set current pose, twist */
  setCurrentResult();

  /* publish ekf result */
  publishEstimateResult();
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
  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;
  double roll, pitch, yaw;
  if (measure_odom_ptr_ != nullptr)
  {
    current_ekf_pose_.pose.position.z = measure_odom_ptr_->pose.pose.position.z;
    tf2::fromMsg(measure_odom_ptr_->pose.pose.orientation, q_tf); /* use Pose pitch and roll */
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  }
  else
  {
    current_ekf_pose_.pose.position.z = 0.0;
    roll = 0;
    pitch = 0;
  }
  // yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  q_tf.setRPY(roll, pitch, yaw);
  tf2::convert(q_tf, current_ekf_pose_.pose.orientation);

  current_ekf_twist_.header.frame_id = pose_frame_id_;
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

/*
 * timerTFCallback
 */
void AWLocalizationNode::timerTFCallback()
{
  if (current_ekf_pose_.header.frame_id == "")
    return;

  // geometry_msgs::TransformStamped transformStamped;
  // transformStamped.header.stamp = ros::Time::now();
  // transformStamped.header.frame_id = current_ekf_pose_.header.frame_id;
  // transformStamped.child_frame_id = "base_link";
  // transformStamped.transform.translation.x = current_ekf_pose_.pose.position.x;
  // transformStamped.transform.translation.y = current_ekf_pose_.pose.position.y;
  // transformStamped.transform.translation.z = current_ekf_pose_.pose.position.z;

  // transformStamped.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
  // transformStamped.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
  // transformStamped.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
  // transformStamped.transform.rotation.w = current_ekf_pose_.pose.orientation.w;

  // tf_br_.sendTransform(transformStamped);
}

/*
 * getTransformFromTF
 */
bool AWLocalizationNode::getTransformFromTF(std::string parent_frame, std::string child_frame,
                                      geometry_msgs::msg::TransformStamped& transform)
{
  // tf2_ros::Buffer tf_buffer;
  // tf2_ros::TransformListener tf_listener(tf_buffer);
  // ros::Duration(0.1).sleep();
  // if (parent_frame.front() == '/')
  //   parent_frame.erase(0, 1);
  // if (child_frame.front() == '/')
  //   child_frame.erase(0, 1);

  // TODO
  // for (int i = 0; i < 50; ++i)
  // {
  //   try
  //   {
  //     transform = tf_buffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
  //     return true;
  //   }
  //   catch (tf2::TransformException& ex)
  //   {
  //     ROS_WARN("%s", ex.what());
  //     ros::Duration(0.1).sleep();
  //   }
  // }
  return false;
}

/*
 * callbackInitialPose
 */
// void AWLocalizationNode::callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
// {
//   geometry_msgs::msg::TransformStamped transform;
//   if (!getTransformFromTF(pose_frame_id_, initialpose->header.frame_id, transform))
//   {
//     // ROS_ERROR("[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
//               // initialpose->header.frame_id.c_str());
//   };

//   Eigen::MatrixXd X(dim_x_, 1);
//   Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

//   X(IDX::X) = initialpose->pose.pose.position.x + transform.transform.translation.x;
//   X(IDX::Y) = initialpose->pose.pose.position.y + transform.transform.translation.y;
//   X(IDX::YAW) = tf2::getYaw(initialpose->pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
//   X(IDX::YAWB) = 0.0;
//   X(IDX::VX) = 0.0;
//   X(IDX::WZ) = 0.0;

//   P(IDX::X, IDX::X) = initialpose->pose.covariance[0];
//   P(IDX::Y, IDX::Y) = initialpose->pose.covariance[6 + 1];
//   P(IDX::YAW, IDX::YAW) = initialpose->pose.covariance[6 * 5 + 5];
//   P(IDX::YAWB, IDX::YAWB) = 0.0001;
//   P(IDX::VX, IDX::VX) = 0.01;
//   P(IDX::WZ, IDX::WZ) = 0.01;

//   ekf_.init(X, P, extend_state_step_);
// };

void AWLocalizationNode::GPSLATLONCallback(
    const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {
  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat;
  currentGPS.longitude = (double)msg->lon;
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

  ltp_odom.pose.pose.position.x = m_dGPS_X;
  // We convert from NED to FLU
  ltp_odom.pose.pose.position.y = m_dGPS_Y;
  ltp_odom.pose.pose.position.z = -ltp_pt.z;

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

  pub_bestpos_odometry->publish(ltp_odom);

  gps_flag = true;
  bGPS = true;
  ////////////////////////////////////////////////////////////////////////////////////
}

void AWLocalizationNode::GPSINSPVACallback(
    const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg) {
  double yaw = (-msg->azimuth) * nif::common::constants::DEG2RAD;
  m_dGPS_Heading = yaw;
  m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
  bGPSHeading = true;
}

void AWLocalizationNode::MessegefilteringCallback(
    const sensor_msgs::msg::Imu ::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg) {

  m_dIMU_yaw_rate = imu_msg->angular_velocity.z;
  m_dVelolcity_X =
      (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
      nif::common::constants::KPH2MS;

  auto ImuCurrentTime = rclcpp::Time(imu_msg->header.stamp);
  if (!bImuFirstCall) {
    bImuFirstCall = true;
    ImuPrevTimeDouble =
        static_cast<double>(ImuCurrentTime.seconds()) +
        static_cast<double>(ImuCurrentTime.nanoseconds()) * 1e-9;
    return;
  }
  ImuTimeDouble = static_cast<double>(ImuCurrentTime.seconds()) +
                  static_cast<double>(ImuCurrentTime.nanoseconds()) * 1e-9;
};

/*
 * callbackOdometryMeasure
 */
void AWLocalizationNode::callbackOdometryMeasure(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  measure_odom_ptr_ = std::make_shared<nav_msgs::msg::Odometry>(*msg);
  if (!use_pose_with_covariance_)
  {
    // std::cout << "No use pose cov" << std::endl;
  }

  if (use_pose_with_covariance_)
  {
    // std::cout << "Use pose cov" << std::endl;
    current_pose_covariance_ = msg->pose.covariance;
  }
};

/*
 * callbackOdometrySystem
 */
void AWLocalizationNode::callbackOdometrySystem(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  if (!use_twist_with_covariance_)
  {
    system_odom_ptr_ = std::make_shared<nav_msgs::msg::Odometry>(*msg);
    twist_linear_x = msg -> twist.twist.linear.x;
      
  }

  if (use_twist_with_covariance_)
  {
    current_twist_covariance_ = msg->twist.covariance;
  } 
};



/*
 * initEKF
 */
void AWLocalizationNode::initEKF()
{
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
void AWLocalizationNode::measurementUpdatePose(const nav_msgs::msg::Odometry::SharedPtr msg)
{
 //  if (msg->header.frame_id != pose_frame_id_)
 //  {
 //    ROS_WARN_DELAYED_THROTTLE(2, "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
 //                              msg->header.frame_id.c_str(), pose_frame_id_.c_str());
 //  }
  Eigen::MatrixXd X_curr(dim_x_, 1); //  curent state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3; //  pos_x, pos_y, yaw, depending on Pose output
  auto t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - msg->header.stamp).seconds() +
                      (t_curr - msg->header.stamp).nanoseconds() * 1e-9 +
                       pose_additional_delay_;
  if (delay_time < 0.0)
  {
    delay_time = 0.0;
   //  ROS_WARN_DELAYED_THROTTLE(1.0, "Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    // ROS_WARN_DELAYED_THROTTLE(1.0,
                              // "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
                              // "extend_state_step * ekf_dt : %f [s]",
                              // delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  // DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  const double yaw_curr = ekf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
  double yaw = tf2::getYaw(msg->pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw); //  normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    // ROS_WARN("[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y))
  {
    // ROS_WARN_DELAYED_THROTTLE(2.0, "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
    //                                "measurement data.");
    return;
  }

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
  if (use_pose_with_covariance_)
  {
    R(0, 0) = current_pose_covariance_.at(0);  //  x - x
    R(0, 1) = current_pose_covariance_.at(1);  //  x - y
    R(0, 2) = current_pose_covariance_.at(5);  //  x - yaw
    R(1, 0) = current_pose_covariance_.at(6);  //  y - x
    R(1, 1) = current_pose_covariance_.at(7);  //  y - y
    R(1, 2) = current_pose_covariance_.at(11); //  y - yaw
    R(2, 0) = current_pose_covariance_.at(30); //  yaw - x
    R(2, 1) = current_pose_covariance_.at(31); //  yaw - y
    R(2, 2) = current_pose_covariance_.at(35); //  yaw - yaw
  }
  else
  {
    const double ekf_yaw = ekf_.getXelement(IDX::YAW);
    const double vx = ekf_.getXelement(IDX::VX);
    const double wz = ekf_.getXelement(IDX::WZ);
    const double cov_pos_x = std::pow(pose_measure_uncertainty_time_ * vx * cos(ekf_yaw), 2.0);
    const double cov_pos_y = std::pow(pose_measure_uncertainty_time_ * vx * sin(ekf_yaw), 2.0);
    const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);
    R(0, 0) = std::pow(pose_stddev_x_, 2) + cov_pos_x; //  pos_x
    R(1, 1) = std::pow(pose_stddev_y_, 2) + cov_pos_y; //  pos_y
    R(2, 2) = std::pow(pose_stddev_yaw_, 2) + cov_yaw; //  yaw
  }

  /* In order to avoid a large change at the time of updating, measuremeent update is performed by dividing at every
   * step. */
  R *= (ekf_rate_ / pose_rate_);

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
void AWLocalizationNode::measurementUpdateTwist(const nav_msgs::msg::Odometry::SharedPtr msg)
{
 //  if (msg->header.frame_id != "base_link")
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
  double delay_time = (t_curr - msg->header.stamp).seconds() +
                      (t_curr - msg->header.stamp).nanoseconds() * 1e-9 +
                      twist_additional_delay_;
                      
  if (delay_time < 0.0)
  {
    // ROS_WARN_DELAYED_THROTTLE(1.0, "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].",
    //                           delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1)
  {
    // ROS_WARN_DELAYED_THROTTLE(1.0,
    //                           "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
    //                           "extend_state_step * ekf_dt : %f [s]",
    //                           delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  // DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << msg->twist.twist.linear.x, -msg->twist.twist.angular.z * 0.5;

  if (isnan(y.array()).any() || isinf(y.array()).any())
  {
    // ROS_WARN("[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX), ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y))
  {
    // ROS_WARN_DELAYED_THROTTLE(2.0, "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
    //                                "measurement data.");
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0; //  for vx
  C(1, IDX::WZ) = 1.0; //  for wz

  /* Set measurement noise covariancs */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (use_twist_with_covariance_)
  {
    R(0, 0) = current_twist_covariance_.at(0);  //  vx - vx
    R(0, 1) = current_twist_covariance_.at(5);  //  vx - wz
    R(1, 0) = current_twist_covariance_.at(30); //  wz - vx
    R(1, 1) = current_twist_covariance_.at(35); //  wz - wz
  }
  else
  {
    R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * ekf_dt_; //  for vx
    R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * ekf_dt_; //  for wz
  }

  /* In order to avoid a large change by update, measurement update is performed by dividing at every step. */
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
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov.inverse() * (x - obj_x);
  // DEBUG_INFO("measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(mahalanobis_squared(0)), dist_max);
  if (mahalanobis_squared(0) > dist_max * dist_max)
  {
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
  current_ekf_odom_.header.frame_id = pose_frame_id_;
  current_ekf_odom_.child_frame_id = "base_link";
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
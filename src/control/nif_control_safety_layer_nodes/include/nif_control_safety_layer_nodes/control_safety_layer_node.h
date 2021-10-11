//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#ifndef ROS2MASTER_CONTROL_SAFETY_LAYER_NODE_H
#define ROS2MASTER_CONTROL_SAFETY_LAYER_NODE_H

#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include <queue>

#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_control_common/control_command_compare.h"

#include <nif_control_common/PID.hpp>

#include "rclcpp/rclcpp.hpp"

namespace nif {
namespace control {

struct GearState {
    int gear;
    double gearRatio;
    double downshiftSpeed;
    double upshiftSpeed;
    GearState(int gear, double gearRatio, double downshiftSpeed,
              double upshiftSpeed) {
        this->gear = gear;
        this->gearRatio = gearRatio;
        this->downshiftSpeed = downshiftSpeed;
        this->upshiftSpeed = upshiftSpeed;
    }
};

/**
 * ControlSafetyLayerNode is responsible to send the final control message to
 * the car. It stores the control commands generated by one or more controllers,
 * and it submits the [best] available control to the vehicle interface. Being
 * subclass of IBaseSynchronizedNode, its run() function is called with a fixed
 * frequency.
 *
 * Its input is of type ControlCommand, which contains the set of controls that
 * the car interface can accept, but as an output each control value is also
 * sent separately.
 *
 */
class ControlSafetyLayerNode : public nif::common::IBaseSynchronizedNode {
public:

  /**
   * Initialize ControlSafetyNodeLayer with custom period.
   * @param node_name
   * @param options
   * @param period Custom synchronization period. It's passed to
   * IBaseSynchronizedNode and determines the frequency run() is called at.
   */template <class DurationRepT, class DurationT>
  ControlSafetyLayerNode(
      const std::string &node_name,
      const std::chrono::duration<DurationRepT, DurationT> period = common::constants::SYNC_PERIOD_DEFAULT_US,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})

      : IBaseSynchronizedNode(node_name, common::NodeType::CONTROL, period, options),
         control_buffer()

  {
    this->control_sub =
        this->create_subscription<nif::common::msgs::ControlCmd>(
            "in_control_cmd", nif::common::constants::QOS_CONTROL_CMD,
            std::bind(&ControlSafetyLayerNode::controlCallback, this,
                      std::placeholders::_1));

    this->control_override_sub =
        this->create_subscription<nif::common::msgs::ControlCmd>(
            "in_override_control_cmd", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE,
            std::bind(&ControlSafetyLayerNode::controlOverrideCallback, this,
                      std::placeholders::_1));

    this->wheel_speed_sub =
            this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
                    "/raptor_dbw_interface/wheel_speed_report", 1,
                    std::bind(&ControlSafetyLayerNode::receiveVelocity, this,
                              std::placeholders::_1));
    this->pt_report_sub =
            this->create_subscription<deep_orange_msgs::msg::PtReport>(
                    "/raptor_dbw_interface/pt_report", 1,
                    std::bind(&ControlSafetyLayerNode::receivePtReport, this,
                              std::placeholders::_1));

    this->perception_steering_sub =
            this->create_subscription<std_msgs::msg::Float32>(
                    "in_perception_steering", nif::common::constants::QOS_CONTROL_CMD,
                    std::bind(&ControlSafetyLayerNode::perceptionSteeringCallback, this,
                              std::placeholders::_1));                              

    this->control_pub = this->create_publisher<nif::common::msgs::ControlCmd>(
        "out_control_cmd",nif::common::constants::QOS_CONTROL_CMD);

    this->steering_control_pub =
        this->create_publisher<nif::common::msgs::ControlSteeringCmd>(
            "out_steering_control_cmd", nif::common::constants::QOS_CONTROL_CMD);

    this->accelerator_control_pub =
        this->create_publisher<nif::common::msgs::ControlAcceleratorCmd>(
            "out_accelerator_control_cmd", nif::common::constants::QOS_CONTROL_CMD);

    this->braking_control_pub =
        this->create_publisher<nif::common::msgs::ControlBrakingCmd>(
            "out_braking_control_cmd", nif::common::constants::QOS_CONTROL_CMD);

    this->gear_control_pub =
        this->create_publisher<nif::common::msgs::ControlGearCmd>(
            "out_gear_control_cmd", nif::common::constants::QOS_CONTROL_CMD);

    this->desired_acceleration_pub =
            this->create_publisher<std_msgs::msg::Float32>(
                    "out_desired_acceleration_cmd", nif::common::constants::QOS_CONTROL_CMD);

    this->diagnostic_hb_pub = this->create_publisher<std_msgs::msg::Int32>(
            "/diagnostics/heartbeat", 10);

    // Automatically boot with lat_autonomy_enabled
    this->declare_parameter("lat_autonomy_enabled", false);
    this->declare_parameter("long_autonomy_enabled", false);
    // Max Steering Angle in Degrees
    this->declare_parameter("max_steering_angle_deg", 20.0);
    // Degrees at which to automatically revert to the override
    this->declare_parameter("steering_auto_override_deg", 4.0);
    // convert from degress to steering units (should be 1 - 1 ?)
    this->declare_parameter("steering_units_multiplier", 1.0);

    // Safety timeouts for odometry and the path. If exceeded, toggle emergency.
    this->declare_parameter("odometry_timeout_sec", 0.1);
    this->declare_parameter("path_timeout_sec", 0.5);
    // Limit the max change in the steering signal over time
    this->declare_parameter("steering_max_ddeg_dt", 5.0);

    //  Invert steering command for simulation
    this->declare_parameter("invert_steering", false);

    this->declare_parameter("time_step", 0.01);
    this->declare_parameter("buffer_empty_counter_threshold", 10);

    this->declare_parameter("throttle.proportional_gain", 4.0);
    this->declare_parameter("throttle.integral_gain", 0.0);
    this->declare_parameter("throttle.derivative_gain", 0.0);
    this->declare_parameter("throttle.max_integrator_error", 10.0);
    this->declare_parameter("throttle.cmd_max", 40.0);
    this->declare_parameter("throttle.cmd_min", 0.0);
    this->declare_parameter("throttle.reset_integral_below_this_cmd", 15.0);

    this->declare_parameter("brake.proportional_gain", 200000.1);
    this->declare_parameter("brake.integral_gain", 0.0);
    this->declare_parameter("brake.derivative_gain", 0.0);
    this->declare_parameter("brake.max_integrator_error", 10.0);
    this->declare_parameter("brake.cmd_max", 2000000.7);
    this->declare_parameter("brake.cmd_min", 0.0);
    this->declare_parameter("brake.reset_integral_below_this_cmd", 100000.0);
    this->declare_parameter("brake.vel_error_deadband_mps", 0.5);

    this->declare_parameter("gear.shift_up", 4000.0);
    this->declare_parameter("gear.shift_down", 2200.0);
    this->declare_parameter("gear.shift_time_ms", 1000);

    this->declare_parameter("safe_des_vel.safe_vel_thres_mph", 30.0);
    this->declare_parameter("safe_des_vel.hard_braking_time", 1.5);
    this->declare_parameter("safe_des_vel.soft_braking_time", 1.0);

    this->declare_parameter("desired_acceleration.cmd_max", 5.0);

    // Read in misc. parameters
    max_steering_angle_deg =
        this->get_parameter("max_steering_angle_deg").as_double();
    steering_auto_override_deg =
        this->get_parameter("steering_auto_override_deg").as_double();
    steering_units_multiplier =
        this->get_parameter("steering_units_multiplier").as_double();

    odometry_timeout_sec =
        this->get_parameter("odometry_timeout_sec").as_double();

    path_timeout_sec =
        this->get_parameter("path_timeout_sec").as_double();

    steering_max_ddeg_dt =
        this->get_parameter("steering_max_ddeg_dt").as_double();

    if (  (odometry_timeout_sec) < 0. ||
    (path_timeout_sec) < 0.) {
      throw rclcpp::exceptions::InvalidParametersException("odometry_timeout_sec or path_timeout_sec parameter is negative.");
    }

    this->parameters_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&ControlSafetyLayerNode::parametersCallback, this, std::placeholders::_1));

//  EMERGENCY LONG CONTROL FEATURES
    this->initializeGears();

    this->safe_vel_thres_mph_ =
            this->get_parameter("safe_des_vel.safe_vel_thres_mph").as_double();
    this->hard_braking_time_ =
            this->get_parameter("safe_des_vel.hard_braking_time").as_double();
    this->soft_braking_time_ =
            this->get_parameter("safe_des_vel.soft_braking_time").as_double();
    this->gear_shift_time_ms =
            this->get_parameter("gear.shift_time_ms").as_int();

      if (gear_shift_time_ms <= 0)
          throw std::range_error("shift_time_ms must be greater than zero.");

    this->ts_ = this->get_parameter("time_step").as_double();
    this->buffer_empty_counter_threshold = this->get_parameter("buffer_empty_counter_threshold").as_int();

    // Create throttle PID object
    this->p_ = this->get_parameter("throttle.proportional_gain").as_double();
    this->i_ = this->get_parameter("throttle.integral_gain").as_double();
    this->d_ = this->get_parameter("throttle.derivative_gain").as_double();
    this->iMax_ =
            this->get_parameter("throttle.max_integrator_error").as_double();
    this->throttle_cmd_max = this->get_parameter("throttle.cmd_max").as_double();
    this->throttle_cmd_min = this->get_parameter("throttle.cmd_min").as_double();
    this->iThrottleReset_ =
            this->get_parameter("throttle.reset_integral_below_this_cmd").as_double();
    this->desired_acceleration_cmd_max = this->get_parameter("desired_acceleration.cmd_max").as_double();

    this->vel_pid_ =
            PID(p_, i_, d_, ts_, iMax_, throttle_cmd_max, throttle_cmd_min);

    // Create brake PID object
    this->bp_ = this->get_parameter("brake.proportional_gain").as_double();
    this->bi_ = this->get_parameter("brake.integral_gain").as_double();
    this->bd_ = this->get_parameter("brake.derivative_gain").as_double();
    this->biMax_ = this->get_parameter("brake.max_integrator_error").as_double();
    this->brakeCmdMax_ = this->get_parameter("brake.cmd_max").as_double();
    this->brakeCmdMin_ = this->get_parameter("brake.cmd_min").as_double();
    this->iBrakeReset_ =
            this->get_parameter("brake.reset_integral_below_this_cmd").as_double();

    this->brake_deadband = this->get_parameter("brake.vel_error_deadband_mps").as_double();

    this->brake_pid_ =
            PID(bp_, bi_, bd_, ts_, biMax_, brakeCmdMax_, brakeCmdMin_);

    this->control_cmd.accelerator_control_cmd.data = 0.0;
    this->control_cmd.steering_control_cmd.data = 0.0;
    this->control_cmd.braking_control_cmd.data = 0.0;
    this->control_cmd.gear_control_cmd.data = 1;

    this->setNodeStatus(common::NODE_INITIALIZED);
  }

private:
  /**
   * Check if msg is valid, then push it in the ControlCmd buffer.
   * @param msg
   */
  void bufferStore(nif::common::msgs::ControlCmd::SharedPtr msg);
  void bufferFlush();
  uint8_t getCommandsCount() const {
    return control_buffer.size();
  }
  // Prevent default constructor to be called from the outside
  ControlSafetyLayerNode();

  // emergency lane flag. Activated in case of emergency.
  bool emergency_lane_enabled = false;
  bool emergency_buffer_empty = false;

  // Automatically boot with lat_autonomy_enabled
  bool lat_autonomy_enabled;
  bool long_autonomy_enabled;

  // Max Steering Angle in Degrees
  double max_steering_angle_deg;

  // Degrees at which to automatically revert to the override
  double steering_auto_override_deg;

  // convert from degress to steering units (should be 1 - 1 ?)
  double steering_units_multiplier;

  // Safety timeouts for odometry and the path. If exceeded, toggle emergency.
  double odometry_timeout_sec;
  // Safety timeouts for odometry and the path. If exceeded, toggle emergency.
  double path_timeout_sec;

  // Maximum steering speed [deg/sec]
  double steering_max_ddeg_dt;

  // Maximum steering speed [deg/sec]
  bool invert_steering;

//OVERRIDE SIIGNALS
  nif::common::msgs::ControlCmd override_control_cmd;
  nif::common::msgs::ControlCmd last_control_cmd;
  nif::common::msgs::ControlCmd control_cmd;
  float perception_steering_cmd = 0.0;
  bool has_perception_steering = false;
  rclcpp::Time override_last_update;
  rclcpp::Time perception_steering_last_update;

  /**
   * Stores control commands coming from the controllers' stack. It's flushed at
   * every iteration by run(), that is it must store only the controls relative
   * to a time quantum.
   */
  std::priority_queue<nif::common::msgs::ControlCmd::SharedPtr,
                      std::vector<nif::common::msgs::ControlCmd::SharedPtr>,
                      nif::control::ControlCommandCompare> control_buffer;

  /**
   * Subscriber to the topic of control commands. Each incoming command is then
   * saved in the buffer (should check the age).
   */
  rclcpp::Subscription<nif::common::msgs::ControlCmd>::SharedPtr control_sub;

  /**
 * Subscriber to the 'privileged' topic of override control commands.
 * If over a certain threshold, this control command is used instead of the ones in the control buffer.
 */
  rclcpp::Subscription<nif::common::msgs::ControlCmd>::SharedPtr control_override_sub;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr perception_steering_sub;

  /**
   * Control publisher. Publishes the effective command to the vehicle interface
   * topic.
   */
  rclcpp::Publisher<nif::common::msgs::ControlCmd>::SharedPtr control_pub;

  /**
   * Steering Control publisher. Publishes the effective command to the vehicle
   * interface topic.
   */
  rclcpp::Publisher<nif::common::msgs::ControlSteeringCmd>::SharedPtr
      steering_control_pub;

  /**
   * Accelerator Control publisher. Publishes the effective command to the
   * vehicle interface topic.
   */
  rclcpp::Publisher<nif::common::msgs::ControlAcceleratorCmd>::SharedPtr
      accelerator_control_pub;

  /**
   * Braking Control publisher. Publishes the effective command to the vehicle
   * interface topic.
   */
  rclcpp::Publisher<nif::common::msgs::ControlBrakingCmd>::SharedPtr
      braking_control_pub;

  /**
   * Gear Control publisher. Publishes the effective command to the vehicle
   * interface topic.
   */
  rclcpp::Publisher<nif::common::msgs::ControlGearCmd>::SharedPtr
      gear_control_pub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    desired_acceleration_pub;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr diagnostic_hb_pub;

  std::map<int, std::shared_ptr<GearState>> gear_states;
  std::shared_ptr<control::GearState> curr_gear_ptr_;

  nif::control::PID vel_pid_;
  nif::control::PID brake_pid_;

  void controlCallback(const nif::common::msgs::ControlCmd::SharedPtr msg);
  void controlOverrideCallback(const nif::common::msgs::ControlCmd::UniquePtr msg);
  void perceptionSteeringCallback(const std_msgs::msg::Float32::UniquePtr msg);

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &vector);
  void run() override;

  bool publishSteeringCmd(const nif::common::msgs::ControlSteeringCmd &msg) const;
  bool publishAcceleratorCmd(common::msgs::ControlAcceleratorCmd msg) const;
  bool publishBrakingCmd(common::msgs::ControlBrakingCmd msg) const;
  bool publishGearCmd(const nif::common::msgs::ControlGearCmd &msg) const;
  bool publishDesiredAcceleration(std_msgs::msg::Float32 msg) const;

    void afterSystemStatusCallback() override;

    void initializeGears() {
        // LOR params
        // this->gear_states = {
        //    {1, std::make_shared<control::GearState>(1, 2.92, -255, 11)},
        //    {2, std::make_shared<control::GearState>(2, 1.875, 9.5, 16)},
        //    {3, std::make_shared<control::GearState>(3, 1.38, 14, 22)},
        //    {4, std::make_shared<control::GearState>(4, 1.5, 17, 30)},
        //    {5, std::make_shared<control::GearState>(5, 0.96, 22, 35)},
        //    {6, std::make_shared<control::GearState>(6, 0.889, 30, 255)}};

        // IMS params
        this->gear_states = {
                {1, std::make_shared<control::GearState>(1, 2.92, -255, 11)},
                {2, std::make_shared<control::GearState>(2, 1.875, 9.5, 22)},
                {3, std::make_shared<control::GearState>(3, 1.38, 19.5, 28.5)},
                {4, std::make_shared<control::GearState>(4, 1.5, 25, 37.5)},
                {5, std::make_shared<control::GearState>(5, 0.96, 35, 44)},
                {6, std::make_shared<control::GearState>(6, 0.889, 41.5, 255)}};

        this->curr_gear_ptr_ = this->gear_states[1];
    }

    int counter_hb = 0;

    int buffer_empty_counter = 0;
    int buffer_empty_counter_threshold = 20;

    double init_tick_ = -1.0;
    double init_vel_ = -1.0;
    double safe_braking_time_ = -1.0;

    double speed_mps_ = 0.0;
    double safe_vel_thres_mph_;
    double hard_braking_time_;
    double soft_braking_time_;
    double safe_des_vel_;
    double orig_des_vel_;

    unsigned int current_gear_ = 0;
    unsigned int engine_speed_ = 0;
    unsigned int gear_shift_time_ms;
    bool engine_running_ = false;
    unsigned int shifting_counter_ = 0;
    double ts_;

    double p_;
    double i_;
    double d_;
    double iMax_;
    double throttle_cmd_max;
    double throttle_cmd_min;
    double iThrottleReset_;
    double desired_acceleration_cmd_max;

    double bp_;
    double bi_;
    double bd_;
    double biMax_;
    double brakeCmdMax_ = 2000000.7;
    double brakeCmdMin_;
    double iBrakeReset_;
    double brake_deadband;

    bool has_vel = false;
    rclcpp::Time vel_recv_time_;

    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_sub;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr pt_report_sub;

    int getGearCmd(double throttle_cmd = 0.0) {
        // Uses joystick cmds if autonomous mode is not enabled
        double upshift_speed = this->curr_gear_ptr_->upshiftSpeed;
        double downshift_speed = this->curr_gear_ptr_->downshiftSpeed;
        unsigned int shift_time_limit = this->gear_shift_time_ms;
        int curr_gear_num = curr_gear_ptr_->gear;
        int commanded_gear = curr_gear_num;

        // grab current translational speed of the car
        double curr_speed = this->speed_mps_;

        // Sets command to current gear if engine is not on or shift attempts denied
        // over the limit
        if (!this->engine_running_ ||
        this->shifting_counter_ * 100 >= shift_time_limit) {
            commanded_gear = this->current_gear_;
            this->shifting_counter_ = 0;
            return commanded_gear;
        }

        // Determine if a shift is required
        if (curr_speed > upshift_speed && throttle_cmd > 0.0 &&
        curr_gear_num < 4) {
            // change to next gear if not in 4th
            curr_gear_ptr_ = this->gear_states[curr_gear_num + 1];
            commanded_gear = curr_gear_num + 1;
            this->shifting_counter_++;
            RCLCPP_INFO(this->get_logger(), "Shifting UP from %d to %d", curr_gear_num,
                        curr_gear_num + 1);
        } else if (curr_speed < downshift_speed && curr_gear_num > 1) {
            // downshift if not already in 1st
            curr_gear_ptr_ = this->gear_states[curr_gear_num - 1];
            commanded_gear = curr_gear_num - 1;
            this->shifting_counter_++;
            RCLCPP_INFO(this->get_logger(), "Shifting DOWN from %d to %d",
                        curr_gear_num, curr_gear_num - 1);
        } else {
            // if still within threshold maintain same gear
            commanded_gear = curr_gear_num;
            this->shifting_counter_ = 0;
        }

        return commanded_gear;
    }

    void receiveVelocity(
            const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
        const double kphToMps = 1.0 / 3.6;
        // front left wheel speed (kph)
        // double front_left = msg->front_left;
        // double front_right = msg->front_right;
        double rear_left = msg->rear_left;
        double rear_right = msg->rear_right;
        // average wheel speeds (kph) and convert to m/s
        this->speed_mps_ = (rear_left + rear_right) * 0.5 * kphToMps;
        this->has_vel = true;
        this->vel_recv_time_ = this->now();
    }

    void receivePtReport(
            const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
        this->current_gear_ = msg->current_gear;
        this->engine_speed_ = msg->engine_rpm;
        this->engine_running_ = (msg->engine_rpm > 500) ? true : false;
    }

    double emergencyVelocityError() {
        double safe_des_velocity = 0.0;
        auto now = this->now();
        double vel_error = - 27.78;

        if (this->has_vel && now - this->vel_recv_time_ < rclcpp::Duration(500000000)) {
            // NOMINAL
            rclcpp::Duration time_diff = this->now() - this->vel_recv_time_;
            double dt = static_cast<double>(time_diff.seconds()) +
                    static_cast<double>(time_diff.nanoseconds()) * 1e-9;

            if (dt > 100 * this->ts_) {
                this->vel_pid_.ResetErrorIntegral();
            }

        // Apply safe desired vel profiler for safe braking w.r.t. pose uncertainty
            safe_des_velocity = emergencyDesVelProfiler();
            vel_error = safe_des_velocity - this->speed_mps_;

        } else {
            // Ego speed is too old to be reliable, blind brake.
            RCLCPP_ERROR_ONCE(this->get_logger(), "Ego velocity is too old, CSL is blindly braking!");
        }
        return vel_error;
    }

    double calculateThrottleCmd(double vel_err) {
        if (vel_err > 0.0) {
            this->vel_pid_.Update(vel_err);
            return this->vel_pid_.CurrentControl();
        } else {
            return 0.0;
        }
    }

    double emergencyBrakeCmd(double vel_err) {
        this->brake_pid_.Update(-vel_err);
        if (this->speed_mps_ < 6.0)
        {
            return this->brakeCmdMax_;
        }
        return this->brake_pid_.CurrentControl();
    }

    double emergencyDesVelProfiler() {
        /*
        Safe desired velocity profiler w.r.t. pose uncertainty
        : Decrease desired velocity while the GPS uncertainty (cov) is larger than
        threshold
        @ Args
          - orig_des_vel : original desired velocity in ROS param
        @ Params
          - insstdev_threshold     : Threshold of the pose stdev
          - safe_vel_thres_mph   : Velocity (Mph) threshold w.r.t. safe braking time
          - hard_braking_time    : safe braking time when faster than
        safe_vel_thres_mph
          - soft_braking_time    : safe braking time when lower than
        safe_vel_thres_mph
        */
        const double mphToMps = 1.0 / 2.237;

        rclcpp::Time curr_time = this->now();
        double curr_stamp = static_cast<double>(curr_time.seconds()) +
                static_cast<double>(curr_time.nanoseconds()) * 1e-9;
        double curr_vel = this->speed_mps_;

        // Set (initial tick, vel) and (safe braking time, slope (braking accel))
        if (this->init_tick_ == -1.0 && this->init_vel_ == -1.0 &&
        this->safe_braking_time_ == -1.0) {
            this->init_tick_ = curr_stamp;
            this->init_vel_ = curr_vel;
            this->safe_braking_time_ = (curr_vel > safe_vel_thres_mph_ * mphToMps)
                    ? hard_braking_time_
                    : soft_braking_time_;
        }

        // Calculate decreasing desired vel
        double duration = curr_stamp - this->init_tick_;
        double safe_des_vel = this->init_vel_ - (this->init_vel_) /
                this->safe_braking_time_ *
                duration;
        // Saturation (safe_des_vel >= 1.0)
        safe_des_vel = (safe_des_vel > 1.5) ? safe_des_vel : 0.0;

        this->safe_des_vel_ = safe_des_vel;

        return safe_des_vel;
    }

    //  TODO define safety checks functions

};
} // namespace control
} // namespace nif

#endif // ROS2MASTER_CONTROL_SAFETY_LAYER_NODE_H

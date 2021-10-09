/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/mono_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera {

MonoCamera::MonoCamera(const std::string& node_name,
                       const rclcpp::NodeOptions& node_options)
  : Node(node_name, node_options) {
  // node handle sharing (for logging)
  node_handle_ = rclcpp::Node::SharedPtr(this);
  cam_.node_handle_ = this->node_handle_;
  api_.node_handle_ = this->node_handle_;

  node_ns_ = this->node_handle_->get_name();

  // Start Vimba & list all available cameras
  api_.start();

  RCLCPP_INFO(this->get_logger(), "avt_vimba_camera_node: %s", node_name);

  // set up config from parms
  frame_id_descriptor.description =
      "The optical camera TF frame set in message headers.";
  this->declare_parameter(
      "frame_id", rclcpp::ParameterValue("camera"), frame_id_descriptor);
  camera_ip_addr_descriptor.description = "The IP address for the camera.";
  this->declare_parameter("camera_ip_addr",
                          rclcpp::ParameterValue("10.0.0.1"),
                          camera_ip_addr_descriptor);
  trig_timestamp_topic_descriptor.description =
      "Sets the topic from which an externally trigged camera receives its "
      "trigger timestamps.";
  this->declare_parameter("trig_timestamp_topic",
                          rclcpp::ParameterValue(""),
                          trig_timestamp_topic_descriptor);
  acquisition_mode_descriptor.description = "Camera acquisition mode";
  this->declare_parameter("acquisition_mode",
                          rclcpp::ParameterValue("Continuous"),
                          acquisition_mode_descriptor);
  acquisition_rate_descriptor.description =
      "Sets the expected triggering rate in externally triggered mode.";
  this->declare_parameter("acquisition_rate",
                          rclcpp::ParameterValue(30),
                          acquisition_rate_descriptor);
  trigger_source_descriptor.description = "Camera trigger source";
  this->declare_parameter("trigger_source",
                          rclcpp::ParameterValue("FixedRate"),
                          trigger_source_descriptor);
  trigger_mode_descriptor.description = "Camera trigger mode";
  this->declare_parameter(
      "trigger_mode", rclcpp::ParameterValue("On"), trigger_mode_descriptor);
  trigger_selector_descriptor.description = "Camera trigger selector";
  this->declare_parameter("trigger_selector",
                          rclcpp::ParameterValue("FrameStart"),
                          trigger_selector_descriptor);
  trigger_activation_descriptor.description = "Camera trigger activation";
  this->declare_parameter("trigger_activation",
                          rclcpp::ParameterValue("RisingEdge"),
                          trigger_activation_descriptor);
  trigger_delay_descriptor.description =
      "Trigger delay in us (only valid when set to external trigger)";
  this->declare_parameter(
      "trigger_delay", rclcpp::ParameterValue(0.0), trigger_delay_descriptor);
  exposure_descriptor.description = "Camera exposure time in microseconds.";
  this->declare_parameter(
      "exposure", rclcpp::ParameterValue(50000), exposure_descriptor);
  exposure_auto_descriptor.description =
      "Sets the camera exposure. If continously automatic, causes the "
      "`~exposure` setting to be ignored.";
  this->declare_parameter("exposure_auto",
                          rclcpp::ParameterValue("Continuous"),
                          exposure_auto_descriptor);
  exposure_auto_alg_descriptor.description =
      "The following algorithms can be used to calculate auto exposure";
  this->declare_parameter("exposure_auto_alg",
                          rclcpp::ParameterValue("FitRange"),
                          exposure_auto_alg_descriptor);
  exposure_auto_tol_descriptor.description =
      "Tolerance in variation from ExposureAutoTarget in which the auto "
      "exposure algorithm will not respond.";
  this->declare_parameter("exposure_auto_tol",
                          rclcpp::ParameterValue(5),
                          exposure_auto_tol_descriptor);
  exposure_auto_max_descriptor.description =
      "The max exposure time in auto exposure mode, in microseconds.";
  this->declare_parameter("exposure_auto_max",
                          rclcpp::ParameterValue(50000),
                          exposure_auto_max_descriptor);
  exposure_auto_min_descriptor.description =
      "The min exposure time in auto exposure mode, in microseconds.";
  this->declare_parameter("exposure_auto_min",
                          rclcpp::ParameterValue(41),
                          exposure_auto_min_descriptor);
  exposure_auto_outliers_descriptor.description =
      "The total pixels from top of the distribution that are ignored by the "
      "auto exposure algorithm (0.01% increments)";
  this->declare_parameter("exposure_auto_outliers",
                          rclcpp::ParameterValue(0),
                          exposure_auto_outliers_descriptor);
  exposure_auto_rate_descriptor.description =
      "The rate at which the auto exposure function changes the exposure "
      "setting.100% is auto exposure adjustments running at full speed, and "
      "50% is half speed.";
  this->declare_parameter("exposure_auto_rate",
                          rclcpp::ParameterValue(100),
                          exposure_auto_rate_descriptor);
  exposure_auto_target_descriptor.description =
      "The auto exposure target mean value as a percentage, from 0=black to "
      "100=white.";
  this->declare_parameter("exposure_auto_target",
                          rclcpp::ParameterValue(50),
                          exposure_auto_target_descriptor);
  gain_descriptor.description = "The gain level in dB.";
  this->declare_parameter("gain", rclcpp::ParameterValue(0), gain_descriptor);
  gain_auto_descriptor.description =
      "Sets the analog gain. If continously automatic, causes the `~gain` "
      "setting to be ignored.";
  this->declare_parameter(
      "gain_auto", rclcpp::ParameterValue("Continuous"), gain_auto_descriptor);
  gain_auto_tol_descriptor.description =
      "Tolerance in variation from GainAutoTarget in which the auto exposure "
      "algorithm will not respond.";
  this->declare_parameter(
      "gain_auto_tol", rclcpp::ParameterValue(5), gain_auto_tol_descriptor);
  gain_auto_max_descriptor.description =
      "The max gain level in auto gain mode, in dB.";
  this->declare_parameter(
      "gain_auto_max", rclcpp::ParameterValue(32), gain_auto_max_descriptor);
  gain_auto_min_descriptor.description =
      "The min gain level in auto gain mode, in dB.";
  this->declare_parameter(
      "gain_auto_min", rclcpp::ParameterValue(0), gain_auto_min_descriptor);
  gain_auto_outliers_descriptor.description =
      "The total pixels from top of the distribution that are ignored by the "
      "auto gain algorithm (0.01% increments).";
  this->declare_parameter("gain_auto_outliers",
                          rclcpp::ParameterValue(0),
                          gain_auto_outliers_descriptor);
  gain_auto_rate_descriptor.description =
      "The rate percentage at which the auto gain function changes.";
  this->declare_parameter(
      "gain_auto_rate", rclcpp::ParameterValue(100), gain_auto_rate_descriptor);
  gain_auto_target_descriptor.description =
      "The general lightness or darkness of the auto gain feature. A "
      "percentage of maximum brightness.";
  this->declare_parameter("gain_auto_target",
                          rclcpp::ParameterValue(50),
                          gain_auto_target_descriptor);
  balance_ratio_abs_descriptor.description =
      "Adjusts the gain of the channel selected in the `~BalanceRatioSelector`";
  this->declare_parameter("balance_ratio_abs",
                          rclcpp::ParameterValue(1.0),
                          balance_ratio_abs_descriptor);
  balance_ratio_selector_descriptor.description =
      "Select the Red or Blue channel to adjust with `~BalanceRatioAbs`";
  this->declare_parameter("balance_ratio_selector",
                          rclcpp::ParameterValue("Red"),
                          balance_ratio_selector_descriptor);
  whitebalance_auto_descriptor.description =
      "Whether whitebalance will continuously adjust to the current scene. "
      "Causes the `~whitebalance_red` and `~whitebalance_blue` settings to be "
      "ignored.";
  this->declare_parameter("whitebalance_auto",
                          rclcpp::ParameterValue("Continuous"),
                          whitebalance_auto_descriptor);
  whitebalance_auto_tol_descriptor.description =
      "Tolerance allowed from the ideal white balance values";
  this->declare_parameter("whitebalance_auto_tol",
                          rclcpp::ParameterValue(5),
                          whitebalance_auto_tol_descriptor);
  whitebalance_auto_rate_descriptor.description =
      "Rate of white balance adjustments, from 1 (slowest) to 100 (fastest).";
  this->declare_parameter("whitebalance_auto_rate",
                          rclcpp::ParameterValue(100),
                          whitebalance_auto_rate_descriptor);
  binning_x_descriptor.description =
      "Number of pixels to bin together horizontally.";
  this->declare_parameter(
      "binning_x", rclcpp::ParameterValue(1), binning_x_descriptor);
  binning_y_descriptor.description =
      "Number of pixels to bin together vertically.";
  this->declare_parameter(
      "binning_y", rclcpp::ParameterValue(1), binning_y_descriptor);
  decimation_x_descriptor.description = "Number of decimation operations in x.";
  this->declare_parameter(
      "decimation_x", rclcpp::ParameterValue(1), decimation_x_descriptor);
  decimation_y_descriptor.description = "Number of decimation operations in y.";
  this->declare_parameter(
      "decimation_y", rclcpp::ParameterValue(1), decimation_y_descriptor);
  width_descriptor.description =
      "Width of the region of interest (0 for automatic).";
  this->declare_parameter(
      "width", rclcpp::ParameterValue(4096), width_descriptor);
  height_descriptor.description =
      "Height of the region of interest (0 for automatic).";
  this->declare_parameter(
      "height", rclcpp::ParameterValue(4096), height_descriptor);
  roi_width_descriptor.description = "X width of the region of interest.";
  this->declare_parameter(
      "roi_width", rclcpp::ParameterValue(0), roi_width_descriptor);
  roi_height_descriptor.description = "Y height of the region of interest.";
  this->declare_parameter(
      "roi_height", rclcpp::ParameterValue(0), roi_height_descriptor);
  roi_offset_x_descriptor.description = "X offset of the region of interest.";
  this->declare_parameter(
      "roi_offset_x", rclcpp::ParameterValue(0), roi_offset_x_descriptor);
  roi_offset_y_descriptor.description = "Y offset of the region of interest.";
  this->declare_parameter(
      "roi_offset_y", rclcpp::ParameterValue(0), roi_offset_y_descriptor);
  pixel_format_descriptor.description = "Format of the image data.";
  this->declare_parameter(
      "pixel_format", rclcpp::ParameterValue("Mono8"), pixel_format_descriptor);
  stream_bytes_per_second_descriptor.description =
      "Limits the data rate of the camera.";
  this->declare_parameter("stream_bytes_per_second",
                          rclcpp::ParameterValue(45000000),
                          stream_bytes_per_second_descriptor);
  ptp_mode_descriptor.description =
      "Controls the PTP behavior of the clock port.";
  this->declare_parameter(
      "ptp_mode", rclcpp::ParameterValue("Off"), ptp_mode_descriptor);
  sync_in_selector_descriptor.description =
      "Selects the sync-out line to control";
  this->declare_parameter("sync_in_selector",
                          rclcpp::ParameterValue("SyncIn1"),
                          sync_in_selector_descriptor);
  sync_out_polarity_descriptor.description =
      "Polarity applied to the sync-out line specified by `sync_out_selector`";
  this->declare_parameter("sync_out_polarity",
                          rclcpp::ParameterValue("Normal"),
                          sync_out_polarity_descriptor);
  sync_out_selector_descriptor.description =
      "Selects the sync-out line to control";
  this->declare_parameter("sync_out_selector",
                          rclcpp::ParameterValue("SyncOut1"),
                          sync_out_selector_descriptor);
  sync_out_source_descriptor.description =
      "Signal source of the sync-out line specified by `sync_out_selector`";
  this->declare_parameter("sync_out_source",
                          rclcpp::ParameterValue("GPO"),
                          sync_out_source_descriptor);
  iris_auto_target_descriptor.description =
      "This is the target image mean value, in percent.";
  // this->declare_parameter("iris_auto_target", rclcpp::ParameterValue(50),
  // iris_auto_target_descriptor); iris_mode_descriptor.description = "Set the
  // iris mode. Disabled: no iris control. Video: enable video iris. VideoOpen:
  // fully open a video iris. VideoClose: fully close a video iris.";
  // this->declare_parameter("iris_mode", rclcpp::ParameterValue("Continuous"),
  // iris_mode_descriptor); iris_video_level_min_descriptor.description =
  // "Minimum video iris level output by the camera, in approximately mV pp. A
  // higher minimum value slows the adjustment time but prevents excessive
  // overshoot."; this->declare_parameter("iris_video_level_min",
  // rclcpp::ParameterValue(110), iris_video_level_min_descriptor);
  // iris_video_level_max_descriptor.description = "Maximum video iris level
  // output by the camera, in approximately mV pp. A lower minimum value slows
  // the adjustment time but prevents excessive overshoot.";
  // this->declare_parameter("iris_video_level_max",
  // rclcpp::ParameterValue(110), iris_video_level_max_descriptor);
  // use_ros_timestamp_descriptor.description = "By default use ROS time as the
  // timestamp for image frames. Use the camera time if 'False'";
  this->declare_parameter("use_ros_timestamp",
                          rclcpp::ParameterValue("True"),
                          use_ros_timestamp_descriptor);

  // callback for parms
  this->set_on_parameters_set_callback(
      std::bind(&MonoCamera::parametersCallback, this, std::placeholders::_1));

  // create an image publisher w/QoS profile
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = 6; // TEST
  camera_info_pub_ = image_transport::create_camera_publisher(
      this, this->node_ns_ + "/image", custom_qos_profile);
  info_man_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  last_frame_ = std::chrono::steady_clock::now();

  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCamera::frameCallback,
                             this,
                             std::placeholders::_1));

  /* [neil-rti] FIXME: I'm not yet certain of the right way to get
   * preloaded-from-yaml parms into the camera configuration at startup.  The
   * call below will fail if the yaml file isn't used or doesn't line up
   * perfectly.
   */
  parametersCallback(
      this->get_parameters({"height",
                            "width",
                            "camera_ip_addr",
                            "acquisition_mode",
                            "acquisition_rate",
                            "balance_ratio_abs",
                            "balance_ratio_selector",
                            "binning_x",
                            "binning_y",
                            "decimation_x",
                            "decimation_y",
                            "exposure",
                            "exposure_auto",
                            "exposure_auto_alg",
                            "exposure_auto_max",
                            "exposure_auto_min",
                            "exposure_auto_outliers",
                            "exposure_auto_rate",
                            "exposure_auto_target",
                            "exposure_auto_tol",
                            "frame_id",
                            "gain",
                            "gain_auto",
                            "gain_auto_max",
                            "gain_auto_min",
                            "gain_auto_outliers",
                            "gain_auto_rate",
                            "gain_auto_target",
                            "gain_auto_tol",
                            // "iris_auto_target", "iris_mode",
                            // "iris_video_level_max", "iris_video_level_min",
                            "pixel_format",
                            "ptp_mode",
                            "roi_height",
                            "roi_offset_x",
                            "roi_offset_y",
                            "roi_width",
                            "stream_bytes_per_second",
                            "sync_in_selector",
                            "sync_out_polarity",
                            "sync_out_selector",
                            "sync_out_source",
                            "trig_timestamp_topic",
                            "trigger_activation",
                            "trigger_delay",
                            "trigger_mode",
                            "trigger_selector",
                            "trigger_source",
                            "use_ros_timestamp",
                            "use_sim_time",
                            "whitebalance_auto",
                            "whitebalance_auto_rate",
                            "whitebalance_auto_tol"}));

  // Start dynamic_reconfigure & run configure()
  // ros1
  // reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::configure,
  // this, _1, _2));

  // NOTE : configure the camera only once -- comment out
  // configure(camera_config_, 0);
}

MonoCamera::~MonoCamera(void) {
  api_.shutdown();
  // ros1 pub_.shutdown();
}

/**
 * parametersCallback()
 * Callback function for parameter settings from other apps, such
 * as 'ros2 param set <node_name> <parm_name> <new_value>'
 **/
rcl_interfaces::msg::SetParametersResult MonoCamera::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto& parameter : parameters) {
    if (parameter.get_name() == "frame_id" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.frame_id_ = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'frame_id' changed to: %s",
                  camera_config_.frame_id_.c_str());
    } else if (parameter.get_name() == "camera_ip_addr" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.camera_ip_addr = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'camera_ip_addr' changed to: %s",
                  camera_config_.camera_ip_addr.c_str());
    } else if (parameter.get_name() == "trig_timestamp_topic" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.trig_timestamp_topic = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trig_timestamp_topic' changed to: %s",
                  camera_config_.trig_timestamp_topic.c_str());
    } else if (parameter.get_name() == "acquisition_mode" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.acquisition_mode = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'acquisition_mode' changed to: %s",
                  camera_config_.acquisition_mode.c_str());
    } else if (parameter.get_name() == "acquisition_rate" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.acquisition_rate = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'acquisition_rate' changed to: %f",
                  camera_config_.acquisition_rate);
    } else if (parameter.get_name() == "trigger_source" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.trigger_source = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trigger_source' changed to: %s",
                  camera_config_.trigger_source.c_str());
    } else if (parameter.get_name() == "trigger_mode" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.trigger_mode = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trigger_mode' changed to: %s",
                  camera_config_.trigger_mode.c_str());
    } else if (parameter.get_name() == "trigger_selector" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.trigger_selector = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trigger_selector' changed to: %s",
                  camera_config_.trigger_selector.c_str());
    } else if (parameter.get_name() == "trigger_activation" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.trigger_activation = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trigger_activation' changed to: %s",
                  camera_config_.trigger_activation.c_str());
    } else if (parameter.get_name() == "trigger_delay" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.trigger_delay = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'trigger_delay' changed to: %f",
                  camera_config_.trigger_delay);
    } else if (parameter.get_name() == "exposure" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.exposure = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure' changed to: %f",
                  camera_config_.exposure);
    } else if (parameter.get_name() == "exposure_auto" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.exposure_auto = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto' changed to: %s",
                  camera_config_.exposure_auto.c_str());
    } else if (parameter.get_name() == "exposure_auto_alg" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.exposure_auto_alg = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_alg' changed to: %s",
                  camera_config_.exposure_auto_alg.c_str());
    } else if (parameter.get_name() == "exposure_auto_tol" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_tol = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_tol' changed to: %d",
                  camera_config_.exposure_auto_tol);
    } else if (parameter.get_name() == "exposure_auto_max" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_max = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_max' changed to: %d",
                  camera_config_.exposure_auto_max);
    } else if (parameter.get_name() == "exposure_auto_min" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_min = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_min' changed to: %d",
                  camera_config_.exposure_auto_min);
    } else if (parameter.get_name() == "exposure_auto_outliers" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_outliers = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_outliers' changed to: %d",
                  camera_config_.exposure_auto_outliers);
    } else if (parameter.get_name() == "exposure_auto_rate" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_rate = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_rate' changed to: %d",
                  camera_config_.exposure_auto_rate);
    } else if (parameter.get_name() == "exposure_auto_target" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.exposure_auto_target = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'exposure_auto_target' changed to: %d",
                  camera_config_.exposure_auto_target);
    } else if (parameter.get_name() == "gain" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.gain = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain' changed to: %f",
                  camera_config_.gain);
    } else if (parameter.get_name() == "gain_auto" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.gain_auto = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto' changed to: %s",
                  camera_config_.gain_auto.c_str());
    } else if (parameter.get_name() == "gain_auto_tol" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.gain_auto_tol = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_tol' changed to: %d",
                  camera_config_.gain_auto_tol);
    } else if (parameter.get_name() == "gain_auto_max" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.gain_auto_max = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_max' changed to: %f",
                  camera_config_.gain_auto_max);
    } else if (parameter.get_name() == "gain_auto_min" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.gain_auto_min = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_min' changed to: %f",
                  camera_config_.gain_auto_min);
    } else if (parameter.get_name() == "gain_auto_outliers" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.gain_auto_outliers = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_outliers' changed to: %d",
                  camera_config_.gain_auto_outliers);
    } else if (parameter.get_name() == "gain_auto_rate" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.gain_auto_rate = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_rate' changed to: %d",
                  camera_config_.gain_auto_rate);
    } else if (parameter.get_name() == "gain_auto_target" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.gain_auto_target = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'gain_auto_target' changed to: %d",
                  camera_config_.gain_auto_target);
    } else if (parameter.get_name() == "balance_ratio_abs" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_DOUBLE) {
      camera_config_.balance_ratio_abs = parameter.as_double();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'balance_ratio_abs' changed to: %f",
                  camera_config_.balance_ratio_abs);
    } else if (parameter.get_name() == "balance_ratio_selector" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.balance_ratio_selector = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'balance_ratio_selector' changed to: %s",
                  camera_config_.balance_ratio_selector.c_str());
    } else if (parameter.get_name() == "whitebalance_auto" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.whitebalance_auto = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'whitebalance_auto' changed to: %s",
                  camera_config_.whitebalance_auto.c_str());
    } else if (parameter.get_name() == "whitebalance_auto_tol" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.whitebalance_auto_tol = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'whitebalance_auto_tol' changed to: %d",
                  camera_config_.whitebalance_auto_tol);
    } else if (parameter.get_name() == "whitebalance_auto_rate" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.whitebalance_auto_rate = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'whitebalance_auto_rate' changed to: %d",
                  camera_config_.whitebalance_auto_rate);
    } else if (parameter.get_name() == "binning_x" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.binning_x = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'binning_x' changed to: %d",
                  camera_config_.binning_x);
    } else if (parameter.get_name() == "binning_y" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.binning_y = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'binning_y' changed to: %d",
                  camera_config_.binning_y);
    } else if (parameter.get_name() == "decimation_x" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.decimation_x = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'decimation_x' changed to: %d",
                  camera_config_.decimation_x);
    } else if (parameter.get_name() == "decimation_y" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.decimation_y = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'decimation_y' changed to: %d",
                  camera_config_.decimation_y);
    } else if (parameter.get_name() == "width" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.width = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'width' changed to: %d",
                  camera_config_.width);
    } else if (parameter.get_name() == "height" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.height = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'height' changed to: %d",
                  camera_config_.height);
    } else if (parameter.get_name() == "roi_width" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.roi_width = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'roi_width' changed to: %d",
                  camera_config_.roi_width);
    } else if (parameter.get_name() == "roi_height" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.roi_height = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'roi_height' changed to: %d",
                  camera_config_.roi_height);
    } else if (parameter.get_name() == "roi_offset_x" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.roi_offset_x = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'roi_offset_x' changed to: %d",
                  camera_config_.roi_offset_x);
    } else if (parameter.get_name() == "roi_offset_y" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.roi_offset_y = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'roi_offset_y' changed to: %d",
                  camera_config_.roi_offset_y);
    } else if (parameter.get_name() == "pixel_format" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.pixel_format = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'pixel_format' changed to: %s",
                  camera_config_.pixel_format.c_str());
    } else if (parameter.get_name() == "stream_bytes_per_second" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.stream_bytes_per_second = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'stream_bytes_per_second' changed to: %d",
                  camera_config_.stream_bytes_per_second);
    } else if (parameter.get_name() == "ptp_mode" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.ptp_mode = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'ptp_mode' changed to: %s",
                  camera_config_.ptp_mode.c_str());
    } else if (parameter.get_name() == "sync_in_selector" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.sync_in_selector = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'sync_in_selector' changed to: %s",
                  camera_config_.sync_in_selector.c_str());
    } else if (parameter.get_name() == "sync_out_polarity" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.sync_out_polarity = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'sync_out_polarity' changed to: %s",
                  camera_config_.sync_out_polarity.c_str());
    } else if (parameter.get_name() == "sync_out_selector" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.sync_out_selector = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'sync_out_selector' changed to: %s",
                  camera_config_.sync_out_selector.c_str());
    } else if (parameter.get_name() == "sync_out_source" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.sync_out_source = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'sync_out_source' changed to: %s",
                  camera_config_.sync_out_source.c_str());
    } else if (parameter.get_name() == "iris_auto_target" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.iris_auto_target = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'iris_auto_target' changed to: %d",
                  camera_config_.iris_auto_target);
    } else if (parameter.get_name() == "iris_mode" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.iris_mode = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'iris_mode' changed to: %s",
                  camera_config_.iris_mode.c_str());
    } else if (parameter.get_name() == "iris_video_level_min" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.iris_video_level_min = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'iris_video_level_min' changed to: %d",
                  camera_config_.iris_video_level_min);
    } else if (parameter.get_name() == "iris_video_level_max" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_INTEGER) {
      camera_config_.iris_video_level_max = parameter.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'iris_video_level_max' changed to: %d",
                  camera_config_.iris_video_level_max);
    } else if (parameter.get_name() == "use_ros_timestamp" &&
               parameter.get_type() ==
                   rclcpp::ParameterType::PARAMETER_STRING) {
      camera_config_.use_ros_timestamp = parameter.as_string();
      RCLCPP_INFO(this->get_logger(),
                  "Parameter 'use_ros_timestamp' changed to: %s",
                  camera_config_.use_ros_timestamp.c_str());
    }
  }

  // update the camera
  configure(camera_config_, 0);
  return result;
}

void MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  rclcpp::Time timestamp = ros_clock_.now();
  rclcpp::Time tic = ros_clock_.now();
  // std::chrono::time_point<std::chrono::system_clock,
  // std::chrono::nanoseconds> systemTime;
  auto systemTime = std::chrono::time_point_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now());
  auto systDur = std::chrono::duration_cast<std::chrono::nanoseconds>(
      systemTime.time_since_epoch());

  if (camera_config_.use_ros_timestamp == "False") {
    // get the timestamp from the image frame
    VmbUint64_t ts_cam;
    vimba_frame_ptr->GetTimestamp(ts_cam);
    rclcpp::Time cameraTimestamp(ts_cam);
    RCLCPP_DEBUG(
        this->get_logger(), "\nROS timestamp:\t\t%s", timestamp.nanoseconds());
    RCLCPP_DEBUG(
        this->get_logger(), "\nSystem timestamp:\t\t%s", systDur.count());
    RCLCPP_DEBUG(this->get_logger(),
                 "\nCamera timestamp:\t\t%s",
                 cameraTimestamp.nanoseconds());

    timestamp = cameraTimestamp;
  }
  // VmbUint64_t fi_cam;
  // vimba_frame_ptr->GetFrameID(fi_cam);
  // This if is commented out because it's not  working reliably
  // if (camera_info_pub_.getNumSubscribers() > 0) {

  sensor_msgs::msg::Image img;
  if (api_.frameToImage(vimba_frame_ptr, img)) {
    sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();
    ci.header.stamp = img.header.stamp = timestamp;
    img.header.frame_id = ci.header.frame_id;
    // img.height = ci.height;
    // img.width = ci.width;
    camera_info_pub_.publish(img, ci);
    // } else {
    // RCLCPP_WARN(this->get_logger(), "Function frameToImage returned 0. No
    // image published.");
    // }
  }

  // updater_.update();
}

/** Dynamic reconfigure callback
 *
 *  Called immediately when callback first defined. Called again
 *  when dynamic reconfigure starts or changes a parameter value.
 *
 *  @param newconfig new Config values
 *  @param level bit-wise OR of reconfiguration levels for all
 *               changed parameters (0xffffffff on initial call)
 **/
void MonoCamera::configure(Config& newconfig, uint32_t level) {
  try {
    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id_ == "") {
      newconfig.frame_id_ = "camera";
    }

    // see if ip addr has changed
    if (newconfig.camera_ip_addr != ip_) {
      ip_ = newconfig.camera_ip_addr;
    }

    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!cam_.isOpened()) {
      cam_.start(ip_, guid_, show_debug_prints_);
    }

    Config config = newconfig;
    cam_.updateConfig(newconfig);
    updateCameraInfo(config);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Error reconfiguring mono_camera node : " << e.what());
  }
}

void MonoCamera::updateCameraInfo(
    const avt_vimba_camera::AvtVimbaCameraConfig& config) {
  // Get camera_info from the manager
  sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();

  // Set the frame id
  ci.header.frame_id = config.frame_id_;

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  // Set the operational parameters in CameraInfo (binning, ROI)
  ci.height = config.height;
  ci.width = config.width;
  ci.binning_x = binning_or_decimation_x;
  ci.binning_y = binning_or_decimation_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  ci.roi.x_offset = config.roi_offset_x;
  ci.roi.y_offset = config.roi_offset_y;
  ci.roi.height = config.roi_height;
  ci.roi.width = config.roi_width;

  // set the new URL and load CameraInfo (if any) from it
  std::string camera_info_url;
  // ros1 nhp_.getParam("camera_info_url", camera_info_url);
  if (camera_info_url != camera_info_url_) {
    info_man_->setCameraName(config.frame_id_);
    if (info_man_->validateURL(camera_info_url)) {
      info_man_->loadCameraInfo(camera_info_url);
      ci = info_man_->getCameraInfo();
    } else {
      RCLCPP_WARN_STREAM(this->get_logger(),
                         "Camera info URL not valid: " << camera_info_url);
    }
  }

  bool roiMatchesCalibration =
      (ci.height == config.roi_height && ci.width == config.roi_width);
  bool resolutionMatchesCalibration =
      (ci.width == config.width && ci.height == config.height);
  // check
  ci.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;

  // push the changes to manager
  info_man_->setCameraInfo(ci);
}
} // namespace avt_vimba_camera

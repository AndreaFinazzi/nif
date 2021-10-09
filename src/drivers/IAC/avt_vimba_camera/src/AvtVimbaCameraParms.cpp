/*
  AvtVimbaCameraParms.cpp
  Converted from the ROS1 drivers' AvtVimbaCamera.cfg file, for use w/ROS2
 */
/*
 Copyright (c) 2020 Neil Puthuff
 Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "avt_vimba_camera/AvtVimbaCameraParms.h"

namespace avt_vimba_camera {
AvtVimbaCameraParms::AvtVimbaCameraParms(rclcpp::Node::SharedPtr& node_handle) {
  nh_ = node_handle;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;

  // #       Name                    Type      Reconfiguration level Description
  // Default   Min   Max # ROS gen.add("frame_id",             str_t,
  // SensorLevels.RECONFIGURE_RUNNING, "The optical camera TF frame set in
  // message headers.", "camera") rcl_interfaces::msg::ParameterDescriptor
  // frame_id_descriptor;
  frame_id_descriptor.description =
      "The optical camera TF frame set in message headers.";
  frame_id_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "frame_id", rclcpp::ParameterValue("camera"), frame_id_descriptor);

  // gen.add("trig_timestamp_topic", str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Sets the topic from which an externally trigged camera receives its
  // trigger timestamps.", "")
  rcl_interfaces::msg::ParameterDescriptor trig_timestamp_topic_descriptor;
  trig_timestamp_topic_descriptor.description =
      "Sets the topic from which an externally trigged camera receives its "
      "trigger timestamps.";
  trig_timestamp_topic_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("trig_timestamp_topic",
                         rclcpp::ParameterValue(""),
                         trig_timestamp_topic_descriptor);

  // # ACQUISITION
  // gen.add("acquisition_mode",     str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Camera acquisition mode", "Continuous", edit_method =
  // acquisition_mode_enum)
  rcl_interfaces::msg::ParameterDescriptor acquisition_mode_descriptor;
  acquisition_mode_descriptor.description = "Camera acquisition mode";
  acquisition_mode_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("acquisition_mode",
                         rclcpp::ParameterValue("Continuous"),
                         acquisition_mode_descriptor);

  // gen.add("acquisition_rate",     double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "Sets the expected triggering rate in externally triggered mode.", 2, 1,
  // 30)
  rcl_interfaces::msg::ParameterDescriptor acquisition_rate_descriptor;
  acquisition_rate_descriptor.description =
      "Sets the expected triggering rate in externally triggered mode.";
  acquisition_rate_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("acquisition_rate",
                         rclcpp::ParameterValue(30),
                         acquisition_rate_descriptor);

  // # TRIGGER
  // gen.add("trigger_source",       str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Camera trigger source", "FixedRate", edit_method = trigger_source_enum)
  rcl_interfaces::msg::ParameterDescriptor trigger_source_descriptor;
  trigger_source_descriptor.description = "Camera trigger source";
  trigger_source_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("trigger_source",
                         rclcpp::ParameterValue("FixedRate"),
                         trigger_source_descriptor);
  // gen.add("trigger_mode",			    str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Camera trigger mode", "On", edit_method = trigger_mode_enum)
  rcl_interfaces::msg::ParameterDescriptor trigger_mode_descriptor;
  trigger_mode_descriptor.description = "Camera trigger mode";
  trigger_mode_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "trigger_mode", rclcpp::ParameterValue("On"), trigger_mode_descriptor);
  // gen.add("trigger_selector",     str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Camera trigger selector", "FrameStart", edit_method =
  // trigger_selector_enum)
  rcl_interfaces::msg::ParameterDescriptor trigger_selector_descriptor;
  trigger_selector_descriptor.description = "Camera trigger selector";
  trigger_selector_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("trigger_selector",
                         rclcpp::ParameterValue("FrameStart"),
                         trigger_selector_descriptor);
  // gen.add("trigger_activation",   str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Camera trigger activation", "RisingEdge", edit_method =
  // trigger_activation_enum)
  rcl_interfaces::msg::ParameterDescriptor trigger_activation_descriptor;
  trigger_activation_descriptor.description = "Camera trigger activation";
  trigger_activation_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("trigger_activation",
                         rclcpp::ParameterValue("RisingEdge"),
                         trigger_activation_descriptor);
  // gen.add("trigger_delay",   		  double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "Trigger delay in us (only valid when set to external trigger)", 0.0, 0.0,
  // 60000000.0)
  rcl_interfaces::msg::ParameterDescriptor trigger_delay_descriptor;
  trigger_delay_descriptor.description =
      "Trigger delay in us (only valid when set to external trigger)";
  trigger_delay_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "trigger_delay", rclcpp::ParameterValue(0.0), trigger_delay_descriptor);
  // # EXPOSURE
  // gen.add("exposure",             double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "Camera exposure time in microseconds.", 50000, 41, 60000000)
  rcl_interfaces::msg::ParameterDescriptor exposure_descriptor;
  exposure_descriptor.description = "Camera exposure time in microseconds.";
  exposure_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "exposure", rclcpp::ParameterValue(50000), exposure_descriptor);
  // gen.add("exposure_auto",        str_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Sets the camera exposure. If continously automatic, causes the `~exposure`
  // setting to be ignored.", "Continuous", edit_method = auto_enum)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_descriptor;
  exposure_auto_descriptor.description =
      "Sets the camera exposure. If continously automatic, causes the "
      "`~exposure` setting to be ignored.";
  exposure_auto_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto",
                         rclcpp::ParameterValue("Continuous"),
                         exposure_auto_descriptor);
  // gen.add("exposure_auto_alg",    str_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "The following algorithms can be used to calculate auto exposure",
  // "FitRange", edit_method = exposure_alg_enum)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_alg_descriptor;
  exposure_auto_alg_descriptor.description =
      "The following algorithms can be used to calculate auto exposure";
  exposure_auto_alg_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_alg",
                         rclcpp::ParameterValue("FitRange"),
                         exposure_auto_alg_descriptor);
  // gen.add("exposure_auto_tol", 	  int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Tolerance in variation from ExposureAutoTarget in which the auto exposure
  // algorithm will not respond.", 5, 0, 50)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_tol_descriptor;
  exposure_auto_tol_descriptor.description =
      "Tolerance in variation from ExposureAutoTarget in which the auto "
      "exposure algorithm will not respond.";
  exposure_auto_tol_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_tol",
                         rclcpp::ParameterValue(5),
                         exposure_auto_tol_descriptor);
  // gen.add("exposure_auto_max",    int_t, 	  SensorLevels.RECONFIGURE_RUNNING,
  // "The max exposure time in auto exposure mode, in microseconds.", 50000, 41,
  // 60000000)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_max_descriptor;
  exposure_auto_max_descriptor.description =
      "he max exposure time in auto exposure mode, in microseconds.";
  exposure_auto_max_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_max",
                         rclcpp::ParameterValue(50000),
                         exposure_auto_max_descriptor);
  // gen.add("exposure_auto_min",    int_t, 	  SensorLevels.RECONFIGURE_RUNNING,
  // "The min exposure time in auto exposure mode, in microseconds.", 41, 41,
  // 60000000)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_min_descriptor;
  exposure_auto_min_descriptor.description =
      "The min exposure time in auto exposure mode, in microseconds.";
  exposure_auto_min_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_min",
                         rclcpp::ParameterValue(41),
                         exposure_auto_min_descriptor);
  // gen.add("exposure_auto_outliers",int_t,	  SensorLevels.RECONFIGURE_RUNNING,
  // "The total pixels from top of the distribution that are ignored by the auto
  // exposure algorithm (0.01% increments)", 0, 0, 10000)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_outliers_descriptor;
  exposure_auto_outliers_descriptor.description =
      "The total pixels from top of the distribution that are ignored by the "
      "auto exposure algorithm (0.01% increments)";
  exposure_auto_outliers_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_outliers",
                         rclcpp::ParameterValue(0),
                         exposure_auto_outliers_descriptor);
  // gen.add("exposure_auto_rate",   int_t, 	  SensorLevels.RECONFIGURE_RUNNING,
  // "The rate at which the auto exposure function changes the exposure
  // setting.100% is auto exposure adjustments running at full speed, and 50% is
  // half speed.", 100, 0, 100)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_rate_descriptor;
  exposure_auto_rate_descriptor.description =
      "The rate at which the auto exposure function changes the exposure "
      "setting.100% is auto exposure adjustments running at full speed, and "
      "50% is half speed.";
  exposure_auto_rate_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_rate",
                         rclcpp::ParameterValue(100),
                         exposure_auto_rate_descriptor);
  // gen.add("exposure_auto_target", int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "The auto exposure target mean value as a percentage, from 0=black to
  // 100=white.", 50, 0, 100)
  rcl_interfaces::msg::ParameterDescriptor exposure_auto_target_descriptor;
  exposure_auto_target_descriptor.description =
      "The auto exposure target mean value as a percentage, from 0=black to "
      "100=white.";
  exposure_auto_target_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("exposure_auto_target",
                         rclcpp::ParameterValue(50),
                         exposure_auto_target_descriptor);
  // # GAIN
  // gen.add("gain",                 double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "The gain level in dB.", 0, 0, 32)
  rcl_interfaces::msg::ParameterDescriptor gain_descriptor;
  gain_descriptor.description = "The gain level in dB.";
  gain_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("gain", rclcpp::ParameterValue(0), gain_descriptor);
  // gen.add("gain_auto",            str_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Sets the analog gain. If continously automatic, causes the `~gain` setting
  // to be ignored.", "Continuous", edit_method = auto_enum)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_descriptor;
  gain_auto_descriptor.description =
      "Sets the analog gain. If continously automatic, causes the `~gain` "
      "setting to be ignored.";
  gain_auto_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "gain_auto", rclcpp::ParameterValue("Continuous"), gain_auto_descriptor);
  // gen.add("gain_auto_tol", 		    int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Tolerance in variation from GainAutoTarget in which the auto exposure
  // algorithm will not respond.", 5, 0, 50)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_tol_descriptor;
  gain_auto_tol_descriptor.description =
      "Tolerance in variation from GainAutoTarget in which the auto exposure "
      "algorithm will not respond.";
  gain_auto_tol_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "gain_auto_tol", rclcpp::ParameterValue(5), gain_auto_tol_descriptor);
  // gen.add("gain_auto_max",        double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "The max gain level in auto gain mode, in dB.", 32, 0, 32)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_max_descriptor;
  gain_auto_max_descriptor.description =
      "The max gain level in auto gain mode, in dB.";
  gain_auto_max_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "gain_auto_max", rclcpp::ParameterValue(32), gain_auto_max_descriptor);
  // gen.add("gain_auto_min",        double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "The min gain level in auto gain mode, in dB.", 0, 0, 32)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_min_descriptor;
  gain_auto_min_descriptor.description =
      "The min gain level in auto gain mode, in dB.";
  gain_auto_min_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "gain_auto_min", rclcpp::ParameterValue(0), gain_auto_min_descriptor);
  // gen.add("gain_auto_outliers",   int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "The total pixels from top of the distribution that are ignored by the auto
  // gain algorithm (0.01% increments).", 0, 0, 10000)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_outliers_descriptor;
  gain_auto_outliers_descriptor.description =
      "The total pixels from top of the distribution that are ignored by the "
      "auto gain algorithm (0.01% increments).";
  gain_auto_outliers_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("gain_auto_outliers",
                         rclcpp::ParameterValue(0),
                         gain_auto_outliers_descriptor);
  // gen.add("gain_auto_rate",       int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "The rate percentage at which the auto gain function changes.", 100, 0,
  // 100)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_rate_descriptor;
  gain_auto_rate_descriptor.description =
      "The rate percentage at which the auto gain function changes.";
  gain_auto_rate_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "gain_auto_rate", rclcpp::ParameterValue(100), gain_auto_rate_descriptor);
  // gen.add("gain_auto_target",     int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "The general lightness or darkness of the auto gain feature. A percentage
  // of maximum brightness.", 50, 0, 100)
  rcl_interfaces::msg::ParameterDescriptor gain_auto_target_descriptor;
  gain_auto_target_descriptor.description =
      "The general lightness or darkness of the auto gain feature. A "
      "percentage of maximum brightness.";
  gain_auto_target_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("gain_auto_target",
                         rclcpp::ParameterValue(50),
                         gain_auto_target_descriptor);
  // # WHITE BALANCE
  // gen.add("balance_ratio_abs",    double_t, SensorLevels.RECONFIGURE_RUNNING,
  // "Adjusts the gain of the channel selected in the
  // `~BalanceRatioSelector`", 1.0, 0.25, 4.0)
  rcl_interfaces::msg::ParameterDescriptor balance_ratio_abs_descriptor;
  balance_ratio_abs_descriptor.description =
      "Adjusts the gain of the channel selected in the `~BalanceRatioSelector`";
  balance_ratio_abs_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("balance_ratio_abs",
                         rclcpp::ParameterValue(1.0),
                         balance_ratio_abs_descriptor);
  // gen.add("balance_ratio_selector",str_t,   SensorLevels.RECONFIGURE_RUNNING,
  // "Select the Red or Blue channel to adjust with `~BalanceRatioAbs`", "Red",
  // edit_method = balance_ratio_enum)
  rcl_interfaces::msg::ParameterDescriptor balance_ratio_selector_descriptor;
  balance_ratio_selector_descriptor.description =
      "Select the Red or Blue channel to adjust with `~BalanceRatioAbs`";
  balance_ratio_selector_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("balance_ratio_selector",
                         rclcpp::ParameterValue("Red"),
                         balance_ratio_selector_descriptor);
  // gen.add("whitebalance_auto",     str_t,   SensorLevels.RECONFIGURE_RUNNING,
  // "Whether whitebalance will continuously adjust to the current scene. Causes
  // the `~whitebalance_red` and `~whitebalance_blue` settings to be ignored.",
  // "Continuous", edit_method = auto_enum)
  rcl_interfaces::msg::ParameterDescriptor whitebalance_auto_descriptor;
  whitebalance_auto_descriptor.description =
      "Whether whitebalance will continuously adjust to the current scene. "
      "Causes the `~whitebalance_red` and `~whitebalance_blue` settings to be "
      "ignored.";
  whitebalance_auto_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("whitebalance_auto",
                         rclcpp::ParameterValue("Continuous"),
                         whitebalance_auto_descriptor);
  // gen.add("whitebalance_auto_tol", int_t,   SensorLevels.RECONFIGURE_RUNNING,
  // "Tolerance allowed from the ideal white balance values", 5, 0, 50)
  rcl_interfaces::msg::ParameterDescriptor whitebalance_auto_tol_descriptor;
  whitebalance_auto_tol_descriptor.description =
      "Tolerance allowed from the ideal white balance values";
  whitebalance_auto_tol_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("whitebalance_auto_tol",
                         rclcpp::ParameterValue(5),
                         whitebalance_auto_tol_descriptor);
  // gen.add("whitebalance_auto_rate",int_t,   SensorLevels.RECONFIGURE_RUNNING,
  // "Rate of white balance adjustments, from 1 (slowest) to 100 (fastest).",
  // 100, 1, 100)
  rcl_interfaces::msg::ParameterDescriptor whitebalance_auto_rate_descriptor;
  whitebalance_auto_rate_descriptor.description =
      "Rate of white balance adjustments, from 1 (slowest) to 100 (fastest).";
  whitebalance_auto_rate_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("whitebalance_auto_rate",
                         rclcpp::ParameterValue(100),
                         whitebalance_auto_rate_descriptor);
  // # BINNING & DEDIMATION
  // gen.add("binning_x",            int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Number of pixels to bin together horizontally.", 1, 1, 8)
  rcl_interfaces::msg::ParameterDescriptor binning_x_descriptor;
  binning_x_descriptor.description =
      "Number of pixels to bin together horizontally.";
  binning_x_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "binning_x", rclcpp::ParameterValue(1), binning_x_descriptor);
  // gen.add("binning_y",            int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Number of pixels to bin together vertically.", 1, 1, 14)
  rcl_interfaces::msg::ParameterDescriptor binning_y_descriptor;
  binning_y_descriptor.description =
      "Number of pixels to bin together vertically.";
  binning_y_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "binning_y", rclcpp::ParameterValue(1), binning_y_descriptor);
  // gen.add("decimation_x",         int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Number of decimation operations in x.", 1, 1, 8)
  rcl_interfaces::msg::ParameterDescriptor decimation_x_descriptor;
  decimation_x_descriptor.description = "Number of decimation operations in x.";
  decimation_x_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "decimation_x", rclcpp::ParameterValue(1), decimation_x_descriptor);
  // gen.add("decimation_y",         int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Number of decimation operations in y.", 1, 1, 8)
  rcl_interfaces::msg::ParameterDescriptor decimation_y_descriptor;
  decimation_y_descriptor.description = "Number of decimation operations in y.";
  decimation_y_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "decimation_y", rclcpp::ParameterValue(1), decimation_y_descriptor);
  // # ROI
  // gen.add("width",                int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Width of the region of interest (0 for automatic).",  4096, 1, 4096)
  rcl_interfaces::msg::ParameterDescriptor width_descriptor;
  width_descriptor.description =
      "Width of the region of interest (0 for automatic).";
  width_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "width", rclcpp::ParameterValue(4096), width_descriptor);
  // gen.add("height",               int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Height of the region of interest (0 for automatic).", 4096, 1, 4096)
  rcl_interfaces::msg::ParameterDescriptor height_descriptor;
  height_descriptor.description =
      "Height of the region of interest (0 for automatic).";
  height_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "height", rclcpp::ParameterValue(4096), height_descriptor);
  // gen.add("roi_width",            int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "X width of the region of interest.", 0, 0, 4095)
  rcl_interfaces::msg::ParameterDescriptor roi_width_descriptor;
  roi_width_descriptor.description = "X width of the region of interest.";
  roi_width_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "roi_width", rclcpp::ParameterValue(0), roi_width_descriptor);
  // gen.add("roi_height",           int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Y height of the region of interest.", 0, 0, 4095)
  rcl_interfaces::msg::ParameterDescriptor roi_height_descriptor;
  roi_height_descriptor.description = "Y height of the region of interest.";
  roi_height_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "roi_height", rclcpp::ParameterValue(0), roi_height_descriptor);
  // gen.add("roi_offset_x",         int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "X offset of the region of interest.", 0, 0, 4095)
  rcl_interfaces::msg::ParameterDescriptor roi_offset_x_descriptor;
  roi_offset_x_descriptor.description = "X offset of the region of interest.";
  roi_offset_x_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "roi_offset_x", rclcpp::ParameterValue(0), roi_offset_x_descriptor);
  // gen.add("roi_offset_y",         int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Y offset of the region of interest.", 0, 0, 4095)
  rcl_interfaces::msg::ParameterDescriptor roi_offset_y_descriptor;
  roi_offset_y_descriptor.description = "Y offset of the region of interest.";
  roi_offset_y_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "roi_offset_y", rclcpp::ParameterValue(0), roi_offset_y_descriptor);
  // # PIXEL FORMAT
  // gen.add("pixel_format",         str_t,    SensorLevels.RECONFIGURE_CLOSE,
  // "Format of the image data.", "Mono8", edit_method=pixelformat_enum)
  rcl_interfaces::msg::ParameterDescriptor pixel_format_descriptor;
  pixel_format_descriptor.description = "Format of the image data.";
  pixel_format_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "pixel_format", rclcpp::ParameterValue("Mono8"), pixel_format_descriptor);
  // # BANDWIDTH
  // gen.add("stream_bytes_per_second", int_t,
  // SensorLevels.RECONFIGURE_RUNNING,"Limits the data rate of the camera.",
  // 45000000, 1, 115000000)
  rcl_interfaces::msg::ParameterDescriptor stream_bytes_per_second_descriptor;
  stream_bytes_per_second_descriptor.description =
      "Limits the data rate of the camera.";
  stream_bytes_per_second_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("stream_bytes_per_second",
                         rclcpp::ParameterValue(45000000),
                         stream_bytes_per_second_descriptor);
  // # PTP
  // gen.add("ptp_mode",         	  str_t,
  // SensorLevels.RECONFIGURE_RUNNING,"Controls the PTP behavior of the clock
  // port.", "Off", edit_method=ptp_mode_enum)
  rcl_interfaces::msg::ParameterDescriptor ptp_mode_descriptor;
  ptp_mode_descriptor.description =
      "Controls the PTP behavior of the clock port.";
  ptp_mode_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "ptp_mode", rclcpp::ParameterValue("Off"), ptp_mode_descriptor);
  // # GPIO
  // gen.add("sync_in_selector",     str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Selects the sync-out line to control", "SyncIn1",
  // edit_method=sync_in_selector_enum)
  rcl_interfaces::msg::ParameterDescriptor sync_in_selector_descriptor;
  sync_in_selector_descriptor.description =
      "Selects the sync-out line to control";
  sync_in_selector_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("sync_in_selector",
                         rclcpp::ParameterValue("SyncIn1"),
                         sync_in_selector_descriptor);
  // gen.add("sync_out_polarity",    str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Polarity applied to the sync-out line specified by `sync_out_selector`",
  // "Normal", edit_method=polarity_enum)
  rcl_interfaces::msg::ParameterDescriptor sync_out_polarity_descriptor;
  sync_out_polarity_descriptor.description =
      "Polarity applied to the sync-out line specified by `sync_out_selector`";
  sync_out_polarity_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("sync_out_polarity",
                         rclcpp::ParameterValue("Normal"),
                         sync_out_polarity_descriptor);
  // gen.add("sync_out_selector",    str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Selects the sync-out line to control", "SyncOut1",
  // edit_method=sync_out_selector_enum)
  rcl_interfaces::msg::ParameterDescriptor sync_out_selector_descriptor;
  sync_out_selector_descriptor.description =
      "Selects the sync-out line to control";
  sync_out_selector_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("sync_out_selector",
                         rclcpp::ParameterValue("SyncOut1"),
                         sync_out_selector_descriptor);
  // gen.add("sync_out_source",      str_t,    SensorLevels.RECONFIGURE_STOP,
  // "Signal source of the sync-out line specified by `sync_out_selector`",
  // "GPO", edit_method=sync_source_enum)
  rcl_interfaces::msg::ParameterDescriptor sync_out_source_descriptor;
  sync_out_source_descriptor.description =
      "Signal source of the sync-out line specified by `sync_out_selector`";
  sync_out_source_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("sync_out_source",
                         rclcpp::ParameterValue("GPO"),
                         sync_out_source_descriptor);
  // # IRIS
  // gen.add("iris_auto_target",     int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "This is the target image mean value, in percent.", 50, 0, 100)
  rcl_interfaces::msg::ParameterDescriptor iris_auto_target_descriptor;
  iris_auto_target_descriptor.description =
      "This is the target image mean value, in percent.";
  iris_auto_target_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("iris_auto_target",
                         rclcpp::ParameterValue(50),
                         iris_auto_target_descriptor);
  // gen.add("iris_mode",            str_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Set the iris mode. Disabled: no iris control. Video: enable video iris.
  // VideoOpen: fully open a video iris. VideoClose: fully close a video iris.",
  // "Continuous", edit_method = auto_enum)
  rcl_interfaces::msg::ParameterDescriptor iris_mode_descriptor;
  iris_mode_descriptor.description =
      "Set the iris mode. Disabled: no iris control. Video: enable video iris. "
      "VideoOpen: fully open a video iris. VideoClose: fully close a video "
      "iris.";
  iris_mode_descriptor.integer_range.push_back(range);
  nh_->declare_parameter(
      "iris_mode", rclcpp::ParameterValue("Continuous"), iris_mode_descriptor);
  // gen.add("iris_video_level_min", int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Minimum video iris level output by the camera, in approximately mV pp. A
  // higher minimum value slows the adjustment time but prevents excessive
  // overshoot.", 110, 0, 150)
  rcl_interfaces::msg::ParameterDescriptor iris_video_level_min_descriptor;
  iris_video_level_min_descriptor.description =
      "Minimum video iris level output by the camera, in approximately mV pp. "
      "A higher minimum value slows the adjustment time but prevents excessive "
      "overshoot.";
  iris_video_level_min_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("iris_video_level_min",
                         rclcpp::ParameterValue(110),
                         iris_video_level_min_descriptor);
  // gen.add("iris_video_level_max", int_t,    SensorLevels.RECONFIGURE_RUNNING,
  // "Maximum video iris level output by the camera, in approximately mV pp. A
  // lower minimum value slows the adjustment time but prevents excessive
  // overshoot.", 110, 0, 150)
  rcl_interfaces::msg::ParameterDescriptor iris_video_level_max_descriptor;
  iris_video_level_max_descriptor.description =
      "Maximum video iris level output by the camera, in approximately mV pp. "
      "A lower minimum value slows the adjustment time but prevents excessive "
      "overshoot.";
  iris_video_level_max_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("iris_video_level_max",
                         rclcpp::ParameterValue(110),
                         iris_video_level_max_descriptor);
}
AvtVimbaCameraParms::~AvtVimbaCameraParms() {}

} // namespace avt_vimba_camera

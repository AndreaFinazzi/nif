/*
  AvtVimbaCameraParms.h
  Converted from the ROS1 drivers' AvtVimbaCamera.cfg file
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

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>

namespace avt_vimba_camera {
class AvtVimbaCameraParms {
public:
  AvtVimbaCameraParms(rclcpp::Node::SharedPtr& node_handle);
  ~AvtVimbaCameraParms();

  enum trigger_source_enum {
    Freerun,   // str_t, "Freerun",   "Run at maximum frame rate"),
    Line1,     // str_t, "Line1",   "External trigger on SyncIn1 line"),
    Line2,     // str_t, "Line2",   "External trigger on SyncIn2 line"),
    Line3,     // str_t, "Line3",   "External trigger on SyncIn3 line"),
    Line4,     // str_t, "Line4",   "External trigger on SyncIn4 line"),
    FixedRate, // str_t, "FixedRate",   "Camera self-triggers at a fixed frame
               // rate defined by `~AcquisitionFrameRateAbs`"),
    Software // str_t, "Software",   "Software inititated image capture")], "Set
             // Trigger Mode")
  };

  enum trigger_activation_enum {
    RisingEdge,  // str_t, "RisingEdge", ""),
    FallingEdge, // str_t, "FallingEdge", ""),
    AnyEdge,     // str_t, "AnyEdge", ""),
    LevelHigh,   // str_t, "LevelHigh", ""),
    LevelLow     // str_t, "LevelLow", "")], "Set Automatic Control")
  };

  enum trigger_mode_enum {
    ModeOn, // str_t, "On", ""),
    ModeOff // str_t, "Off", "")], "Trigger mode")
  };

  enum trigger_selector_enum {
    FrameStart,       // str_t, "FrameStart", ""),
    AcquisitionStart, // str_t, "AcquisitionStart", ""),
    AcquisitionEnd,   // str_t, "AcquisitionEnd", ""),
    AcquisitionRecord // str_t, "AcquisitionRecord", "")], "Trigger activation
                      // selector")
  };
  enum acquisition_mode_enum {
    Continuous, // str_t, "Continuous", "After an acquisition start event, the
                // camera will continuously receive frame trigger events."),
    SingleFrame, // str_t, "SingleFrame", "The camera will only deliver a single
                 // frame trigger event"),
    MultiFrame, // str_t, "MultiFrame", "The camera will acquire the number of
                // images specified by `~AcquisitionFrameCount`. Further trigger
                // events will be ignored"),
    Recorder // str_t, "Recorder", "The camera will continuously record images
             // into the camera on-board memory")], "Set Trigger Mode")
  };

  // NOTE: Your AVT camera model likely doesn't support all these formats,
  // check the datasheet to confirm which formats are supported.
  enum pixelformat_enum {
    Mono8,           // str_t, "Mono8", ""),
    Mono10,          // str_t, "Mono10", ""),
    Mono10Packed,    // str_t, "Mono10Packed", ""),
    Mono12,          // str_t, "Mono12", ""),
    Mono12Packed,    // str_t, "Mono12Packed", ""),
    BayerGR8,        // str_t, "BayerGR8", ""),
    BayerRG8,        // str_t, "BayerRG8", ""),
    BayerGB8,        // str_t, "BayerGB8", ""),
    BayerBG8,        // str_t, "BayerBG8", ""),
    BayerGR10,       // str_t, "BayerGR10", ""),
    BayerRG10,       // str_t, "BayerRG10", ""),
    BayerGB10,       // str_t, "BayerGB10", ""),
    BayerBG10,       // str_t, "BayerBG10", ""),
    BayerGR12,       // str_t, "BayerGR12", ""),
    BayerRG12,       // str_t, "BayerRG12", ""),
    BayerGB12,       // str_t, "BayerGB12", ""),
    BayerBG12,       // str_t, "BayerBG12", ""),
    BayerGR10Packed, // str_t, "BayerGR10Packed", ""),
    BayerRG10Packed, // str_t, "BayerRG10Packed", ""),
    BayerGB10Packed, // str_t, "BayerGB10Packed", ""),
    BayerBG10Packed, // str_t, "BayerBG10Packed", ""),
    BayerGR12Packed, // str_t, "BayerGR12Packed", ""),
    BayerRG12Packed, // str_t, "BayerRG12Packed", ""),
    BayerGB12Packed, // str_t, "BayerGB12Packed", ""),
    BayerBG12Packed, // str_t, "BayerBG12Packed", ""),
    RGB8Packed,      // str_t, "RGB8Packed", ""),
    BGR8Packed       // str_t, "BGR8Packed", "")
  };

  enum auto_enum {
    Off,  // str_t, "Off", ""),
    Once, // str_t, "Once", ""),
    Auto  // str_t, "Continuous", "")], "Set Automatic Control")
  };

  enum balance_ratio_enum {
    Red, // str_t, "Red", ""),
    Blue // str_t, "Blue", "")], "Select the Red or Blue channel to adjust with
         // `~BalanceRatioAbs`")
  };

  enum polarity_enum {
    Normal, // str_t, "Normal", ""),
    Invert  // str_t, "Invert", "")], "Polarity")
  };

  enum ptp_mode_enum {
    PtpOff,    // str_t, "Off",    ""),
    PtpSlave,  // str_t, "Slave",  ""),
    PtpMaster, // str_t, "Master", ""),
    PtpAuto    // str_t, "Auto",   "")], "Select PrecissionTimeProtocol Mode")
  };

  enum sync_in_selector_enum {
    SyncIn1, // str_t, "SyncIn1", ""),
    SyncIn2, // str_t, "SyncIn2", ""),
    SyncIn3, // str_t, "SyncIn3", ""),
    SyncIn4  // str_t, "SyncIn4", "")],"Sync-in selector")
  };

  enum sync_out_selector_enum {
    SyncOut1, // str_t, "SyncOut1", ""),
    SyncOut2, // str_t, "SyncOut2", ""),
    SyncOut3, // str_t, "SyncOut3", ""),
    SyncOut4  // str_t, "SyncOut4", "")],"Sync-out selector")
  };

  enum sync_source_enum {
    GPO,                     // str_t, "GPO", ""),
    AcquisitionTriggerReady, // str_t, "AcquisitionTriggerReady", ""),
    FrameTriggerReady,       // str_t, "FrameTriggerReady", ""),
    FrameTrigger,            // str_t, "FrameTrigger", ""),
    Exposing,                // str_t, "Exposing", ""),
    FrameReadout,            // str_t, "FrameReadout", ""),
    Imaging,                 // str_t, "Imaging", ""),
    Acquiring,               // str_t, "Acquiring", ""),
    LineIn1,                 // str_t, "LineIn1", ""),
    LineIn2                  // str_t, "LineIn2", "")],"Sync-out signal")
  };

  enum exposure_alg_enum {
    Mean, // str_t, "Mean", "[Default] The arithmetic mean of the histogram of
          // the current image is compared to ExposureAutoTarget, and the next
          // image adjusted in exposure time to meet this target. Bright areas
          // are allowed to saturate"),
    FitRange // str_t, "FitRange", "The histogram of the current image is
             // measured, and the exposure time of the next image is adjusted so
             // bright areas are not saturated")],"The following algorithms can
             // be used to calculate auto exposure"
  };

  rcl_interfaces::msg::ParameterDescriptor frame_id_descriptor;

private:
  rclcpp::Node::SharedPtr nh_;

}; // class AvtVimbaCameraParms
} // namespace avt_vimba_camera
